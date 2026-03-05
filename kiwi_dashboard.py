#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════╗
║        KIWI BOT DASHBOARD  —  ROS2 Humble + RPi SSH          ║
║        Loads maps from RPi via SSH                           ║
╠══════════════════════════════════════════════════════════════╣
║  SETUP:   pip install paramiko                              ║
║  CONFIG:  Edit RPI_HOST, RPI_USER, RPI_PASS, RPI_MAP_PATH  ║
║  RUN:     python3 kiwi_dashboard.py                          ║
║  BROWSER: http://localhost:8080                              ║
╠══════════════════════════════════════════════════════════════╣
║  FEATURES:                                                   ║
║  • Connect to RPi via SSH to load PGM + YAML                 ║
║  • Dashboard displays maps from RPi /maps directory          ║
║  • ROS2 joystick → /cmd_vel,  /odom → TF, /dis_data → bar   ║
║  • [P] key → waypoint marking, export JSON                   ║
╚══════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node        import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg      import Odometry
from std_msgs.msg      import String

import threading, json, math, time
import hashlib, base64, struct, io
import http.server, socketserver
from typing import Set
from pathlib import Path

# SSH connection to RPi
try:
    import paramiko
    HAS_SSH = True
except ImportError:
    HAS_SSH = False

# RPi SSH Config
RPI_HOST = "192.168.1.100"  # Change to your RPi IP
RPI_USER = "rpi"          # Change to your RPi username
RPI_PASS = "sabarisc"       # Change to your RPi password
RPI_MAP_PATH = "/"  # Path on RPi
HTTP_PORT=8080

def _pgm_to_png_bytes(pgm_data):
    """Convert PGM image data to PNG bytes for browser display"""
    try:
        import struct
        lines = pgm_data.decode('utf-8', errors='ignore').split('\n')
        idx = 0
        # Parse PGM header
        magic = lines[idx].strip()
        idx += 1
        while lines[idx].strip().startswith('#'):
            idx += 1
        w, h = map(int, lines[idx].split())
        idx += 1
        maxval = int(lines[idx].strip())
        idx += 1
        # Skip to binary data
        header_size = sum(len(l.encode()) + 1 for l in lines[:idx])
        pixels = pgm_data[header_size:]
        # Convert grayscale to RGBA for canvas
        rgba = bytearray()
        for byte in pixels[:w*h]:
            rgba.extend([byte, byte, byte, 255])
        # Simple PNG encoding (just return raw RGBA as data URL won't work, use canvas instead)
        return bytes(rgba), w, h
    except Exception as e:
        print(f"[PGM] Parse error: {e}")
        return None, 0, 0

def _ssh_get_file(filename):
    """Fetch file from RPi via SSH"""
    if not HAS_SSH:
        return None
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(RPI_HOST, username=RPI_USER, password=RPI_PASS, timeout=5)
        sftp = ssh.open_sftp()
        remote_path = f"{RPI_MAP_PATH}/{filename}"
        local_io = io.BytesIO()
        sftp.getfo(remote_path, local_io)
        sftp.close()
        ssh.close()
        return local_io.getvalue()
    except Exception as e:
        print(f"[SSH] Error fetching {filename}: {e}")
        return None

def _ssh_list_files():
    """List map files on RPi"""
    if not HAS_SSH:
        return []
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(RPI_HOST, username=RPI_USER, password=RPI_PASS, timeout=5)
        sftp = ssh.open_sftp()
        files = sftp.listdir(RPI_MAP_PATH)
        sftp.close()
        ssh.close()
        return sorted(files)
    except Exception as e:
        print(f"[SSH] Error listing files: {e}")
        return []


# ═══════════════════════════════════════════════════════
#  STDLIB WEBSOCKET  (RFC 6455 — no pip needed)
# ═══════════════════════════════════════════════════════
WS_MAGIC = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

def _ws_accept_key(k):
    return base64.b64encode(hashlib.sha1(k.encode()+WS_MAGIC).digest()).decode()

def _recv_exact(sock, n):
    buf = b""
    while len(buf)<n:
        c=sock.recv(n-len(buf))
        if not c: raise ConnectionError("closed")
        buf+=c
    return buf

def _ws_read_frame(sock):
    h=_recv_exact(sock,2)
    op=h[0]&0x0F; masked=(h[1]&0x80)!=0; pl=h[1]&0x7F
    if pl==126: pl=struct.unpack(">H",_recv_exact(sock,2))[0]
    elif pl==127: pl=struct.unpack(">Q",_recv_exact(sock,8))[0]
    mk=_recv_exact(sock,4) if masked else b""
    data=_recv_exact(sock,pl)
    if masked: data=bytes(b^mk[i%4] for i,b in enumerate(data))
    return op,data

def _ws_send(sock,text):
    p=text.encode(); n=len(p)
    h=bytearray([0x81])
    if n<126: h.append(n)
    elif n<65536: h+=bytearray([126])+struct.pack(">H",n)
    else: h+=bytearray([127])+struct.pack(">Q",n)
    try: sock.sendall(bytes(h)+p); return True
    except: return False

class _WSClient:
    def __init__(self,sock,addr): self.sock=sock; self.addr=addr; self._lk=threading.Lock()
    def send(self,t):
        with self._lk: return _ws_send(self.sock,t)
    def close(self):
        try: self.sock.close()
        except: pass

def _bcast(data):
    msg=json.dumps(data); dead=set()
    with ws_clients_lock: snap=set(ws_clients)
    for c in snap:
        if not c.send(msg): dead.add(c)
    if dead:
        with ws_clients_lock: ws_clients.difference_update(dead)

def _handle_ws(client):
    with ws_clients_lock: ws_clients.add(client)
    print(f"[WS] +{client.addr}")
    try:
        while True:
            op,payload=_ws_read_frame(client.sock)
            if op==0x8: break
            if op==0x9: _ws_send(client.sock,b"",0xA)
            if op in(0x1,0x2):
                try:
                    d=json.loads(payload.decode())
                    if d.get("type")=="cmd_vel":
                        with state_lock:
                            shared_state["cmd_vel"]={
                                "linear_x":  float(d.get("linear_x",0.0)),
                                "linear_y":  float(d.get("linear_y",0.0)),
                                "angular_z": float(d.get("angular_z",0.0)),
                            }
                except Exception as e: print(f"[WS] err {e}")
    except: pass
    finally:
        with ws_clients_lock: ws_clients.discard(client)
        client.close(); print(f"[WS] -{client.addr}")


# ═══════════════════════════════════════════════════════
#  HTTP + WS HANDLER  (single port 8080)
# ═══════════════════════════════════════════════════════
class _Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/api/files":
            files = _ssh_list_files()
            self.send_response(200)
            self.send_header("Content-Type","application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"files": files}).encode())
            return
        elif self.path.startswith("/api/download/"):
            filename = self.path.split("/")[-1]
            data = _ssh_get_file(filename)
            if data:
                self.send_response(200)
                ct = "image/x-pgm" if filename.endswith(".pgm") else "text/yaml"
                self.send_header("Content-Type", ct)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()
            return
        if self.headers.get("Upgrade","").lower()=="websocket" and self.path=="/ws":
            self._upgrade(); return
        body=_HTML.encode()
        self.send_response(200)
        self.send_header("Content-Type","text/html; charset=utf-8")
        self.send_header("Content-Length",str(len(body)))
        self.send_header("Cache-Control","no-cache")
        self.end_headers(); self.wfile.write(body)

    def _upgrade(self):
        key=self.headers.get("Sec-WebSocket-Key","")
        if not key: self.send_response(400); self.end_headers(); return
        resp=(f"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
              f"Connection: Upgrade\r\nSec-WebSocket-Accept: {_ws_accept_key(key)}\r\n\r\n")
        self.wfile.write(resp.encode()); self.wfile.flush()
        raw=self.connection; raw.setblocking(True)
        c=_WSClient(raw,self.client_address)
        t=threading.Thread(target=_handle_ws,args=(c,),daemon=True)
        t.start(); t.join()

    def log_message(self,*_): pass

class _Srv(socketserver.ThreadingMixIn,http.server.HTTPServer):
    allow_reuse_address=True; daemon_threads=True

def _run_server():
    s=_Srv(("",HTTP_PORT),_Handler)
    print(f"[SRV] http://localhost:{HTTP_PORT}")
    s.serve_forever()


# ═══════════════════════════════════════════════════════
#  EMBEDDED HTML
# ═══════════════════════════════════════════════════════
_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>KiwiBot Dashboard</title>
<style>
:root{
  --bg:#09111d;--p1:#0d1a28;--p2:#111f30;--bd:#1a2d3f;
  --ac:#00d4ff;--ac2:#ff6b35;--gr:#3dffaa;--re:#ff3d5a;
  --tx:#cce4f5;--mu:#2e4a62;--mu2:#4a7090;
}
*{margin:0;padding:0;box-sizing:border-box;}
html,body{height:100%;overflow:hidden;}
body{background:var(--bg);color:var(--tx);font-family:'Courier New',monospace;display:flex;flex-direction:column;}

/* HEADER */
header{display:flex;align-items:center;justify-content:space-between;padding:7px 18px;border-bottom:1px solid var(--bd);background:var(--p1);flex-shrink:0;}
.logo{font-size:15px;font-weight:900;letter-spacing:3px;color:var(--ac);text-transform:uppercase;}
.logo b{color:var(--ac2);}
.hbar{display:flex;gap:12px;align-items:center;}
.badge{font-size:10px;padding:3px 10px;border-radius:20px;border:1px solid;letter-spacing:.5px;transition:all .3s;}
.badge.on {color:var(--gr);border-color:var(--gr);background:rgba(61,255,170,.06);}
.badge.off{color:var(--re);border-color:var(--re);background:rgba(255,61,90,.06);}
.badge.cn {color:#ffcc00;border-color:#ffcc00;background:rgba(255,204,0,.06);}
.ptip{font-size:10px;color:var(--mu2);}
.ptip b{color:var(--ac2);}

/* DEBUG BAR */
.dbg{padding:2px 14px;background:#060d16;border-bottom:1px solid var(--bd);font-size:9px;color:var(--mu2);display:flex;gap:14px;flex-shrink:0;}
.dbg b{color:#ffcc00;}

/* LAYOUT */
.main{display:flex;flex:1;overflow:hidden;}

/* ── LEFT: MAP ── */
.mleft{flex:1;display:flex;flex-direction:column;overflow:hidden;border-right:1px solid var(--bd);}

/* MAP UPLOAD OVERLAY */
.map-upload-zone{
  flex:1;display:flex;flex-direction:column;align-items:center;justify-content:center;
  gap:16px;padding:30px;
}
.upload-card{
  background:var(--p1);border:2px dashed var(--bd);border-radius:10px;
  padding:32px 40px;text-align:center;max-width:440px;width:100%;
  transition:border-color .2s;
}
.upload-card:hover{border-color:var(--ac);}
.upload-card.dragover{border-color:var(--ac);background:rgba(0,212,255,.05);}
.upload-icon{font-size:42px;margin-bottom:12px;}
.upload-title{font-size:14px;color:var(--ac);font-weight:700;letter-spacing:2px;margin-bottom:8px;}
.upload-sub{font-size:11px;color:var(--mu2);line-height:1.8;margin-bottom:18px;}
.upload-row{display:flex;gap:10px;justify-content:center;flex-wrap:wrap;}
.file-btn{
  padding:8px 18px;border:1px solid var(--bd);border-radius:4px;
  background:var(--p2);color:var(--tx);font-family:inherit;font-size:11px;
  cursor:pointer;transition:all .15s;
}
.file-btn:hover{border-color:var(--ac);background:rgba(0,212,255,.08);}
.file-btn:disabled{opacity:.4;cursor:not-allowed;border-color:var(--bd);background:transparent;}
.file-btn.ready{border-color:var(--gr);color:var(--gr);}
.load-btn{
  padding:9px 24px;background:rgba(0,212,255,.12);border:1px solid var(--ac);
  color:var(--ac);font-family:inherit;font-size:11px;cursor:pointer;
  border-radius:4px;letter-spacing:1px;text-transform:uppercase;
  transition:all .15s;opacity:.4;pointer-events:none;
}
.load-btn.active{opacity:1;pointer-events:all;}
.load-btn.active:hover{background:rgba(0,212,255,.22);}
.map-info-row{display:flex;gap:20px;font-size:10px;color:var(--mu2);}
.map-info-row b{color:var(--ac2);}

/* MAP CANVAS AREA (shown after load) */
.map-canvas-area{flex:1;display:none;flex-direction:column;overflow:hidden;}
.map-canvas-area.visible{display:flex;}
.mapwrap{flex:1;display:flex;align-items:center;justify-content:center;padding:12px;overflow:auto;}
.mapinner{display:inline-flex;flex-direction:column;}
.rrow{display:flex;}
.corner{width:26px;height:26px;background:var(--p1);border-right:1px solid var(--bd);border-bottom:1px solid var(--bd);}
.rxw{height:26px;background:var(--p1);border-bottom:1px solid var(--bd);overflow:hidden;}
.ryw{width:26px;background:var(--p1);border-right:1px solid var(--bd);overflow:hidden;}
canvas{display:block;image-rendering:pixelated;}
#mc{cursor:crosshair;}
.legend{padding:4px 14px;background:var(--p1);border-top:1px solid var(--bd);display:flex;gap:16px;flex-shrink:0;}
.leg{font-size:10px;color:var(--mu2);}
.sstrip{padding:5px 14px;border-top:1px solid var(--bd);background:var(--p1);display:flex;gap:18px;flex-wrap:wrap;align-items:center;flex-shrink:0;}
.sv .k{font-size:10px;color:var(--mu2);}
.sv .v{font-size:10px;color:var(--ac);font-weight:700;}

/* reload map btn */
.reload-map-btn{
  font-size:9px;color:var(--mu2);background:none;border:1px solid var(--bd);
  cursor:pointer;font-family:inherit;padding:2px 8px;border-radius:3px;
  transition:all .15s;margin-left:auto;
}
.reload-map-btn:hover{border-color:var(--ac2);color:var(--ac2);}

/* ── RIGHT: CONTROLS ── */
.mright{width:295px;background:var(--p1);display:flex;flex-direction:column;overflow-y:auto;flex-shrink:0;}
.mright::-webkit-scrollbar{width:3px;}
.mright::-webkit-scrollbar-thumb{background:var(--bd);}
.sec{padding:10px 14px;border-bottom:1px solid var(--bd);}
.st{font-size:9px;color:var(--mu);letter-spacing:2px;text-transform:uppercase;margin-bottom:8px;}

/* map meta */
.meta-grid{display:grid;grid-template-columns:1fr 1fr;gap:4px;}
.mb{background:var(--p2);border:1px solid var(--bd);border-radius:3px;padding:5px 8px;}
.mb .k{font-size:8px;color:var(--mu2);}
.mb .v{font-size:11px;color:var(--ac2);font-weight:700;margin-top:1px;}

/* odom */
.og{display:grid;grid-template-columns:1fr 1fr 1fr;gap:5px;}
.ob{background:var(--p2);border:1px solid var(--bd);border-radius:4px;padding:6px 8px;text-align:center;}
.ob .k{font-size:8px;color:var(--mu2);}
.ob .n{font-size:14px;color:var(--ac);font-weight:700;margin-top:1px;}

/* joystick */
.jarea{display:flex;justify-content:center;align-items:center;gap:16px;padding:2px 0;}
.jpad{width:128px;height:128px;border-radius:50%;background:radial-gradient(circle,#0d1f32,#060c15);border:2px solid var(--bd);position:relative;cursor:pointer;touch-action:none;flex-shrink:0;}
.jpad:hover{border-color:var(--ac);}
.jring{position:absolute;inset:14px;border-radius:50%;border:1px solid rgba(0,212,255,.1);pointer-events:none;}
.jch,.jcv{position:absolute;background:rgba(0,212,255,.09);pointer-events:none;}
.jch{left:0;right:0;height:1px;top:50%;}
.jcv{top:0;bottom:0;width:1px;left:50%;}
.jknob{position:absolute;width:36px;height:36px;border-radius:50%;background:radial-gradient(circle at 38% 32%,#ff8050,#b02800);top:50%;left:50%;transform:translate(-50%,-50%);pointer-events:none;box-shadow:0 0 12px rgba(255,107,53,.55);}
.rcol{display:flex;flex-direction:column;gap:8px;align-items:center;}
.rbtn{width:44px;height:44px;border-radius:50%;background:var(--p2);border:2px solid var(--bd);color:var(--ac);font-size:20px;cursor:pointer;display:flex;align-items:center;justify-content:center;user-select:none;transition:all .12s;}
.rbtn:hover{border-color:var(--ac);background:rgba(0,212,255,.08);}
.rbtn.held{background:rgba(0,212,255,.18);border-color:var(--ac);}
.rlbl{font-size:8px;color:var(--mu);letter-spacing:1px;}
.spdr{display:flex;align-items:center;gap:8px;margin-top:6px;}
.spdr label{font-size:9px;color:var(--mu2);white-space:nowrap;}
input[type=range]{flex:1;accent-color:var(--ac);}
.spdv{font-size:11px;color:var(--ac);min-width:32px;text-align:right;font-weight:700;}

/* cmd_vel */
.cvg{display:grid;grid-template-columns:1fr 1fr 1fr;gap:4px;margin-top:5px;}
.cb{background:var(--p2);border:1px solid var(--bd);border-radius:3px;padding:5px 6px;text-align:center;}
.cb .k{font-size:8px;color:var(--mu2);}
.cb .n{font-size:12px;color:var(--ac2);font-weight:700;}

/* origin */
.obtn{width:100%;padding:7px 12px;background:transparent;border:1px solid var(--bd);color:var(--ac2);font-family:inherit;font-size:11px;cursor:pointer;border-radius:3px;transition:all .15s;text-align:left;}
.obtn:hover{border-color:var(--ac2);background:rgba(255,107,53,.07);}
.obtn.on{border-color:var(--ac2);background:rgba(255,107,53,.15);color:#fff;}
.oinf{margin-top:5px;font-size:10px;color:var(--mu2);}
.oinf b{color:var(--ac2);}

/* waypoints */
.wlist{max-height:165px;overflow-y:auto;margin-top:3px;}
.wlist::-webkit-scrollbar{width:3px;}
.wlist::-webkit-scrollbar-thumb{background:var(--bd);}
.wi{display:flex;align-items:center;gap:7px;padding:5px 8px;margin-bottom:4px;background:var(--p2);border:1px solid var(--bd);border-radius:4px;}
.wn{width:17px;height:17px;border-radius:50%;background:var(--ac2);color:#fff;font-size:8px;font-weight:700;display:flex;align-items:center;justify-content:center;flex-shrink:0;}
.wb{flex:1;}
.wxy{font-size:11px;color:var(--ac);}
.wyaw{font-size:10px;color:var(--gr);}
.wdel{background:none;border:none;color:var(--mu);cursor:pointer;font-size:13px;padding:2px 4px;transition:color .15s;}
.wdel:hover{color:var(--re);}
.wempty{text-align:center;color:var(--mu2);font-size:11px;padding:12px 0;line-height:2.2;}
.wempty b{color:var(--ac2);}

/* buttons */
.expbtn{width:100%;padding:8px;background:rgba(0,212,255,.08);border:1px solid var(--ac);color:var(--ac);font-family:inherit;font-size:11px;cursor:pointer;border-radius:3px;letter-spacing:1.5px;text-transform:uppercase;transition:all .15s;}
.expbtn:hover{background:rgba(0,212,255,.18);}
.clrbtn{font-size:9px;color:var(--re);background:none;border:1px solid transparent;cursor:pointer;font-family:inherit;padding:2px 7px;border-radius:3px;transition:all .15s;}
.clrbtn:hover{border-color:var(--re);background:rgba(255,61,90,.09);}

/* popup */
.pover{position:fixed;inset:0;background:rgba(0,0,0,.65);z-index:1000;display:flex;align-items:center;justify-content:center;}
.ppop{background:var(--p1);border:1px solid var(--ac);border-radius:6px;padding:18px;width:450px;max-width:94vw;display:flex;flex-direction:column;gap:10px;}
.ppop textarea{min-height:190px;background:var(--bg);border:1px solid var(--bd);color:#9abfd8;font-family:inherit;font-size:11px;padding:10px;border-radius:3px;resize:vertical;outline:none;}
.prow{display:flex;gap:8px;}
.pbtn{flex:1;padding:8px;background:rgba(0,212,255,.09);border:1px solid var(--ac);color:var(--ac);font-family:inherit;font-size:11px;cursor:pointer;border-radius:3px;}
.pbtn:hover{background:rgba(0,212,255,.2);}
.pbtn.cl{background:rgba(255,61,90,.07);border-color:var(--re);color:var(--re);}
.pbtn.cl:hover{background:rgba(255,61,90,.18);}

@keyframes fl{0%,100%{outline:none}45%{outline:3px solid var(--gr);outline-offset:1px;}}
.fl{animation:fl .5s ease;}
</style>
</head>
<body>

<header>
  <div class="logo">Kiwi<b>Bot</b> Dashboard</div>
  <div class="hbar">
    <div class="ptip">Press <b>[P]</b> to mark waypoint</div>
    <div class="badge off" id="wsBadge">● OFFLINE</div>
  </div>
</header>

<div class="dbg">
  WS: <b id="dbgUrl">—</b> &nbsp;|&nbsp;
  retries:<span id="dbgR">0</span> &nbsp;|&nbsp;
  rx:<span id="dbgRx">0</span> &nbsp;|&nbsp;
  odom:<span id="dbgO">0</span>
</div>

<div class="main">

  <!-- ══ LEFT: MAP ══ -->
  <div class="mleft">

    <!-- UPLOAD ZONE (shown before map loaded) -->
    <div class="map-upload-zone" id="uploadZone">
      <div style="background:rgba(255,61,90,.1);border:1px solid rgba(255,61,90,.3);padding:12px;border-radius:6px;margin-bottom:12px;text-align:center;">
        <div style="font-size:11px;color:#ff3d5a;font-weight:700;">⚠ CONNECTION STATUS</div>
        <div style="font-size:10px;color:var(--mu2);margin-top:4px;">Waiting for ROS2 bot... <span id="connStatus">OFFLINE</span></div>
      </div>
      <div class="upload-card" id="dropCard">
        <div class="upload-icon">🗺️</div>
        <div class="upload-title">Load Arena Map</div>
        <div class="upload-sub">
          Upload your ROS map files to render the arena.<br>
          <b style="color:var(--ac)">PGM</b> — occupancy grid image<br>
          <b style="color:var(--ac)">YAML</b> — map metadata (resolution, origin)
        </div>
        <div class="upload-row">
          <button class="file-btn" id="pgmBtn" onclick="showPgmList()" disabled>
            📷 PGM from RPi
          </button>
          <button class="file-btn" id="yamlBtn" onclick="showYamlList()" disabled>
            📄 YAML from RPi
          </button>
        </div>
        <div style="margin-top:14px">
          <button class="load-btn" id="loadBtn" onclick="loadMap()">⬆ Load Map</button>
        </div>
      </div>
      <div class="map-info-row" id="fileStatus">
        <span id="pngStatus">PGM: <b>not selected</b></span>
        <span id="yamlStatus">YAML: <b>not selected</b></span>
      </div>
    </div>

    <!-- CANVAS AREA (shown after map loaded) -->
    <div class="map-canvas-area" id="canvasArea">
      <div class="mapwrap">
        <div class="mapinner">
          <div class="rrow">
            <div class="corner"></div>
            <div class="rxw"><canvas id="rulerX"></canvas></div>
          </div>
          <div style="display:flex">
            <div class="ryw"><canvas id="rulerY"></canvas></div>
            <canvas id="mc"></canvas>
          </div>
        </div>
      </div>
      <div class="legend">
        <div class="leg">▽ <span style="color:var(--ac)">Robot front</span></div>
        <div class="leg">⊕ <span style="color:var(--ac2)">Origin (0,0)</span></div>
        <div class="leg">● <span style="color:var(--ac2)">Waypoints</span></div>
        <button class="reload-map-btn" onclick="reloadMap()">🔄 Load new map</button>
      </div>
      <div class="sstrip">
        <div class="sv"><span class="k">DIS: </span><span class="v" id="sDis">—</span></div>
        <div class="sv"><span class="k">X: </span><span class="v" id="sX">0.000</span></div>
        <div class="sv"><span class="k">Y: </span><span class="v" id="sY">0.000</span></div>
        <div class="sv"><span class="k">YAW: </span><span class="v" id="sYaw">0°</span></div>
        <div class="sv" style="margin-left:auto"><span class="k">MAP: </span><span class="v" id="sMapName">—</span></div>
      </div>
    </div>
  </div>

  <!-- ══ RIGHT: CONTROLS ══ -->
  <div class="mright">

    <!-- map meta -->
    <div class="sec">
      <div class="st">Map Info</div>
      <div class="meta-grid">
        <div class="mb"><div class="k">RESOLUTION</div><div class="v" id="mRes">—</div></div>
        <div class="mb"><div class="k">SIZE (px)</div><div class="v" id="mSize">—</div></div>
        <div class="mb"><div class="k">ORIGIN X</div><div class="v" id="mOx">—</div></div>
        <div class="mb"><div class="k">ORIGIN Y</div><div class="v" id="mOy">—</div></div>
      </div>
    </div>

    <!-- origin -->
    <div class="sec">
      <div class="st">Map Origin (0,0)</div>
      <button class="obtn" id="obtn" onclick="toggleOrig()">🎯 Click map to set origin</button>
      <div class="oinf" id="oinf">Not set — click button then click map</div>
    </div>

    <!-- odom -->
    <div class="sec">
      <div class="st">Odometry  /odom</div>
      <div class="og">
        <div class="ob"><div class="k">X (m)</div><div class="n" id="oX">0.000</div></div>
        <div class="ob"><div class="k">Y (m)</div><div class="n" id="oY">0.000</div></div>
        <div class="ob"><div class="k">YAW °</div><div class="n" id="oYaw">0</div></div>
      </div>
    </div>

    <!-- joystick -->
    <div class="sec">
      <div class="st">Joystick → /cmd_vel</div>
      <div class="jarea">
        <div class="jpad" id="jpad">
          <div class="jring"></div><div class="jch"></div><div class="jcv"></div>
          <div class="jknob" id="jknob"></div>
        </div>
        <div class="rcol">
          <div class="rbtn" id="rotL"
            onmousedown="startRot(1)"  onmouseup="stopRot()" onmouseleave="stopRot()"
            ontouchstart="startRot(1)" ontouchend="stopRot()">↺</div>
          <div class="rlbl">ROTATE</div>
          <div class="rbtn" id="rotR"
            onmousedown="startRot(-1)"  onmouseup="stopRot()" onmouseleave="stopRot()"
            ontouchstart="startRot(-1)" ontouchend="stopRot()">↻</div>
        </div>
      </div>
      <div class="spdr">
        <label>MAX SPD</label>
        <input type="range" id="spdR" min="0.05" max="1.0" step="0.05" value="0.3" oninput="onSpd()">
        <span class="spdv" id="spdV">0.30</span>
      </div>
    </div>

    <!-- cmd_vel -->
    <div class="sec">
      <div class="st">cmd_vel (live)</div>
      <div class="cvg">
        <div class="cb"><div class="k">LIN.X</div><div class="n" id="cvX">0.00</div></div>
        <div class="cb"><div class="k">LIN.Y</div><div class="n" id="cvY">0.00</div></div>
        <div class="cb"><div class="k">ANG.Z</div><div class="n" id="cvZ">0.00</div></div>
      </div>
    </div>

    <!-- waypoints -->
    <div class="sec" style="flex:1">
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px;">
        <div class="st" style="margin:0">Waypoints [P]</div>
        <button class="clrbtn" onclick="clearWP()">✕ Clear</button>
      </div>
      <div class="wlist" id="wlist">
        <div class="wempty">No waypoints.<br>Drive robot → press <b>[P]</b></div>
      </div>
    </div>

    <!-- export -->
    <div class="sec">
      <button class="expbtn" onclick="exportWP()">⬇ Export Waypoints JSON</button>
    </div>

  </div>
</div>

<script>
'use strict';

// ══════════════════════════════════════════════════
// MAP STATE
// ══════════════════════════════════════════════════
let mapImg      = null;   // HTMLImageElement
let mapYaml     = null;   // parsed yaml object {resolution, origin:[x,y,z], ...}
let mapName     = '';
let MAP_W       = 0;
let MAP_H       = 0;
let MAP_RES_M   = 0.05;   // metres per pixel (from yaml)
let MAP_ORIGIN  = [0,0];  // real-world x,y of bottom-left pixel (from yaml)

let pngFile     = null;
let yamlText    = null;

// ── canvas ──────────────────────────────────────
const mc   = document.getElementById('mc');
const mctx = mc.getContext('2d');
let SC = 10;   // display scale: screen px per map px

// ── robot & interaction state ────────────────────
let origPx  = null;   // {x,y} map-pixel where user placed origin
let setOrig = false;
let robot   = {x:0, y:0, yaw:0};
let waypts  = [];

// ══════════════════════════════════════════════════
// FILE UPLOAD HANDLERS (from RPi)
// ══════════════════════════════════════════════════
async function showPgmList(){
  const res = await fetch('/api/files');
  const data = await res.json();
  const pgmFiles = data.files.filter(f=>f.match(/\.pgm$/i));
  showFileDialog('Select PGM File', pgmFiles, 'pgm');
}

async function showYamlList(){
  const res = await fetch('/api/files');
  const data = await res.json();
  const yamlFiles = data.files.filter(f=>f.match(/\.ya?ml$/i));
  showFileDialog('Select YAML File', yamlFiles, 'yaml');
}

function showFileDialog(title, files, type){
  const list = files.map(f=>`<div style="padding:8px;cursor:pointer;border-bottom:1px solid var(--bd)" onclick="selectFile('${f}','${type}')">${f}</div>`).join('');
  const html = `<div class="pover" onclick="this.remove()"><div class="ppop" onclick="event.stopPropagation()"><div style="color:var(--ac);font-size:12px">${title}</div><div style="max-height:300px;overflow-y:auto;border:1px solid var(--bd)">${list}</div></div></div>`;
  const div = document.createElement('div');
  div.innerHTML = html;
  document.body.appendChild(div.firstElementChild);
}

async function selectFile(filename, type){
  const res = await fetch(`/api/download/${filename}`);
  if(res.ok){
    const blob = await res.blob();
    if(type==='pgm'){
      pngFile = {name: filename, blob: blob};
      document.getElementById('pgmBtn').classList.add('ready');
      document.getElementById('pgmBtn').textContent = '✓ ' + filename;
      document.getElementById('pngStatus').innerHTML = `PGM: <b style="color:var(--gr)">${filename}</b>`;
    } else {
      yamlText = await blob.text();
      document.getElementById('yamlBtn').classList.add('ready');
      document.getElementById('yamlBtn').textContent = '✓ ' + filename;
      document.getElementById('yamlStatus').innerHTML = `YAML: <b style="color:var(--gr)">${filename}</b>`;
    }
    checkBothReady();
    document.querySelector('.pover')?.remove();
  }
}

function checkBothReady(){
  const btn = document.getElementById('loadBtn');
  if(pngFile && yamlText){ btn.classList.add('active'); }
}

// drag & drop on the card
const dropCard = document.getElementById('dropCard');
dropCard.addEventListener('dragover', e=>{ e.preventDefault(); dropCard.classList.add('dragover'); });
dropCard.addEventListener('dragleave',()=>dropCard.classList.remove('dragover'));
dropCard.addEventListener('drop', e=>{
  e.preventDefault(); dropCard.classList.remove('dragover');
  const files = [...e.dataTransfer.files];
  files.forEach(f=>{
    if(f.name.match(/\.pgm$/i)){
      pngFile=f;
      document.getElementById('pngBtn').textContent='✓ '+f.name;
      document.getElementById('pngBtn').classList.add('ready');
      document.getElementById('pngStatus').innerHTML=`PGM: <b style="color:var(--gr)">${f.name}</b>`;
    }
    if(f.name.match(/\.ya?ml$/i)){
      document.getElementById('yamlBtn').textContent='✓ '+f.name;
      document.getElementById('yamlBtn').classList.add('ready');
      document.getElementById('yamlStatus').innerHTML=`YAML: <b style="color:var(--gr)">${f.name}</b>`;
      const r=new FileReader(); r.onload=ev=>{yamlText=ev.target.result;checkBothReady();}; r.readAsText(f);
    }
  });
  checkBothReady();
});

// ══════════════════════════════════════════════════
// YAML PARSER  (minimal — handles ROS map YAML)
// Parses: resolution, origin, width/height (optional)
// ══════════════════════════════════════════════════
function parseYaml(text){
  const result = {};
  text.split('\n').forEach(line=>{
    line = line.trim();
    if(!line || line.startsWith('#')) return;
    const colon = line.indexOf(':');
    if(colon<0) return;
    const key = line.slice(0,colon).trim();
    let   val = line.slice(colon+1).trim();
    // handle array like [x, y, z]
    if(val.startsWith('[')){
      val = val.replace(/[\[\]]/g,'').split(',').map(v=>parseFloat(v.trim()));
    } else if(!isNaN(parseFloat(val))){
      val = parseFloat(val);
    }
    result[key] = val;
  });
  return result;
}

// ══════════════════════════════════════════════════
// LOAD MAP
// ══════════════════════════════════════════════════
function loadMap(){
  if(!pngFile || !yamlText){ alert('Please select both PNG and YAML files.'); return; }

  // Parse YAML
  mapYaml = parseYaml(yamlText);
  MAP_RES_M  = mapYaml.resolution || 0.05;
  MAP_ORIGIN = Array.isArray(mapYaml.origin) ? mapYaml.origin.slice(0,2) : [0,0];
  mapName    = pngFile.name.replace(/\.[^.]+$/,'');

  // Update meta panel
  document.getElementById('mRes').textContent  = MAP_RES_M.toFixed(4)+' m/px';
  document.getElementById('mOx').textContent   = MAP_ORIGIN[0].toFixed(3);
  document.getElementById('mOy').textContent   = MAP_ORIGIN[1].toFixed(3);
  document.getElementById('sMapName').textContent = mapName;

  // Load PGM image
  const reader = new FileReader();
  reader.onload = (e) => {
    const pgmData = e.target.result;
    loadPgmImage(pgmData);
  };
  reader.onerror = ()=>{ alert('Could not read PGM file.'); };
  reader.readAsArrayBuffer(pngFile.blob || pngFile);
}

function loadPgmImage(pgmData){
  const header = new TextDecoder().decode(new Uint8Array(pgmData, 0, 200));
  const lines = header.split('\\n');
  let idx = 0;
  while(lines[idx].startsWith('#')) idx++;
  const [w, h] = lines[idx].split(' ').map(Number);
  idx++;
  const maxval = Number(lines[idx]);
  let headerBytes = 0;
  for(let i=0; i<=idx; i++) headerBytes += (lines[i].length + 1);
  
  const pixelData = new Uint8Array(pgmData, headerBytes, w*h);
  const canvas = document.createElement('canvas');
  canvas.width = w; canvas.height = h;
  const ctx = canvas.getContext('2d');
  const imgData = ctx.createImageData(w, h);
  for(let i=0; i<w*h; i++){
    const val = pixelData[i];
    imgData.data[i*4] = val;
    imgData.data[i*4+1] = val;
    imgData.data[i*4+2] = val;
    imgData.data[i*4+3] = 255;
  }
  ctx.putImageData(imgData, 0, 0);
  
  mapImg = new Image();
  mapImg.onload = ()=>{
    MAP_W = w;
    MAP_H = h;
    document.getElementById('mSize').textContent = MAP_W+'×'+MAP_H;
    const auto_px_x = (-MAP_ORIGIN[0]) / MAP_RES_M;
    const auto_px_y = MAP_H - (-MAP_ORIGIN[1]) / MAP_RES_M;
    if(auto_px_x>=0 && auto_px_x<MAP_W && auto_px_y>=0 && auto_px_y<MAP_H){
      origPx = {x: Math.round(auto_px_x), y: Math.round(auto_px_y)};
      document.getElementById('obtn').textContent = '🎯 Origin auto-set — click to move';
      document.getElementById('oinf').innerHTML = `Auto from YAML: px(${origPx.x},${origPx.y}) → <b>world(0,0)</b>`;
    }
    initCanvas();
    document.getElementById('uploadZone').style.display='none';
    document.getElementById('canvasArea').classList.add('visible');
  };
  mapImg.src = canvas.toDataURL();
}
}

function reloadMap(){
  // Reset and show upload zone again
  mapImg=null; mapYaml=null; pngFile=null; yamlText=null; origPx=null; waypts=[];
  robot={x:0,y:0,yaw:0};
  document.getElementById('uploadZone').style.display='';
  document.getElementById('canvasArea').classList.remove('visible');
  document.getElementById('pgmBtn').textContent='📷 PGM from RPi';
  document.getElementById('pgmBtn').classList.remove('ready');
  document.getElementById('yamlBtn').textContent='📄 YAML from RPi';
  document.getElementById('yamlBtn').classList.remove('ready');
  document.getElementById('pngStatus').innerHTML='PGM: <b>not selected</b>';
  document.getElementById('yamlStatus').innerHTML='YAML: <b>not selected</b>';
  document.getElementById('loadBtn').classList.remove('active');
  updateWPList();
}

// ══════════════════════════════════════════════════
// CANVAS  (dynamic size from map image)
// ══════════════════════════════════════════════════
function initCanvas(){
  if(!mapImg) return;
  const avail = Math.min(window.innerHeight-165, window.innerWidth-330);
  // fit map to available space
  const scX = avail / MAP_W;
  const scY = (window.innerHeight-165) / MAP_H;
  SC = Math.max(1, Math.min(scX, scY, 16));
  mc.width  = Math.round(MAP_W * SC);
  mc.height = Math.round(MAP_H * SC);
  drawRulerX(); drawRulerY(); redraw();
}

// ── Rulers ──────────────────────────────────────
function drawRulerX(){
  const c=document.getElementById('rulerX');
  c.width=mc.width; c.height=26;
  const g=c.getContext('2d');
  g.fillStyle='#0d1a28'; g.fillRect(0,0,c.width,26);
  g.font='8px Courier New'; g.textAlign='center';
  // label in metres
  const totalM = MAP_W * MAP_RES_M;
  const step   = niceStep(totalM, 10);
  for(let m=0; m<=totalM+step; m+=step){
    const px = (m/MAP_RES_M)*SC;
    if(px>mc.width+5) break;
    const maj = true;
    g.strokeStyle='rgba(0,212,255,.4)'; g.lineWidth=1;
    g.beginPath(); g.moveTo(px+.5,8); g.lineTo(px+.5,26); g.stroke();
    g.fillStyle='#2e4a62'; g.fillText(m.toFixed(1)+'m',px,7);
  }
}
function drawRulerY(){
  const c=document.getElementById('rulerY');
  c.width=26; c.height=mc.height;
  const g=c.getContext('2d');
  g.fillStyle='#0d1a28'; g.fillRect(0,0,26,c.height);
  const totalM = MAP_H * MAP_RES_M;
  const step   = niceStep(totalM, 10);
  for(let m=0; m<=totalM+step; m+=step){
    const py = (m/MAP_RES_M)*SC;
    if(py>mc.height+5) break;
    g.strokeStyle='rgba(0,212,255,.4)'; g.lineWidth=1;
    g.beginPath(); g.moveTo(8,py+.5); g.lineTo(26,py+.5); g.stroke();
    g.save(); g.fillStyle='#2e4a62'; g.font='8px Courier New';
    g.textAlign='center'; g.translate(8,py); g.rotate(-Math.PI/2);
    g.fillText(m.toFixed(1)+'m',0,3); g.restore();
  }
}
function niceStep(range, targetTicks){
  const raw = range/targetTicks;
  const mag = Math.pow(10,Math.floor(Math.log10(raw)));
  const norm = raw/mag;
  const nice = norm<1.5?1:norm<3.5?2:norm<7.5?5:10;
  return nice*mag;
}

// ══════════════════════════════════════════════════
// COORDINATE MAPPING
//
// ROS odom:  x = forward,  y = left
// Map image: pixel(0,0) = top-left
//            x increases RIGHT, y increases DOWN
//
// After axis swap fix:
//   odom.x (forward) → canvas -Y  (moving forward = up on map)
//   odom.y (strafe)  → canvas +X  (moving left    = left on map)
//
// Also accounting for YAML origin offset:
//   world(0,0) → originPx on the canvas (set by user or auto from YAML)
// ══════════════════════════════════════════════════
function r2c(){
  if(!origPx) return null;
  // odom metres → map pixels relative to origin
  // odom.x forward = canvas -Y (up), odom.y strafe = canvas +X
  return {
    x: (origPx.x + (robot.y / MAP_RES_M)) * SC,
    y: (origPx.y - (robot.x / MAP_RES_M)) * SC
  };
}
function w2c(wp){
  if(!origPx) return null;
  return {
    x: (origPx.x + (wp.y_m / MAP_RES_M)) * SC,
    y: (origPx.y - (wp.x_m / MAP_RES_M)) * SC
  };
}

// ══════════════════════════════════════════════════
// DRAW ROBOT — INVERTED TRIANGLE (nose/face at BOTTOM)
// ══════════════════════════════════════════════════
function drawRobot(cx,cy,yaw,col){
  const sz = Math.max(8, SC*1.2);
  mctx.save();
  mctx.translate(cx,cy);
  // yaw=0 → facing forward = canvas UP (-Y direction)
  // nose drawn at +sz (bottom of triangle = inverted)
  // rotate: -yaw for canvas (Y-flip) + PI to flip triangle
  mctx.rotate(Math.PI - yaw);

  // ── inverted triangle (nose at bottom = front of robot) ──
  mctx.beginPath();
  mctx.moveTo(0, sz);             // NOSE / FRONT — pointing DOWN (inverted)
  mctx.lineTo(-sz*0.65, -sz*0.7);
  mctx.lineTo( sz*0.65, -sz*0.7);
  mctx.closePath();
  mctx.fillStyle   = col+'28';
  mctx.fill();
  mctx.strokeStyle = col;
  mctx.lineWidth   = 2;
  mctx.stroke();

  // dot at nose tip
  mctx.fillStyle = col;
  mctx.beginPath(); mctx.arc(0, sz*0.72, 2.5, 0, Math.PI*2); mctx.fill();

  // direction line from center to nose
  mctx.strokeStyle = col+'aa';
  mctx.lineWidth   = 1.5;
  mctx.beginPath(); mctx.moveTo(0,0); mctx.lineTo(0,sz*0.7); mctx.stroke();

  mctx.restore();
}

function drawArr(ox,oy,deg,len,col,lw){
  const r=deg*Math.PI/180, ex=ox+Math.cos(r)*len, ey=oy-Math.sin(r)*len;
  mctx.save(); mctx.strokeStyle=col; mctx.fillStyle=col; mctx.lineWidth=lw;
  mctx.beginPath(); mctx.moveTo(ox,oy); mctx.lineTo(ex,ey); mctx.stroke();
  const a=Math.atan2(ey-oy,ex-ox),hl=7,ha=Math.PI/6;
  mctx.beginPath(); mctx.moveTo(ex,ey);
  mctx.lineTo(ex-hl*Math.cos(a-ha),ey-hl*Math.sin(a-ha));
  mctx.lineTo(ex-hl*Math.cos(a+ha),ey-hl*Math.sin(a+ha));
  mctx.closePath(); mctx.fill(); mctx.restore();
}

// ══════════════════════════════════════════════════
// RENDER MAP
// ══════════════════════════════════════════════════
function redraw(){
  if(!mapImg) return;
  mctx.clearRect(0,0,mc.width,mc.height);

  // Draw map image scaled
  mctx.imageSmoothingEnabled = false;
  mctx.drawImage(mapImg, 0, 0, mc.width, mc.height);

  // Subtle grid overlay in metres
  const totalMx = MAP_W * MAP_RES_M;
  const totalMy = MAP_H * MAP_RES_M;
  const step    = niceStep(Math.max(totalMx,totalMy), 20);
  mctx.lineWidth=.5; mctx.strokeStyle='rgba(0,212,255,.15)';
  for(let m=0;m<=totalMx;m+=step){
    const px=(m/MAP_RES_M)*SC;
    mctx.beginPath(); mctx.moveTo(px+.5,0); mctx.lineTo(px+.5,mc.height); mctx.stroke();
  }
  for(let m=0;m<=totalMy;m+=step){
    const py=(m/MAP_RES_M)*SC;
    mctx.beginPath(); mctx.moveTo(0,py+.5); mctx.lineTo(mc.width,py+.5); mctx.stroke();
  }

  // Origin marker
  if(origPx){
    const ox=origPx.x*SC, oy=origPx.y*SC;
    mctx.save();
    mctx.strokeStyle='#ff6b35'; mctx.lineWidth=2;
    mctx.beginPath(); mctx.moveTo(ox-13,oy); mctx.lineTo(ox+13,oy); mctx.stroke();
    mctx.beginPath(); mctx.moveTo(ox,oy-13); mctx.lineTo(ox,oy+13); mctx.stroke();
    mctx.beginPath(); mctx.arc(ox,oy,5,0,Math.PI*2); mctx.stroke();
    mctx.fillStyle='#ff6b35'; mctx.font='bold 9px Courier New';
    mctx.textAlign='left'; mctx.fillText('(0,0)',ox+7,oy-7); mctx.restore();
  }

  // Waypoints
  waypts.forEach((wp,i)=>{
    const p=w2c(wp); if(!p)return;
    mctx.save();
    mctx.fillStyle='#09111d'; mctx.beginPath(); mctx.arc(p.x,p.y,6,0,Math.PI*2); mctx.fill();
    mctx.strokeStyle='#ff6b35'; mctx.lineWidth=2; mctx.beginPath(); mctx.arc(p.x,p.y,6,0,Math.PI*2); mctx.stroke();
    mctx.fillStyle='#ff6b35'; mctx.font='bold 7px Courier New';
    mctx.textAlign='center'; mctx.textBaseline='middle'; mctx.fillText(i+1,p.x,p.y); mctx.restore();
    drawArr(p.x,p.y,wp.yaw_deg,SC*1.4,'#3dffaa',1.5);
    mctx.save(); mctx.font='8px Courier New';
    const lb=`(${wp.x_m.toFixed(2)},${wp.y_m.toFixed(2)})`, tw=mctx.measureText(lb).width;
    mctx.fillStyle='rgba(9,17,29,.85)'; mctx.fillRect(p.x+9,p.y-14,tw+4,11);
    mctx.fillStyle='#00d4ff'; mctx.textAlign='left'; mctx.textBaseline='alphabetic';
    mctx.fillText(lb,p.x+11,p.y-5); mctx.restore();
  });

  // Robot TF (inverted triangle)
  const rp=r2c();
  if(rp){
    drawRobot(rp.x, rp.y, robot.yaw, '#00d4ff');
    mctx.save(); mctx.font='8px Courier New'; mctx.fillStyle='rgba(0,212,255,.85)';
    mctx.textAlign='left'; mctx.textBaseline='alphabetic';
    mctx.fillText(`x:${robot.x.toFixed(2)} y:${robot.y.toFixed(2)}`, rp.x+SC, rp.y-4);
    mctx.restore();
  }
  mctx.textBaseline='alphabetic'; mctx.textAlign='left';
}

// ══════════════════════════════════════════════════
// ORIGIN PLACEMENT
// ══════════════════════════════════════════════════
function toggleOrig(){
  setOrig=!setOrig;
  const b=document.getElementById('obtn');
  b.classList.toggle('on',setOrig);
  b.textContent = setOrig ? '⏳ Click on the map...'
    : (origPx ? '🎯 Origin set — click to move' : '🎯 Click map to set origin');
}
mc.addEventListener('click',e=>{
  if(!setOrig) return;
  const r=mc.getBoundingClientRect();
  const mx=Math.floor((e.clientX-r.left)/SC);
  const my=Math.floor((e.clientY-r.top)/SC);
  if(mx<0||mx>=MAP_W||my<0||my>=MAP_H) return;
  origPx={x:mx,y:my}; setOrig=false;
  document.getElementById('obtn').classList.remove('on');
  document.getElementById('obtn').textContent='🎯 Origin set — click to move';
  document.getElementById('oinf').innerHTML=`px(${mx},${my}) → <b>world(0,0)</b>`;
  redraw();
});
window.addEventListener('resize',()=>{ if(mapImg) initCanvas(); });

// ══════════════════════════════════════════════════
// WEBSOCKET
// ══════════════════════════════════════════════════
let ws=null, retries=0, rxCnt=0, odomCnt=0;
function wsUrl(){ return `ws://${window.location.host}/ws`; }
function setBadge(s){
  const b=document.getElementById('wsBadge');
  if(s==='on'){b.textContent='● ONLINE';b.className='badge on';}
  else if(s==='c'){b.textContent='● CONNECTING…';b.className='badge cn';}
  else{b.textContent='● OFFLINE';b.className='badge off';}
}
function connect(){
  if(ws&&ws.readyState<=1) return;
  const url=wsUrl();
  document.getElementById('dbgUrl').textContent=url;
  setBadge('c');
  try{ ws=new WebSocket(url); }catch(e){ retry(); return; }
  ws.onopen=()=>{ 
    retries=0; 
    document.getElementById('dbgR').textContent='0'; 
    setBadge('on');
    document.getElementById('connStatus').textContent='CONNECTED';
    document.getElementById('connStatus').style.color='var(--gr)';
    document.getElementById('pngBtn').disabled=false;
    document.getElementById('yamlBtn').disabled=false;
  };
  ws.onclose=()=>{ 
    setBadge('off'); 
    document.getElementById('connStatus').textContent='OFFLINE';
    document.getElementById('connStatus').style.color='var(--re)';
    document.getElementById('pngBtn').disabled=true;
    document.getElementById('yamlBtn').disabled=true;
    retry(); 
  };
  ws.onerror=()=>{ console.warn('[WS] err'); };
  ws.onmessage=e=>{
    rxCnt++; document.getElementById('dbgRx').textContent=rxCnt;
    try{
      const d=JSON.parse(e.data);
      if(d.type==='odom'){
        odomCnt++; document.getElementById('dbgO').textContent=odomCnt;
        robot.x=d.x; robot.y=d.y; robot.yaw=d.yaw;
        const deg=(d.yaw*180/Math.PI).toFixed(1);
        document.getElementById('oX').textContent=d.x.toFixed(3);
        document.getElementById('oY').textContent=d.y.toFixed(3);
        document.getElementById('oYaw').textContent=deg;
        document.getElementById('sX').textContent=d.x.toFixed(3);
        document.getElementById('sY').textContent=d.y.toFixed(3);
        document.getElementById('sYaw').textContent=deg+'°';
        if(mapImg) redraw();
      }
      if(d.type==='dis_data') document.getElementById('sDis').textContent=d.data;
    }catch(_){}
  };
}
function retry(){
  retries++; document.getElementById('dbgR').textContent=retries;
  setTimeout(connect, Math.min(500*Math.pow(2,Math.min(retries-1,4)),8000));
}
function send(obj){
  if(ws&&ws.readyState===WebSocket.OPEN) try{ ws.send(JSON.stringify(obj)); }catch(_){}
}
document.addEventListener('visibilitychange',()=>{ if(!document.hidden&&(!ws||ws.readyState>1)) connect(); });
connect();

// ══════════════════════════════════════════════════
// JOYSTICK
// ══════════════════════════════════════════════════
const jpad=document.getElementById('jpad');
const jknob=document.getElementById('jknob');
let jA=false,jLX=0,jLY=0,jAZ=0,maxSpd=0.3;

function setKnob(dx,dy){
  const R=jpad.offsetWidth/2-20, d=Math.sqrt(dx*dx+dy*dy);
  const cl=Math.min(d,R), ang=Math.atan2(dy,dx);
  const kx=Math.cos(ang)*cl, ky=Math.sin(ang)*cl;
  jknob.style.left=(50+kx/R*50)+'%'; jknob.style.top=(50+ky/R*50)+'%';
  jLX=(kx/R)*maxSpd; jLY=-(ky/R)*maxSpd; updateCV();
}
function jStart(e){jA=true;jMove(e);}
function jMove(e){
  if(!jA) return;
  const r=jpad.getBoundingClientRect();
  const pt=e.touches?e.touches[0]:e;
  setKnob(pt.clientX-r.left-r.width/2, pt.clientY-r.top-r.height/2);
}
function jEnd(){jA=false;jLX=0;jLY=0;jknob.style.left='50%';jknob.style.top='50%';updateCV();}
jpad.addEventListener('mousedown',jStart);
window.addEventListener('mousemove',jMove);
window.addEventListener('mouseup',jEnd);
jpad.addEventListener('touchstart',e=>{e.preventDefault();jStart(e);},{passive:false});
window.addEventListener('touchmove',e=>{jMove(e);},{passive:false});
window.addEventListener('touchend',jEnd);

function startRot(d){ jAZ=d*maxSpd*1.5; document.getElementById(d>0?'rotL':'rotR').classList.add('held'); updateCV(); }
function stopRot(){ jAZ=0; document.getElementById('rotL').classList.remove('held'); document.getElementById('rotR').classList.remove('held'); updateCV(); }
function onSpd(){ maxSpd=parseFloat(document.getElementById('spdR').value); document.getElementById('spdV').textContent=maxSpd.toFixed(2); }
function updateCV(){
  document.getElementById('cvX').textContent=jLY.toFixed(2);
  document.getElementById('cvY').textContent=(-jLX).toFixed(2);
  document.getElementById('cvZ').textContent=jAZ.toFixed(2);
}
setInterval(()=>send({type:'cmd_vel', linear_x:jLY, linear_y:-jLX, angular_z:jAZ}),100);

// ══════════════════════════════════════════════════
// WAYPOINTS  [P key]
// ══════════════════════════════════════════════════
document.addEventListener('keydown',e=>{ if(e.key==='p'||e.key==='P') markWP(); });
function markWP(){
  waypts.push({
    id: waypts.length+1,
    x_m: +robot.x.toFixed(4), y_m: +robot.y.toFixed(4),
    yaw_deg: Math.round(robot.yaw*180/Math.PI), yaw_rad: robot.yaw
  });
  document.body.classList.remove('fl'); void document.body.offsetWidth; document.body.classList.add('fl');
  setTimeout(()=>document.body.classList.remove('fl'),500);
  updateWPList(); redraw();
}
function updateWPList(){
  const el=document.getElementById('wlist');
  if(!waypts.length){ el.innerHTML='<div class="wempty">No waypoints.<br>Drive robot → press <b>[P]</b></div>'; return; }
  el.innerHTML=waypts.map((wp,i)=>`
    <div class="wi">
      <div class="wn">${i+1}</div>
      <div class="wb">
        <div class="wxy">x:${wp.x_m} &nbsp;y:${wp.y_m} m</div>
        <div class="wyaw">yaw: ${wp.yaw_deg}°</div>
      </div>
      <button class="wdel" onclick="delWP(${i})">✕</button>
    </div>`).join('');
}
function delWP(i){ waypts.splice(i,1); waypts.forEach((w,j)=>w.id=j+1); updateWPList(); redraw(); }
function clearWP(){ if(!waypts.length)return; if(confirm('Clear all waypoints?')){ waypts=[]; updateWPList(); redraw(); } }

// ══════════════════════════════════════════════════
// EXPORT
// ══════════════════════════════════════════════════
function exportWP(){
  if(!waypts.length){ alert('No waypoints to export.'); return; }
  const data={
    map: mapName,
    map_resolution_m: MAP_RES_M,
    origin_world: MAP_ORIGIN,
    origin_px: origPx,
    waypoints: waypts.map(w=>({id:w.id,x_m:w.x_m,y_m:w.y_m,yaw_deg:w.yaw_deg}))
  };
  const js=JSON.stringify(data,null,2);
  try{
    const b=new Blob([js],{type:'application/json'}),url=URL.createObjectURL(b),a=document.createElement('a');
    a.href=url; a.download='kiwi_waypoints.json'; document.body.appendChild(a); a.click();
    setTimeout(()=>{ URL.revokeObjectURL(url); document.body.removeChild(a); },1000);
  }catch(_){}
  showPopup(js);
}
function showPopup(js){
  const old=document.getElementById('__pop'); if(old)old.remove();
  const div=document.createElement('div'); div.id='__pop'; div.className='pover';
  div.innerHTML=`<div class="ppop">
    <div style="color:var(--ac);font-size:12px;font-weight:700;letter-spacing:1px">📄 kiwi_waypoints.json</div>
    <textarea readonly>${js}</textarea>
    <div class="prow">
      <button class="pbtn" onclick="navigator.clipboard.writeText(document.querySelector('#__pop textarea').value).then(()=>{this.textContent='✓ Copied!'})">📋 Copy</button>
      <button class="pbtn cl" onclick="document.getElementById('__pop').remove()">✕ Close</button>
    </div></div>`;
  document.body.appendChild(div);
}
</script>
</body>
</html>
"""


# ═══════════════════════════════════════════════════════
#  ROS2 NODE
# ═══════════════════════════════════════════════════════
class KiwiDashboardNode(Node):
    def __init__(self):
        super().__init__("kiwi_dashboard")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom",     self._odom_cb, 10)
        self.create_subscription(String,   "/dis_data", self._dis_cb,  10)
        self.create_timer(0.1, self._pub_cmd)
        self.get_logger().info("KiwiBot Dashboard started ✓")
        self.get_logger().info(f"Open browser → http://localhost:{HTTP_PORT}")

    def _pub_cmd(self):
        with state_lock:
            cv = dict(shared_state["cmd_vel"])
        t = Twist()
        t.linear.x  = cv["linear_x"]
        t.linear.y  = cv["linear_y"]
        t.angular.z = cv["angular_z"]
        self._pub.publish(t)

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )
        _bcast({"type":"odom","x":round(x,4),"y":round(y,4),"yaw":round(yaw,5)})

    def _dis_cb(self, msg: String):
        _bcast({"type":"dis_data","data":msg.data})


# ═══════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════
def main():
    threading.Thread(target=_run_server, daemon=True).start()
    time.sleep(0.2)
    print(f"[INFO] Browser → http://localhost:{HTTP_PORT}")
    rclpy.init()
    node = KiwiDashboardNode()
    print("[ROS2] Spinning — Ctrl+C to quit")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Bye!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()