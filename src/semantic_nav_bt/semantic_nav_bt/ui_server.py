"""Semantic autonomous-driving web UI (stdlib HTTP + rclpy, no bridge).

Serves a single-page UI on http://<host>:8080 with
  - semantic command console (goto_place / goto_object / patrol /
    return_home) wired to /semantic_command
  - live mission status feed from /semantic_status (+ robot pose from
    /amcl_pose)
  - TOSM DB management: objects / places / robots tables from the KG;
    object-level owner edits (refute -> absent, correct type, toggle
    isMovable) written back to the KG json with history entries — the
    mission BT's mediator hot-reloads the file on its next resolve.
  - Gazebo live view: MJPEG relays of the world's overhead camera and the
    robot's onboard camera (CompressedImage passthrough, no re-encode).

  ros2 run semantic_nav_bt ui_server --ros-args -p port:=8080
"""
import json
import math
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

BLK = "/home/caselab/Downloads/Cyclone360_data/blk360_seg/outputs"

PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>TOSM Semantic Mission Console</title><style>
body{font-family:system-ui,sans-serif;margin:0;background:#f5f6f8;color:#1c2733}
header{background:#1d3557;color:#fff;padding:10px 18px;font-size:18px;font-weight:600}
.wrap{display:grid;grid-template-columns:340px 1fr;gap:14px;padding:14px}
.card{background:#fff;border-radius:10px;box-shadow:0 1px 4px rgba(0,0,0,.08);padding:14px}
h3{margin:2px 0 10px;font-size:14px;text-transform:uppercase;letter-spacing:.05em;color:#457b9d}
button{background:#1d3557;color:#fff;border:0;border-radius:6px;padding:7px 12px;margin:2px;cursor:pointer;font-size:13px}
button:hover{background:#457b9d} button.warn{background:#b23b3b}
select,input{padding:6px;border:1px solid #ccd;border-radius:6px;font-size:13px;margin:2px}
#statuslog{height:190px;overflow-y:auto;font-family:ui-monospace,monospace;font-size:12px;background:#101820;color:#9fefb0;border-radius:8px;padding:8px}
#statuslog .old{color:#5a7a64}
table{border-collapse:collapse;width:100%;font-size:12.5px}
th,td{border-bottom:1px solid #e3e7ee;padding:5px 7px;text-align:left}
th{background:#eef2f7;position:sticky;top:0}
.tab{display:inline-block;padding:7px 14px;cursor:pointer;border-radius:8px 8px 0 0;background:#dde4ee;margin-right:4px;font-size:13px}
.tab.on{background:#fff;font-weight:600}
.tbox{background:#fff;border-radius:0 10px 10px 10px;box-shadow:0 1px 4px rgba(0,0,0,.08);padding:12px;max-height:520px;overflow-y:auto}
.badge{padding:1px 7px;border-radius:9px;font-size:11px;color:#fff}
.v{background:#2f855a}.u{background:#a0aec0}.r{background:#b23b3b}.o{background:#805ad5}
.small{font-size:11.5px;color:#666}
</style></head><body>
<header>TOSM Semantic Mission Console <span id=conn class=small></span></header>
<div class=wrap>
<div>
  <div class=card><h3>Semantic command</h3>
    <div>place <select id=selPlace></select><button onclick="goPlace()">goto_place</button></div>
    <div>object <select id=selObj></select><button onclick="goObj()">goto_object</button></div>
    <div style="margin-top:6px">
      <button onclick="cmd({cmd:'patrol'})">patrol (all places)</button>
      <button class=warn onclick="cmd({cmd:'return_home'})">return home</button>
    </div>
    <div class=small style="margin-top:6px">battery <input id=batt type=range min=0 max=100 value=100
      oninput="setBatt(this.value)" style="width:130px"> <span id=battv>100%</span> (sim)</div>
  </div>
  <div class=card style="margin-top:12px"><h3>Mission status</h3>
    <div id=statuslog></div>
    <div class=small id=robotpose style="margin-top:6px"></div>
  </div>
</div>
<div>
  <div class=card style="margin-bottom:12px"><h3>Gazebo live view</h3>
    <div style="display:flex;gap:12px;flex-wrap:wrap">
      <div style="position:relative"><div class=small>overhead (world frame, +y up)</div>
        <img id=ovimg src="/stream/overhead" style="height:320px;border-radius:8px;background:#222">
        <div id=robotmark style="position:absolute;width:13px;height:13px;border:3px solid #e63946;
          border-radius:50%;box-shadow:0 0 7px #e63946;transform:translate(-50%,-50%);
          display:none;pointer-events:none"></div></div>
      <div><div class=small>robot camera</div>
        <img src="/stream/robot" style="height:320px;border-radius:8px;background:#222"></div>
    </div>
  </div>
  <div>
    <span class="tab on" id=t_obj onclick="tab('obj')">Objects</span>
    <span class=tab id=t_pla onclick="tab('pla')">Places</span>
    <span class=tab id=t_rob onclick="tab('rob')">Robots</span>
    <span class=tab id=t_bt onclick="tab('bt')">Behavior Tree</span>
  </div>
  <div class=tbox id=tbl></div>
</div>
</div>
<script>
let CUR='obj', STATE=null, LOG=[];
function tab(t){CUR=t;for(const x of ['obj','pla','rob','bt'])
  document.getElementById('t_'+x).className='tab'+(x===t?' on':'');render();}
// ---- Groot-composition live BT view: left-to-right layout, type line over
// ---- instance name (as Groot2 draws it), UI design language kept
const BTC={RUNNING:'#ffb703',SUCCESS:'#3ddc84',FAILURE:'#ff5964',INVALID:'#5c6370'};
const BTICON={Fallback:'?',ReactiveFallback:'R?',Sequence:'→',
  ReactiveSequence:'R→',FailureIsSuccess:'✓',ForceSuccess:'✓',
  Root:'▣'};
const BTKC={composite:'#ff7ab8',subtree:'#e8eaed',decorator:'#5eead4',
  action:'#e8eaed',condition:'#e8eaed'};
let BTDIR=localStorage.getItem('btdir')||'h';   // 'h' = Groot-like, 'v' = top-down
function btSvg(bt){
  // synthetic subtree chip so the composition matches Groot's canvas
  const tree={name:'Root',cls:'Root',kind:'subtree',status:bt.status,
    children:[bt]};
  const horiz=BTDIR==='h';
  const BW=170,BH=46,GX=horiz?46:16,GY=horiz?13:44;let leaf=0;const nodes=[];
  (function walk(n,d,pi){const me={n:n,d:d,pi:pi,i:nodes.length};nodes.push(me);
    if(!n.children.length){me.c=leaf++;}
    else{const cs=n.children.map(c=>walk(c,d+1,me.i));
      me.c=(cs[0]+cs[cs.length-1])/2;}
    return me.c;})(tree,0,-1);
  const depth=Math.max(...nodes.map(m=>m.d))+1;
  const W=horiz?depth*(BW+GX)+GX:leaf*(BW+GX)+GX,
        H=horiz?leaf*(BH+GY)+GY+14:depth*(BH+GY)+14;
  const px=m=>horiz?GX/2+m.d*(BW+GX):GX/2+m.c*(BW+GX),
        py=m=>horiz?12+m.c*(BH+GY):12+m.d*(BH+GY);
  const sc=Math.min(1,(horiz?480:520)/H);
  let s=`<svg viewBox="0 0 ${W} ${H}" width="${Math.round(W*sc)}" height="${Math.round(H*sc)}"
    style="background:#232629;border-radius:8px">`;
  for(const m of nodes){if(m.pi<0)continue;const p=nodes[m.pi];
    let x1,y1,x2,y2,d1,d2;
    if(horiz){x1=px(p)+BW;y1=py(p)+BH/2;x2=px(m);y2=py(m)+BH/2;
      d1=`${x1+GX*0.55},${y1}`;d2=`${x2-GX*0.55},${y2}`;}
    else{x1=px(p)+BW/2;y1=py(p)+BH;x2=px(m)+BW/2;y2=py(m);
      d1=`${x1},${y1+GY*0.55}`;d2=`${x2},${y2-GY*0.55}`;}
    s+=`<path d="M${x1},${y1} C${d1} ${d2} ${x2},${y2}"
      stroke="#0fb8ad" stroke-width="2" fill="none"/>
      <circle cx="${x1}" cy="${y1}" r="3" fill="#0fb8ad"/>
      <circle cx="${x2}" cy="${y2}" r="3" fill="#0fb8ad"/>`;}
  for(const m of nodes){const n=m.n,c=BTC[n.status]||'#5c6370';
    const icon=BTICON[n.cls]||(n.kind==='condition'?'≡':'⚡');
    const tcol=BTKC[n.kind]||'#e8eaed';
    const two=n.name!==n.cls;
    const glow=n.status==='RUNNING'?` filter="drop-shadow(0 0 5px ${c})"`:'';
    s+=`<g${glow}><rect x="${px(m)}" y="${py(m)}" width="${BW}" height="${BH}"
      rx="7" fill="#3b4045" stroke="${c}" stroke-width="2.5"/>
      <text x="${px(m)+BW/2}" y="${py(m)+(two?19:28)}" text-anchor="middle"
        fill="${tcol}" font-size="12.5" font-weight="600"
        font-family="system-ui">${icon} ${n.cls}</text>`;
    if(two){s+=`<text x="${px(m)+BW/2}" y="${py(m)+36}" text-anchor="middle"
        fill="#c3c9d1" font-size="10.5" font-family="system-ui">${n.name}</text>`;}
    s+='</g>';}
  s+='</svg>';
  const leg=Object.entries(BTC).map(([k,v])=>
    `<span style="color:${v}">&#9632; ${k.toLowerCase()}</span>`).join(' &nbsp; ');
  return `<div style="overflow-x:auto">${s}</div>
    <div class=small style="margin-top:6px">
    <button onclick="BTDIR=BTDIR==='h'?'v':'h';localStorage.setItem('btdir',BTDIR);render()"
      style="padding:3px 9px;font-size:11px">${horiz?'↕ top-down':'↔ left-to-right'}</button>
    &nbsp; ${leg} &nbsp;|&nbsp; live from /bt_snapshot
    (tick ${''+new Date().toLocaleTimeString()})</div>`;}
function cmd(c){fetch('/api/command',{method:'POST',body:JSON.stringify(c)});}
function goPlace(){cmd({cmd:'goto_place',target:document.getElementById('selPlace').value});}
function goObj(){cmd({cmd:'goto_object',target:document.getElementById('selObj').value});}
function setBatt(v){document.getElementById('battv').textContent=v+'%';
  fetch('/api/battery',{method:'POST',body:JSON.stringify({level:v/100})});}
function edit(name,action,extra){let body={name:name,action:action,...extra};
  fetch('/api/object',{method:'POST',body:JSON.stringify(body)}).then(()=>poll());}
function stBadge(s){if(!s)return '';let c=s.startsWith('verified')?(s==='verified_owner'?'o':'v')
  :(s.startsWith('refuted')?'r':'u');return `<span class="badge ${c}">${s}</span>`;}
function render(){if(!STATE)return;const d=STATE;let h='';
if(CUR==='obj'){h='<table><tr><th>name</th><th>type</th><th>status</th><th>conf</th><th>pose</th><th>movable</th><th>owner actions</th></tr>';
  for(const o of d.objects){h+=`<tr><td>${o.name}</td><td>${o.type}</td><td>${stBadge(o.status)}</td>
   <td>${o.confidence??''}</td><td>(${o.x},${o.y})</td><td>${o.isMovable?'✓':'✗'}</td>
   <td><button onclick="edit('${o.name}','refute')">refute</button>
   <button onclick="edit('${o.name}','set_type',{type:prompt('correct type for ${o.name}:','${o.type}')})">type</button>
   <button onclick="edit('${o.name}','toggle_movable')">movable</button></td></tr>`;}
  h+='</table>';}
if(CUR==='pla'){h='<table><tr><th>region</th><th>name</th><th>members</th><th>verified</th><th>key object</th><th>centroid</th></tr>';
  for(const p of d.places){h+=`<tr><td>${p.region}</td><td><b>${p.name}</b></td><td>${p.members}</td>
   <td>${p.verified}</td><td>${p.key||''}</td><td>(${p.cx},${p.cy})</td></tr>`;}h+='</table>';}
if(CUR==='bt'){h=d.bt?btSvg(d.bt):
  '<div class=small>no /bt_snapshot yet — is mission_bt running?</div>';}
if(CUR==='rob'){for(const r of d.robots){h+=`<table><tr><th colspan=2>${r.name} (${r.symbolic.type}, ${r.symbolic.drive})</th></tr>`;
  h+=`<tr><td>pose</td><td>(${r.explicit.pose.x}, ${r.explicit.pose.y})</td></tr>`;
  h+=`<tr><td>footprint</td><td>${r.explicit.footprint.length} × ${r.explicit.footprint.width} × ${r.explicit.footprint.height} m</td></tr>`;
  h+=`<tr><td>limits</td><td>${JSON.stringify(r.explicit.limits)}</td></tr>`;
  h+=`<tr><td>sensors</td><td>${r.explicit.sensors.map(s=>s.joint).join(', ')}</td></tr>`;
  h+=`<tr><td>implicit</td><td>${JSON.stringify(r.implicit)}</td></tr>`;
  h+=`<tr><td>place</td><td>${r.isInsideOf}</td></tr></table><br>`;}}
document.getElementById('tbl').innerHTML=h;}
function poll(){fetch('/api/state').then(r=>r.json()).then(d=>{STATE=d;
  document.getElementById('conn').textContent=' — live';
  const sp=document.getElementById('selPlace');
  if(sp.options.length!==d.places.length){sp.innerHTML='';
    for(const p of d.places){sp.add(new Option(p.name,p.name));}}
  const so=document.getElementById('selObj');
  const vs=d.objects.filter(o=>o.status&&o.status.startsWith('verified'));
  if(so.options.length!==vs.length){so.innerHTML='';
    for(const o of vs){so.add(new Option(o.name+' ('+o.type+')',o.name));}}
  const lg=document.getElementById('statuslog');
  lg.innerHTML=d.status_log.map((s,i)=>`<div class="${i<d.status_log.length-1?'old':''}">${s}</div>`).join('');
  lg.scrollTop=lg.scrollHeight;
  document.getElementById('robotpose').textContent=d.robot_pose?
    `robot @ (${d.robot_pose[0].toFixed(2)}, ${d.robot_pose[1].toFixed(2)})  battery ${(d.battery*100).toFixed(0)}%`:'';
  // overhead camera: static top-down at (-2,-1.5,13), hfov 1.3, 960x720
  // -> linear world->pixel map (48.57 px/m, +y up / +x right)
  const mk=document.getElementById('robotmark'),img=document.getElementById('ovimg');
  if(d.robot_pose&&img.clientWidth>0){const PPM=48.57;
    const u=480+(d.robot_pose[0]+2.0)*PPM, v=360-(d.robot_pose[1]+1.5)*PPM;
    mk.style.left=(img.offsetLeft+u*img.clientWidth/960)+'px';
    mk.style.top=(img.offsetTop+v*img.clientHeight/720)+'px';
    mk.style.display='block';}
  render();}).catch(()=>{document.getElementById('conn').textContent=' — offline';});}
setInterval(poll,1000);poll();
</script></body></html>"""


class UIServer(Node):
    def __init__(self):
        super().__init__("semantic_ui")
        p = self.declare_parameter
        self.kg_path = p("kg_path", f"{BLK}/testroom_epochs_kg.json").value
        self.places_path = p("places_path",
                             f"{BLK}/place_layer_T3_slic.json").value
        self.naming_path = p("naming_path",
                             f"{BLK}/place_ring_naming.json").value
        self.port = p("port", 8080).value
        self.cmd_pub = self.create_publisher(String, "semantic_command", 10)
        from sensor_msgs.msg import BatteryState
        self.batt_pub = self.create_publisher(BatteryState, "battery_state",
                                              10)
        self.status_log = []
        self.battery = 1.0
        self.robot_pose = None
        self.bt = None
        self.create_subscription(String, "bt_snapshot",
                                 self._on_bt, 10)
        self.create_subscription(String, "semantic_status", self._on_status,
                                 10)
        # nav2 AMCL latches amcl_pose (reliable + transient_local, depth 1);
        # match it so we get the last pose even while the robot is idle
        from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                               QoSReliabilityPolicy)
        amcl_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose",
                                 self._on_pose, amcl_qos)
        self._last = None
        # Gazebo live views: keep the latest JPEG per camera and fan it out
        # to /stream/<key> MJPEG clients (compressed passthrough).
        from sensor_msgs.msg import CompressedImage
        from rclpy.qos import qos_profile_sensor_data
        self.frames = {"overhead": None, "robot": None}
        self.frame_cv = threading.Condition()
        self.create_subscription(
            CompressedImage, "/overhead/image_raw/compressed",
            lambda m: self._on_frame("overhead", m), qos_profile_sensor_data)
        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed",
            lambda m: self._on_frame("robot", m), qos_profile_sensor_data)

    def _on_frame(self, key, msg):
        with self.frame_cv:
            self.frames[key] = bytes(msg.data)
            self.frame_cv.notify_all()

    def _on_bt(self, msg):
        try:
            self.bt = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_status(self, msg):
        d = json.loads(msg.data)
        self.battery = d.get("battery", self.battery)
        if d["status"] != self._last:
            self._last = d["status"]
            self.status_log.append(d["status"])
            self.status_log = self.status_log[-40:]

    def _on_pose(self, msg):
        pp = msg.pose.pose.position
        self.robot_pose = (pp.x, pp.y)

    # ------------------------------------------------------------- state --
    def state(self):
        kg = json.load(open(self.kg_path))
        pl = json.load(open(self.places_path))
        names = {}
        try:
            book = json.load(open(self.naming_path))
            ep = book.get("T3_slic_rev4") or book.get("T3_slic") or {}
            names = {k: v["name"] for k, v in ep.items()
                     if isinstance(v, dict)}
        except (OSError, json.JSONDecodeError):
            pass
        objs = []
        for n in kg["nodes"]:
            if n.get("presence") == "absent":
                continue
            objs.append({"name": n["name"], "type": n.get("type"),
                         "status": n.get("status"),
                         "confidence": n.get("confidence"),
                         "x": round(n["pose"]["x"], 2),
                         "y": round(n["pose"]["y"], 2),
                         "isMovable": n.get("implicit", {}).get("isMovable")})
        objs.sort(key=lambda o: (str(o["status"]), o["name"]))
        try:
            sc = json.load(open(f"{BLK}/t3_place_scoped_relations.json"))
            keys = sc.get("keyObjects", {})
        except (OSError, json.JSONDecodeError):
            keys = {}
        places = []
        for p in pl["semanticPlaces"]:
            places.append({"region": p["name"],
                           "name": names.get(p["name"], p["name"]),
                           "members": p.get("memberCount"),
                           "verified": p.get("verifiedMemberCount"),
                           "key": keys.get(p["name"]),
                           "cx": round(p["centroid"][0], 2),
                           "cy": round(p["centroid"][1], 2)})
        return {"objects": objs, "places": places,
                "robots": kg.get("robots", []),
                "status_log": self.status_log,
                "battery": self.battery,
                "robot_pose": self.robot_pose,
                "bt": self.bt}

    # ------------------------------------------------------------- edits --
    def edit_object(self, req):
        kg = json.load(open(self.kg_path))
        name, action = req.get("name"), req.get("action")
        hit = None
        for n in kg["nodes"]:
            if n["name"] == name and n.get("presence") != "absent":
                hit = n
                break
        if hit is None:
            return {"error": f"no present object '{name}'"}
        hist = hit.setdefault("history", [])
        if action == "refute":
            hist.append({"revision": kg.get("revision"),
                         "change": "owner_ui_refuted", "was": hit["type"]})
            hit["presence"] = "absent"
            hit["status"] = "refuted_owner_ui"
        elif action == "set_type":
            t = (req.get("type") or "").strip()
            if not t or t == "null":
                return {"error": "empty type"}
            hist.append({"revision": kg.get("revision"),
                         "change": "owner_ui_type", "from": hit["type"],
                         "to": t})
            hit["type"] = t
            hit["status"] = "verified_owner"
            hit["confidence"] = 1.0
        elif action == "toggle_movable":
            imp = hit.setdefault("implicit", {})
            imp["isMovable"] = not imp.get("isMovable", False)
            hist.append({"revision": kg.get("revision"),
                         "change": "owner_ui_movable",
                         "to": imp["isMovable"]})
        else:
            return {"error": f"unknown action '{action}'"}
        json.dump(kg, open(self.kg_path, "w"), indent=1)
        return {"ok": True}

    def publish_command(self, cmd):
        self.cmd_pub.publish(String(data=json.dumps(cmd)))

    def publish_battery(self, level):
        from sensor_msgs.msg import BatteryState
        m = BatteryState()
        m.percentage = float(level)
        self.batt_pub.publish(m)


def make_handler(node):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json"):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/":
                self._send(200, PAGE, "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._send(200, json.dumps(node.state()))
            elif self.path.startswith("/stream/"):
                self._stream(self.path.rsplit("/", 1)[-1])
            else:
                self._send(404, "{}")

        def _stream(self, key):
            if key not in node.frames:
                return self._send(404, "{}")
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with node.frame_cv:
                        node.frame_cv.wait(timeout=1.0)
                        buf = node.frames.get(key)
                    if buf is None:
                        continue
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n"
                        b"Content-Length: %d\r\n\r\n" % len(buf))
                    self.wfile.write(buf)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                return self._send(400, '{"error":"bad json"}')
            if self.path == "/api/command":
                node.publish_command(req)
                self._send(200, '{"ok":true}')
            elif self.path == "/api/battery":
                node.publish_battery(req.get("level", 1.0))
                self._send(200, '{"ok":true}')
            elif self.path == "/api/object":
                self._send(200, json.dumps(node.edit_object(req)))
            else:
                self._send(404, "{}")
    return H


def main():
    rclpy.init()
    node = UIServer()
    srv = ThreadingHTTPServer(("0.0.0.0", node.port), make_handler(node))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(f"TOSM UI on http://localhost:{node.port}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    srv.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
