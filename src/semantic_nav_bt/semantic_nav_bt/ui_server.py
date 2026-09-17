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

import yaml
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

BLK = "/home/caselab/Downloads/Cyclone360_data/blk360_seg/outputs"


def load_map_png(yaml_path):
    """ROS map (yaml + pgm) -> (png bytes, info dict for the browser)."""
    import io
    import yaml as _yaml
    from PIL import Image
    y = _yaml.safe_load(open(yaml_path))
    pgm = os.path.join(os.path.dirname(yaml_path), y["image"])
    im = Image.open(pgm).convert("L")
    # trinary pgm: 254 free / 205 unknown / 0 occupied -> soften for the UI
    lut = [int(30 + v * 0.85) for v in range(256)]
    im = im.point(lut)
    buf = io.BytesIO()
    im.save(buf, format="PNG")
    info = {"res": float(y["resolution"]), "ox": float(y["origin"][0]),
            "oy": float(y["origin"][1]), "w": im.width, "h": im.height}
    return buf.getvalue(), info

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
      <button class=warn style="background:#7a0000;font-weight:700" onclick="cmd({cmd:'stop'})">&#9632; STOP (cancel all, hold)</button>
    </div>
    <div class=small style="margin-top:6px">battery <input id=batt type=range min=0 max=100 value=100
      oninput="setBatt(this.value)" style="width:130px"> <span id=battv>100%</span> (sim)</div>
  </div>
  <div class=card style="margin-top:12px"><h3>Mission status</h3>
    <div id=statuslog></div>
    <div class=small id=robotpose style="margin-top:6px"></div>
  </div>
  <div class=card style="margin-top:12px"><h3>Robot map (AMCL) &rarr; KG frame</h3>
    <div id=mapdrop style="border:2px dashed #9ab;border-radius:8px;padding:10px;text-align:center;color:#567;font-size:12.5px;cursor:pointer">
      drop the robot's <b>map .pgm + .yaml</b> here (or click to choose)<br>
      <input id=mapfiles type=file multiple accept=".pgm,.yaml,.yml" style="display:none"></div>
    <div class=small id=regmsg style="margin-top:6px"></div>
    <div class=small style="margin-top:4px">
      <label><input type=checkbox id=regoverlay checked onchange="drawMap()"> show registered robot map on the map panel</label>
      <label style="margin-left:8px"><input type=checkbox id=reginit checked> use stored offset as initial guess</label></div>
  </div>
  <div class=card style="margin-top:12px"><h3>3D semantic modeling from E57</h3>
    <div id=e57drop style="border:2px dashed #9ab;border-radius:8px;padding:10px;text-align:center;color:#567;font-size:12.5px;cursor:pointer">
      drop a registered <b>.e57</b> here (or click) &middot; or type a server path below<br>
      <input id=e57file type=file accept=".e57" style="display:none"></div>
    <div class=small id=e57msg style="margin-top:4px"></div>
    <div style="margin-top:6px"><input id=e57path placeholder="server path to .e57" style="width:96%"></div>
    <div style="margin-top:6px"><input id=pname placeholder="site / scan name" style="width:46%">
      <select id=pmode style="width:48%"><option value=new>new site (new map + KG)</option><option value=epoch>new epoch of the test room</option></select></div>
    <div class=small style="margin-top:6px">
      <label><input type=checkbox id=pvlm checked> VLM verification</label>
      budget $<input id=pbudget type=number value=5 step=0.5 style="width:50px">
      <label style="margin-left:6px"><input type=checkbox id=pover checked> overhead pass</label></div>
    <div style="margin-top:8px"><button onclick="startPipeline()" id=pstart>&#9654; Start 3D semantic modeling</button>
      <button class=warn onclick="fetch('/api/pipeline/stop',{method:'POST',body:'{}'})">stop</button>
      <button onclick="loadPipelineResult()" id=pload style="display:none">load result into console</button></div>
    <div id=psteps class=small style="margin-top:8px"></div>
    <div id=plog style="display:none;height:120px;overflow-y:auto;font-family:ui-monospace,monospace;font-size:11px;background:#101820;color:#cfe;border-radius:8px;padding:6px;margin-top:6px"></div>
  </div>
</div>
<div>
  <div class=card style="margin-bottom:12px"><h3>Gazebo live view</h3>
    <div style="display:flex;gap:12px;flex-wrap:wrap">
      <div><div class=small>overhead (world frame, +y up)
          <button style="padding:1px 8px" onclick="ovRot(-15)">&#10226;</button>
          <button style="padding:1px 8px" onclick="ovRot(15)">&#10227;</button>
          <button style="padding:1px 8px" onclick="ovReset()">reset</button>
          <span style="color:#999">wheel=zoom · drag=pan · right-drag=3D · hover=coords</span>
          <span id=ovcoord style="font-family:ui-monospace,monospace;color:#1d3557"></span></div>
        <div id=ovview style="width:427px;height:320px;overflow:hidden;border-radius:8px;
            background:#222;position:relative;cursor:grab">
          <div id=ovwrap style="position:absolute;left:0;top:0;transform-origin:213px 160px">
            <img id=ovimg src="/stream/overhead" style="display:block;height:320px">
            <canvas id=ovcanvas style="position:absolute;left:0;top:0;pointer-events:none"></canvas>
            <div id=robotmark style="position:absolute;width:13px;height:13px;border:3px solid #e63946;
              border-radius:50%;box-shadow:0 0 7px #e63946;transform:translate(-50%,-50%);
              display:none;pointer-events:none"></div>
          <div id=xhair style="position:absolute;width:15px;height:15px;
              border:1.5px solid #1d3557;border-radius:50%;background:rgba(29,53,87,.15);
              transform:translate(-50%,-50%);display:none;pointer-events:none"></div>
          </div>
        </div></div>
      <div><div class=small>semantic map (occupancy grid, KG frame)
          <button style="padding:1px 8px" onclick="mapMode('init')" id=b_init>set initial pose</button>
          <button style="padding:1px 8px" onclick="mapMode('goal')" id=b_goal>send goal</button>
          <button style="padding:1px 8px" onclick="mapMode(null)">cancel</button>
          <button style="padding:1px 8px" onclick="mapReset()">reset view</button>
          <span id=maphint style="color:#999">wheel=zoom · drag=pan · in a mode: press, drag for heading, release</span>
          <span id=mapcoord style="font-family:ui-monospace,monospace;color:#1d3557"></span></div>
        <div id=mapview style="width:427px;height:320px;overflow:hidden;border-radius:8px;
            background:#333;position:relative;cursor:grab">
          <div id=mapwrap style="position:absolute;left:0;top:0;transform-origin:0 0">
            <img id=mapimg src="/map.png" style="display:block;image-rendering:pixelated">
            <canvas id=mapcanvas style="position:absolute;left:0;top:0;pointer-events:none"></canvas>
          </div>
        </div></div>
    </div>
  </div>
  <div>
    <span class="tab on" id=t_obj onclick="tab('obj')">Objects</span>
    <span class=tab id=t_pla onclick="tab('pla')">Places</span>
    <span class=tab id=t_rob onclick="tab('rob')">Robots</span>
    <span class=tab id=t_bt onclick="tab('bt')">Behavior Tree</span>
  </div>
  <div class=tbox id=tbl></div>
  <div class=tbox id=jsonpanel style="display:none">
   <b id=jsontitle></b>
   <div class=small>raw DB record — edits write straight back to the json
   store (objects get a history entry); mediator / Isaac watcher / this
   console all pick it up from the file</div>
   <textarea id=jsontext spellcheck=false
     style="width:100%;height:280px;font-family:monospace;font-size:12px;
     margin-top:6px"></textarea><br>
   <button onclick="saveJson()">save to DB</button>
   <button onclick="document.getElementById('jsonpanel').style.display='none'">close</button>
   <span id=jsonmsg class=small></span>
  </div>
</div>
</div>
<script>
let CUR='obj', STATE=null, LOG=[];
function tab(t){CUR=t;for(const x of ['obj','pla','rob','bt'])
  document.getElementById('t_'+x).className='tab'+(x===t?' on':'');render();}
// ---- overhead view: zoom / pan / 2D rotate / 3D orbit + overlays ---------
let OV={s:1,tx:0,ty:0,r:0}, SELS=[], CAM={mode:'top',az:90,el:60,t:0};
const PAL=['#e6194b','#3cb44b','#d4a800','#4363d8','#f58231','#911eb4',
  '#0aa6a6','#f032e6','#7a9e12','#c05858','#008080','#8b6fc9','#9a6324',
  '#800000','#3f9e6e','#808000','#b05e2c','#000075'];
function objColor(name){const i=STATE?STATE.objects.findIndex(o=>o.name===name):0;
  return PAL[(i<0?0:i)%PAL.length];}
function ovApply(){document.getElementById('ovwrap').style.transform=
  `translate(${OV.tx}px,${OV.ty}px) rotate(${OV.r}deg) scale(${OV.s})`;}
function ovRot(d){OV.r+=d;ovApply();}
function sendCam(body){const now=Date.now();
  if(body.reset||now-CAM.t>150){CAM.t=now;
    fetch('/api/campose',{method:'POST',body:JSON.stringify(body)});}}
function ovReset(){OV={s:1,tx:0,ty:0,r:0};ovApply();
  CAM.mode='top';CAM.az=90;CAM.el=60;sendCam({reset:true});
  drawOverlay();poll();}
window.addEventListener('load',()=>{const v=document.getElementById('ovview');
  v.addEventListener('contextmenu',e=>e.preventDefault());
  v.addEventListener('wheel',e=>{e.preventDefault();
    OV.s=Math.min(6,Math.max(0.5,OV.s*(e.deltaY<0?1.15:1/1.15)));ovApply();},{passive:false});
  let dr=null,rdr=null;
  v.addEventListener('mousedown',e=>{e.preventDefault();
    if(e.button===2){rdr=[e.clientX,e.clientY];}
    else{dr=[e.clientX,e.clientY];v.style.cursor='grabbing';}});
  window.addEventListener('mousemove',e=>{
    if(dr){OV.tx+=e.clientX-dr[0];OV.ty+=e.clientY-dr[1];
      dr=[e.clientX,e.clientY];ovApply();}
    if(rdr){CAM.mode='3d';
      CAM.az-=(e.clientX-rdr[0])*0.5;
      CAM.el=Math.min(88,Math.max(15,CAM.el+(e.clientY-rdr[1])*0.4));
      rdr=[e.clientX,e.clientY];sendCam({az:CAM.az,el:CAM.el});drawOverlay();}});
  window.addEventListener('mouseup',e=>{
    if(rdr&&e.button===2){sendCam({az:CAM.az,el:CAM.el});}
    dr=null;rdr=null;v.style.cursor='grab';});
  // hover readout: invert the CSS transform (translate ∘ rotate ∘ scale about
  // the 213,160 origin), then the linear overhead-camera projection
  v.addEventListener('mousemove',e=>{
    const out=document.getElementById('ovcoord'),xh=document.getElementById('xhair');
    if(CAM.mode!=='top'){out.textContent=' 3D view — press reset for coords';
      xh.style.display='none';return;}
    const img=document.getElementById('ovimg');
    if(!img.clientWidth){return;}
    const R=v.getBoundingClientRect();
    const px=e.clientX-R.left-OV.tx, py=e.clientY-R.top-OV.ty;
    const ORX=213,ORY=160,a=-OV.r*Math.PI/180;
    const dx=px-ORX,dy=py-ORY;
    const qx=ORX+(dx*Math.cos(a)-dy*Math.sin(a))/OV.s;
    const qy=ORY+(dx*Math.sin(a)+dy*Math.cos(a))/OV.s;
    const PPM=48.57;
    const u=qx*960/img.clientWidth, w=qy*720/img.clientHeight;
    const X=(u-480)/PPM-2.0, Y=(360-w)/PPM-1.5;
    xh.style.left=qx+'px';xh.style.top=qy+'px';xh.style.display='block';
    let near='';
    if(STATE){let best=null;
      for(const o of STATE.objects){const d=Math.hypot(o.x-X,o.y-Y);
        if(!best||d<best[0])best=[d,o];}
      if(best&&best[0]<2.0)near=` · nearest ${best[1].name} (${best[1].type}) ${best[0].toFixed(2)} m`;}
    out.textContent=` x=${X.toFixed(2)} y=${Y.toFixed(2)}${near}`;});
  v.addEventListener('mouseleave',()=>{
    document.getElementById('xhair').style.display='none';});});
function pickObj(name){const i=SELS.findIndex(s=>s.kind==='obj'&&s.o.name===name);
  if(i>=0){SELS.splice(i,1);drawOverlay();render();return;}
  const o=STATE.objects.find(v=>v.name===name);
  SELS.push({kind:'obj',o:o});drawOverlay();drawMap();render();}
function pickPlace(region){const i=SELS.findIndex(s=>s.kind==='place'&&s.region===region);
  if(i>=0){SELS.splice(i,1);drawOverlay();render();return;}
  fetch('/api/region?name='+region).then(r=>r.json()).then(c=>{
    SELS.push({kind:'place',region:region,cells:c});drawOverlay();drawMap();render();});}
function drawOverlay(){const img=document.getElementById('ovimg'),
  cv=document.getElementById('ovcanvas');
  if(!img||!img.clientWidth)return;
  cv.width=img.clientWidth;cv.height=img.clientHeight;
  const ctx=cv.getContext('2d');ctx.clearRect(0,0,cv.width,cv.height);
  if(CAM.mode==='3d'||!SELS.length)return;   // overlays valid only top-down
  const PPM=48.57,sx=img.clientWidth/960,sy=img.clientHeight/720;
  const wx=x=>(480+(x+2.0)*PPM)*sx, wy=y=>(360-(y+1.5)*PPM)*sy;
  for(const S of SELS){
    if(S.kind==='place'){const c=S.cells,cs=Math.max(1.5,c.cs*PPM*sx);
      const [r,g,b]=c.color;
      ctx.fillStyle=`rgba(${r},${g},${b},.45)`;
      for(const p of c.cells){ctx.fillRect(wx(p[0])-cs/2,wy(p[1])-cs/2,cs,cs);}}}
  for(const S of SELS){
    if(S.kind==='obj'){const o=S.o,cx=wx(o.x),cy=wy(o.y),col=objColor(o.name);
      ctx.save();ctx.translate(cx,cy);ctx.rotate(-(o.theta||0));
      const w=(o.length||0.5)*PPM*sx,h=(o.width||0.5)*PPM*sy;
      ctx.fillStyle=col+'4d';ctx.strokeStyle=col;ctx.lineWidth=2.5;
      ctx.fillRect(-w/2,-h/2,w,h);ctx.strokeRect(-w/2,-h/2,w,h);ctx.restore();
      ctx.font='bold 11px system-ui';ctx.fillStyle=col;
      ctx.fillText(o.name,cx+5,cy-5);}}}
// ---- occupancy-grid map panel: KG-frame overlays, initial pose / goal ------
let MV={s:1,tx:0,ty:0}, MAPMODE=null, MDRAG=null, MAPINFO=null;
function mapApply(){document.getElementById('mapwrap').style.transform=
  `translate(${MV.tx}px,${MV.ty}px) scale(${MV.s})`;}
function mapReset(){const img=document.getElementById('mapimg');
  const v=document.getElementById('mapview');
  if(img.naturalWidth){MV.s=Math.min(v.clientWidth/img.naturalWidth,v.clientHeight/img.naturalHeight);
    MV.tx=(v.clientWidth-img.naturalWidth*MV.s)/2;MV.ty=(v.clientHeight-img.naturalHeight*MV.s)/2;}
  mapApply();drawMap();}
function mapMode(m){MAPMODE=m;MDRAG=null;
  document.getElementById('b_init').style.background=m==='init'?'#b23b3b':'';
  document.getElementById('b_goal').style.background=m==='goal'?'#2f855a':'';
  document.getElementById('maphint').textContent=m?
    (m==='init'?' INITIAL POSE: press at the robot position, drag toward its front, release':
     ' GOAL: press at the target, drag toward the desired heading, release'):
    ' wheel=zoom · drag=pan · in a mode: press, drag for heading, release';
  document.getElementById('mapview').style.cursor=m?'crosshair':'grab';drawMap();}
// world <-> map-image pixel (ROS map yaml: origin at bottom-left, +y up)
function w2p(x,y){const M=MAPINFO;return [(x-M.ox)/M.res,M.h-(y-M.oy)/M.res];}
function p2w(u,v){const M=MAPINFO;return [M.ox+u*M.res,M.oy+(M.h-v)*M.res];}
function mapPix(e){const v=document.getElementById('mapview').getBoundingClientRect();
  return [(e.clientX-v.left-MV.tx)/MV.s,(e.clientY-v.top-MV.ty)/MV.s];}
window.addEventListener('load',()=>{const v=document.getElementById('mapview');
  const img=document.getElementById('mapimg');
  img.addEventListener('load',mapReset);
  v.addEventListener('wheel',e=>{e.preventDefault();const [u0,v0]=mapPix(e);
    const f=e.deltaY<0?1.15:1/1.15;MV.s=Math.min(12,Math.max(0.2,MV.s*f));
    const R=v.getBoundingClientRect();MV.tx=e.clientX-R.left-u0*MV.s;MV.ty=e.clientY-R.top-v0*MV.s;
    mapApply();},{passive:false});
  let pan=null;
  v.addEventListener('mousedown',e=>{e.preventDefault();if(e.button!==0)return;
    if(MAPMODE&&MAPINFO){const [u,w]=mapPix(e);MDRAG={p0:p2w(u,w),p1:null};drawMap();}
    else{pan=[e.clientX,e.clientY];v.style.cursor='grabbing';}});
  window.addEventListener('mousemove',e=>{
    if(pan){MV.tx+=e.clientX-pan[0];MV.ty+=e.clientY-pan[1];pan=[e.clientX,e.clientY];mapApply();}
    if(MDRAG){const [u,w]=mapPix(e);MDRAG.p1=p2w(u,w);drawMap();}});
  v.addEventListener('mousemove',e=>{if(!MAPINFO)return;const [u,w]=mapPix(e);const [x,y]=p2w(u,w);
    let near='';if(STATE){let best=null;for(const o of STATE.objects){const d=Math.hypot(o.x-x,o.y-y);
      if(!best||d<best[0])best=[d,o];}
      if(best&&best[0]<2.0)near=` · ${best[1].name} ${best[0].toFixed(2)} m`;}
    document.getElementById('mapcoord').textContent=` x=${x.toFixed(2)} y=${y.toFixed(2)}${near}`;});
  window.addEventListener('mouseup',e=>{
    if(MDRAG){const p0=MDRAG.p0,p1=MDRAG.p1||p0;
      const yaw=(p1===p0||Math.hypot(p1[0]-p0[0],p1[1]-p0[1])<0.05)?null:Math.atan2(p1[1]-p0[1],p1[0]-p0[0]);
      const body={x:p0[0],y:p0[1],yaw:yaw};
      fetch(MAPMODE==='init'?'/api/initialpose':'/api/goal',{method:'POST',body:JSON.stringify(body)});
      MDRAG=null;mapMode(null);}
    pan=null;if(!MAPMODE)v.style.cursor='grab';});});
function drawMap(){const img=document.getElementById('mapimg'),cv=document.getElementById('mapcanvas');
  if(!img.naturalWidth||!MAPINFO)return;cv.width=img.naturalWidth;cv.height=img.naturalHeight;
  const ctx=cv.getContext('2d');ctx.clearRect(0,0,cv.width,cv.height);const m=1/MAPINFO.res;
  if(ROBMAP&&document.getElementById('regoverlay').checked){ctx.fillStyle='rgba(255,140,0,.55)';
    for(const p of ROBMAP){const [u,w]=w2p(p[0],p[1]);ctx.fillRect(u-0.6,w-0.6,1.4,1.4);}}
  if(!STATE)return;
  // places (selected) -> cells
  for(const S of SELS){if(S.kind==='place'){const c=S.cells,cs=Math.max(1,c.cs*m);const [r,g,b]=c.color;
    ctx.fillStyle=`rgba(${r},${g},${b},.45)`;
    for(const p of c.cells){const [u,w]=w2p(p[0],p[1]);ctx.fillRect(u-cs/2,w-cs/2,cs,cs);}}}
  // all objects: verified colored, unverified gray; selected ones bold
  for(const o of STATE.objects){const [u,w]=w2p(o.x,o.y);const sel=SELS.some(S=>S.kind==='obj'&&S.o.name===o.name);
    const ver=o.status&&o.status.startsWith('verified');const col=ver?objColor(o.name):'#8a8f98';
    ctx.save();ctx.translate(u,w);ctx.rotate(-(o.theta||0));
    const L=(o.length||0.5)*m,W=(o.width||0.5)*m;
    ctx.fillStyle=col+(sel?'80':'33');ctx.strokeStyle=col;ctx.lineWidth=sel?3:1.2;
    ctx.fillRect(-L/2,-W/2,L,W);ctx.strokeRect(-L/2,-W/2,L,W);ctx.restore();
    if(sel||o.level==='high'){ctx.font=(sel?'bold ':'')+'9px system-ui';ctx.fillStyle=col;ctx.fillText(o.name,u+3,w-3);}}
  // robot with heading
  if(STATE.robot_pose){const [u,w]=w2p(STATE.robot_pose[0],STATE.robot_pose[1]);const th=STATE.robot_pose[2]||0;
    ctx.save();ctx.translate(u,w);ctx.rotate(-th);ctx.strokeStyle='#e63946';ctx.fillStyle='rgba(230,57,70,.35)';ctx.lineWidth=2;
    ctx.beginPath();ctx.arc(0,0,0.45*m,0,6.283);ctx.fill();ctx.stroke();
    ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(0.9*m,0);ctx.stroke();ctx.restore();}
  // drag preview (initial pose / goal arrow)
  if(MDRAG){const [u,w]=w2p(MDRAG.p0[0],MDRAG.p0[1]);ctx.strokeStyle=MAPMODE==='init'?'#b23b3b':'#2f855a';ctx.lineWidth=3;
    ctx.beginPath();ctx.arc(u,w,0.3*m,0,6.283);ctx.stroke();
    if(MDRAG.p1){const [u1,w1]=w2p(MDRAG.p1[0],MDRAG.p1[1]);ctx.beginPath();ctx.moveTo(u,w);ctx.lineTo(u1,w1);ctx.stroke();}}}
// ---- robot map upload -> registration -> bridge offset ----------------
let ROBMAP=null;
window.addEventListener('load',()=>{const dz=document.getElementById('mapdrop'),fi=document.getElementById('mapfiles');
  dz.addEventListener('click',()=>fi.click());
  dz.addEventListener('dragover',e=>{e.preventDefault();dz.style.background='#eef4fb';});
  dz.addEventListener('dragleave',()=>{dz.style.background='';});
  dz.addEventListener('drop',e=>{e.preventDefault();dz.style.background='';sendMapFiles(e.dataTransfer.files);});
  fi.addEventListener('change',()=>sendMapFiles(fi.files));});
function sendMapFiles(files){const fd=new FormData();let n=0;
  for(const f of files){if(/\\.(pgm|yaml|yml)$/i.test(f.name)){fd.append('file',f,f.name);n++;}}
  if(n<2){document.getElementById('regmsg').textContent=' need both .pgm and .yaml';return;}
  fd.append('use_init',document.getElementById('reginit').checked?'1':'0');
  document.getElementById('regmsg').textContent=' uploading + registering...';
  fetch('/api/robotmap',{method:'POST',body:fd}).then(r=>r.json()).then(d=>{
    if(d.error){document.getElementById('regmsg').textContent=' '+d.error;return;}
    const o=d.map_offset,st=d.stats;
    document.getElementById('regmsg').innerHTML=`offset X ${o[0].toFixed(3)} Y ${o[1].toFixed(3)} YAW ${o[2].toFixed(2)}&deg; &middot; mean ${(st.mean_m*100).toFixed(1)} cm, inliers&lt;15cm ${(st.inlier_frac_15cm*100).toFixed(0)}% &middot; sent to bridge`;
    ROBMAP=d.cells;drawMap();});}
// ---- E57 upload + pipeline control ----------------------------------------
window.addEventListener('load',()=>{const dz=document.getElementById('e57drop'),fi=document.getElementById('e57file');
  dz.addEventListener('click',()=>fi.click());
  dz.addEventListener('dragover',e=>{e.preventDefault();dz.style.background='#eef4fb';});
  dz.addEventListener('dragleave',()=>{dz.style.background='';});
  dz.addEventListener('drop',e=>{e.preventDefault();dz.style.background='';if(e.dataTransfer.files[0])uploadE57(e.dataTransfer.files[0]);});
  fi.addEventListener('change',()=>{if(fi.files[0])uploadE57(fi.files[0]);});
  pollPipeline();setInterval(pollPipeline,3000);});
function uploadE57(f){const msg=document.getElementById('e57msg');
  msg.textContent=` uploading ${f.name} (${(f.size/1e9).toFixed(2)} GB)...`;
  const xhr=new XMLHttpRequest();xhr.open('PUT','/api/e57?name='+encodeURIComponent(f.name));
  xhr.upload.onprogress=e=>{if(e.lengthComputable)msg.textContent=` uploading ${f.name}: ${(100*e.loaded/e.total).toFixed(0)}%`;};
  xhr.onload=()=>{try{const d=JSON.parse(xhr.responseText);
    if(d.path){document.getElementById('e57path').value=d.path;msg.textContent=' stored: '+d.path;
      if(!document.getElementById('pname').value)document.getElementById('pname').value=f.name.replace(/\\.e57$/i,'');}
    else msg.textContent=' '+(d.error||'upload failed');}catch(e){msg.textContent=' upload failed';}};
  xhr.onerror=()=>{msg.textContent=' upload failed';};xhr.send(f);}
function startPipeline(){const body={e57:document.getElementById('e57path').value,name:document.getElementById('pname').value,
  mode:document.getElementById('pmode').value,vlm:document.getElementById('pvlm').checked,
  budget_usd:parseFloat(document.getElementById('pbudget').value||'0'),overhead:document.getElementById('pover').checked};
  if(!body.e57||!body.name){document.getElementById('psteps').textContent='need an E57 path and a name';return;}
  fetch('/api/pipeline/start',{method:'POST',body:JSON.stringify(body)}).then(r=>r.json()).then(d=>{
    document.getElementById('psteps').textContent=d.error?d.error:'started';pollPipeline();});}
const STEPICON={running:'&#9654;',done:'&#10003;',skipped:'&#8211;',failed:'&#10007;'};
function pollPipeline(){fetch('/api/pipeline/status').then(r=>r.json()).then(d=>{
  const el=document.getElementById('psteps'),lg=document.getElementById('plog');
  if(!d||!d.steps){return;}
  let h=`<b>${d.job.name}</b> &middot; ${d.state} &middot; cost $${(d.cost_usd||0).toFixed(2)}<br>`;
  for(const s of d.steps){h+=`<div style="color:${s.state==='failed'?'#b23b3b':s.state==='running'?'#1d3557':'#2f855a'}">${STEPICON[s.state]||''} ${s.label}${s.note?' <span style=color:#666>'+s.note+'</span>':''}</div>`;}
  if(d.error)h+=`<div style=color:#b23b3b>${d.error}</div>`;
  el.innerHTML=h;lg.style.display='block';lg.innerHTML=(d.log||[]).slice(-40).map(x=>`<div>${x.replace(/</g,'&lt;')}</div>`).join('');lg.scrollTop=lg.scrollHeight;
  document.getElementById('pload').style.display=d.state==='done'?'':'none';}).catch(()=>{});}
function loadPipelineResult(){fetch('/api/pipeline/load',{method:'POST',body:'{}'}).then(r=>r.json()).then(d=>{
  document.getElementById('psteps').innerHTML+=`<div>${d.error||('console switched to '+d.kg_path+' (restart mediator/BT with the same paths for missions)')}</div>`;
  ROBMAP=null;MAPINFO=null;document.getElementById('mapimg').src='/map.png?'+Date.now();poll();});}
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
if(CUR==='obj'){h=`<div class=small style="margin:2px 0 8px;line-height:1.5">
   <b>Owner actions</b> — write straight back to the knowledge graph (with a
   history entry); the mission mediator hot-reloads the file, so an edit
   changes robot behavior from the next command, no restart.
   &nbsp;<b>refute</b>: this object does not exist (phantom) → marked absent,
   excluded from goals &nbsp;·&nbsp; <b>type</b>: correct the label → promoted
   to <span class="badge o">verified_owner</span> (conf 1.0)
   &nbsp;·&nbsp; <b>movable</b>: toggle isMovable (implicit layer; arm-pickup
   eligibility)</div>`;
  h+='<table><tr><th>name</th><th>type</th><th>status</th><th>conf</th><th>pose</th><th>level</th><th>movable</th><th>owner actions</th></tr>';
  for(const o of d.objects){const on=SELS.some(s=>s.kind==='obj'&&s.o.name===o.name);
   const lv=o.level?`<span title="height level: low = crossed by the 2D-lidar scan plane, mid = elevated (desk/wall), high = ceiling">${o.level}</span>`:'';
   const un=o.labelStability==='unstable'?` <span title="label unstable across structure-condition ensemble (raw / no-ceiling / removed) — owner check recommended" style="cursor:help">⚠</span>`:'';
   h+=`<tr style="cursor:pointer${on?';background:'+objColor(o.name)+'33':''}" onclick="pickObj('${o.name}')">
   <td>${o.name}</td><td>${o.type}${un}</td><td>${stBadge(o.status)}</td>
   <td>${o.confidence??''}</td><td>(${o.x},${o.y})</td><td>${lv}</td><td>${o.isMovable?'✓':'✗'}</td>
   <td><button title="Owner refutation: '${o.name}' does not exist in the room (phantom / structure noise). Marks it absent in the KG — the mediator will refuse it as a goal from the next command."
     onclick="event.stopPropagation();edit('${o.name}','refute')">refute</button>
   <button title="Correct the semantic label of '${o.name}'. The new type is stored as verified_owner with confidence 1.0 and survives later re-scans (owner feedback outranks the verifier)."
     onclick="event.stopPropagation();edit('${o.name}','set_type',{type:prompt('correct type for ${o.name}:','${o.type}')})">type</button>
   <button title="Toggle isMovable for '${o.name}' (implicit layer). Controls e.g. whether a pick mission is allowed on this object."
     onclick="event.stopPropagation();edit('${o.name}','toggle_movable')">movable</button>
   <button title="Open the raw DB record of '${o.name}' for direct JSON editing."
     onclick="event.stopPropagation();openJson('obj','${o.name}')">json</button></td></tr>`;}
  h+='</table>';}
if(CUR==='pla'){h=`<div class=small style="margin:2px 0 8px">click a row to shade the
   place's SLIC segmentation cells on the overhead view</div>`;
  h+='<table><tr><th>region</th><th>name</th><th>members</th><th>verified</th><th>key object</th><th>centroid</th><th></th></tr>';
  for(const p of d.places){const s=SELS.find(v=>v.kind==='place'&&v.region===p.region);
   const bg=s?`;background:rgba(${s.cells.color[0]},${s.cells.color[1]},${s.cells.color[2]},.25)`:'';
   h+=`<tr style="cursor:pointer${bg}" onclick="pickPlace('${p.region}')">
   <td>${p.region}</td><td><b>${p.name}</b></td><td>${p.members}</td>
   <td>${p.verified}</td><td>${p.key||''}</td><td>(${p.cx},${p.cy})</td>
   <td><button onclick="event.stopPropagation();openJson('place','${p.region}')">json</button></td></tr>`;}h+='</table>';}
if(CUR==='bt'){h=d.bt?btSvg(d.bt):
  '<div class=small>no /bt_snapshot yet — is mission_bt running?</div>';}
if(CUR==='rob'){for(const r of d.robots){h+=`<table><tr><th colspan=2>${r.name} (${r.symbolic.type}, ${r.symbolic.drive})
  <button style="float:right" onclick="openJson('robot','${r.name}')">json</button></th></tr>`;
  h+=`<tr><td>pose</td><td>(${r.explicit.pose.x}, ${r.explicit.pose.y})</td></tr>`;
  h+=`<tr><td>footprint</td><td>${r.explicit.footprint.length} × ${r.explicit.footprint.width} × ${r.explicit.footprint.height} m</td></tr>`;
  h+=`<tr><td>limits</td><td>${JSON.stringify(r.explicit.limits)}</td></tr>`;
  h+=`<tr><td>sensors</td><td>${r.explicit.sensors.map(s=>s.joint).join(', ')}</td></tr>`;
  h+=`<tr><td>implicit</td><td>${JSON.stringify(r.implicit)}</td></tr>`;
  h+=`<tr><td>place</td><td>${r.isInsideOf}</td></tr></table><br>`;}}
document.getElementById('tbl').innerHTML=h;}
let JEDIT=null;
function openJson(kind,name){
  fetch('/api/getjson',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({kind,name})}).then(r=>r.json()).then(d=>{
    if(d.error){alert(d.error);return;}
    JEDIT={kind,name};
    document.getElementById('jsontitle').textContent=`${kind}: ${name}`;
    document.getElementById('jsontext').value=JSON.stringify(d.data,null,1);
    document.getElementById('jsonmsg').textContent='';
    document.getElementById('jsonpanel').style.display='block';
    document.getElementById('jsonpanel').scrollIntoView({behavior:'smooth'});});}
function saveJson(){
  if(!JEDIT)return;
  let data;
  try{data=JSON.parse(document.getElementById('jsontext').value);}
  catch(e){document.getElementById('jsonmsg').textContent=' invalid JSON: '+e.message;return;}
  fetch('/api/setjson',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({kind:JEDIT.kind,name:JEDIT.name,data})})
   .then(r=>r.json()).then(d=>{
    document.getElementById('jsonmsg').textContent=d.ok?' saved \u2713':(' '+(d.error||'failed'));});}
function poll(){fetch('/api/state').then(r=>r.json()).then(d=>{STATE=d;
  document.getElementById('conn').textContent=' — live';
  const sp=document.getElementById('selPlace');
  if(sp.options.length!==d.places.length){sp.innerHTML='';
    for(const p of d.places){sp.add(new Option(p.name,p.region));}}
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
  if(CAM.mode==='3d'){mk.style.display='none';}
  else if(d.robot_pose&&img.clientWidth>0){const PPM=48.57;
    const u=480+(d.robot_pose[0]+2.0)*PPM, v=360-(d.robot_pose[1]+1.5)*PPM;
    mk.style.left=(img.offsetLeft+u*img.clientWidth/960)+'px';
    mk.style.top=(img.offsetTop+v*img.clientHeight/720)+'px';
    mk.style.display='block';}
  if(d.map)MAPINFO=d.map;if(!ROBMAP&&d.robot_map_cells)ROBMAP=d.robot_map_cells;
  drawOverlay();drawMap();
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
        # occupancy grid shown in the map panel (KG frame; the same map the
        # place layer and the Gazebo world are built on)
        self.map_yaml = p("map_yaml",
                          "/home/caselab/ammr_twin/map_vis_n2_1.yaml").value
        self.map_png, self.map_info = None, None
        try:
            self.map_png, self.map_info = load_map_png(self.map_yaml)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"map not loaded ({self.map_yaml}): {e}")
        from geometry_msgs.msg import PoseStamped
        # operator-picked initial pose / goal from the map panel. Sim: these
        # are Nav2's own topics; real robot: remap to /kg_initialpose and
        # /kg_goal_pose so the bridge converts KG frame -> robot frame.
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped,
                                              "initialpose", 10)
        # robot AMCL map -> KG frame offset: stored on disk, latched to the
        # bridge on /kg_map_offset (String JSON {"map_offset":[X,Y,YAW_DEG]})
        from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                               QoSReliabilityPolicy)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.offset_pub = self.create_publisher(String, "kg_map_offset", latched)
        self.offset_path = p("offset_path",
                             os.path.expanduser("~/ammr_twin/robot_map_offset.json")).value
        self.robot_map_cells = None
        try:
            off = json.load(open(self.offset_path))
            self.offset_pub.publish(String(data=json.dumps(off)))
            from .map_register import transformed_cells
            if os.path.exists(off.get("robot_yaml", "")):
                self.robot_map_cells = transformed_cells(off["robot_yaml"], off["map_offset"])
        except (OSError, json.JSONDecodeError, KeyError):
            pass
        self.goal_pub = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.cmd_pub = self.create_publisher(String, "semantic_command", 10)
        # operator STOP fan-out: the nav bridge / robot side listens here too
        self.stop_pub = self.create_publisher(String, "semantic_stop", 10)
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
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.robot_pose = (pp.x, pp.y, yaw)

    # ------------------------------------------------ E57 -> semantic model --
    SEG = os.path.dirname(BLK)

    def pipeline_start(self, req):
        import subprocess
        if getattr(self, "pipe_proc", None) and self.pipe_proc.poll() is None:
            return {"error": "a pipeline is already running"}
        e57 = req.get("e57", "")
        if not os.path.exists(e57):
            return {"error": f"E57 not found: {e57}"}
        name = "".join(c if c.isalnum() or c in "-_" else "_" for c in req.get("name", "scan"))
        job = {"name": name, "e57": e57, "mode": req.get("mode", "new"),
               "map_id": "testroom" if req.get("mode") == "epoch" else name,
               "vlm": bool(req.get("vlm", True)), "budget_usd": float(req.get("budget_usd", 5.0)),
               "overhead": bool(req.get("overhead", True)),
               "kg_graph": self.kg_path, "kg_map_yaml": self.map_yaml,
               "bounds": "outputs/vis_n2_room_bounds.json",
               "ref_clean": "outputs/vis_n2_det_filt/clean.ply"}
        jdir = os.path.join(BLK, f"{name}_objects"); os.makedirs(jdir, exist_ok=True)
        jpath = os.path.join(jdir, "job.json"); json.dump(job, open(jpath, "w"), indent=1)
        self.pipe_status = os.path.join(jdir, "pipeline_status.json")
        if os.path.exists(self.pipe_status):
            os.remove(self.pipe_status)
        log = open(os.path.join(jdir, "pipeline.log"), "a")
        self.pipe_proc = subprocess.Popen(
            [os.path.join(self.SEG, ".venv", "bin", "python"),
             os.path.join(self.SEG, "scripts", "semantic_pipeline.py"), "--job", jpath],
            cwd=self.SEG, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.status_log.append(f"semantic modeling started: {name} ({job['mode']})")
        return {"ok": True, "job": job}

    def pipeline_status(self):
        p = getattr(self, "pipe_status", None)
        if not p or not os.path.exists(p):
            return {}
        try:
            d = json.load(open(p))
        except (OSError, json.JSONDecodeError):
            return {}
        proc = getattr(self, "pipe_proc", None)
        if d.get("state") == "running" and proc is not None and proc.poll() is not None:
            d["state"] = "failed"; d["error"] = f"pipeline process exited ({proc.returncode})"
        return d

    def pipeline_stop(self):
        import signal
        proc = getattr(self, "pipe_proc", None)
        if proc and proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            return {"ok": True}
        return {"error": "nothing running"}

    def pipeline_load(self):
        d = self.pipeline_status()
        if d.get("state") != "done":
            return {"error": "no finished pipeline"}
        o = d["outputs"]
        self.kg_path, self.places_path = o["kg"], o["places"]
        self.naming_path, self.naming_key = o["naming"], o.get("naming_key")
        if o.get("map_yaml") and os.path.exists(o["map_yaml"]):
            self.map_yaml = o["map_yaml"]
            try:
                self.map_png, self.map_info = load_map_png(self.map_yaml)
            except Exception as e:  # noqa: BLE001
                return {"error": f"map reload failed: {e}"}
        self.robot_map_cells = None
        self.status_log.append(f"console switched to {os.path.basename(self.kg_path)}")
        return {"ok": True, "kg_path": self.kg_path, "map_yaml": self.map_yaml}

    def register_robot_map(self, files, use_init=True):
        """files: {name: bytes} with one .pgm and one .yaml. Saves them,
        registers onto the KG map, stores + publishes the offset."""
        import datetime
        from .map_register import register, transformed_cells
        d = os.path.expanduser(f"~/ammr_twin/robot_maps/{datetime.datetime.now():%Y%m%d_%H%M%S}")
        os.makedirs(d, exist_ok=True)
        ypath = None
        for name, data in files.items():
            path = os.path.join(d, os.path.basename(name))
            open(path, "wb").write(data)
            if name.lower().endswith((".yaml", ".yml")):
                ypath = path
        if ypath is None:
            return {"error": "no .yaml among the uploaded files"}
        y = yaml.safe_load(open(ypath))
        if not os.path.exists(os.path.join(d, os.path.basename(y.get("image", "")))):
            return {"error": f"yaml 'image: {y.get('image')}' not among the uploaded files"}
        y["image"] = os.path.basename(y["image"])
        yaml.safe_dump(y, open(ypath, "w"))
        init = None
        if use_init and os.path.exists(self.offset_path):
            try:
                init = json.load(open(self.offset_path))["map_offset"]
            except (OSError, json.JSONDecodeError, KeyError):
                init = None
        log = self.get_logger().info
        try:
            out = register(ypath, self.map_yaml, init, 30.0 if init else None, log=log)
        except Exception as e:  # noqa: BLE001
            return {"error": f"registration failed: {e}"}
        out["timestamp"] = datetime.datetime.now().isoformat(timespec="seconds")
        json.dump(out, open(self.offset_path, "w"), indent=1)
        self.offset_pub.publish(String(data=json.dumps(out)))
        self.robot_map_cells = transformed_cells(ypath, out["map_offset"])
        out["cells"] = self.robot_map_cells
        self.status_log.append(
            f"robot map registered: offset {out['map_offset']} "
            f"(mean {out['stats']['mean_m']*100:.1f} cm)")
        return out

    def publish_initialpose(self, req):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = float(req["x"])
        m.pose.pose.position.y = float(req["y"])
        yaw = req.get("yaw")
        if yaw is None:
            yaw = self.robot_pose[2] if self.robot_pose else 0.0
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.068
        m.pose.covariance = cov
        self.init_pub.publish(m)
        self.status_log.append(
            f"initial pose set from console: ({req['x']:.2f}, {req['y']:.2f}, "
            f"{math.degrees(yaw):.0f} deg)")
        return {"ok": True, "yaw": yaw}

    def publish_goal(self, req):
        from geometry_msgs.msg import PoseStamped
        m = PoseStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = float(req["x"])
        m.pose.position.y = float(req["y"])
        yaw = req.get("yaw")
        if yaw is None and self.robot_pose:   # face the goal from the robot
            yaw = math.atan2(req["y"] - self.robot_pose[1],
                             req["x"] - self.robot_pose[0])
        yaw = yaw or 0.0
        m.pose.orientation.z = math.sin(yaw / 2)
        m.pose.orientation.w = math.cos(yaw / 2)
        self.goal_pub.publish(m)
        self.status_log.append(
            f"direct goal from console: ({req['x']:.2f}, {req['y']:.2f}, "
            f"{math.degrees(yaw):.0f} deg)")
        return {"ok": True, "yaw": yaw}

    # ------------------------------------------------------------- state --
    def state(self):
        kg = json.load(open(self.kg_path))
        pl = json.load(open(self.places_path))
        names = {}
        try:
            book = json.load(open(self.naming_path))
            ep = (book.get(getattr(self, "naming_key", None) or "") or
                  book.get("T3_slic_rev4") or book.get("T3_slic") or {})
            names = {k: v["name"] for k, v in ep.items()
                     if isinstance(v, dict)}
        except (OSError, json.JSONDecodeError):
            pass
        objs = []
        for n in kg["nodes"]:
            if n.get("presence") == "absent":
                continue
            dims = n.get("dimensions", {})
            objs.append({"name": n["name"], "type": n.get("type"),
                         "status": n.get("status"),
                         "confidence": n.get("confidence"),
                         "x": round(n["pose"]["x"], 2),
                         "y": round(n["pose"]["y"], 2),
                         "length": round(dims.get("length", 0.5), 2),
                         "width": round(dims.get("width", 0.5), 2),
                         "theta": round(n["pose"].get("theta", 0.0), 3),
                         "isMovable": n.get("implicit", {}).get("isMovable"),
                         "level": n.get("implicit", {}).get("heightLevel"),
                         "labelStability": n.get("implicit", {})
                         .get("labelStability")})
        objs.sort(key=lambda o: (str(o["status"]), o["name"]))
        try:
            sc = json.load(open(f"{BLK}/t3_place_scoped_relations.json"))
            keys = sc.get("keyObjects", {})
        except (OSError, json.JSONDecodeError):
            keys = {}
        # duplicate LLM names (e.g. two clutter_zone regions) get _01/_02
        # suffixes for display; commands always carry the region id
        base = [names.get(p["name"], p["name"])
                for p in pl["semanticPlaces"]]
        from collections import Counter
        cnt, seen = Counter(base), {}
        places = []
        for p, b in zip(pl["semanticPlaces"], base):
            if cnt[b] > 1:
                seen[b] = seen.get(b, 0) + 1
                label = f"{b}_{seen[b]:02d}"
            else:
                label = b
            places.append({"region": p["name"],
                           "name": label,
                           "members": p.get("memberCount"),
                           "verified": p.get("verifiedMemberCount"),
                           "key": keys.get(p["name"]),
                           "cx": round(p["centroid"][0], 2),
                           "cy": round(p["centroid"][1], 2)})
        return {"map": self.map_info, "robot_map_cells": self.robot_map_cells, "objects": objs, "places": places,
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

    def get_entity_json(self, req):
        """Raw DB record for the clicked entity (object / place / robot)."""
        kind, name = req.get("kind"), req.get("name")
        if kind == "obj":
            kg = json.load(open(self.kg_path))
            for n in kg["nodes"]:
                if n["name"] == name:
                    return {"kind": kind, "name": name, "data": n}
            return {"error": f"no object '{name}'"}
        if kind == "robot":
            kg = json.load(open(self.kg_path))
            for r in kg.get("robots", []):
                if r.get("name") == name:
                    return {"kind": kind, "name": name, "data": r}
            return {"error": f"no robot '{name}'"}
        if kind == "place":
            pl = json.load(open(self.places_path))
            for p in pl["semanticPlaces"]:
                if p.get("name") == name:
                    return {"kind": kind, "name": name, "data": p}
            return {"error": f"no place '{name}'"}
        return {"error": f"unknown kind '{kind}'"}

    def set_entity_json(self, req):
        """Owner JSON edit: replace the entity's record in the DB file.
        Objects get a history entry; every consumer (mediator hot-reload,
        Isaac watcher, this UI) picks the change up from the file."""
        import shutil as _sh
        import time as _t
        kind, name, data = req.get("kind"), req.get("name"), req.get("data")
        if not isinstance(data, dict):
            return {"error": "data must be a JSON object"}
        if kind == "obj":
            need = [k for k in ("name", "type", "pose", "dimensions")
                    if k not in data]
            if need:
                return {"error": f"object record missing {need}"}
            kg = json.load(open(self.kg_path))
            for i, n in enumerate(kg["nodes"]):
                if n["name"] == name:
                    hist = list(data.get("history",
                                         n.get("history", [])))
                    hist.append({"revision": kg.get("revision"),
                                 "change": "owner_json_edit",
                                 "time": _t.strftime("%Y-%m-%dT%H:%M:%S")})
                    data["history"] = hist
                    kg["nodes"][i] = data
                    _sh.copy(self.kg_path, self.kg_path + ".uiedit.bak")
                    json.dump(kg, open(self.kg_path, "w"), indent=1,
                              ensure_ascii=False)
                    return {"ok": True}
            return {"error": f"no object '{name}'"}
        if kind == "robot":
            kg = json.load(open(self.kg_path))
            for i, r in enumerate(kg.get("robots", [])):
                if r.get("name") == name:
                    kg["robots"][i] = data
                    _sh.copy(self.kg_path, self.kg_path + ".uiedit.bak")
                    json.dump(kg, open(self.kg_path, "w"), indent=1,
                              ensure_ascii=False)
                    return {"ok": True}
            return {"error": f"no robot '{name}'"}
        if kind == "place":
            if "name" not in data or "centroid" not in data:
                return {"error": "place record missing name/centroid"}
            pl = json.load(open(self.places_path))
            for i, p in enumerate(pl["semanticPlaces"]):
                if p.get("name") == name:
                    pl["semanticPlaces"][i] = data
                    _sh.copy(self.places_path,
                             self.places_path + ".uiedit.bak")
                    json.dump(pl, open(self.places_path, "w"), indent=1,
                              ensure_ascii=False)
                    return {"ok": True}
            return {"error": f"no place '{name}'"}
        return {"error": f"unknown kind '{kind}'"}

    def set_campose(self, req):
        """Orbit the Gazebo overhead camera (right-drag in the UI): move the
        static overhead_cam model with gz set_pose. reset -> top-down."""
        import math
        import subprocess
        if req.get("reset"):
            pos = (-2.0, -1.5, 13.0)
            rpy = (0.0, 1.5708, 1.5708)
        else:
            az = math.radians(float(req.get("az", 90.0)))
            el = math.radians(
                min(88.0, max(15.0, float(req.get("el", 60.0)))))
            tx, ty, tz, r = -2.0, -1.5, 0.6, 12.5
            pos = (tx + r * math.cos(el) * math.cos(az),
                   ty + r * math.cos(el) * math.sin(az),
                   tz + r * math.sin(el))
            rpy = (0.0, el, az + math.pi)
        cr, sr = math.cos(rpy[0] / 2), math.sin(rpy[0] / 2)
        cp, sp = math.cos(rpy[1] / 2), math.sin(rpy[1] / 2)
        cy, sy = math.cos(rpy[2] / 2), math.sin(rpy[2] / 2)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        msg = (f'name: "overhead_cam", position: {{x: {pos[0]:.3f}, '
               f'y: {pos[1]:.3f}, z: {pos[2]:.3f}}}, orientation: '
               f'{{x: {qx:.5f}, y: {qy:.5f}, z: {qz:.5f}, w: {qw:.5f}}}')
        try:
            subprocess.run(
                ["gz", "service", "-s", "/world/visn2_room/set_pose",
                 "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                 "--timeout", "300", "--req", msg],
                capture_output=True, timeout=2.0)
            return {"ok": True}
        except (OSError, subprocess.TimeoutExpired) as e:
            return {"error": str(e)}

    def region_cells(self, region):
        """SLIC segmentation cells (world-frame x,y at 0.05 m pitch) for the
        overhead-view overlay."""
        try:
            pl = json.load(open(self.places_path))
            for p in pl["semanticPlaces"]:
                if p["name"] == region:
                    rgb = [int(v) for v in
                           str(p.get("color", "120,120,120")).split(",")]
                    return {"cells": p.get("cells", []), "cs": 0.05,
                            "color": rgb}
        except (OSError, json.JSONDecodeError, ValueError):
            pass
        return {"cells": [], "cs": 0.05, "color": [120, 120, 120]}

    def publish_command(self, cmd):
        self.cmd_pub.publish(String(data=json.dumps(cmd)))
        if isinstance(cmd, dict) and cmd.get("cmd") in ("stop", "cancel"):
            self.stop_pub.publish(String(data="stop"))

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
            elif self.path == "/api/pipeline/status":
                self._send(200, json.dumps(node.pipeline_status()))
            elif self.path.startswith("/api/region"):
                from urllib.parse import urlparse, parse_qs
                q = parse_qs(urlparse(self.path).query)
                self._send(200, json.dumps(
                    node.region_cells(q.get("name", [""])[0])))
            elif self.path == "/map.png":
                if node.map_png is None:
                    return self._send(404, "{}")
                self._send(200, node.map_png, "image/png")
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

        def do_PUT(self):
            if not self.path.startswith("/api/e57"):
                return self._send(404, "{}")
            from urllib.parse import urlparse, parse_qs, unquote
            q = parse_qs(urlparse(self.path).query)
            name = os.path.basename(unquote(q.get("name", ["scan.e57"])[0]))
            if not name.lower().endswith(".e57"):
                return self._send(400, '{"error":"not an .e57"}')
            n = int(self.headers.get("Content-Length", 0))
            d = os.path.expanduser("~/ammr_twin/scans"); os.makedirs(d, exist_ok=True)
            path = os.path.join(d, name)
            with open(path, "wb") as f:      # stream to disk in 8 MB chunks
                left = n
                while left > 0:
                    chunk = self.rfile.read(min(8 << 20, left))
                    if not chunk:
                        break
                    f.write(chunk); left -= len(chunk)
            self._send(200, json.dumps({"path": path, "bytes": n - left}))

        def _multipart(self, n):
            """Minimal multipart/form-data parser -> (files{name: bytes}, fields{})"""
            ctype = self.headers.get("Content-Type", "")
            boundary = ctype.split("boundary=")[-1].encode()
            body = self.rfile.read(n)
            files, fields = {}, {}
            for part in body.split(b"--" + boundary)[1:]:
                if part.strip() in (b"", b"--"):
                    continue
                head, _, data = part.partition(b"\r\n\r\n")
                data = data[:-2] if data.endswith(b"\r\n") else data
                disp = head.decode(errors="ignore")
                name = disp.split('name="')[1].split('"')[0] if 'name="' in disp else ""
                if 'filename="' in disp:
                    fn = disp.split('filename="')[1].split('"')[0]
                    files[fn] = data
                else:
                    fields[name] = data.decode(errors="ignore")
            return files, fields

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            if self.path == "/api/robotmap":
                files, fields = self._multipart(n)
                return self._send(200, json.dumps(node.register_robot_map(
                    files, fields.get("use_init", "1") == "1")))
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                return self._send(400, '{"error":"bad json"}')
            if self.path == "/api/command":
                node.publish_command(req)
                self._send(200, '{"ok":true}')
            elif self.path == "/api/pipeline/start":
                self._send(200, json.dumps(node.pipeline_start(req)))
            elif self.path == "/api/pipeline/stop":
                self._send(200, json.dumps(node.pipeline_stop()))
            elif self.path == "/api/pipeline/load":
                self._send(200, json.dumps(node.pipeline_load()))
            elif self.path == "/api/initialpose":
                self._send(200, json.dumps(node.publish_initialpose(req)))
            elif self.path == "/api/goal":
                self._send(200, json.dumps(node.publish_goal(req)))
            elif self.path == "/api/battery":
                node.publish_battery(req.get("level", 1.0))
                self._send(200, '{"ok":true}')
            elif self.path == "/api/object":
                self._send(200, json.dumps(node.edit_object(req)))
            elif self.path == "/api/campose":
                self._send(200, json.dumps(node.set_campose(req)))
            elif self.path == "/api/getjson":
                self._send(200, json.dumps(node.get_entity_json(req)))
            elif self.path == "/api/setjson":
                self._send(200, json.dumps(node.set_entity_json(req)))
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
