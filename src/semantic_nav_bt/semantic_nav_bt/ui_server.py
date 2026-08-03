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
  SELS.push({kind:'obj',o:o});drawOverlay();render();}
function pickPlace(region){const i=SELS.findIndex(s=>s.kind==='place'&&s.region===region);
  if(i>=0){SELS.splice(i,1);drawOverlay();render();return;}
  fetch('/api/region?name='+region).then(r=>r.json()).then(c=>{
    SELS.push({kind:'place',region:region,cells:c});drawOverlay();render();});}
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
  drawOverlay();
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
            elif self.path.startswith("/api/region"):
                from urllib.parse import urlparse, parse_qs
                q = parse_qs(urlparse(self.path).query)
                self._send(200, json.dumps(
                    node.region_cells(q.get("name", [""])[0])))
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
