#pragma once
// ============================================================================
//  DashboardPage.h  -  the phone dashboard (served from flash)
// ============================================================================
#include <Arduino.h>

static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>BalanceBot</title>
<style>
:root{--bg:#0f1218;--card:#181d27;--fg:#e6e9ef;--mut:#8a93a6;--acc:#4fd1c5}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0 auto;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif;padding:10px;max-width:640px}
header{display:flex;gap:8px;align-items:center;margin-bottom:10px}
h1{font-size:18px;margin:0;flex:1}
h1 small{font-size:11px;color:var(--mut);font-weight:400}
.pill{padding:3px 10px;border-radius:99px;background:#2a3140;font-size:12px;white-space:nowrap}
.pill.ok{background:#14532d}.pill.bad{background:#7f1d1d}
.s1,.s5{background:#92400e}.s2{background:#14532d}.s3,.s4,.s6{background:#7f1d1d}
.card{background:var(--card);border-radius:14px;padding:12px;margin-bottom:10px}
.card h2{font-size:14px;margin:0 0 8px}
.hint{font-size:12px;color:var(--mut);line-height:1.45;margin:0 0 10px}
.row{display:flex;gap:8px;margin-bottom:8px}.row:last-child{margin-bottom:0}
button{flex:1;padding:12px 6px;border:0;border-radius:10px;background:#2a3140;color:var(--fg);font-size:14px;font-weight:600;touch-action:manipulation}
button:active{filter:brightness(1.4)}
.go{background:#15803d}.stop{background:#b91c1c}.big{padding:18px 6px;font-size:18px}
.tog.on{background:var(--acc);color:#05201e}
#padwrap{display:flex;justify-content:center;margin:6px 0 12px}
#pad{position:relative;width:min(76vw,290px);aspect-ratio:1;border-radius:50%;background:radial-gradient(#242b3a,#151a23);border:2px solid #2f3748;touch-action:none;user-select:none}
#knob{position:absolute;left:32%;top:32%;width:36%;height:36%;border-radius:50%;background:var(--acc);opacity:.92}
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;text-align:center;margin-bottom:8px}
.grid.g3{grid-template-columns:repeat(3,1fr)}
.grid b{display:block;font-size:17px}.grid span{font-size:11px;color:var(--mut)}
canvas{width:100%;height:140px;background:#10141c;border-radius:8px;display:block}
details{border-top:1px solid #262d3c;padding:8px 0}details:first-child{border-top:0}
summary{font-weight:600;cursor:pointer;padding:4px 0}
.prm{display:grid;grid-template-columns:1fr 96px;gap:4px 8px;align-items:center;margin:10px 0}
.prm label{font-size:13px;color:var(--mut)}
.prm input[type=range]{grid-column:1/-1;width:100%}
.prm input[type=number]{width:100%;background:#10141c;color:var(--fg);border:1px solid #2f3748;border-radius:6px;padding:6px;font-size:14px}
#applied{font-size:12px;color:var(--acc);margin-top:6px;min-height:16px}
.eff{color:var(--acc);font-size:12px;font-weight:600}
select{width:100%;background:#10141c;color:var(--fg);border:1px solid #2f3748;border-radius:6px;padding:8px;font-size:14px}
.warn{color:#fbbf24}
#toast{position:fixed;left:50%;bottom:18px;transform:translateX(-50%);background:#2a3140;padding:10px 16px;border-radius:10px;opacity:0;transition:opacity .25s;pointer-events:none;max-width:90vw;text-align:center;z-index:9}
#toast.show{opacity:1}
</style></head><body>
<header><h1>BalanceBot <small>fw 2.5</small></h1><span id="conn" class="pill bad">offline</span><span id="state" class="pill">-</span></header>

<div class="card">
  <div class="row"><button id="bStart" class="big go">START</button><button id="bStop" class="big stop">STOP</button></div>
  <div class="row"><button id="bHold" class="tog">Hold position: OFF</button><button id="bMark">Set home</button><button id="bRet">Return home</button></div>
</div>

<div class="card">
  <h2>IMU setup &amp; bench check <span style="font-weight:400;color:var(--mut)">(do this once, robot STOPPED)</span></h2>
  <p class="hint">
    <b>1.</b> Hold the robot upright and still, press <i>Capture upright</i>.<br>
    <b>2.</b> Tilt it straight FORWARD about 25&deg; (the way it should drive), hold still, press <i>Capture forward</i>.<br>
    <b>3.</b> Press <i>Tilt test</i> (wheels in the air): tilt forward &rarr; wheels must roll forward, tilt back &rarr; backward, level &rarr; stop. Press STOP.<br>
    <b>4.</b> Hold the robot exactly at its balance point (where it neither falls forward nor back) and press <i>Set balance point</i>.
  </p>
  <div class="row"><button id="bCalUp">1. Capture upright</button><button id="bCalFwd">2. Capture forward</button></div>
  <div class="row"><button id="bTilt">3. Tilt test</button><button id="bSetBal">4. Set balance point</button></div>
  <div class="grid g3" style="margin-top:10px">
    <div><b id="i_ax">0</b><span>accel X (g)</span></div>
    <div><b id="i_ay">0</b><span>accel Y (g)</span></div>
    <div><b id="i_az">0</b><span>accel Z (g)</span></div>
    <div><b id="i_gx">0</b><span>gyro X (&deg;/s)</span></div>
    <div><b id="i_gy">0</b><span>gyro Y (&deg;/s)</span></div>
    <div><b id="i_gz">0</b><span>gyro Z (&deg;/s)</span></div>
  </div>
  <div class="hint" style="margin:0">Mounting: <b id="i_ori">-</b> &nbsp;|&nbsp; wheel command: <b id="t_ws">0</b> mm/s &nbsp;|&nbsp; accel command: <b id="t_ac">0</b> mm/s&sup2;</div>
</div>

<div class="card">
  <div id="padwrap"><div id="pad"><div id="knob"></div></div></div>
  <div id="driving"></div>
</div>

<div class="card">
  <div class="grid">
    <div><b id="t_a">0</b><span>angle &deg;</span></div>
    <div><b id="t_r">0</b><span>gyro &deg;/s</span></div>
    <div><b id="t_v">0</b><span>speed mm/s</span></div>
    <div><b id="t_tl">0</b><span>target lean &deg;</span></div>
    <div><b id="t_x">0</b><span>x mm</span></div>
    <div><b id="t_y">0</b><span>y mm</span></div>
    <div><b id="t_h">0</b><span>heading &deg;</span></div>
    <div><b id="t_n">-</b><span>mode</span></div>
  </div>
  <canvas id="chart" width="600" height="140"></canvas>
  <div class="hint" style="margin:6px 0 0">teal = angle (&plusmn;20&deg;) &nbsp; orange = wheel speed (&plusmn; top speed)</div>
  <div class="hint" id="health" style="margin:4px 0 0">-</div>
  <div class="hint warn" id="fallbox" style="margin:4px 0 0"></div>
</div>

<div class="card">
  <h2>Auto-tune</h2>
  <p class="hint">The robot can measure itself and set the balance, speed and position gains for you. <b>How:</b> press START and let it balance, then press <i>Auto-tune now</i> and keep hands off and the stick centred for about 15 s. It nudges the wheels with small random pushes, works out how tall the robot is (centre of mass above the axle), and sets the gains. Afterwards press STOP, then <i>Save to flash</i> to keep them. Manual tuning below still works as before; <i>Undo</i> brings back the values from before.</p>
  <div class="hint" id="tunest" style="margin:4px 0">Auto-tune: idle</div>
  <div class="row"><button id="bTune">Auto-tune now</button><button id="bUntune">Undo auto-tune</button></div>
  <div id="tunegroups"></div>
  <div class="row"><button id="bHeight">Set gains from Robot height</button></div>
  <p class="hint">If you know the height you can type it above and press <i>Set gains from Robot height</i> instead (balance the robot on your finger to find where its centre of mass is, and measure up from the axle). Taller robot = larger Kd; the master sliders then fine-tune.</p>
  <p class="hint"><a href="/log" target="_blank" style="color:#7dd3c0">Open flight log</a> - the last 3 s before the robot stopped or fell (angle, gyro, wheel speed...). Copy it and send it to get a diagnosis.</p>
</div>

<div class="card">
  <h2>Tuning</h2>
  <p class="hint"><b>Master PID</b> is for coarse tuning of the whole robot: <b>Master Kp</b> multiplies every P gain (Balance Kp, Speed Kp, Position Kp), <b>Master Kd</b> every D gain (Balance Kd, Speed Kd) and <b>Master Ki</b> every I gain (Speed Ki). The green number after each gain below is what the robot really uses (its own value &times; master). Tune with the master first; then fine-tune a single loop with its own sliders.</p>
  <div id="groups"></div><div id="applied"></div>
</div>

<div class="card">
  <h2>Motors &amp; hardware</h2>
  <p class="hint">These are stored in flash by <i>Save to flash</i>. <b>Microstepping</b> only tells the robot what your CNC-shield caps are (change both together): 1/1 = no caps, 1/2 = MS1, 1/4 = MS2, 1/8 = MS1+MS2, 1/16 = MS3, 1/32 = MS1+MS3. It is applied the next time you press START.</p>
  <div class="prm"><label>Microstepping</label>
    <select id="s_microsteps"><option value="1">1/1 (full step)</option><option value="2">1/2</option><option value="4">1/4</option><option value="8">1/8 (default)</option><option value="16">1/16</option><option value="32">1/32</option></select></div>
  <div class="row"><button id="bInvL" class="tog">Reverse left motor</button><button id="bInvR" class="tog">Reverse right motor</button></div>
  <div class="row"><button id="bInvD" class="tog">Reverse drive</button><button id="bInvS" class="tog">Reverse steering</button></div>
  <div id="hwgroups"></div>
  <div class="hint">Top wheel speed possible at this microstepping: <b id="t_lim">-</b> mm/s. The Max motor speed above limits the wheels in <i>every</i> mode (balancing, joystick, return home), so lower it if the wheels spin too fast.</div>
  <p class="hint">While balancing, the robot slowly learns its true balance point (<i>auto-trim</i>, shown in the status line under the chart) so it stops creeping. <i>Keep auto-trim</i> stores what it learned as the balance trim; then press Save to flash.</p>
  <div class="row"><button id="bZero">Zero IMU (hold upright, still)</button><button id="bKeep">Keep auto-trim</button></div>
  <div class="row"><button id="bCal">Calibrate gyro</button><button id="bTest">Motor test</button></div>
  <div class="row"><button id="bSave">Save to flash</button><button id="bReset">Reset tuning</button></div>
</div>
<div id="toast"></div>

<script>
const $=id=>document.getElementById(id);
const GROUPS=[
 {t:'Driving',box:'driving',open:true,p:[
  ['maxSpeed','Joystick max speed (mm/s)',50,1000,10],
  ['maxTurn','Max turn rate (deg/s)',10,360,5],
  ['rampUp','Ramp up - accel (mm/s2)',50,3000,50],
  ['rampDown','Ramp down - decel (mm/s2)',50,4000,50],
  ['turnRamp','Turn ramp (deg/s2)',50,3000,50],
  ['returnSpeed','Return-home speed (mm/s)',50,800,10]]},
 {t:'MASTER PID (start here)',open:true,p:[['mKp','Master Kp (x)',0.2,3,0.01],['mKi','Master Ki (x)',0,5,0.01],['mKd','Master Kd (x)',0.2,3,0.01]]},
 {t:'Balance PD (inner loop)',p:[['balKp','Kp',0,3000,1],['balKd','Kd',0,300,0.1]]},
 {t:'Speed PID (outer loop)',p:[['spdKp','Kp',0,0.05,0.0001],['spdKi','Ki',0,0.05,0.00005],['spdKd','Kd',0,0.02,0.0001]]},
 {t:'Position & heading hold',p:[['posKp','Position Kp',0,3,0.01],['hdgKp','Heading Kp (independent of master)',0,10,0.1]]},
 {t:'Robot & filter',p:[
  ['angleOffset','Balance trim (deg)',-15,15,0.05],
  ['alpha','Filter alpha',0.9,0.9999,0.0005],
  ['maxTilt','Max lean (deg)',1,20,0.5],
  ['fallAngle','Fall cut-off (deg)',15,60,1],
  ['maxAccel','Wheel accel limit (mm/s2)',500,10000,100],
  ['wheelDia','Wheel diameter (mm)',20,200,0.5],
  ['wheelBase','Wheel base (mm)',50,400,1]]},
 {t:'Auto trim & gyro yaw',p:[['trimRate','Auto balance-trim speed (0 = off)',0,0.02,0.0005],['yawAssist','Gyro yaw assist for steering/heading (1 = on, 0 = off)',0,1,1]]},
 {t:'Robot height & auto-tune',box:'tunegroups',open:true,p:[['comH','Robot height: centre of mass above axle (mm, 0 = not known)',0,600,5],['tuneOnStart','Auto-tune at every START (1 = on, 0 = off)',0,1,1]]},
 {t:'Max motor speed (all modes)',box:'hwgroups',open:true,p:[['maxWheel','Max motor speed (mm/s)',100,1200,10]]}
];
const EK=['balKp','balKd','spdKp','spdKi','spdKd','posKp'];   // order of the "e" array from the robot
const HW=[['invLeft','bInvL','Reverse left motor'],['invRight','bInvR','Reverse right motor'],['invDrive','bInvD','Reverse drive'],['invSteer','bInvS','Reverse steering']];
const STATES=['STOPPED','ARMED','BALANCING','FELL OVER','IMU FAULT','TILT TEST','RUNAWAY'];
const NAV=['-','turning','driving','final turn'];
let lastTn=-1,params={},hist=[],hist2=[],lim=600,tt,lastSeq=-1,paramsLoaded=false,wake=null;

function toast(m){const t=$('toast');t.textContent=m;t.classList.add('show');clearTimeout(tt);tt=setTimeout(()=>t.classList.remove('show'),3500);}
// every action is a plain HTTP GET (no extra libraries on the ESP32)
function send(s){
  const isSet=s.startsWith('set:');
  let url,key='';
  if(isSet){const p=s.split(':');key=p[1];url='/set?k='+p[1]+'&v='+p[2];}
  else url='/cmd?c='+s;
  return fetch(url,{cache:'no-store'}).then(r=>r.json()).then(d=>{
    if(isSet){
      if(d.ok){params[key]=d.v;$('applied').textContent='Robot applied: '+key+' = '+(+d.v.toPrecision(5));updEff(d.e);updHw();}
      else toast('Robot rejected '+key);
    }
    return d;
  }).catch(()=>{toast('No answer from the robot - check the WiFi');});
}
function refreshParams(){return fetch('/params',{cache:'no-store'}).then(r=>r.json()).then(d=>{loadParams(d.p);updEff(d.e);setLim(d.lim);}).catch(()=>{});}
function updEff(e){if(!e)return;EK.forEach((k,i)=>{const el=$('e_'+k);if(el)el.textContent='\u2192 '+(+e[i].toPrecision(4));});}
function setLim(v){if(v!==undefined)$('t_lim').textContent=v;}
function updHw(){
  HW.forEach(([k,b,t])=>{const on=params[k]>0.5;$(b).textContent=t+': '+(on?'ON':'OFF');$(b).className='tog'+(on?' on':'');});
  if(params.microsteps)$('s_microsteps').value=params.microsteps;
}
const sleep=ms=>new Promise(r=>{wake=r;setTimeout(r,ms);});
function setOnline(on){$('conn').textContent=on?'online':'offline';$('conn').className='pill '+(on?'ok':'bad');}
// one request both sends the joystick and returns telemetry
async function loop(){
  for(;;){
    try{
      if(!paramsLoaded){const r=await fetch('/params',{cache:'no-store'});const d=await r.json();loadParams(d.p);updEff(d.e);setLim(d.lim);paramsLoaded=true;}
      const r=await fetch('/joy?x='+jx.toFixed(2)+'&y='+jy.toFixed(2),{cache:'no-store'});
      const d=await r.json();
      setOnline(true);onTel(d);
      if(d.mseq!==lastSeq){if(lastSeq!==-1&&d.msg)toast(d.msg);lastSeq=d.mseq;}
    }catch(e){setOnline(false);await sleep(500);}
    await sleep(active?30:150);
  }
}

function build(){
  GROUPS.forEach(g=>{
    const det=document.createElement('details');
    if(g.open)det.open=true;
    det.innerHTML='<summary>'+g.t+'</summary>';
    g.p.forEach(([k,label,min,max,step])=>{
      const row=document.createElement('div');row.className='prm';
      row.innerHTML='<label>'+label+' <span class="eff" id="e_'+k+'"></span></label><input type="number" step="any" id="n_'+k+'"><input type="range" id="r_'+k+'" min="'+min+'" max="'+max+'" step="'+step+'">';
      det.appendChild(row);
    });
    $(g.box||'groups').appendChild(det);
    g.p.forEach(([k,,min,max])=>{
      const n=$('n_'+k),r=$('r_'+k);let last=0;
      r.oninput=()=>{n.value=r.value;const t=Date.now();if(t-last>60){last=t;send('set:'+k+':'+r.value);}};
      r.onchange=()=>send('set:'+k+':'+r.value);
      n.onchange=()=>{let v=Math.min(max,Math.max(min,parseFloat(n.value)||0));n.value=v;r.value=v;send('set:'+k+':'+v);};
    });
  });
}
function loadParams(p){
  for(const k in p){params[k]=p[k];const n=$('n_'+k),r=$('r_'+k);if(n){n.value=p[k];r.value=p[k];}}
  updHold();updHw();
}
function updHold(){
  const on=params.holdPos>0.5;
  $('bHold').textContent='Hold position: '+(on?'ON':'OFF');
  $('bHold').className='tog'+(on?' on':'');
}

function onTel(d){
  $('t_a').textContent=d.a.toFixed(1);$('t_r').textContent=d.r.toFixed(0);
  $('t_v').textContent=d.v;$('t_tl').textContent=d.tl.toFixed(1);
  $('t_x').textContent=d.x;$('t_y').textContent=d.y;$('t_h').textContent=d.h.toFixed(0);
  $('t_n').textContent=d.nav?NAV[d.nav]:(d.hold?'holding':'-');
  $('t_ws').textContent=d.ws;$('t_ac').textContent=d.ac;
  $('i_ax').textContent=d.ax.toFixed(2);$('i_ay').textContent=d.ay.toFixed(2);$('i_az').textContent=d.az.toFixed(2);
  $('i_gx').textContent=d.gx.toFixed(0);$('i_gy').textContent=d.gy.toFixed(0);$('i_gz').textContent=d.gz.toFixed(0);
  $('i_ori').textContent=d.ori||'-';
  const s=$('state');s.textContent=STATES[d.st]||'?';s.className='pill s'+d.st;
  const rs=d.rs>=5?'-':d.rs.toFixed(2);
  $('health').textContent='Control loop: longest '+d.dt+' ms, late loops '+d.late+'  |  top wheel speed '+d.lim+' mm/s  |  auto-trim '+d.at.toFixed(2)+'\u00b0  |  yaw gyro '+(d.yg?'ok':'unchecked')+'  |  wobble '+(d.wh||0).toFixed(1)+' Hz / '+(d.wd||0).toFixed(1)+'\u00b0  |  wheel response '+rs+(d.rs<5?(d.rs<-0.25?'  (motors REVERSED?)':(d.rs<0.35?'  (wheels not following - skipping steps?)':'  (ok)')):'');
  if(lastTn>=0&&lastTn<100&&(d.tn<0||d.tn>=100))setTimeout(refreshParams,300);   // auto-tune just ended: show the new gains
  lastTn=d.tn;
  $('tunest').textContent=(d.tn>=0&&d.tn<100)?('Auto-tune running: '+d.tn+'%  (keep hands off, stick centred)'):'Auto-tune: idle';
  $('fallbox').textContent=d.fall?('Last stop: '+d.fall):'';
  setLim(d.lim);lim=d.lim||600;
  hist.push(d.a);if(hist.length>200)hist.shift();
  hist2.push(d.ws);if(hist2.length>200)hist2.shift();draw();
}
function draw(){
  const c=$('chart'),x=c.getContext('2d'),w=c.width,h=c.height;
  x.clearRect(0,0,w,h);x.strokeStyle='#2f3748';x.beginPath();x.moveTo(0,h/2);x.lineTo(w,h/2);x.stroke();
  x.strokeStyle='#4fd1c5';x.lineWidth=2;x.beginPath();
  hist.forEach((v,i)=>{const px=i*w/200,py=h/2-Math.max(-20,Math.min(20,v))*h/40;i?x.lineTo(px,py):x.moveTo(px,py);});
  x.stroke();
  x.strokeStyle='#f6ad55';x.lineWidth=1.5;x.beginPath();
  hist2.forEach((v,i)=>{const px=i*w/200,py=h/2-Math.max(-1,Math.min(1,v/lim))*h/2;i?x.lineTo(px,py):x.moveTo(px,py);});
  x.stroke();
}

// ---- joystick
const pad=$('pad'),knob=$('knob');let jx=0,jy=0,active=false;
function stick(e){
  const r=pad.getBoundingClientRect(),R=r.width/2;
  let dx=(e.clientX-r.left-R)/R,dy=(e.clientY-r.top-R)/R;
  const m=Math.hypot(dx,dy);if(m>1){dx/=m;dy/=m;}
  jx=dx;jy=-dy;
  knob.style.transform='translate('+(dx*R*0.64)+'px,'+(dy*R*0.64)+'px)';
}
pad.addEventListener('pointerdown',e=>{active=true;if(wake)wake();pad.setPointerCapture(e.pointerId);stick(e);});
pad.addEventListener('pointermove',e=>{if(active)stick(e);});
function release(){active=false;jx=jy=0;knob.style.transform='translate(0,0)';if(wake)wake();}
pad.addEventListener('pointerup',release);
pad.addEventListener('pointercancel',release);
document.addEventListener('visibilitychange',()=>{if(document.hidden)release();});

// ---- buttons
$('bStart').onclick=()=>send('start');
$('bStop').onclick=()=>send('stop');
$('bMark').onclick=()=>send('mark');
$('bRet').onclick=()=>send('return');
$('bCal').onclick=()=>send('calib');
$('bTest').onclick=()=>send('test');
$('bSave').onclick=()=>send('save');
$('bCalUp').onclick=()=>send('calup');
$('bCalFwd').onclick=()=>send('calfwd').then(()=>setTimeout(refreshParams,1200));
$('bTilt').onclick=()=>send('tilt');
$('bTune').onclick=()=>send('tune');
$('bUntune').onclick=()=>send('untune').then(()=>setTimeout(refreshParams,500));
$('bHeight').onclick=()=>send('height').then(()=>setTimeout(refreshParams,500));
$('bKeep').onclick=()=>send('keeptrim').then(()=>setTimeout(refreshParams,500));
$('bZero').onclick=()=>send('zero').then(()=>setTimeout(refreshParams,2500));
HW.forEach(([k,b])=>{$(b).onclick=()=>{const v=params[k]>0.5?0:1;params[k]=v;updHw();send('set:'+k+':'+v);};});
$('s_microsteps').onchange=()=>{const v=$('s_microsteps').value;send('set:microsteps:'+v).then(()=>{toast('Now put the same caps on the CNC shield, then press START again');setTimeout(refreshParams,300);});};
$('bSetBal').onclick=()=>send('setbal').then(()=>setTimeout(refreshParams,1200));
$('bReset').onclick=()=>{if(confirm('Reset tuning values to defaults? (IMU setup and balance point are kept)'))send('reset').then(()=>setTimeout(refreshParams,300));};
$('bHold').onclick=()=>{const v=params.holdPos>0.5?0:1;params.holdPos=v;send('set:holdPos:'+v);updHold();};

build();loop();
</script></body></html>)rawliteral";
