#pragma once

constexpr char productionPageJapanese[] = R"HTML(
<!doctype html>
<html lang="ja">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>水上ボート 統合運転</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#f4f5f7;color:#18202a;font:16px system-ui,sans-serif}
main{max-width:580px;margin:auto;padding:18px}h1{font-size:23px;margin:2px 0 14px}
.status,.panel{border:1px solid #d7dce2;border-radius:10px;background:#fff;padding:14px}
.status{margin-bottom:12px;background:#fff4ce}.status.ok{background:#dcf3e3}.status.bad{background:#fde0df}
.group{border-top:1px solid #e2e5e9;margin-top:15px;padding-top:13px}.group:first-child{border-top:0;margin-top:0;padding-top:0}
.group-title{font-weight:800;margin-bottom:9px}.switches{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.switch{display:flex;align-items:center;gap:9px;border:1px solid #d7dce2;border-radius:8px;padding:10px;font-weight:700}
select{width:100%;min-height:44px;border:1px solid #c9cfd6;border-radius:8px;background:#fff;padding:8px;font:inherit}
label.output{display:block;font-weight:700;margin:13px 0 7px}.range{display:grid;grid-template-columns:1fr 62px;gap:10px;align-items:center}
input[type=range]{width:100%}.number{text-align:right;font-weight:700;font-variant-numeric:tabular-nums}
.buttons{display:grid;grid-template-columns:1fr 1fr;gap:9px;margin-top:20px}
button{min-height:48px;border:0;border-radius:8px;font:700 16px system-ui}button:disabled{opacity:.42}
.start{background:#24863d;color:#fff}.stop{background:#d9dee4}.estop{grid-column:1/-1;background:#c92532;color:#fff}.clear{background:#efb521;color:#18202a}
.message{margin-top:14px;min-height:46px}.detail,.hint{margin-top:10px;color:#59636e;font-size:14px;line-height:1.55}.hint{background:#f2f5f8;border-radius:8px;padding:9px}
</style>
<main>
  <h1>統合運転</h1>
  <div id="connection" class="status">XIAOとGNSSを確認しています…</div>
  <section class="panel">
    <div class="group">
      <div class="group-title">制御選択（停止中のみ変更可能）</div>
      <div class="switches">
        <label class="switch"><input id="waypoint-enable" type="checkbox">ウェイポイント</label>
        <label class="switch"><input id="attitude-enable" type="checkbox">姿勢制御</label>
      </div>
      <label class="output" for="target">固定ウェイポイント</label>
      <select id="target">
        <option value="0">A — 35.45327, 136.07198</option>
        <option value="1">B — 35.44437, 136.07399</option>
        <option value="2">C — 35.43214, 136.07628</option>
        <option value="3">D — 35.42587, 136.09377</option>
        <option value="4">E — 35.42542, 136.12038</option>
        <option value="5">F — 35.43055, 136.14585</option>
        <option value="6">G — 35.44150, 136.12081</option>
        <option value="7">H — 35.44196, 136.09429</option>
      </select>
      <div id="mode-hint" class="hint">全出力を手動操作します。</div>
    </div>

    <div class="group" id="manual-section">
      <div class="group-title">手動入力</div>
      <label class="output" for="left">左前翼 CH0</label><div class="range"><input id="left" type="range" min="-1" max="1" step="0.01" value="0"><span id="left-number" class="number">0.00</span></div>
      <label class="output" for="right">右前翼 CH1</label><div class="range"><input id="right" type="range" min="-1" max="1" step="0.01" value="0"><span id="right-number" class="number">0.00</span></div>
      <label class="output" for="rear">後部ヨー CH2</label><div class="range"><input id="rear" type="range" min="-1" max="1" step="0.01" value="0"><span id="rear-number" class="number">0.00</span></div>
      <label class="output" for="propulsion">推進</label><div class="range"><input id="propulsion" type="range" min="0" max="1" step="0.01" value="0"><span id="propulsion-number" class="number">0.00</span></div>
    </div>

    <div class="buttons">
      <button id="start" class="start">開始</button>
      <button id="stop" class="stop">停止</button>
      <button id="estop" class="estop">緊急停止</button>
    </div>
    <div id="message" class="message">制御方法を選んで開始してください。</div>
    <div id="detail" class="detail">全出力OFF</div>
  </section>
</main>
<script>
const get=id=>document.getElementById(id);
let latest=null;let requestsInFlight=0;let selectionInitialized=false;
const outputIds=['left','right','rear','propulsion'];
function manualQuery(){return outputIds.map(id=>id+'='+encodeURIComponent(get(id).value)).join('&')}
function selectionQuery(){return 'waypoint='+(get('waypoint-enable').checked?1:0)+'&attitude='+(get('attitude-enable').checked?1:0)+'&target='+encodeURIComponent(get('target').value)}
function modeHint(){
  const wp=get('waypoint-enable').checked,att=get('attitude-enable').checked;
  get('mode-hint').textContent=wp?(att?'固定点へLOS航行し、左右前翼で姿勢・高さも制御します。':'固定点へLOS航行します。前翼は中立で、姿勢補正は行いません。'):(att?'左右前翼を姿勢・高さ制御し、後部ヨーと推進は手動です。':'左右前翼・後部ヨー・推進をすべて手動操作します。');
  updateControlAvailability();
}
function updateControlAvailability(){
  const locked=latest&&!['idle','error'].includes(latest.operation),transition=locked&&latest.operation!=='running',wp=get('waypoint-enable').checked,att=get('attitude-enable').checked;
  get('waypoint-enable').disabled=locked;get('attitude-enable').disabled=locked;get('target').disabled=locked||!wp;
  get('left').disabled=transition||wp||att;get('right').disabled=transition||wp||att;get('rear').disabled=transition||wp;get('propulsion').disabled=transition||wp;
}
async function post(path){
  requestsInFlight++;
  try{const response=await fetch(path,{method:'POST'});const result=await response.json();get('message').textContent=result.message||'指令を送りました。'}
  catch(error){get('message').textContent='通信側XIAOへ指令を送れませんでした。'}
  requestsInFlight--;renderButtons();
}
function renderButtons(){
  if(!latest){get('start').disabled=true;return}
  const idle=['idle','error'].includes(latest.operation),wp=get('waypoint-enable').checked,att=get('attitude-enable').checked;
  const imuNeeded=wp||att,gnssReady=latest.gnss.communication_valid&&latest.gnss.control_valid;
  get('start').disabled=requestsInFlight>0||!idle||latest.control.safety===4||!latest.connected||!latest.actuators.pca_ready||(imuNeeded&&!latest.sensors.imu_valid)||(wp&&!gnssReady);
  get('stop').disabled=false;
}
function render(state){
  latest=state;const connection=get('connection');
  const locked=!['idle','error'].includes(state.operation);
  if(!selectionInitialized||locked){get('waypoint-enable').checked=Boolean(state.selection.waypoint);get('attitude-enable').checked=Boolean(state.selection.attitude);get('target').value=String(state.selection.target_index);selectionInitialized=true;modeHint()}
  if(!state.connected){connection.className='status bad';connection.textContent=state.ever_received?'XIAOとの通信が切れています':'XIAOを待っています'}
  else if(!state.actuators.pca_ready){connection.className='status bad';connection.textContent='XIAO接続済み・PCA9685未接続'}
  else{const wpSelected=get('waypoint-enable').checked,fix=state.gnss.communication_valid&&state.gnss.control_valid;connection.className=!wpSelected||fix?'status ok':'status';connection.textContent='制御側XIAO '+state.control.safety_name+' / '+(wpSelected?('GNSS '+(fix?'FIX':'WAIT')+' / 衛星 '+state.gnss.satellites):('GNSS不要 / 衛星 '+state.gnss.satellites))+' / SD '+(state.storage.sd_ready?'OK':'未認識')}
  get('message').textContent=state.message;const running=state.operation==='running';
  get('detail').textContent=running?state.selection.mode_name+' / PWM '+state.actuators.left_us+'・'+state.actuators.right_us+'・'+state.actuators.rear_us+' µs / Duty '+Number(state.actuators.applied_duty).toFixed(3)+' / 目標距離 '+Number(state.control.waypoint_distance_m).toFixed(1)+' m':'全出力OFF / 停止理由 '+state.control.stop_reason_name+' / PCAエラー '+state.actuators.pwm_errors;
  const emergency=state.control.safety===4;get('estop').textContent=emergency?'緊急停止を解除':'緊急停止';get('estop').classList.toggle('clear',emergency);
  updateControlAvailability();renderButtons();
}
async function poll(){try{const response=await fetch('/api/status',{cache:'no-store'});if(!response.ok)throw new Error();render(await response.json())}catch(error){latest=null;get('connection').className='status bad';get('connection').textContent='通信側XIAOから状態を取得できません';get('start').disabled=true}}
outputIds.forEach(id=>get(id).addEventListener('input',()=>{get(id+'-number').textContent=Number(get(id).value).toFixed(2)}));
outputIds.forEach(id=>get(id).addEventListener('change',()=>{if(latest&&latest.operation==='running'&&!latest.selection.waypoint)post('/api/value?'+manualQuery())}));
get('waypoint-enable').addEventListener('change',modeHint);get('attitude-enable').addEventListener('change',modeHint);
get('start').addEventListener('click',()=>post('/api/start?'+manualQuery()+'&'+selectionQuery()));
get('stop').addEventListener('click',()=>post('/api/stop'));
get('estop').addEventListener('click',()=>post(latest&&latest.control.safety===4?'/api/clear-estop':'/api/estop'));
setInterval(poll,300);modeHint();poll();
</script>
</html>
)HTML";
