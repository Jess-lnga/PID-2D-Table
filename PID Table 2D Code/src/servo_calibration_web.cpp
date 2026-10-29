#include "servo_calibration_web.h"
#include "servo_calibration.h"
#include "servo_control.h"
#include "runtime_lock.h"
#include <errno.h>
#include <stdlib.h>

namespace ServoCalibrationWeb
{
namespace
{
WebServer* web;
bool unsignedArg(const char* name, uint32_t& value)
{
  if (!web->hasArg(name)) return false;
  String text = web->arg(name);
  if (!text.length() || text[0] < '0' || text[0] > '9') return false;
  char* end;
  errno = 0;
  unsigned long parsed = strtoul(text.c_str(), &end, 10);
  if (errno || *end || parsed > UINT32_MAX) return false;
  value = parsed; return true;
}
bool integerArg(const char* name, int& value)
{
  if (!web->hasArg(name)) return false;
  String text = web->arg(name);
  if (!text.length()) return false;
  char* end;
  errno = 0;
  long parsed = strtol(text.c_str(), &end, 10);
  if (errno || end == text.c_str() || *end || parsed < -100 || parsed > 100) return false;
  value = parsed; return true;
}
void response(bool ok)
{
  web->sendHeader("Cache-Control", "no-store");
  web->send(ok ? 200 : 409, "application/json", stateJson());
}
void command(const char* action)
{
  RuntimeLock::Guard guard;
  uint32_t session, revision;
  if (!unsignedArg("session", session) || !unsignedArg("revision", revision)
      || !ServoCalibration::checkCommand(session, revision)) { guard.release(); response(false); return; }
  int value = 0;
  bool ok = false;
  if (!strcmp(action, "select")) ok = integerArg("servo", value) && ServoCalibration::chooseServo(value);
  if (!strcmp(action, "assign")) { int joint; ok = integerArg("servo", value) && integerArg("joint", joint) && ServoCalibration::assignJoint(value, joint); }
  if (!strcmp(action, "adjust")) ok = integerArg("delta", value) && ServoCalibration::adjust(value);
  if (!strcmp(action, "done")) ok = ServoCalibration::done();
  if (!strcmp(action, "test")) ok = ServoCalibration::testAgain();
  if (!strcmp(action, "save")) ok = integerArg("joint", value) && ServoCalibration::save(value);
  if (!strcmp(action, "back")) ok = ServoCalibration::backToSelection();
  if (!strcmp(action, "exit")) ok = ServoCalibration::closeMenu();
  guard.release();
  response(ok);
}
const char HTML[] PROGMEM = R"calhtml(
<style>
#servoCalMenu [hidden] { display:none !important; }
.cal-grid { display:flex; flex-wrap:wrap; justify-content:center; gap:10px; }
.cal-card { border:1px solid #ccd5df; border-radius:8px; padding:12px; flex:1; min-width:140px; }
.cal-table { border-collapse:collapse; width:100%; margin:15px 0; }
.cal-table td,.cal-table th { padding:8px; border:1px solid #ddd; }
#calMessage { color:#b44322; min-height:24px; }
#calTargetArm { transform-origin:130px 130px; transition:transform 120ms linear; }
#servoCalMenu button { margin:5px; }
</style>
<div id="servoCalMenu" hidden class="servo-box">
<h2>Calibrate servo</h2>
<p>Le servo sélectionné est commandé physiquement. Les autres gardent leur position ; la démo est suspendue.</p>
<div id="calMessage" role="status"></div>
<div id="calChoose">
<h3>Choisir le servomoteur</h3>
<div class="cal-grid" id="calServoCards"></div>
</div>
<div id="calWork" hidden>
<h3 id="calTitle"></h3>
<svg viewBox="0 0 350 220" style="width:100%;max-width:400px" aria-label="Angle cible du palonnier">
<line x1="35" y1="130" x2="335" y2="130" stroke="#aaa" stroke-dasharray="5 5"/>
<rect x="80" y="130" width="70" height="50" fill="#526b83" rx="5"/>
<text x="183" y="151" font-size="13">0° : radial extérieur →</text>
<g id="calTargetArm"><line x1="130" y1="130" x2="235" y2="130" stroke="#1977b3" stroke-width="12" stroke-linecap="round"/><circle cx="235" cy="130" r="9" fill="#e67d28"/></g>
<circle cx="130" cy="130" r="9" fill="#222"/>
<text id="calAngleText" x="185" y="30" font-size="19"></text>
<text x="25" y="205" font-size="12">Angle positif : l'extrémité du palonnier monte.</text>
</svg>
<p class="imu-note">Dessin de consigne, sans mesure de l'angle réel du servo.</p>
<p id="calInstruction"></p>
<p><strong id="calTicks"></strong></p>
<div id="calAdjust">
<label>Pas : <select id="calStep"><option value="1">1 tick</option><option value="10" selected>10 ticks</option><option value="50">50 ticks</option></select></label>
<button data-cal-command onclick="calAdjust(-1)">−</button><button data-cal-command onclick="calAdjust(1)">+</button>
<button data-cal-command onclick="calCommand('done')">Done</button>
</div>
<table class="cal-table"><thead><tr><th>Angle</th><th>PWM enregistré (ticks)</th></tr></thead><tbody id="calPoints"></tbody></table>
<div id="calValidate" hidden>
<label>Identifiant cinématique : <select id="calJoint"></select></label>
<p>La vérification physique est-elle correcte ?</p>
<button data-cal-command onclick="calCommand('test')">Rejouer 90° → −30°</button>
<button data-cal-command onclick="calCommand('save',{joint:document.getElementById('calJoint').value})">Valider et enregistrer</button>
</div>
<button data-cal-command onclick="calCommand('back')">Annuler / choisir un autre servo</button>
</div>
<button data-cal-command onclick="calCommand('exit')">Retour au menu principal</button>
<p class="imu-note" id="calBounds"></p>
</div>
)calhtml";

const char SCRIPT[] PROGMEM = R"caljs(
let calToken = 0;
try { calToken = Number(localStorage.getItem('servoCalSession')) || 0; } catch (error) {}
let calState = null;
let calBusy = false;
let calJointServo = 0;
let calCardsKey = '';
let calJointKey = '';
function rememberCalToken(value) {
  calToken = value;
  try { localStorage.setItem('servoCalSession', String(value)); } catch (error) {}
}
function calDisableCommands(disabled) {
  document.querySelectorAll('[data-cal-command]').forEach(button => { button.disabled = disabled; });
}
function renderServoCalibration(data) {
  if (calState && calState.session === data.session && data.revision < calState.revision) return;
  calState = data;
  const owned = data.menuActive && calToken === data.session;
  document.getElementById('mainMenu').hidden = owned;
  document.getElementById('servoCalMenu').hidden = !owned;
  document.getElementById('openServoCal').textContent = data.menuActive ? 'Reprendre Calibrate servo' : 'Calibrate servo';
  document.getElementById('openServoCal').disabled = calBusy;
  if (!owned) return;
  document.getElementById('calMessage').textContent = data.message;
  document.getElementById('calChoose').hidden = data.stage !== 0;
  document.getElementById('calWork').hidden = data.stage === 0;
  document.getElementById('calAdjust').hidden = data.stage !== 1;
  document.getElementById('calValidate').hidden = data.stage !== 3;
  calDisableCommands(calBusy);
  const pins = [25,26,27], angles = [-30,0,45,90];
  const cardsKey = JSON.stringify([data.session, data.records]);
  if (cardsKey !== calCardsKey) {
    const options = (record,i) => [1,2,3].map(q => {
      const defaultQ = record.joint || [1,2,3].find(id=>!data.records.some(r=>r.joint===id));
      const occupied = data.records.some((r,j)=>j!==i && r.joint===q);
      return '<option value="'+q+'" '+(occupied&&!record.joint?'disabled':'')+' '+(q===defaultQ?'selected':'')+'>q'+q+(occupied&&record.joint?' (échange)':'')+'</option>';
    }).join('');
    document.getElementById('calServoCards').innerHTML = data.records.map((record,i) =>
      '<div class="cal-card"><strong>Servo '+(i+1)+' · GPIO '+pins[i]+'</strong><p>'+
      (record.valid ? 'Calibré'+'<br>'+record.ticks.join(' / ')+' ticks' : 'Non calibré')+
      (record.joint ? '<br>Identifiant : q'+record.joint : '<br>Identifiant non affecté')+
      '</p><button data-cal-command onclick="calCommand(\'select\',{servo:'+(i+1)+'})">Calibrer</button>'+
      '<p><label>Identifiant : <select id="calMap'+(i+1)+'">'+options(record,i)+'</select></label></p>'+
      '<button data-cal-command onclick="calCommand(\'assign\',{servo:'+(i+1)+',joint:document.getElementById(\'calMap'+(i+1)+'\').value})">Enregistrer l’identifiant</button></div>').join('');
    calCardsKey = cardsKey;
  }
  calDisableCommands(calBusy);
  document.getElementById('calBounds').textContent = 'PWM 50 Hz / 16 bits · limites '+data.minTicks+' à '+data.maxTicks+' ticks (500–2500 µs). Ajuster progressivement selon la mécanique.';
  if (data.stage === 0) { calJointServo = 0; calJointKey = ''; return; }
  document.getElementById('calTitle').textContent = 'Servo '+data.servo+' · '+
    (data.stage===1 ? 'point '+(data.point+1)+'/4' : data.stage===2 ? 'Vérification automatique' : data.stage===3 ? 'Validation finale' : 'Session suspendue');
  document.getElementById('calTargetArm').style.transform = 'rotate('+(-data.targetDeg)+'deg)';
  document.getElementById('calAngleText').textContent = 'Cible : '+data.targetDeg.toFixed(1)+'°';
  document.getElementById('calInstruction').textContent = data.stage===1 ?
    'Ajuster + / − jusqu’à aligner le palonnier réel sur '+angles[data.point]+'°, puis Done.' :
    data.stage===2 ? 'Parcours 90°, 45°, 0°, −30° : maintien de 1 seconde à chaque position.' :
    data.stage===3 ? 'Vérifier le résultat, choisir q1/q2/q3, puis enregistrer en mémoire flash.' :
    'Les mouvements sont arrêtés. Recommencer la sélection ou revenir au menu principal.';
  const tick = data.servo ? data.actualTicks[data.servo-1] : 0;
  document.getElementById('calTicks').textContent = tick+' ticks · '+(tick*20000/65535).toFixed(2)+' µs';
  document.getElementById('calPoints').innerHTML = angles.map((angle,i) => '<tr><td>'+angle+'°</td><td>'+
    (data.draft.ticks[i] || '—')+'</td></tr>').join('');
  const jointSelect = document.getElementById('calJoint');
  const previous = Number(jointSelect.value);
  const occupied = data.records.filter((r,i)=>r.joint && i!==data.servo-1).map(r=>r.joint);
  const canSwap = Boolean(data.records[data.servo-1].joint);
  const available = [1,2,3].filter(q=>canSwap || !occupied.includes(q));
  const saved = data.records[data.servo-1].joint;
  const preferred = calJointServo===data.servo && available.includes(previous) ? previous :
    available.includes(saved) ? saved : available.includes(data.servo) ? data.servo : available[0];
  const jointKey = JSON.stringify([data.session,data.servo,data.records.map(r=>r.joint)]);
  if (jointKey !== calJointKey) {
    const optionsHtml = [1,2,3].map(q=>'<option value="'+q+'" '+(occupied.includes(q)&&!canSwap?'disabled':'')+' '+(q===preferred?'selected':'')+'>q'+q+(occupied.includes(q)?canSwap?' (échanger les identifiants)':' (déjà affecté)':'')+'</option>').join('');
    jointSelect.innerHTML = optionsHtml;
    calJointKey = jointKey;
  }
  calJointServo = data.servo;
}
async function openCalMenu() {
  if (calBusy) return;
  if (calState && calState.menuActive) {
    rememberCalToken(calState.session);
    renderServoCalibration(calState);
    return;
  }
  await calCommand('open');
}
function calAdjust(sign) {
  return calCommand('adjust',{delta:sign*Number(document.getElementById('calStep').value)});
}
async function calCommand(action, extra={}) {
  if (calBusy || (action!=='open' && (!calState || calToken!==calState.session))) return;
  calBusy = true; calDisableCommands(true);
  const controller = new AbortController();
  const timeout = setTimeout(()=>controller.abort(),3000);
  try {
    const body = new URLSearchParams(extra);
    if (action!=='open') { body.set('session',calToken); body.set('revision',calState.revision); }
    const response = await fetch('/servo-cal/'+action,{method:'POST',body,signal:controller.signal});
    const state = await response.json();
    if (action==='open' && response.ok) rememberCalToken(state.session);
    if (action==='exit' && response.ok) rememberCalToken(0);
    renderServoCalibration(state);
    if (!response.ok) document.getElementById('calMessage').textContent = state.message || 'Commande refusée. Actualiser l’état.';
  } catch (error) {
    document.getElementById('calMessage').textContent = 'Connexion interrompue : commande non confirmée. Attendre la reconnexion.';
  } finally {
    clearTimeout(timeout); calBusy=false; calDisableCommands(false);
    document.getElementById('openServoCal').disabled=false;
  }
}
)caljs";
}

const char* html() { return HTML; }
const char* script() { return SCRIPT; }
String stateJson()
{
  RuntimeLock::Guard guard;
  const auto s = ServoCalibration::getState();
  String json = "{\"menuActive\":";
  json += s.menuActive ? "true" : "false";
  json += ",\"stage\":" + String(int(s.stage));
  json += ",\"session\":" + String(s.session);
  json += ",\"revision\":" + String(s.revision);
  json += ",\"servo\":" + String(s.servo);
  json += ",\"point\":" + String(s.point);
  json += ",\"targetDeg\":" + String(s.targetDeg, 2);
  json += ",\"message\":\""; json += ServoCalibration::error(); json += "\"";
  json += ",\"minTicks\":" + String(ServoCalibration::MIN_TICKS);
  json += ",\"maxTicks\":" + String(ServoCalibration::MAX_TICKS);
  auto curve = [](const ServoCalibration::Calibration& c) {
    String value = "{\"valid\":" + String(int(c.valid)) + ",\"joint\":" + String(c.joint) + ",\"ticks\":[";
    for (int i=0;i<4;++i) { if(i) value+=","; value+=String(c.ticks[i]); }
    return value+"]}";
  };
  json += ",\"draft\":"+curve(s.draft)+",\"records\":[";
  for (int i=1;i<=3;++i) { if(i>1)json+=","; json+=curve(ServoCalibration::getCalibration(i)); }
  json += "],\"actualTicks\":[";
  for (int i=1;i<=3;++i) { if(i>1)json+=","; json+=String(ServoControl::getServoTicks(i)); }
  return json+"]}";
}
void registerRoutes(WebServer& server)
{
  web = &server;
  server.on("/servo-cal/open", HTTP_POST, []() { bool ok; { RuntimeLock::Guard guard; ok=ServoCalibration::openMenu(); } response(ok); });
  server.on("/servo-cal/select", HTTP_POST, []() { command("select"); });
  server.on("/servo-cal/assign", HTTP_POST, []() { command("assign"); });
  server.on("/servo-cal/adjust", HTTP_POST, []() { command("adjust"); });
  server.on("/servo-cal/done", HTTP_POST, []() { command("done"); });
  server.on("/servo-cal/test", HTTP_POST, []() { command("test"); });
  server.on("/servo-cal/save", HTTP_POST, []() { command("save"); });
  server.on("/servo-cal/back", HTTP_POST, []() { command("back"); });
  server.on("/servo-cal/exit", HTTP_POST, []() { command("exit"); });
}
}
