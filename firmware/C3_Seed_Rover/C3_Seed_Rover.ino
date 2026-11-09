/*
  Seed Dispensing Rover - ESP32-C3 Super Mini + TC1508A motor driver + MPU6050 + servo
  Wi-Fi controller hosted on the rover: joystick driving, seed dispensing
  (single / continuous), servo + square-path + gyro calibration, OTA updates.
  Everything is non-blocking (millis() state machines).

  Board  : ESP32C3 Dev Module (ESP32 Arduino Core 3.x), USB CDC On Boot: Enabled
  Library: WebSockets by Markus Sattler (2.7.x)
  Wi-Fi  : AP "C3-ROVER" / "12345678"  ->  http://192.168.4.1
  OTA    : host c3-rover, password 12345678

  Wiring:
    GPIO3  -> TC1508A INT1   GPIO4 -> INT2   (MOTOR-A: right motor pair)
    GPIO5  -> TC1508A INT3   GPIO6 -> INT4   (MOTOR-B: left motor pair)
    GPIO7  -> not connected on the TC1508A (STBY on a DRV8833)
    GPIO10 -> servo signal (servo powered from the 5 V buck)
    GPIO20 -> MPU6050 SDA    GPIO21 -> MPU6050 SCL   (MPU6050 VCC -> 3V3)
    GND common to battery, TC1508A, buck, servo and MPU6050
    Side/direction mapping for this wiring: SIDES_SWAPPED true, RIGHT_REVERSED false, JOY_SWAP_LR false.
    On the original DRV8833 wiring (AO = left, BO = right) use false / true / true.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <WebSocketsServer.h>   // "WebSockets" library by Markus Sattler
#include <Wire.h>               // MPU6050 over I2C (no extra library)

// ---------------- Pins ----------------
#define AIN1 3
#define AIN2 4
#define BIN1 5
#define BIN2 6
#define STBY 7
#define SERVO_PIN 10        // seed-dispenser servo signal (free GPIO - change if wired elsewhere)

// ---------------- PWM ----------------
#define PWM_FREQ 1000       // 1 kHz: much more torque from BO motors than 20 kHz
#define PWM_RES  8          // 0-255

#define DRIVE_SPEED 150     // forward / backward
#define TURN_SPEED  170     // pivot left / right

#define SIDES_SWAPPED  true // TC1508A wiring: GPIO3/4 (MOTOR-A) drive the RIGHT wheels, GPIO5/6 (MOTOR-B) the LEFT
#define RIGHT_REVERSED false // right side polarity reversed in software (true on the old DRV8833 wiring)
#define SLOW_DECAY     true // slow decay: fixes one side stalling on pivot turns

// ---------------- Timing / limits ----------------
#define SETTLE_TIME   150   // ms brake between forward and turn (0 = none). Reduces current spike + skid.
#define LINK_TIMEOUT  2500  // ms without any browser request while moving -> stop
#define JOY_TIMEOUT   500   // ms without a joystick packet while driving -> stop
#define JOY_DEADZONE  10    // joystick range is -100..100
#define JOY_SWAP_LR   false // swap joystick left/right (true on the old DRV8833 wiring)
#define JOY_MAX_PWM   100   // PWM at 100% speed (TC1508A: kept low so the driver stays cool)
#define JOY_MIN_PWM   50    // lowest PWM while moving = small joystick push / slider at 20 %
#define JOY_START_PWM 40    // soft start: a side leaving standstill begins here...
#define JOY_ACCEL     300   // ...and speeds up at this many PWM/s (gentle on the driver and gearboxes)
#define JOY_RAMP      900   // PWM change per second (0 -> 150 in ~0.17 s) = smooth start/stop

// ---------------- MPU6050 (gyro turns, hold straight, tilt safety) ----------------
#define MPU_SDA       20
#define MPU_SCL       21
#define MPU_ADDR      0x68  // AD0 low (0x69 if AD0 is tied high)
#define GYRO_LSB      65.5f // LSB per deg/s at +-500 deg/s range
#define TILT_LIMIT    35.0f // deg: stop motors + dispenser beyond this
#define TILT_CLEAR    25.0f // deg: must stay below this for 1 s before driving again
#define TURN_TIMEOUT  4000  // ms: a gyro turn gives up after this
#define HOLD_KP       4.0f  // hold straight: PWM correction per degree of drift
#define HOLD_MAX      40    // hold straight: max PWM correction
#define FWD_MIN  100
#define FWD_MAX  10000
#define TURN_MIN 50
#define TURN_MAX 3000

// ---------------- Servo (seed dispenser) ----------------
#define SERVO_FREQ    50    // standard hobby servo frame: 50 Hz (20 ms)
#define SERVO_RES     14    // 14-bit duty -> ~1.2 us steps
#define SERVO_MIN_US  500   // pulse at 0 deg   (SG90/MG90 typical 500-2400 us)
#define SERVO_MAX_US  2400  // pulse at 180 deg
#define SERVO_MAX_SPD 600   // deg/s limit for swing speed (0 = move at servo's own max)

// ---------------- Wi-Fi ----------------
const char* ssid     = "C3-ROVER";
const char* password = "12345678";

WebServer server(80);
WebSocketsServer ws(81);        // joystick: one persistent connection
Preferences prefs;

// ---------------- Settings (saved in flash) ----------------
unsigned long forwardTime = 2000;
unsigned long turnTime    = 480;
bool  useGyro   = true;             // gyro decides when a turn is done (timed if no MPU)
float turnAngle = 90;               // deg per test turn / square corner
float turnTrim  = 8;                // deg: stop this much early to allow for coasting
bool  holdOn    = true;             // use the gyro to drive straight

// ---------------- State machine ----------------
enum Mode  { MODE_IDLE, MODE_TEST, MODE_SQUARE, MODE_JOY };
unsigned long lastJoyMs = 0;
int   joyTgtL = 0, joyTgtR = 0;     // target PWM per side (-JOY_MAX_PWM..JOY_MAX_PWM)
float joyCurL = 0, joyCurR = 0;     // ramped PWM actually applied
unsigned long joyLastRamp = 0;
bool  joyReleasing = false;
bool  joyStraight = false;          // stick pushed straight ahead/back -> gyro holds heading
bool  joyHoldActive = false;
float joyHoldTarget = 0;
int   joySpeedPct = 60;
const char* resetReason = "Unknown";

enum Phase { PH_FORWARD, PH_PAUSE_A, PH_TURN, PH_PAUSE_B };

Mode  mode  = MODE_IDLE;
Phase phase = PH_FORWARD;
uint8_t side = 1;
unsigned long phaseStart    = 0;
unsigned long phaseDuration = 0;
unsigned long lastClientMs  = 0;
unsigned long lastStaCheck  = 0;
char statusText[40] = "Ready";

// ---------------- Servo settings (saved in flash, namespace "servo") ----------------
// Hopper angles are LOCKED: calibrated for this dispenser mechanism and not changeable
// from the controller. Change them only here (and re-test) if the mechanism is rebuilt.
const int servoClosed = 135;        // deg: hopper closed
const int servoOpen   = 75;         // deg: hopper open, drops seeds
int servoSpeed  = 200;              // swing speed, deg/s (0 = full speed)
unsigned long servoHold = 100;      // ms held open (calibrated)
int servoSwings = 1;                // open/close cycles per dispense

// ---------------- Servo motion state ----------------
enum SvPhase { SV_IDLE, SV_OPENING, SV_HOLD, SV_CLOSING };
SvPhase svPhase = SV_IDLE;
float servoPos = 20, servoTarget = 20;
int   servoMoveSpeed = 0;
unsigned long servoLastMs = 0, svHoldStart = 0;
int   svLeft = 0, svRunClosed = 20, svRunOpen = 90, svRunSpeed = 180;
unsigned long svRunHold = 300;

// ---------------- Continuous dispense (while driving) ----------------
bool  contOn = false;
int   contSpeed = 200, contSwings = 1;
unsigned long contInterval = 1000;  // ms between dispenses
unsigned long contNext = 0;
unsigned long dispCount = 0;        // completed dispense cycles since power-on

// =====================================================================
//  Web page
// =====================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0d0f12">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-capable" content="yes">
<title>C3 Rover</title>
<style>
:root{--bg:#0d0f12;--s1:#15181d;--s2:#1c2027;--s3:#252a33;--bd:#2a3039;--tx:#e7e9ec;--mu:#8a929e;--fa:#5d6470;
--ac:#3b82f6;--acp:#2563eb;--acs:rgba(59,130,246,.14);--dg:#dc2626;--dgp:#b91c1c;--wn:#f59e0b;--wns:rgba(245,158,11,.12);--ok:#22c55e;--r:12px}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
[hidden]{display:none!important}
html,body{margin:0;background:var(--bg);color:var(--tx)}
body{font:15px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;padding:0 0 calc(92px + env(safe-area-inset-bottom));-webkit-user-select:none;user-select:none}
.wrap{max-width:980px;margin:0 auto;padding:0 14px}
.num{font-variant-numeric:tabular-nums}
/* header */
header{position:sticky;top:0;z-index:5;background:var(--bg);border-bottom:1px solid var(--bd)}
.hd{display:flex;align-items:center;justify-content:space-between;height:56px}
.brand{display:flex;align-items:center;gap:10px}
.logo{width:30px;height:30px;border-radius:8px;background:var(--ac);display:grid;place-items:center}
.logo svg{width:18px;height:18px}
.bt{font-weight:650;font-size:16px;letter-spacing:.2px}
.bs{font-size:12px;color:var(--mu);margin-top:-2px}
.pill{display:inline-flex;align-items:center;gap:7px;height:30px;padding:0 12px;border-radius:15px;background:var(--s2);border:1px solid var(--bd);font-size:13px;color:var(--mu)}
.dot{width:8px;height:8px;border-radius:50%;background:var(--fa)}
.pill.on .dot{background:var(--ok)}.pill.on{color:var(--tx)}
.pill.off .dot{background:var(--dg)}
nav{display:flex;gap:4px;padding:0 0 10px;overflow-x:auto;scrollbar-width:none}
nav::-webkit-scrollbar{display:none}
.tab{flex:1;min-width:84px;height:38px;border:0;border-radius:9px;background:transparent;color:var(--mu);font:inherit;font-weight:550;font-size:14px}
.tab.on{background:var(--s2);color:var(--tx);box-shadow:inset 0 0 0 1px var(--bd)}
/* layout */
main.wrap{padding-top:14px}
.grid{display:grid;gap:12px}
.card{background:var(--s1);border:1px solid var(--bd);border-radius:var(--r);padding:16px}
.ct{display:flex;align-items:center;justify-content:space-between;margin:0 0 12px}
.ct h2{margin:0;font-size:13px;font-weight:600;text-transform:uppercase;letter-spacing:.6px;color:var(--mu)}
.ct .aux{font-size:13px;color:var(--mu)}
.hint{font-size:13px;color:var(--mu);margin-top:10px}
.hint.warn{color:var(--wn)}
@media(min-width:720px){.two{grid-template-columns:1fr 1fr}.span{grid-column:1/-1}}
/* status */
.status{display:flex;align-items:center;gap:12px;background:var(--s1);border:1px solid var(--bd);border-left:3px solid var(--fa);border-radius:var(--r);padding:12px 16px}
.status.active{border-left-color:var(--ac)}
.status.warn{border-left-color:var(--wn);background:var(--wns)}
.status.err{border-left-color:var(--dg)}
.stt{flex:1;min-width:0;font-weight:600;font-size:16px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.badge{font-size:12px;font-weight:600;color:var(--mu);background:var(--s2);border:1px solid var(--bd);border-radius:6px;padding:3px 8px;white-space:nowrap}
/* KPI tiles */
.kpis{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}
@media(min-width:720px){.kpis{grid-template-columns:repeat(4,1fr)}}
.kpi{background:var(--s1);border:1px solid var(--bd);border-radius:var(--r);padding:10px 14px}
.kl{font-size:12px;color:var(--mu)}
.kv{font-size:20px;font-weight:650;margin-top:1px}
.kpi.bad{border-color:var(--wn)}.kpi.bad .kv{color:var(--wn)}
/* joystick */
.joyw{display:flex;flex-direction:column;align-items:center}
#joy{position:relative;width:min(250px,66vw,58vh);aspect-ratio:1;border-radius:50%;background:var(--s2);border:1px solid var(--bd);touch-action:none;margin:4px 0 6px}
#joy::before,#joy::after{content:"";position:absolute;background:var(--s3);pointer-events:none}
#joy::before{left:50%;top:12%;bottom:12%;width:1px}
#joy::after{top:50%;left:12%;right:12%;height:1px}
.ring{position:absolute;inset:22%;border-radius:50%;border:1px dashed var(--s3);pointer-events:none}
#knob{position:absolute;z-index:1;width:34%;aspect-ratio:1;border-radius:50%;background:var(--s3);border:2px solid var(--ac);left:50%;top:50%;transform:translate(-50%,-50%);pointer-events:none}
#joy.held #knob{background:var(--ac)}
.jout{font:12px ui-monospace,Menlo,Consolas,monospace;color:var(--fa);min-height:16px}
/* controls */
.field{margin-top:14px}
.ct+.field,.ct+.cols>.field{margin-top:0}
.fl{display:flex;justify-content:space-between;font-size:13px;color:var(--mu);margin-bottom:6px}
.fl b{color:var(--tx);font-weight:600}
.inp{display:flex;align-items:center;height:46px;background:var(--bg);border:1px solid var(--bd);border-radius:10px;overflow:hidden}
.inp:focus-within{border-color:var(--ac)}
.inp input{flex:1;min-width:0;height:100%;border:0;background:transparent;color:var(--tx);font:inherit;font-size:17px;font-variant-numeric:tabular-nums;padding:0 12px;outline:0;-webkit-user-select:text;user-select:text}
.inp .u{padding:0 12px;color:var(--fa);font-size:13px}
.inp button{height:100%;min-width:48px;border:0;border-left:1px solid var(--bd);background:var(--s2);color:var(--tx);font:inherit;font-size:14px}
.inp button.l{border-left:0;border-right:1px solid var(--bd)}
.inp button:active{background:var(--s3)}
.cols{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1fr);gap:10px}
.fl{gap:8px}.fl span{white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:28px;background:transparent;margin:0}
input[type=range]::-webkit-slider-runnable-track{height:4px;border-radius:2px;background:var(--s3)}
input[type=range]::-moz-range-track{height:4px;border-radius:2px;background:var(--s3)}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;border-radius:50%;background:var(--tx);border:0;margin-top:-9px}
input[type=range]::-moz-range-thumb{width:22px;height:22px;border-radius:50%;background:var(--tx);border:0}
.btn{display:flex;align-items:center;justify-content:center;gap:8px;width:100%;height:48px;border:1px solid var(--bd);border-radius:10px;background:var(--s2);color:var(--tx);font:inherit;font-weight:600;font-size:15px}
.btn:active{background:var(--s3)}
.btn.pri{background:var(--ac);border-color:var(--ac);color:#fff}
.btn.pri:active{background:var(--acp)}
.btn:disabled,.inp button:disabled,input:disabled{opacity:.4}
.btns{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:14px}
.big{height:64px;font-size:17px}
.row{display:flex;align-items:center;justify-content:space-between;gap:12px;min-height:48px}
.row+.row{border-top:1px solid var(--bd)}
.rl{font-size:15px}.rs{font-size:12px;color:var(--mu)}
.sw{position:relative;flex:none;width:50px;height:30px;border-radius:15px;border:0;background:var(--s3);padding:0}
.sw::after{content:"";position:absolute;top:3px;left:3px;width:24px;height:24px;border-radius:50%;background:#fff;transition:left .15s}
.sw[aria-checked=true]{background:var(--ac)}
.sw[aria-checked=true]::after{left:23px}
.sw:disabled{opacity:.4}
details{margin-top:14px;border-top:1px solid var(--bd);padding-top:4px}
summary{list-style:none;cursor:pointer;display:flex;justify-content:space-between;align-items:center;height:40px;font-size:14px;color:var(--mu)}
summary::-webkit-details-marker{display:none}
summary::after{content:"+";font-size:18px}
details[open] summary::after{content:"\2212"}
.kv2{display:grid;grid-template-columns:auto 1fr;gap:8px 16px;font-size:14px}
.kv2 dt{color:var(--mu)}.kv2 dd{margin:0;text-align:right}
pre{margin:0;font:12px/1.5 ui-monospace,Menlo,Consolas,monospace;color:var(--mu);white-space:pre-wrap;max-height:260px;overflow:auto;-webkit-user-select:text;user-select:text}
/* stop bar + toast */
.stopbar{position:fixed;left:0;right:0;bottom:0;z-index:6;background:var(--bg);border-top:1px solid var(--bd);padding:10px 14px calc(10px + env(safe-area-inset-bottom))}
#stop{display:block;width:100%;max-width:952px;margin:0 auto;height:60px;border:0;border-radius:12px;background:var(--dg);color:#fff;font:inherit;font-size:18px;font-weight:700;letter-spacing:1.5px;touch-action:manipulation}
#stop:active{background:var(--dgp)}
.toast{position:fixed;left:50%;bottom:calc(96px + env(safe-area-inset-bottom));z-index:7;transform:translateX(-50%);max-width:calc(100% - 28px);padding:10px 16px;border-radius:10px;background:var(--s3);border:1px solid var(--bd);font-size:14px;opacity:0;pointer-events:none;transition:opacity .2s}
.toast.show{opacity:1}
.toast.ok{border-color:#1f6f3c}.toast.err{border-color:var(--dg)}.toast.warn{border-color:var(--wn)}
.caution{display:flex;gap:10px;align-items:flex-start;margin-top:14px;padding:12px;border-radius:10px;background:var(--wns);border:1px solid rgba(245,158,11,.35);font-size:13px;color:var(--tx)}
.caution svg{flex:none;width:18px;height:18px;color:var(--wn);margin-top:1px}
.caution b{color:var(--wn);font-weight:600}
.ro{background:var(--bg);border:1px solid var(--bd);border-radius:10px;padding:10px 14px}
.lockd{display:inline-flex;align-items:center;gap:5px}
.lockd svg{width:14px;height:14px}
/* drive layout: phone portrait (default) */
.drive{display:grid;gap:12px;margin-top:12px}
.mid{display:contents}
.kpis{order:1}.jcard{order:2}.spcard{order:3}.dcard{order:4}
#stMini{display:none}
/* tablet / desktop */
@media(min-width:720px) and (min-height:501px){
.drive{grid-template-columns:1fr 1fr}
.kpis{grid-column:1/-1}
.jcard{grid-row:span 2}
}
/* phone landscape: cockpit - joystick | stats | dispenser */
@media(orientation:landscape) and (max-height:500px){
body{padding-bottom:calc(64px + env(safe-area-inset-bottom))}
header{position:static}
header .wrap{display:flex;align-items:center;gap:14px;height:52px}
.hd{display:contents}
.bs{display:none}
nav{order:1;flex:1;padding:0}
.tab{height:34px}
.pill{order:2}
main.wrap{padding-top:10px}
body.t-drive #stCard{display:none}
.drive{grid-template-columns:auto 1fr 1fr;gap:10px;margin-top:0}
.jcard{order:1}.mid{order:2;display:flex;flex-direction:column;gap:10px}.dcard{order:3}
#stMini{display:flex;padding:8px 12px}
#stMini .stt{font-size:14px}
.kpis{grid-template-columns:1fr 1fr;gap:10px}
.kpi{padding:6px 12px}.kv{font-size:17px}
.card{padding:12px}
.jcard .ct,#jout{display:none}
#joy{width:min(210px,52vh);margin:0}
.dcard .big{height:52px}
.stopbar{padding:8px 14px calc(8px + env(safe-area-inset-bottom))}
#stop{height:48px}
.toast{bottom:calc(72px + env(safe-area-inset-bottom))}
}
</style></head><body class="t-drive">

<header><div class="wrap">
<div class="hd">
<div class="brand">
<div class="logo"><svg viewBox="0 0 24 24" fill="none" stroke="#fff" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M12 22V12"/><path d="M12 12C12 7 8 5 4 5c0 5 3 7 8 7z"/><path d="M12 14c0-4 3-6 8-6 0 4-3 6-8 6z"/></svg></div>
<div><div class="bt">C3 Rover</div><div class="bs">Seed dispenser</div></div>
</div>
<div id="link" class="pill"><span class="dot"></span><span id="linkT">Connecting</span></div>
</div>
<nav>
<button class="tab on" data-t="drive">Drive</button>
<button class="tab" data-t="disp">Dispenser</button>
<button class="tab" data-t="cal">Calibration</button>
<button class="tab" data-t="sys">System</button>
</nav>
</div></header>

<main class="wrap">
<div id="stCard" class="status"><div id="stText" class="stt">Connecting to rover...</div><div id="stMode" class="badge">-</div></div>

<!-- ===== DRIVE ===== -->
<section id="p-drive" class="panel drive">
<div class="card jcard">
<div class="ct"><h2>Drive</h2><span class="aux">Hold and drag</span></div>
<div class="joyw">
<div id="joy"><div class="ring"></div><div id="knob"></div></div>
<div class="jout" id="jout">Released</div>
</div>
</div>
<div class="mid">
<div id="stMini" class="status"><div id="stMiniT" class="stt">-</div></div>
<div class="kpis">
<div class="kpi"><div class="kl">Heading</div><div class="kv num" id="kHd">--</div></div>
<div class="kpi" id="kTlBox"><div class="kl">Tilt</div><div class="kv num" id="kTl">--</div></div>
<div class="kpi"><div class="kl">Dispense cycles</div><div class="kv num" id="kDc">0</div></div>
<div class="kpi"><div class="kl">Motors L / R</div><div class="kv num" id="kMo">0 / 0</div></div>
</div>
<div class="card spcard">
<div class="fl"><span>Speed limit</span><b class="num"><span id="spv">60</span>%</b></div>
<input id="spd" type="range" min="20" max="100" step="5" value="60">
</div>
</div>
<div class="card dcard">
<div class="ct"><h2>Seed dispenser</h2><span class="aux" id="svState">Idle</span></div>
<button id="dBtn" class="btn pri big">Dispense once</button>
<div class="row" style="margin-top:8px">
<div><div class="rl">Continuous dispense</div><div class="rs" id="cSub">Drops seeds at a fixed interval</div></div>
<button id="cBtn" class="sw" role="switch" aria-checked="false" aria-label="Continuous dispense"></button>
</div>
<details>
<summary>Dispense options</summary>
<div class="field"><div class="fl"><span>Swing speed</span><span>0 = full speed</span></div>
<div class="inp"><input id="dsp" type="number" inputmode="numeric" min="0" max="600" step="10"><span class="u">&deg;/s</span></div></div>
<div class="cols">
<div class="field"><div class="fl"><span>Cycles per drop</span></div>
<div class="inp"><input id="dn" type="number" inputmode="numeric" min="1" max="20"></div></div>
<div class="field"><div class="fl"><span>Interval</span></div>
<div class="inp"><input id="di" type="number" inputmode="numeric" min="100" max="10000" step="100"><span class="u">ms</span></div></div>
</div>
<div class="hint">Speed and cycles follow the saved Dispenser settings (change them here for this session only). Interval is saved on this phone.</div>
</details>
</div>
</section>

<!-- ===== DISPENSER SETTINGS ===== -->
<section id="p-disp" class="panel grid two" style="margin-top:12px" hidden>
<div class="card">
<div class="ct"><h2>Hopper angles</h2><span class="aux lockd"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect x="5" y="11" width="14" height="10" rx="2"/><path d="M8 11V7a4 4 0 0 1 8 0v4"/></svg>Locked</span></div>
<div class="cols">
<div class="ro"><div class="kl">Closed</div><div class="kv num"><span id="svcV">-</span>&deg;</div></div>
<div class="ro"><div class="kl">Open</div><div class="kv num"><span id="svoV">-</span>&deg;</div></div>
</div>
<div class="caution">
<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M10.3 3.9 1.8 18a2 2 0 0 0 1.7 3h17a2 2 0 0 0 1.7-3L13.7 3.9a2 2 0 0 0-3.4 0z"/><path d="M12 9v4M12 17h.01"/></svg>
<div><b>Calibrated for this hopper.</b> The open and closed angles are fixed in the firmware and can't be changed from the controller. A wrong angle can jam the hopper or spill seeds.</div>
</div>
</div>
<div class="card">
<div class="ct"><h2>Dispense timing</h2><span class="aux" id="svDirty"></span></div>
<div class="cols">
<div class="field"><div class="fl"><span>Swing speed</span></div><div class="inp"><input id="svsp" type="number" inputmode="numeric" min="0" max="600" step="10"><span class="u">&deg;/s</span></div></div>
<div class="field"><div class="fl"><span>Hold open</span></div><div class="inp"><input id="svh" type="number" inputmode="numeric" min="0" max="5000" step="50"><span class="u">ms</span></div></div>
</div>
<div class="field"><div class="fl"><span>Swings per dispense</span></div><div class="inp"><input id="svn" type="number" inputmode="numeric" min="1" max="20"></div></div>
<div class="caution">
<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M10.3 3.9 1.8 18a2 2 0 0 0 1.7 3h17a2 2 0 0 0 1.7-3L13.7 3.9a2 2 0 0 0-3.4 0z"/><path d="M12 9v4M12 17h.01"/></svg>
<div>These settings change how many seeds drop. Press <b>Test</b> with seeds in the hopper before you press <b>Save</b>.</div>
</div>
<div class="btns">
<button id="svTest" class="btn">Test</button>
<button id="svSave" class="btn pri">Save</button>
</div>
<div class="hint">Test uses the values above without saving. Swing speed 0 = full speed.</div>
</div>
</section>

<!-- ===== CALIBRATION (square + gyro) ===== -->
<section id="p-cal" class="panel grid two" style="margin-top:12px" hidden>
<div class="card">
<div class="ct"><h2>Square path</h2><span class="aux" id="calDirty"></span></div>
<div class="field"><div class="fl"><span>Forward time per side</span></div>
<div class="inp"><input id="fwd" class="lock" type="number" inputmode="numeric" min="100" max="10000" step="10"><span class="u">ms</span></div></div>
<div class="field"><div class="fl"><span>Right-turn time</span><span>used without gyro</span></div>
<div class="inp">
<button class="l step lock" data-d="-50">&minus;50</button><button class="l step lock" data-d="-10">&minus;10</button>
<input id="turn" class="lock" type="number" inputmode="numeric" min="50" max="3000" step="5" style="text-align:center">
<button class="step lock" data-d="10">+10</button><button class="step lock" data-d="50">+50</button>
</div></div>
<div class="btns">
<button id="test" class="btn lock">Test right turn</button>
<button id="square" class="btn lock">Run square</button>
</div>
</div>
<div class="card">
<div class="ct"><h2>Gyro (MPU6050)</h2><span class="aux" id="mpuState">-</span></div>
<div class="row"><div><div class="rl">Gyro turns</div><div class="rs">Stop turns on measured angle</div></div><button id="gy" class="sw lock" role="switch" aria-checked="false" aria-label="Gyro turns"></button></div>
<div class="row"><div><div class="rl">Hold straight</div><div class="rs">Correct drift when driving straight</div></div><button id="hh" class="sw lock" role="switch" aria-checked="false" aria-label="Hold straight"></button></div>
<div class="cols">
<div class="field"><div class="fl"><span>Turn angle</span></div><div class="inp"><input id="ta" class="lock" type="number" inputmode="numeric" min="10" max="360"><span class="u">&deg;</span></div></div>
<div class="field"><div class="fl"><span>Stop early</span></div><div class="inp"><input id="tt" class="lock" type="number" inputmode="numeric" min="0" max="45"><span class="u">&deg;</span></div></div>
</div>
<div class="hint" id="mpuHint"></div>
<div class="btns">
<button id="mcal" class="btn lock">Recalibrate gyro</button>
<button id="save" class="btn pri lock">Save settings</button>
</div>
</div>
</section>

<!-- ===== SYSTEM ===== -->
<section id="p-sys" class="panel grid two" style="margin-top:12px" hidden>
<div class="card">
<div class="ct"><h2>Device</h2></div>
<dl class="kv2 num">
<dt>Uptime</dt><dd id="yUp">-</dd>
<dt>Last reset</dt><dd id="yRst">-</dd>
<dt>Free memory</dt><dd id="yHeap">-</dd>
<dt>Gyro</dt><dd id="yMpu">-</dd>
<dt>Turn direction</dt><dd id="ySk">-</dd>
<dt>Connection</dt><dd id="yLink">-</dd>
<dt>Wi-Fi</dt><dd>C3-ROVER &middot; 192.168.4.1</dd>
</dl>
<div class="hint" id="yWarn"></div>
</div>
<div class="card">
<div class="ct"><h2>Event log</h2><span class="aux">latest last</span></div>
<pre id="rlog">-</pre>
</div>
<div class="card span">
<div class="ct"><h2>Diagnostics</h2></div>
<pre id="dbg">-</pre>
</div>
</section>
</main>

<div id="toast" class="toast"></div>
<div class="stopbar"><button id="stop">STOP</button></div>

<script>
const $=id=>document.getElementById(id);
const store={get(k){try{return localStorage.getItem(k);}catch(_){return null;}},set(k,v){try{localStorage.setItem(k,v);}catch(_){}}};
const MODES=['Idle','Turn test','Square run','Driving'];
let S=null,lastSt=0,cur='drive';
let calLoaded=false,svLoaded=false,dLoaded=false,contOn=false,lastUp=-1,reboots=0;
let saved={fwd:0,turn:0,ta:0,tt:0,gy:false,hh:false};

// ---------- helpers ----------
function get(path){
  const c=new AbortController(),t=setTimeout(()=>c.abort(),1500);
  return fetch(path,{cache:'no-store',signal:c.signal}).then(r=>r.json()).finally(()=>clearTimeout(t));
}
let toastT=0;
function toast(t,k){const e=$('toast');e.textContent=t;e.className='toast show '+(k||'');clearTimeout(toastT);toastT=setTimeout(()=>{e.className='toast';},3000);}
function val(id){const e=$(id),v=parseInt(e.value,10);return(v>=+e.min&&v<=+e.max)?v:NaN;}
function sw(id){return $(id).getAttribute('aria-checked')==='true';}
function setSw(id,on){$(id).setAttribute('aria-checked',on?'true':'false');}
function f1(v){return Number(v).toFixed(1);}
function pct(p){return Math.round(p*100/255);}
function hms(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60),x=s%60;return (h?h+'h ':'')+(h||m?m+'m ':'')+x+'s';}
function act(path){
  return get(path).then(d=>{toast(d.msg,d.ok?'ok':'err');reqStatus();return d;})
    .catch(()=>{toast('No response from rover','err');return null;});
}

// ---------- tabs ----------
function tab(n){
  cur=n;
  document.body.className='t-'+n;
  document.querySelectorAll('.tab').forEach(b=>b.classList.toggle('on',b.dataset.t===n));
  document.querySelectorAll('.panel').forEach(p=>{p.hidden=p.id!=='p-'+n;});
  if(n!=='drive')jEnd();
  store.set('tab',n);
  if(n==='sys')loadLog();
}
document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>tab(b.dataset.t));

// ---------- status ----------
function setSt(text,cls,badge){
  $('stText').textContent=text;$('stMiniT').textContent=text;
  $('stCard').className='status'+cls;$('stMini').className='status'+cls;
  $('stMode').textContent=badge;
}
function onStatus(d){
  S=d;lastSt=Date.now();
  setSt(d.status,d.tlt?' warn':(d.busy?' active':''),d.tlt?'Tilted':(MODES[d.md]||'Idle'));
  $('kHd').textContent=d.mpu?f1(d.hd)+'°':'--';
  $('kTl').textContent=d.mpu?f1(d.tl)+'°':'--';
  $('kTlBox').classList.toggle('bad',!!d.tlt);
  $('kDc').textContent=d.dc;
  $('kMo').textContent=pct(d.ml)+' / '+pct(d.mr);
  renderDisp(d);renderServo(d);renderCal(d);renderSys(d);
}
let inflight=false;
function httpPoll(){
  if(inflight||jOn)return;               // the rover serves one HTTP request at a time
  inflight=true;
  get('/status').then(onStatus).catch(()=>{}).finally(()=>{inflight=false;});
}
function reqStatus(){
  if(jOn)return;                         // while driving the radio carries joystick packets only
  if(wsOpen()){if(ws.bufferedAmount===0)ws.send('s');}
  else httpPoll();
}
setInterval(()=>{
  reqStatus();
  const on=lastSt&&Date.now()-lastSt<1500;
  $('link').className='pill '+(on?'on':(lastSt?'off':''));
  $('linkT').textContent=on?'Connected':(lastSt?'Offline':'Connecting');
  if(lastSt&&!on)setSt('Connection lost',' err','Offline');
},250);

// ---------- STOP ----------
function stop(n){
  get('/stop').then(d=>{toast(d.msg,'ok');reqStatus();})
    .catch(()=>{if(n<3)stop(n+1);else toast('STOP not confirmed - check the rover','err');});
}
$('stop').addEventListener('pointerdown',e=>{e.preventDefault();jEnd();stop(1);});
$('stop').addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){jEnd();stop(1);}});

// ---------- dispenser (drive tab) ----------
function dOpts(){
  const s=val('dsp'),n=val('dn'),i=val('di');
  if([s,n,i].some(isNaN)){toast('Speed 0-600, cycles 1-20, interval 100-10000 ms','err');return null;}
  store.set('dopts',JSON.stringify({s,n,i}));
  return 'sp='+s+'&n='+n+'&i='+i;
}
function showCont(on){
  contOn=on;setSw('cBtn',on);
  $('cSub').textContent=on?'Running - every '+($('di').value||'?')+' ms':'Drops seeds at a fixed interval';
}
function sendCont(on){
  const p=dOpts();if(!p)return;
  get('/cont?on='+(on?1:0)+'&'+p).then(d=>{toast(d.msg,d.ok?'ok':'err');if(d.ok)showCont(on);})
    .catch(()=>toast('No response from rover','err'));
}
$('dBtn').onclick=()=>{const p=dOpts();if(p)act('/disp?'+p);};
$('cBtn').onclick=()=>sendCont(!contOn);
['dsp','dn','di'].forEach(id=>$(id).onchange=()=>{if(contOn)sendCont(true);else dOpts();});
let dSS=null,dSN=null;                  // rover's saved swing speed / swings last applied here
function renderDisp(d){
  if(!dLoaded){                          // speed + cycles come from the rover; interval from this phone
    let o=null;try{o=JSON.parse(store.get('dopts'));}catch(_){}
    $('di').value=(o&&o.i>=100&&o.i<=10000)?o.i:1000;
    dLoaded=true;
  }
  if(d.ss!==dSS||d.sn!==dSN){            // first load, or Save on the Dispenser tab
    const first=dSS===null;
    dSS=d.ss;dSN=d.sn;
    $('dsp').value=d.ss;$('dn').value=d.sn;
    if(!first&&contOn&&d.co)sendCont(true);   // running continuous dispense picks up the new values
  }
  if(d.co!==contOn)showCont(d.co);       // rover may switch it off (STOP, tilt, link loss)
  $('svState').textContent=d.svb?'Dispensing':(d.co?'Continuous':'Idle');
  $('dBtn').disabled=d.svb||d.tlt;
}

// ---------- dispense timing (hopper angles are locked in firmware) ----------
const SVF=['svsp','svh','svn'];
function svParams(){
  const s=val('svsp'),h=val('svh'),n=val('svn');
  if([s,h,n].some(isNaN)){toast('Speed 0-600, hold 0-5000, swings 1-20','err');return null;}
  return 'sp='+s+'&h='+h+'&n='+n;
}
let svSaved=null;
function svIsDirty(){return !!svSaved&&SVF.some((id,i)=>val(id)!==svSaved[i]);}
function svDirty(){
  const d=svIsDirty();
  $('svDirty').textContent=d?'Unsaved changes':'';
  $('svDirty').style.color=d?'var(--wn)':'';
}
SVF.forEach(id=>$(id).addEventListener('input',svDirty));
$('svTest').onclick=()=>{const p=svParams();if(p)act('/servo/test?'+p);};
$('svSave').onclick=()=>{
  const p=svParams();if(!p)return;
  if(!confirm('Save new dispense timing?'+String.fromCharCode(10,10)+'This changes how many seeds drop on every dispense. Make sure you tested it first.'))return;
  act('/servo/save?'+p);
};
function renderServo(d){
  $('svcV').textContent=d.sc;$('svoV').textContent=d.so;
  const prev=svSaved;
  svSaved=[d.ss,d.sh,d.sn];
  // first load, or the saved values changed while the fields were untouched: show the saved values
  if(!svLoaded||(prev&&SVF.some((id,i)=>prev[i]!==svSaved[i])&&!SVF.some((id,i)=>val(id)!==prev[i]))){
    $('svsp').value=d.ss;$('svh').value=d.sh;$('svn').value=d.sn;svLoaded=true;
  }
  svDirty();
  ['svTest','svSave'].forEach(id=>$(id).disabled=d.svb||d.tlt);
}

// ---------- square + gyro calibration ----------
function dirtyCal(){
  return val('fwd')!==saved.fwd||val('turn')!==saved.turn||val('ta')!==saved.ta||val('tt')!==saved.tt||
    sw('gy')!==saved.gy||sw('hh')!==saved.hh;
}
function updDirty(){
  const d=dirtyCal();
  $('calDirty').textContent=d?'Unsaved changes':'Saved';
  $('calDirty').style.color=d?'var(--wn)':'';
}
function renderCal(d){
  saved={fwd:d.fwd,turn:d.turn,ta:d.ta,tt:d.tt,gy:d.gy,hh:d.hh};
  if(!calLoaded){
    $('fwd').value=d.fwd;$('turn').value=d.turn;$('ta').value=d.ta;$('tt').value=d.tt;
    setSw('gy',d.gy);setSw('hh',d.hh);calLoaded=true;
  }
  $('mpuState').textContent=d.mpu?(d.tlt?'Tilted':'Online'):'Not found';
  $('mpuState').style.color=d.mpu&&!d.tlt?'var(--ok)':'var(--wn)';
  const h=$('mpuHint');
  if(!d.mpu){h.textContent='MPU6050 not detected - turns use the timed value. Check SDA GPIO20, SCL GPIO21, VCC 3V3.';h.className='hint warn';}
  else if(!d.sk){h.textContent='Run one Test right turn so the rover learns the gyro direction.';h.className='hint warn';}
  else{h.textContent='Heading '+f1(d.hd)+'°  ·  tilt '+f1(d.tl)+'°. Keep the rover still while recalibrating.';h.className='hint';}
  document.querySelectorAll('.lock').forEach(e=>e.disabled=d.busy);
  updDirty();
}
['fwd','turn','ta','tt'].forEach(id=>$(id).addEventListener('input',updDirty));
['gy','hh'].forEach(id=>$(id).onclick=()=>{setSw(id,!sw(id));updDirty();});
document.querySelectorAll('.step').forEach(b=>b.onclick=()=>{
  const e=$('turn');
  e.value=Math.min(+e.max,Math.max(+e.min,(parseInt(e.value,10)||0)+(+b.dataset.d)));
  updDirty();
});
$('save').onclick=()=>{
  const f=val('fwd'),t=val('turn'),ta=val('ta'),tt=val('tt');
  if(isNaN(f)||isNaN(t)){toast('Forward 100-10000 ms, turn 50-3000 ms','err');return;}
  if(isNaN(ta)||isNaN(tt)){toast('Turn angle 10-360°, stop early 0-45°','err');return;}
  act('/save?fwd='+f+'&turn='+t+'&ta='+ta+'&tt='+tt+'&gy='+(sw('gy')?1:0)+'&hh='+(sw('hh')?1:0));
};
$('test').onclick=()=>{
  const t=val('turn');
  if(isNaN(t)){toast('Turn must be 50-3000 ms','err');return;}
  act('/test?turn='+t);
};
$('square').onclick=()=>{
  if(dirtyCal()&&!confirm('You have unsaved changes. Run the square with the saved values?'))return;
  act('/square');
};
$('mcal').onclick=()=>{toast('Calibrating - keep the rover still');act('/mpu/cal');};

// ---------- drive power ----------

// ---------- system ----------
function renderSys(d){
  if(lastUp>=0&&d.up<lastUp)reboots++;   // uptime went backwards = rover rebooted
  lastUp=d.up;
  $('yUp').textContent=hms(d.up);
  $('yRst').textContent=d.rst;
  $('yHeap').textContent=(d.heap/1024).toFixed(1)+' KB';
  $('yMpu').textContent=d.mpu?'MPU6050 online':'Not found';
  $('ySk').textContent=d.sk?'Learned':'Not learned yet';
  $('yLink').textContent=wsOpen()?'WebSocket':'HTTP fallback';
  const w=[];
  if(reboots)w.push('Rover rebooted '+reboots+'x while this page was open.');
  if(d.rst.indexOf('BROWNOUT')>=0)w.push('Last reset was a brownout - check the 5 V supply.');
  $('yWarn').textContent=w.join(' ');$('yWarn').className='hint'+(w.length?' warn':'');
}
function loadLog(){
  if(jOn)return;
  fetch('/log',{cache:'no-store'}).then(r=>r.text()).then(t=>{$('rlog').textContent=t.trim()||'(empty)';}).catch(()=>{});
}
setInterval(()=>{if(cur==='sys')loadLog();},2000);
let tx=0,rx=0,skip=0,htx=0;
setInterval(()=>{
  if(cur!=='sys')return;
  const st=ws?['connecting','open','closing','closed'][ws.readyState]:'none';
  $('dbg').textContent='websocket '+st+'  |  stick held '+jOn+'  x '+jx+'  y '+jy+
    '\npackets sent '+tx+'  replies '+rx+'  skipped '+skip+'  http '+htx+'  buffered '+(ws?ws.bufferedAmount:0);
},500);

// ---------- speed limit (remembered on this phone) ----------
let spd=60;
{const v=parseInt(store.get('spd'),10);if(v>=20&&v<=100)spd=v;}
$('spd').value=spd;$('spv').textContent=spd;
$('spd').oninput=()=>{spd=+$('spd').value;$('spv').textContent=spd;store.set('spd',spd);};

// ---------- WebSocket (port 81): joystick + status ----------
let ws=null;
function wsOpen(){return ws&&ws.readyState===1;}
function wsConnect(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=()=>reqStatus();
  ws.onmessage=e=>{
    const t=e.data;
    if(t.charAt(0)==='{'){try{onStatus(JSON.parse(t));}catch(_){}return;}
    rx++;lastSt=Date.now();             // joystick replies keep the link indicator alive
    if(!jOn)return;
    $('jout').textContent=t;
    const m=t.match(/L (-?\d+)\s+R (-?\d+)/);
    if(m)$('kMo').textContent=pct(+m[1])+' / '+pct(+m[2]);
  };
  ws.onclose=()=>{setTimeout(wsConnect,1000);};
  ws.onerror=()=>{try{ws.close();}catch(_){}};
}
wsConnect();

// ---------- joystick ----------
const joy=$('joy'),knob=$('knob');
let jx=0,jy=0,jOn=false,jBusy=false,pid=null;
function jSet(cx,cy){
  const r=joy.getBoundingClientRect(),R=r.width/2,m=R-knob.offsetWidth/2;
  let dx=cx-r.left-R,dy=cy-r.top-R;
  const d=Math.hypot(dx,dy);
  if(d>m){dx*=m/d;dy*=m/d;}
  knob.style.transform='translate(calc(-50% + '+dx+'px),calc(-50% + '+dy+'px))';
  jx=Math.round(dx/m*100);jy=Math.round(-dy/m*100);
}
function jSend(){
  if(!jOn)return;
  if(wsOpen()){
    if(ws.bufferedAmount===0){ws.send(jx+','+jy+','+spd);tx++;}
    else skip++;
    return;
  }
  if(jBusy)return;                         // fallback: HTTP while the socket is down
  jBusy=true;htx++;
  get('/joy?x='+jx+'&y='+jy+'&s='+spd)
    .then(d=>{lastSt=Date.now();if(jOn)$('jout').textContent=d.msg;})
    .catch(()=>{if(jOn)$('jout').textContent='No reply';})
    .finally(()=>{jBusy=false;});
}
function jStart(cx,cy){jOn=true;joy.classList.add('held');jSet(cx,cy);jSend();}
function jEnd(){
  if(!jOn)return;
  jOn=false;jx=jy=0;
  joy.classList.remove('held');
  knob.style.transform='translate(-50%,-50%)';
  $('jout').textContent='Released';
  if(wsOpen()){ws.send('0,0');return;}
  get('/joy?x=0&y=0').catch(()=>stop(1));
}
if(window.PointerEvent){
  joy.addEventListener('pointerdown',e=>{
    e.preventDefault();pid=e.pointerId;
    try{joy.setPointerCapture(pid);}catch(_){}
    jStart(e.clientX,e.clientY);
  });
  joy.addEventListener('pointermove',e=>{if(jOn&&e.pointerId===pid)jSet(e.clientX,e.clientY);});
  joy.addEventListener('pointerup',e=>{if(e.pointerId===pid)jEnd();});
  joy.addEventListener('pointercancel',e=>{if(e.pointerId===pid)jEnd();});
}else{
  joy.addEventListener('touchstart',e=>{e.preventDefault();const t=e.touches[0];jStart(t.clientX,t.clientY);},{passive:false});
  joy.addEventListener('touchmove',e=>{e.preventDefault();const t=e.touches[0];if(jOn)jSet(t.clientX,t.clientY);},{passive:false});
  joy.addEventListener('touchend',e=>{e.preventDefault();jEnd();},{passive:false});
  joy.addEventListener('touchcancel',jEnd);
}
document.addEventListener('visibilitychange',()=>{if(document.hidden)jEnd();});
window.addEventListener('blur',jEnd);
setInterval(jSend,100);
{const t=store.get('tab');if(t&&$('p-'+t))tab(t);}   // reopen the last tab
reqStatus();
</script>
</body></html>
)rawliteral";

// =====================================================================
//  Motor control
// =====================================================================
// speed: -255..255, positive = forward, 0 = brake
void driveChannel(uint8_t in1, uint8_t in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed == 0) {               // brake (both high)
    ledcWrite(in1, 255);
    ledcWrite(in2, 255);
    return;
  }
#if SLOW_DECAY
  if (speed > 0) { ledcWrite(in1, 255);         ledcWrite(in2, 255 - speed); }
  else           { ledcWrite(in1, 255 + speed); ledcWrite(in2, 255); }
#else
  if (speed > 0) { ledcWrite(in1, speed); ledcWrite(in2, 0); }
  else           { ledcWrite(in1, 0);     ledcWrite(in2, -speed); }
#endif
}

int pwmL = 0, pwmR = 0;             // last applied per side (for the dashboard)
void setLeft(int speed) {
  pwmL = constrain(speed, -255, 255);
  if (SIDES_SWAPPED) driveChannel(BIN1, BIN2, speed);
  else               driveChannel(AIN1, AIN2, speed);
}
void setRight(int speed) {
  pwmR = constrain(speed, -255, 255);
  int v = RIGHT_REVERSED ? -speed : speed;
  if (SIDES_SWAPPED) driveChannel(AIN1, AIN2, v);
  else               driveChannel(BIN1, BIN2, v);
}

void stopMotors()  { setLeft(0);             setRight(0); }
void forward()     { setLeft(DRIVE_SPEED);   setRight(DRIVE_SPEED); }
void backward()    { setLeft(-DRIVE_SPEED);  setRight(-DRIVE_SPEED); }
void rotateRight() { setLeft(TURN_SPEED);    setRight(-TURN_SPEED); }
void rotateLeft()  { setLeft(-TURN_SPEED);   setRight(TURN_SPEED); }

// =====================================================================
//  Servo (seed dispenser) - non-blocking, speed-limited
// =====================================================================
void logEvent(const char* msg);     // defined with the debug log below

void servoWrite(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  uint32_t us = SERVO_MIN_US + (uint32_t)((SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0f);
  ledcWrite(SERVO_PIN, us * ((1UL << SERVO_RES) - 1) / 20000UL);
}

void servoMoveTo(int deg, int speed) {
  servoTarget = constrain(deg, 0, 180);
  servoMoveSpeed = speed;
}

bool servoArrived() { return servoPos == servoTarget; }

void servoStartDispense(int closedA, int openA, int speed, unsigned long hold, int swings) {
  svRunClosed = closedA;
  svRunOpen   = openA;
  svRunSpeed  = speed;
  svRunHold   = hold;
  svLeft      = swings;
  svPhase     = SV_OPENING;
  servoMoveTo(svRunOpen, svRunSpeed);
}

void servoStop() {                  // halt where it is (also ends continuous dispense)
  contOn = false;
  svPhase = SV_IDLE;
  servoTarget = servoPos;
}

void updateServo() {
  unsigned long now = millis();
  float dt = (now - servoLastMs) / 1000.0f;
  servoLastMs = now;

  if (!servoArrived()) {
    if (servoMoveSpeed <= 0) {
      servoPos = servoTarget;
    } else {
      float step = servoMoveSpeed * dt;
      servoPos = servoPos < servoTarget ? min(servoPos + step, servoTarget)
                                        : max(servoPos - step, servoTarget);
    }
    servoWrite(servoPos);
  }

  switch (svPhase) {
    case SV_OPENING:
      if (servoArrived()) { svPhase = SV_HOLD; svHoldStart = now; }
      break;
    case SV_HOLD:
      if (now - svHoldStart >= svRunHold) { svPhase = SV_CLOSING; servoMoveTo(svRunClosed, svRunSpeed); }
      break;
    case SV_CLOSING:
      if (servoArrived()) {
        if (--svLeft > 0) { svPhase = SV_OPENING; servoMoveTo(svRunOpen, svRunSpeed); }
        else {
          svPhase = SV_IDLE;
          dispCount++;
          if (contOn) contNext = now + contInterval;   // wait, then dispense again
          else        logEvent("Dispense done");
        }
      }
      break;
    default:
      break;
  }

  // Continuous mode: start the next dispense when the gap has passed
  if (contOn && svPhase == SV_IDLE && (long)(now - contNext) >= 0) {
    servoStartDispense(servoClosed, servoOpen, contSpeed, servoHold, contSwings);
  }
}

// =====================================================================
//  State machine
// =====================================================================
// ---- Debug event log (ring buffer, shown on the page) ----
#define LOG_LINES 20
char logLines[LOG_LINES][48];
uint8_t logHead = 0, logCount = 0;

void logEvent(const char* msg) {
  unsigned long t = millis();
  snprintf(logLines[logHead], sizeof(logLines[0]), "%lu.%lu %s", t / 1000, (t % 1000) / 100, msg);
  Serial.println(logLines[logHead]);
  logHead = (logHead + 1) % LOG_LINES;
  if (logCount < LOG_LINES) logCount++;
}

void setStatus(const char* s) {
  strlcpy(statusText, s, sizeof(statusText));
  logEvent(statusText);
}

void stopSequence(const char* reason) {
  stopMotors();                   // motors first, always
  mode = MODE_IDLE;
  joyTgtL = joyTgtR = 0;
  joyCurL = joyCurR = 0;
  joyReleasing = false;
  setStatus(reason);
}

// ---------------- Joystick (proportional + ramped) ----------------
// Demand -1..1 -> PWM. Anything non-zero starts at JOY_MIN_PWM so the wheels actually move.
int demandToPwm(float v, int maxPwm) {
  if (fabsf(v) < 0.05f) return 0;
  int p = JOY_MIN_PWM + (int)((maxPwm - JOY_MIN_PWM) * fabsf(v));
  return v > 0 ? p : -p;
}

// Arcade mix: y = throttle, x = steering. x,y in -100..100, speedPct 20..100.
void joySetTarget(int x, int y, int speedPct) {
  joyStraight = abs(x) < 8 && abs(y) >= 20;   // no steering -> let the gyro hold heading
  if (JOY_SWAP_LR) x = -x;
  float fx = x / 100.0f, fy = y / 100.0f;
  float l = fy + fx, r = fy - fx;
  float m = max(fabsf(l), fabsf(r));
  if (m > 1.0f) { l /= m; r /= m; }
  // Speed slider 20..100 % spans the usable range: 20 % = slowest crawl, 100 % = JOY_MAX_PWM
  int maxPwm = JOY_MIN_PWM + (JOY_MAX_PWM - JOY_MIN_PWM) * (constrain(speedPct, 20, 100) - 20) / 80;
  joyTgtL = demandToPwm(l, maxPwm);
  joyTgtR = demandToPwm(r, maxPwm);
}

// One ramp step toward the target (dt in seconds).
//  - Starting from standstill: begin at JOY_START_PWM and speed up gently at JOY_ACCEL, so the
//    stalled motors never get a sudden large step (that trips the driver's over-current limit).
//  - Slowing down / stopping: JOY_RAMP; below JOY_MIN_PWM it drops straight to 0 (brake) so the
//    motors don't sit humming.
//  - Reversing: slow to 0 first, then soft-start the other way.
float rampSide(float cur, int tgt, float dt) {
  float down = JOY_RAMP * dt;
  if (tgt == 0 || (cur != 0 && (cur > 0) != (tgt > 0))) {     // stop, or reverse via 0
    float n = cur > 0 ? max(cur - down, 0.0f) : min(cur + down, 0.0f);
    return fabsf(n) < JOY_MIN_PWM ? 0 : n;
  }
  float a = fabsf(cur), t = abs(tgt), sgn = tgt > 0 ? 1.0f : -1.0f;
  if (a < t) a = (a == 0) ? min((float)JOY_START_PWM, t) : min(a + JOY_ACCEL * dt, t);
  else       a = max(a - down, t);
  return sgn * a;
}

// =====================================================================
//  MPU6050 - raw I2C registers, no library
// =====================================================================
bool  mpuOk = false;
uint8_t mpuFails = 0;
float gyroBiasZ = 0;                // deg/s drift measured while still
float yawRaw = 0;                   // integrated gyro Z, deg (sign depends on mounting)
int8_t yawSign = -1;                // heading() = yawSign * yawRaw, + = turned right
bool  signKnown = false;            // learned from the first right turn, then saved
float rollF = 0, pitchF = 0, rollRef = 0, pitchRef = 0, tiltDeg = 0;
unsigned long mpuLastUs = 0;
bool  tiltLatched = false;
unsigned long tiltOkSince = 0;

// Turn / hold-straight state
bool  turnByGyro = false;
float turnStartRaw = 0;
bool  holdActive = false;
float holdTarget = 0;
bool  measurePending = false;       // report the real angle shortly after a test turn
unsigned long measureAt = 0;

float heading() { return yawSign * yawRaw; }
float angleDiff(float a, float b) { return fmodf(a - b + 540.0f, 360.0f) - 180.0f; }

bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuRead(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// gz in deg/s (bias not removed), roll/pitch in deg from the accelerometer
bool mpuSample(float& gz, float& roll, float& pitch) {
  uint8_t b[14];
  if (!mpuRead(0x3B, b, 14)) return false;
  int16_t ax = (b[0] << 8) | b[1], ay = (b[2] << 8) | b[3], az = (b[4] << 8) | b[5];
  int16_t rgz = (b[12] << 8) | b[13];
  gz    = rgz / GYRO_LSB;
  roll  = atan2f(ay, az) * RAD_TO_DEG;
  pitch = atan2f(-ax, sqrtf((float)ay * ay + (float)az * az)) * RAD_TO_DEG;
  return true;
}

// Rover must be STILL: measures gyro drift and the level reference (~1 s, blocking)
bool mpuCalibrate() {
  stopMotors();
  float sum = 0, r = 0, p = 0;
  int n = 0;
  for (int i = 0; i < 400; i++) {
    float gz, ro, pi;
    if (mpuSample(gz, ro, pi)) { sum += gz; r += ro; p += pi; n++; }
    delay(2);
  }
  if (n < 200) return false;
  gyroBiasZ = sum / n;
  rollRef  = rollF  = r / n;
  pitchRef = pitchF = p / n;
  yawRaw = 0;
  tiltDeg = 0;
  mpuLastUs = micros();
  return true;
}

bool mpuBegin() {
  Wire.begin(MPU_SDA, MPU_SCL, 400000);
  Wire.setTimeOut(5);
  uint8_t who = 0;
  if (!mpuRead(0x75, &who, 1)) return false;    // WHO_AM_I (clones may not return 0x68)
  Serial.printf("MPU WHO_AM_I = 0x%02X\n", who);
  mpuWrite(0x6B, 0x01);   // wake up, gyro X clock
  delay(50);
  mpuWrite(0x1A, 0x03);   // low-pass filter ~44 Hz
  mpuWrite(0x1B, 0x08);   // gyro +-500 deg/s
  mpuWrite(0x1C, 0x00);   // accel +-2 g
  delay(50);
  return mpuCalibrate();
}

void updateMpu() {
  if (!mpuOk) return;
  unsigned long nowUs = micros();
  if (nowUs - mpuLastUs < 5000) return;          // 200 Hz
  float gz, ro, pi;
  if (!mpuSample(gz, ro, pi)) {
    if (++mpuFails > 20) { mpuOk = false; logEvent("MPU lost - timed turns"); }
    return;
  }
  mpuFails = 0;
  float dt = (nowUs - mpuLastUs) / 1e6f;
  mpuLastUs = nowUs;
  gz -= gyroBiasZ;
  if (fabsf(gz) < 0.3f) gz = 0;                  // ignore tiny drift while still
  yawRaw += gz * dt;
  rollF  += 0.1f * angleDiff(ro, rollF);         // smoothed tilt
  pitchF += 0.1f * angleDiff(pi, pitchF);
  float dr = angleDiff(rollF, rollRef), dp = angleDiff(pitchF, pitchRef);
  tiltDeg = sqrtf(dr * dr + dp * dp);
}

// delta = yawRaw change during a RIGHT turn -> which gyro direction means "right"
void learnYawSign(float delta) {
  if (fabsf(delta) < 30) return;
  int8_t s = delta > 0 ? 1 : -1;
  if (signKnown && s == yawSign) return;
  yawSign = s;
  signKnown = true;
  prefs.begin("rover", false);
  prefs.putChar("ys", s);
  prefs.putBool("sk", true);
  prefs.end();
  logEvent("Gyro direction learned");
}

// PWM correction to steer back to target heading (+ = steer right)
int holdCorrection(float target) {
  return constrain((int)(HOLD_KP * (target - heading())), -HOLD_MAX, HOLD_MAX);
}

bool canHold() { return mpuOk && holdOn && signKnown; }

// Square forward legs: keep the start heading
void updateSquareHold() {
  static unsigned long last = 0;
  if (!(mode == MODE_SQUARE && phase == PH_FORWARD && holdActive)) return;
  if (millis() - last < 20) return;
  last = millis();
  int c = holdCorrection(holdTarget);
  setLeft(DRIVE_SPEED + c);
  setRight(DRIVE_SPEED - c);
}

// Tilt safety: stop everything past TILT_LIMIT, re-enable after 1 s back near level
void checkTilt() {
  if (!mpuOk) return;
  if (!tiltLatched && tiltDeg > TILT_LIMIT) {
    tiltLatched = true;
    tiltOkSince = 0;
    servoStop();
    stopSequence("TILT - stopped");
    return;
  }
  if (tiltLatched) {
    if (tiltDeg < TILT_CLEAR) {
      if (tiltOkSince == 0) tiltOkSince = millis();
      if (millis() - tiltOkSince > 1000) {
        tiltLatched = false;
        tiltOkSince = 0;
        setStatus("Level again - Ready");
      }
    } else {
      tiltOkSince = 0;
    }
  }
}

// After a test turn: wait for coasting to finish, then show the real angle
void updateMeasure() {
  if (!measurePending || (long)(millis() - measureAt) < 0) return;
  measurePending = false;
  if (mode != MODE_IDLE || !mpuOk) return;
  float delta = yawRaw - turnStartRaw;
  learnYawSign(delta);
  char b[40];
  snprintf(b, sizeof(b), "Turned %.1f deg (%s)", fabsf(delta), turnByGyro ? "gyro" : "timed");
  setStatus(b);
}

void enterPhase(Phase p) {
  char buf[40];
  phase = p;
  phaseStart = millis();
  switch (p) {
    case PH_FORWARD:
      phaseDuration = forwardTime;
      holdActive = canHold();          // gyro keeps this leg straight
      holdTarget = heading();
      forward();
      snprintf(buf, sizeof(buf), "Forward - Side %u", side);
      break;
    case PH_TURN:
      turnByGyro = mpuOk && useGyro;   // stop on angle, else on time
      turnStartRaw = yawRaw;
      phaseDuration = turnByGyro ? TURN_TIMEOUT : turnTime;
      rotateRight();
      snprintf(buf, sizeof(buf), "Turning right - Side %u%s", side, turnByGyro ? " (gyro)" : "");
      break;
    case PH_PAUSE_A:
    case PH_PAUSE_B:
      phaseDuration = SETTLE_TIME;
      stopMotors();
      snprintf(buf, sizeof(buf), "Pause - Side %u", side);
      break;
  }
  setStatus(buf);
}

void startTest(unsigned long ms) {
  mode = MODE_TEST;
  measurePending = false;
  turnByGyro = mpuOk && useGyro;
  turnStartRaw = yawRaw;
  phaseStart = millis();
  phaseDuration = turnByGyro ? TURN_TIMEOUT : ms;
  lastClientMs = millis();
  rotateRight();
  setStatus(turnByGyro ? "Testing right turn (gyro)" : "Testing right turn");
}

void startSquare() {
  mode = MODE_SQUARE;
  side = 1;
  lastClientMs = millis();
  enterPhase(PH_FORWARD);
}

void updateSequence() {
  if (mode != MODE_TEST && mode != MODE_SQUARE) return;   // never touch joystick/idle
  unsigned long elapsed = millis() - phaseStart;
  bool done = elapsed >= phaseDuration;

  // Gyro turn: finish when the angle is reached (trim allows for coasting)
  bool turning = (mode == MODE_TEST) || (phase == PH_TURN);
  if (turning && turnByGyro) {
    if (mpuOk && fabsf(yawRaw - turnStartRaw) >= turnAngle - turnTrim) done = true;
    else if (!mpuOk && elapsed >= turnTime) done = true;           // MPU lost mid-turn
    else if (done) logEvent("Gyro turn timeout");
  }
  if (!done) return;

  if (mode == MODE_TEST) {
    stopSequence("Turn test complete");
    measurePending = mpuOk;          // report the real angle once it stops coasting
    measureAt = millis() + 400;
    return;
  }

  switch (phase) {
    case PH_FORWARD: enterPhase(PH_PAUSE_A); break;
    case PH_PAUSE_A: enterPhase(PH_TURN);    break;
    case PH_TURN:    enterPhase(PH_PAUSE_B); break;
    case PH_PAUSE_B:
      if (mpuOk) {                   // log the real corner angle
        float delta = yawRaw - turnStartRaw;
        learnYawSign(delta);
        char b[40];
        snprintf(b, sizeof(b), "Corner %u: %.1f deg", side, fabsf(delta));
        logEvent(b);
      }
      if (side >= 4) {
        stopSequence("Square completed");
      } else {
        side++;
        enterPhase(PH_FORWARD);
      }
      break;
  }
}

// Ramp motor PWM toward the joystick targets (called every loop)
void updateJoy() {
  if (mode != MODE_JOY) return;
  unsigned long now = millis();
  if (now == joyLastRamp) return;
  float dt = (now - joyLastRamp) / 1000.0f;
  joyLastRamp = now;

  // Straight ahead/back: gyro trims the two sides so the rover doesn't drift
  int tl = joyTgtL, tr = joyTgtR;
  if (joyStraight && canHold() && tl != 0) {
    if (!joyHoldActive) { joyHoldActive = true; joyHoldTarget = heading(); }
    int c = holdCorrection(joyHoldTarget);
    tl = constrain(tl + c, -255, 255);
    tr = constrain(tr - c, -255, 255);
  } else {
    joyHoldActive = false;
  }

  float l = rampSide(joyCurL, tl, dt);
  float r = rampSide(joyCurR, tr, dt);
  int ol = (int)l, orr = (int)r;
  if (ol != pwmL) setLeft(ol);       // compare with what the motor actually has
  if (orr != pwmR) setRight(orr);
  joyCurL = l;
  joyCurR = r;

  if (joyReleasing && l == 0 && r == 0) stopSequence("Joystick released");
}

// Failsafe: stop if the phone disconnects or stops polling while moving
void checkLink() {
  // Continuous dispense needs the phone too: never keep dropping seeds unattended
  if (contOn && millis() - lastClientMs > LINK_TIMEOUT) {
    contOn = false;
    logEvent("Continuous dispense off: connection lost");
  }

  if (mode == MODE_IDLE) return;

  if (mode == MODE_JOY && millis() - lastJoyMs > JOY_TIMEOUT) {
    stopSequence("Connection lost");
    return;
  }

  if (millis() - lastClientMs > LINK_TIMEOUT) {
    stopSequence("Connection lost");
    return;
  }
  if (millis() - lastStaCheck >= 200) {
    lastStaCheck = millis();
    if (WiFi.softAPgetStationNum() == 0) stopSequence("Connection lost");
  }
}

// =====================================================================
//  Web handlers (all return immediately)
// =====================================================================
void sendResult(int code, bool ok, const char* msg) {
  char buf[128];
  snprintf(buf, sizeof(buf), "{\"ok\":%s,\"msg\":\"%s\"}", ok ? "true" : "false", msg);
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", buf);
}

bool rejectIfBusy() {
  if (mode != MODE_IDLE) {
    sendResult(409, false, "Rover busy - press STOP first");
    return true;
  }
  return false;
}

bool rejectIfTilted() {
  if (tiltLatched) {
    sendResult(409, false, "Rover tilted - set it level");
    return true;
  }
  return false;
}

void handleRoot() {
  lastClientMs = millis();
  server.send(200, "text/html", INDEX_HTML);
}

// Status JSON - sent over HTTP (/status) and over the WebSocket (reply to "s")
void buildStatus(char* buf, size_t n) {
  snprintf(buf, n,
           "{\"status\":\"%s\",\"busy\":%s,\"md\":%d,\"fwd\":%lu,\"turn\":%lu,\"up\":%lu,\"rst\":\"%s\",\"heap\":%lu,"
           "\"sv\":%d,\"svb\":%s,\"sc\":%d,\"so\":%d,\"ss\":%d,\"sh\":%lu,\"sn\":%d,\"co\":%s,\"dc\":%lu,"
           "\"ml\":%d,\"mr\":%d,"
           "\"mpu\":%s,\"hd\":%.1f,\"tl\":%.1f,\"tlt\":%s,\"gy\":%s,\"ta\":%.0f,\"tt\":%.0f,\"hh\":%s,\"sk\":%s}",
           statusText, mode != MODE_IDLE ? "true" : "false", (int)mode, forwardTime, turnTime,
           millis() / 1000, resetReason, (unsigned long)ESP.getFreeHeap(),
           (int)(servoPos + 0.5f), svPhase != SV_IDLE ? "true" : "false",
           servoClosed, servoOpen, servoSpeed, servoHold, servoSwings, contOn ? "true" : "false", dispCount,
           pwmL, pwmR,
           mpuOk ? "true" : "false", angleDiff(heading(), 0), tiltDeg, tiltLatched ? "true" : "false",
           useGyro ? "true" : "false", turnAngle, turnTrim, holdOn ? "true" : "false",
           signKnown ? "true" : "false");
}

void handleStatus() {
  lastClientMs = millis();
  char buf[720];
  buildStatus(buf, sizeof(buf));
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

void handleStop() {
  servoStop();
  stopSequence("Stopped");        // stop before replying
  lastClientMs = millis();
  sendResult(200, true, "Stopped");
}

// Shared by the WebSocket and the HTTP fallback. Returns "L <pwm> R <pwm>" (targets).
const char* joyCommand(int x, int y, int speedPct) {
  static char reply[32];
  lastClientMs = millis();
  x = constrain(x, -100, 100);
  y = constrain(y, -100, 100);
  joySpeedPct = constrain(speedPct, 20, 100);

  if (x * x + y * y < JOY_DEADZONE * JOY_DEADZONE) {   // released / centred: ramp to a stop
    if (mode == MODE_JOY) {
      joyTgtL = joyTgtR = 0;
      joyReleasing = true;
    }
    return "Stop";
  }

  if (tiltLatched) return "TILTED - level the rover";

  lastJoyMs = millis();
  if (mode != MODE_JOY) {         // joystick takes over (cancels test/square)
    stopMotors();
    mode = MODE_JOY;
    joyCurL = joyCurR = 0;
    joyLastRamp = millis();
    setStatus("Joystick driving");
  }
  joyReleasing = false;
  joySetTarget(x, y, joySpeedPct);
  snprintf(reply, sizeof(reply), "L %d  R %d", joyTgtL, joyTgtR);
  return reply;
}

void handleJoy() {
  int x = server.arg("x").toInt(), y = server.arg("y").toInt();
  int s = server.hasArg("s") ? server.arg("s").toInt() : joySpeedPct;
  if (x || y) {
    char b[32];
    snprintf(b, sizeof(b), "HTTP joy %d,%d", x, y);
    logEvent(b);
  }
  sendResult(200, true, joyCommand(x, y, s));
}

// ---------------- Servo handlers ----------------
int argOr(const char* name, long def) { return server.hasArg(name) ? server.arg(name).toInt() : def; }

// Reads sp,h,n (defaults = saved values). Angles are locked and never read from requests.
// Sends 400 and returns false if out of range.
bool readServoArgs(int& sp, unsigned long& h, int& n) {
  sp = argOr("sp", servoSpeed);
  long hh = argOr("h", servoHold);
  n  = argOr("n",  servoSwings);
  if (sp < 0 || sp > SERVO_MAX_SPD || hh < 0 || hh > 5000 || n < 1 || n > 20) {
    sendResult(400, false, "Speed 0-600, hold 0-5000, swings 1-20");
    return false;
  }
  h = (unsigned long)hh;
  return true;
}

void handleServoTest() {            // one dispense with the values on screen (not saved)
  lastClientMs = millis();
  if (rejectIfTilted()) return;
  if (svPhase != SV_IDLE) { sendResult(409, false, "Already dispensing"); return; }
  int sp, n; unsigned long h;
  if (!readServoArgs(sp, h, n)) return;
  servoStartDispense(servoClosed, servoOpen, sp, h, n);
  logEvent("Test dispense");
  sendResult(200, true, "Test dispense");
}

void handleServoSave() {
  lastClientMs = millis();
  if (svPhase != SV_IDLE) { sendResult(409, false, "Dispensing - press STOP"); return; }
  int sp, n; unsigned long h;
  if (!readServoArgs(sp, h, n)) return;
  servoSpeed = sp; servoHold = h; servoSwings = n;
  prefs.begin("servo", false);
  prefs.putInt("sp", sp);
  prefs.putULong("h", h);
  prefs.putInt("n", n);
  prefs.end();
  logEvent("Dispense settings saved");
  sendResult(200, true, "Dispense settings saved");
}

// ---------------- Dispense while driving (uses saved closed/open angles + hold) ----------------
// /disp?sp=&n=            one dispense
// /cont?on=1&sp=&n=&i=    continuous on/off (i = ms between dispenses)
bool readDispArgs(int& sp, int& n) {
  sp = argOr("sp", servoSpeed);
  n  = argOr("n",  servoSwings);
  if (sp < 0 || sp > SERVO_MAX_SPD || n < 1 || n > 20) {
    sendResult(400, false, "Speed 0-600, cycles 1-20");
    return false;
  }
  return true;
}

void handleDispense() {
  lastClientMs = millis();
  if (rejectIfTilted()) return;
  if (svPhase != SV_IDLE) { sendResult(409, false, "Already dispensing"); return; }
  int sp, n;
  if (!readDispArgs(sp, n)) return;
  servoStartDispense(servoClosed, servoOpen, sp, servoHold, n);
  sendResult(200, true, "Dispensing");
}

void handleContinuous() {
  lastClientMs = millis();
  if (argOr("on", 0) == 0) {
    contOn = false;
    logEvent("Continuous dispense off");
    sendResult(200, true, "Continuous OFF");
    return;
  }
  if (rejectIfTilted()) return;
  int sp, n;
  if (!readDispArgs(sp, n)) return;
  long i = argOr("i", contInterval);
  if (i < 100 || i > 10000) { sendResult(400, false, "Interval 100-10000 ms"); return; }
  bool wasOn = contOn;
  contSpeed = sp; contSwings = n; contInterval = (unsigned long)i;
  if (!wasOn) {
    contOn = true;
    contNext = millis();          // first dispense right away
    logEvent("Continuous dispense on");
  }
  sendResult(200, true, "Continuous ON");
}

void handleLog() {
  String out;
  out.reserve(LOG_LINES * 50);
  for (uint8_t i = 0; i < logCount; i++) {
    uint8_t idx = (logHead + LOG_LINES - logCount + i) % LOG_LINES;
    out += logLines[idx];
    out += '\n';
  }
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", out);
}

// WebSocket (port 81): phone sends "x,y" text every 100 ms while the stick is held
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case WStype_DISCONNECTED: {
      char b[32];
      snprintf(b, sizeof(b), "WS #%u disconnected", num);
      logEvent(b);
      if (mode == MODE_JOY) stopSequence("Connection lost");
      break;
    }
    case WStype_CONNECTED: {
      char b[32];
      snprintf(b, sizeof(b), "WS #%u connected", num);
      logEvent(b);
      break;
    }
    case WStype_TEXT: {
      char buf[24];
      size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
      memcpy(buf, payload, n);
      buf[n] = '\0';
      if (buf[0] == 's') {                         // status request (also keeps the link alive)
        lastClientMs = millis();
        char sb[720];
        buildStatus(sb, sizeof(sb));
        ws.sendTXT(num, sb);
        break;
      }
      int x = 0, y = 0, s = joySpeedPct;           // "x,y,speed" (speed optional)
      if (sscanf(buf, "%d,%d,%d", &x, &y, &s) >= 2) ws.sendTXT(num, joyCommand(x, y, s));
      break;
    }
    default:
      break;
  }
}

void handleTest() {
  lastClientMs = millis();
  if (rejectIfBusy() || rejectIfTilted()) return;

  unsigned long ms = turnTime;    // default: saved value
  if (server.hasArg("turn")) {
    long v = server.arg("turn").toInt();
    if (v < TURN_MIN || v > TURN_MAX) {
      sendResult(400, false, "Turn must be 50-3000 ms");
      return;
    }
    ms = (unsigned long)v;        // test value only, not saved
  }
  startTest(ms);

  char buf[64];
  snprintf(buf, sizeof(buf), "Right turn test: %lu ms", ms);
  sendResult(200, true, buf);
}

void handleSquare() {
  lastClientMs = millis();
  if (rejectIfBusy() || rejectIfTilted()) return;
  startSquare();

  char buf[80];
  snprintf(buf, sizeof(buf), "Square started: forward %lu ms, turn %lu ms", forwardTime, turnTime);
  sendResult(200, true, buf);
}

void handleSave() {
  lastClientMs = millis();
  if (rejectIfBusy()) return;

  if (!server.hasArg("fwd") || !server.hasArg("turn")) {
    sendResult(400, false, "Missing values");
    return;
  }
  long f = server.arg("fwd").toInt();
  long t = server.arg("turn").toInt();
  if (f < FWD_MIN || f > FWD_MAX || t < TURN_MIN || t > TURN_MAX) {
    sendResult(400, false, "Forward 100-10000 ms, turn 50-3000 ms");
    return;
  }

  // Optional gyro settings
  long ta = argOr("ta", (long)turnAngle), tt = argOr("tt", (long)turnTrim);
  if (ta < 10 || ta > 360 || tt < 0 || tt > 45) {
    sendResult(400, false, "Turn angle 10-360, stop early 0-45");
    return;
  }

  forwardTime = (unsigned long)f;
  turnTime    = (unsigned long)t;
  turnAngle   = ta;
  turnTrim    = tt;
  if (server.hasArg("gy")) useGyro = argOr("gy", 1) != 0;
  if (server.hasArg("hh")) holdOn  = argOr("hh", 1) != 0;

  prefs.begin("rover", false);
  prefs.putULong("fwd", forwardTime);
  prefs.putULong("turn", turnTime);
  prefs.putFloat("ta", turnAngle);
  prefs.putFloat("tt", turnTrim);
  prefs.putBool("gy", useGyro);
  prefs.putBool("hh", holdOn);
  prefs.end();

  setStatus("Settings saved");
  char buf[64];
  snprintf(buf, sizeof(buf), "Saved: forward %lu ms, turn %lu ms", forwardTime, turnTime);
  sendResult(200, true, buf);
}

// Re-measure gyro drift + level (rover must be still). Also retries a missing MPU.
void handleMpuCal() {
  lastClientMs = millis();
  if (rejectIfBusy()) return;
  bool ok = mpuOk ? mpuCalibrate() : mpuBegin();
  mpuOk = ok;
  mpuFails = 0;
  logEvent(ok ? "Gyro calibrated" : "MPU not found");
  sendResult(ok ? 200 : 500, ok, ok ? "Gyro calibrated, heading = 0" : "MPU6050 not found - check wiring");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// =====================================================================
//  Setup / loop
// =====================================================================
void loadSettings() {
  prefs.begin("rover", false);
  unsigned long f = prefs.getULong("fwd", 2000);
  unsigned long t = prefs.getULong("turn", 480);   // 480 ms = calibrated 90 deg
  prefs.end();
  forwardTime = (f >= FWD_MIN && f <= FWD_MAX) ? f : 2000;
  turnTime    = (t >= TURN_MIN && t <= TURN_MAX) ? t : 480;

  prefs.begin("rover", true);
  turnAngle = constrain(prefs.getFloat("ta", 90), 10.0f, 360.0f);
  turnTrim  = constrain(prefs.getFloat("tt", 8), 0.0f, 45.0f);
  useGyro   = prefs.getBool("gy", true);
  holdOn    = prefs.getBool("hh", true);
  for (const char* k : {"jp", "jx", "jm", "jk"}) if (prefs.isKey(k)) prefs.remove(k);   // old drive-power settings (now fixed)
  signKnown = prefs.getBool("sk", false);
  yawSign   = prefs.getChar("ys", -1) > 0 ? 1 : -1;
  prefs.end();
}

const char* readResetReason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "Power on";
    case ESP_RST_SW:       return "Software / OTA";
    case ESP_RST_BROWNOUT: return "BROWNOUT (power dip)";
    case ESP_RST_PANIC:    return "Crash";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "Watchdog";
    case ESP_RST_USB:      return "USB";
    default:               return "Other";
  }
}

void setup() {
  Serial.begin(115200);
  resetReason = readResetReason();

  // Driver disabled until PWM pins are in a known (brake) state
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);

  const uint8_t pins[] = { AIN1, AIN2, BIN1, BIN2 };
  for (uint8_t p : pins) {
    if (!ledcAttach(p, PWM_FREQ, PWM_RES)) {
      Serial.printf("ledcAttach failed on GPIO%u\n", p);
    }
  }
  stopMotors();
  digitalWrite(STBY, HIGH);

  loadSettings();
  Serial.printf("Settings: forward %lu ms, turn %lu ms\n", forwardTime, turnTime);

  // Servo: load saved timing (angles are locked constants) and park at the closed position
  prefs.begin("servo", false);
  servoSpeed  = constrain(prefs.getInt("sp", 200), 0, SERVO_MAX_SPD);
  servoHold   = min((unsigned long)prefs.getULong("h", 100), 5000UL);
  servoSwings = constrain(prefs.getInt("n", 1), 1, 20);
  prefs.end();
  if (!ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES)) Serial.println("ledcAttach failed on servo pin");
  servoPos = servoTarget = servoClosed;
  servoWrite(servoPos);
  servoLastMs = millis();

  // MPU6050: keep the rover STILL for ~1 s here (gyro drift + level calibration).
  // Wait first so the servo's jump to "closed" doesn't shake the calibration.
  delay(700);
  mpuOk = mpuBegin();
  Serial.println(mpuOk ? "MPU6050 ready" : "MPU6050 not found - timed turns only");

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(ssid, password, 1, 0, 1);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);   // after softAP so it is not reset
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/",       handleRoot);
  server.on("/status", handleStatus);
  server.on("/stop",   handleStop);
  server.on("/joy",    handleJoy);
  server.on("/log",    handleLog);
  server.on("/servo/test", handleServoTest);
  server.on("/servo/save", handleServoSave);
  server.on("/disp",       handleDispense);
  server.on("/cont",       handleContinuous);
  server.on("/test",   handleTest);
  server.on("/square", handleSquare);
  server.on("/save",   handleSave);
  server.on("/mpu/cal", handleMpuCal);
  server.onNotFound(handleNotFound);
  server.begin();

  ws.begin();
  ws.onEvent(onWsEvent);
  ws.enableHeartbeat(1000, 600, 2);   // drop dead phone connections -> DISCONNECTED -> stop

  // ---- OTA: upload new code over Wi-Fi (port "c3-rover at 192.168.4.1") ----
  ArduinoOTA.setHostname("c3-rover");
  ArduinoOTA.setPassword("12345678");
  ArduinoOTA.onStart([] { stopSequence("Updating firmware"); });
  ArduinoOTA.onEnd([] { Serial.println("OTA done, rebooting"); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA error %u\n", e); });
  ArduinoOTA.begin();

  setStatus("Ready");
}

void loop() {
  ArduinoOTA.handle();
  ws.loop();
  server.handleClient();
  updateMpu();
  checkTilt();
  updateSequence();
  updateSquareHold();
  updateMeasure();
  updateJoy();
  updateServo();
  checkLink();
}
