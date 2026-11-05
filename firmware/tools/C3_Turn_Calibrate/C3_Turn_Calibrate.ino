/*
  ESP32-C3 Super Mini + DRV8833 - simple 90 degree turn calibration
  Connect phone to Wi-Fi "C3-ROVER" (12345678) and open http://192.168.4.1

  GPIO3 -> AIN1, GPIO4 -> AIN2 (left)   GPIO5 -> BIN1, GPIO6 -> BIN2 (right)
  GPIO7 -> STBY, GND common
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h>

#define AIN1 3
#define AIN2 4
#define BIN1 5
#define BIN2 6
#define STBY 7

#define PWM_FREQ 1000     // 1 kHz: much more torque from BO motors than 20 kHz
#define PWM_RES  8
#define TURN_SPEED 170
#define RIGHT_REVERSED true   // flip if the right side spins the wrong way

#define RAMP_START 60         // soft start: PWM at the beginning of a turn
#define RAMP_TIME  100        // ms to ramp from RAMP_START up to TURN_SPEED

const char* ssid     = "C3-ROVER";
const char* password = "12345678";

WebServer server(80);
Preferences prefs;

unsigned long leftTime  = 500;   // ms for 90 degrees left
unsigned long rightTime = 500;   // ms for 90 degrees right

bool turning = false;
bool turnIsLeft = false;
int  turnPwm = 0;             // PWM currently applied during the ramp
unsigned long turnStart = 0, turnDuration = 0;
const char* status = "Ready";

// ---------------- Motors ----------------
// Slow decay: one input held HIGH, the other PWM'd inverted (drive <-> brake).
// Keeps motor current flowing between pulses -> more torque at the same PWM.
void driveChannel(uint8_t in1, uint8_t in2, int speed) {
  if (speed > 0)      { ledcWrite(in1, 255);         ledcWrite(in2, 255 - speed); }
  else if (speed < 0) { ledcWrite(in1, 255 + speed); ledcWrite(in2, 255); }
  else                { ledcWrite(in1, 255);         ledcWrite(in2, 255); }  // brake
}
void setLeft(int s)  { driveChannel(AIN1, AIN2, s); }
void setRight(int s) { driveChannel(BIN1, BIN2, RIGHT_REVERSED ? -s : s); }

void stopMotors()  { setLeft(0);           setRight(0); }
void rotateLeft()  { setLeft(-TURN_SPEED); setRight(TURN_SPEED); }
void rotateRight() { setLeft(TURN_SPEED);  setRight(-TURN_SPEED); }

// Pivot at a given PWM (used by the soft-start ramp)
void applyTurn(int pwm) {
  if (pwm == turnPwm) return;
  turnPwm = pwm;
  if (turnIsLeft) { setLeft(-pwm); setRight(pwm); }
  else            { setLeft(pwm);  setRight(-pwm); }
}

// ---------------- Web page ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Turn Calibration</title>
<style>
body{font-family:sans-serif;background:#121212;color:#eee;max-width:420px;margin:auto;padding:16px}
h2{text-align:center}
#st{font-size:22px;font-weight:bold;text-align:center;padding:14px;border-radius:10px;background:#263238}
label{display:block;margin:14px 0 6px;color:#aaa}
input{width:100%;box-sizing:border-box;font-size:22px;padding:10px;text-align:center;border-radius:8px;border:1px solid #444;background:#000;color:#fff}
button{width:100%;font-size:20px;font-weight:bold;padding:16px;margin-top:10px;border:0;border-radius:10px;color:#fff}
.l{background:#1565c0}.r{background:#ef6c00}.s{background:#455a64}.x{background:#d32f2f;font-size:24px;padding:22px}
</style></head><body>
<h2>90&deg; Turn Calibration</h2>
<div id="st">...</div>
<label>Left turn time (ms)</label><input id="lt" type="number" min="50" max="3000">
<button class="l" onclick="go('/left?ms='+lt.value)">TEST LEFT TURN</button>
<label>Right turn time (ms)</label><input id="rt" type="number" min="50" max="3000">
<button class="r" onclick="go('/right?ms='+rt.value)">TEST RIGHT TURN</button>
<button class="s" onclick="go('/save?l='+lt.value+'&r='+rt.value)">SAVE BOTH</button>
<button class="x" onclick="go('/stop')">STOP</button>
<script>
let first=true;
function go(p){fetch(p).then(r=>r.text()).then(t=>st.textContent=t).catch(()=>st.textContent='No response');}
function poll(){fetch('/status').then(r=>r.json()).then(d=>{
  st.textContent=d.status;
  if(first){lt.value=d.l;rt.value=d.r;first=false;}
}).catch(()=>st.textContent='Connection lost');}
setInterval(poll,400);poll();
</script></body></html>
)rawliteral";

// ---------------- Helpers ----------------
bool readMs(const char* arg, unsigned long& out) {
  if (!server.hasArg(arg)) return false;
  long v = server.arg(arg).toInt();
  if (v < 50 || v > 3000) return false;
  out = v;
  return true;
}

void startTurn(bool left, unsigned long ms) {
  turnDuration = ms;
  turnStart = millis();
  turning = true;
  turnIsLeft = left;
  turnPwm = 0;
  applyTurn(RAMP_START);
  status = left ? "Turning left" : "Turning right";
}

// ---------------- Handlers ----------------
void handleTurn(bool left) {
  if (turning) { server.send(409, "text/plain", "Busy - press STOP"); return; }
  unsigned long ms;
  if (!readMs("ms", ms)) { server.send(400, "text/plain", "Time must be 50-3000 ms"); return; }
  startTurn(left, ms);
  server.send(200, "text/plain", left ? "Turning left" : "Turning right");
}

void handleStop() {
  stopMotors();
  turning = false;
  status = "Stopped";
  server.send(200, "text/plain", "Stopped");
}

void handleSave() {
  if (turning) { server.send(409, "text/plain", "Busy - press STOP"); return; }
  unsigned long l, r;
  if (!readMs("l", l) || !readMs("r", r)) { server.send(400, "text/plain", "Time must be 50-3000 ms"); return; }
  leftTime = l;
  rightTime = r;
  prefs.begin("turncal", false);
  prefs.putULong("l", leftTime);
  prefs.putULong("r", rightTime);
  prefs.end();
  status = "Saved";
  server.send(200, "text/plain", "Saved");
}

void handleStatus() {
  char buf[96];
  snprintf(buf, sizeof(buf), "{\"status\":\"%s\",\"l\":%lu,\"r\":%lu}", status, leftTime, rightTime);
  server.send(200, "application/json", buf);
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);
  ledcAttach(AIN1, PWM_FREQ, PWM_RES);
  ledcAttach(AIN2, PWM_FREQ, PWM_RES);
  ledcAttach(BIN1, PWM_FREQ, PWM_RES);
  ledcAttach(BIN2, PWM_FREQ, PWM_RES);
  stopMotors();
  digitalWrite(STBY, HIGH);

  prefs.begin("turncal", false);
  leftTime  = prefs.getULong("l", 500);
  rightTime = prefs.getULong("r", 500);
  prefs.end();

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(ssid, password, 1, 0, 1);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.println(WiFi.softAPIP());

  server.on("/",       [] { server.send(200, "text/html", PAGE); });
  server.on("/left",   [] { handleTurn(true); });
  server.on("/right",  [] { handleTurn(false); });
  server.on("/stop",   handleStop);
  server.on("/save",   handleSave);
  server.on("/status", handleStatus);
  server.begin();

  // ---- OTA: upload new code over Wi-Fi (port "c3-rover at 192.168.4.1") ----
  ArduinoOTA.setHostname("c3-rover");
  ArduinoOTA.setPassword("12345678");
  ArduinoOTA.onStart([] {
    stopMotors();                    // never drive while flashing
    turning = false;
    status = "Updating firmware";
    Serial.println("OTA start");
  });
  ArduinoOTA.onEnd([] { Serial.println("OTA done, rebooting"); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA error %u\n", e); });
  ArduinoOTA.begin();
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();

  // Non-blocking turn timer with soft-start ramp
  if (turning) {
    unsigned long elapsed = millis() - turnStart;
    if (elapsed >= turnDuration) {
      stopMotors();
      turning = false;
      status = "Turn done - check angle";
    } else if (elapsed < RAMP_TIME) {
      applyTurn(RAMP_START + (TURN_SPEED - RAMP_START) * (long)elapsed / RAMP_TIME);
    } else {
      applyTurn(TURN_SPEED);
    }
  }
}
