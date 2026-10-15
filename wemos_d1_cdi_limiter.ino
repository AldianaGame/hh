/*
 * ==============================================================================
 *  FIRMWARE CDI RPM LIMITER & 30-STEP RUMBLE IDLE TUNER - WEMOS D1 MINI (ESP8266)
 *  Fitur Lengkap:
 *   1. Limiter CDI PWM (Soft-Cut & Hard-Cut) dengan Saklar Master ON/OFF & Histeresis
 *   2. Rumble Idle 30-Step Sequencer (Panel kotak-kotak kustom irama idle pincang)
 *   3. Pengatur Kecepatan / Tempo Rumble Idle (1x Cepat, 2x Sedang, 3x Lambat, 4x Helikopter)
 *   4. Saklar On/Off Independen untuk Setiap Fitur (Limiter, Backfire, Rumble, Launch, Shift)
 *   5. Web Panel Wi-Fi AP (Setel kotak on/off & tempo via Browser HP di 192.168.4.1)
 *   6. Memori EEPROM (Parameter & saklar tersimpan permanen saat kontak motor off)
 *   7. Overrun Backfire / Pops & Bangs (Letupan Knalpot saat Lepas Gas)
 *   8. 2-Step Launch Control & Shift Light Warning LED
 *  Board: Wemos D1 Mini / NodeMCU ESP8266
 * ==============================================================================
 *
 *  KONEKSI PIN WEMOS D1 MINI:
 *  - D2 (GPIO4): Input Sensor Pulser (WAJIB melalui Optocoupler PC817 + 1N4148)
 *  - D1 (GPIO5): Output PWM Pemutus Pulser (Ke PC817 Shunt / MOSFET IRLZ44N)
 *  - D6 (GPIO12): Tombol 2-Step Launch Control (Push button ke GND)
 *  - D4 (GPIO2): Output Indikator Shift Light LED
 *  - 5V & GND: Dari Buck Converter 12V Aki -> 5.0V DC Stabil
 *
 *  CARA KONEKSI KE WEB TUNER HP:
 *  1. Nyalakan motor / kontak on.
 *  2. Buka Wi-Fi di HP, sambungkan ke "WEMOS-CDI-LIMITER" (Password: "12345678").
 *  3. Buka browser Google Chrome / Safari, ketik: http://192.168.4.1
 *  4. Anda bisa klik kotak 30-step, mengatur tempo, dan mengubah saklar on/off fitur langsung dari HP!
 * ==============================================================================
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

ESP8266WebServer server(80);
const char* ap_ssid = "WEMOS-CDI-LIMITER";
const char* ap_pass = "12345678";

// ==================== DEFINISI PIN ====================
#define PIN_PULSER_IN      D2    // GPIO 4  - Input pulser sensor RPM
#define PIN_CUT_PWM        D1    // GPIO 5  - Output pemutus pulser PWM (Shunt)
#define PIN_LAUNCH_SW      D6    // GPIO 12 - Tombol 2-Step Launch Control (Active LOW)
#define PIN_SHIFT_LIGHT    D4    // GPIO 2  - Indikator Shift Light (LED)

// ==================== STRUKTUR DATA EEPROM ====================
#define EEPROM_MAGIC 0xCD17 // Pengenal validasi memori EEPROM versi 30-step + histeresis
struct TunerConfig {
  uint16_t magic;
  uint8_t  enableLimiter;      // 1: ON, 0: OFF (Bypass)
  uint16_t limiterHysteresis;  // Histeresis RPM (e.g. 250 RPM)
  uint32_t maxRpm;
  uint32_t softCutBand;
  uint32_t launchRpm;
  uint8_t  limiterMode;       // 0: PWM Soft, 1: Cycle Skip, 2: Bee-R Pop, 3: Hard Cut
  uint8_t  enableBackfire;
  uint8_t  backfireMode;      // 0: Decel, 1: Flame, 2: Antilag
  uint8_t  backfireAggression;
  uint8_t  enableRumble;
  uint16_t rumbleMaxRpm;
  uint8_t  rumbleSpeed;       // 1 = Cepat (1 pulsa/step), 2 = Sedang, 3 = Lambat, 4 = Helikopter
  uint8_t  enableLaunch;
  uint8_t  enableShiftLight;
  uint32_t rumbleBitmask30;   // 30-bit: bit i = 1 (Fire / Busi Nyala), bit i = 0 (Cut / Pulser Dipotong)
};

// ==================== PARAMETER RUNTIME (DAPAT DIUBAH VIA WEB) ====================
bool           activeEnableLimiter      = true;      // Master switch Rev Limiter
uint16_t       activeLimiterHysteresis  = 250;       // Histeresis RPM (default 250 RPM)
volatile unsigned long maxRpmThreshold  = 10500UL;   // Batas RPM Hard Cut
volatile unsigned long softCutBandRpm   = 600UL;     // Rentang Soft Cut (RPM)
volatile unsigned long launchRpmLimit   = 5500UL;    // Batas RPM saat Launch Control aktif
uint8_t        activeLimiterMode        = 0;         // 0: PWM Soft Cut
bool           activeEnableBackfire     = true;
uint8_t        activeBackfireMode       = 0;         // 0: Decel Pops
uint8_t        activeBackfireAggression = 3;
bool           activeEnableRumble       = true;
uint16_t       activeRumbleMaxRpm       = 2200;
uint8_t        activeRumbleSpeed        = 2;         // Kecepatan tempo default (2 pulsa per kotak)
bool           activeEnableLaunch       = true;
bool           activeEnableShiftLight   = true;

// Array 30-Step Sequencer Rumble Idle (1 = Fire / Busi Nyala, 0 = Cut / Pulser Dipotong)
uint8_t rumbleSteps[30] = { 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0 };

const uint8_t  PULSES_PER_REV           = 1;         // 1 pulsa per putaran kruk as
const uint32_t DEBOUNCE_MICROS          = 2500UL;    // Filter noise EMF (abaikan interval < 2500 us)
const uint32_t PWM_FREQUENCY_HZ         = 1000;      // Frekuensi PWM ESP8266 (Hz)
const bool     INVERT_CUT_LOGIC         = false;

// ==================== VARIABEL INTERRUPT & STATE ====================
volatile unsigned long lastPulseMicros = 0;
volatile unsigned long pulseIntervalUs = 0;
volatile bool newPulseDetected = false;
volatile unsigned long pulseCount = 0;

unsigned long currentRpm = 0;
unsigned long filteredRpm = 0;
unsigned long previousRpm = 0;
unsigned long lastSerialTime = 0;

int currentPwmDuty = 0; // Rentang 0 - 1023
bool isHardCutActive = false;
bool isSoftCutActive = false;
bool isLaunchModeActive = false;
bool isRumbleActive = false;
bool isBackfireActive = false;
uint8_t activeRumbleStep = 0;

// ==================== ISR (INTERRUPT SERVICE ROUTINE) ====================
void ICACHE_RAM_ATTR isrPulserHandler() {
  unsigned long now = micros();
  unsigned long interval = now - lastPulseMicros;

  if (interval >= DEBOUNCE_MICROS) {
    pulseIntervalUs = interval;
    lastPulseMicros = now;
    newPulseDetected = true;
    pulseCount++;
  }
}

// ==================== KONTROL OUTPUT PEMUTUS PULSER ====================
void setPulserCutDuty(int duty) {
  duty = constrain(duty, 0, 1023);
  currentPwmDuty = duty;
  int actualOutput = INVERT_CUT_LOGIC ? (1023 - duty) : duty;

  if (duty <= 0) {
    digitalWrite(PIN_CUT_PWM, INVERT_CUT_LOGIC ? HIGH : LOW);
  } else if (duty >= 1023) {
    digitalWrite(PIN_CUT_PWM, INVERT_CUT_LOGIC ? LOW : HIGH);
  } else {
    analogWrite(PIN_CUT_PWM, actualOutput);
  }
}

// ==================== FUNGSI LOAD / SAVE EEPROM ====================
void loadConfigFromEEPROM() {
  EEPROM.begin(512);
  TunerConfig cfg;
  EEPROM.get(0, cfg);
  if (cfg.magic == EEPROM_MAGIC) {
    activeEnableLimiter = (cfg.enableLimiter == 1);
    activeLimiterHysteresis = (cfg.limiterHysteresis >= 25 && cfg.limiterHysteresis <= 1000) ? cfg.limiterHysteresis : 250;
    maxRpmThreshold = cfg.maxRpm;
    softCutBandRpm = cfg.softCutBand;
    launchRpmLimit = cfg.launchRpm;
    activeLimiterMode = cfg.limiterMode;
    activeEnableBackfire = (cfg.enableBackfire == 1);
    activeBackfireMode = cfg.backfireMode;
    activeBackfireAggression = cfg.backfireAggression;
    activeEnableRumble = (cfg.enableRumble == 1);
    activeRumbleMaxRpm = cfg.rumbleMaxRpm;
    activeRumbleSpeed = (cfg.rumbleSpeed >= 1 && cfg.rumbleSpeed <= 4) ? cfg.rumbleSpeed : 2;
    activeEnableLaunch = (cfg.enableLaunch == 1);
    activeEnableShiftLight = (cfg.enableShiftLight == 1);

    for (int i = 0; i < 30; i++) {
      rumbleSteps[i] = (cfg.rumbleBitmask30 >> i) & 0x01;
    }
    Serial.println(F("[EEPROM] Parameter, Histeresis & 30-Step Sequencer dimuat dari flash!"));
  } else {
    Serial.println(F("[EEPROM] Memori kosong. Menggunakan konfigurasi bawaan."));
  }
}

void saveConfigToEEPROM() {
  TunerConfig cfg;
  cfg.magic = EEPROM_MAGIC;
  cfg.enableLimiter = activeEnableLimiter ? 1 : 0;
  cfg.limiterHysteresis = activeLimiterHysteresis;
  cfg.maxRpm = maxRpmThreshold;
  cfg.softCutBand = softCutBandRpm;
  cfg.launchRpm = launchRpmLimit;
  cfg.limiterMode = activeLimiterMode;
  cfg.enableBackfire = activeEnableBackfire ? 1 : 0;
  cfg.backfireMode = activeBackfireMode;
  cfg.backfireAggression = activeBackfireAggression;
  cfg.enableRumble = activeEnableRumble ? 1 : 0;
  cfg.rumbleMaxRpm = activeRumbleMaxRpm;
  cfg.rumbleSpeed = activeRumbleSpeed;
  cfg.enableLaunch = activeEnableLaunch ? 1 : 0;
  cfg.enableShiftLight = activeEnableShiftLight ? 1 : 0;

  cfg.rumbleBitmask30 = 0;
  for (int i = 0; i < 30; i++) {
    if (rumbleSteps[i] > 0) {
      cfg.rumbleBitmask30 |= (1UL << i);
    }
  }

  EEPROM.put(0, cfg);
  EEPROM.commit();
  Serial.println(F("[EEPROM] Parameter, Saklar & 30-step berhasil disimpan permanen!"));
}

// ==================== WEB SERVER DASHBOARD DENGAN PANEL 30 STEP ====================
const char PAGE_INDEX[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
  <title>Wemos CDI Tuner - 30 Step Sequencer & Speed</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif}
    body{background:#09090b;color:#f4f4f5;padding:14px;text-align:center}
    .container{max-width:460px;margin:0 auto;background:#18181b;border-radius:24px;border:1px solid #27272a;padding:18px;box-shadow:0 10px 30px rgba(0,0,0,0.8)}
    .header{display:flex;align-items:center;justify-content:space-between;margin-bottom:14px;padding-bottom:10px;border-bottom:1px solid #27272a}
    .badge{background:#a855f7;color:#fff;font-size:10px;font-weight:900;padding:4px 10px;border-radius:20px}
    .gauge-card{background:#09090b;border:1px solid #3f3f46;border-radius:16px;padding:14px;margin-bottom:14px}
    .rpm-display{font-size:38px;font-weight:900;font-family:monospace;color:#38bdf8}
    .status-pill{display:inline-block;margin-top:4px;font-size:10px;font-weight:bold;padding:3px 10px;border-radius:20px;background:#27272a;color:#a1a1aa}
    .panel-box{text-align:left;background:#121214;padding:12px;border-radius:14px;border:1px solid #27272a;margin-bottom:12px}
    .title-row{display:flex;justify-content:space-between;font-size:12px;font-weight:bold;margin-bottom:8px}
    
    .steps-grid{display:grid;grid-template-columns:repeat(6,1fr);gap:6px;margin:10px 0}
    .step-btn{padding:8px 0;font-size:10px;font-weight:bold;font-family:monospace;border-radius:8px;border:1px solid #3f3f46;cursor:pointer;transition:all 0.1s}
    .step-btn.fire{background:#064e3b;color:#34d399;border-color:#059669;box-shadow:0 0 6px rgba(16,185,129,0.3)}
    .step-btn.cut{background:#450a0a;color:#f87171;border-color:#dc2626}
    .step-btn.active{outline:2px solid #38bdf8;transform:scale(1.08);z-index:2}
    
    .preset-row{display:flex;flex-wrap:wrap;gap:6px;margin-top:8px}
    .preset-btn{padding:6px 10px;font-size:10px;border-radius:6px;background:#27272a;color:#ddd;border:none;cursor:pointer}
    .preset-btn:active{background:#a855f7;color:#fff}
    
    input[type=range]{width:100%;height:8px;border-radius:4px;accent-color:#06b6d4;cursor:pointer}
    select{width:100%;padding:8px;background:#09090b;border:1px solid #3f3f46;color:#fff;border-radius:8px;font-size:12px;margin-top:4px}
    .btn-save{width:100%;padding:14px;border:none;border-radius:12px;background:linear-gradient(135deg,#a855f7,#3b82f6);color:#fff;font-size:14px;font-weight:900;cursor:pointer;margin-top:10px;box-shadow:0 0 20px rgba(168,85,247,0.4)}
    .toast{display:none;position:fixed;bottom:20px;left:50%;transform:translateX(-50%);background:#10b981;color:#000;font-weight:bold;padding:10px 20px;border-radius:30px}
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <div style="text-align:left">
        <h2 style="font-size:16px;font-weight:900;color:#fff">WEMOS CDI TUNER</h2>
        <span style="font-size:10px;color:#a1a1aa">Panel 30 Step Irama, Saklar &amp; Histeresis</span>
      </div>
      <span class="badge">PRO TUNER</span>
    </div>

    <!-- Live Tachometer Display -->
    <div class="gauge-card">
      <div class="rpm-display" id="liveRpm">0</div>
      <div style="font-size:10px;color:#71717a;font-family:monospace">ENGINE RPM</div>
      <div class="status-pill" id="liveStatus">MENUNGGU SENSOR PULSER</div>
    </div>

    <!-- PUSAT SAKLAR ON/OFF FITUR -->
    <div class="panel-box" style="border-color:#38bdf8">
      <div class="title-row" style="color:#7dd3fc">
        <span>SAKLAR ON / OFF FITUR:</span>
      </div>
      <div style="display:grid;grid-template-columns:repeat(2,1fr);gap:6px;font-size:11px">
        <label style="display:flex;align-items:center;gap:6px;background:#09090b;padding:6px 8px;border-radius:6px;border:1px solid #27272a;cursor:pointer">
          <input type="checkbox" id="swLimiter" checked>
          <span style="color:#f87171;font-weight:bold">Limiter CDI</span>
        </label>
        <label style="display:flex;align-items:center;gap:6px;background:#09090b;padding:6px 8px;border-radius:6px;border:1px solid #27272a;cursor:pointer">
          <input type="checkbox" id="swLaunch" checked>
          <span style="color:#38bdf8;font-weight:bold">2-Step Launch</span>
        </label>
        <label style="display:flex;align-items:center;gap:6px;background:#09090b;padding:6px 8px;border-radius:6px;border:1px solid #27272a;cursor:pointer">
          <input type="checkbox" id="swBackfire" checked>
          <span style="color:#fb923c;font-weight:bold">Backfire Pops</span>
        </label>
        <label style="display:flex;align-items:center;gap:6px;background:#09090b;padding:6px 8px;border-radius:6px;border:1px solid #27272a;cursor:pointer">
          <input type="checkbox" id="swRumble" checked>
          <span style="color:#c084fc;font-weight:bold">Rumble Idle</span>
        </label>
      </div>
    </div>

    <!-- PANEL 30 STEP KOTAK-KOTAK RUMBLE IDLE -->
    <div class="panel-box" style="border-color:#7e22ce">
      <div class="title-row" style="color:#d8b4fe">
        <span>KOTAK 30-STEP IRAMA RUMBLE:</span>
        <span id="stepCursorText" style="color:#38bdf8;font-family:monospace">STEP 01</span>
      </div>
      <div style="font-size:10px;color:#a1a1aa">
        <span style="color:#34d399">■ HIJAU = FIRE (Busi Nyala)</span> | <span style="color:#f87171">■ MERAH = CUT (Pulser Di-skip)</span>
      </div>

      <!-- Container 30 Kotak -->
      <div class="steps-grid" id="stepsGrid"></div>

      <!-- Tombol Preset Irama Cepat -->
      <div class="preset-row">
        <button type="button" class="preset-btn" onclick="applyPreset('herex')">Herex Drag</button>
        <button type="button" class="preset-btn" onclick="applyPreset('harley')">Harley V-Twin</button>
        <button type="button" class="preset-btn" onclick="applyPreset('roadrace')">Road Race 280°</button>
        <button type="button" class="preset-btn" onclick="applyPreset('allfire')">Semua Fire</button>
      </div>

      <!-- Pilihan Kecepatan / Tempo Rumble Idle -->
      <div style="margin-top:10px;padding-top:8px;border-top:1px solid #27272a;text-align:left">
        <div style="display:flex;justify-content:space-between;font-size:11px;font-weight:bold;color:#d8b4fe;margin-bottom:6px">
          <span>KECEPATAN / TEMPO:</span>
          <span id="speedValText" style="color:#38bdf8">2x (Sedang)</span>
        </div>
        <div style="display:grid;grid-template-columns:repeat(4,1fr);gap:4px">
          <button type="button" class="preset-btn" id="spdBt1" onclick="setSpeed(1)">1x Cepat</button>
          <button type="button" class="preset-btn" id="spdBt2" onclick="setSpeed(2)">2x Sedang</button>
          <button type="button" class="preset-btn" id="spdBt3" onclick="setSpeed(3)">3x Lambat</button>
          <button type="button" class="preset-btn" id="spdBt4" onclick="setSpeed(4)">4x Helikopter</button>
        </div>
      </div>

      <div style="margin-top:10px;font-size:11px">
        <span>Batas Maksimal RPM Rumble: </span><b id="rumbleMaxVal" style="color:#c084fc">2,200 RPM</b>
        <input type="range" id="rumbleMax" min="1800" max="2800" step="50" oninput="document.getElementById('rumbleMaxVal').innerText=this.value+' RPM'">
      </div>
    </div>

    <!-- Parameter Limiter & Histeresis -->
    <div class="panel-box" style="border-color:#b91c1c">
      <div class="title-row" style="color:#fca5a5">
        <span>MAX RPM (CUT AKTIF):</span>
        <span id="maxRpmVal" style="color:#ef4444;font-family:monospace">10,500 RPM</span>
      </div>
      <input type="range" id="maxRpm" min="7000" max="14000" step="100" oninput="updateLimiterUI()">

      <div class="title-row" style="margin-top:10px;color:#fde047">
        <span>HISTERESIS LIMITER:</span>
        <span id="hystVal" style="color:#facc15;font-family:monospace">250 RPM</span>
      </div>
      <input type="range" id="hystRpm" min="50" max="600" step="25" value="250" oninput="updateLimiterUI()">
      <div style="display:flex;justify-content:space-between;font-size:10px;color:#71717a;margin-top:2px">
        <span>Cut Mulai: <b id="cutOnText" style="color:#f87171">10,500 RPM</b></span>
        <span>Lepas Cut: <b id="cutOffText" style="color:#34d399">10,250 RPM</b></span>
      </div>

      <div class="title-row" style="margin-top:10px">
        <span>RENTANG SOFT CUT BAND:</span>
        <span id="softBandVal" style="color:#f59e0b;font-family:monospace">600 RPM</span>
      </div>
      <input type="range" id="softBand" min="200" max="1500" step="50" oninput="document.getElementById('softBandVal').innerText=this.value+' RPM'">

      <div class="title-row" style="margin-top:10px">
        <span>BACKFIRE AGGRESSION:</span>
        <span id="bfVal" style="color:#f97316;font-family:monospace">Level 3</span>
      </div>
      <input type="range" id="bfAgg" min="1" max="5" step="1" oninput="document.getElementById('bfVal').innerText='Level '+this.value">
    </div>

    <button type="button" class="btn-save" onclick="saveToEeprom()">SIMPAN KE MEMORI EEPROM</button>
  </div>

  <div id="toast" class="toast">Semua Saklar &amp; Parameter Berhasil Disimpan ke EEPROM!</div>

  <script>
    var currentSteps = [1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0];
    var currentSpeed = 2;

    function renderSteps(activeStepIdx){
      var grid = document.getElementById('stepsGrid');
      grid.innerHTML = '';
      for (var i = 0; i < 30; i++){
        var btn = document.createElement('div');
        var isFire = currentSteps[i] === 1;
        btn.className = 'step-btn ' + (isFire ? 'fire' : 'cut') + (i === activeStepIdx ? ' active' : '');
        btn.innerText = (i+1 < 10 ? '0' : '') + (i+1) + (isFire ? ' ●' : ' ✕');
        btn.setAttribute('data-idx', i);
        btn.onclick = function(){
          var idx = Number(this.getAttribute('data-idx'));
          currentSteps[idx] = currentSteps[idx] === 1 ? 0 : 1;
          renderSteps(activeStepIdx);
        };
        grid.appendChild(btn);
      }
    }
    renderSteps(0);

    function setSpeed(s){
      currentSpeed = s;
      updateSpeedUI();
    }

    function updateSpeedUI(){
      for(var i=1;i<=4;i++){
        var b = document.getElementById('spdBt'+i);
        if(b) {
          b.style.background = (currentSpeed === i ? '#a855f7' : '#27272a');
          b.style.color = (currentSpeed === i ? '#ffffff' : '#dddddd');
          b.style.fontWeight = (currentSpeed === i ? 'bold' : 'normal');
        }
      }
      var t = document.getElementById('speedValText');
      if(t) t.innerText = currentSpeed === 1 ? '1x Cepat' : currentSpeed === 2 ? '2x Sedang' : currentSpeed === 3 ? '3x Lambat' : '4x Helikopter';
    }
    updateSpeedUI();

    function updateLimiterUI(){
      var max = Number(document.getElementById('maxRpm').value);
      var hyst = Number(document.getElementById('hystRpm').value);
      document.getElementById('maxRpmVal').innerText = max.toLocaleString() + ' RPM';
      document.getElementById('hystVal').innerText = hyst + ' RPM';
      document.getElementById('cutOnText').innerText = max.toLocaleString() + ' RPM';
      document.getElementById('cutOffText').innerText = Math.max(1000, max - hyst).toLocaleString() + ' RPM';
    }

    function applyPreset(type){
      if (type === 'herex'){
        currentSteps = [1,1,1,0, 1,1,1,1,0,0, 1,1,1,0, 1,1,1,0,0, 1,1,1,1,0, 1,1,1,1,0,0];
      } else if (type === 'harley'){
        currentSteps = [1,1,0,0, 1,0,0,0, 1,1,0,0, 1,0,0,0, 1,1,0,0, 1,0,0,0, 1,1,0,0, 1,0];
      } else if (type === 'roadrace'){
        currentSteps = [1,1,1,1,0, 1,1,1,1,0, 1,1,1,1,0, 1,1,1,1,0, 1,1,1,1,0, 1,1,1,1,0];
      } else if (type === 'allfire'){
        currentSteps = new Array(30).fill(1);
      }
      renderSteps(0);
    }

    setInterval(function(){
      fetch('/api/data')
        .then(r => r.json())
        .then(d => {
          document.getElementById('liveRpm').innerText = d.rpm.toLocaleString();
          document.getElementById('liveStatus').innerText = d.status;
          document.getElementById('stepCursorText').innerText = 'STEP ' + ((d.step + 1 < 10 ? '0':'') + (d.step + 1));
          renderSteps(d.step);
        }).catch(e=>{});
    }, 100);

    fetch('/api/config')
      .then(r => r.json())
      .then(d => {
        if (d.enableLimiter !== undefined) document.getElementById('swLimiter').checked = d.enableLimiter;
        if (d.enableLaunch !== undefined) document.getElementById('swLaunch').checked = d.enableLaunch;
        if (d.enableBackfire !== undefined) document.getElementById('swBackfire').checked = d.enableBackfire;
        if (d.enableRumble !== undefined) document.getElementById('swRumble').checked = d.enableRumble;

        document.getElementById('maxRpm').value = d.maxRpm;
        document.getElementById('softBand').value = d.softBand;
        document.getElementById('bfAgg').value = d.bfAgg;
        document.getElementById('rumbleMax').value = d.rumbleMax;
        if (d.hystRpm) document.getElementById('hystRpm').value = d.hystRpm;
        updateLimiterUI();

        document.getElementById('softBandVal').innerText = Number(d.softBand) + ' RPM';
        document.getElementById('bfVal').innerText = 'Level ' + d.bfAgg;
        document.getElementById('rumbleMaxVal').innerText = Number(d.rumbleMax) + ' RPM';
        if (d.rumbleSpeed) {
          currentSpeed = Number(d.rumbleSpeed);
          updateSpeedUI();
        }
        if (d.steps && d.steps.length === 30){
          currentSteps = d.steps;
          renderSteps(0);
        }
      }).catch(e=>{});

    function saveToEeprom(){
      var payload = {
        enableLimiter: document.getElementById('swLimiter').checked,
        enableLaunch: document.getElementById('swLaunch').checked,
        enableBackfire: document.getElementById('swBackfire').checked,
        enableRumble: document.getElementById('swRumble').checked,
        maxRpm: Number(document.getElementById('maxRpm').value),
        hystRpm: Number(document.getElementById('hystRpm').value),
        softBand: Number(document.getElementById('softBand').value),
        bfAgg: Number(document.getElementById('bfAgg').value),
        rumbleMax: Number(document.getElementById('rumbleMax').value),
        rumbleSpeed: currentSpeed,
        steps: currentSteps
      };

      fetch('/api/save', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(payload)
      }).then(r => {
        var t = document.getElementById('toast');
        t.style.display = 'block';
        setTimeout(() => t.style.display = 'none', 2500);
      });
    }
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", PAGE_INDEX);
}

void handleApiData() {
  String status = "NORMAL JALAN";
  if (!activeEnableLimiter) status = "LIMITER BYPASS (OFF)";
  else if (isHardCutActive) status = "HARD CUT 100% GROUNDED!";
  else if (isSoftCutActive) status = "PWM SOFT CUT ACTIVE";
  else if (isBackfireActive) status = "BACKFIRE FLAME POP!!";
  else if (isRumbleActive) status = "RUMBLE IDLE 30-STEP CHOP";

  String json = "{";
  json += "\"rpm\":" + String(currentRpm) + ",";
  json += "\"duty\":" + String(map(currentPwmDuty, 0, 1023, 0, 100)) + ",";
  json += "\"status\":\"" + status + "\",";
  json += "\"step\":" + String(activeRumbleStep) + ",";
  json += "\"hardCut\":" + String(isHardCutActive ? "true" : "false") + ",";
  json += "\"softCut\":" + String(isSoftCutActive ? "true" : "false") + ",";
  json += "\"rumble\":" + String(isRumbleActive ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}

void handleApiConfig() {
  String json = "{";
  json += "\"enableLimiter\":" + String(activeEnableLimiter ? "true" : "false") + ",";
  json += "\"hystRpm\":" + String(activeLimiterHysteresis) + ",";
  json += "\"enableLaunch\":" + String(activeEnableLaunch ? "true" : "false") + ",";
  json += "\"enableBackfire\":" + String(activeEnableBackfire ? "true" : "false") + ",";
  json += "\"enableRumble\":" + String(activeEnableRumble ? "true" : "false") + ",";
  json += "\"maxRpm\":" + String(maxRpmThreshold) + ",";
  json += "\"softBand\":" + String(softCutBandRpm) + ",";
  json += "\"bfAgg\":" + String(activeBackfireAggression) + ",";
  json += "\"rumbleMax\":" + String(activeRumbleMaxRpm) + ",";
  json += "\"rumbleSpeed\":" + String(activeRumbleSpeed) + ",";
  json += "\"steps\":[";
  for (int i = 0; i < 30; i++) {
    json += String(rumbleSteps[i]);
    if (i < 29) json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleApiSave() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    int idx;

    idx = body.indexOf("\"enableLimiter\":");
    if (idx != -1) activeEnableLimiter = body.substring(idx + 16, idx + 20).startsWith("true");

    idx = body.indexOf("\"enableLaunch\":");
    if (idx != -1) activeEnableLaunch = body.substring(idx + 15, idx + 19).startsWith("true");

    idx = body.indexOf("\"enableBackfire\":");
    if (idx != -1) activeEnableBackfire = body.substring(idx + 17, idx + 21).startsWith("true");

    idx = body.indexOf("\"enableRumble\":");
    if (idx != -1) activeEnableRumble = body.substring(idx + 15, idx + 19).startsWith("true");

    idx = body.indexOf("\"hystRpm\":");
    if (idx != -1) activeLimiterHysteresis = body.substring(idx + 10).toInt();

    idx = body.indexOf("\"maxRpm\":");
    if (idx != -1) maxRpmThreshold = body.substring(idx + 9).toInt();

    idx = body.indexOf("\"softBand\":");
    if (idx != -1) softCutBandRpm = body.substring(idx + 11).toInt();

    idx = body.indexOf("\"bfAgg\":");
    if (idx != -1) activeBackfireAggression = body.substring(idx + 8).toInt();

    idx = body.indexOf("\"rumbleMax\":");
    if (idx != -1) activeRumbleMaxRpm = body.substring(idx + 12).toInt();

    idx = body.indexOf("\"rumbleSpeed\":");
    if (idx != -1) activeRumbleSpeed = body.substring(idx + 14).toInt();

    // Parse array "steps":[1,1,0, ...]
    idx = body.indexOf("\"steps\":[");
    if (idx != -1) {
      int startIdx = idx + 9;
      int endIdx = body.indexOf("]", startIdx);
      if (endIdx != -1) {
        String stepsStr = body.substring(startIdx, endIdx);
        int stepIdx = 0;
        int commaIdx = 0;
        while (stepIdx < 30) {
          int nextComma = stepsStr.indexOf(',', commaIdx);
          String valStr = (nextComma == -1) ? stepsStr.substring(commaIdx) : stepsStr.substring(commaIdx, nextComma);
          valStr.trim();
          rumbleSteps[stepIdx] = valStr.toInt() > 0 ? 1 : 0;
          stepIdx++;
          if (nextComma == -1) break;
          commaIdx = nextComma + 1;
        }
      }
    }

    saveConfigToEEPROM();
    server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"EEPROM Saved!\"}");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n[INIT] Memulai CDI Limiter & 30-Step Rumble Tuner Wemos D1 Mini..."));

  loadConfigFromEEPROM();

  pinMode(PIN_PULSER_IN, INPUT_PULLUP);
  pinMode(PIN_CUT_PWM, OUTPUT);
  pinMode(PIN_LAUNCH_SW, INPUT_PULLUP);
  pinMode(PIN_SHIFT_LIGHT, OUTPUT);

  digitalWrite(PIN_CUT_PWM, INVERT_CUT_LOGIC ? HIGH : LOW);
  digitalWrite(PIN_SHIFT_LIGHT, LOW);

  analogWriteFreq(PWM_FREQUENCY_HZ);
  analogWriteRange(1023);
  setPulserCutDuty(0);

  // Pasang interrupt pulser pada sinyal jatuh (FALLING edge dari optocoupler)
  attachInterrupt(digitalPinToInterrupt(PIN_PULSER_IN), isrPulserHandler, FALLING);

  // Inisialisasi Wi-Fi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);
  Serial.print(F("[WIFI] Access Point Siap: "));
  Serial.println(ap_ssid);
  Serial.print(F("[WIFI] Alamat IP Web Tuner: http://"));
  Serial.println(WiFi.softAPIP());

  // Daftarkan route HTTP server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleApiData);
  server.on("/api/config", HTTP_GET, handleApiConfig);
  server.on("/api/save", HTTP_POST, handleApiSave);
  server.begin();
  Serial.println(F("[HTTP] Web Server Tuner berjalan di port 80"));
}

// ==================== LOOP UTAMA ====================
void loop() {
  server.handleClient();
  unsigned long currentMillis = millis();

  // 1. HITUNG RPM DARI INTERVAL INTERRUPT PULSER
  if (micros() - lastPulseMicros > 1200000UL) {
    currentRpm = 0;
    filteredRpm = 0;
    setPulserCutDuty(0);
    isHardCutActive = false;
    isSoftCutActive = false;
    isRumbleActive = false;
    isBackfireActive = false;
  } else if (newPulseDetected) {
    noInterrupts();
    unsigned long interval = pulseIntervalUs;
    newPulseDetected = false;
    interrupts();

    if (interval > 0) {
      unsigned long rawRpm = (60000000UL / interval) / PULSES_PER_REV;
      previousRpm = currentRpm;
      filteredRpm = (filteredRpm * 3 + rawRpm) / 4;
      currentRpm = filteredRpm;
    }
  }

  // 2. CEK STATUS TOMBOL 2-STEP LAUNCH CONTROL
  isLaunchModeActive = activeEnableLaunch && (digitalRead(PIN_LAUNCH_SW) == LOW);
  unsigned long effectiveLimitRpm = isLaunchModeActive ? launchRpmLimit : maxRpmThreshold;
  unsigned long effectiveSoftStart = (effectiveLimitRpm > softCutBandRpm) ? (effectiveLimitRpm - softCutBandRpm) : 0;

  isRumbleActive = false;
  isBackfireActive = false;

  // 3. FITUR RUMBLE IDLE / CHOP CAM DENGAN PANEL 30-STEP SEQUENCER & KECEPATAN
  if (activeEnableRumble && currentRpm >= 700 && currentRpm <= activeRumbleMaxRpm) {
    isRumbleActive = true;
    uint8_t speedDiv = (activeRumbleSpeed >= 1 && activeRumbleSpeed <= 4) ? activeRumbleSpeed : 2;
    activeRumbleStep = (pulseCount / speedDiv) % 30; // Kecepatan tempo langkah kustom

    // Cek status step: 1 = FIRE (busi nyala), 0 = CUT (pulser dipotong ke ground)
    if (rumbleSteps[activeRumbleStep] == 0) {
      setPulserCutDuty(1023); // Pulser di-ground (percikan dipotong)
    } else {
      setPulserCutDuty(0);    // Pulser lolos (percikan normal)
    }
  }
  // 4. FITUR OVERRUN BACKFIRE / POP & BANGS
  else if (activeEnableBackfire && previousRpm > currentRpm && currentRpm >= 3800 && (previousRpm - currentRpm) >= 80) {
    isBackfireActive = true;
    if (activeBackfireMode == 2) {
      if ((currentMillis / (35 - activeBackfireAggression * 4)) % 2 == 0) setPulserCutDuty(1023);
      else setPulserCutDuty(0);
    } else if (activeBackfireMode == 1) {
      if (pulseCount % 3 != 0) setPulserCutDuty(1023);
      else setPulserCutDuty(0);
    } else {
      if ((pulseCount % (8 - activeBackfireAggression)) == 0) setPulserCutDuty(1023);
      else setPulserCutDuty(0);
    }
  }
  // 5. LOGIKA REV LIMITER UTAMA DENGAN SAKLAR MASTER & HISTERESIS
  else if (activeEnableLimiter) {
    unsigned long cutDisengageRpm = (effectiveLimitRpm > activeLimiterHysteresis) ? (effectiveLimitRpm - activeLimiterHysteresis) : 0;

    // Histeresis state check: Cut aktif di >= effectiveLimitRpm,
    // dan hanya mati kembali saat RPM turun di bawah cutDisengageRpm!
    if (currentRpm >= effectiveLimitRpm) {
      isHardCutActive = true;
    } else if (currentRpm < cutDisengageRpm) {
      isHardCutActive = false;
    }

    if (isHardCutActive) {
      setPulserCutDuty(1023); // 100% pulser dipotong / di-ground
      isSoftCutActive = false;
    } else if (activeLimiterMode == 0 && currentRpm >= effectiveSoftStart) {
      // Mode 0: PWM Soft Cut gradual
      isSoftCutActive = true;
      unsigned long rpmDiff = currentRpm - effectiveSoftStart;
      int targetDuty = map(rpmDiff, 0, softCutBandRpm, 150, 850);
      setPulserCutDuty(targetDuty);
    } else if (activeLimiterMode == 1 && currentRpm >= effectiveSoftStart) {
      // Mode 1: Cycle Skip
      isSoftCutActive = true;
      if (pulseCount % 3 == 0) setPulserCutDuty(1023);
      else setPulserCutDuty(0);
    } else if (activeLimiterMode == 2 && currentRpm >= effectiveSoftStart) {
      // Mode 2: Bee-R Pop & Bang
      isSoftCutActive = true;
      if ((currentMillis / 18) % 2 == 0) setPulserCutDuty(1023);
      else setPulserCutDuty(0);
    } else {
      setPulserCutDuty(0);
      isSoftCutActive = false;
    }
  } else {
    // SAKLAR LIMITER MATI (BYPASS) - Pulser bebas lolos tanpa batasan cut
    setPulserCutDuty(0);
    isHardCutActive = false;
    isSoftCutActive = false;
  }

  // 6. SHIFT LIGHT INDICATOR
  if (activeEnableShiftLight && currentRpm >= (maxRpmThreshold - 400)) {
    digitalWrite(PIN_SHIFT_LIGHT, HIGH);
  } else {
    digitalWrite(PIN_SHIFT_LIGHT, LOW);
  }

  // 7. TELEMETRI SERIAL MONITOR
  if (currentMillis - lastSerialTime >= 100) {
    lastSerialTime = currentMillis;
    Serial.print(F("RPM: "));
    Serial.print(currentRpm);
    Serial.print(F(" | PWM_Cut: "));
    Serial.print(map(currentPwmDuty, 0, 1023, 0, 100));
    Serial.print(F("% | Status: "));
    if (!activeEnableLimiter)   Serial.println(F("[LIMITER BYPASS]"));
    else if (isHardCutActive)  Serial.println(F("[HARD CUT (HYSTERESIS LATCH)]"));
    else if (isSoftCutActive)  Serial.println(F("[SOFT CUT PWM]"));
    else if (isBackfireActive) Serial.println(F("[BACKFIRE POP!!]"));
    else if (isRumbleActive) {
      Serial.print(F("[30-STEP RUMBLE: STEP "));
      Serial.print(activeRumbleStep + 1);
      Serial.print(F(" (Speed "));
      Serial.print(activeRumbleSpeed);
      Serial.println(F("x)]"));
    }
    else                       Serial.println(F("[NORMAL PASS]"));
  }
}
