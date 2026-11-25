//============================================================
// Slice_BUCK - P-FET high-side buck, 0-12 V from the 12 V bus
// PWM on D9 (Timer1 OC1A, 31.25 kHz, 9-bit), Vout sensed on A0
// through a 20.0k / 10.0k divider (A0 = Vout / 3).
// D9 high = Q1 on. D9 low or floating = Q1 off.
// Serial control; SETV regulates Vout on A0 (PI loop, slew limit,
// over-voltage cut-off). Timer0 (millis) is untouched.
//============================================================

//---------------- Config & Pins ----------------
const uint8_t PIN_FB  = A0;     // divider tap
const uint8_t PIN_PWM = 9;      // OC1A -> R2 -> Q3 gate driver

// ADC & supply
const float ADC_REF_V   = 5.0f; // Nano default AREF (+5V rail)
const int   ADC_MAX     = 1023;

// Divider (Vout -->[R_TOP]--> A0 -->[R_BOT]--> GND), R7 / R8
float R_TOP_OHM = 20000.0f;     // R7, set via SETTOP to calibrate
const float R_BOT_OHM = 10000.0f; // R8

// PWM: Timer1 mode 14 (fast PWM, TOP = ICR1), no prescaler
const uint16_t PWM_TOP      = 511;   // 16 MHz / 512 = 31.25 kHz
const uint16_t PWM_FULL_MIN = 496;   // ~97 %: from here on drive 100 %
uint16_t duty = 0;                   // 0..PWM_TOP as applied
int openDuty = 0;                    // SET request, 0..PWM_TOP
bool enabled = false;

// Regulation (SETV)
const float VOUT_MAX     = 12.0f;   // highest SETV accepted
const float OV_MARGIN    = 1.0f;    // cut off if Vout > setpoint + this
const float OV_ABS       = 13.0f;   // or above this in any mode
const uint8_t OV_COUNT   = 50;      // consecutive 1 ms samples
const float SLEW_V_PER_S = 5.0f;    // setpoint ramp
const float KP           = 4.0f;    // counts per volt
const float KI           = 2.5f;    // counts per volt per 1 ms sample
bool  regulate  = false;
float v_target  = 0.0f;   // requested
float v_ramp    = 0.0f;   // slewed setpoint the loop follows
float integ     = 0.0f;   // integrator, counts
uint8_t ovHits  = 0;

// Timing
const unsigned long CTRL_MS  = 1;
const unsigned long PRINT_MS = 1000;
unsigned long t_lastCtrl  = 0;
unsigned long t_lastPrint = 0;

//---------------- PWM ----------------
void pwmInit() {
  digitalWrite(PIN_PWM, LOW);
  pinMode(PIN_PWM, OUTPUT);
  TCCR1A = _BV(WGM11);                           // OC1A disconnected for now
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);  // mode 14, clk/1
  ICR1   = PWM_TOP;
  OCR1A  = 0;
}

// Apply a duty in counts. 0 and >= PWM_FULL_MIN disconnect OC1A and
// drive the pin, so there is no 1-clock glitch at 0 and no off-time
// too short for the gate driver to turn Q1 off near 100 %.
void applyDuty(int d) {
  if (!enabled || d <= 0) {
    duty = 0;
    TCCR1A &= ~_BV(COM1A1);
    digitalWrite(PIN_PWM, LOW);
  } else if (d >= PWM_FULL_MIN) {
    duty = PWM_TOP;
    TCCR1A &= ~_BV(COM1A1);
    digitalWrite(PIN_PWM, HIGH);
  } else {
    duty = (uint16_t)d;
    OCR1A = duty;
    TCCR1A |= _BV(COM1A1);
  }
}

//---------------- Helpers ----------------
float adcToVolts(int raw) {
  return (raw * ADC_REF_V) / ADC_MAX;
}

float estimateVout(float v_fb) {
  return v_fb * (1.0f + (R_TOP_OHM / R_BOT_OHM));
}

float readVout() {
  return estimateVout(adcToVolts(analogRead(PIN_FB)));
}

void shutdownOutput(const __FlashStringHelper *why, float v) {
  enabled = false;
  regulate = false;
  integ = 0.0f;
  applyDuty(0);
  Serial.print(F("ERR: "));
  Serial.print(why);
  Serial.print(F(" vout:"));
  Serial.print(v, 2);
  Serial.println(F("V, output off"));
}

// 1 ms control step: over-voltage check always, PI loop under SETV
void controlStep() {
  float v = readVout();

  if (enabled) {
    // above the slewed setpoint while still switching: the loop is not
    // holding Vout (a falling setpoint with duty 0 is only the output
    // capacitor discharging, not a fault)
    bool ov = (v > OV_ABS) || (regulate && duty > 0 && v > v_ramp + OV_MARGIN);
    ovHits = ov ? (ovHits < 255 ? ovHits + 1 : 255) : 0;
    if (ovHits >= OV_COUNT) {
      ovHits = 0;
      shutdownOutput(F("OV"), v);
      return;
    }
  }
  if (!enabled || !regulate) return;

  // slew the setpoint toward the target
  const float step = SLEW_V_PER_S * (CTRL_MS / 1000.0f);
  if (v_ramp < v_target) v_ramp = min(v_ramp + step, v_target);
  else if (v_ramp > v_target) v_ramp = max(v_ramp - step, v_target);

  float e = v_ramp - v;
  integ += KI * e;
  integ = constrain(integ, 0.0f, (float)PWM_TOP);  // anti-windup
  float u = integ + KP * e;
  applyDuty((int)constrain(u, 0.0f, (float)PWM_TOP));
}

void printStatus() {
  int   raw   = analogRead(PIN_FB);
  float v_fb  = adcToVolts(raw);
  float v_out = estimateVout(v_fb);

  Serial.print(F("STATUS | run:"));
  Serial.print(enabled ? "1" : "0");
  Serial.print(F(" mode:"));
  Serial.print(regulate ? F("SETV") : F("SET"));
  Serial.print(F(" duty:"));
  Serial.print(duty);
  Serial.print(F("/"));
  Serial.print(PWM_TOP);
  Serial.print(F(" raw:"));
  Serial.print(raw);
  Serial.print(F(" v_fb:"));
  Serial.print(v_fb, 3);
  Serial.print(F("V r_top:"));
  Serial.print(R_TOP_OHM, 0);
  Serial.print(F(" r_bot:"));
  Serial.print(R_BOT_OHM, 0);
  Serial.print(F(" est_vout:"));
  Serial.print(v_out, 3);
  Serial.print(F("V"));
  if (regulate) {
    Serial.print(F(" target:"));
    Serial.print(v_target, 2);
    Serial.print(F("V ramp:"));
    Serial.print(v_ramp, 2);
    Serial.print(F("V"));
  }
  Serial.println();
}

const char HELP[] = "CMDS: ON | OFF | SET <0-255> | SETV <0-12 V> | SETTOP <ohm> | STATUS";

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;

  if (s.equalsIgnoreCase("ON")) {
    enabled = true;
    ovHits = 0;
    if (regulate) { v_ramp = readVout(); integ = duty; }
    else applyDuty(openDuty);
    Serial.println(F("OK: ON"));
    return;
  }
  if (s.equalsIgnoreCase("OFF")) {
    enabled = false;
    integ = 0.0f;
    applyDuty(0);
    Serial.println(F("OK: OFF"));
    return;
  }
  if (s.startsWith("SETV ")) {
    float v = s.substring(5).toFloat();
    if (v < 0.0f || v > VOUT_MAX) { Serial.println(F("ERR: SETV 0-12")); return; }
    if (!regulate) { v_ramp = readVout(); integ = duty; }
    regulate = true;
    v_target = v;
    Serial.print(F("OK: V="));
    Serial.println(v_target, 2);
    return;
  }
  if (s.startsWith("SET ")) {
    // open loop: 0-255 scaled onto the 9-bit timer
    int val = constrain(s.substring(4).toInt(), 0, 255);
    regulate = false;
    openDuty = (int)(((long)val * PWM_TOP + 127) / 255);
    applyDuty(openDuty);
    Serial.print(F("OK: PWM="));
    Serial.println(val);
    return;
  }
  if (s.startsWith("SETTOP ")) {
    long rt = s.substring(7).toInt();
    if (rt > 0) {
      R_TOP_OHM = (float)rt;
      Serial.print(F("OK: RTOP="));
      Serial.println((long)R_TOP_OHM);
    } else Serial.println(F("ERR: RTOP"));
    return;
  }
  if (s.equalsIgnoreCase("STATUS")) {
    printStatus();
    return;
  }

  Serial.println(HELP);
}

void pollSerial() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length()) { handleLine(buf); buf = ""; }
    } else {
      if (buf.length() < 80) buf += c; // avoid runaway
    }
  }
}

//---------------- Arduino ----------------
void setup() {
  pwmInit();               // first: Q1 stays off
  Serial.begin(115200);
  delay(50);
  Serial.print(F("Buck ready. "));
  Serial.println(HELP);
}

void loop() {
  pollSerial();

  unsigned long now = millis();
  if (now - t_lastCtrl >= CTRL_MS) {
    t_lastCtrl = now;
    controlStep();
  }
  if (now - t_lastPrint >= PRINT_MS) {
    t_lastPrint = now;
    printStatus();
  }
}
