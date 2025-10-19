//============================================================
// Buck Converter Test - TIP120, L=560 uH, FB on A0, PWM on D6
// Non-blocking status prints; serial control
//============================================================

//---------------- Config & Pins ----------------
const uint8_t PIN_FB  = A0;     // feedback node
const uint8_t PIN_PWM = 6;      // PWM to driver/base (use a base resistor)

// ADC & supply
const float ADC_REF_V   = 5.0f; // Nano default AREF
const int   ADC_MAX     = 1023;

// Divider model (Vout -->[R_TOP]--> FB -->[R_P]--> GND)
float R_TOP_OHM = 98000.0f;    // fixed top resistor (R1)
float R_P_OHM   = 98000.0f;    // effective pot FB->GND; set via SETRP

// PWM
const int PWM_MIN = 0;
const int PWM_MAX = 255;
int pwmDuty = 0;
bool enabled = false;

// Timing
const unsigned long PRINT_MS = 1000;
unsigned long t_lastPrint = 0;

//---------------- Helpers ----------------
float adcToVolts(int raw) {
  return (raw * ADC_REF_V) / ADC_MAX;
}

float estimateVout(float v_fb) {
  // Vout = Vfb * (1 + R_TOP/R_P)
  if (R_P_OHM <= 1.0f) return NAN;
  return v_fb * (1.0f + (R_TOP_OHM / R_P_OHM));
}

void applyPwm() {
  analogWrite(PIN_PWM, enabled ? pwmDuty : 0);
}

void printStatus() {
  int   raw   = analogRead(PIN_FB);
  float v_fb  = adcToVolts(raw);
  float v_out = estimateVout(v_fb);

  Serial.print("STATUS | run:");
  Serial.print(enabled ? "1" : "0");
  Serial.print(" pwm:");
  Serial.print(pwmDuty);
  Serial.print(" raw:");
  Serial.print(raw);
  Serial.print(" v_fb:");
  Serial.print(v_fb, 3);
  Serial.print("V r_top:");
  Serial.print(R_TOP_OHM, 0);
  Serial.print(" r_p:");
  Serial.print(R_P_OHM, 0);
  Serial.print(" est_vout:");
  if (isnan(v_out)) Serial.print("NaN");
  else { Serial.print(v_out, 3); Serial.print("V"); }
  Serial.println();
}

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;

  if (s.equalsIgnoreCase("ON")) {
    enabled = true;
    applyPwm();
    Serial.println("OK: ON");
    return;
  }
  if (s.equalsIgnoreCase("OFF")) {
    enabled = false;
    applyPwm();
    Serial.println("OK: OFF");
    return;
  }
  if (s.startsWith("SET ")) {
    int val = s.substring(4).toInt();
    val = constrain(val, PWM_MIN, PWM_MAX);
    pwmDuty = val;
    applyPwm();
    Serial.print("OK: PWM=");
    Serial.println(pwmDuty);
    return;
  }
  if (s.startsWith("SETRP ")) {
    long rp = s.substring(6).toInt();
    if (rp >= 0) {
      R_P_OHM = (float)rp;
      Serial.print("OK: RP=");
      Serial.println((long)R_P_OHM);
    } else Serial.println("ERR: RP");
    return;
  }
  if (s.startsWith("SETTOP ")) {
    long rt = s.substring(7).toInt();
    if (rt > 0) {
      R_TOP_OHM = (float)rt;
      Serial.print("OK: RTOP=");
      Serial.println((long)R_TOP_OHM);
    } else Serial.println("ERR: RTOP");
    return;
  }
  if (s.equalsIgnoreCase("STATUS")) {
    printStatus();
    return;
  }

  Serial.println("CMDS: ON | OFF | SET <0-255> | SETRP <ohm> | SETTOP <ohm> | STATUS");
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
  pinMode(PIN_PWM, OUTPUT);
  analogWrite(PIN_PWM, 0);
  Serial.begin(115200);
  delay(50);
  Serial.println("Buck test ready. CMDS: ON | OFF | SET <0-255> | SETRP <ohm> | SETTOP <ohm> | STATUS");
}

void loop() {
  pollSerial();

  unsigned long now = millis();
  if (now - t_lastPrint >= PRINT_MS) {
    t_lastPrint = now;
    printStatus();
  }
}
