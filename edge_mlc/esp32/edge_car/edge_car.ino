// ESP32 motor controller for the RC car (Arduino IDE, board "ESP32 Dev Module").
// Library: ESP32Servo (Library Manager).
//
// Wiring ASSUMPTIONS (change the pin numbers to match yours):
//   RC receiver CH1 (steering) signal -> GPIO 34      RC receiver CH2 (throttle) -> GPIO 35
//   Steering servo signal             <- GPIO 18      ESC (motor) signal          <- GPIO 19
//   Pi USB cable -> ESP32 USB port (Serial, 115200 baud). Common ground everywhere.
//
// Serial protocol (one line per message):
//   Pi -> ESP32 : C,<seq>,<thr>,<steer>,<cap>   thr/steer -1000..1000, cap 0..1000
//                 K,<cap>                       speed cap only
//   ESP32 -> Pi : T,<ms>,<thr_out>,<steer_out>,<src>,<cap>,<failsafe>   at 20 Hz
//
// Rules:
//   * network commands are used while fresh (< 300 ms old)
//   * otherwise, if RC_BACKUP is enabled and the RC receiver is sending, RC is used
//   * no command from any source for 500 ms -> FAILSAFE: throttle neutral
//   * throttle is always limited to +/- cap (speed cap from the Pi)

#include <ESP32Servo.h>

const int PIN_RC_STEER = 34, PIN_RC_THR = 35, PIN_SERVO = 18, PIN_ESC = 19;
const bool RC_BACKUP = false;          // true only for the "no infrastructure" demo
const uint32_t NET_FRESH_MS = 300, FAILSAFE_MS = 500;
const int NEUTRAL_US = 1500, SPAN_US = 500;   // ASSUMPTION: 1000-2000 us, 1500 = neutral

Servo servo, esc;
volatile uint32_t rcRise[2], rcWidth[2], rcSeen[2];
int netThr = 0, netSteer = 0, cap = 1000;
uint32_t lastNet = 0, lastAny = 0, lastTelem = 0;
String line;

void IRAM_ATTR isrSteer() { uint32_t t = micros(); if (digitalRead(PIN_RC_STEER)) rcRise[0] = t; else { rcWidth[0] = t - rcRise[0]; rcSeen[0] = millis(); } }
void IRAM_ATTR isrThr()   { uint32_t t = micros(); if (digitalRead(PIN_RC_THR))   rcRise[1] = t; else { rcWidth[1] = t - rcRise[1]; rcSeen[1] = millis(); } }

int usToUnit(uint32_t us) { return constrain(((int)us - NEUTRAL_US) * 1000 / SPAN_US, -1000, 1000); }
int unitToUs(int u) { return NEUTRAL_US + (long)u * SPAN_US / 1000; }

void handleLine(const String &s) {
  if (s.startsWith("C,")) {
    int a = s.indexOf(',', 2), b = s.indexOf(',', a + 1), c = s.indexOf(',', b + 1);
    if (a < 0 || b < 0 || c < 0) return;
    netThr = s.substring(a + 1, b).toInt();
    netSteer = s.substring(b + 1, c).toInt();
    cap = s.substring(c + 1).toInt();
    lastNet = lastAny = millis();
  } else if (s.startsWith("K,")) {
    cap = s.substring(2).toInt();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RC_STEER, INPUT); pinMode(PIN_RC_THR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_STEER), isrSteer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_THR), isrThr, CHANGE);
  servo.attach(PIN_SERVO, 1000, 2000); esc.attach(PIN_ESC, 1000, 2000);
  servo.writeMicroseconds(NEUTRAL_US); esc.writeMicroseconds(NEUTRAL_US);
  delay(2000);                          // let the ESC arm at neutral
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') { handleLine(line); line = ""; }
    else if (line.length() < 64) line += ch;
  }
  uint32_t now = millis();
  int thr = 0, steer = 0, failsafe = 0;
  const char *src = "NONE";
  bool rcValid = RC_BACKUP && now - rcSeen[0] < 100 && now - rcSeen[1] < 100
                 && rcWidth[1] > 900 && rcWidth[1] < 2100;

  if (now - lastNet < NET_FRESH_MS) { thr = netThr; steer = netSteer; src = "NET"; }
  else if (rcValid) { thr = usToUnit(rcWidth[1]); steer = usToUnit(rcWidth[0]); src = "RC"; lastAny = now; }

  if (now - lastAny > FAILSAFE_MS) { thr = 0; failsafe = 1; src = "NONE"; }
  thr = constrain(thr, -cap, cap);      // speed cap from the Pi (graceful degradation)

  esc.writeMicroseconds(unitToUs(thr));
  servo.writeMicroseconds(unitToUs(steer));

  if (now - lastTelem >= 50) {
    lastTelem = now;
    Serial.printf("T,%lu,%d,%d,%s,%d,%d\n", now, thr, steer, src, cap, failsafe);
  }
}
