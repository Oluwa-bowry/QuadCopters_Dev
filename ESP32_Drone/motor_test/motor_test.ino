// ═══════════════════════════════════════════════════════════
// MOTOR TEST SCRIPT
// Spins each motor for 5 seconds in order, then stops.
// REMOVE PROPS BEFORE RUNNING!
// Pinout:
//   M1 Front-Right  GPIO 33  CCW
//   M2 Rear-Right   GPIO 34  CW
//   M3 Rear-Left    GPIO 25  CCW
//   M4 Front-Left   GPIO 26  CW
// ═══════════════════════════════════════════════════════════

#include <DShotRMT.h>

DShotRMT escFR(GPIO_NUM_33, DSHOT300, false); // M1 Front-Right
DShotRMT escBR(GPIO_NUM_34, DSHOT300, false); // M2 Rear-Right
DShotRMT escBL(GPIO_NUM_25, DSHOT300, false); // M3 Rear-Left
DShotRMT escFL(GPIO_NUM_26, DSHOT300, false); // M4 Front-Left

const int TEST_THROTTLE = 200;  // DShot value ~10% - enough to spin, low risk
const int SPIN_MS       = 5000; // 5 seconds per motor
const int PAUSE_MS      = 1500; // pause between motors

void sendAll(int fr, int br, int bl, int fl) {
  escFR.sendThrottle(fr);
  escBR.sendThrottle(br);
  escBL.sendThrottle(bl);
  escFL.sendThrottle(fl);
}

void stopAll(int durationMs) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)durationMs) {
    sendAll(0, 0, 0, 0);
    delay(3);
  }
}

void spinMotor(int motorNum, int durationMs) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)durationMs) {
    int fr = (motorNum == 1) ? TEST_THROTTLE : 0;
    int br = (motorNum == 2) ? TEST_THROTTLE : 0;
    int bl = (motorNum == 3) ? TEST_THROTTLE : 0;
    int fl = (motorNum == 4) ? TEST_THROTTLE : 0;
    sendAll(fr, br, bl, fl);
    delay(3);
  }
}

void setup() {
  Serial.begin(115200);

  escFR.begin();
  escBR.begin();
  escBL.begin();
  escFL.begin();
  delay(300);

  // Arm ESCs
  Serial.println("Arming ESCs...");
  stopAll(2000);

  Serial.println("Starting motor test. PROPS OFF!");
  delay(1000);

  // M1 - Front-Right - GPIO 33
  Serial.println("M1 - Front-Right (GPIO 33) CCW");
  spinMotor(1, SPIN_MS);
  stopAll(PAUSE_MS);

  // M2 - Rear-Right - GPIO 34
  Serial.println("M2 - Rear-Right (GPIO 34) CW");
  spinMotor(2, SPIN_MS);
  stopAll(PAUSE_MS);

  // M3 - Rear-Left - GPIO 25
  Serial.println("M3 - Rear-Left (GPIO 25) CCW");
  spinMotor(3, SPIN_MS);
  stopAll(PAUSE_MS);

  // M4 - Front-Left - GPIO 26
  Serial.println("M4 - Front-Left (GPIO 26) CW");
  spinMotor(4, SPIN_MS);
  stopAll(PAUSE_MS);

  Serial.println("Motor test complete. Reboot to run again.");
}

void loop() {
  // All done - keep ESCs happy with zero throttle
  sendAll(0, 0, 0, 0);
  delay(3);
}
