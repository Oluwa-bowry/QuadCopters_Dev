// ═══════════════════════════════════════════════════════════
// MOTOR TEST SCRIPT
// Spins each motor for 5 seconds in order, then stops.
// REMOVE PROPS BEFORE RUNNING!
// Pinout (matches flight controller):
//   M1 Front-Right  GPIO 33  CCW  (escFR)
//   M2 Rear-Right   GPIO 34  CW   (escBR)
//   M3 Rear-Left    GPIO 25  CCW  (escBL)
//   M4 Front-Left   GPIO 26  CW   (escFL)
// NOTE: GPIO 34 is input-only on ESP32 - if M2 does not spin,
//       recheck your wiring or reassign to a valid output pin.
// ═══════════════════════════════════════════════════════════

#include <DShotRMT.h>

DShotRMT escFR(GPIO_NUM_33, DSHOT300, false); // M1 Front-Right
DShotRMT escBR(GPIO_NUM_34, DSHOT300, false); // M2 Rear-Right  ← GPIO34 is input-only on ESP32!
DShotRMT escBL(GPIO_NUM_25, DSHOT300, false); // M3 Rear-Left
DShotRMT escFL(GPIO_NUM_26, DSHOT300, false); // M4 Front-Left

const int TEST_THROTTLE = 200;  // DShot ~10%
const int SPIN_MS       = 5000; // 5 seconds per motor
const int PAUSE_MS      = 1500; // pause between motors

// Send a throttle value to every ESC individually by name
void stopAll(int durationMs) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)durationMs) {
    escFR.sendThrottle(0);
    escBR.sendThrottle(0);
    escBL.sendThrottle(0);
    escFL.sendThrottle(0);
    delay(3);
  }
}

void testMotor(const char* label, DShotRMT& esc, int durationMs) {
  Serial.print("Spinning: ");
  Serial.println(label);
  unsigned long start = millis();
  while (millis() - start < (unsigned long)durationMs) {
    // Only the target ESC gets throttle; others get 0
    escFR.sendThrottle((&esc == &escFR) ? TEST_THROTTLE : 0);
    escBR.sendThrottle((&esc == &escBR) ? TEST_THROTTLE : 0);
    escBL.sendThrottle((&esc == &escBL) ? TEST_THROTTLE : 0);
    escFL.sendThrottle((&esc == &escFL) ? TEST_THROTTLE : 0);
    delay(3);
  }
  Serial.println("Stopping...");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Initialising ESCs...");
  escFR.begin();
  escBR.begin();
  escBL.begin();
  escFL.begin();
  delay(300);

  // Arm sequence - send zero throttle for 2 s
  Serial.println("Arming ESCs (2s)...");
  stopAll(2000);

  Serial.println("=== MOTOR TEST START - PROPS OFF! ===");
  delay(500);

  testMotor("M1 Front-Right GPIO33 CCW", escFR, SPIN_MS);
  stopAll(PAUSE_MS);

  testMotor("M2 Rear-Right  GPIO34 CW ", escBR, SPIN_MS);
  stopAll(PAUSE_MS);

  testMotor("M3 Rear-Left   GPIO25 CCW", escBL, SPIN_MS);
  stopAll(PAUSE_MS);

  testMotor("M4 Front-Left  GPIO26 CW ", escFL, SPIN_MS);
  stopAll(PAUSE_MS);

  Serial.println("=== TEST COMPLETE - reboot to run again ===");
}

void loop() {
  // Keep ESCs happy with zero throttle indefinitely
  escFR.sendThrottle(0);
  escBR.sendThrottle(0);
  escBL.sendThrottle(0);
  escFL.sendThrottle(0);
  delay(3);
}
