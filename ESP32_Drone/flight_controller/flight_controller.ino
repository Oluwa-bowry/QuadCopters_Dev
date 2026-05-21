//THERE IS NO WARRANTY FOR THE SOFTWARE, TO THE EXTENT PERMITTED BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR 
//OTHER PARTIES PROVIDE THE SOFTWARE "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
//OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE SOFTWARE IS WITH THE CUSTOMER. SHOULD THE 
//SOFTWARE PROVE DEFECTIVE, THE CUSTOMER ASSUMES THE COST OF ALL NECESSARY SERVICING, REPAIR, OR CORRECTION EXCEPT TO THE EXTENT SET OUT UNDER THE HARDWARE WARRANTY IN THESE TERMS.


#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BMP280.h>
#include <DShotRMT.h>
#define MAVLINK_DIALECT common
#include <MAVLink.h>

HardwareSerial mav(2);             // CRSF receiver on UART2: RX=16, TX=17
HardwareSerial flowSerial(0);      // MTF-02P optical flow+LiDAR, UART0 RX=GPIO3 (Serial)
//Adafruit_BMP280 bmp280;

// CRSF Constants
#define CRSF_SYNC_BYTE    0xC8  // receiver address
#define CRSF_SYNC_BYTE_FC 0xEE  // flight controller address (ELRS sends this)
#define CRSF_MAX_FRAME_SIZE 64

uint8_t crsfBuffer[CRSF_MAX_FRAME_SIZE];
uint8_t crsfIndex = 0;

volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccXCalibration,AccYCalibration,AccZCalibration;

// Measured resting angle of the IMU on the frame - auto-set during startup
float rollLevelTrim  = 0.0f;
float pitchLevelTrim = 0.0f;

// DShot motor objects - pinout confirmed via ESC_PWM_Test
DShotRMT escFR(GPIO_NUM_33, DSHOT300, false); // M1 Front-Right
DShotRMT escBR(GPIO_NUM_32, DSHOT300, false); // M2 Rear-Right
DShotRMT escBL(GPIO_NUM_25, DSHOT300, false); // M3 Rear-Left
DShotRMT escFL(GPIO_NUM_26, DSHOT300, false); // M4 Front-Left

float PAngleRoll=2; float PAnglePitch=PAngleRoll;   // 2
float IAngleRoll=0.2; float IAnglePitch=IAngleRoll; // 0.2
float DAngleRoll=0.05; float DAnglePitch=DAngleRoll; // 0.001 // indoor


// dont touch 
float PRateRoll = 1.1; // 1.3 indoor
float IRateRoll = 1.5;  // 1.5
float DRateRoll = 0.0005; //0.0005 indoor

float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 4;
float IRateYaw = 0.3f;
float DRateYaw = 0.005f;

float PAngleYaw = 2.0f;
float IAngleYaw = 0.1f;
float DAngleYaw = 0.0f;

float yawAngle = 0.0f;       // integrated heading (deg)
float yawAngleTarget = 0.0f; // desired heading (deg)

uint32_t LoopTimer;
float t = 0.004f;      // minimum loop period (used for loop-rate enforcement only)
uint32_t prevLoopMicros = 0; // for measuring actual dt

// PWM-to-DShot converter (1000-2000 us -> 120-2000 DShot throttle)
// Starts at 120 so motors spin immediately from the bottom of the stick.
int pwmToDshot(float pwm) {
  if (pwm < 1000) pwm = 1000;
  if (pwm > 2000) pwm = 2000;
  return map((int)pwm, 1000, 2000, 120, 2000);
}

volatile int ReceiverValue[8] = {1500,1500,1000,1500,1000,1000,1000,1000}; // 8 CRSF channels mapped to 1000-2000 us
// [0]=Roll(CH1) [1]=Pitch(CH2) [2]=Throttle(CH3) [3]=Yaw(CH4)
// [4]=L2(CH5)   [5]=L1(CH6)   [6]=R1(CH7)       [7]=R2(CH8)

volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
volatile float KalmanGainPitch;
volatile float KalmanGainRoll;

int ThrottleIdle = 1140;   // minimum spin speed during flight
int ThrottleLanding = 1100; // gentle landing idle (below this = disarm)
int ThrottleCutOff = 1000; // sent to ESCs when disarmed

const int ArmThrottleMax = 1100;     // allow arming even if stick minimum is not exactly 1000
const int MinArmedDshot = 150;       // minimum DShot value that reliably spins most motors+

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PrevErrorAngleYaw = 0, PrevItermAngleYaw = 0;
volatile float PIDReturn[] = {0, 0, 0};

//Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;


float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

//bool bmp280Available = false;
//float bmpTemperatureC = 0.0f;
//float bmpPressurePa = 0.0f;
//float bmpRawAltitudeM = 0.0f;
//float bmpAbsoluteAltitudeM = 0.0f;
//float bmpRelativeAltitudeM = 0.0f;
//float bmpVerticalSpeedMps = 0.0f;
//float bmpAltitudeOffsetM = 0.0f;
//uint32_t lastBmpUpdateMs = 0;

bool altitudeHoldEnabled = false;
float altitudeHoldTargetM = 0.0f;
float altitudeHoldBaseThrottle = 1400.0f;
float altitudeHoldIntegrator = 0.0f;
float altitudeHoldPrevErrorM = 0.0f;
float altitudeHoldThrottleCorrection = 0.0f;
//bool bmpReferenceReady = false;
//bool bmpNewDataReady = false; // set true each time BMP delivers a fresh sample

const uint8_t ALT_HOLD_STATE_OFF = 0;
const uint8_t ALT_HOLD_STATE_READY = 1;
const uint8_t ALT_HOLD_STATE_ACTIVE = 2;

uint8_t altitudeHoldState = ALT_HOLD_STATE_OFF;

// ── 3-loop cascade altitude hold state (outer alt P → mid rate PID → inner accel PI) ──
float altHoldDesiredClimbRate = 0.0f;  // cm/s  — output of altitude P loop
float altHoldDesiredAccelG    = 0.0f;  // net g  — output of rate PID (Earth frame)
float altHoldRateIntegrator   = 0.0f;
float altHoldRatePrevError    = 0.0f;
float altHoldAccelIntegrator  = 0.0f;
float altHoldAccelPrevError   = 0.0f;
float imuVertAccelNetG        = 0.0f;  // filtered net vertical accel, Earth frame (g)
float imuVertAccelFiltG       = 0.0f;  // LPF running state

// ── MTF-02P optical flow + LiDAR globals (MAVLink via UART1 RX=GPIO4) ──
float ofVelocityX      = 0.0f;   // cm/s — optical flow body-X velocity
float ofVelocityY      = 0.0f;   // cm/s — optical flow body-Y velocity
float ofVelocityZ      = 0.0f;   // cm/s — vertical speed (LiDAR derivative)
uint8_t ofQuality      = 0;      // optical flow quality 0–255
float ofDistanceCm     = 0.0f;   // smoothed LiDAR distance from ground (cm)
float ofRelAltCm       = 0.0f;   // height above reference (= dist − refDist, cm)
float ofRefDistanceCm  = -1.0f;  // first valid reading = ground reference; -1 = unset
float ofLastDistanceCm = 0.0f;   // previous sample used for velocity derivative
uint32_t ofLastDistanceUs = 0;   // micros() at last valid reading (for Vz dt)
uint32_t ofLastDistanceMs = 0;   // millis() at last valid reading (for timeout)
bool ofDistValid       = false;  // true when LiDAR reports a value in 10–600 cm
bool ofFlowValid       = false;  // true when optical flow quality > 40
bool ofNewDataReady    = false;  // set true on each fresh LiDAR sample
// IIR filter coefficients (matching standalone MTF-02P code)
const float OF_ALPHA_XY    = 0.15f;
const float OF_ALPHA_Z     = 0.10f;
const float OF_ALPHA_DIST  = 0.20f;
const float OF_FOCAL_FACTOR = 11.4f;  // focal-length scale: flow_raw × height / focal → cm/s

// ── Position hold — optical flow velocity PI ──
float posHoldIntX   = 0.0f;  // body-X velocity integrator state
float posHoldIntY   = 0.0f;  // body-Y velocity integrator state
// Heavily-smoothed (alpha=0.05) velocity used only by position hold.
// Suppresses the high-frequency noise in ofVelocityX/Y (alpha=0.15)
// that caused fast roll/pitch oscillations at the 250 Hz loop rate.
float phVelX        = 0.0f;
float phVelY        = 0.0f;
bool  ofFlowNewData = false;  // true when a fresh OPTICAL_FLOW message was parsed

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// Arm/disarm state - controlled by CH5 switch (ReceiverValue[4])
// Arm:   CH5 > 1500  AND  throttle < 1050 (safety: arm only at low throttle)
// Disarm: CH5 < 1500  (instant, at any throttle)
bool isArmed = false;

// ── Best available altitude for alt-hold activation capture (metres) ──
static inline float currentAltHoldAltM() {
  if (ofDistValid && ofRefDistanceCm > 0.0f) return ofRelAltCm / 100.0f;
  return 0.0f; // BMP280 fallback removed
}

// ── Parse all pending MAVLink bytes from the MTF-02P (UART1) ──
void parseMavlinkStream() {
  mavlink_message_t msg;
  mavlink_status_t  status;
  while (flowSerial.available() > 0) {
    uint8_t b = (uint8_t)flowSerial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, b, &msg, &status)) {
      switch (msg.msgid) {

        case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
          mavlink_distance_sensor_t dist;
          mavlink_msg_distance_sensor_decode(&msg, &dist);
          float rawCm = (float)dist.current_distance;  // MAVLink spec: field is in cm
          if (rawCm >= 10.0f && rawCm <= 600.0f) {     // MTF-02P valid range
            ofDistValid = true;
            if (ofRefDistanceCm < 0.0f) ofRefDistanceCm = rawCm; // first = ground ref
            if (ofDistanceCm    < 1.0f) ofDistanceCm    = rawCm; // seed filter
            ofDistanceCm = ofDistanceCm * (1.0f - OF_ALPHA_DIST) + rawCm * OF_ALPHA_DIST;
            ofRelAltCm   = ofDistanceCm - ofRefDistanceCm;
            // Vertical velocity: finite-difference over a valid dt window
            uint32_t nowUs = micros();
            if (ofLastDistanceMs > 0) {
              float dtUs = (float)(nowUs - ofLastDistanceUs);
              if (dtUs > 5000.0f && dtUs < 200000.0f) {  // 5 ms – 200 ms
                float rawVz = (ofDistanceCm - ofLastDistanceCm) / (dtUs / 1000000.0f);
                ofVelocityZ = ofVelocityZ * (1.0f - OF_ALPHA_Z) + rawVz * OF_ALPHA_Z;
              }
            }
            ofLastDistanceCm = ofDistanceCm;
            ofLastDistanceUs = nowUs;
            ofLastDistanceMs = millis();
            ofNewDataReady   = true;
          } else {
            ofDistValid  = false;
            ofVelocityZ *= 0.5f;  // damp stale estimate
          }
          break;
        }

        case MAVLINK_MSG_ID_OPTICAL_FLOW: {
          mavlink_optical_flow_t flow;
          mavlink_msg_optical_flow_decode(&msg, &flow);
          ofQuality = flow.quality;
          // Height scale: LiDAR distance if valid
          float scaleH = ofDistValid ? ofDistanceCm : 0.0f;
          if (ofQuality > 40 && scaleH > 10.0f) {
            ofFlowValid = true;
            float rawVx = (flow.flow_x * scaleH) / OF_FOCAL_FACTOR;
            float rawVy = (flow.flow_y * scaleH) / OF_FOCAL_FACTOR;
            ofVelocityX = ofVelocityX * (1.0f - OF_ALPHA_XY) + rawVx * OF_ALPHA_XY;
            ofVelocityY = ofVelocityY * (1.0f - OF_ALPHA_XY) + rawVy * OF_ALPHA_XY;
            // Extra heavy smoothing for position hold (cuts noise ~3× more)
            phVelX = phVelX * 0.95f + ofVelocityX * 0.05f;
            phVelY = phVelY * 0.95f + ofVelocityY * 0.05f;
            ofFlowNewData = true;
          } else {
            ofFlowValid  = false;
            ofVelocityX *= 0.5f;
            ofVelocityY *= 0.5f;
            phVelX      *= 0.90f;
            phVelY      *= 0.90f;
          }
          break;
        }

        default:
          break;
      }
    }
  }
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState + (t*KalmanInput);
  KalmanUncertainty=KalmanUncertainty + (t*t*4*4); //here 4 is the vairnece of IMU i.e 4 deg/s
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3); //std deviation of error is 3 deg
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

// Decode a complete CRSF frame and populate ReceiverValue[]
void processCRSFFrame(uint8_t *frame, uint8_t size) {
  uint8_t type = frame[2];
  // RC Channels Packed frame (type 0x16), payload = 22 bytes
  if (type == 0x16 && size >= 26) {
    const uint8_t *payload = &frame[3];
    for (int i = 0; i < 8; i++) {
      uint16_t bitOfs = i * 11;
      uint16_t byteOfs = bitOfs / 8;
      uint8_t  bitRem  = bitOfs % 8;
      uint32_t val = ((uint32_t)payload[byteOfs] |
                     ((uint32_t)payload[byteOfs + 1] << 8) |
                     ((uint32_t)payload[byteOfs + 2] << 16)) >> bitRem;
      uint16_t crsfVal = val & 0x7FF;
      // Map CRSF 11-bit range (172-1811) to PWM range (1000-2000 us)
      ReceiverValue[i] = map((long)crsfVal, 172, 1811, 1000, 2000);
    }
  }
}

// Parse incoming bytes and assemble CRSF frames
void parseCRSF(uint8_t byteIn) {
  static bool    receiving = false;
  static uint8_t length    = 0;

  if (!receiving) {
    if (byteIn == CRSF_SYNC_BYTE || byteIn == CRSF_SYNC_BYTE_FC) {
      receiving = true;
      crsfIndex = 0;
      crsfBuffer[crsfIndex++] = byteIn;
    }
    return;
  }

  crsfBuffer[crsfIndex++] = byteIn;

  if (crsfIndex == 2) {
    length = byteIn;
    if (length > CRSF_MAX_FRAME_SIZE - 2) {
      receiving = false;
      return;
    }
  }

  if (crsfIndex >= (uint8_t)(length + 2)) {
    processCRSFFrame(crsfBuffer, crsfIndex);
    receiving = false;
    crsfIndex = 0;
  }
}

void gyro_signals(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29; //*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
  float Pterm = P * Error;
  float Iterm = PrevIterm +( I * (Error + PrevError) * (t/2));
  if (Iterm > 400)
  {
    Iterm = 400;
  }
  else if (Iterm < -400)
  {
  Iterm = -400;
  }
  float Dterm = D *( (Error - PrevError)/t);
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400)
  {
    PIDOutput = 400;
  }
  else if (PIDOutput < -400)
  {
    PIDOutput = -400;
  }
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

// ═══════════════════════════════════════════════════════════
// ALTITUDE HOLD - set to 1 to enable, 0 to disable
// Altitude hold: CH6 (ReceiverValue[5] > 1500) activates hold at current altitude
// ═══════════════════════════════════════════════════════════
#define ALT_HOLD_ENABLE 1

//void updateBMP280(uint32_t nowMs)
//{
//  if (!bmp280Available) {
//    return;
//  }
//
//  // Poll at 200ms — BMP280 produces a new sample every ~170ms at STANDBY_MS_125,
//  // so 200ms guarantees we always read a fresh sample without wasting cycles.
//  const uint32_t samplePeriodMs = 200;
//  if (nowMs - lastBmpUpdateMs < samplePeriodMs) {
//    return;
//  }
//
//  float previousAltitude = bmpAbsoluteAltitudeM;
//  uint32_t previousUpdateMs = lastBmpUpdateMs;
//  lastBmpUpdateMs = nowMs;
//
//  // Non-blocking BMP280 reads with minimal delay to avoid loop stalling
//  bmpTemperatureC = bmp280.readTemperature();
//  bmpPressurePa = bmp280.readPressure();
//  bmpRawAltitudeM = bmp280.readAltitude(1013.25f);
//  bmpNewDataReady = true; // signal altitude hold to update
//
//  // Heavier LPF on altitude to reduce barometer noise.
//  if (previousUpdateMs == 0) {
//    bmpAbsoluteAltitudeM = bmpRawAltitudeM;
//  } else {
//    bmpAbsoluteAltitudeM = (0.95f * bmpAbsoluteAltitudeM) + (0.05f * bmpRawAltitudeM);
//  }
//
//  if (previousUpdateMs != 0) {
//    float deltaTime = (nowMs - previousUpdateMs) / 1000.0f;
//    if (deltaTime > 0.0f) {
//      float rawVerticalSpeed = (bmpAbsoluteAltitudeM - previousAltitude) / deltaTime;
//      bmpVerticalSpeedMps = (0.9f * bmpVerticalSpeedMps) + (0.1f * rawVerticalSpeed); // heavy LPF on vspeed
//    }
//  }
//
//  if (!isArmed && ReceiverValue[2] < 1050) {
//    bmpAltitudeOffsetM = (0.98f * bmpAltitudeOffsetM) + (0.02f * bmpAbsoluteAltitudeM);
//  }
//
//  bmpRelativeAltitudeM = bmpAbsoluteAltitudeM - bmpAltitudeOffsetM;
//}

void resetAltitudeHold()
{
  altitudeHoldEnabled = false;
  altitudeHoldTargetM = currentAltHoldAltM();
  altitudeHoldBaseThrottle = constrain(ReceiverValue[2], 1250, 1700);
  altitudeHoldIntegrator = 0.0f;
  altitudeHoldPrevErrorM = 0.0f;
  altitudeHoldThrottleCorrection = 0.0f;
  // 3-loop cascade state
  altHoldDesiredClimbRate = 0.0f;
  altHoldDesiredAccelG    = 0.0f;
  altHoldRateIntegrator   = 0.0f;
  altHoldRatePrevError    = 0.0f;
  altHoldAccelIntegrator  = 0.0f;
  altHoldAccelPrevError   = 0.0f;
}

void setAltitudeHoldState(uint8_t newState)
{
  if (altitudeHoldState == newState) {
    return;
  }

  altitudeHoldState = newState;

  if (newState != ALT_HOLD_STATE_ACTIVE) {
    resetAltitudeHold();
  } else {
    altitudeHoldEnabled = true;
    float curAlt = currentAltHoldAltM();
    // Normal altitude hold at current altitude
    altitudeHoldTargetM      = curAlt;
    altitudeHoldBaseThrottle = constrain(ReceiverValue[2], 1250, 1700);
    altitudeHoldIntegrator = 0.0f;
    altitudeHoldPrevErrorM = 0.0f;
    altitudeHoldThrottleCorrection = 0.0f;
  }
}

//void calibrateBMP280Reference()
//{
//  if (!bmp280Available) {
//    return;
//  }
//
//  const int baselineSamples = 32;
//  float altitudeSum = 0.0f;
//  float tempSum = 0.0f;
//  float pressureSum = 0.0f;
//
//  for (int i = 0; i < baselineSamples; i++) {
//    tempSum += bmp280.readTemperature();
//    delay(5);
//    pressureSum += bmp280.readPressure();
//    delay(5);
//    altitudeSum += bmp280.readAltitude(1013.25f);
//    delay(35);
//  }
//
//  bmpTemperatureC = tempSum / baselineSamples;
//  bmpPressurePa = pressureSum / baselineSamples;
//  bmpRawAltitudeM = altitudeSum / baselineSamples;
//  bmpAbsoluteAltitudeM = bmpRawAltitudeM;
//  bmpAltitudeOffsetM = bmpAbsoluteAltitudeM;
//  bmpRelativeAltitudeM = 0.0f;
//  bmpVerticalSpeedMps = 0.0f;
//  altitudeHoldTargetM = 0.0f;
//  altitudeHoldIntegrator = 0.0f;
//  altitudeHoldPrevErrorM = 0.0f;
//  altitudeHoldThrottleCorrection = 0.0f;
//  bmpReferenceReady = true;
//  lastBmpUpdateMs = millis();
//}

void updateAltitudeHold(float dt, uint32_t nowMs)
{
#if ALT_HOLD_ENABLE == 0
  (void)dt;
  (void)nowMs;
  setAltitudeHoldState(ALT_HOLD_STATE_OFF);
  return;
#endif

  bool altitudeSwitchOn = (ReceiverValue[5] > 1500) || (ReceiverValue[7] > 1500);
  bool lidarReady  = ofDistValid && ofRefDistanceCm > 0.0f;
  bool sensorReady = lidarReady; // BMP280 removed; LiDAR only
  bool canArmHold = sensorReady && isArmed;

  if (!altitudeSwitchOn || !canArmHold) {
    setAltitudeHoldState(ALT_HOLD_STATE_OFF);
    return;
  }

  // Wait in READY until pilot throttle is above spool zone.
  if (ReceiverValue[2] < 1200) {
    setAltitudeHoldState(ALT_HOLD_STATE_READY);
    return;
  }

  if (altitudeHoldState != ALT_HOLD_STATE_ACTIVE) {
    setAltitudeHoldState(ALT_HOLD_STATE_ACTIVE);
  }

  altitudeHoldEnabled = true;

  // ══════════════════════════════════════════════════════════════════
  // OUTER P LOOP + MID RATE PI LOOP
  // Altitude source: LiDAR (MTF-02P) only (BMP280 removed).
  // Gated on ofNewDataReady (LiDAR ~10–50 Hz).
  // ══════════════════════════════════════════════════════════════════

  // LiDAR is the only altitude source
  float    currentAltM    = ofRelAltCm / 100.0f;
  float    currentRateMps = ofVelocityZ / 100.0f;
  bool     newData        = ofNewDataReady;
  uint32_t lastSensorMs   = ofLastDistanceMs;

  if (newData) {
    ofNewDataReady = false;

    // Use wall-clock time between altitude-hold updates, NOT the 4 ms main-loop dt.
    static uint32_t lastAltHoldUpdateMs = 0;
    float altDt = (lastAltHoldUpdateMs > 0)
                    ? constrain((nowMs - lastAltHoldUpdateMs) / 1000.0f, 0.005f, 0.5f)
                    : dt;
    lastAltHoldUpdateMs = nowMs;

    // Stick override: nudges altitude target with large throttle deflection.
    float stickOffset = ReceiverValue[2] - altitudeHoldBaseThrottle;
    if (fabsf(stickOffset) > 35.0f) {
      altitudeHoldTargetM += stickOffset * 0.0012f * altDt;
    }

    // ── OUTER P LOOP: altitude error (cm) → desired climb rate (cm/s) ──
    const float P_ALT = 0.6f;
    float altErrorCm = (altitudeHoldTargetM - currentAltM) * 100.0f;
    const float climbRateCap = 200.0f;
    altHoldDesiredClimbRate = constrain(P_ALT * altErrorCm, -climbRateCap, climbRateCap);

    // ── MID RATE PI LOOP: rate error (cm/s) → throttle correction (PWM) ──
    const float P_RATE_THR = 1.0f;
    const float I_RATE_THR = 0.25f;
    float measuredClimbRateCmps = currentRateMps * 100.0f;
    float rateError = altHoldDesiredClimbRate - measuredClimbRateCmps;
    altHoldRateIntegrator += rateError * altDt;
    altHoldRateIntegrator = constrain(altHoldRateIntegrator, -400.0f, 400.0f);
    altHoldRatePrevError = rateError;
    altitudeHoldThrottleCorrection = (P_RATE_THR * rateError)
                                   + (I_RATE_THR * altHoldRateIntegrator);
    altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, -220.0f, 220.0f);

  } else {
    // Waiting for fresh sensor sample — check for timeout
    if (nowMs - lastSensorMs > 500) {
      setAltitudeHoldState(ALT_HOLD_STATE_READY);
      return;
    }
    // Hold last throttle correction until next sensor update
  }

  InputThrottle = altitudeHoldBaseThrottle + altitudeHoldThrottleCorrection;
}

// ═══════════════════════════════════════════════════════════
// MOTOR TEST  -  set to 1 to run, 0 for normal flight
// Spins each motor for 2 s at low throttle then stops.
// Watch which motor spins to confirm orientation.
// REMOVE PROPS BEFORE RUNNING!
// ═══════════════════════════════════════════════════════════
#define MOTOR_TEST_ENABLE 0

// ═══════════════════════════════════════════════════════════
// MANUAL THROTTLE (NO RECEIVER) - set to 1 for testing without CRSF
// Use Serial commands: + to increase, - to decrease, s to stop
// REMOVE PROPS BEFORE RUNNING!
// ═══════════════════════════════════════════════════════════
#define MANUAL_THROTTLE_MODE 0

int manualThrottle = 1000; // starts disarmed (< 1050 = DShot 0). Use '+' to increase.

void handleManualThrottle() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '+') {
      manualThrottle += 50;
      if (manualThrottle > 1800) manualThrottle = 1800;
    }
    else if (cmd == '-') {
      manualThrottle -= 50;
      if (manualThrottle < 1050) manualThrottle = 1050;
    }
    else if (cmd == 's') {
      manualThrottle = 1000;  // below 1050 = disarm
    }
  }
  ReceiverValue[2] = manualThrottle;
  ReceiverValue[0] = 1500; // neutral roll
  ReceiverValue[1] = 1500; // neutral pitch
  ReceiverValue[3] = 1500; // neutral yaw
}

void runMotorTest()
{
  const int TEST_THROTTLE = 100; // DShot value (0-2000), ~5% - enough to spin, low risk
  const int SPIN_MS       = 2000;
  const int PAUSE_MS      = 1000;

  // Helper: stop all
  auto stopAll = [&]() {
    for (int i = 0; i < 100; i++) {
      escFR.sendThrottle(0);
      escBR.sendThrottle(0);
      escBL.sendThrottle(0);
      escFL.sendThrottle(0);
      delay(3);
    }
  };

  // Helper: run one motor for SPIN_MS
  auto spinMotor = [&](const char* name, int m) {
    unsigned long start = millis();
    while (millis() - start < (unsigned long)SPIN_MS) {
      escFR.sendThrottle(m == 1 ? TEST_THROTTLE : 0);
      escBR.sendThrottle(m == 2 ? TEST_THROTTLE : 0);
      escBL.sendThrottle(m == 3 ? TEST_THROTTLE : 0);
      escFL.sendThrottle(m == 4 ? TEST_THROTTLE : 0);
      delay(3);
    }
    stopAll();
    delay(PAUSE_MS);
  };
  delay(3000); // safety pause

  spinMotor("M1 Front-Right (pin 32) CCW", 1);
  spinMotor("M2 Rear-Right  (pin 26) CW ", 2);
  spinMotor("M3 Rear-Left   (pin 33) CCW", 3);
  spinMotor("M4 Front-Left  (pin 25) CW ", 4);
  while (true) { delay(1000); } // halt - require reboot after test
}

// ═══════════════════════════════════════════════════════════════════════
// POSITION HOLD  —  nulls horizontal drift using optical flow velocity
//
// Active when: armed + altitude hold ACTIVE + optical flow quality good.
// Injects a lean-angle bias into DesiredAngleRoll/Pitch (called BEFORE
// the angle PIDs so the full cascade sees the corrected target).
//
// ⚠  SIGN TUNING: if the drone moves in the WRONG direction on first
//    test, flip PH_PITCH_SIGN and/or PH_ROLL_SIGN to +1.0f.
// ═══════════════════════════════════════════════════════════════════════
void updatePositionHold(float dt) {
  // ⚠  If the drone corrects in the WRONG direction, flip the sign:
  const float PH_PITCH_SIGN = -1.0f;  // -1: forward drift → pitch back
  const float PH_ROLL_SIGN  = -1.0f;  // -1: rightward drift → roll left

  bool active = isArmed
                && (altitudeHoldState == ALT_HOLD_STATE_ACTIVE)
                && ofFlowValid;  // position hold active during ascent to prevent drift

  if (!active) {
    posHoldIntX *= 0.95f;
    posHoldIntY *= 0.95f;
    return;
  }

  // Pilot stick override: let the pilot steer freely
  bool stickActive = (fabsf((float)ReceiverValue[0] - 1500.0f) > 50.0f)
                  || (fabsf((float)ReceiverValue[1] - 1500.0f) > 50.0f);
  if (stickActive) {
    posHoldIntX *= 0.90f;
    posHoldIntY *= 0.90f;
    return;
  }

  // ── Velocity PI using phVelX/Y (alpha=0.05, ~3× smoother than ofVelocityX/Y) ──
  const float P_PH = 0.04f;
  const float I_PH = 0.006f;

  posHoldIntX = constrain(posHoldIntX + phVelX * dt, -80.0f, 80.0f);
  posHoldIntY = constrain(posHoldIntY + phVelY * dt, -80.0f, 80.0f);

  float pitchCorr = PH_PITCH_SIGN * (P_PH * phVelX + I_PH * posHoldIntX);
  float rollCorr  = PH_ROLL_SIGN  * (P_PH * phVelY + I_PH * posHoldIntY);

  pitchCorr = constrain(pitchCorr, -8.0f, 8.0f);
  rollCorr  = constrain(rollCorr,  -8.0f, 8.0f);

  DesiredAnglePitch += pitchCorr;
  DesiredAngleRoll  += rollCorr;
}

void setup(void) {

#if MOTOR_TEST_ENABLE
  // ESCs must be initialised before test
  escFR.begin(); escBL.begin(); escFL.begin(); escBR.begin();
  delay(300);
  for (int i = 0; i < 400; i++) { // arm
    escFR.sendThrottle(0); escBL.sendThrottle(0);
    escFL.sendThrottle(0); escBR.sendThrottle(0);
    delay(3);
  }
  runMotorTest(); // never returns
#endif

int led_time=100;
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);

  mav.begin(921600, SERIAL_8N1, 16, 17);      // CRSF receiver on UART2: RX=16, TX=17
  flowSerial.begin(115200, SERIAL_8N1, 3, 1);  // MTF-02P optical flow+LiDAR on UART0: RX=GPIO3, TX=GPIO1
  delay(100);
  
  Wire.begin();
  delay(250);

  // BMP280 init commented out
  //Wire.setClock(100000);
  //delay(10);
  //bmp280Available = bmp280.begin(0x76);
  //if (bmp280Available) {
  //  bmp280.setSampling(
  //    Adafruit_BMP280::MODE_NORMAL,
  //    Adafruit_BMP280::SAMPLING_X2,
  //    Adafruit_BMP280::SAMPLING_X16,
  //    Adafruit_BMP280::FILTER_X16,
  //    Adafruit_BMP280::STANDBY_MS_125
  //  );
  //  calibrateBMP280Reference();
  //}

  Wire.setClock(400000);  // 400kHz for MPU6050

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // ── Live IMU calibration FIRST (before ESC arming) - keep drone flat & still ──
  {
    const int CAL_SAMPLES = 2000;
    double sumGX=0, sumGY=0, sumGZ=0;
    double sumAX=0, sumAY=0, sumAZ=0;
    for (int i = 0; i < CAL_SAMPLES; i++) {
      Wire.beginTransmission(0x68);
      Wire.write(0x1B); Wire.write(0x8);
      Wire.endTransmission();
      Wire.beginTransmission(0x68);
      Wire.write(0x43);
      Wire.endTransmission();
      Wire.requestFrom(0x68, 6);
      sumGX += (int16_t)(Wire.read()<<8 | Wire.read());
      sumGY += (int16_t)(Wire.read()<<8 | Wire.read());
      sumGZ += (int16_t)(Wire.read()<<8 | Wire.read());

      Wire.beginTransmission(0x68);
      Wire.write(0x1C); Wire.write(0x10);
      Wire.endTransmission();
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission();
      Wire.requestFrom(0x68, 6);
      sumAX += (int16_t)(Wire.read()<<8 | Wire.read());
      sumAY += (int16_t)(Wire.read()<<8 | Wire.read());
      sumAZ += (int16_t)(Wire.read()<<8 | Wire.read());
      delay(1);
    }
    RateCalibrationRoll  = (sumGX / CAL_SAMPLES) / 65.5f;
    RateCalibrationPitch = (sumGY / CAL_SAMPLES) / 65.5f;
    RateCalibrationYaw   = (sumGZ / CAL_SAMPLES) / 65.5f;

    float rawAX = (float)(sumAX / CAL_SAMPLES) / 4096.0f;
    float rawAY = (float)(sumAY / CAL_SAMPLES) / 4096.0f;
    float rawAZ = (float)(sumAZ / CAL_SAMPLES) / 4096.0f;
    rollLevelTrim  = atan(rawAY / sqrt(rawAX*rawAX + rawAZ*rawAZ)) * 57.29f;
    pitchLevelTrim = -atan(rawAX / sqrt(rawAY*rawAY + rawAZ*rawAZ)) * 57.29f;

    AccXCalibration = rawAX;
    AccYCalibration = rawAY;
    AccZCalibration = rawAZ - 1.0f; // remove 1g
  }

  // ── DShot ESC init (AFTER calibration to avoid vibration corruption) ──
  escFR.begin();
  escBL.begin();
  escFL.begin();
  escBR.begin();
  delay(300);

  // ── ESC arming: send zero throttle for ~1.2 s ──
  for (int i = 0; i < 400; i++) {
    escFR.sendThrottle(0);
    escBL.sendThrottle(0);
    escFL.sendThrottle(0);
    escBR.sendThrottle(0);
    delay(3);
  }

  // ── Wait for ESCs to complete initialization ──
  {
    unsigned long waitStart = millis();
    while (millis() - waitStart < 5000) {
      escFR.sendThrottle(0);
      escBL.sendThrottle(0);
      escFL.sendThrottle(0);
      escBR.sendThrottle(0);
      delay(3);
    }
  }

  digitalWrite(15, LOW);
  digitalWrite(15, HIGH);
  delay(500);
  digitalWrite(15, LOW);
  delay(500);

  prevLoopMicros = micros();
  LoopTimer = micros();
}

void loop(void) {
  // Measure actual elapsed time for this iteration
  uint32_t nowMicros = micros();
  uint32_t nowMs = millis();
  float dt = (nowMicros - prevLoopMicros) / 1000000.0f;
  if (dt <= 0.0f || dt > 0.05f) dt = 0.004f; // sanity clamp: reject 0 or >50ms
  prevLoopMicros = nowMicros;

  //updateBMP280(nowMs); // BMP280 disabled
  parseMavlinkStream();

#if MANUAL_THROTTLE_MODE
  handleManualThrottle();
#else
  // Read and parse incoming CRSF data
  static uint32_t crsfByteCount = 0;
  while (mav.available()) {
    crsfByteCount++;
    parseCRSF((uint8_t)mav.read());
  }
#endif

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;

  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29;
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;

  // Complementary filter
  float tau = 6.0f;
  float alpha = tau / (tau + dt);
  complementaryAngleRoll  = alpha * (complementaryAngleRoll  + RateRoll  * dt) + (1.0f - alpha) * AngleRoll;
  complementaryAnglePitch = alpha * (complementaryAnglePitch + RatePitch * dt) + (1.0f - alpha) * AnglePitch;
  complementaryAngleRoll  = (complementaryAngleRoll  >  20) ?  20 : ((complementaryAngleRoll  < -20) ? -20 : complementaryAngleRoll);
  complementaryAnglePitch = (complementaryAnglePitch >  20) ?  20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);

  // ── Earth-frame net vertical acceleration ──
  {
    float pitchRad = complementaryAnglePitch * (3.14159265f / 180.0f);
    float rollRad  = complementaryAngleRoll  * (3.14159265f / 180.0f);
    float az_raw = -AccX * sinf(pitchRad)
                 + AccY * cosf(pitchRad) * sinf(rollRad)
                 + AccZ * cosf(pitchRad) * cosf(rollRad)
                 - 1.0f;
    imuVertAccelFiltG = 0.7f * imuVertAccelFiltG + 0.3f * az_raw;
    imuVertAccelNetG  = imuVertAccelFiltG;
  }

  DesiredAngleRoll  = 0.1f*(ReceiverValue[0]-1500);
  DesiredAnglePitch = 0.1f*(ReceiverValue[1]-1500);
  updatePositionHold(dt);
  InputThrottle=ReceiverValue[2];

  // Yaw heading hold
  yawAngle += RateYaw * dt;
  float rawYawStick = 0.15f * (ReceiverValue[3] - 1500);
  if (fabsf(rawYawStick) >= 3.0f) {
    yawAngleTarget += (-rawYawStick) * dt;
  }
  float ErrorAngleYaw = yawAngleTarget - yawAngle;
  float PtermAngleYaw = PAngleYaw * ErrorAngleYaw;
  float ItermAngleYaw = PrevItermAngleYaw + (IAngleYaw * (ErrorAngleYaw + PrevErrorAngleYaw) * (dt / 2));
  ItermAngleYaw = constrain(ItermAngleYaw, -400.0f, 400.0f);
  float DtermAngleYaw = DAngleYaw * ((ErrorAngleYaw - PrevErrorAngleYaw) / dt);
  float PIDOutputAngleYaw = PtermAngleYaw + ItermAngleYaw + DtermAngleYaw;
  PIDOutputAngleYaw = constrain(PIDOutputAngleYaw, -400.0f, 400.0f);
  DesiredRateYaw = PIDOutputAngleYaw;
  PrevErrorAngleYaw = ErrorAngleYaw;
  PrevItermAngleYaw = ItermAngleYaw;

  // Roll angle PID
  ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (dt / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / dt);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  // Pitch angle PID
  ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (dt / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / dt);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // Rate errors
  ErrorRateRoll  = DesiredRateRoll  - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw   = DesiredRateYaw   - RateYaw;

  // Roll rate PID
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (dt / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / dt);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Pitch rate PID
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (dt / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / dt);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Yaw rate PID
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (dt / 2));
  ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / dt);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  updateAltitudeHold(dt, nowMs);

  if (InputThrottle > 1800) {
    InputThrottle = 1800;
  }

  MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw); // front right CCW
  MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw); // rear  right CW
  MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw); // rear  left  CCW
  MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw); // front left  CW

  if (MotorInput1 > 2000) MotorInput1 = 1999;
  if (MotorInput2 > 2000) MotorInput2 = 1999;
  if (MotorInput3 > 2000) MotorInput3 = 1999;
  if (MotorInput4 > 2000) MotorInput4 = 1999;

  if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

  // ── Arm / Disarm via CH5 switch ──
  if (ReceiverValue[4] > 1500 && ReceiverValue[2] < ArmThrottleMax && !isArmed) {
    isArmed = true;
  }
  if (ReceiverValue[4] < 1500 && isArmed && ReceiverValue[2] < 1200) {
    isArmed = false;
  }

  // ── Motor output ──
  if (!isArmed) {
    PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
    PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
    PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;
    PrevItermAngleRoll=0; PrevItermAnglePitch=0;
    PrevErrorAngleYaw=0;  PrevItermAngleYaw=0;
    yawAngle=0.0f; yawAngleTarget=0.0f;
    altHoldRateIntegrator=0.0f; altHoldAccelIntegrator=0.0f;
    altHoldDesiredClimbRate=0.0f; altHoldDesiredAccelG=0.0f;
    altHoldRatePrevError=0.0f; altHoldAccelPrevError=0.0f;
    imuVertAccelFiltG=0.0f;
    escFR.sendThrottle(0);
    escBR.sendThrottle(0);
    escBL.sendThrottle(0);
    escFL.sendThrottle(0);
  } else {
    escFR.sendThrottle(pwmToDshot(MotorInput1)); // pin 33 - front right CCW
    escBR.sendThrottle(pwmToDshot(MotorInput2)); // pin 35 - rear  right CW
    escBL.sendThrottle(pwmToDshot(MotorInput3)); // pin 25 - rear  left  CCW
    escFL.sendThrottle(pwmToDshot(MotorInput4)); // pin 26 - front left  CW
  }

  // Enforce loop rate (250 Hz = 4ms period)
  while (micros() - LoopTimer < (t*1000000)) {
    // busy wait
  }
  LoopTimer = micros();
}
