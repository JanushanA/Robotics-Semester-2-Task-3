// Arduino Uno rescue robot: QTR line tracking, MPU feedback and SD movement programs.

// Log markers connect QTR sensing, acceleration processing and autonomous movement to the three logs.
#define ENABLE_LCD 1
#define ENABLE_SD 1
#define SET_RTC_FROM_COMPILE_TIME 0

#include <Wire.h>
#include <string.h>
#include <math.h>
#if ENABLE_SD
#include <SPI.h>
#include <SD.h>
#endif

// Sensors 7/5/3/1 use individual black thresholds measured in microseconds.
const byte QTR_PINS[4] = {4, 7, A0, A1};
const unsigned int QTR_BLACK[4] = {700, 420, 800, 900};
const unsigned int QTR_TIMEOUT_US = 4500;
const byte SONAR_PIN = 2;
const byte MPU_ADDR = 0x69;
const byte RTC_ADDR = 0x68;
const byte LCD_ADDR = 0x27;
const byte SD_CS = 10;
const bool LEFT_REVERSED = false;
const bool RIGHT_REVERSED = true;

// Choose the forward axis and signs to match a flat-mounted MPU.
const byte MPU_FORWARD_AXIS = 0; // 0=X, 1=Y
const float MPU_FORWARD_SIGN = 1.0f;
const float MPU_CW_SIGN = 1.0f; // Flip to -1 if right turns report negative gyro Z.
// Line error sets the requested turn rate; gyro feedback adjusts steering within PWM limits.
const float LINE_KP = 0.020f;
const float RATE_KP = 0.11f;
const float TARGET_YAW_GAIN = 0.055f;
const int MAX_STEER = 32;
const int PIVOT_PWM = 75;
const unsigned long SEARCH_TIMEOUT_MS = 1200;

// Keep these defaults when an SD setting is missing or invalid.
struct MissionConfig {
  byte cruisePwm = 60;
  byte obstacleStopCm = 25;
  bool preferLeft = true;
  bool danceAtBoot = false;
} mission;

// RunMode chooses the overall behaviour; NavMode tracks progress within line following.
enum RunMode : byte { RUN_LINE, RUN_DANCE, RUN_IDLE };
RunMode runMode = RUN_IDLE;

enum EventCode : byte {
  EV_BOOT = 1,
  EV_LINE_LOST,
  EV_LINE_FOUND,
  EV_JUNCTION,
  EV_JUNCTION_EXIT,
  EV_SEARCH_FAIL,
  EV_OBSTACLE,
  EV_CLEAR,
  EV_DANCE_BEGIN,
  EV_DANCE_END,
  EV_DANCE_ERROR,
  EV_TURN_TIMEOUT
};

// Record when a software event occurred, its category and one supporting value.
struct RobotEvent {
  unsigned long elapsedMs;
  byte eventCode;
  int16_t eventDetail;
};

// Bounded circular queue preserves recent events with fixed SRAM usage.
class EventRing {
  RobotEvent events[8];
  byte oldestEventIndex = 0;
  byte eventCount = 0;
  unsigned int droppedEventCount = 0;

public:
  void push(EventCode eventCode, int eventDetail = 0) {
    // Wrap around eight slots, overwriting and counting the oldest event when full.
    byte writeIndex = (oldestEventIndex + eventCount) % 8;
    if (eventCount == 8) {
      oldestEventIndex = (oldestEventIndex + 1) % 8;
      droppedEventCount++;
    } else {
      eventCount++;
    }
    events[writeIndex].elapsedMs = millis();
    events[writeIndex].eventCode = eventCode;
    events[writeIndex].eventDetail = eventDetail;
  }
  // Peek reads the oldest event; pop removes it, both in constant time.
  bool peek(RobotEvent &oldestEvent) const {
    if (!eventCount) {
      return false;
    }
    oldestEvent = events[oldestEventIndex];
    return true;
  }
  void pop() {
    if (eventCount) {
      oldestEventIndex = (oldestEventIndex + 1) % 8;
      eventCount--;
    }
  }
  unsigned int lost() const {
    return droppedEventCount;
  }
} eventRing;

// Check elapsed time without waiting; unsigned subtraction handles timer wraparound.
class TaskTimer {
  unsigned long lastRunMs = 0;

public:
  void reset(unsigned long currentTimeMs) {
    lastRunMs = currentTimeMs;
  }
  bool due(unsigned long intervalMs, unsigned long currentTimeMs) {
    if (currentTimeMs - lastRunMs < intervalMs) {
      return false;
    }
    lastRunMs = currentTimeMs;
    return true;
  }
};

class DriveController {
  int leftPwmCommand = 0;
  int rightPwmCommand = 0;
  // Signed PWM controls direction and power; polarity reversal compensates for motor mounting.
  void motor(byte forwardPin, byte reversePin, int signedPwm, bool reversePolarity) {
    signedPwm = constrain(signedPwm, -255, 255);
    if (reversePolarity) {
      signedPwm = -signedPwm;
    }
    if (signedPwm > 0) {
      analogWrite(forwardPin, signedPwm);
      digitalWrite(reversePin, LOW);
    } else if (signedPwm < 0) {
      digitalWrite(forwardPin, LOW);
      analogWrite(reversePin, -signedPwm);
    } else {
      digitalWrite(forwardPin, LOW);
      digitalWrite(reversePin, LOW);
    }
  }

public:
  void begin() {
    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    stop();
  }
  void set(int requestedLeftPwm, int requestedRightPwm) {
    leftPwmCommand = constrain(requestedLeftPwm, -110, 110);
    rightPwmCommand = constrain(requestedRightPwm, -110, 110);
    motor(3, 5, leftPwmCommand, LEFT_REVERSED);
    motor(6, 9, rightPwmCommand, RIGHT_REVERSED);
  }
  void stop() {
    set(0, 0);
  }
  int leftPWM() const {
    return leftPwmCommand;
  }
  int rightPWM() const {
    return rightPwmCommand;
  }
  // Infer forward intent from PWM commands; this does not measure actual movement.
  bool translating() const {
    return (leftPwmCommand + rightPwmCommand) > 35;
  }
} drive;

// LOG1_A | Measure RC discharge time; 4500 us means timeout, which helped isolate wiring faults, no longer normalised.
class QTRArray {
  unsigned int dischargeTimesUs[4] = {0, 0, 0, 0};
  byte detectedBlackCount = 0;

public:
  void update() {
    // Charge all four sensor outputs before timing their discharge.
    for (byte sensorIndex = 0; sensorIndex < 4; sensorIndex++) {
      pinMode(QTR_PINS[sensorIndex], OUTPUT);
      digitalWrite(QTR_PINS[sensorIndex], HIGH);
    }
    // Allow 10 us for charging; darker surfaces usually discharge more slowly.
    delayMicroseconds(10);
    unsigned long measurementStartUs = micros();
    // Track completed channels so each falling signal is recorded once.
    bool measurementComplete[4] = {false, false, false, false};
    byte remainingSensors = 4;
    // Release the outputs and poll them against one shared start time.
    for (byte sensorIndex = 0; sensorIndex < 4; sensorIndex++) {
      pinMode(QTR_PINS[sensorIndex], INPUT);
      digitalWrite(QTR_PINS[sensorIndex], LOW);
    }
    while (remainingSensors && micros() - measurementStartUs < QTR_TIMEOUT_US) {
      unsigned int elapsedMicroseconds = micros() - measurementStartUs;
      for (byte sensorIndex = 0; sensorIndex < 4; sensorIndex++) {
        if (!measurementComplete[sensorIndex] && digitalRead(QTR_PINS[sensorIndex]) == LOW) {
          dischargeTimesUs[sensorIndex] = elapsedMicroseconds;
          measurementComplete[sensorIndex] = true;
          remainingSensors--;
        }
      }
    }
    // Cap unfinished readings at 4500 us; these timeout values also exceed the black thresholds.
    for (byte sensorIndex = 0; sensorIndex < 4; sensorIndex++) {
      if (!measurementComplete[sensorIndex]) {
        dischargeTimesUs[sensorIndex] = QTR_TIMEOUT_US;
      }
    }
    detectedBlackCount = 0;
    for (byte sensorIndex = 0; sensorIndex < 4; sensorIndex++) {
      if (isBlack(sensorIndex)) {
        detectedBlackCount++;
      }
    }
  }
  unsigned int rawTimeUs(byte sensorIndex) const {
    return dischargeTimesUs[sensorIndex];
  }
  // LOG1_B | Per-sensor raw thresholds replace unstable normalisation; WBBW gives a centred position.
  bool isBlack(byte sensorIndex) const {
    return dischargeTimesUs[sensorIndex] >= QTR_BLACK[sensorIndex];
  }
  byte blackSensorCount() const {
    return detectedBlackCount;
  }
  // Average the positions of sensors classified as black.
  int linePosition() const {
    // Use positions 0/1000/2000/3000, so the two middle sensors average to 1500.
    int positionSum = 0;
    byte activeSensorCount = 0;
    for (byte sensorIndex = 0; sensorIndex < 4; sensorIndex++) {
      if (isBlack(sensorIndex)) {
        positionSum += sensorIndex * 1000;
        activeSensorCount++;
      }
    }
    // No line and all black also return 1500, so check the black-sensor count before steering.
    return activeSensorCount ? positionSum / activeSensorCount : 1500;
  }
  void printPattern(Print &outputStream) const {
    for (byte sensorIndex = 0; sensorIndex < 4; sensorIndex++) {
      outputStream.print(isBlack(sensorIndex) ? 'B' : 'W');
    }
  }
} qtr;

class UltrasonicSensor {
  unsigned int distanceCm = 0;

public:
  void update() {
    pinMode(SONAR_PIN, OUTPUT);
    digitalWrite(SONAR_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_PIN, LOW);
    pinMode(SONAR_PIN, INPUT);
    // Divide the round-trip echo time by two; zero means no echo within 12 ms.
    unsigned long echoDurationUs = pulseIn(SONAR_PIN, HIGH, 12000UL);
    distanceCm = echoDurationUs ? (unsigned int)(echoDurationUs * 0.0343f / 2.0f) : 0;
  }
  unsigned int cm() const {
    return distanceCm;
  }
} sonar;

// Stop at the obstacle threshold; require three readings beyond threshold + 5 cm to clear.
class SafetySupervisor {
  bool obstacleBlocked = false;
  byte consecutiveClearReadings = 0;

public:
  void update(unsigned int measuredDistanceCm) {
    if (measuredDistanceCm > 0 && measuredDistanceCm <= mission.obstacleStopCm) {
      consecutiveClearReadings = 0;
      if (!obstacleBlocked) {
        obstacleBlocked = true;
        eventRing.push(EV_OBSTACLE, measuredDistanceCm);
      }
    } else if (obstacleBlocked && measuredDistanceCm > (unsigned int)mission.obstacleStopCm + 5) {
      if (++consecutiveClearReadings >= 3) {
        obstacleBlocked = false;
        consecutiveClearReadings = 0;
        eventRing.push(EV_CLEAR, measuredDistanceCm);
      }
    } else if (obstacleBlocked) {
      consecutiveClearReadings = 0;
    }
    // NO_ECHO is unknown; it never clears a previously detected obstacle.
  }
  bool isBlocked() const {
    return obstacleBlocked;
  }
} safety;

// Use gyro feedback for steering and acceleration-derived speed for diagnostics only.
class MPU6050Sensor {
  bool sensorReady = false;
  float gyroBiasDegPerSecond = 0;
  float forwardBiasCounts = 0;
  float headingDegrees = 0;
  float yawRateDegPerSecond = 0;
  float filteredAccelerationMps2 = 0;
  float previousAccelerationMps2 = 0;
  float estimatedSpeedMps = 0;
  float temperatureC = 0;
  unsigned long lastSampleUs = 0;

  bool writeReg(byte registerAddress, byte value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(registerAddress);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }
  bool sample(int16_t &rawForwardAcceleration, int16_t &rawYawRate, int16_t &rawTemperature) {
    // Acceleration, temperature and gyro registers are read in one 14-byte burst.
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) {
      return false;
    }
    Wire.requestFrom(MPU_ADDR, (byte)14);
    if (Wire.available() < 14) {
      return false;
    }
    // Combine each high/low byte pair into a signed 16-bit sensor reading.
    int16_t rawAccelerationX = ((int16_t)Wire.read() << 8) | Wire.read();
    int16_t rawAccelerationY = ((int16_t)Wire.read() << 8) | Wire.read();
    Wire.read();
    Wire.read(); // Z acceleration is not used in flat-ground estimation.
    rawTemperature = ((int16_t)Wire.read() << 8) | Wire.read();
    Wire.read();
    Wire.read();
    Wire.read();
    Wire.read(); // Skip gyro X and Y.
    rawYawRate = ((int16_t)Wire.read() << 8) | Wire.read();
    rawForwardAcceleration = MPU_FORWARD_AXIS == 0 ? rawAccelerationX : rawAccelerationY;
    return true;
  }

public:
  void begin() {
    Wire.beginTransmission(MPU_ADDR);
    if (Wire.endTransmission() != 0) {
      Serial.println(F("[MPU] 0x69 not detected"));
      return;
    }
    // Wake, low-pass filter, +/-500 deg/s gyro, +/-2g accelerometer.
    if (!writeReg(0x6B, 0x00) || !writeReg(0x1A, 0x03) || !writeReg(0x1B, 0x08) ||
        !writeReg(0x1C, 0x00)) {
      return;
    }
    delay(100);
    Serial.println(F("[MPU] Keep robot level and still for bias calibration"));
    float yawRateTotal = 0;
    float forwardAccelerationTotal = 0;
    int validSampleCount = 0;
    for (int sampleIndex = 0; sampleIndex < 150; sampleIndex++) {
      int16_t rawForwardAcceleration;
      int16_t rawYawRate;
      int16_t rawTemperature;
      if (sample(rawForwardAcceleration, rawYawRate, rawTemperature)) {
        yawRateTotal += rawYawRate / 65.5f;
        forwardAccelerationTotal += rawForwardAcceleration;
        validSampleCount++;
      }
      delay(5);
    }
    if (!validSampleCount) {
      Serial.println(F("[MPU] Calibration failed"));
      return;
    }
    // Average successful stationary samples to estimate gyro and forward-axis bias.
    gyroBiasDegPerSecond = yawRateTotal / validSampleCount;
    forwardBiasCounts = forwardAccelerationTotal / validSampleCount;
    headingDegrees = 0;
    estimatedSpeedMps = 0;
    lastSampleUs = micros();
    sensorReady = true;
    Serial.println(F("[MPU] Gyro feedback and experimental speed estimate enabled"));
  }
  void update() {
    if (!sensorReady) {
      return;
    }
    int16_t rawForwardAcceleration;
    int16_t rawYawRate;
    int16_t rawTemperature;
    // Skip failed reads; existing measurements remain unchanged and may become stale.
    if (!sample(rawForwardAcceleration, rawYawRate, rawTemperature)) {
      return;
    }
    unsigned long currentTimeUs = micros();
    // Convert sample spacing to seconds and skip gaps longer than 150 ms.
    float elapsedSeconds = (currentTimeUs - lastSampleUs) / 1000000.0f;
    lastSampleUs = currentTimeUs;
    if (elapsedSeconds <= 0 || elapsedSeconds > 0.15f) {
      return;
    }

    // Remove gyro bias and integrate yaw rate into heading relative to startup.
    yawRateDegPerSecond = rawYawRate / 65.5f - gyroBiasDegPerSecond;
    if (yawRateDegPerSecond > -0.5f && yawRateDegPerSecond < 0.5f) {
      yawRateDegPerSecond = 0;
    }
    headingDegrees += yawRateDegPerSecond * elapsedSeconds;
    while (headingDegrees >= 360) {
      headingDegrees -= 360;
    }
    while (headingDegrees < 0) {
      headingDegrees += 360;
    }

    // LOG2_A | Convert acceleration to m/s^2, apply a deadband, then blend 80% old and 20% new.
    float forwardAccelerationMps2 = ((float)rawForwardAcceleration - forwardBiasCounts) *
                                    (9.80665f / 16384.0f) * MPU_FORWARD_SIGN;
    if (forwardAccelerationMps2 > -0.16f && forwardAccelerationMps2 < 0.16f) {
      forwardAccelerationMps2 = 0;
    }
    filteredAccelerationMps2 = 0.8f * filteredAccelerationMps2 + 0.2f * forwardAccelerationMps2;
    // LOG2_B | Integrate adjacent accelerations for speed; resets and limits reduce drift effects.
    if (!drive.translating() || fabsf(yawRateDegPerSecond) > 50) {
      estimatedSpeedMps = 0;
      previousAccelerationMps2 = 0;
    } else {
      // Average the previous and current acceleration, then multiply by elapsed seconds.
      estimatedSpeedMps +=
          (previousAccelerationMps2 + filteredAccelerationMps2) * 0.5f * elapsedSeconds;
      if (estimatedSpeedMps < 0) {
        estimatedSpeedMps = 0;
      }
      if (estimatedSpeedMps > 1.5f) {
        estimatedSpeedMps = 1.5f;
      }
      previousAccelerationMps2 = filteredAccelerationMps2;
    }
    temperatureC = rawTemperature / 340.0f + 36.53f;
  }
  void resetSpeed() {
    estimatedSpeedMps = 0;
    previousAccelerationMps2 = 0;
  }
  bool ok() const {
    return sensorReady;
  }
  float degrees() const {
    return headingDegrees;
  }
  float rateCW() const {
    return yawRateDegPerSecond * MPU_CW_SIGN;
  }
  float rate() const {
    return yawRateDegPerSecond;
  }
  float accel() const {
    return filteredAccelerationMps2;
  }
  float estimatedSpeed() const {
    return estimatedSpeedMps;
  }
  float tempC() const {
    return temperatureC;
  }
} mpu;

// Read clock timestamps by converting packed BCD digits into ordinary numbers.
class RTCClock {
  bool clockReady = false;
  byte currentHour = 0;
  byte currentMinute = 0;
  byte currentSecond = 0;
  byte fromBCD(byte bcdValue) {
    return (bcdValue >> 4) * 10 + (bcdValue & 15);
  }
#if SET_RTC_FROM_COMPILE_TIME
  byte toBCD(byte decimalValue) {
    return ((decimalValue / 10) << 4) | (decimalValue % 10);
  }
  void adjustCompileTime() {
    const char *compileTimeText = __TIME__;
    byte hours = (compileTimeText[0] - '0') * 10 + (compileTimeText[1] - '0');
    byte minutes = (compileTimeText[3] - '0') * 10 + (compileTimeText[4] - '0');
    byte seconds = (compileTimeText[6] - '0') * 10 + (compileTimeText[7] - '0');
    Wire.beginTransmission(RTC_ADDR);
    Wire.write((byte)0);
    Wire.write(toBCD(seconds));
    Wire.write(toBCD(minutes));
    Wire.write(toBCD(hours));
    Wire.endTransmission();
  }
#endif
public:
  void begin() {
    Wire.beginTransmission(RTC_ADDR);
    if (Wire.endTransmission() != 0) {
      Serial.println(F("[RTC] 0x68 missing"));
      return;
    }
#if SET_RTC_FROM_COMPILE_TIME
    adjustCompileTime();
#endif
    update();
  }
  void update() {
    Wire.beginTransmission(RTC_ADDR);
    Wire.write((byte)0);
    if (Wire.endTransmission(false) != 0) {
      return;
    }
    Wire.requestFrom(RTC_ADDR, (byte)3);
    if (Wire.available() < 3) {
      return;
    }
    byte seconds = fromBCD(Wire.read() & 0x7F);
    byte minutes = fromBCD(Wire.read() & 0x7F);
    byte hourRegister = Wire.read();
    byte hours = 0;
    // Convert 12-hour AM/PM readings to 24-hour time, including midnight and noon.
    if (hourRegister & 0x40) {
      hours = fromBCD(hourRegister & 0x1F);
      if ((hourRegister & 0x20) && hours != 12) {
        hours += 12;
      }
      if (!(hourRegister & 0x20) && hours == 12) {
        hours = 0;
      }
    } else {
      hours = fromBCD(hourRegister & 0x3F);
    }
    if (hours > 23 || minutes > 59 || seconds > 59) {
      return;
    }
    currentHour = hours;
    currentMinute = minutes;
    currentSecond = seconds;
    clockReady = true;
  }
  bool ok() const {
    return clockReady;
  }
  byte hour() const {
    return currentHour;
  }
  byte minute() const {
    return currentMinute;
  }
  byte second() const {
    return currentSecond;
  }
  void print(Print &outputStream) const {
    if (!clockReady) {
      outputStream.print(F("NO RTC"));
      return;
    }
    if (currentHour < 10) {
      outputStream.print('0');
    }
    outputStream.print(currentHour);
    outputStream.print(':');
    if (currentMinute < 10) {
      outputStream.print('0');
    }
    outputStream.print(currentMinute);
    outputStream.print(':');
    if (currentSecond < 10) {
      outputStream.print('0');
    }
    outputStream.print(currentSecond);
  }
} rtc;

// LOG3_A | Combine line-position error with yaw-rate feedback to adjust left and right PWM.
void fusedLineSteer(int linePosition) {
  int linePositionError = linePosition - 1500;
  // Ignore small line errors and limit the requested yaw rate to +/-85 degrees per second.
  if (abs(linePositionError) < 70) {
    linePositionError = 0;
  }
  float targetYawRate = constrain(linePositionError * TARGET_YAW_GAIN, -85.0f, 85.0f);
  float steeringCorrection = LINE_KP * linePositionError;
  if (mpu.ok()) {
    steeringCorrection += RATE_KP * (targetYawRate - mpu.rateCW());
  }
  // Limit steering to +/-32 PWM and reduce forward power as line error increases.
  steeringCorrection = constrain(steeringCorrection, -(float)MAX_STEER, (float)MAX_STEER);
  int basePwm = mission.cruisePwm - constrain(abs(linePositionError) / 90, 0, 12);
  drive.set(constrain(basePwm + (int)steeringCorrection, 20, 100),
            constrain(basePwm - (int)steeringCorrection, 20, 100));
}

// Wrap heading differences into +/-180 degrees so a 350-to-10-degree turn becomes +20.
float headingDelta(float startingHeading, float currentHeading) {
  float headingDifference = currentHeading - startingHeading;
  while (headingDifference > 180) {
    headingDifference -= 360;
  }
  while (headingDifference < -180) {
    headingDifference += 360;
  }
  return headingDifference * MPU_CW_SIGN;
}

// LOG3_B | Track following, junction turns and lost-line recovery with explicit navigation states.
enum NavMode : byte { NAV_WAIT, NAV_FOLLOW, NAV_PROBE, NAV_TURN, NAV_SEARCH };

class NavigationFSM {
  NavMode navigationMode = NAV_WAIT;
  unsigned long stateEnteredMs = 0;
  unsigned long allBlackStartMs = 0;
  unsigned long lastJunctionMs = 0;
  int8_t lastLineDirection = -1;
  int8_t junctionTurnDirection = -1;
  float turnStartHeading = 0;

  void change(NavMode nextMode) {
    if (navigationMode == nextMode) {
      return;
    }
    navigationMode = nextMode;
    stateEnteredMs = millis();
    if (nextMode == NAV_SEARCH) {
      eventRing.push(EV_LINE_LOST);
    }
    if (nextMode == NAV_FOLLOW) {
      eventRing.push(EV_LINE_FOUND);
    }
  }

public:
  void reset() {
    navigationMode = NAV_WAIT;
    stateEnteredMs = millis();
    allBlackStartMs = 0;
    drive.stop();
  }
  // Shift active timestamps by the pause duration so stopped time does not count as movement.
  void shiftTime(unsigned long pauseDurationMs) {
    stateEnteredMs += pauseDurationMs;
    if (allBlackStartMs) {
      allBlackStartMs += pauseDurationMs;
    }
    if (lastJunctionMs) {
      lastJunctionMs += pauseDurationMs;
    }
  }
  NavMode current() const {
    return navigationMode;
  }
  const __FlashStringHelper *stateName() const {
    switch (navigationMode) {
    case NAV_WAIT:
      return F("WAIT");
    case NAV_FOLLOW:
      return F("FOLLOW");
    case NAV_PROBE:
      return F("PROBE");
    case NAV_TURN:
      return F("TURN");
    case NAV_SEARCH:
      return F("SEARCH");
    }
    return F("UNKNOWN");
  }
  void update(unsigned long currentTimeMs) {
    unsigned long stateElapsedMs = currentTimeMs - stateEnteredMs;
    byte detectedBlackCount = qtr.blackSensorCount();
    switch (navigationMode) {
    // Start tracking only when one to three sensors detect black.
    case NAV_WAIT:
      drive.stop();
      if (detectedBlackCount > 0 && detectedBlackCount < 4) {
        change(NAV_FOLLOW);
      }
      break;

    case NAV_FOLLOW:
      if (detectedBlackCount == 0) {
        change(NAV_SEARCH);
        drive.stop();
        break;
      }
      // Accept all-black junctions after 80 ms, with a 1100 ms gap between junctions.
      if (detectedBlackCount == 4) {
        if (!allBlackStartMs) {
          allBlackStartMs = currentTimeMs;
        }
        drive.set(42, 42);
        if (currentTimeMs - allBlackStartMs >= 80 && currentTimeMs - lastJunctionMs >= 1100) {
          junctionTurnDirection = mission.preferLeft ? -1 : 1;
          eventRing.push(EV_JUNCTION, junctionTurnDirection);
          change(NAV_PROBE);
        }
        break;
      }
      allBlackStartMs = 0;
      if (qtr.linePosition() < 1200) {
        lastLineDirection = -1;
      } else if (qtr.linePosition() > 1800) {
        lastLineDirection = 1;
      }
      fusedLineSteer(qtr.linePosition());
      break;

    // Probe forward for 180 ms to move farther into the junction before pivoting.
    case NAV_PROBE:
      drive.set(43, 43);
      if (stateElapsedMs >= 180) {
        turnStartHeading = mpu.degrees();
        change(NAV_TURN);
      }
      break;

    case NAV_TURN: {
      // Gyroscope angular feedback limits corner overshoot when available.
      float turnedDegrees =
          mpu.ok() ? junctionTurnDirection * headingDelta(turnStartHeading, mpu.degrees()) : 0;
      if (junctionTurnDirection < 0) {
        drive.set(-PIVOT_PWM, PIVOT_PWM);
      } else {
        drive.set(PIVOT_PWM, -PIVOT_PWM);
      }
      // Rejoin a partial line after 32 degrees or 420 ms; recover if turning exceeds 2300 ms.
      bool turnProgressReached = mpu.ok() ? turnedDegrees >= 32 : stateElapsedMs >= 420;
      if (detectedBlackCount > 0 && detectedBlackCount < 4 && turnProgressReached) {
        lastJunctionMs = currentTimeMs;
        eventRing.push(EV_JUNCTION_EXIT, (int)turnedDegrees);
        change(NAV_FOLLOW);
      } else if (stateElapsedMs >= 2300) {
        lastJunctionMs = currentTimeMs;
        eventRing.push(EV_TURN_TIMEOUT);
        change(NAV_SEARCH);
      }
      break;
    }

    // Search toward the last line side, reverse after 500 ms and stop after 1200 ms.
    case NAV_SEARCH:
      if (detectedBlackCount > 0 && detectedBlackCount < 4) {
        change(NAV_FOLLOW);
        break;
      }
      if ((stateElapsedMs < 500 ? lastLineDirection : -lastLineDirection) < 0) {
        drive.set(-70, 70);
      } else {
        drive.set(70, -70);
      }
      if (stateElapsedMs >= SEARCH_TIMEOUT_MS) {
        drive.stop();
        eventRing.push(EV_SEARCH_FAIL);
        change(NAV_WAIT);
      }
      break;
    }
  }
} navigation;

// LOG3_C | Compile SD movements into bounded instructions, then execute them with a loop stack; virtual machine w/ sensors running; program counter(whatsnext),loop stack(repeted sections), movement states, timestamps
enum Opcode : byte { OP_MOVE = 1, OP_LOOP_OPEN, OP_LOOP_CLOSE };
struct Instruction {
  byte opcode;
  char movement;
  unsigned int movementAmount;
  byte repeatCount;
};
struct LoopFrame {
  byte bodyStartIndex;
  byte remainingRepeats;
};

class DanceVM {
  static const byte MAX_CODE = 48;
  static const byte MAX_DEPTH = 3;
  static const unsigned long MAX_TOTAL_MS = 180000UL;
  Instruction instructions[MAX_CODE];
  LoopFrame frames[MAX_DEPTH];
  byte instructionTotal = 0;
  byte instructionIndex = 0;
  byte loopDepth = 0;
  bool programValid = false;
  bool programRunning = false;
  bool movementActive = false;
  unsigned long movementStartMs = 0;
  float turnStartHeading = 0;

  // All instruction writes go through one capacity check, preventing array overflow.
  bool append(byte opcode, char movement, unsigned int movementAmount, byte repeatCount) {
    if (instructionTotal >= MAX_CODE) {
      return false;
    }
    instructions[instructionTotal].opcode = opcode;
    instructions[instructionTotal].movement = movement;
    instructions[instructionTotal].movementAmount = movementAmount;
    instructions[instructionTotal].repeatCount = repeatCount;
    instructionTotal++;
    return true;
  }

  // Validate movement tokens, nested repeats and estimated duration before execution.
  bool compile(const char *sourceText) {
    instructionTotal = 0;
    programValid = false;
    byte nestingDepth = 0;
    unsigned int repeatMultiplier = 1;
    byte repeatStack[MAX_DEPTH];
    unsigned long estimatedDurationMs = 0;
    const char *sourceCursor = sourceText;
    while (*sourceCursor) {
      // Skip comments and whitespace before interpreting a command or loop token.
      if (*sourceCursor == '#') {
        while (*sourceCursor && *sourceCursor != '\n') {
          sourceCursor++;
        }
        continue;
      }
      if (*sourceCursor == ' ' || *sourceCursor == '\n' || *sourceCursor == '\r' ||
          *sourceCursor == '\t') {
        sourceCursor++;
        continue;
      }
      if (*sourceCursor >= '0' && *sourceCursor <= '9') {
        unsigned int loopRepeatCount = 0;
        while (*sourceCursor >= '0' && *sourceCursor <= '9') {
          loopRepeatCount = loopRepeatCount * 10 + (*sourceCursor++ - '0');
          if (loopRepeatCount > 9) {
            return false;
          }
        }
        while (*sourceCursor == ' ') {
          sourceCursor++;
        }
        if (*sourceCursor++ != '[' || loopRepeatCount < 1 || nestingDepth >= MAX_DEPTH) {
          return false;
        }
        if (repeatMultiplier * loopRepeatCount > 729) {
          return false;
        }
        if (!append(OP_LOOP_OPEN, 0, 0, (byte)loopRepeatCount)) {
          return false;
        }
        // Save each repeat count so nested movements contribute their full estimated duration.
        repeatStack[nestingDepth++] = loopRepeatCount;
        repeatMultiplier *= loopRepeatCount;
        continue;
      }
      if (*sourceCursor == ']') {
        if (!nestingDepth || !append(OP_LOOP_CLOSE, 0, 0, 0)) {
          return false;
        }
        repeatMultiplier /= repeatStack[--nestingDepth];
        sourceCursor++;
        continue;
      }
      char movementCommand = *sourceCursor++;
      if (movementCommand != 'F' && movementCommand != 'B' && movementCommand != 'L' &&
          movementCommand != 'R' && movementCommand != 'S') {
        return false;
      }
      unsigned int movementAmount = 0;
      bool hasDigits = false;
      while (*sourceCursor >= '0' && *sourceCursor <= '9') {
        hasDigits = true;
        movementAmount = movementAmount * 10 + (*sourceCursor++ - '0');
        if (movementAmount > 4000) {
          return false;
        }
      }
      // Use defaults for omitted amounts; accept 5-180 degrees or 80-4000 ms, rejecting zero.
      if (!hasDigits) {
        movementAmount = (movementCommand == 'L' || movementCommand == 'R') ? 90 : 350;
      } else if (movementAmount == 0) {
        return false;
      }
      if (movementCommand == 'L' || movementCommand == 'R') {
        if (movementAmount < 5 || movementAmount > 180) {
          return false;
        }
      } else if (movementAmount < 80 || movementAmount > 4000) {
        return false;
      }
      // Estimate turn duration at 18 ms per degree and cap the estimated program length at 180 seconds.
      unsigned long movementDurationMs = (movementCommand == 'L' || movementCommand == 'R')
                                             ? movementAmount * 18UL
                                             : movementAmount;
      if (movementDurationMs * repeatMultiplier > MAX_TOTAL_MS - estimatedDurationMs) {
        return false;
      }
      estimatedDurationMs += movementDurationMs * repeatMultiplier;
      if (!append(OP_MOVE, movementCommand, movementAmount, 0)) {
        return false;
      }
    }
    programValid = instructionTotal > 0 && nestingDepth == 0;
    return programValid;
  }

  // Process loop instructions until the next movement, using frames to track repetitions.
  void nextStep(unsigned long currentTimeMs) {
    movementActive = false;
    while (instructionIndex < instructionTotal) {
      const Instruction &instruction = instructions[instructionIndex];
      if (instruction.opcode == OP_LOOP_OPEN) {
        if (loopDepth >= MAX_DEPTH) {
          stop();
          return;
        }
        frames[loopDepth].bodyStartIndex = instructionIndex + 1;
        frames[loopDepth].remainingRepeats = instruction.repeatCount;
        loopDepth++;
        instructionIndex++;
        continue;
      }
      if (instruction.opcode == OP_LOOP_CLOSE) {
        if (!loopDepth) {
          stop();
          return;
        }
        // Repeat the current loop body or pop its frame when no repetitions remain.
        LoopFrame &loopFrame = frames[loopDepth - 1];
        if (--loopFrame.remainingRepeats) {
          instructionIndex = loopFrame.bodyStartIndex;
        } else {
          loopDepth--;
          instructionIndex++;
        }
        continue;
      }
      movementStartMs = currentTimeMs;
      turnStartHeading = mpu.degrees();
      movementActive = true;
      return;
    }
    programRunning = false;
    drive.stop();
    runMode = RUN_IDLE;
    eventRing.push(EV_DANCE_END);
    Serial.println(F("[DANCE] Program complete; motors stopped"));
  }

public:
  bool load(const char *programText) {
    programRunning = false;
    movementActive = false;
    instructionIndex = 0;
    loopDepth = 0;
    programValid = compile(programText);
    if (!programValid) {
      eventRing.push(EV_DANCE_ERROR);
    }
    return programValid;
  }
  bool available() const {
    return programValid;
  }
  bool isRunning() const {
    return programRunning;
  }
  byte instructionCount() const {
    return instructionTotal;
  }
  byte programCounter() const {
    return instructionIndex;
  }
  char currentMove() const {
    return programRunning && movementActive ? instructions[instructionIndex].movement : '-';
  }
  void stop() {
    programRunning = false;
    movementActive = false;
    drive.stop();
  }
  // Adjust only an active movement's start time; the instruction keeps its progress.
  void pauseFor(unsigned long pauseDurationMs) {
    if (programRunning && movementActive) {
      movementStartMs += pauseDurationMs;
    }
  }
  bool start() {
    if (!programValid) {
      return false;
    }
    instructionIndex = 0;
    loopDepth = 0;
    movementActive = false;
    programRunning = true;
    drive.stop();
    eventRing.push(EV_DANCE_BEGIN);
    return true;
  }
  void update(unsigned long currentTimeMs) {
    if (!programRunning) {
      return;
    }
    if (!movementActive) {
      nextStep(currentTimeMs);
      if (!programRunning) {
        return;
      }
    }
    // Update one movement at a time so the main loop can check obstacles between calls.
    Instruction &instruction = instructions[instructionIndex];
    unsigned long movementElapsedMs = currentTimeMs - movementStartMs;
    bool movementFinished = false;
    switch (instruction.movement) {
    case 'F':
      drive.set(mission.cruisePwm, mission.cruisePwm);
      movementFinished = movementElapsedMs >= instruction.movementAmount;
      break;
    case 'B':
      drive.set(-mission.cruisePwm, -mission.cruisePwm);
      movementFinished = movementElapsedMs >= instruction.movementAmount;
      break;
    case 'S':
      drive.stop();
      movementFinished = movementElapsedMs >= instruction.movementAmount;
      break;
    case 'L':
    case 'R': {
      int turnDirection = instruction.movement == 'R' ? 1 : -1;
      float turnedDegrees = turnDirection * headingDelta(turnStartHeading, mpu.degrees());
      // Reduce turn power near the target; use gyro progress or timed fallback with a timeout.
      int turnPwm =
          constrain((int)((instruction.movementAmount - turnedDegrees) * 0.3f) + 48, 55, 85);
      drive.set(turnDirection * turnPwm, -turnDirection * turnPwm);
      if (mpu.ok()) {
        movementFinished = turnedDegrees >= instruction.movementAmount - 3 ||
                           movementElapsedMs > instruction.movementAmount * 28UL + 1200UL;
      } else {
        movementFinished = movementElapsedMs >= instruction.movementAmount * 7UL;
      }
      if (mpu.ok() && movementElapsedMs > instruction.movementAmount * 28UL + 1200UL) {
        eventRing.push(EV_TURN_TIMEOUT);
      }
      break;
    }
    }
    // Finish once, advance the program counter, and let the next call select an action.
    if (movementFinished) {
      drive.stop();
      instructionIndex++;
      movementActive = false;
    }
  }
} danceVM;

#if ENABLE_LCD
// Drive the LCD through its I2C expander and rotate through diagnostic pages.
class CompactLCD {
  bool displayReady = false;
  byte displayPage = 0;
  void expander(byte dataBits) {
    Wire.beginTransmission(LCD_ADDR);
    Wire.write(dataBits | 0x08);
    Wire.endTransmission();
  }
  void pulse(byte dataBits) {
    expander(dataBits | 0x04);
    delayMicroseconds(1);
    expander(dataBits & ~0x04);
    delayMicroseconds(50);
  }
  void write4(byte dataBits) {
    expander(dataBits);
    pulse(dataBits);
  }
  void send(byte byteValue, byte registerSelect) {
    write4((byteValue & 0xF0) | registerSelect);
    write4(((byteValue << 4) & 0xF0) | registerSelect);
  }
  void command(byte commandByte) {
    send(commandByte, 0);
  }
  void character(char characterValue) {
    send(characterValue, 1);
  }
  void text(const char *textCursor) {
    while (*textCursor) {
      character(*textCursor++);
    }
  }
  void number(long numberValue) {
    char numberBuffer[12];
    ltoa(numberValue, numberBuffer, 10);
    text(numberBuffer);
  }
  void cursor(byte column, byte row) {
    command(0x80 | (column + (row ? 0x40 : 0)));
  }
  void erase(byte row) {
    cursor(0, row);
    text("                ");
    cursor(0, row);
  }

public:
  void begin() {
    Wire.beginTransmission(LCD_ADDR);
    displayReady = Wire.endTransmission() == 0;
    if (!displayReady) {
      return;
    }
    delay(50);
    write4(0x30);
    delayMicroseconds(4500);
    write4(0x30);
    delayMicroseconds(4500);
    write4(0x30);
    delayMicroseconds(150);
    write4(0x20);
    command(0x28);
    command(0x08);
    command(0x01);
    delay(2);
    command(0x06);
    command(0x0C);
    cursor(0, 0);
    text("ROBOT STARTING");
  }
  bool ok() const {
    return displayReady;
  }
  void update() {
    if (!displayReady) {
      return;
    }
    erase(0);
    erase(1);
    if (displayPage == 0) {
      text("PAT ");
      for (byte sensorIndex = 0; sensorIndex < 4; sensorIndex++) {
        character(qtr.isBlack(sensorIndex) ? 'B' : 'W');
        if (sensorIndex < 3) {
          character(' ');
        }
      }
      cursor(0, 1);
      text("POS ");
      number(qtr.linePosition());
    } else if (displayPage == 1) {
      text("MODE ");
      if (safety.isBlocked()) {
        text("OBSTACLE");
      } else if (runMode == RUN_DANCE) {
        text("DANCE");
      } else if (runMode == RUN_IDLE) {
        text("IDLE");
      } else if (navigation.current() == NAV_FOLLOW) {
        text("FOLLOW");
      } else if (navigation.current() == NAV_TURN) {
        text("TURN");
      } else if (navigation.current() == NAV_SEARCH) {
        text("SEARCH");
      } else {
        text("WAIT");
      }
      cursor(0, 1);
      text("DIST ");
      if (sonar.cm()) {
        number(sonar.cm());
        text("cm");
      } else {
        text("NO ECHO");
      }
    } else if (displayPage == 2) {
      text("HEAD ");
      if (mpu.ok()) {
        number((long)mpu.degrees());
        character((char)223);
      } else {
        text("OFFLINE");
      }
      cursor(0, 1);
      text("V~ ");
      // Integer centimetres per second are an experimental IMU estimate.
      number((long)(mpu.estimatedSpeed() * 100));
      text("cm/s EST");
    } else {
      text("CLOCK ");
      if (rtc.ok()) {
        if (rtc.hour() < 10) {
          character('0');
        }
        number(rtc.hour());
        character(':');
        if (rtc.minute() < 10) {
          character('0');
        }
        number(rtc.minute());
        character(':');
        if (rtc.second() < 10) {
          character('0');
        }
        number(rtc.second());
      } else {
        text("NO RTC");
      }
      cursor(0, 1);
      if (runMode == RUN_DANCE) {
        text("MOVE ");
        character(danceVM.currentMove());
        text(" PC ");
        number(danceVM.programCounter());
      } else {
        text("PWM ");
        number(drive.leftPWM());
        character('/');
        number(drive.rightPWM());
      }
    }
    displayPage = (displayPage + 1) % 4;
  }
};
CompactLCD lcd;
#endif

#if ENABLE_SD
// Load settings and movements from SD, then save sensor snapshots and event records.
class SDStorage {
  bool cardReady = false;

  // Reject malformed integers rather than silently converting them to zero.
  int parseNumber(const char *numberText) {
    if (!*numberText) {
      return -1;
    }
    unsigned int value = 0;
    while (*numberText) {
      if (*numberText < '0' || *numberText > '9') {
        return -1;
      }
      value = value * 10 + (*numberText++ - '0');
      if (value > 999) {
        return -1;
      }
    }
    return value;
  }
  void applyMission(const char *configLine) {
    if (strncmp(configLine, "SPEED=", 6) == 0) {
      int settingValue = parseNumber(configLine + 6);
      if (settingValue >= 40 && settingValue <= 90) {
        mission.cruisePwm = settingValue;
      }
    } else if (strncmp(configLine, "STOP_CM=", 8) == 0) {
      int settingValue = parseNumber(configLine + 8);
      if (settingValue >= 10 && settingValue <= 80) {
        mission.obstacleStopCm = settingValue;
      }
    } else if (strcmp(configLine, "TURN=LEFT") == 0) {
      mission.preferLeft = true;
    } else if (strcmp(configLine, "TURN=RIGHT") == 0) {
      mission.preferLeft = false;
    } else if (strcmp(configLine, "MODE=DANCE") == 0) {
      mission.danceAtBoot = true;
    } else if (strcmp(configLine, "MODE=LINE") == 0) {
      mission.danceAtBoot = false;
    }
  }
  // Reject configuration lines that exceed the buffer instead of applying truncated values.
  void loadMission() {
    File missionFile = SD.open("MISSION.TXT");
    if (!missionFile) {
      Serial.println(F("[SD] No MISSION.TXT; defaults used"));
      return;
    }
    char configLine[32];
    byte characterCount = 0;
    bool lineTooLong = false;
    while (missionFile.available()) {
      char inputCharacter = (char)missionFile.read();
      if (inputCharacter == '\n' || inputCharacter == '\r') {
        if (characterCount && !lineTooLong) {
          configLine[characterCount] = 0;
          applyMission(configLine);
        }
        characterCount = 0;
        lineTooLong = false;
      } else if (characterCount < sizeof(configLine) - 1) {
        configLine[characterCount++] = inputCharacter;
      } else {
        lineTooLong = true;
      }
    }
    if (characterCount && !lineTooLong) {
      configLine[characterCount] = 0;
      applyMission(configLine);
    }
    missionFile.close();
  }
  void loadDance() {
    File danceFile = SD.open("DANCE.TXT");
    if (!danceFile) {
      Serial.println(F("[SD] No DANCE.TXT"));
      return;
    }
    // A fixed source buffer and bounded compiler avoid heap fragmentation on the Uno.
    static char source[192];
    unsigned int characterCount = 0;
    while (danceFile.available() && characterCount < sizeof(source) - 1) {
      source[characterCount++] = (char)danceFile.read();
    }
    // Reject sources above 191 characters instead of compiling truncated movement programs.
    bool sourceTooLong = danceFile.available() > 0;
    danceFile.close();
    source[characterCount] = 0;
    if (sourceTooLong || !danceVM.load(source)) {
      Serial.println(F("[DANCE] Invalid syntax, duration, nesting or program size"));
      eventRing.push(EV_DANCE_ERROR);
      return;
    }
    Serial.print(F("[DANCE] Compiled instructions: "));
    Serial.println(danceVM.instructionCount());
  }

public:
  void begin() {
    cardReady = SD.begin(SD_CS);
    if (!cardReady) {
      Serial.println(F("[SD] Card initialisation failed"));
      return;
    }
    loadMission();
    loadDance();
    File telemetryFile = SD.open("NAVLOG.CSV", FILE_WRITE);
    if (telemetryFile) {
      if (!telemetryFile.size()) {
        telemetryFile.println(
            F("time_s,clock,raw7,raw5,raw3,raw1,position,distance_cm,heading_deg,yaw_deg_s,accel_m_s2,speed_est_m_s,left_pwm,right_pwm,run_mode,nav_mode,blocked,dance_pc"));
      }
      telemetryFile.close();
    }
    File eventsFile = SD.open("EVENTS.CSV", FILE_WRITE);
    if (eventsFile) {
      if (!eventsFile.size()) {
        eventsFile.println(F("elapsed_ms,event_code,detail"));
      }
      eventsFile.close();
    }
    Serial.println(F("[SD] Mission, dance program and telemetry ready"));
  }
  bool ok() const {
    return cardReady;
  }
  void log() {
    if (!cardReady) {
      return;
    }
    File telemetryFile = SD.open("NAVLOG.CSV", FILE_WRITE);
    if (telemetryFile) {
      telemetryFile.print(millis() / 1000UL);
      telemetryFile.print(',');
      rtc.print(telemetryFile);
      telemetryFile.print(',');
      for (byte sensorIndex = 0; sensorIndex < 4; sensorIndex++) {
        telemetryFile.print(qtr.rawTimeUs(sensorIndex));
        telemetryFile.print(',');
      }
      telemetryFile.print(qtr.linePosition());
      telemetryFile.print(',');
      telemetryFile.print(sonar.cm());
      telemetryFile.print(',');
      if (mpu.ok()) {
        telemetryFile.print(mpu.degrees(), 1);
      }
      telemetryFile.print(',');
      if (mpu.ok()) {
        telemetryFile.print(mpu.rate(), 1);
      }
      telemetryFile.print(',');
      if (mpu.ok()) {
        telemetryFile.print(mpu.accel(), 2);
      }
      telemetryFile.print(',');
      if (mpu.ok()) {
        telemetryFile.print(mpu.estimatedSpeed(), 3);
      }
      telemetryFile.print(',');
      telemetryFile.print(drive.leftPWM());
      telemetryFile.print(',');
      telemetryFile.print(drive.rightPWM());
      telemetryFile.print(',');
      telemetryFile.print((int)runMode);
      telemetryFile.print(',');
      telemetryFile.print((int)navigation.current());
      telemetryFile.print(',');
      telemetryFile.print(safety.isBlocked() ? 1 : 0);
      telemetryFile.print(',');
      telemetryFile.println(danceVM.programCounter());
      telemetryFile.close();
    }
    // Drain queued events after opening the file; individual write failures are not checked.
    File eventsFile = SD.open("EVENTS.CSV", FILE_WRITE);
    if (eventsFile) {
      RobotEvent queuedEvent;
      while (eventRing.peek(queuedEvent)) {
        eventsFile.print(queuedEvent.elapsedMs);
        eventsFile.print(',');
        eventsFile.print(queuedEvent.eventCode);
        eventsFile.print(',');
        eventsFile.println(queuedEvent.eventDetail);
        eventRing.pop();
      }
      eventsFile.close();
    }
  }
} storage;
#endif

TaskTimer sonarTimer;
TaskTimer mpuTimer;
TaskTimer rtcTimer;
TaskTimer serialTimer;
TaskTimer lcdTimer;
TaskTimer sdTimer;
unsigned long previousLoopStartUs = 0;
unsigned long longestLoopUs = 0;
unsigned int slowLoopCount = 0;
unsigned long safetyPauseStartMs = 0;

void printDashboard() {
  Serial.print(F("QTR "));
  for (byte sensorIndex = 0; sensorIndex < 4; sensorIndex++) {
    Serial.print(qtr.rawTimeUs(sensorIndex));
    if (sensorIndex < 3) {
      Serial.print(',');
    }
  }
  Serial.print(F(" PAT:"));
  qtr.printPattern(Serial);
  Serial.print(F(" POS:"));
  Serial.print(qtr.linePosition());
  Serial.print(F(" RUN:"));
  Serial.print((int)runMode);
  Serial.print(F(" NAV:"));
  Serial.print(navigation.stateName());
  Serial.print(F(" DIST:"));
  Serial.print(sonar.cm());
  Serial.print(F(" YAW:"));
  Serial.print(mpu.degrees(), 1);
  Serial.print(F(" RATE:"));
  Serial.print(mpu.rateCW(), 1);
  Serial.print(F(" SPD_EST:"));
  Serial.print(mpu.estimatedSpeed(), 3);
  Serial.print(F("m/s"));
  Serial.print(F(" PWM:"));
  Serial.print(drive.leftPWM());
  Serial.print('/');
  Serial.print(drive.rightPWM());
  Serial.print(F(" SAFE:"));
  Serial.print(safety.isBlocked() ? F("BLOCK") : F("OK"));
  Serial.print(F(" DANCE_PC:"));
  Serial.print(danceVM.programCounter());
  Serial.print(F(" MAX_LOOP_US:"));
  Serial.print(longestLoopUs);
  Serial.print(F(" SLOW_LOOPS:"));
  Serial.print(slowLoopCount);
  Serial.print(F(" DROPPED_EVENTS:"));
  Serial.println(eventRing.lost());
}

// Fixed-width serial command parsing avoids dynamic String allocation.
char commandBuffer[16];
byte commandLength = 0;
void handleCommand(const char *commandText) {
  if (strcmp(commandText, "stop") == 0) {
    danceVM.stop();
    drive.stop();
    runMode = RUN_IDLE;
    Serial.println(F("[CTRL] STOP"));
  } else if (strcmp(commandText, "line") == 0) {
    danceVM.stop();
    navigation.reset();
    runMode = RUN_LINE;
    Serial.println(F("[CTRL] LINE FOLLOWING"));
  } else if (strcmp(commandText, "dance") == 0) {
    navigation.reset();
    if (danceVM.start()) {
      runMode = RUN_DANCE;
      Serial.println(F("[CTRL] DANCE STARTED"));
    } else {
      Serial.println(F("[CTRL] No valid SD dance program"));
    }
  } else if (strcmp(commandText, "status") == 0) {
    printDashboard();
  } else {
    Serial.println(F("Commands: line, dance, stop, status"));
  }
}
// Read commands up to a newline; overflow resets the length and can leave a command suffix.
void processSerial() {
  while (Serial.available()) {
    char inputCharacter = (char)Serial.read();
    if (inputCharacter == '\n' || inputCharacter == '\r') {
      if (commandLength) {
        commandBuffer[commandLength] = 0;
        handleCommand(commandBuffer);
      }
      commandLength = 0;
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = inputCharacter;
    } else {
      commandLength = 0;
    }
  }
}

// Initialise hardware and check the obstacle sensor before entering the control loop.
void setup() {
  Serial.begin(115200);
  Wire.begin();
  drive.begin();
  eventRing.push(EV_BOOT);
  Serial.println(F("ROBOT: IMU-FUSED LINE CONTROL + DANCE BYTECODE VM"));

#if ENABLE_LCD
  lcd.begin();
#endif
  rtc.begin();
  mpu.begin();
#if ENABLE_SD
  storage.begin();
#endif
  sonar.update();
  safety.update(sonar.cm());
  navigation.reset();

  // Use line mode if an SD-requested dance cannot start.
  if (mission.danceAtBoot && danceVM.start()) {
    runMode = RUN_DANCE;
  } else {
    runMode = RUN_LINE;
  }

  unsigned long currentTimeMs = millis();
  sonarTimer.reset(currentTimeMs);
  mpuTimer.reset(currentTimeMs);
  rtcTimer.reset(currentTimeMs);
  serialTimer.reset(currentTimeMs);
  lcdTimer.reset(currentTimeMs);
  sdTimer.reset(currentTimeMs);
  previousLoopStartUs = micros();
  Serial.print(F("SPEED="));
  Serial.print(mission.cruisePwm);
  Serial.print(F(" STOP_CM="));
  Serial.print(mission.obstacleStopCm);
  Serial.print(F(" BOOT_MODE="));
  Serial.println(runMode == RUN_DANCE ? F("DANCE") : F("LINE"));
  Serial.println(F("Serial commands: line, dance, stop, status"));
}

// LOG3_D | Obstacles override movement; shift action timestamps after a pause to preserve progress.
void loop() {
  unsigned long currentTimeMs = millis();
  // Track intervals between loop starts and count cycles longer than 100 ms.
  unsigned long loopDurationUs = micros() - previousLoopStartUs;
  previousLoopStartUs = micros();
  if (loopDurationUs > longestLoopUs) {
    longestLoopUs = loopDurationUs;
  }
  if (loopDurationUs > 100000UL) {
    slowLoopCount++;
  }

  processSerial();
  qtr.update();

  if (mpuTimer.due(20, currentTimeMs)) {
    mpu.update();
  }
  if (sonarTimer.due(50, currentTimeMs)) {
    bool wasBlocked = safety.isBlocked();
    sonar.update();
    safety.update(sonar.cm());
    if (!wasBlocked && safety.isBlocked()) {
      safetyPauseStartMs = currentTimeMs;
    }
    if (wasBlocked && !safety.isBlocked()) {
      // Apply the same safety-pause duration to navigation and programmed movement.
      unsigned long pauseDurationMs = currentTimeMs - safetyPauseStartMs;
      navigation.shiftTime(pauseDurationMs);
      danceVM.pauseFor(pauseDurationMs);
    }
  }

  // Safety always preempts the FSM and bytecode interpreter.
  if (safety.isBlocked()) {
    drive.stop();
    mpu.resetSpeed();
  } else if (runMode == RUN_LINE) {
    navigation.update(currentTimeMs);
  } else if (runMode == RUN_DANCE) {
    danceVM.update(currentTimeMs);
  } else {
    drive.stop();
  }

  // Schedule slower display and logging tasks after movement decisions.
  if (rtcTimer.due(1000, currentTimeMs)) {
    rtc.update();
  }
  if (serialTimer.due(300, currentTimeMs)) {
    printDashboard();
  }
#if ENABLE_LCD
  if (lcdTimer.due(1000, currentTimeMs)) {
    lcd.update();
  }
#endif
#if ENABLE_SD
  if (sdTimer.due(1000, currentTimeMs)) {
    storage.log();
  }
#endif
}
