#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L1X.h>
#include <DFRobotDFPlayerMini.h>
#include <math.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

// I2C: MPU6050 and VL53L1X
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// Ultrasonic sensors
#define LEFT_TRIG_PIN  16
#define LEFT_ECHO_PIN  34
#define RIGHT_TRIG_PIN 17
#define RIGHT_ECHO_PIN 35
#define REAR_TRIG_PIN  12
#define REAR_ECHO_PIN  13

// DFPlayer UART2
#define DFPLAYER_RX_PIN 18  // ESP32 RX <- DFPlayer TX
#define DFPLAYER_TX_PIN 19  // ESP32 TX -> DFPlayer RX

// Vibration motor driver inputs
#define LEFT_MOTOR_PIN  25
#define RIGHT_MOTOR_PIN 26

// IMU motion thresholds
const float ACCEL_MOVING_THRESHOLD = 1.15f; // m/s^2 from gravity magnitude
const float GYRO_MOVING_THRESHOLD  = 0.45f; // rad/s
const float ACCEL_STILL_THRESHOLD  = 0.85f; // m/s^2
const float GYRO_STILL_THRESHOLD   = 0.30f; // rad/s

const unsigned long STANDING_CONFIRM_MS = 2000;
const unsigned long IMU_SAMPLE_INTERVAL_MS = 100;
const unsigned long MOTION_PRINT_INTERVAL_MS = 1000;

// Faster persistence filter than the old 10/7 filter.
const uint8_t WINDOW_SIZE = 5;
const uint8_t REQUIRED_MATCHES = 3;
const float TOLERANCE_CM = 8.0f;

const unsigned long FRONT_SAMPLE_INTERVAL_MS = 60;
const unsigned long DECISION_INTERVAL_MS = 80;
const unsigned long DISTANCE_PRINT_INTERVAL_MS = 1500;

// Three ultrasonic sensors are pinged one at a time to reduce cross-talk.
const unsigned long SONAR_TRIGGER_SPACING_MS = 35;
const uint32_t ULTRASONIC_TIMEOUT_US = 20000UL;

// Selection stability and anti-flicker.
const unsigned long CANDIDATE_CONFIRM_MS = 200;

const unsigned long WARNING_AUDIO_INTERVAL_MS = 4000;
const unsigned long HIGH_RISK_AUDIO_FALLBACK_MS = 2500;
const unsigned long STABLE_OBSTACLE_AUDIO_MUTE_MS = 6000;
const float STABLE_OBSTACLE_DISTANCE_DELTA_CM = 15.0f;
const uint8_t DFPLAYER_VOLUME = 18;
const bool DFPLAYER_STARTUP_TEST = true;

// Rear haptic pattern
const unsigned long REAR_HAPTIC_PHASE_MS = 1000;

// Vibration PWM
const uint32_t MOTOR_PWM_FREQUENCY_HZ = 5000;
const uint8_t MOTOR_PWM_RESOLUTION_BITS = 8;
const uint8_t LEFT_MOTOR_CHANNEL = 0;  // Used by ESP32 Arduino core 2.x
const uint8_t RIGHT_MOTOR_CHANNEL = 1; // Used by ESP32 Arduino core 2.x

Adafruit_MPU6050 mpu;
VL53L1X tof;
HardwareSerial dfSerial(2);
DFRobotDFPlayerMini player;

bool tofReady = false;
bool dfPlayerReady = false;

enum MotionMode {
  MODE_MOVING,
  MODE_STANDING
};

MotionMode currentMode = MODE_MOVING;

enum Direction {
  DIR_NONE,
  DIR_FRONT,
  DIR_LEFT,
  DIR_RIGHT,
  DIR_REAR
};

enum SonarId {
  SONAR_LEFT,
  SONAR_RIGHT,
  SONAR_REAR
};

struct ReadingFilter {
  float values[WINDOW_SIZE];
  uint8_t nextIndex;
  uint8_t sampleCount;
};

struct SensorState {
  bool reliable;
  float cm;
  uint8_t matches;
};

struct Obstacle {
  Direction direction;
  uint8_t zone;
  float cm;
  bool valid;
};

ReadingFilter frontFilter = {};
ReadingFilter leftFilter = {};
ReadingFilter rightFilter = {};
ReadingFilter rearFilter = {};

unsigned long stillSinceMs = 0;
unsigned long lastImuSampleMs = 0;
unsigned long lastMotionPrintMs = 0;
unsigned long lastFrontSampleMs = 0;
unsigned long lastDecisionMs = 0;
unsigned long lastDistancePrintMs = 0;
unsigned long lastSonarTriggerMs = 0;

bool stillTimerRunning = false;
bool motionFilterInitialized = false;
bool directionSensorsWereActive = false;
SonarId nextSonar = SONAR_LEFT;

float filteredAccelerationChange = 0.0f;
float filteredGyroMagnitude = 0.0f;

SensorState frontState = {};
SensorState leftState = {};
SensorState rightState = {};
SensorState rearState = {};

Direction activeDirection = DIR_NONE;
uint8_t activeZone = 0;
float activeDistanceCm = NAN;
uint8_t activeTrack = 0;
unsigned long lastWarningAudioMs = 0;

bool stableObstacleAudioMuted = false;
Direction stableWatchDirection = DIR_NONE;
uint8_t stableWatchZone = 0;
float stableWatchDistanceCm = NAN;
unsigned long stableWatchSinceMs = 0;

Direction candidateDirection = DIR_NONE;
uint8_t candidateZone = 0;
float candidateDistanceCm = NAN;
unsigned long candidateSinceMs = 0;

unsigned long rearHapticLastSwitchMs = 0;
bool rearHapticLeftPhase = true;

// Ultrasonic echo capture state.
volatile bool leftEchoArmed = false;
volatile bool leftEchoSawRise = false;
volatile bool leftEchoDone = false;
volatile uint32_t leftEchoRiseUs = 0;
volatile uint32_t leftEchoPulseUs = 0;

volatile bool rightEchoArmed = false;
volatile bool rightEchoSawRise = false;
volatile bool rightEchoDone = false;
volatile uint32_t rightEchoRiseUs = 0;
volatile uint32_t rightEchoPulseUs = 0;

volatile bool rearEchoArmed = false;
volatile bool rearEchoSawRise = false;
volatile bool rearEchoDone = false;
volatile uint32_t rearEchoRiseUs = 0;
volatile uint32_t rearEchoPulseUs = 0;

bool leftPingActive = false;
bool rightPingActive = false;
bool rearPingActive = false;
uint32_t leftPingStartedUs = 0;
uint32_t rightPingStartedUs = 0;
uint32_t rearPingStartedUs = 0;

void IRAM_ATTR leftEchoISR() {
  if (!leftEchoArmed) return;

  uint32_t edgeTime = micros();
  if (digitalRead(LEFT_ECHO_PIN) == HIGH) {
    leftEchoRiseUs = edgeTime;
    leftEchoSawRise = true;
  } else if (leftEchoSawRise) {
    leftEchoPulseUs = edgeTime - leftEchoRiseUs;
    leftEchoDone = true;
    leftEchoSawRise = false;
    leftEchoArmed = false;
  }
}

void IRAM_ATTR rightEchoISR() {
  if (!rightEchoArmed) return;

  uint32_t edgeTime = micros();
  if (digitalRead(RIGHT_ECHO_PIN) == HIGH) {
    rightEchoRiseUs = edgeTime;
    rightEchoSawRise = true;
  } else if (rightEchoSawRise) {
    rightEchoPulseUs = edgeTime - rightEchoRiseUs;
    rightEchoDone = true;
    rightEchoSawRise = false;
    rightEchoArmed = false;
  }
}

void IRAM_ATTR rearEchoISR() {
  if (!rearEchoArmed) return;

  uint32_t edgeTime = micros();
  if (digitalRead(REAR_ECHO_PIN) == HIGH) {
    rearEchoRiseUs = edgeTime;
    rearEchoSawRise = true;
  } else if (rearEchoSawRise) {
    rearEchoPulseUs = edgeTime - rearEchoRiseUs;
    rearEchoDone = true;
    rearEchoSawRise = false;
    rearEchoArmed = false;
  }
}

void resetFilter(ReadingFilter &filter) {
  for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
    filter.values[i] = NAN;
  }
  filter.nextIndex = 0;
  filter.sampleCount = 0;
}

void resetDirectionFilters() {
  resetFilter(leftFilter);
  resetFilter(rightFilter);
  resetFilter(rearFilter);
}

void clearSelection(unsigned long now) {
  activeDirection = DIR_NONE;
  activeZone = 0;
  activeDistanceCm = NAN;
  activeTrack = 0;
  lastWarningAudioMs = 0;

  stableObstacleAudioMuted = false;
  stableWatchDirection = DIR_NONE;
  stableWatchZone = 0;
  stableWatchDistanceCm = NAN;
  stableWatchSinceMs = now;

  candidateDirection = DIR_NONE;
  candidateZone = 0;
  candidateDistanceCm = NAN;
  candidateSinceMs = now;
}

void cancelUltrasonicCaptures() {
  noInterrupts();
  leftEchoArmed = false;
  leftEchoSawRise = false;
  leftEchoDone = false;
  rightEchoArmed = false;
  rightEchoSawRise = false;
  rightEchoDone = false;
  rearEchoArmed = false;
  rearEchoSawRise = false;
  rearEchoDone = false;
  interrupts();

  leftPingActive = false;
  rightPingActive = false;
  rearPingActive = false;
}

void addReading(ReadingFilter &filter, float distanceCm) {
  filter.values[filter.nextIndex] = distanceCm;
  filter.nextIndex = (filter.nextIndex + 1) % WINDOW_SIZE;

  if (filter.sampleCount < WINDOW_SIZE) {
    filter.sampleCount++;
  }
}

float getMedian(const ReadingFilter &filter) {
  float sorted[WINDOW_SIZE];
  uint8_t count = 0;

  for (uint8_t i = 0; i < filter.sampleCount; i++) {
    float value = filter.values[i];
    if (!isnan(value) && value > 0.0f) {
      sorted[count++] = value;
    }
  }

  if (count == 0) return NAN;

  for (uint8_t i = 0; i < count; i++) {
    for (uint8_t j = i + 1; j < count; j++) {
      if (sorted[j] < sorted[i]) {
        float temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }

  if (count % 2 == 1) {
    return sorted[count / 2];
  }

  return (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0f;
}

bool getPersistentDistance(const ReadingFilter &filter,
                           float &medianCm,
                           uint8_t &matchingCount) {
  matchingCount = 0;

  if (filter.sampleCount < WINDOW_SIZE) return false;

  medianCm = getMedian(filter);
  if (isnan(medianCm)) return false;

  for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
    float value = filter.values[i];
    if (!isnan(value) && fabsf(value - medianCm) <= TOLERANCE_CM) {
      matchingCount++;
    }
  }

  return matchingCount >= REQUIRED_MATCHES;
}

float readTofCm() {
  if (!tofReady || !tof.dataReady()) return NAN;

  uint16_t distanceMm = tof.read(false);
  tof.timeoutOccurred();

  if (distanceMm == 0) return NAN;

  float distanceCm = distanceMm / 10.0f;
  if (distanceCm < 2.0f || distanceCm > 400.0f) return NAN;
  return distanceCm;
}

float ultrasonicPulseToCm(uint32_t durationUs) {
  float distanceCm = durationUs * 0.0343f / 2.0f;
  if (distanceCm < 2.0f || distanceCm > 400.0f) return NAN;
  return distanceCm;
}

void startUltrasonicPing(SonarId sonar) {
  if (sonar == SONAR_LEFT) {
    noInterrupts();
    leftEchoSawRise = false;
    leftEchoDone = false;
    leftEchoArmed = true;
    interrupts();

    leftPingStartedUs = micros();
    leftPingActive = true;

    digitalWrite(LEFT_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(LEFT_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(LEFT_TRIG_PIN, LOW);
  } else if (sonar == SONAR_RIGHT) {
    noInterrupts();
    rightEchoSawRise = false;
    rightEchoDone = false;
    rightEchoArmed = true;
    interrupts();

    rightPingStartedUs = micros();
    rightPingActive = true;

    digitalWrite(RIGHT_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(RIGHT_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(RIGHT_TRIG_PIN, LOW);
  } else {
    noInterrupts();
    rearEchoSawRise = false;
    rearEchoDone = false;
    rearEchoArmed = true;
    interrupts();

    rearPingStartedUs = micros();
    rearPingActive = true;

    digitalWrite(REAR_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(REAR_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(REAR_TRIG_PIN, LOW);
  }
}

void serviceLeftEcho(uint32_t nowUs) {
  bool completed = false;
  bool timedOut = false;
  uint32_t pulseUs = 0;

  noInterrupts();
  if (leftEchoDone) {
    completed = true;
    pulseUs = leftEchoPulseUs;
    leftEchoDone = false;
  } else if (leftPingActive &&
             (uint32_t)(nowUs - leftPingStartedUs) >= ULTRASONIC_TIMEOUT_US) {
    leftEchoArmed = false;
    leftEchoSawRise = false;
    timedOut = true;
  }
  interrupts();

  if (completed) {
    leftPingActive = false;
    addReading(leftFilter, ultrasonicPulseToCm(pulseUs));
  } else if (timedOut) {
    leftPingActive = false;
    addReading(leftFilter, NAN);
  }
}

void serviceRightEcho(uint32_t nowUs) {
  bool completed = false;
  bool timedOut = false;
  uint32_t pulseUs = 0;

  noInterrupts();
  if (rightEchoDone) {
    completed = true;
    pulseUs = rightEchoPulseUs;
    rightEchoDone = false;
  } else if (rightPingActive &&
             (uint32_t)(nowUs - rightPingStartedUs) >= ULTRASONIC_TIMEOUT_US) {
    rightEchoArmed = false;
    rightEchoSawRise = false;
    timedOut = true;
  }
  interrupts();

  if (completed) {
    rightPingActive = false;
    addReading(rightFilter, ultrasonicPulseToCm(pulseUs));
  } else if (timedOut) {
    rightPingActive = false;
    addReading(rightFilter, NAN);
  }
}

void serviceRearEcho(uint32_t nowUs) {
  bool completed = false;
  bool timedOut = false;
  uint32_t pulseUs = 0;

  noInterrupts();
  if (rearEchoDone) {
    completed = true;
    pulseUs = rearEchoPulseUs;
    rearEchoDone = false;
  } else if (rearPingActive &&
             (uint32_t)(nowUs - rearPingStartedUs) >= ULTRASONIC_TIMEOUT_US) {
    rearEchoArmed = false;
    rearEchoSawRise = false;
    timedOut = true;
  }
  interrupts();

  if (completed) {
    rearPingActive = false;
    addReading(rearFilter, ultrasonicPulseToCm(pulseUs));
  } else if (timedOut) {
    rearPingActive = false;
    addReading(rearFilter, NAN);
  }
}

void serviceUltrasonic(unsigned long now) {
  uint32_t nowUs = micros();
  serviceLeftEcho(nowUs);
  serviceRightEcho(nowUs);
  serviceRearEcho(nowUs);

  bool directionSensorsActive = (currentMode == MODE_MOVING);

  if (!directionSensorsActive) {
    if (directionSensorsWereActive) {
      cancelUltrasonicCaptures();
      resetDirectionFilters();
    }
    directionSensorsWereActive = false;
    return;
  }

  if (!directionSensorsWereActive) {
    resetDirectionFilters();
    nextSonar = SONAR_LEFT;
    lastSonarTriggerMs = now;
  }
  directionSensorsWereActive = true;

  if (leftPingActive || rightPingActive || rearPingActive) return;
  if (now - lastSonarTriggerMs < SONAR_TRIGGER_SPACING_MS) return;

  startUltrasonicPing(nextSonar);
  lastSonarTriggerMs = now;

  if (nextSonar == SONAR_LEFT) nextSonar = SONAR_RIGHT;
  else if (nextSonar == SONAR_RIGHT) nextSonar = SONAR_REAR;
  else nextSonar = SONAR_LEFT;
}

uint8_t getZone(float distanceCm) {
  if (isnan(distanceCm)) return 0;
  if (distanceCm < 70.0f)  return 3; // Very close
  if (distanceCm < 150.0f) return 2; // Warning
  if (distanceCm <= 250.0f) return 1; // Safe
  return 0; // Out
}

const char *zoneName(uint8_t zone) {
  switch (zone) {
    case 3: return "VERY_CLOSE";
    case 2: return "WARNING";
    case 1: return "SAFE";
    default: return "OUT";
  }
}

const char *directionName(Direction direction) {
  switch (direction) {
    case DIR_FRONT: return "FRONT";
    case DIR_LEFT:  return "LEFT";
    case DIR_RIGHT: return "RIGHT";
    case DIR_REAR:  return "REAR";
    default:        return "NONE";
  }
}

void printSensor(const char *name, bool active, const SensorState &state) {
  Serial.print(name);
  Serial.print("=");

  if (!active) {
    Serial.print("STANDBY");
    return;
  }

  if (!state.reliable) {
    Serial.print("WAIT");
    return;
  }

  Serial.print(state.cm, 1);
  Serial.print("cm/");
  Serial.print(zoneName(getZone(state.cm)));
  Serial.print("/");
  Serial.print(state.matches);
  Serial.print("/");
  Serial.print(WINDOW_SIZE);
}

Obstacle makeObstacle(Direction direction, const SensorState &state) {
  Obstacle obstacle;
  obstacle.direction = direction;
  obstacle.zone = state.reliable ? getZone(state.cm) : 0;
  obstacle.cm = state.cm;
  obstacle.valid = state.reliable && obstacle.zone > 0;
  return obstacle;
}

bool isBetterObstacle(const Obstacle &candidate, const Obstacle &current) {
  if (!candidate.valid) return false;
  if (!current.valid) return true;

  if (candidate.zone != current.zone) {
    return candidate.zone > current.zone;
  }

  return candidate.cm < current.cm;
}

Obstacle chooseRawPriority(bool directionSensorsActive) {
  Obstacle best;
  best.direction = DIR_NONE;
  best.zone = 0;
  best.cm = NAN;
  best.valid = false;

  Obstacle front = makeObstacle(DIR_FRONT, frontState);
  if (isBetterObstacle(front, best)) best = front;

  if (directionSensorsActive) {
    Obstacle left = makeObstacle(DIR_LEFT, leftState);
    Obstacle right = makeObstacle(DIR_RIGHT, rightState);
    Obstacle rear = makeObstacle(DIR_REAR, rearState);

    if (isBetterObstacle(left, best)) best = left;
    if (isBetterObstacle(right, best)) best = right;
    if (isBetterObstacle(rear, best)) best = rear;
  }

  return best;
}

bool shouldSwitchFromActive(const Obstacle &rawBest, unsigned long now) {
  if (!rawBest.valid) return activeDirection != DIR_NONE;
  if (activeDirection == DIR_NONE) return true;

  if (rawBest.direction == activeDirection && rawBest.zone == activeZone) {
    activeDistanceCm = rawBest.cm;
    return false;
  }

  if (rawBest.zone > activeZone) return true;
  if (rawBest.zone < activeZone) {
    return now - candidateSinceMs >= CANDIDATE_CONFIRM_MS;
  }

  float sameZoneMarginCm = 20.0f;
  if (rawBest.zone == 3) sameZoneMarginCm = 2.0f;
  else if (rawBest.zone == 2) sameZoneMarginCm = 10.0f;

  if (rawBest.cm + sameZoneMarginCm < activeDistanceCm) {
    return now - candidateSinceMs >= CANDIDATE_CONFIRM_MS;
  }

  return false;
}

uint8_t getTrackForObstacle(Direction direction, uint8_t zone) {
  if (zone < 1 || zone > 3) return 0;

  uint8_t offset = zone - 1;

  switch (direction) {
    case DIR_FRONT: return 1 + offset;
    case DIR_LEFT:  return 4 + offset;
    case DIR_RIGHT: return 7 + offset;
    case DIR_REAR:  return 10 + offset;
    default:        return 0;
  }
}

void setupMotorPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  bool leftOk =
    ledcAttach(LEFT_MOTOR_PIN, MOTOR_PWM_FREQUENCY_HZ,
               MOTOR_PWM_RESOLUTION_BITS);
  bool rightOk =
    ledcAttach(RIGHT_MOTOR_PIN, MOTOR_PWM_FREQUENCY_HZ,
               MOTOR_PWM_RESOLUTION_BITS);

  if (!leftOk || !rightOk) {
    Serial.println("ERROR: Could not configure motor PWM.");
  }

  ledcWrite(LEFT_MOTOR_PIN, 0);
  ledcWrite(RIGHT_MOTOR_PIN, 0);
#else
  ledcSetup(LEFT_MOTOR_CHANNEL, MOTOR_PWM_FREQUENCY_HZ,
            MOTOR_PWM_RESOLUTION_BITS);
  ledcSetup(RIGHT_MOTOR_CHANNEL, MOTOR_PWM_FREQUENCY_HZ,
            MOTOR_PWM_RESOLUTION_BITS);

  ledcAttachPin(LEFT_MOTOR_PIN, LEFT_MOTOR_CHANNEL);
  ledcAttachPin(RIGHT_MOTOR_PIN, RIGHT_MOTOR_CHANNEL);

  ledcWrite(LEFT_MOTOR_CHANNEL, 0);
  ledcWrite(RIGHT_MOTOR_CHANNEL, 0);
#endif
}

void writeMotorDuty(uint8_t pin, uint8_t channel, uint8_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

void setMotorIntensity(uint8_t leftPercent, uint8_t rightPercent) {
  uint8_t leftDuty = (uint16_t)leftPercent * 255 / 100;
  uint8_t rightDuty = (uint16_t)rightPercent * 255 / 100;

  writeMotorDuty(LEFT_MOTOR_PIN, LEFT_MOTOR_CHANNEL, leftDuty);
  writeMotorDuty(RIGHT_MOTOR_PIN, RIGHT_MOTOR_CHANNEL, rightDuty);
}

void stopMotors() {
  setMotorIntensity(0, 0);
}

uint8_t getIntensityForZone(uint8_t zone) {
  if (zone == 1) return 30;
  if (zone == 2) return 60;
  if (zone == 3) return 100;
  return 0;
}

void applyHaptic(Direction direction, uint8_t zone, unsigned long now) {
  uint8_t intensity = getIntensityForZone(zone);

  if (direction == DIR_FRONT) {
    setMotorIntensity(intensity, intensity);
  } else if (direction == DIR_LEFT) {
    setMotorIntensity(intensity, 0);
  } else if (direction == DIR_RIGHT) {
    setMotorIntensity(0, intensity);
  } else if (direction == DIR_REAR) {
    if (now - rearHapticLastSwitchMs >= REAR_HAPTIC_PHASE_MS) {
      rearHapticLastSwitchMs = now;
      rearHapticLeftPhase = !rearHapticLeftPhase;
    }

    if (rearHapticLeftPhase) {
      setMotorIntensity(intensity, 0);
    } else {
      setMotorIntensity(0, intensity);
    }
  } else {
    stopMotors();
  }
}

void resetStableObstacleAudioWatch(unsigned long now) {
  stableObstacleAudioMuted = false;
  stableWatchDirection = activeDirection;
  stableWatchZone = activeZone;
  stableWatchDistanceCm = activeDistanceCm;
  stableWatchSinceMs = now;
}

void playActiveAudioNow(const char *reason, unsigned long now) {
  if (!dfPlayerReady || activeTrack == 0 || stableObstacleAudioMuted) return;

  player.playMp3Folder(activeTrack);
  lastWarningAudioMs = now;

  Serial.print(reason);
  Serial.print(" track ");
  Serial.println(activeTrack);
}

void updateStableObstacleAudioMute(unsigned long now) {
  if (activeDirection == DIR_NONE || activeZone == 0 || isnan(activeDistanceCm)) {
    stableObstacleAudioMuted = false;
    stableWatchDirection = DIR_NONE;
    stableWatchZone = 0;
    stableWatchDistanceCm = NAN;
    stableWatchSinceMs = now;
    return;
  }

  bool watchChanged =
    activeDirection != stableWatchDirection ||
    activeZone != stableWatchZone ||
    isnan(stableWatchDistanceCm);

  if (watchChanged) {
    resetStableObstacleAudioWatch(now);
    return;
  }

  bool distanceChanged =
    fabsf(activeDistanceCm - stableWatchDistanceCm) >
    STABLE_OBSTACLE_DISTANCE_DELTA_CM;

  if (distanceChanged) {
    bool wasMuted = stableObstacleAudioMuted;
    resetStableObstacleAudioWatch(now);

    if (wasMuted) {
      playActiveAudioNow("Audio unmuted for changed obstacle", now);
    }
    return;
  }

  if (!stableObstacleAudioMuted &&
      now - stableWatchSinceMs >= STABLE_OBSTACLE_AUDIO_MUTE_MS) {
    stableObstacleAudioMuted = true;

    if (dfPlayerReady && activeTrack != 0) {
      player.stop();
    }

    Serial.println("Audio muted: obstacle stayed at same distance. Haptics remain active.");
  }
}

void activateObstacle(const Obstacle &obstacle, unsigned long now) {
  uint8_t newTrack = obstacle.valid ?
    getTrackForObstacle(obstacle.direction, obstacle.zone) : 0;

  if (newTrack == 0) {
    if (activeTrack != 0 && dfPlayerReady) {
      player.stop();
    }
    clearSelection(now);
    stopMotors();
    return;
  }

  bool selectionChanged =
    obstacle.direction != activeDirection || obstacle.zone != activeZone;

  activeDirection = obstacle.direction;
  activeZone = obstacle.zone;
  activeDistanceCm = obstacle.cm;

  if (selectionChanged) {
    if (activeTrack != 0 && dfPlayerReady) {
      player.stop();
    }

    activeTrack = newTrack;
    lastWarningAudioMs = now;
    rearHapticLastSwitchMs = now;
    rearHapticLeftPhase = true;
    resetStableObstacleAudioWatch(now);

    if (dfPlayerReady) {
      Serial.print("Playing obstacle audio track ");
      Serial.println(activeTrack);
      player.playMp3Folder(activeTrack);
    }
  }

  applyHaptic(activeDirection, activeZone, now);
}

void updateDecision(unsigned long now) {
  frontState.reliable =
    getPersistentDistance(frontFilter, frontState.cm, frontState.matches);

  bool directionSensorsActive = (currentMode == MODE_MOVING);

  leftState = {};
  rightState = {};
  rearState = {};
  leftState.cm = NAN;
  rightState.cm = NAN;
  rearState.cm = NAN;

  if (directionSensorsActive) {
    leftState.reliable =
      getPersistentDistance(leftFilter, leftState.cm, leftState.matches);
    rightState.reliable =
      getPersistentDistance(rightFilter, rightState.cm, rightState.matches);
    rearState.reliable =
      getPersistentDistance(rearFilter, rearState.cm, rearState.matches);
  }

  Obstacle rawBest = chooseRawPriority(directionSensorsActive);

  bool candidateChanged =
    rawBest.direction != candidateDirection || rawBest.zone != candidateZone;

  if (candidateChanged) {
    candidateDirection = rawBest.direction;
    candidateZone = rawBest.zone;
    candidateDistanceCm = rawBest.cm;
    candidateSinceMs = now;
  } else {
    candidateDistanceCm = rawBest.cm;
  }

  if (shouldSwitchFromActive(rawBest, now)) {
    activateObstacle(rawBest, now);
  } else {
    applyHaptic(activeDirection, activeZone, now);
  }

  updateStableObstacleAudioMute(now);
}

void serviceAudio(unsigned long now) {
  if (!dfPlayerReady || activeTrack == 0) return;
  if (stableObstacleAudioMuted) return;

  if (activeZone == 2 &&
      now - lastWarningAudioMs >= WARNING_AUDIO_INTERVAL_MS) {
    player.playMp3Folder(activeTrack);
    lastWarningAudioMs = now;

    Serial.print("Repeating warning audio track ");
    Serial.println(activeTrack);
  } else if (activeZone == 3 &&
             now - lastWarningAudioMs >= HIGH_RISK_AUDIO_FALLBACK_MS) {
    player.playMp3Folder(activeTrack);
    lastWarningAudioMs = now;

    Serial.print("High-risk audio fallback repeat track ");
    Serial.println(activeTrack);
  }
}

void serviceDfPlayerEvents() {
  if (!dfPlayerReady || !player.available()) return;

  uint8_t eventType = player.readType();
  int value = player.read();

  if (eventType == DFPlayerError) {
    Serial.print("DFPlayer error: ");
    Serial.println(value);
  } else if (eventType == DFPlayerPlayFinished) {
    Serial.print("Audio track finished: ");
    Serial.println(value);

    if (activeZone == 3 &&
        activeTrack != 0 &&
        !stableObstacleAudioMuted &&
        value == activeTrack) {
      player.playMp3Folder(activeTrack);
      lastWarningAudioMs = millis();
      Serial.print("Repeating high-risk audio track ");
      Serial.println(activeTrack);
    }
  }
}

void updateMotionMode(unsigned long now) {
  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;

  if (!mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent)) {
    Serial.println("ERROR: MPU6050 read failed.");
    stillTimerRunning = false;
    currentMode = MODE_MOVING;
    return;
  }

  float ax = accelEvent.acceleration.x;
  float ay = accelEvent.acceleration.y;
  float az = accelEvent.acceleration.z;
  float accelMagnitude = sqrtf(ax * ax + ay * ay + az * az);
  float rawAccelerationChange = fabsf(accelMagnitude - 9.81f);

  float gx = gyroEvent.gyro.x;
  float gy = gyroEvent.gyro.y;
  float gz = gyroEvent.gyro.z;
  float rawGyroMagnitude = sqrtf(gx * gx + gy * gy + gz * gz);

  if (!motionFilterInitialized) {
    filteredAccelerationChange = rawAccelerationChange;
    filteredGyroMagnitude = rawGyroMagnitude;
    motionFilterInitialized = true;
  } else {
    const float alpha = 0.35f;
    filteredAccelerationChange =
      alpha * rawAccelerationChange +
      (1.0f - alpha) * filteredAccelerationChange;
    filteredGyroMagnitude =
      alpha * rawGyroMagnitude +
      (1.0f - alpha) * filteredGyroMagnitude;
  }

  bool movingNow =
    filteredAccelerationChange >= ACCEL_MOVING_THRESHOLD ||
    filteredGyroMagnitude >= GYRO_MOVING_THRESHOLD;

  bool stillNow =
    filteredAccelerationChange <= ACCEL_STILL_THRESHOLD &&
    filteredGyroMagnitude <= GYRO_STILL_THRESHOLD;

  MotionMode previousMode = currentMode;

  if (movingNow) {
    stillTimerRunning = false;
    currentMode = MODE_MOVING;
  } else if (stillNow) {
    if (!stillTimerRunning) {
      stillTimerRunning = true;
      stillSinceMs = now;
    } else if (now - stillSinceMs >= STANDING_CONFIRM_MS) {
      currentMode = MODE_STANDING;
    }
  } else {
    stillTimerRunning = false;
  }

  if (previousMode != currentMode) {
    Serial.print("Mode: ");
    Serial.println(currentMode == MODE_MOVING ? "MOVING" : "STANDING");
    resetStableObstacleAudioWatch(now);

    if (currentMode == MODE_STANDING) {
      cancelUltrasonicCaptures();
      resetDirectionFilters();

      if (activeDirection != DIR_FRONT && activeDirection != DIR_NONE) {
        clearSelection(now);
        stopMotors();
      }
    } else {
      resetDirectionFilters();
    }
  }

  if (now - lastMotionPrintMs >= MOTION_PRINT_INTERVAL_MS) {
    lastMotionPrintMs = now;
    Serial.print("Motion ");
    Serial.print(currentMode == MODE_MOVING ? "MOVING" : "STANDING");
    Serial.print(" accel=");
    Serial.print(filteredAccelerationChange, 2);
    Serial.print(" gyro=");
    Serial.println(filteredGyroMagnitude, 2);
  }
}

void printDistances(unsigned long now) {
  if (now - lastDistancePrintMs < DISTANCE_PRINT_INTERVAL_MS) return;
  lastDistancePrintMs = now;

  bool directionSensorsActive = (currentMode == MODE_MOVING);

  printSensor("FRONT", true, frontState);
  Serial.print("  ");
  printSensor("LEFT", directionSensorsActive, leftState);
  Serial.print("  ");
  printSensor("RIGHT", directionSensorsActive, rightState);
  Serial.print("  ");
  printSensor("REAR", directionSensorsActive, rearState);
  Serial.print("  ACTIVE=");
  Serial.print(directionName(activeDirection));
  Serial.print("/");
  Serial.println(zoneName(activeZone));
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  pinMode(REAR_TRIG_PIN, OUTPUT);
  pinMode(REAR_ECHO_PIN, INPUT);

  digitalWrite(LEFT_TRIG_PIN, LOW);
  digitalWrite(RIGHT_TRIG_PIN, LOW);
  digitalWrite(REAR_TRIG_PIN, LOW);

  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  setupMotorPwm();

  attachInterrupt(digitalPinToInterrupt(LEFT_ECHO_PIN),
                  leftEchoISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ECHO_PIN),
                  rightEchoISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REAR_ECHO_PIN),
                  rearEchoISR, CHANGE);

  resetFilter(frontFilter);
  resetDirectionFilters();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  Serial.println("Starting MPU6050 motion detection...");

  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found. Check power, ground, SDA, and SCL.");
    while (true) delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 ready.");

  tof.setTimeout(100);

  if (tof.init()) {
    tof.setDistanceMode(VL53L1X::Long);
    tof.setMeasurementTimingBudget(50000);
    tof.startContinuous(50);
    tofReady = true;
    Serial.println("ToF ready.");
  } else {
    Serial.println("ToF unavailable; front readings will show WAIT.");
  }

  dfSerial.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
  Serial.println("Starting DFPlayer...");

  if (player.begin(dfSerial)) {
    dfPlayerReady = true;
    player.volume(DFPLAYER_VOLUME);
    Serial.println("DFPlayer ready. Volume set to 18.");
    if (DFPLAYER_STARTUP_TEST) {
      Serial.println("DFPlayer startup test: playing /mp3/0001.mp3");
      player.playMp3Folder(1);
    }
  } else {
    Serial.println("DFPlayer initialization failed. Audio disabled.");
  }

  clearSelection(millis());
  Serial.println("Mode: MOVING");
}

void loop() {
  unsigned long now = millis();

  if (now - lastImuSampleMs >= IMU_SAMPLE_INTERVAL_MS) {
    lastImuSampleMs = now;
    updateMotionMode(now);
  }

  if (now - lastFrontSampleMs >= FRONT_SAMPLE_INTERVAL_MS) {
    lastFrontSampleMs = now;
    addReading(frontFilter, readTofCm());
  }

  serviceUltrasonic(now);

  if (now - lastDecisionMs >= DECISION_INTERVAL_MS) {
    lastDecisionMs = now;
    updateDecision(now);
    printDistances(now);
  }

  serviceAudio(now);
  serviceDfPlayerEvents();
}
