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

// Side ultrasonic sensors
#define LEFT_TRIG_PIN  16
#define LEFT_ECHO_PIN  34
#define RIGHT_TRIG_PIN 17
#define RIGHT_ECHO_PIN 35

// DFPlayer UART2
#define DFPLAYER_RX_PIN 18  // ESP32 RX <- DFPlayer TX
#define DFPLAYER_TX_PIN 19  // ESP32 TX -> DFPlayer RX

// Vibration motor driver inputs
#define LEFT_MOTOR_PIN  25
#define RIGHT_MOTOR_PIN 26

// IMU motion thresholds
const float ACCEL_MOVING_THRESHOLD = 0.80f; // m/s^2 from gravity magnitude
const float GYRO_MOVING_THRESHOLD  = 0.35f; // rad/s
const float ACCEL_STILL_THRESHOLD  = 0.50f; // m/s^2
const float GYRO_STILL_THRESHOLD   = 0.25f; // rad/s

const unsigned long STANDING_CONFIRM_MS = 5000;
const unsigned long IMU_SAMPLE_INTERVAL_MS = 100;
const unsigned long MOTION_PRINT_INTERVAL_MS = 500;

// Distance filter
const uint8_t WINDOW_SIZE = 10;
const uint8_t REQUIRED_MATCHES = 7;
const float TOLERANCE_CM = 5.0f;

const unsigned long DISTANCE_SAMPLE_INTERVAL_MS = 100;
const unsigned long DECISION_INTERVAL_MS = 100;
const unsigned long DISTANCE_PRINT_INTERVAL_MS = 2000;

// Ultrasonic timing:
// Left and right triggers are 50 ms apart; each sensor is triggered
// about once every 100 ms. The 20 ms timeout corresponds to about 343 cm.
const unsigned long SONAR_TRIGGER_SPACING_MS = 50;
const uint32_t ULTRASONIC_TIMEOUT_US = 20000UL;

const unsigned long WARNING_AUDIO_INTERVAL_MS = 4000;
const uint8_t DFPLAYER_VOLUME = 18;

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
  DIR_RIGHT
};

struct ReadingFilter {
  float values[WINDOW_SIZE];
  uint8_t nextIndex;
  uint8_t sampleCount;
};

ReadingFilter frontFilter = {};
ReadingFilter leftFilter = {};
ReadingFilter rightFilter = {};

unsigned long stillSinceMs = 0;
unsigned long lastImuSampleMs = 0;
unsigned long lastMotionPrintMs = 0;
unsigned long lastFrontSampleMs = 0;
unsigned long lastDecisionMs = 0;
unsigned long lastDistancePrintMs = 0;
unsigned long lastSonarTriggerMs = 0;

bool stillTimerRunning = false;
bool motionFilterInitialized = false;
bool sideSensorsWereActive = false;
bool nextSonarIsLeft = true;

float filteredAccelerationChange = 0.0f;
float filteredGyroMagnitude = 0.0f;

float frontCm = NAN;
float leftCm = NAN;
float rightCm = NAN;

uint8_t frontMatches = 0;
uint8_t leftMatches = 0;
uint8_t rightMatches = 0;

bool frontReliable = false;
bool leftReliable = false;
bool rightReliable = false;

Direction selectedDirection = DIR_NONE;
uint8_t selectedZone = 0;

// Audio selection state
Direction activeDirection = DIR_NONE;
uint8_t activeZone = 0;
uint8_t activeTrack = 0;
unsigned long lastWarningAudioMs = 0;

// Ultrasonic echo capture state.
// Echo edges are captured by GPIO interrupts so pulseIn() does not block
// the 100 ms IMU and decision schedule.
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

bool leftPingActive = false;
bool rightPingActive = false;
uint32_t leftPingStartedUs = 0;
uint32_t rightPingStartedUs = 0;

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

void resetFilter(ReadingFilter &filter) {
  for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
    filter.values[i] = NAN;
  }

  filter.nextIndex = 0;
  filter.sampleCount = 0;
}

void resetSideFilters() {
  resetFilter(leftFilter);
  resetFilter(rightFilter);
}

void cancelUltrasonicCaptures() {
  noInterrupts();

  leftEchoArmed = false;
  leftEchoSawRise = false;
  leftEchoDone = false;

  rightEchoArmed = false;
  rightEchoSawRise = false;
  rightEchoDone = false;

  interrupts();

  leftPingActive = false;
  rightPingActive = false;
}

void addReading(ReadingFilter &filter, float distanceCm) {
  // Invalid readings are stored so older readings eventually expire.
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

  // Non-blocking read: dataReady() was checked first.
  uint16_t distanceMm = tof.read(false);
  tof.timeoutOccurred();

  if (distanceMm == 0) return NAN;

  float distanceCm = distanceMm / 10.0f;

  if (distanceCm > 400.0f) return NAN;
  return distanceCm;
}

void startUltrasonicPing(bool useLeftSensor) {
  if (useLeftSensor) {
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
  } else {
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
  }
}

float ultrasonicPulseToCm(uint32_t durationUs) {
  float distanceCm = durationUs * 0.0343f / 2.0f;

  // Reject readings outside the practical HC-SR04 range.
  if (distanceCm < 2.0f || distanceCm > 400.0f) return NAN;
  return distanceCm;
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

uint8_t getZone(float distanceCm) {
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
    default:        return "NONE";
  }
}

void printSensor(const char *name, bool active, bool reliable,
                 float distanceCm, uint8_t matchingCount) {
  Serial.print(name);
  Serial.print("=");

  if (!active) {
    Serial.print("STANDBY");
    return;
  }

  if (!reliable) {
    Serial.print("WAIT");
    return;
  }

  Serial.print(distanceCm, 1);
  Serial.print("cm/");
  Serial.print(zoneName(getZone(distanceCm)));
  Serial.print("/");
  Serial.print(matchingCount * 10);
  Serial.print("%");
}

Direction choosePriority(bool fReliable, float fCm,
                         bool lReliable, float lCm,
                         bool rReliable, float rCm) {
  // First choose the most severe zone. Within the same zone, choose
  // the closest obstacle. Exact ties retain FRONT, LEFT, RIGHT order.
  Direction bestDirection = DIR_NONE;
  uint8_t bestZone = 0;
  float bestDistance = INFINITY;

  if (fReliable) {
    uint8_t zone = getZone(fCm);
    if (zone > bestZone ||
        (zone > 0 && zone == bestZone && fCm < bestDistance)) {
      bestDirection = DIR_FRONT;
      bestZone = zone;
      bestDistance = fCm;
    }
  }

  if (lReliable) {
    uint8_t zone = getZone(lCm);
    if (zone > bestZone ||
        (zone > 0 && zone == bestZone && lCm < bestDistance)) {
      bestDirection = DIR_LEFT;
      bestZone = zone;
      bestDistance = lCm;
    }
  }

  if (rReliable) {
    uint8_t zone = getZone(rCm);
    if (zone > bestZone ||
        (zone > 0 && zone == bestZone && rCm < bestDistance)) {
      bestDirection = DIR_RIGHT;
      bestZone = zone;
      bestDistance = rCm;
    }
  }

  return bestDirection;
}

uint8_t getTrackForObstacle(Direction direction, uint8_t zone) {
  if (zone < 1 || zone > 3) return 0;

  // Track groups: front 1–3, left 4–6, right 7–9.
  // In each group: safe, warning, very close.
  uint8_t offset = zone - 1;

  switch (direction) {
    case DIR_FRONT: return 1 + offset;
    case DIR_LEFT:  return 4 + offset;
    case DIR_RIGHT: return 7 + offset;
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

void applyHaptic(Direction direction, uint8_t zone) {
  uint8_t intensity = 0;

  if (zone == 1) intensity = 30;
  else if (zone == 2) intensity = 60;
  else if (zone == 3) intensity = 100;

  if (direction == DIR_FRONT) {
    setMotorIntensity(intensity, intensity);
  } else if (direction == DIR_LEFT) {
    setMotorIntensity(intensity, 0);
  } else if (direction == DIR_RIGHT) {
    setMotorIntensity(0, intensity);
  } else {
    stopMotors();
  }
}

void selectObstacle(Direction direction, uint8_t zone, unsigned long now) {
  uint8_t track = getTrackForObstacle(direction, zone);

  if (track == 0) {
    if (activeTrack != 0 && dfPlayerReady) {
      player.stop();
    }

    activeDirection = DIR_NONE;
    activeZone = 0;
    activeTrack = 0;
    lastWarningAudioMs = 0;
    stopMotors();
    return;
  }

  bool selectionChanged =
    direction != activeDirection || zone != activeZone;

  if (selectionChanged) {
    if (activeTrack != 0 && dfPlayerReady) {
      player.stop();
    }

    activeDirection = direction;
    activeZone = zone;
    activeTrack = track;
    lastWarningAudioMs = now;

    // On a selection change, play the matching track immediately.
    // Safe audio is not replayed while this same selection remains active.
    if (dfPlayerReady) {
      Serial.print("Playing obstacle audio track ");
      Serial.println(activeTrack);
      player.playMp3Folder(activeTrack);
    }
  }

  // Motors remain at the selected intensity until direction/zone changes.
  applyHaptic(activeDirection, activeZone);
}

void updateDecision(unsigned long now) {
  frontReliable =
    getPersistentDistance(frontFilter, frontCm, frontMatches);

  bool sideSensorsActive = (currentMode == MODE_MOVING);

  leftReliable = false;
  rightReliable = false;
  leftCm = NAN;
  rightCm = NAN;
  leftMatches = 0;
  rightMatches = 0;

  if (sideSensorsActive) {
    leftReliable =
      getPersistentDistance(leftFilter, leftCm, leftMatches);
    rightReliable =
      getPersistentDistance(rightFilter, rightCm, rightMatches);
  }

  selectedDirection = choosePriority(
    frontReliable, frontCm,
    leftReliable, leftCm,
    rightReliable, rightCm
  );

  selectedZone = 0;

  if (selectedDirection == DIR_FRONT) selectedZone = getZone(frontCm);
  if (selectedDirection == DIR_LEFT)  selectedZone = getZone(leftCm);
  if (selectedDirection == DIR_RIGHT) selectedZone = getZone(rightCm);

  selectObstacle(selectedDirection, selectedZone, now);
}

void serviceAudio(unsigned long now) {
  if (!dfPlayerReady || activeTrack == 0) return;

  // Warning audio repeats every four seconds while the selection is unchanged.
  if (activeZone == 2 &&
      now - lastWarningAudioMs >= WARNING_AUDIO_INTERVAL_MS) {
    player.playMp3Folder(activeTrack);
    lastWarningAudioMs = now;

    Serial.print("Repeating warning audio track ");
    Serial.println(activeTrack);
  }

  // Safe audio plays once. Very-close audio is restarted on its
  // DFPlayerPlayFinished event in the main loop.
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

    // Repeat high-risk audio immediately after completion while the
    // same very-close selection remains active.
    if (activeZone == 3 &&
        activeTrack != 0 &&
        value == activeTrack) {
      player.playMp3Folder(activeTrack);
      Serial.print("Repeating high-risk audio track ");
      Serial.println(activeTrack);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);

  digitalWrite(LEFT_TRIG_PIN, LOW);
  digitalWrite(RIGHT_TRIG_PIN, LOW);

  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  setupMotorPwm();

  attachInterrupt(digitalPinToInterrupt(LEFT_ECHO_PIN),
                  leftEchoISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ECHO_PIN),
                  rightEchoISR, CHANGE);

  resetFilter(frontFilter);
  resetFilter(leftFilter);
  resetFilter(rightFilter);

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
  } else {
    Serial.println("DFPlayer initialization failed. Audio disabled.");
  }

  Serial.println("Mode: MOVING");
}

void loop() {
  unsigned long now = millis();

  // Sample the IMU every 100 ms.
  if (now - lastImuSampleMs >= IMU_SAMPLE_INTERVAL_MS) {
    lastImuSampleMs = now;

    sensors_event_t accelEvent;
    sensors_event_t gyroEvent;
    sensors_event_t tempEvent;

    if (!mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent)) {
      Serial.println("ERROR: MPU6050 read failed.");
      stillTimerRunning = false;
      currentMode = MODE_MOVING;
    } else {
      float ax = accelEvent.acceleration.x;
      float ay = accelEvent.acceleration.y;
      float az = accelEvent.acceleration.z;

      float accelMagnitude = sqrtf(ax * ax + ay * ay + az * az);
      float rawAccelerationChange = fabsf(accelMagnitude - 9.81f);

      float gx = gyroEvent.gyro.x;
      float gy = gyroEvent.gyro.y;
      float gz = gyroEvent.gyro.z;
      float rawGyroMagnitude = sqrtf(gx * gx + gy * gy + gz * gz);

      // Light low-pass filtering reduces noisy threshold crossings
      // that can repeatedly reset stand
