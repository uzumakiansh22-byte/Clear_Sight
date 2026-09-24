/*
  ============================================================
  ClearSight Assistive Glasses - ESP32 Prototype
  ============================================================

  Hardware:
  - ESP32-WROOM / ESP32 DevKit
  - VL53L1X ToF
  - MPU6050 IMU
  - Left HC-SR04
  - Right HC-SR04
  - Left vibration motor
  - Right vibration motor
  - DFPlayer Mini

  I2C:
  SDA = GPIO 21
  SCL = GPIO 22

  Ultrasonic:
  Left:
    TRIG = GPIO 16
    ECHO = GPIO 34

  Right:
    TRIG = GPIO 17
    ECHO = GPIO 35

  Motors:
    Left  = GPIO 25
    Right = GPIO 26

  DFPlayer:
    DFPlayer TX -> ESP32 GPIO 18
    DFPlayer RX -> ESP32 GPIO 19

  IMPORTANT:
  HC-SR04 ECHO must be level shifted / voltage divided
  before connecting to ESP32 GPIO.

  Motor outputs should control MOSFET/transistor drivers,
  NOT the motors directly from ESP32 GPIO.

  DFPlayer SD:
    /MP3/0001.mp3
    /MP3/0002.mp3
    ...
    /MP3/0009.mp3
*/


// ============================================================
// LIBRARIES
// ============================================================

#include <Wire.h>
#include <VL53L1X.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <DFRobotDFPlayerMini.h>


// ============================================================
// PIN DEFINITIONS
// ============================================================

// I2C
#define SDA_PIN 21
#define SCL_PIN 22

// Left ultrasonic
#define LEFT_TRIG_PIN 16
#define LEFT_ECHO_PIN 34

// Right ultrasonic
#define RIGHT_TRIG_PIN 17
#define RIGHT_ECHO_PIN 35

// Vibration motors
#define LEFT_MOTOR_PIN 25
#define RIGHT_MOTOR_PIN 26

// DFPlayer
#define DF_RX_PIN 18
#define DF_TX_PIN 19


// ============================================================
// DISTANCE SETTINGS
// ============================================================

// All internal distance calculations are in millimeters.

const float SAFE_DISTANCE_MM = 2000.0;

const float LOW_RISK_DISTANCE_MM = 1000.0;

const float HIGH_RISK_DISTANCE_MM = 300.0;


// ============================================================
// SENSOR TIMING
// ============================================================

const unsigned long SENSOR_INTERVAL_MS = 80;

const unsigned long DEBUG_INTERVAL_MS = 1000;

const unsigned long MOVEMENT_INTERVAL_MS = 100;

const unsigned long STANDING_TIME_MS = 5000;


// ============================================================
// PERSISTENCE SETTINGS
// ============================================================

const int REQUIRED_CONFIRMATIONS = 2;

const int REQUIRED_CLEAR_READINGS = 3;


// ============================================================
// CONFIDENCE SETTINGS
// ============================================================

const int MIN_CONFIDENCE_FOR_ALERT = 45;


// ============================================================
// SENSOR HEALTH
// ============================================================

enum SensorHealth
{
  SENSOR_WORKING,
  SENSOR_DEGRADED,
  SENSOR_FAILED
};


// ============================================================
// DIRECTION
// ============================================================

enum Direction
{
  DIR_NONE,
  DIR_FRONT,
  DIR_LEFT,
  DIR_RIGHT
};


// ============================================================
// RISK LEVEL
//
// IMPORTANT:
// Do NOT name these LOW/HIGH because Arduino defines
// LOW and HIGH as GPIO macros.
// ============================================================

enum RiskLevel
{
  RISK_SAFE,
  RISK_LOW,
  RISK_MEDIUM,
  RISK_HIGH
};


// ============================================================
// MOVEMENT STATE
// ============================================================

enum MovementState
{
  MOVEMENT_UNKNOWN,
  MOVEMENT_MOVING,
  MOVEMENT_STANDING
};


// ============================================================
// SENSOR STATE
// ============================================================

struct SensorState
{
  float rawDistanceMM;
  float filteredDistanceMM;

  bool valid;

  int confidence;

  RiskLevel risk;

  SensorHealth health;

  int validCount;
  int invalidCount;

  int confirmationCount;
  int clearCount;

  unsigned long lastValidTime;
  unsigned long lastReadTime;
};


// ============================================================
// FINAL ALERT DECISION
// ============================================================

struct AlertDecision
{
  bool active;

  Direction direction;

  RiskLevel riskLevel;

  float distanceMM;

  int confidence;

  int priority;

  unsigned long timestamp;
};


// ============================================================
// GLOBAL OBJECTS
// ============================================================

VL53L1X tof;

Adafruit_MPU6050 mpu;

HardwareSerial dfSerial(2);

DFRobotDFPlayerMini dfPlayer;


// ============================================================
// SENSOR STATES
// ============================================================

SensorState frontSensor;

SensorState leftSensor;

SensorState rightSensor;


// ============================================================
// FINAL DECISION
// ============================================================

AlertDecision finalDecision;


// ============================================================
// MOVEMENT VARIABLES
// ============================================================

MovementState movementState = MOVEMENT_UNKNOWN;

float previousAccelMagnitude = 0.0;

unsigned long lastMovementCheck = 0;

unsigned long lastMovementTime = 0;


// ============================================================
// HAPTIC STATE
// ============================================================

bool hapticState = false;

Direction hapticDirection = DIR_NONE;

RiskLevel hapticRisk = RISK_SAFE;

unsigned long hapticTimer = 0;


// ============================================================
// AUDIO STATE
// ============================================================

int lastAudioTrack = 0;

unsigned long lastAudioTime = 0;

const unsigned long AUDIO_REPEAT_DELAY = 1500;


// ============================================================
// TIMERS
// ============================================================

unsigned long lastSensorRead = 0;

unsigned long lastDebugPrint = 0;


// ============================================================
// FILTER HISTORY
// ============================================================

const int FILTER_SIZE = 5;

float frontHistory[FILTER_SIZE];

float leftHistory[FILTER_SIZE];

float rightHistory[FILTER_SIZE];

int frontHistoryCount = 0;

int leftHistoryCount = 0;

int rightHistoryCount = 0;


// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void stopAllMotors();

void updateHapticFeedback();

void updateAudio();

void readAllSensors();

void readFrontToF();

void readLeftUltrasonic();

void readRightUltrasonic();

void readIMU();

void updateMovementState();

void processSensorStates();

void makeFinalDecision();

void printDebug();

float medianFilter(float values[], int count);

float applyEMA(float oldValue, float newValue);

RiskLevel classifyRisk(float distanceMM);

int calculateConfidence(SensorState &sensor);

int calculatePriority(Direction direction, float distanceMM, int confidence);

void updateSensorHealth(SensorState &sensor);

void updateSensorPersistence(SensorState &sensor);

void playAudioForDecision();

int getAudioTrack(Direction direction, RiskLevel risk);

const char* directionToString(Direction direction);

const char* riskToString(RiskLevel risk);

const char* movementToString(MovementState state);

void initializeSensorState(SensorState &sensor);


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println("==========================================");
  Serial.println(" ClearSight Prototype");
  Serial.println(" ESP32 Sensor + Haptic + Audio System");
  Serial.println("==========================================");
  Serial.println();


  // ----------------------------------------------------------
  // MOTOR PINS
  // ----------------------------------------------------------

  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);

  stopAllMotors();


  // ----------------------------------------------------------
  // ULTRASONIC PINS
  // ----------------------------------------------------------

  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);

  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);

  digitalWrite(LEFT_TRIG_PIN, LOW);
  digitalWrite(RIGHT_TRIG_PIN, LOW);


  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.setClock(400000);

  Serial.println("I2C started.");
  Serial.println("SDA = GPIO21");
  Serial.println("SCL = GPIO22");


  // ----------------------------------------------------------
  // INITIALIZE SENSOR STATES
  // ----------------------------------------------------------

  initializeSensorState(frontSensor);

  initializeSensorState(leftSensor);

  initializeSensorState(rightSensor);


  // ----------------------------------------------------------
  // VL53L1X
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("Initializing VL53L1X...");

  tof.setTimeout(500);

  if (tof.init())
  {
    Serial.println("VL53L1X: OK");

    tof.setDistanceMode(VL53L1X::Long);

    tof.setMeasurementTimingBudget(50000);

    tof.startContinuous(60);
  }
  else
  {
    Serial.println("VL53L1X: FAILED");
    frontSensor.health = SENSOR_FAILED;
  }


  // ----------------------------------------------------------
  // MPU6050
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("Initializing MPU6050...");

  if (mpu.begin())
  {
    Serial.println("MPU6050: OK");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  else
  {
    Serial.println("MPU6050: FAILED");
  }


  // ----------------------------------------------------------
  // DFPLAYER
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("Initializing DFPlayer...");

  dfSerial.begin(
    9600,
    SERIAL_8N1,
    DF_RX_PIN,
    DF_TX_PIN
  );

  delay(500);

  if (dfPlayer.begin(dfSerial))
  {
    Serial.println("DFPlayer: OK");

    dfPlayer.volume(22);

    dfPlayer.EQ(DFPLAYER_EQ_NORMAL);

    dfPlayer.outputDevice(DFPLAYER_DEVICE_SD);

    delay(500);

    Serial.println("DFPlayer SD selected.");
  }
  else
  {
    Serial.println("DFPlayer: FAILED");
    Serial.println("Check:");
    Serial.println("1. SD card");
    Serial.println("2. TX/RX wiring");
    Serial.println("3. Speaker wiring");
  }


  // ----------------------------------------------------------
  // INITIAL VALUES
  // ----------------------------------------------------------

  finalDecision.active = false;

  finalDecision.direction = DIR_NONE;

  finalDecision.riskLevel = RISK_SAFE;

  finalDecision.distanceMM = 0;

  finalDecision.confidence = 0;

  finalDecision.priority = 0;

  finalDecision.timestamp = millis();


  // ----------------------------------------------------------
  // STARTUP MESSAGE
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("------------------------------------------");
  Serial.println("System ready.");
  Serial.println("------------------------------------------");
  Serial.println();

  delay(1000);
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  unsigned long now = millis();


  // ----------------------------------------------------------
  // SENSOR READING
  // ----------------------------------------------------------

  if (now - lastSensorRead >= SENSOR_INTERVAL_MS)
  {
    lastSensorRead = now;

    readAllSensors();

    processSensorStates();

    makeFinalDecision();
  }


  // ----------------------------------------------------------
  // HAPTIC
  // ----------------------------------------------------------

  updateHapticFeedback();


  // ----------------------------------------------------------
  // AUDIO
  // ----------------------------------------------------------

  updateAudio();


  // ----------------------------------------------------------
  // DEBUG
  // ----------------------------------------------------------

  if (now - lastDebugPrint >= DEBUG_INTERVAL_MS)
  {
    lastDebugPrint = now;

    printDebug();
  }
}


// ============================================================
// INITIALIZE SENSOR STATE
// ============================================================

void initializeSensorState(SensorState &sensor)
{
  sensor.rawDistanceMM = 0;

  sensor.filteredDistanceMM = 0;

  sensor.valid = false;

  sensor.confidence = 0;

  sensor.risk = RISK_SAFE;

  sensor.health = SENSOR_WORKING;

  sensor.validCount = 0;

  sensor.invalidCount = 0;

  sensor.confirmationCount = 0;

  sensor.clearCount = 0;

  sensor.lastValidTime = 0;

  sensor.lastReadTime = 0;
}


// ============================================================
// READ ALL SENSORS
// ============================================================

void readAllSensors()
{
  readFrontToF();

  delay(2);

  readLeftUltrasonic();

  delay(2);

  readRightUltrasonic();

  readIMU();
}


// ============================================================
// FRONT TOF
// ============================================================

void readFrontToF()
{
  if (frontSensor.health == SENSOR_FAILED)
  {
    return;
  }

  uint16_t distance = tof.read();

  frontSensor.lastReadTime = millis();

  if (tof.timeoutOccurred())
  {
    frontSensor.valid = false;

    frontSensor.invalidCount++;

    return;
  }


  // VL53L1X result is millimeters

  if (distance == 0 || distance > 4000)
  {
    frontSensor.valid = false;

    frontSensor.invalidCount++;

    return;
  }


  frontSensor.rawDistanceMM = distance;

  frontSensor.valid = true;

  frontSensor.validCount++;

  frontSensor.invalidCount = 0;

  frontSensor.lastValidTime = millis();


  // ----------------------------------------------------------
  // FILTER
  // ----------------------------------------------------------

  if (frontHistoryCount < FILTER_SIZE)
  {
    frontHistory[frontHistoryCount] = distance;

    frontHistoryCount++;
  }
  else
  {
    for (int i = 0; i < FILTER_SIZE - 1; i++)
    {
      frontHistory[i] = frontHistory[i + 1];
    }

    frontHistory[FILTER_SIZE - 1] = distance;
  }


  float medianValue =
    medianFilter(frontHistory, frontHistoryCount);


  if (frontSensor.filteredDistanceMM == 0)
  {
    frontSensor.filteredDistanceMM = medianValue;
  }
  else
  {
    frontSensor.filteredDistanceMM =
      applyEMA(
        frontSensor.filteredDistanceMM,
        medianValue
      );
  }
}


// ============================================================
// LEFT ULTRASONIC
// ============================================================

void readLeftUltrasonic()
{
  unsigned long duration;

  digitalWrite(LEFT_TRIG_PIN, LOW);

  delayMicroseconds(2);

  digitalWrite(LEFT_TRIG_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(LEFT_TRIG_PIN, LOW);


  duration = pulseIn(
    LEFT_ECHO_PIN,
    HIGH,
    25000
  );


  leftSensor.lastReadTime = millis();


  if (duration == 0)
  {
    leftSensor.valid = false;

    leftSensor.invalidCount++;

    return;
  }


  float distanceCM =
    duration * 0.0343 / 2.0;


  float distanceMM =
    distanceCM * 10.0;


  if (
    distanceMM < 20 ||
    distanceMM > 4000
  )
  {
    leftSensor.valid = false;

    leftSensor.invalidCount++;

    return;
  }


  leftSensor.rawDistanceMM = distanceMM;

  leftSensor.valid = true;

  leftSensor.validCount++;

  leftSensor.invalidCount = 0;

  leftSensor.lastValidTime = millis();


  // ----------------------------------------------------------
  // FILTER
  // ----------------------------------------------------------

  if (leftHistoryCount < FILTER_SIZE)
  {
    leftHistory[leftHistoryCount] =
      distanceMM;

    leftHistoryCount++;
  }
  else
  {
    for (int i = 0; i < FILTER_SIZE - 1; i++)
    {
      leftHistory[i] =
        leftHistory[i + 1];
    }

    leftHistory[FILTER_SIZE - 1] =
      distanceMM;
  }


  float medianValue =
    medianFilter(
      leftHistory,
      leftHistoryCount
    );


  if (leftSensor.filteredDistanceMM == 0)
  {
    leftSensor.filteredDistanceMM =
      medianValue;
  }
  else
  {
    leftSensor.filteredDistanceMM =
      applyEMA(
        leftSensor.filteredDistanceMM,
        medianValue
      );
  }
}


// ============================================================
// RIGHT ULTRASONIC
// ============================================================

void readRightUltrasonic()
{
  unsigned long duration;

  digitalWrite(RIGHT_TRIG_PIN, LOW);

  delayMicroseconds(2);

  digitalWrite(RIGHT_TRIG_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(RIGHT_TRIG_PIN, LOW);


  duration = pulseIn(
    RIGHT_ECHO_PIN,
    HIGH,
    25000
  );


  rightSensor.lastReadTime = millis();


  if (duration == 0)
  {
    rightSensor.valid = false;

    rightSensor.invalidCount++;

    return;
  }


  float distanceCM =
    duration * 0.0343 / 2.0;


  float distanceMM =
    distanceCM * 10.0;


  if (
    distanceMM < 20 ||
    distanceMM > 4000
  )
  {
    rightSensor.valid = false;

    rightSensor.invalidCount++;

    return;
  }


  rightSensor.rawDistanceMM = distanceMM;

  rightSensor.valid = true;

  rightSensor.validCount++;

  rightSensor.invalidCount = 0;

  rightSensor.lastValidTime = millis();


  // ----------------------------------------------------------
  // FILTER
  // ----------------------------------------------------------

  if (rightHistoryCount < FILTER_SIZE)
  {
    rightHistory[rightHistoryCount] =
      distanceMM;

    rightHistoryCount++;
  }
  else
  {
    for (int i = 0; i < FILTER_SIZE - 1; i++)
    {
      rightHistory[i] =
        rightHistory[i + 1];
    }

    rightHistory[FILTER_SIZE - 1] =
      distanceMM;
  }


  float medianValue =
    medianFilter(
      rightHistory,
      rightHistoryCount
    );


  if (rightSensor.filteredDistanceMM == 0)
  {
    rightSensor.filteredDistanceMM =
      medianValue;
  }
  else
  {
    rightSensor.filteredDistanceMM =
      applyEMA(
        rightSensor.filteredDistanceMM,
        medianValue
      );
  }
}


// ============================================================
// IMU
// ============================================================

void readIMU()
{
  unsigned long now = millis();

  if (
    now - lastMovementCheck <
    MOVEMENT_INTERVAL_MS
  )
  {
    return;
  }

  lastMovementCheck = now;


  sensors_event_t accel;

  sensors_event_t gyro;

  sensors_event_t temp;


  mpu.getEvent(
    &accel,
    &gyro,
    &temp
  );


  float ax = accel.acceleration.x;

  float ay = accel.acceleration.y;

  float az = accel.acceleration.z;


  float magnitude =
    sqrt(
      ax * ax +
      ay * ay +
      az * az
    );


  float change =
    fabs(
      magnitude -
      previousAccelMagnitude
    );


  previousAccelMagnitude =
    magnitude;


  // Meaningful acceleration change
  if (change > 0.35)
  {
    lastMovementTime = now;

    movementState = MOVEMENT_MOVING;
  }


  // If no meaningful movement for 5 seconds
  if (
    movementState == MOVEMENT_MOVING &&
    now - lastMovementTime >
    STANDING_TIME_MS
  )
  {
    movementState = MOVEMENT_STANDING;
  }


  if (movementState == MOVEMENT_UNKNOWN)
  {
    movementState = MOVEMENT_MOVING;

    lastMovementTime = now;
  }
}


// ============================================================
// MOVEMENT STATE UPDATE
// ============================================================

void updateMovementState()
{
  // Movement is already updated in readIMU().
  //
  // This function intentionally remains simple.
  // IMU does NOT directly control the motors.
}


// ============================================================
// FILTER SENSOR STATES
// ============================================================

void processSensorStates()
{
  updateMovementState();


  // ----------------------------------------------------------
  // CLASSIFY FRONT
  // ----------------------------------------------------------

  if (frontSensor.valid)
  {
    frontSensor.risk =
      classifyRisk(
        frontSensor.filteredDistanceMM
      );
  }
  else
  {
    frontSensor.risk = RISK_SAFE;
  }


  // ----------------------------------------------------------
  // CLASSIFY LEFT
  // ----------------------------------------------------------

  if (leftSensor.valid)
  {
    leftSensor.risk =
      classifyRisk(
        leftSensor.filteredDistanceMM
      );
  }
  else
  {
    leftSensor.risk = RISK_SAFE;
  }


  // ----------------------------------------------------------
  // CLASSIFY RIGHT
  // ----------------------------------------------------------

  if (rightSensor.valid)
  {
    rightSensor.risk =
      classifyRisk(
        rightSensor.filteredDistanceMM
      );
  }
  else
  {
    rightSensor.risk = RISK_SAFE;
  }


  // ----------------------------------------------------------
  // PERSISTENCE
  // ----------------------------------------------------------

  updateSensorPersistence(frontSensor);

  updateSensorPersistence(leftSensor);

  updateSensorPersistence(rightSensor);


  // ----------------------------------------------------------
  // HEALTH
  // ----------------------------------------------------------

  updateSensorHealth(frontSensor);

  updateSensorHealth(leftSensor);

  updateSensorHealth(rightSensor);


  // ----------------------------------------------------------
  // CONFIDENCE
  // ----------------------------------------------------------

  frontSensor.confidence =
    calculateConfidence(frontSensor);

  leftSensor.confidence =
    calculateConfidence(leftSensor);

  rightSensor.confidence =
    calculateConfidence(rightSensor);
}


// ============================================================
// RISK CLASSIFICATION
// ============================================================

RiskLevel classifyRisk(float distanceMM)
{
  if (distanceMM <= 0)
  {
    return RISK_SAFE;
  }


  if (distanceMM < HIGH_RISK_DISTANCE_MM)
  {
    return RISK_HIGH;
  }


  if (distanceMM < LOW_RISK_DISTANCE_MM)
  {
    return RISK_MEDIUM;
  }


  if (distanceMM <= SAFE_DISTANCE_MM)
  {
    return RISK_LOW;
  }


  return RISK_SAFE;
}


// ============================================================
// MEDIAN FILTER
// ============================================================

float medianFilter(
  float values[],
  int count
)
{
  if (count <= 0)
  {
    return 0;
  }


  float sorted[FILTER_SIZE];


  for (int i = 0; i < count; i++)
  {
    sorted[i] = values[i];
  }


  for (int i = 0; i < count - 1; i++)
  {
    for (int j = i + 1; j < count; j++)
    {
      if (sorted[j] < sorted[i])
      {
        float temp = sorted[i];

        sorted[i] = sorted[j];

        sorted[j] = temp;
      }
    }
  }


  if (count % 2 == 1)
  {
    return sorted[count / 2];
  }


  return (
    sorted[count / 2 - 1] +
    sorted[count / 2]
  ) / 2.0;
}


// ============================================================
// EMA FILTER
// ============================================================

float applyEMA(
  float oldValue,
  float newValue
)
{
  const float alpha = 0.35;

  return (
    alpha * newValue +
    (1.0 - alpha) * oldValue
  );
}


// ============================================================
// SENSOR PERSISTENCE
// ============================================================

void updateSensorPersistence(
  SensorState &sensor
)
{
  if (!sensor.valid)
  {
    sensor.clearCount++;

    if (
      sensor.clearCount >=
      REQUIRED_CLEAR_READINGS
    )
    {
      sensor.confirmationCount = 0;
    }

    return;
  }


  sensor.clearCount = 0;


  if (sensor.risk != RISK_SAFE)
  {
    sensor.confirmationCount++;

    if (
      sensor.confirmationCount >
      REQUIRED_CONFIRMATIONS
    )
    {
      sensor.confirmationCount =
        REQUIRED_CONFIRMATIONS;
    }
  }
  else
  {
    sensor.confirmationCount = 0;
  }
}


// ============================================================
// SENSOR HEALTH
// ============================================================

void updateSensorHealth(
  SensorState &sensor
)
{
  if (
    sensor.invalidCount >= 8
  )
  {
    sensor.health = SENSOR_FAILED;

    return;
  }


  if (
    sensor.invalidCount >= 3
  )
  {
    sensor.health = SENSOR_DEGRADED;

    return;
  }


  if (
    sensor.valid &&
    sensor.invalidCount == 0
  )
  {
    sensor.health = SENSOR_WORKING;
  }
}


// ============================================================
// CONFIDENCE
// ============================================================

int calculateConfidence(
  SensorState &sensor
)
{
  if (!sensor.valid)
  {
    return 0;
  }


  int confidence = 50;


  // ----------------------------------------------------------
  // Persistence
  // ----------------------------------------------------------

  if (
    sensor.confirmationCount >=
    REQUIRED_CONFIRMATIONS
  )
  {
    confidence += 20;
  }
  else
  {
    confidence += 5;
  }


  // ----------------------------------------------------------
  // Sensor health
  // ----------------------------------------------------------

  if (
    sensor.health ==
    SENSOR_WORKING
  )
  {
    confidence += 20;
  }
  else if (
    sensor.health ==
    SENSOR_DEGRADED
  )
  {
    confidence += 5;
  }
  else
  {
    confidence -= 30;
  }


  // ----------------------------------------------------------
  // Consecutive valid readings
  // ----------------------------------------------------------

  if (sensor.validCount >= 5)
  {
    confidence += 10;
  }


  // ----------------------------------------------------------
  // Limit
  // ----------------------------------------------------------

  if (confidence > 100)
  {
    confidence = 100;
  }

  if (confidence < 0)
  {
    confidence = 0;
  }


  return confidence;
}


// ============================================================
// PRIORITY
// ============================================================

int calculatePriority(
  Direction direction,
  float distanceMM,
  int confidence
)
{
  if (distanceMM <= 0)
  {
    return 0;
  }


  int priority = 0;


  // ----------------------------------------------------------
  // Direction base priority
  // ----------------------------------------------------------

  if (direction == DIR_FRONT)
  {
    priority += 20;
  }
  else if (direction == DIR_LEFT)
  {
    priority += 20;
  }
  else if (direction == DIR_RIGHT)
  {
    priority += 20;
  }


  // ----------------------------------------------------------
  // Distance priority
  // ----------------------------------------------------------

  if (distanceMM < 300)
  {
    priority += 60;
  }
  else if (distanceMM < 1000)
  {
    priority += 40;
  }
  else if (distanceMM <= 2000)
  {
    priority += 20;
  }


  // ----------------------------------------------------------
  // Confidence
  // ----------------------------------------------------------

  priority += confidence / 5;


  return priority;
}


// ============================================================
// FINAL DECISION
// ============================================================

void makeFinalDecision()
{
  finalDecision.active = false;

  finalDecision.direction = DIR_NONE;

  finalDecision.riskLevel = RISK_SAFE;

  finalDecision.distanceMM = 0;

  finalDecision.confidence = 0;

  finalDecision.priority = 0;

  finalDecision.timestamp = millis();


  // ----------------------------------------------------------
  // FRONT CANDIDATE
  // ----------------------------------------------------------

  if (
    frontSensor.valid &&
    frontSensor.confirmationCount >=
    REQUIRED_CONFIRMATIONS &&
    frontSensor.confidence >=
    MIN_CONFIDENCE_FOR_ALERT &&
    frontSensor.risk != RISK_SAFE
  )
  {
    finalDecision.active = true;

    finalDecision.direction = DIR_FRONT;

    finalDecision.riskLevel =
      frontSensor.risk;

    finalDecision.distanceMM =
      frontSensor.filteredDistanceMM;

    finalDecision.confidence =
      frontSensor.confidence;

    finalDecision.priority =
      calculatePriority(
        DIR_FRONT,
        frontSensor.filteredDistanceMM,
        frontSensor.confidence
      );
  }


  // ----------------------------------------------------------
  // LEFT CANDIDATE
  // ----------------------------------------------------------

  if (
    leftSensor.valid &&
    leftSensor.confirmationCount >=
    REQUIRED_CONFIRMATIONS &&
    leftSensor.confidence >=
    MIN_CONFIDENCE_FOR_ALERT &&
    leftSensor.risk != RISK_SAFE
  )
  {
    int priority =
      calculatePriority(
        DIR_LEFT,
        leftSensor.filteredDistanceMM,
        leftSensor.confidence
      );


    if (
      !finalDecision.active ||
      priority > finalDecision.priority
    )
    {
      finalDecision.active = true;

      finalDecision.direction = DIR_LEFT;

      finalDecision.riskLevel =
        leftSensor.risk;

      finalDecision.distanceMM =
        leftSensor.filteredDistanceMM;

      finalDecision.confidence =
        leftSensor.confidence;

      finalDecision.priority =
        priority;
    }
  }


  // ----------------------------------------------------------
  // RIGHT CANDIDATE
  // ----------------------------------------------------------

  if (
    rightSensor.valid &&
    rightSensor.confirmationCount >=
    REQUIRED_CONFIRMATIONS &&
    rightSensor.confidence >=
    MIN_CONFIDENCE_FOR_ALERT &&
    rightSensor.risk != RISK_SAFE
  )
  {
    int priority =
      calculatePriority(
        DIR_RIGHT,
        rightSensor.filteredDistanceMM,
        rightSensor.confidence
      );


    if (
      !finalDecision.active ||
      priority > finalDecision.priority
    )
    {
      finalDecision.active = true;

      finalDecision.direction = DIR_RIGHT;

      finalDecision.riskLevel =
        rightSensor.risk;

      finalDecision.distanceMM =
        rightSensor.filteredDistanceMM;

      finalDecision.confidence =
        rightSensor.confidence;

      finalDecision.priority =
        priority;
    }
  }
}


// ============================================================
// STOP MOTORS
// ============================================================

void stopAllMotors()
{
  digitalWrite(LEFT_MOTOR_PIN, LOW);

  digitalWrite(RIGHT_MOTOR_PIN, LOW);
}


// ============================================================
// HAPTIC FEEDBACK
//
// IMPORTANT:
// There is ONLY ONE copy of this function.
// No HapticPattern is used.
// ============================================================

void updateHapticFeedback()
{
  // ----------------------------------------------------------
  // NO ALERT
  // ----------------------------------------------------------

  if (!finalDecision.active)
  {
    stopAllMotors();

    hapticState = false;

    hapticDirection = DIR_NONE;

    hapticRisk = RISK_SAFE;

    return;
  }


  // ----------------------------------------------------------
  // NEW DIRECTION OR NEW RISK
  // ----------------------------------------------------------

  if (
    finalDecision.direction !=
      hapticDirection ||
    finalDecision.riskLevel !=
      hapticRisk
  )
  {
    hapticDirection =
      finalDecision.direction;

    hapticRisk =
      finalDecision.riskLevel;


    // Start immediately

    hapticState = true;

    hapticTimer = millis();


    // LEFT

    if (
      finalDecision.direction ==
      DIR_LEFT
    )
    {
      digitalWrite(
        LEFT_MOTOR_PIN,
        HIGH
      );

      digitalWrite(
        RIGHT_MOTOR_PIN,
        LOW
      );
    }


    // RIGHT

    else if (
      finalDecision.direction ==
      DIR_RIGHT
    )
    {
      digitalWrite(
        LEFT_MOTOR_PIN,
        LOW
      );

      digitalWrite(
        RIGHT_MOTOR_PIN,
        HIGH
      );
    }


    // FRONT

    else if (
      finalDecision.direction ==
      DIR_FRONT
    )
    {
      digitalWrite(
        LEFT_MOTOR_PIN,
        HIGH
      );

      digitalWrite(
        RIGHT_MOTOR_PIN,
        HIGH
      );
    }


    return;
  }


  // ----------------------------------------------------------
  // HAPTIC TIMING
  // ----------------------------------------------------------

  unsigned long onTime = 0;

  unsigned long offTime = 0;


  switch (
    finalDecision.riskLevel
  )
  {
    case RISK_LOW:

      // Slow vibration
      onTime = 150;
      offTime = 850;

      break;


    case RISK_MEDIUM:

      // Medium vibration
      onTime = 200;
      offTime = 300;

      break;


    case RISK_HIGH:

      // Rapid vibration
      onTime = 250;
      offTime = 100;

      break;


    default:

      stopAllMotors();

      hapticState = false;

      return;
  }


  // ----------------------------------------------------------
  // MOTOR ON
  // ----------------------------------------------------------

  if (
    hapticState &&
    millis() - hapticTimer >=
    onTime
  )
  {
    hapticState = false;

    hapticTimer = millis();

    stopAllMotors();
  }


  // ----------------------------------------------------------
  // MOTOR OFF
  // ----------------------------------------------------------

  else if (
    !hapticState &&
    millis() - hapticTimer >=
    offTime
  )
  {
    hapticState = true;

    hapticTimer = millis();


    if (
      finalDecision.direction ==
      DIR_LEFT
    )
    {
      digitalWrite(
        LEFT_MOTOR_PIN,
        HIGH
      );

      digitalWrite(
        RIGHT_MOTOR_PIN,
        LOW
      );
    }


    else if (
      finalDecision.direction ==
      DIR_RIGHT
    )
    {
      digitalWrite(
        LEFT_MOTOR_PIN,
        LOW
      );

      digitalWrite(
        RIGHT_MOTOR_PIN,
        HIGH
      );
    }


    else if (
      finalDecision.direction ==
      DIR_FRONT
    )
    {
      digitalWrite(
        LEFT_MOTOR_PIN,
        HIGH
      );

      digitalWrite(
        RIGHT_MOTOR_PIN,
        HIGH
      );
    }


    else
    {
      stopAllMotors();

      hapticState = false;
    }
  }
}


// ============================================================
// AUDIO UPDATE
// ============================================================

void updateAudio()
{
  if (!finalDecision.active)
  {
    lastAudioTrack = 0;

    return;
  }


  int track =
    getAudioTrack(
      finalDecision.direction,
      finalDecision.riskLevel
    );


  if (track <= 0)
  {
    return;
  }


  unsigned long now = millis();


  // Only play when:
  // 1. track changed
  // OR
  // 2. enough time passed

  if (
    track != lastAudioTrack ||
    now - lastAudioTime >
    AUDIO_REPEAT_DELAY
  )
  {
    playAudioForDecision();

    lastAudioTrack = track;

    lastAudioTime = now;
  }
}


// ============================================================
// PLAY AUDIO
// ============================================================

void playAudioForDecision()
{
  int track =
    getAudioTrack(
      finalDecision.direction,
      finalDecision.riskLevel
    );


  if (track <= 0)
  {
    return;
  }


  // Folder MP3:
  // /MP3/0001.mp3 etc.

  dfPlayer.playMp3Folder(track);
}


// ============================================================
// AUDIO TRACK MAPPING
// ============================================================

int getAudioTrack(
  Direction direction,
  RiskLevel risk
)
{
  if (direction == DIR_FRONT)
  {
    if (risk == RISK_LOW)
    {
      return 1;
    }

    if (risk == RISK_MEDIUM)
    {
      return 2;
    }

    if (risk == RISK_HIGH)
    {
      return 3;
    }
  }


  if (direction == DIR_LEFT)
  {
    if (risk == RISK_LOW)
    {
      return 4;
    }

    if (risk == RISK_MEDIUM)
    {
      return 5;
    }

    if (risk == RISK_HIGH)
    {
      return 6;
    }
  }


  if (direction == DIR_RIGHT)
  {
    if (risk == RISK_LOW)
    {
      return 7;
    }

    if (risk == RISK_MEDIUM)
    {
      return 8;
    }

    if (risk == RISK_HIGH)
    {
      return 9;
    }
  }


  return 0;
}


// ============================================================
// DEBUG OUTPUT
// ============================================================

void printDebug()
{
  Serial.println();
  Serial.println("==========================================");
  Serial.println("CLEARSIGHT STATUS");
  Serial.println("==========================================");


  // ----------------------------------------------------------
  // FRONT
  // ----------------------------------------------------------

  Serial.print("FRONT ToF: ");

  if (frontSensor.valid)
  {
    Serial.print(
      frontSensor.filteredDistanceMM
    );

    Serial.print(" mm");

    Serial.print(" | ");

    Serial.print(
      riskToString(
        frontSensor.risk
      )
    );

    Serial.print(" | Conf ");

    Serial.print(
      frontSensor.confidence
    );

    Serial.print("%");
  }
  else
  {
    Serial.print("INVALID");
  }


  Serial.println();


  // ----------------------------------------------------------
  // LEFT
  // ----------------------------------------------------------

  Serial.print("LEFT HC-SR04: ");

  if (leftSensor.valid)
  {
    Serial.print(
      leftSensor.filteredDistanceMM
    );

    Serial.print(" mm");

    Serial.print(" | ");

    Serial.print(
      riskToString(
        leftSensor.risk
      )
    );

    Serial.print(" | Conf ");

    Serial.print(
      leftSensor.confidence
    );

    Serial.print("%");
  }
  else
  {
    Serial.print("INVALID");
  }


  Serial.println();


  // ----------------------------------------------------------
  // RIGHT
  // ----------------------------------------------------------

  Serial.print("RIGHT HC-SR04: ");

  if (rightSensor.valid)
  {
    Serial.print(
      rightSensor.filteredDistanceMM
    );

    Serial.print(" mm");

    Serial.print(" | ");

    Serial.print(
      riskToString(
        rightSensor.risk
      )
    );

    Serial.print(" | Conf ");

    Serial.print(
      rightSensor.confidence
    );

    Serial.print("%");
  }
  else
  {
    Serial.print("INVALID");
  }


  Serial.println();


  // ----------------------------------------------------------
  // MOVEMENT
  // ----------------------------------------------------------

  Serial.print("MOVEMENT: ");

  Serial.println(
    movementToString(
      movementState
    )
  );


  // ----------------------------------------------------------
  // FINAL DECISION
  // ----------------------------------------------------------

  Serial.println("------------------------------------------");

  Serial.print("FINAL ALERT: ");

  if (finalDecision.active)
  {
    Serial.println("ACTIVE");

    Serial.print("Direction: ");

    Serial.println(
      directionToString(
        finalDecision.direction
      )
    );


    Serial.print("Risk: ");

    Serial.println(
      riskToString(
        finalDecision.riskLevel
      )
    );


    Serial.print("Distance: ");

    Serial.print(
      finalDecision.distanceMM
    );

    Serial.println(" mm");


    Serial.print("Confidence: ");

    Serial.print(
      finalDecision.confidence
    );

    Serial.println("%");


    Serial.print("Priority: ");

    Serial.println(
      finalDecision.priority
    );
  }
  else
  {
    Serial.println("NO ALERT");
  }


  Serial.println("==========================================");
}


// ============================================================
// DIRECTION TO STRING
// ============================================================

const char* directionToString(
  Direction direction
)
{
  switch (direction)
  {
    case DIR_FRONT:
      return "FRONT";

    case DIR_LEFT:
      return "LEFT";

    case DIR_RIGHT:
      return "RIGHT";

    default:
      return "NONE";
  }
}


// ============================================================
// RISK TO STRING
// ============================================================

const char* riskToString(
  RiskLevel risk
)
{
  switch (risk)
  {
    case RISK_LOW:
      return "LOW";

    case RISK_MEDIUM:
      return "MEDIUM";

    case RISK_HIGH:
      return "HIGH";

    default:
      return "SAFE";
  }
}


// ============================================================
// MOVEMENT TO STRING
// ============================================================

const char* movementToString(
  MovementState state
)
{
  switch (state)
  {
    case MOVEMENT_MOVING:
      return "MOVING";

    case MOVEMENT_STANDING:
      return "STANDING";

    default:
      return "UNKNOWN";
  }
}