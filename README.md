# CLEAR SIGHT

### Offline ESP32-Based Assistive Smart Glasses Prototype

CLEAR SIGHT is an **offline assistive smart-glasses prototype** designed to help visually impaired users become more aware of nearby obstacles through **directional vibration feedback and prerecorded audio alerts**.

The current prototype uses an **ESP32** as the central controller and combines distance sensors with an IMU to detect obstacles, determine their direction and approximate risk level, and provide corresponding feedback.

> **Project Status:** Prototype / Proof of Concept
> This repository contains the firmware developed for the current prototype. It is **not the final production version** of the complete CLEAR SIGHT system.

---

# 1. Prototype Overview

The current prototype focuses primarily on **obstacle detection and directional feedback**.

The prototype uses:

* **ESP32-WROOM / ESP32 DevKit** — central controller
* **VL53L1X ToF sensor** — front obstacle detection
* **HC-SR04 ultrasonic sensor** — left obstacle detection
* **HC-SR04 ultrasonic sensor** — right obstacle detection
* **MPU6050 IMU** — movement/standing detection
* **Two vibration motors** — directional haptic feedback
* **DFPlayer Mini + speaker** — prerecorded audio feedback
* **MicroSD card** — stores predefined audio messages

The system operates locally without requiring:

* Internet
* Cloud processing
* Smartphone connection
* GPS
* External AI services

---

# 2. Prototype Hardware

| Component                    | Purpose                  | ESP32 Connection  |
| ---------------------------- | ------------------------ | ----------------- |
| VL53L1X ToF                  | Front obstacle detection | SDA 21, SCL 22    |
| MPU6050                      | Movement detection       | SDA 21, SCL 22    |
| Left HC-SR04                 | Left obstacle detection  | TRIG 16, ECHO 34  |
| Right HC-SR04                | Right obstacle detection | TRIG 17, ECHO 35  |
| Left vibration motor driver  | Left haptic feedback     | GPIO 25           |
| Right vibration motor driver | Right haptic feedback    | GPIO 26           |
| DFPlayer TX                  | Serial communication     | GPIO 18           |
| DFPlayer RX                  | Serial communication     | GPIO 19           |
| DFPlayer speaker             | Audio output             | DFPlayer SPK pins |

### DFPlayer UART

The prototype uses ESP32 HardwareSerial:

```text
DFPlayer TX → ESP32 GPIO18
DFPlayer RX → ESP32 GPIO19
DFPlayer GND → ESP32 GND
```

The ESP32 communicates with the DFPlayer through a hardware UART interface.

---

# 3. Prototype Sensor Arrangement

The current prototype uses three directions for obstacle detection:

```text
                 FRONT
             VL53L1X ToF
                  ↓
                  ↓
          ┌───────────────┐
          │     USER      │
          │               │
          └───────────────┘
          ↙               ↘
       LEFT               RIGHT
    HC-SR04              HC-SR04
```

The front ToF sensor is used for detecting **objects in front of the user**.

The left and right ultrasonic sensors provide additional directional coverage.

The current prototype does **not** implement dedicated ground, pothole, hole, or drop-off detection.

---

# 4. Distance Risk Zones

The prototype divides detected obstacles into three warning levels.

| Distance   | Risk Level | Feedback                                 |
| ---------- | ---------- | ---------------------------------------- |
| > 200 cm   | SAFE       | No warning                               |
| 100–200 cm | LOW        | Slow vibration + low-level audio         |
| 30–100 cm  | MEDIUM     | Faster vibration + closer-obstacle audio |
| < 30 cm    | HIGH       | Rapid vibration + very-close audio       |

These values are configurable in the firmware.

---

# 5. Directional Haptic Feedback

The vibration feedback is directional.

### Left obstacle

```text
LEFT MOTOR  → ON
RIGHT MOTOR → OFF
```

### Right obstacle

```text
LEFT MOTOR  → OFF
RIGHT MOTOR → ON
```

### Front obstacle

```text
LEFT MOTOR  → ON
RIGHT MOTOR → ON
```

The vibration pattern becomes faster as the detected obstacle becomes closer.

The firmware uses a **non-blocking `millis()`-based haptic controller** instead of long `delay()` calls, allowing the sensors and decision system to continue operating while vibration feedback is being generated.

---

# 6. Prerecorded Audio Feedback

The prototype uses a DFPlayer Mini with prerecorded audio files.

The current mapping is:

```text
0001.mp3 → Front obstacle detected
0002.mp3 → Front obstacle close
0003.mp3 → Front obstacle very close

0004.mp3 → Left obstacle detected
0005.mp3 → Left obstacle close
0006.mp3 → Left obstacle very close

0007.mp3 → Right obstacle detected
0008.mp3 → Right obstacle close
0009.mp3 → Right obstacle very close
```

The audio is **event/state based** rather than continuously repeated.

For example:

```text
150 cm → Front LOW
        → Play front LOW message once

80 cm  → Front MEDIUM
        → Play front CLOSE message

25 cm  → Front HIGH
        → Play front VERY CLOSE message
```

If the obstacle remains at the same risk level, the same message is not continuously replayed.

---

# 7. Sensor Processing

The firmware does not directly send raw sensor readings to the motors or audio system.

The processing pipeline is:

```text
Sensor Reading
      ↓
Validity Check
      ↓
Outlier Rejection
      ↓
Median Filtering
      ↓
EMA Smoothing
      ↓
Persistence Check
      ↓
Risk Classification
      ↓
Confidence Calculation
      ↓
Priority Calculation
      ↓
Final Alert Decision
      ↓
 ┌──────────────┐
 │              │
HAPTIC        AUDIO
```

This architecture reduces the effect of isolated noisy measurements.

---

# 8. Persistence and Confidence

The prototype does not treat a single sensor reading as a confirmed obstacle.

Multiple valid readings are used to establish persistence.

The firmware also calculates a confidence value based on factors such as:

* Measurement validity
* Measurement stability
* Persistence
* Sensor health
* Recent valid measurements

This helps prevent a single unrealistic reading from immediately triggering a strong warning.

---

# 9. Sensor Health

Each obstacle sensor maintains an internal health state:

```text
WORKING
DEGRADED
FAILED
```

Temporary invalid readings do not immediately cause the sensor to be considered failed.

This allows the remaining sensors to continue operating if one sensor becomes unavailable.

---

# 10. Priority-Based Decision Making

Multiple obstacles can be detected simultaneously.

Instead of allowing every sensor to independently control the motors, the firmware generates a single final `AlertDecision`.

For example:

```text
FRONT = 80 cm
LEFT  = 150 cm
RIGHT = 25 cm
```

The system evaluates:

* Distance
* Risk
* Confidence
* Persistence

and selects the most urgent confirmed obstacle for the current feedback cycle.

The final decision contains:

```cpp
struct AlertDecision {
    bool active;
    Direction direction;
    RiskLevel riskLevel;
    float distanceMM;
    int confidence;
    int priority;
    unsigned long timestamp;
};
```

Both the haptic and audio controllers receive this final decision.

---

# 11. MPU6050 Movement Detection

The MPU6050 is currently used for **movement-state detection**, not as an obstacle sensor.

The firmware distinguishes between:

```text
MOVING
STANDING
```

If the user remains stationary for approximately five seconds, the system can reduce side ultrasonic activity while continuing to monitor the front ToF sensor and MPU6050.

When meaningful movement is detected again, side obstacle sensing resumes.

The IMU therefore enhances system awareness without becoming the primary obstacle-detection mechanism.

---

# 12. Current Prototype Limitations

The current prototype is intentionally limited to demonstrate the core concept.

It does **not** currently provide:

* Rear obstacle detection
* Dedicated pothole detection
* Dedicated hole/drop-off detection
* Full ground/terrain analysis
* Real-time text-to-speech
* Bone-conduction audio
* GPS navigation
* OCR
* Currency recognition
* Scene description
* Cloud AI
* Full 360° environmental awareness

These are potential features for future versions of the system.

---

# 13. Future Full Product

The final CLEAR SIGHT product would require additional sensing and a more advanced perception architecture.

A possible future sensor arrangement would include:

```text
                       FRONT
                 ┌─────────────┐
                 │ Front ToF   │
                 │ horizontal  │
                 └─────────────┘
                        ↓

               Ground / Pothole
                 ┌─────────────┐
                 │ Downward    │
                 │ angled ToF  │
                 └─────────────┘

        LEFT                         RIGHT
      Sensor                         Sensor
         ↓                             ↓

                    USER
                     ↑

                 REAR SENSORS
                     ↑
```

---

# 14. Additional Rear Sensor Coverage

The final product could include additional sensors facing backward.

For example:

```text
Front → ToF
Left  → Ultrasonic / ToF
Right → Ultrasonic / ToF
Rear  → Additional HC-SR04 / ToF
```

This would allow the system to detect obstacles approaching from behind.

The exact number and placement of sensors would depend on the desired field of view, physical glasses design, power consumption, sensor interference, and available processing resources.

---

# 15. HC-SR04 vs ToF for Future Versions

The current prototype uses HC-SR04 sensors for left and right detection because they are inexpensive and useful for demonstrating directional obstacle detection.

In a future version, some or all ultrasonic sensors could potentially be replaced with additional ToF sensors.

### HC-SR04 advantages

* Low cost
* Simple distance measurement
* Useful for prototype development

### ToF advantages

* Compact
* Digital interface
* Suitable for small wearable designs
* Can provide more integrated sensing options

A future design could therefore use multiple ToF sensors instead of relying primarily on ultrasonic sensors.

The final sensor selection should be based on:

* Range
* Field of view
* Size
* Power consumption
* Environmental performance
* Crosstalk/interference
* Cost
* Required detection accuracy

---

# 16. Future Ground and Pothole Detection

The current front ToF sensor is **not used for ground detection**.

For the final product, an additional ToF sensor could be mounted at a downward angle to observe the area of ground ahead of the user.

For example:

```text
          User
           │
           │
      ┌────┴────┐
      │ Glasses │
      └────┬────┘
           │
           │  Downward ToF
           │       ↘
           │         ↘
           │           ↓
───────────┴────────────── Ground
                       ↓
                  Detection area
```

The downward-facing sensor could measure the expected ground distance continuously.

The firmware could then compare:

```text
Expected ground distance
        vs
Current ground distance
```

A significant change could indicate a possible:

* Pothole
* Step
* Drop-off
* Raised surface
* Uneven terrain

However, this would require a substantially different algorithm from the current obstacle-detection code.

---

# 17. Required Code Changes for Ground Detection

The future firmware should treat **ground sensing as a separate detection subsystem**.

Instead of classifying the downward ToF simply as:

```text
SAFE / LOW / MEDIUM / HIGH
```

the system could maintain a ground model such as:

```text
NORMAL GROUND
POSSIBLE RAISED OBJECT
POSSIBLE POTHOLE
POSSIBLE DROP-OFF
UNCERTAIN
```

A possible processing pipeline would be:

```text
Downward ToF
      ↓
Distance Filtering
      ↓
Ground Baseline Estimation
      ↓
Short-Term Trend Analysis
      ↓
Change Detection
      ↓
Persistence Check
      ↓
Ground Hazard Classification
      ↓
Confidence
      ↓
Priority
      ↓
Final Alert Decision
```

The algorithm should not trigger a pothole warning from a single unusual measurement.

Multiple measurements should be used to determine whether the change is persistent and significant.

---

# 18. Ground Detection and User Movement

Ground detection would also need to consider the user's movement.

Head movement and body movement can naturally change the angle of a downward-facing sensor.

Therefore, the final system could combine:

```text
Downward ToF
      +
MPU6050 orientation/movement data
```

to distinguish between:

```text
Actual ground-level change
```

and:

```text
Sensor/head movement
```

This would make the ground-detection system more reliable.

---

# 19. Future Alert Architecture

The final system would have more detection categories:

```text
                 SENSOR LAYER
                      │
       ┌──────────────┼──────────────┐
       ↓              ↓              ↓
   FRONT/REAR      LEFT/RIGHT     GROUND
   OBSTACLES       OBSTACLES      HAZARDS
       │              │              │
       └──────────────┼──────────────┘
                      ↓
               SENSOR FUSION
                      ↓
                 VALIDATION
                      ↓
                  FILTERING
                      ↓
                 PERSISTENCE
                      ↓
                  CONFIDENCE
                      ↓
                   PRIORITY
                      ↓
             FINAL ALERT DECISION
                      ↓
              ┌───────┴────────┐
              ↓                ↓
           HAPTICS        AUDIO/TTS
```

This would allow the final product to determine not only that an obstacle exists, but also **what type of hazard has been detected and where it is located**.

---

# 20. Future Audio System

The current prototype uses:

```text
DFPlayer Mini
+
microSD card
+
prerecorded MP3 files
+
speaker
```

This approach is useful for prototype testing because the audio responses are simple and predictable.

The full product would replace this with:

```text
ESP32 / Future Processing Platform
            ↓
      Alert Decision
            ↓
     Text-to-Speech
            ↓
   Bone-Conduction Speaker
```

Instead of having a fixed message such as:

> "Obstacle ahead close"

the final product could generate more dynamic alerts based on the current sensor information.

For example:

```text
"Obstacle ahead, approximately one meter."

"Obstacle on your left."

"Very close obstacle on your right."

"Possible drop-off ahead."

"Possible pothole ahead."
```

This would allow the system to provide more detailed and real-time information than a fixed collection of MP3 files.

---

# 21. Bone-Conduction Audio

The final product is intended to use **bone-conduction speakers** instead of the prototype's external/local speaker.

This would allow audio information to be presented while keeping the user's ears more open to surrounding environmental sounds.

The final audio subsystem would therefore need to replace the DFPlayer-specific logic with a real-time TTS interface.

The current DFPlayer controller is intentionally kept separate from the main decision architecture so that it can eventually be replaced by a TTS controller.

---

# 22. Prototype vs Full Product

| Feature                 | Current Prototype  | Future Full Product                   |
| ----------------------- | ------------------ | ------------------------------------- |
| ESP32 control           | Yes                | Yes / upgraded processing if required |
| Front ToF               | Yes                | Yes                                   |
| Left obstacle sensing   | HC-SR04            | HC-SR04 or ToF                        |
| Right obstacle sensing  | HC-SR04            | HC-SR04 or ToF                        |
| Rear sensing            | No                 | Additional sensors                    |
| Ground sensing          | No                 | Downward-facing ToF                   |
| Pothole detection       | No                 | Planned                               |
| Drop-off detection      | No                 | Planned                               |
| MPU6050                 | Movement detection | Movement + orientation assistance     |
| Vibration feedback      | Yes                | Yes                                   |
| DFPlayer                | Yes                | No                                    |
| Prerecorded MP3         | Yes                | No                                    |
| TTS                     | No                 | Planned                               |
| Bone-conduction speaker | No                 | Planned                               |
| GPS navigation          | No                 | Future feature                        |
| OCR                     | No                 | Future feature                        |
| Currency recognition    | No                 | Future feature                        |
| Scene understanding     | No                 | Future feature                        |
| AI assistant            | No                 | Future feature                        |
| Offline operation       | Yes                | Targeted where practical              |

---

# 23. Firmware Evolution

The current prototype firmware is structured so that additional sensing can be incorporated without completely redesigning the decision system.

The current architecture is:

```text
SENSORS
   ↓
VALIDATION
   ↓
FILTERING
   ↓
PERSISTENCE
   ↓
CONFIDENCE
   ↓
PRIORITY
   ↓
FINAL DECISION
   ↓
OUTPUT
```

The future architecture can extend this to:

```text
FRONT OBSTACLE SENSOR
        │
LEFT OBSTACLE SENSOR
        │
RIGHT OBSTACLE SENSOR
        │
REAR OBSTACLE SENSOR
        │
GROUND ToF
        │
MPU6050
        ↓
   SENSOR FUSION
        ↓
VALIDATION + FILTERING
        ↓
PERSISTENCE
        ↓
HAZARD CLASSIFICATION
        ↓
CONFIDENCE
        ↓
PRIORITY
        ↓
FINAL ALERT
        ↓
 ┌──────┴───────┐
 ↓              ↓
HAPTIC        TTS AUDIO
               ↓
       BONE-CONDUCTION
          SPEAKER
```

---

# 24. Important Prototype Disclaimer

This repository represents a **prototype research/development implementation**.

The current system is intended to demonstrate the feasibility of:

* Multi-sensor obstacle detection
* Directional obstacle identification
* Distance-based risk classification
* Sensor filtering
* Confidence-based decision making
* Priority-based alert selection
* Directional vibration feedback
* Audio warning generation

It should **not be considered a finished assistive or safety-critical medical/wearable product**.

The prototype has limited sensor coverage and does not guarantee detection of every environmental hazard.

A production system would require extensive hardware validation, sensor characterization, environmental testing, fail-safe design, power management, enclosure/wearability testing, usability testing, and appropriate safety/regulatory evaluation.

---

# 25. Future Development Roadmap

### Phase 1 — Current Prototype

* [x] ESP32 controller
* [x] Front ToF
* [x] Left ultrasonic sensor
* [x] Right ultrasonic sensor
* [x] MPU6050
* [x] Directional vibration
* [x] DFPlayer audio
* [x] Distance risk levels
* [x] Filtering
* [x] Persistence
* [x] Confidence
* [x] Priority decision

### Phase 2 — Expanded Environmental Awareness

* [ ] Rear obstacle sensors
* [ ] Additional ToF sensors
* [ ] Improved sensor fusion
* [ ] Improved movement/orientation estimation
* [ ] Wider environmental coverage

### Phase 3 — Ground Hazard Detection

* [ ] Downward-facing ToF
* [ ] Ground baseline estimation
* [ ] Pothole detection
* [ ] Step/raised-surface detection
* [ ] Drop-off detection
* [ ] Ground-hazard confidence system

### Phase 4 — Advanced Audio

* [ ] Replace DFPlayer with TTS
* [ ] Dynamic real-time alert generation
* [ ] Bone-conduction speaker
* [ ] Context-aware voice messages
* [ ] Audio prioritization

### Phase 5 — Full Assistive Platform

Potential future additions include:

* [ ] GPS navigation
* [ ] OCR
* [ ] Currency recognition
* [ ] Scene understanding
* [ ] AI-assisted interaction
* [ ] Expanded environmental awareness

---

# 26. Conclusion

CLEAR SIGHT currently demonstrates the core concept of an **offline, sensor-based assistive smart-glasses system**.

The prototype focuses on reliable obstacle detection from the front, left, and right, followed by filtering, persistence, confidence evaluation, priority selection, and directional feedback through vibration and prerecorded audio.

The prototype is intentionally smaller than the planned final product.

The future system can expand the sensing field with additional rear sensors and a downward-facing ToF sensor for ground hazards, while replacing prerecorded DFPlayer audio with real-time text-to-speech through bone-conduction speakers.

The firmware architecture is designed to provide a foundation for these future extensions without changing the fundamental principle:

```text
SENSE
  ↓
UNDERSTAND
  ↓
PRIORITIZE
  ↓
ALERT
```

**CLEAR SIGHT — Prototype today, expandable assistive platform for the future.**
