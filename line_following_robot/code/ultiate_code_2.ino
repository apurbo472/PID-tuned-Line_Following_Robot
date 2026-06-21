#include <QTRSensors.h>

// Motor driver pins
#define STBY_PIN      9
#define PWMA          6
#define AI1           4
#define AI2           5
#define PWMB         10
#define BI1           7
#define BI2           8

// Start switch pin
#define START_SWITCH  3

// PID constants - SNIPIER TURNS
const float Kp = 0.16;        // bumped from 0.13
const float Ki = 0.00005;     // same
const float Kd = 1.1;         // same

// Derivative filtering
const float DERIVATIVE_FILTER = 0.35;

// Motor control - BIGGER TURN PIVOT
const int BASE_SPEED    = 70;
const int MAX_SPEED     = 150;
const int TURN_BOOST    = 80;

// Sensor setup
#define NUM_SENSORS 8
const uint8_t sensorPins[NUM_SENSORS] = { A7, A6, A5, A4, A3, A2, A1, A0 };
QTRSensorsAnalog qtra(sensorPins, NUM_SENSORS);
unsigned int sensorValues[NUM_SENSORS];
unsigned int position = 0;

// Recovery & timing
const unsigned long RECOVERY_DELAY = 450;   // ms

// PID variables
int lastError = 0;
int integral  = 0;
unsigned long lastFoundTime = 0;

// Line tracking vars
int lastLineSide = 0;  // -1 = left, 1 = right, 0 = center/unknown

// State control
bool robotRunning   = false;
unsigned long startTime      = 0;
bool startTriggered = false;

// Dual-mode tracking
bool whiteLineMode = false;
const int BLACK_SURFACE_THRESHOLD   = 800;
const int NUM_CONSECUTIVE_READINGS  = 5;
int surfaceReadingCount = 0;

// --- memory of last-good sensor snapshot ---
unsigned int lastSeenSensorValues[NUM_SENSORS] = {0}; // copy of last sensors when on line
unsigned long lastSeenTime = 0;                      // when lastSeenSensorValues was updated

// Special-case thresholds (tune these to your sensors/surface)
const int OUTER_BLACK_THRESHOLD = 550;   // for black-line mode: outer sensors value > this => strong black
const int OUTER_WHITE_THRESHOLD = 500;   // for black-line mode: outer sensors value < this => white/background
const int WHITE_MODE_LINE_TH    = 300;   // for white-line mode: value < this => line
const int WHITE_MODE_BG_TH      = 400;   // for white-line mode: value > this => background
const unsigned long SPECIAL_PIVOT_TIME = 120; // ms to pivot right in the special case

// Function prototypes
void setMotor(int leftSpeed, int rightSpeed);
void blinkStandbyLED();
void performTurnRecovery();
unsigned int calculateLinePosition(bool whiteLine);
unsigned int calculateLinePositionFrom(const unsigned int vals[], bool whiteLine);

void setup() {
  // Motor pins
  pinMode(AI1, OUTPUT); pinMode(AI2, OUTPUT);
  pinMode(BI1, OUTPUT); pinMode(BI2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);

  // Start switch + LED
  pinMode(START_SWITCH, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  // Enhanced calibration with movement
  digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i < 120; i++) {
    qtra.calibrate();
    if (i % 20 == 0) {
      setMotor(50, -50);
      delay(5);
      setMotor(-50, 50);
      delay(5);
    }
    delay(10);
  }
  setMotor(0, 0);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  bool switchActive = (digitalRead(START_SWITCH) == LOW);
  if (!switchActive) {
    setMotor(0, 0);
    robotRunning   = false;
    startTriggered = false;
    lastError      = 0;
    integral       = 0;
    blinkStandbyLED();
    return;
  }

  // Start sequence
  if (!startTriggered) {
    startTriggered = true;
    startTime      = millis();
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if (!robotRunning) {
    if (millis() - startTime < 2000) return;
    robotRunning = true;
    digitalWrite(LED_BUILTIN, LOW);
  }

  // —— LINE FOLLOWING ——
  qtra.readCalibrated(sensorValues);





// If ALL sensors see black -> go straight and skip special pivots
bool allBlack = true;
for (int i = 0; i < NUM_SENSORS; i++) {
  if (sensorValues[i] <= OUTER_BLACK_THRESHOLD) { allBlack = false; break; }
}
if (allBlack && !whiteLineMode) {
  setMotor(BASE_SPEED, BASE_SPEED);        // drive straight at base speed
  for (int i = 0; i < NUM_SENSORS; i++)    // update last-seen snapshot
    lastSeenSensorValues[i] = sensorValues[i];
  lastFoundTime = millis();
  lastError = 0;
  return; // important — skip the pivot/recovery logic this loop
}









  // -----------------------------
  // NEW: Special-case logic requested:
  // If most-right-2 sensors see line and most-left-2 see background,
  // stop briefly then turn RIGHT (in-place) to re-acquire.
  // Works for both black-line and white-line modes (uses appropriate thresholds).
  // -----------------------------
if (!whiteLineMode) {


  // if ((   sensorValues[0] > OUTER_BLACK_THRESHOLD && sensorValues[1] > OUTER_BLACK_THRESHOLD)
  //     && (sensorValues[2] > OUTER_BLACK_THRESHOLD && sensorValues[3] > OUTER_BLACK_THRESHOLD)
  //     && (sensorValues[4] > OUTER_BLACK_THRESHOLD && sensorValues[5] > OUTER_BLACK_THRESHOLD)
  //     && (sensorValues[6] > OUTER_BLACK_THRESHOLD && sensorValues[7] > OUTER_BLACK_THRESHOLD)) {

  //   setMotor(20,20); delay(40);}


  // Right side strong, left side clear -> pivot RIGHT
  if ((sensorValues[3] > OUTER_BLACK_THRESHOLD)
      && (sensorValues[4] > OUTER_BLACK_THRESHOLD) && //sensorValues[5] > OUTER_BLACK_THRESHOLD)
       (sensorValues[6] > OUTER_BLACK_THRESHOLD && sensorValues[7] > OUTER_BLACK_THRESHOLD )) {
    

    setMotor(BASE_SPEED,0); delay(50);



    // Pivot right until center sensors detect line
    unsigned long pivotStart = millis();
    while (true) {
      setMotor(MAX_SPEED, -MAX_SPEED);
      qtra.readCalibrated(sensorValues);
     if (sensorValues[0] > OUTER_BLACK_THRESHOLD || sensorValues[1] > OUTER_BLACK_THRESHOLD) break;

      // Add timeout to avoid lock (2 seconds max)
      if (millis() - pivotStart > 2000) break;
    }
    setMotor(0, 0);
    lastError = 0;
    return;
  }
  // Hard LEFT pivot detection:
  if ((sensorValues[0] > OUTER_BLACK_THRESHOLD && sensorValues[1] > OUTER_BLACK_THRESHOLD)
      //&& (sensorValues[2] > OUTER_BLACK_THRESHOLD 
    &&( sensorValues[3] > OUTER_BLACK_THRESHOLD)
      && (sensorValues[4] > OUTER_BLACK_THRESHOLD )
     ) {
    

    setMotor(0,BASE_SPEED); delay(50);


    // Pivot left until center sensors detect line
    unsigned long pivotStart = millis();
    while (true) {
      setMotor(-MAX_SPEED, MAX_SPEED);
      qtra.readCalibrated(sensorValues);
    if (sensorValues[6] > OUTER_BLACK_THRESHOLD || sensorValues[7] > OUTER_BLACK_THRESHOLD) break;

      if (millis() - pivotStart > 2000) break;
    }
    setMotor(0, 0);
    lastError = 0;
    return;
  }
} else {
  // White line mode - similar sharp turn detection with thresholds reversed
  if ((sensorValues[6] < OUTER_WHITE_THRESHOLD && sensorValues[7] < OUTER_WHITE_THRESHOLD) &&
      (sensorValues[0] > OUTER_BLACK_THRESHOLD && sensorValues[1] > OUTER_BLACK_THRESHOLD)) {
    
    unsigned long pivotStart = millis();
    while (true) {
      setMotor(MAX_SPEED, -MAX_SPEED);
      qtra.readCalibrated(sensorValues);
      if (sensorValues[3] < OUTER_WHITE_THRESHOLD || sensorValues[4] < OUTER_WHITE_THRESHOLD) break;

      if (millis() - pivotStart > 2000) break;
    }
    setMotor(0, 0);
    lastError = 0;
    return;
  }
  if ((sensorValues[0] < OUTER_WHITE_THRESHOLD && sensorValues[1] < OUTER_WHITE_THRESHOLD) &&
      (sensorValues[6] > OUTER_BLACK_THRESHOLD && sensorValues[7] > OUTER_BLACK_THRESHOLD)) {
    
    unsigned long pivotStart = millis();
    while (true) {
      setMotor(-MAX_SPEED, MAX_SPEED);
      qtra.readCalibrated(sensorValues);
      if (sensorValues[3] < OUTER_WHITE_THRESHOLD || sensorValues[4] < OUTER_WHITE_THRESHOLD) break;

      if (millis() - pivotStart > 2000) break;
    }
    setMotor(0, 0);
    lastError = 0;
    return;
  }





// }
//  else {
//     // White-line mode: low values = line
//     if ((sensorValues[6] < WHITE_MODE_LINE_TH && sensorValues[7] < WHITE_MODE_LINE_TH)
//         && (sensorValues[0] > WHITE_MODE_BG_TH && sensorValues[1] > WHITE_MODE_BG_TH)) {

//       // Stop briefly
//       setMotor(0, 0);
//       delay(40);

//       // Pivot right for a short window, but break early if center sensors detect the white line again
//       unsigned long pivotStart = millis();
//       while (millis() - pivotStart < SPECIAL_PIVOT_TIME) {
//         setMotor(MAX_SPEED, -MAX_SPEED);
//         qtra.readCalibrated(sensorValues);
//         for (int i = 2; i <= 5; i++) { // inner sensors
//           if (sensorValues[i] < WHITE_MODE_LINE_TH) { setMotor(0,0); lastError = 0; return; }
//         }
//       }
//       lastError = 0;
//       return;
//     }


  }




 




  // Surface detection & mode switching
  unsigned long sum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) sum += sensorValues[i];
  int avg = sum / NUM_SENSORS;
  if (avg > BLACK_SURFACE_THRESHOLD) {
    if (++surfaceReadingCount >= NUM_CONSECUTIVE_READINGS)
      whiteLineMode = true;
  } else {
    surfaceReadingCount = 0;
    whiteLineMode = false;
  }

  position = calculateLinePosition(whiteLineMode);

  // On-line detection
  bool onLine = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (whiteLineMode ? (sensorValues[i] < 300) : (sensorValues[i] > 500)) {
      onLine = true;
      lastFoundTime = millis();
      break;
    }
  }

  // If on line, store snapshot of sensor readings for later use
  if (onLine) {
    for (int i = 0; i < NUM_SENSORS; i++) lastSeenSensorValues[i] = sensorValues[i];
    lastSeenTime = millis();
  }

  // Track last side seen (outer sensors)
  if (onLine) {
    if (whiteLineMode) {
      if (sensorValues[0] < 350 || sensorValues[1] < 350) lastLineSide = 1;
      else if (sensorValues[6] < 350 || sensorValues[7] < 350) lastLineSide = -1;
    } else {
      if (sensorValues[0] > 550 || sensorValues[1] > 550) lastLineSide = 1;
      else if (sensorValues[6] > 550 || sensorValues[7] > 550) lastLineSide = -1;
    }
  }

  int error = position - 3500;

  // TIGHTER dead-zone
  if (abs(error) < 400) error = 0;

  // Line-loss recovery with last-seen logic
  if (!onLine) {
    unsigned long dt = millis() - lastFoundTime;

    if (dt < 400) {
      // very recent loss: keep going straight briefly
      error = 0;
    } 
    else if (dt < RECOVERY_DELAY) {
      // moderate loss: use lastSeenSensorValues if available (prefer it)
      if (lastSeenTime != 0 && millis() - lastSeenTime < 1000) {
        // compute a position from the last seen GOOD snapshot
        unsigned int lastSeenPos = calculateLinePositionFrom(lastSeenSensorValues, whiteLineMode); // ~0..7000
        int seenError = (int)lastSeenPos - 3500;

        // If last-seen strongly on one side, prefer a hard bias toward that side
        if (abs(seenError) > 1200) {
          // scale up to make correction aggressive but bounded
          error = (seenError > 0) ? min(3000, seenError * 1) : max(-3000, seenError * 1);
        } else {
          // fallback to lastLineSide or lastError
          error = (lastLineSide != 0) ? (lastLineSide * 3000) : (lastError > 0 ? 3000 : -3000);
        }

        // QUICK pivot if lastSeen snapshot has a strong outer sensor reading (kept as fallback)
        if (!whiteLineMode) {
          if (lastSeenSensorValues[0] > 600 || lastSeenSensorValues[1] > 600) {
            unsigned long pivotStart = millis();
            while (millis() - pivotStart < 100) {
              setMotor(-MAX_SPEED, MAX_SPEED);
              qtra.readCalibrated(sensorValues);
              for (int i = 0; i < NUM_SENSORS; i++)
                if (whiteLineMode ? (sensorValues[i] < 300) : (sensorValues[i] > 500)) return;
            }
            lastError = 0;
            return;
          }
          if (lastSeenSensorValues[7] > 600 || lastSeenSensorValues[6] > 600) {
            unsigned long pivotStart = millis();
            while (millis() - pivotStart < 100) {
              setMotor(MAX_SPEED, -MAX_SPEED);
              qtra.readCalibrated(sensorValues);
              for (int i = 0; i < NUM_SENSORS; i++)
                if (whiteLineMode ? (sensorValues[i] < 300) : (sensorValues[i] > 500)) return;
            }
            lastError = 0;
            return;
          }
        } else {
          if (lastSeenSensorValues[0] < 300 || lastSeenSensorValues[1] < 300) {
            unsigned long pivotStart = millis();
            while (millis() - pivotStart < 100) {
              setMotor(-MAX_SPEED, MAX_SPEED);
              qtra.readCalibrated(sensorValues);
              for (int i = 0; i < NUM_SENSORS; i++)
                if (whiteLineMode ? (sensorValues[i] < 300) : (sensorValues[i] > 500)) return;
            }
            lastError = 0;
            return;
          }
          if (lastSeenSensorValues[7] < 300 || lastSeenSensorValues[6] < 300) {
            unsigned long pivotStart = millis();
            while (millis() - pivotStart < 100) {
              setMotor(MAX_SPEED, -MAX_SPEED);
              qtra.readCalibrated(sensorValues);
              for (int i = 0; i < NUM_SENSORS; i++)
                if (whiteLineMode ? (sensorValues[i] < 300) : (sensorValues[i] > 500)) return;
            }
            lastError = 0;
            return;
          }
        }

      } else {
        // No recent snapshot: fallback to previous behavior
        error = (lastLineSide != 0)
              ? (lastLineSide * 3000)
              : (lastError > 0 ? 3000 : -3000);
      }
    }
    else {
      // Long loss: go to full recovery
      performTurnRecovery();
      return;
    }
  }

  // PID with derivative filter
  int derivative;
  static float filteredDerivative = 0;
  derivative = error - lastError;
  filteredDerivative = filteredDerivative * (1.0 - DERIVATIVE_FILTER)
                     + derivative * DERIVATIVE_FILTER;

  int motorSpeed = (int)(Kp * error + Kd * filteredDerivative);

  // EARLY & BIGGER turn boost
  int turnBoost = 0;
  if (abs(error) > 2000) {
    turnBoost = map(abs(error), 2000, 3500, 0, TURN_BOOST);
  }

  int leftSpeed  = BASE_SPEED + motorSpeed;
  int rightSpeed = BASE_SPEED - motorSpeed;
  if (error < 0) {
    leftSpeed  -= turnBoost;
    rightSpeed += turnBoost;
  } else {
    leftSpeed  += turnBoost;
    rightSpeed -= turnBoost;
  }

  leftSpeed  = constrain(leftSpeed,  -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  setMotor(leftSpeed, rightSpeed);
  lastError = error;
}

// Weighted line position
unsigned int calculateLinePosition(bool whiteLine) {
  long wSum = 0, sSum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = whiteLine ? (1000 - sensorValues[i]) : sensorValues[i];
    int weight = (i == 0 || i == 7) ? 1500
               : (i == 1 || i == 6) ? 1200
               : 1000;
    wSum += (long)val * i * weight;
    sSum += val;
  }
  return sSum ? wSum / sSum : 3500;
}

// same calc but from provided sensor array (used for last-seen snapshot)
unsigned int calculateLinePositionFrom(const unsigned int vals[], bool whiteLine) {
  long wSum = 0, sSum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = whiteLine ? (1000 - (int)vals[i]) : (int)vals[i];
    int weight = (i == 0 || i == 7) ? 1500
               : (i == 1 || i == 6) ? 1200
               : 1000;
    wSum += (long)val * i * weight;
    sSum += val;
  }
  return sSum ? wSum / sSum : 3500;
}

// Aggressive turn recovery
void performTurnRecovery() {
  if (lastLineSide == -1)       setMotor(-MAX_SPEED, MAX_SPEED);
  else if (lastLineSide == 1)   setMotor( MAX_SPEED, -MAX_SPEED);
  else if (lastError < 0)       setMotor(-MAX_SPEED, MAX_SPEED);
  else                          setMotor( MAX_SPEED, -MAX_SPEED);

  unsigned long start = millis();
  // LONGER hard-pivot window for line re-acquisition
  while (millis() - start < 200) {
    qtra.readCalibrated(sensorValues);
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (whiteLineMode ? (sensorValues[i] < 300)
                        : (sensorValues[i] > 500)) {
        return;
      }
    }
  }
}

// Motor drive helper
void setMotor(int leftSpeed, int rightSpeed) {
  digitalWrite(AI1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(AI2, leftSpeed > 0 ? LOW  : HIGH);
  analogWrite(PWMA, abs(leftSpeed));

  digitalWrite(BI1, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(BI2, rightSpeed > 0 ? LOW  : HIGH);
  analogWrite(PWMB, abs(rightSpeed));
}

// Standby LED blink
void blinkStandbyLED() {
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 500) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = millis();
  }
}
