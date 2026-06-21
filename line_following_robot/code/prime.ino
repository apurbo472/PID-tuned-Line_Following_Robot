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
const int BASE_SPEED    = 120;
const int MAX_SPEED     = 180;   // raised from 160
const int TURN_BOOST    = 80;   // raised from 80

// Sensor setup
#define NUM_SENSORS 8
const uint8_t sensorPins[NUM_SENSORS] = { A7, A6, A5, A4, A3, A2, A1, A0 };
QTRSensorsAnalog qtra(sensorPins, NUM_SENSORS);
unsigned int sensorValues[NUM_SENSORS];
unsigned int position = 0;

// Recovery & timing
const unsigned long RECOVERY_DELAY = 50;   // lowered from 400ms

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

// Function prototypes
void setMotor(int leftSpeed, int rightSpeed);
void blinkStandbyLED();
void performTurnRecovery();
unsigned int calculateLinePosition(bool whiteLine);

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
  for (int i = 0; i < 200; i++) {
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

  // On‑line detection
  bool onLine = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (whiteLineMode ? (sensorValues[i] < 300) : (sensorValues[i] > 500)) {
      onLine = true;
      lastFoundTime = millis();
      break;
    }
  }

  // Track last side seen
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

  // TIGHTER dead‑zone
  if (abs(error) < 400) error = 0;

  // Line‑loss recovery
  if (!onLine) {
    unsigned long dt = millis() - lastFoundTime;
    if (dt < 200) {
      error = 0;
    } 
    else if (dt < RECOVERY_DELAY) {
      error = (lastLineSide != 0)
            ? (lastLineSide * 3000)
            : (lastError > 0 ? 3000 : -3000);
    }
    else {
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

  int motorSpeed = Kp * error + Kd * filteredDerivative;

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

// Aggressive turn recovery
void performTurnRecovery() {
  if (lastLineSide == -1)       setMotor(-MAX_SPEED, MAX_SPEED);
  else if (lastLineSide == 1)   setMotor( MAX_SPEED, -MAX_SPEED);
  else if (lastError < 0)       setMotor(-MAX_SPEED, MAX_SPEED);
  else                          setMotor( MAX_SPEED, -MAX_SPEED);

  unsigned long start = millis();
  // LONGER hard‑pivot window for line re-acquisition
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
