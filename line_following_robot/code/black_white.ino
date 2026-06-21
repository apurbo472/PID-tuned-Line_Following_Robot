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

// PID constants - TUNED FOR SMOOTH AND STABLE TRACKING
const float Kp = 0.08;
const float Ki = 0.0000;
const float Kd = 1.25;

// Derivative filtering
const float DERIVATIVE_FILTER = 0.25;  // Smoother filtering to avoid jitter

// Motor control speeds
const int BASE_SPEED = 90;
const int MAX_SPEED = 160;
const int TURN_BOOST = 80;  // Extra power for sharp corners

// Sensor setup
#define NUM_SENSORS 8
const uint8_t sensorPins[NUM_SENSORS] = { A7, A6, A5, A4, A3, A2, A1, A0 };
QTRSensorsAnalog qtra(sensorPins, NUM_SENSORS);
unsigned int sensorValues[NUM_SENSORS];
unsigned int position = 0;

// PID variables
int lastError = 0;
int integral = 0;
unsigned long lastFoundTime = 0;

// Line tracking variables
int lastLineSide = 0;  // 1 = left, -1 = right, 0 = center/unknown
const unsigned long RECOVERY_DELAY = 400;

// State control
bool robotRunning = false;
unsigned long startTime = 0;
bool startTriggered = false;

// Dual mode tracking
bool whiteLineMode = false;
const int BLACK_SURFACE_THRESHOLD = 800;
const int NUM_CONSECUTIVE_READINGS = 5;
int surfaceReadingCount = 0;

// Line memory for last 20 error values
#define MEMORY_SIZE 20
int errorMemory[MEMORY_SIZE] = {0};
int memoryIndex = 0;

// Function prototypes
void setMotor(int leftSpeed, int rightSpeed);
void blinkStandbyLED();
void performTurnRecovery();
unsigned int calculateLinePosition(bool whiteLine);

void setup() {
  pinMode(AI1, OUTPUT); pinMode(AI2, OUTPUT);
  pinMode(BI1, OUTPUT); pinMode(BI2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);

  pinMode(START_SWITCH, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  // Swing back and forth over the line to calibrate all sensors
  for (int i = 0; i < 200; i++) {
    qtra.calibrate();
    if (i < 50 || i >= 150) {
      setMotor(45, -45); // Turn right
    } else {
      setMotor(-45, 45); // Turn left
    }
    delay(20);
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

  qtra.readCalibrated(sensorValues);
  unsigned long sensorSum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorSum += sensorValues[i];
  }
  int sensorAverage = sensorSum / NUM_SENSORS;

  if (sensorAverage > BLACK_SURFACE_THRESHOLD) {
    surfaceReadingCount++;
    if (surfaceReadingCount >= NUM_CONSECUTIVE_READINGS) {
      whiteLineMode = true;
    }
  } else {
    surfaceReadingCount = 0;
    whiteLineMode = false;
  }

  position = calculateLinePosition(whiteLineMode);

  // Check if line is detected on any sensor
  bool onLine = false;
  unsigned int maxVal = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    unsigned int rawVal = sensorValues[i];
    if (rawVal > 1000) rawVal = 1000;
    unsigned int value = whiteLineMode ? (1000 - rawVal) : rawVal;
    if (value > maxVal) {
      maxVal = value;
    }
  }

  if (maxVal > 500) {
    onLine = true;
    lastFoundTime = millis();
  }

  // Update last seen line side
  if (onLine) {
    unsigned int s0 = whiteLineMode ? (1000 - min(sensorValues[0], 1000)) : sensorValues[0];
    unsigned int s1 = whiteLineMode ? (1000 - min(sensorValues[1], 1000)) : sensorValues[1];
    unsigned int s6 = whiteLineMode ? (1000 - min(sensorValues[6], 1000)) : sensorValues[6];
    unsigned int s7 = whiteLineMode ? (1000 - min(sensorValues[7], 1000)) : sensorValues[7];

    if (s0 > 500 || s1 > 500) {
      lastLineSide = 1;  // Left
    } else if (s6 > 500 || s7 > 500) {
      lastLineSide = -1; // Right
    }
  }

  int error = position - 3500;
  if (abs(error) < 50) error = 0; // Small deadband to prevent high-frequency jitter at center

  errorMemory[memoryIndex] = error;
  memoryIndex = (memoryIndex + 1) % MEMORY_SIZE;

  if (!onLine) {
    unsigned long dt = millis() - lastFoundTime;
    if (dt < 200) {
      // Hold the last error instead of going straight to keep turning towards line
      error = lastError;
    } 
    else if (dt < RECOVERY_DELAY) {
      int avgError = 0;
      for (int i = 0; i < MEMORY_SIZE; i++) avgError += errorMemory[i];
      avgError /= MEMORY_SIZE;
      error = (avgError > 0) ? 3500 : (avgError < 0) ? -3500 : 0;
    } else {
      performTurnRecovery();
      return;
    }
  }

  int derivative = error - lastError;
  static float filteredDerivative = 0;
  filteredDerivative = (filteredDerivative * (1.0 - DERIVATIVE_FILTER)) 
                     + (derivative * DERIVATIVE_FILTER);

  int motorSpeed = Kp * error + Kd * filteredDerivative;
  int turnBoost = 0;
  if (abs(error) > 1500) {
    turnBoost = map(abs(error), 1500, 3500, 0, TURN_BOOST);
  }

  int leftSpeed = BASE_SPEED + motorSpeed;
  int rightSpeed = BASE_SPEED - motorSpeed;
  if (error < 0) {
    leftSpeed -= turnBoost;
    rightSpeed += turnBoost;
  } else {
    leftSpeed += turnBoost;
    rightSpeed -= turnBoost;
  }

  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  setMotor(leftSpeed, rightSpeed);
  lastError = error;
}

unsigned int calculateLinePosition(bool whiteLine) {
  unsigned long weightedSum = 0;
  unsigned long sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    unsigned int rawVal = sensorValues[i];
    if (rawVal > 1000) rawVal = 1000;
    unsigned long value = whiteLine ? (1000 - rawVal) : rawVal;
    
    // Apply symmetric weights to boost outer sensor sensitivity
    unsigned long weight = 1000;
    if (i == 0 || i == 7) weight = 1500;
    else if (i == 1 || i == 6) weight = 1200;
    
    unsigned long weightedValue = value * weight;
    weightedSum += weightedValue * (i * 1000UL);
    sum += weightedValue;
  }
  return (sum == 0) ? 3500 : (weightedSum / sum);
}

void performTurnRecovery() {
  // Turn back towards the direction the line was last seen
  if (lastLineSide == 1) { // Left side
    setMotor(-MAX_SPEED, MAX_SPEED); // Spin left
  } else if (lastLineSide == -1) { // Right side
    setMotor(MAX_SPEED, -MAX_SPEED); // Spin right
  } else {
    if (lastError < 0) {
      setMotor(-MAX_SPEED, MAX_SPEED);
    } else {
      setMotor(MAX_SPEED, -MAX_SPEED);
    }
  }
  unsigned long recoveryStart = millis();
  while (millis() - recoveryStart < 150) {
    qtra.readCalibrated(sensorValues);
    for (int i = 0; i < NUM_SENSORS; i++) {
      unsigned int rawVal = sensorValues[i];
      if (rawVal > 1000) rawVal = 1000;
      unsigned int value = whiteLineMode ? (1000 - rawVal) : rawVal;
      if (value > 500) return; // Recovered the line!
    }
  }
}

void setMotor(int leftSpeed, int rightSpeed) {
  if (leftSpeed == 0) {
    digitalWrite(AI1, LOW);
    digitalWrite(AI2, LOW);
    analogWrite(PWMA, 0);
  } else {
    digitalWrite(AI1, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(AI2, leftSpeed > 0 ? LOW  : HIGH);
    analogWrite(PWMA, abs(leftSpeed));
  }

  if (rightSpeed == 0) {
    digitalWrite(BI1, LOW);
    digitalWrite(BI2, LOW);
    analogWrite(PWMB, 0);
  } else {
    digitalWrite(BI1, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(BI2, rightSpeed > 0 ? LOW  : HIGH);
    analogWrite(PWMB, abs(rightSpeed));
  }
}

void blinkStandbyLED() {
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 500) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = millis();
  }
}