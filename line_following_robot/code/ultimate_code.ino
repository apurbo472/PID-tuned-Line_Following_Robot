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

// PID constants - OPTIMIZED FOR STABILITY
const float Kp = 0.13;
const float Ki = 0.00005;    // Reduced to prevent windup
const float Kd = 01.3;       // Increased for better damping

// Derivative filtering
const float DERIVATIVE_FILTER = 0.35;  // Stronger filtering

// Motor control - OPTIMIZED FOR SHARPER TURNS
const int BASE_SPEED = 90;
const int MAX_SPEED = 160;
const int TURN_BOOST = 80;  // Increased turn boost

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
int lastLineSide = 0;  // -1 = left, 1 = right, 0 = center/unknown
const unsigned long RECOVERY_DELAY = 400;  // Reduced recovery delay

// State control
bool robotRunning = false;
unsigned long startTime = 0;
bool startTriggered = false;

// Dual mode tracking
bool whiteLineMode = false;
const int BLACK_SURFACE_THRESHOLD = 800;
const int NUM_CONSECUTIVE_READINGS = 5;
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
  for (int i = 0; i < 100; i++) {
    qtra.calibrate();
    
    // Add movement every 20 iterations for better calibration
    if (i % 20 == 0) {
      setMotor(50, -50);
      delay(5);
      setMotor(-50, 50);
      delay(5);
    }
    delay(10);
  }
  setMotor(0, 0);  // Ensure motors stop
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  bool switchActive = (digitalRead(START_SWITCH) == LOW);
  if (!switchActive) {
    // Standby
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
  




// Right after: qtra.readCalibrated(sensorValues);

if ((sensorValues[0] > 550 || sensorValues[1] > 550) && sensorValues[6] < 500 && sensorValues[7] < 500) {
  // Left side sees black, right side is white → turn left
  setMotor(-MAX_SPEED, MAX_SPEED);
  return;
}
else if ((sensorValues[6] > 550 || sensorValues[7] > 550) && sensorValues[0] < 500 && sensorValues[1] < 500) {
  // Right side sees black, left side is white → turn right
  setMotor(MAX_SPEED, -MAX_SPEED);
  return;
}
else if ((sensorValues[0] > 500 || sensorValues[1] > 500) && (sensorValues[6] > 500 || sensorValues[7] > 500)) {
  // Both corners see black → go straight
  setMotor(BASE_SPEED, BASE_SPEED);
  return;
}



















  // Surface detection and mode switching
  unsigned long sensorSum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorSum += sensorValues[i];
  }
  int sensorAverage = sensorSum / NUM_SENSORS;
  
  // Detect if we're on a black surface (potential white line)
  if (sensorAverage > BLACK_SURFACE_THRESHOLD) {
    surfaceReadingCount++;
    if (surfaceReadingCount >= NUM_CONSECUTIVE_READINGS) {
      whiteLineMode = true;  // Switch to white-line mode
    }
  } else {
    surfaceReadingCount = 0;
    whiteLineMode = false;  // Default to black-line mode
  }

  // Custom weighted position calculation (more efficient than qtra.readLine)
  position = calculateLinePosition(whiteLineMode);

  // Detect if robot is on line with optimized thresholds
  bool onLine = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (whiteLineMode) {
      if (sensorValues[i] < 300) {
        onLine = true;
        lastFoundTime = millis();
        break;
      }
    } else {
      if (sensorValues[i] > 500) {
        onLine = true;
        lastFoundTime = millis();
        break;
      }
    }
  }

  // Track which side the line was last seen on (outer sensors only)
  if (onLine) {
    if (whiteLineMode) {
      // White line (low values) - outer sensors only
      if (sensorValues[0] < 350 || sensorValues[1] < 350 ) {
        lastLineSide = 1;   // Right side
      } else if (sensorValues[6] < 350 || sensorValues[7] < 350 ) {
        lastLineSide = -1;  // Left side
      }








    } else {
      // Black line (high values) - outer sensors only
      if (sensorValues[0] > 550 || sensorValues[1] > 550 ) {
        lastLineSide = 1;   // Right side


      } else if (sensorValues[6] > 550 || sensorValues[7] > 550 ) {
        lastLineSide = -1;  // Left side
      }
    }
  }








  int error = position - 3500;
  
  // Wider deadzone to reduce oscillation
  if (abs(error) < 400) error = 0;  // Increased from 200






  // Handle line loss with faster recovery
  if (!onLine) {
    unsigned long dt = millis() - lastFoundTime;
    
    if (dt < 300) {  // Reduced from 400
      // Continue straight briefly
      error = 0;
    } 
    else if (dt < RECOVERY_DELAY ) {
      // Gentle turn in last known direction
      error = (lastLineSide != 0) ? (lastLineSide * 3000) : (lastError > 0 ? 3000 : -3000);
    }
    else {
      // Aggressive turn recovery after delay
      performTurnRecovery();
      return;
    }
  }

  // Enhanced PID calculation with stronger derivative filtering
  int derivative = error - lastError;
  static float filteredDerivative = 0;
  filteredDerivative = (filteredDerivative * (1.0 - DERIVATIVE_FILTER)) 
                     + (derivative * DERIVATIVE_FILTER);
  
  int motorSpeed = Kp * error + Kd * filteredDerivative;
  
  // Dynamic turn boost for sharper turns
  int turnBoost = 0;
  if (abs(error) > 1500) {
    turnBoost = map(abs(error), 1500, 3500, 0, TURN_BOOST);
  }

  int leftSpeed = BASE_SPEED + motorSpeed;
  int rightSpeed = BASE_SPEED - motorSpeed;
  
  // Apply turn boost to outer wheel
  if (error < 0) {  // Turning left
    leftSpeed -= turnBoost;
    rightSpeed += turnBoost;
  } else {          // Turning right
    leftSpeed += turnBoost;
    rightSpeed -= turnBoost;
  }
  
  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  
  setMotor(leftSpeed, rightSpeed);
  
  // Update last error
  lastError = error;
}

// Custom weighted position calculation
unsigned int calculateLinePosition(bool whiteLine) {
  long weightedSum = 0;
  long sum = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = whiteLine ? (1000 - sensorValues[i]) : sensorValues[i];
    
    // Apply non-linear weighting for better corner detection
    int weight = 1000;  // Default weight
    if (i == 0 || i == 7) weight = 1500;  // Highest weight for outer sensors
    if (i == 1 || i == 6) weight = 1200;  // Medium weight for near-outer sensors
    
    weightedSum += (long)value * i * weight;
    sum += value;
  }
  
  return (sum == 0) ? 3500 : weightedSum / sum;
}

// Aggressive turn recovery
void performTurnRecovery() {
  if (lastLineSide == -1) {
    // MAX power turn left: right forward, left backward
    setMotor(-MAX_SPEED, MAX_SPEED);
  } 
  else if (lastLineSide == 1) {
    // MAX power turn right: left forward, right backward
    setMotor(MAX_SPEED, -MAX_SPEED);
  } 
  else {
    // Use last error direction
    if (lastError < 0) {
      setMotor(-MAX_SPEED, MAX_SPEED); // Turn left
    } else {
      setMotor(MAX_SPEED, -MAX_SPEED); // Turn right
    }
  }
  
  // Faster recovery time (150ms)
  unsigned long recoveryStart = millis();
  while (millis() - recoveryStart < 150) {  // Reduced from 200
    // Continue checking for line during recovery
    qtra.readCalibrated(sensorValues);
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (whiteLineMode) {
        if (sensorValues[i] < 300) return; // White line found
      } else {
        if (sensorValues[i] > 500) return; // Black line found
      }
    }
  }
}

// Motor control function
void setMotor(int leftSpeed, int rightSpeed) {
  // Left Motor
  digitalWrite(AI1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(AI2, leftSpeed > 0 ? LOW  : HIGH);
  analogWrite(PWMA, abs(leftSpeed));
  
  // Right Motor
  digitalWrite(BI1, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(BI2, rightSpeed > 0 ? LOW  : HIGH);
  analogWrite(PWMB, abs(rightSpeed));
}

// Standby LED function
void blinkStandbyLED() {
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 500) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = millis();
  }
}