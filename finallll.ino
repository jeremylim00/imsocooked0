#include <BluetoothSerial.h>
#include <Servo.h>

BluetoothSerial SerialBT;
Servo myServo;

// -------------------------- PINS --------------------------
int IN1 = 14, IN2 = 12, ENA = 27; // Motor A
int IN3 = 22, IN4 = 23, ENB = 5;  // Motor B

int ir1 = 32, ir2 = 33, ir3 = 25, ir4 = 26; // IR Sensors
int servoPin = 13;

// -------------------------- STATES -----------------------
bool autoMode = false;
int targetLocation = 0;
int leftTurnCount = 0;

// Full-black consecutive count (resets when not all black)
int fullBlackCounter = 0;
const int FULL_BLACK_REQUIRED = 2; // require 2 consecutive reads

// Turn flags/timers
bool turnInProgress = false;
unsigned long turnStartTime = 0;
const unsigned long turnDuration = 360; // ms

// Left turn lock to avoid over-counting
bool leftTurnLock = false;

// Post-left-turn stop/timer
bool postLeftTurnStopActive = false;
unsigned long postLeftTurnStartTime = 0;
unsigned long postLeftTurnDuration = 250; // ms wait after left turn before checking IR

// Permanent full-stop after left-turn (autonomy disabled, manual allowed)
bool leftTurnFullStop = false;

bool servoClosed = false;  // false = 0°, true = 90°


// PID variables
float Kp = 19.0f, Ki = 0.0f, Kd = 5.0f;
float error = 0.0f, lastError = 0.0f, sumError = 0.0f;
int baseSpeed = 50;

// -------------------------- SETUP ------------------------
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-Car");

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  pinMode(ir1, INPUT); pinMode(ir2, INPUT);
  pinMode(ir3, INPUT); pinMode(ir4, INPUT);

  myServo.attach(servoPin);
  myServo.write(0);

  stopCar();
  Serial.println("Ready");
}

// -------------------------- MOTOR FUNCTIONS ------------------------
void forward(int speed = 140) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}

void backward(int speed = 140) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}

void turnLeft(int leftSpeedVal = 140, int rightSpeedVal = 140) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, leftSpeedVal); analogWrite(ENB, rightSpeedVal);
}

void turnRight(int leftSpeedVal = 140, int rightSpeedVal = 140) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, leftSpeedVal); analogWrite(ENB, rightSpeedVal);
}

void stopCar() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

// -------------------------- LOOP ------------------------
void loop() {
  // ------------------- BLUETOOTH CONTROL -------------------
  if (SerialBT.available()) {
    char cmd = SerialBT.read();

    // START autonomous only if not permanently stopped
    if (cmd == 'A') {
      if (!leftTurnFullStop) {
        autoMode = true;
        Serial.println("Autonomous ON");
      } else {
        Serial.println("Cannot start autonomous: car in permanent stop. Use manual controls.");
      }
    }

    // Manual control is ALWAYS allowed when not in autoMode.
    if (!autoMode) {
      if (cmd == 'F') forward();
      if (cmd == 'B') backward();
      if (cmd == 'L') turnLeft();
      if (cmd == 'R') turnRight();
      if (cmd == 'X') stopCar();
      if(cmd=='0'){
      if(servoClosed){
        myServo.write(0);     // open
        servoClosed = false;
        Serial.println("Servo opened (0°)");
      } else {
        myServo.write(90);    // close
        servoClosed = true;
        Serial.println("Servo closed (90°)");
      }
    }
     
    } else {
      // allow target selection while in autonomous
      if (cmd == 'S') targetLocation = 1;
      if (cmd == 'T') targetLocation = 2;
      if (cmd == 'C') targetLocation = 3;
    }

    // STOP/reset autonomous (does NOT clear permanent stop)
    if (cmd == 'P') {
      autoMode = false;
      targetLocation = 0;
      leftTurnCount = 0;
      fullBlackCounter = 0;
      turnInProgress = false;
      leftTurnLock = false;
      postLeftTurnStopActive = false;
      Serial.println("Autonomous stopped/reset (manual mode).");
    }
  }

  // ------------------- AUTONOMOUS MODE -------------------
  if (autoMode && targetLocation > 0 && !leftTurnFullStop) {
    runAutonomous();
  }
}

// -------------------------- AUTONOMOUS PID + TURN ------------------------
void runAutonomous() {
  unsigned long now = millis();

  bool s1 = digitalRead(ir1);
  bool s2 = digitalRead(ir2);
  bool s3 = digitalRead(ir3);
  bool s4 = digitalRead(ir4);

  Serial.print("IR: "); Serial.print(s1); Serial.print(' ');
  Serial.print(s2); Serial.print(' ');
  Serial.print(s3); Serial.print(' ');
  Serial.println(s4);

  bool allBlack = s1 && s2 && s3 && s4;

  // If permanent stop has been triggered ensure autonomy is off and return
  if (leftTurnFullStop) {
    autoMode = false;
    targetLocation = 0;
    return;
  }

  // --- full-black consecutive counter (resets when not allBlack) ---
  if (allBlack) fullBlackCounter++;
  else fullBlackCounter = 0;

  // -------------------- HANDLE TURNS --------------------
  if (!turnInProgress) {
    // 1) FULL BLACK destination (highest priority)
    if (allBlack && fullBlackCounter >= FULL_BLACK_REQUIRED) {
      stopCar();
      turnInProgress = true;
      turnStartTime = now;
      Serial.println("Event: FULL BLACK (destination) triggered");
      return;
    }

    // 2) LEFT TURN — count only once per junction using lock
    if ((s1 && s2 && s3) || (s1 && s2)) {
      if (!leftTurnLock) {
        leftTurnCount++;
        leftTurnLock = true;
        //Serial.print("Left turn counted -> "); Serial.println(leftTurnCount);

        if (leftTurnCount == targetLocation) {
          // perform left turn (asymmetric wheel speeds to pivot)
          turnLeft(80, 150);
          turnInProgress = true;
          turnStartTime = now;

          // start post-left-turn monitoring (with timer)
          postLeftTurnStopActive = true;
          postLeftTurnStartTime = now;
          //Serial.println("Performing target LEFT TURN; starting post-left monitor");
          return;
        }
      }
    } else {
      // release lock once pattern no longer detected
      leftTurnLock = false;
    }

    // 3) RIGHT TURN (only if left pattern not active)
    if ((s2 && s3 && s4) || (s3 && s4)) {
      turnRight(180, 80);
      turnInProgress = true;
      turnStartTime = now;
      Serial.println("Performing RIGHT TURN");
      return;
    }

  } else {
    // Continue ongoing turn until duration expires
    if (now - turnStartTime >= turnDuration) {
      turnInProgress = false;
      Serial.println("Turn finished");
    }
  }

  // -------------------- POST LEFT TURN FULL STOP --------------------
  if(postLeftTurnStopActive){
    if(now - postLeftTurnStartTime >= postLeftTurnDuration){
      if(s3){ // stop only when ALL IR detect black
        stopCar();
        Serial.println("Car permanently stopped after left turn!");
        leftTurnFullStop = true;   // permanent stop
        postLeftTurnStopActive = false;
        turnInProgress = false;
        autoMode = false;
        targetLocation = 0;
      }
    }
  }

  // -------------------- PID LINE FOLLOWING --------------------
  if (!turnInProgress && !postLeftTurnStopActive) {
    int leftSpeedVal, rightSpeedVal;

    // Determine coarse error from sensors
    if (s2 && s3) error = 0.0f;
    else if (s2) error = 1.0f;
    else if (s3) error = -1.0f;
    else if (s1) error = 2.0f;
    else if (s4) error = -2.0f;
    else error = lastError * 0.5f; // slightly decay last error if lost

    float P = error;
    sumError += error;
    if (error == 0.0f) sumError = 0.0f;
    float I = sumError;
    float D = error - lastError;
    lastError = error;

    float correction = Kp * P + Ki * I + Kd * D;

    leftSpeedVal = constrain((int)(baseSpeed - correction), 30, 100);
    rightSpeedVal = constrain((int)(baseSpeed + correction), 30, 120);

    // Drive motors with PID result
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, leftSpeedVal);
    analogWrite(ENB, rightSpeedVal);
  }
}
