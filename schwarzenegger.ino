#include<Servo.h>    // standard library for servo motor
#include<HCSR04.h> // Sketch -> Include Library > Add .ZIP Library > Select the HCSR04_vX.X.X.zip file in the repository folder.

int IRPin = A0; // Analog input pin that the sensor is attached to

// You can instead of using all void() use this enum for directions and how the motor works for more effiency and clean overview
bool goesForward = false;
enum Direction { STOP, FORWARD, LEFT, RIGHT };
Direction currentDirection = STOP;
const float IRValueMin = 0;
const float IRValueMax = 60; // TODO define this
const float IRThreshold = -1; // TODO define this
const float ServoControlMin = 0; // TODO define this
const float ServoControlMax = 1; // TODO define this

// Motor control
const int Lmotor = 6;
const int Rmotor = 7;

// LED and Buzzer 
const int buzzer = 9;
const int LEDorange = 2;
const int LEDred = 3;
const int LEDgreen = 4;

// Touch Sensor 
const int touchSensor = 5;
int touchCount = 0;
bool wasTouched = false;
unsigned long lastTouchTime = 0;
const unsigned long doubleTapWindow = 500; // user has 600ms to triple touch

// Sonar & Servo
#define debug true
#define servo_power 2
#define servo_pin 3
#define trig_pin A1
#define echo_pin A2
const int sonarMaxDistance = 250;
const float distanceChangeThreshold = 0.4; // TODO test
const int servoTotalAngle = 180;
const int servoDirections = 20; // make sure servoTotalAngle/servoDirections is a whole number
const int servoChangeAngleDelay = 200;
const int servoAngleStepSize = servoTotalAngle / servoDirections;
const bool servoSwivel = true; // if false, rotates
int servoDirectionNumber = 0;
HCSR04 sonar = HCSR04(trig_pin, echo_pin); // sensor function
Servo servo = Servo();
bool servoTurnsRight = true;
int sonarDistances[servoDirections];

// Robot state
enum State {IDLE_, INVESTIGATING, HOSTILE, FRIENDLY};
int currentState = IDLE_;

void setup() {
  Serial.begin(9600);
  
  pinMode(Lmotor, OUTPUT); //pin 6
  pinMode(Rmotor, OUTPUT); //pin 7

  // infrared
  pinMode(IRPin, INPUT); // Set the sensor pin as an input

  servo.attach(servo_pin); // the servo pin

  setServo(0); //set servomotor in forward position

  // now we calibrate the sensor to get the distance
  sonar.dist();
  delay(100);
  sonar.dist();
  delay(100);
  sonar.dist();
  delay(100);

  //touchsensor : 
  pinMode(touchSensor, INPUT);
  //LED buzzer 
  pinMode(buzzer, OUTPUT); 
  pinMode(LEDorange, OUTPUT);
  pinMode(LEDred, OUTPUT);
  pinMode(LEDgreen, OUTPUT);

  pinMode(servo_power, OUTPUT);
  digitalWrite(servo_power, HIGH);

  Serial.println("Setup complete");
  Serial.print("States | IDLE_: ");
  Serial.print(IDLE_);
  Serial.print(" | INVESTIGATING: ");
  Serial.print(INVESTIGATING);
  Serial.print(" | HOSTILE: ");
  Serial.print(HOSTILE);
  Serial.print(" | FRIENDLY: ");
  Serial.println(FRIENDLY);
}

// and the in the void loop you can change state based on button pressed or timer runs out, like example
// this is only an example since i dont know the intricate parts of the button press yet but
void loop() {
  switch(currentState) {
    case IDLE_: {
      /// ACTION ///
      // Robot stays in place
      moveStop();

      // Sjoerd
      /// STATE SWITCH ///

      // Button pressed -> Friendly
      if (touchSensorPressed()) {
        updateRobotState(FRIENDLY);
        break;
      }

      // Distance to a certain direction changes significicantly -> Investigating
      int newServoDirectionNumber; // Calculate new direction
      if (servoTurnsRight) {
        newServoDirectionNumber = servoDirectionNumber + 1;
      } else {
        newServoDirectionNumber = servoDirectionNumber - 1;
      }

      if (servoSwivel) {
        if (newServoDirectionNumber < 0 || newServoDirectionNumber >= servoDirections) { // Switch direction
          servoTurnsRight = !servoTurnsRight;
          break;
        }
      } else { // rotate fully
        if (newServoDirectionNumber < 0) {
          newServoDirectionNumber = servoDirections - 1;
        } else if (newServoDirectionNumber >= servoDirections) {
          newServoDirectionNumber = 0;
        }
      }

      setServo(newServoDirectionNumber * servoAngleStepSize); // Set direction & wait; this updates servoDirection
      delay(servoChangeAngleDelay);

      float cDist = sonar.dist();

      if (debug) {
        Serial.print("Distance at ");
        Serial.print(newServoDirectionNumber * servoAngleStepSize);
        Serial.print(" = ");
        Serial.println(cDist);
        Serial.print("Distances now: [");
        for (int i = 0; i < servoDirections - 1; i++) {
          Serial.print(sonarDistances[i]);
          Serial.print(", ");
        }
        Serial.print(sonarDistances[servoDirections - 1]);
        Serial.println("]");
      }

      if (updateAndTestSonar(newServoDirectionNumber, cDist, distanceChangeThreshold)) { // Ping sonar and update
        updateRobotState(INVESTIGATING);
      }
      break;
    }
    case INVESTIGATING: {
      /// ACTION ///
      // Salim
      // Drive in direction of potential intruder to pick up heat signature. Avoid obstacles where possible

      /// STATE SWITCH ///
      // Salim
      // if heat signature detected (so there's a person) & they are not authenticated withing 5 seconds -> Hostile
//      if (IRValue > threshold) {
//        currentState = HOSTILE;
//      }
      
      // If user authenticated -> Friendly
      if (touchSensorPressed()) {
        updateRobotState(FRIENDLY);
      }
      
       // Salim
      // if nothing detected for 20 seconds, drive back to original location -> Idle. Avoid obstacles where possible
      break;
    }
    case HOSTILE: { // Tom & Jen
      /// ACTION ///
      // TODO Chasing -> then maintain distance to avoid getting smacked. Avoid obstacles where possible
    
      int IRValue = analogRead(IRPin); // Read the value from the IR sensor
      float distance = sonar.dist();
      delay(200); 
      
     if (distance < 20) {  // so while still checking if there are no obstacles
       currentDirection = STOP; // if there are we stop
       // dont know the range of the irValues so we cannot set a threshold yet?
     } else if (IRValue > IRThreshold) { // then we check for the treshold again
       moveTowardsTarget(IRValue); // using the code we can move toward a certain angle but dont know if the robot can this yet?
     } else {
       currentState = IDLE_;  // when we cannot find the target anymore
     }
      
      /// STATE SWITCH ///
      // if no heat signature detected for X time -> Idle
      // if three separate taps on the button -> Friendly
      bool isTouched = digitalRead(touchSensor) == HIGH; 
      if (isTouched && !wasTouched) {
        if (millis() - lastTouchTime <= doubleTapWindow) { //verify if the last touch was made in a short enough time to be consider as "succesive" touch
          touchCount++; 
          if (touchCount == 3) { //if 3 succesive touch then turn on the led 
            updateRobotState(FRIENDLY);
          }
        } else {
          touchCount = 1; //one bc the first touch is still taken into account
        }
        wasTouched = true;

        lastTouchTime = millis(); //update the touch time
      } else if (!isTouched && wasTouched) {
        wasTouched = false; //put back wastouch to 0 if the sensor is not activated anymore
      }

      if (touchCount != 3) { //tursive touch then turn on the led 
        updateRobotState(HOSTILE);
      }
      break;
    }
    case FRIENDLY: { // Xavier + all the LEDs & Buzzers & Button in the idle mode
      /// ACTION ///
      // Robot stays in place
      moveStop();

      /// STATE SWITCH ///
      // if button is pressed again and then 10 seconds pass -> Idle
      if (touchSensorPressed()) {
        delay(10000);
        updateRobotState(IDLE_);
      }
      break;
    }
  }
}

// ?
void updateMovement(Direction dir) {
  switch (dir) {
    case FORWARD:
      digitalWrite(Lmotor, HIGH);
      digitalWrite(Rmotor, HIGH);
      break;
    case LEFT:
      digitalWrite(Lmotor, HIGH);
      digitalWrite(Rmotor, LOW);
      break;
    case RIGHT:
      digitalWrite(Lmotor, LOW);
      digitalWrite(Rmotor, HIGH);
      break;
    case STOP:
    default:
      digitalWrite(Lmotor, LOW);
      digitalWrite(Rmotor, LOW);
      break;
  }
  currentDirection = dir;
}

// Sjoerd
void setServo(int direction) {
  servoDirectionNumber = round(direction / servoAngleStepSize);
  servo.write(direction);
}

// Xavier
bool touchSensorPressed() {
  if (touchSensor == HIGH) {
    return true; 
  } else {
    return false; // 
  }
}

// Sjoerd
// Scans in all 360 degree distances and returns true if there is a change in distance significant enough
bool updateAndTestSonar(int dirIndex, int cm, float distanceChangeRequired) {
  int previousCm = sonarDistances[dirIndex];
  sonarDistances[dirIndex] = round(cm);
  if (previousCm == 0) { // first measurement
    return false;
  }
  // the distance difference is more than the distance change requirement
  return abs(cm - previousCm) > distanceChangeRequired * max(cm, previousCm);
}

// Sjoerd
void updateRobotState(int state) {
  Serial.print("State switched from ");
  Serial.print(currentState);
  Serial.print(" to ");
  Serial.println(state);
  currentState = state;
  updateRobotAppearance(state);
  return;
}

// Xavier
void updateRobotAppearance(int state) {
  switch (state) {
    case IDLE_: {
      // Set LED to OFF
      digitalWrite(LEDorange, LOW);
      digitalWrite(LEDred, LOW);
      digitalWrite(LEDgreen, LOW);
      // Set buzzer to OFF
      digitalWrite(buzzer, LOW);
      break;
    }
    case INVESTIGATING: {
      // Set LED to Orange
      digitalWrite(LEDorange, HIGH);
      digitalWrite(LEDred, LOW);
      digitalWrite(LEDgreen, LOW);
      // Set buzzer to "beeping" bc it sounds nice 
      tone(buzzer, 50); // Send 0.05KHz sound signal
      delay(1000);      // for 1 sec TODO fix that this blocks the robot's thought processes for a second
      noTone(buzzer);   // Stop sound
      break;
    }
    case HOSTILE: {
      // Set LED to red
      digitalWrite(LEDorange, LOW);
      digitalWrite(LEDred, HIGH);
      digitalWrite(LEDgreen, LOW);
      // Set buzzer to "terminator"
      tone(buzzer, 3000);
      delay(1000);
      noTone(buzzer);
      break;
    }
    case FRIENDLY: {
      // Set LED to Green
      digitalWrite(LEDorange, LOW);
      digitalWrite(LEDred, LOW);
      digitalWrite(LEDgreen, HIGH);
      // Set Buzzer to "Happy"
      tone(buzzer, 300);
      delay(1000);
      noTone(buzzer);
      break;
    }
  }
  return;
}

//
//void loop() {
//  int distanceR = 0;
//  int distanceL = 0;
//
//  delay(50);
//
//  if (distance <= 20){  //so something gets too close
//    moveStop();
//    delay(300);
//    distanceR = lookR();
//    delay(300);
//    distanceL = lookL();
//    delay(300);
//
//    if (distance >= distanceL){ //if there is an obstacle on the left
//      turnR();
//      moveStop();
//    }
//    else{ //if the left is clear
//      turnL();
//      moveStop();
//    }
//  }
//  else {
//    moveForward();
//  }
//  distance = readPing();
//}
//
void moveTowardsTarget(int IRValue) {
 // and the function would look a bit like with for a motor driver that uses servo control
//  leftMotor.write(map(IRValue, IRValueMin, IRValueMax, ServoControlMin, ServoControlMax)); //map you can move the wheel in certain angles and degrees
//  rightMotor.write(map(IRValue, IRValueMin, IRValueMax, ServoControlMin, ServoControlMax)) //based on the ir value perceived
}
//
// int lookL(){
//   setServo(170);
//   delay(500);
//   int distance = readPing();
//   delay(100);
//   setServo(115);
//   return distance;
// }
//
// int lookR(){
//   setServo(50);
//   delay(500);
//   int distance = readPing();
//   delay(100);
//   setServo(115);
//   return distance;
// }
//
// int readPing(){
//   delay(70);
//   int cm = sonar.ping_cm();
//   if (cm==0){
//     cm = 250;
//   }
//   return cm;
// }

void moveStop(){
  digitalWrite(Lmotor, LOW);
  digitalWrite(Rmotor, LOW);
}

void moveForward(){
  if(!goesForward){
    goesForward = true;
    digitalWrite(Lmotor, HIGH);
    digitalWrite(Rmotor, HIGH);
  }
}

void turnL(){
  digitalWrite(Lmotor, HIGH);
  digitalWrite(Rmotor, LOW);
  delay(500);

  digitalWrite(Lmotor, HIGH);
  digitalWrite(Rmotor, HIGH);
}

void turnR(){
  digitalWrite(Lmotor, LOW);
  digitalWrite(Rmotor, HIGH);
  delay(500);

  digitalWrite(Lmotor, HIGH);
  digitalWrite(Rmotor, HIGH);
}
