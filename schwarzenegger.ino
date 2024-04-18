#include<NewPing.h>  //https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
#include<Servo.h>    // standard library for servo motor

// You can instead of using all void() use this enum for directions and how the motor works for more effiency and clean overview
enum Direction { STOP, FORWARD, LEFT, RIGHT };
Direction currentDirection = STOP;

// Motor control
const int Lmotor = 6;
const int Rmotor = 7;

// sensorpins for servo
#define trig_pin A1
#define echo_pin A2

// starting values for the system 
int max_distance = 200;
bool goesForward = false;
int distance = 100;

NewPing sonar(trig_pin, echo_pin, max_distance); // sensor function
Servo servo1;

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

// this way of setting enum can also be implemented with states to change from idle to friendly to hostile
enum State {IDLE_, INVESTIGATING, HOSTILE, FRIENDLY};

State currentState = IDLE_;

void setup() {
  pinMode(Lmotor, OUTPUT); //pin 6
  pinMode(Rmotor, OUTPUT); //pin 7

  servo1.attach(9); // the servo pin

  servo1.write(115); //set servomotor in forward position
  delay(2000);

  // now we calibrate the sensor to get the distance
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
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

// and the in the void loop you can change state based on button pressed or timer runs out, like example
// this is only an example since i dont know the intricate parts of the button press yet but
void loop() {
  switch(currentState) {
    case IDLE_: {
      /// ACTION ///
      updateRobotAppearance();

      /// STATE SWITCH ///

      // Button pressed -> Friendly
      if (buttonPressed()) {
        currentState = FRIENDLY;
      }
       // Sjoerd
      // 360 degrees Directional distance change significant enough to be intruder -> Investigating

      break;
    }
    case INVESTIGATING: {
      /// ACTION ///
      updateRobotAppearance();
       // Salim
      // Drive in direction of potential intruder to pick up heat signature. Avoid obstacles where possible

      /// STATE SWITCH ///
       // Salim
      // if heat signature detected (so there's a person) & they are not authenticated withing 5 seconds -> Hostile
//      if (IRValue > threshold) {
//        currentState = HOSTILE;
//      }
      
      // if user authenticated -> Friendly
      if (buttonPressed()) {
        currentState = FRIENDLY;
      }
      
       // Salim
      // if nothing detected for 20 seconds, drive back to original location -> Idle. Avoid obstacles where possible
      break;
    }
    case HOSTILE: { // Tom & Jen
      /// ACTION ///
      updateRobotAppearance();
      // TODO Chasing -> then maintain distance to avoid getting smacked. Avoid obstacles where possible
//      if (distance < 20) {  // so while still checking if there are no obstacles
//        currentDirection = STOP; // if there are we stop
//      } else if (IRValue > threshold) { // then we check for the treshold again
//        moveTowardsTarget(IRValue); // 
//      } else {
//        currentState = IDLE_;  // when we cannot find the target anymore
//      }
      
      /// STATE SWITCH ///
      // if no heat signature detected for X time -> Idle
      // if three separate taps on the button -> Friendly
      break;
    }
    case FRIENDLY: { // Xavier + all the LEDs & Buzzers & Button in the idle mode
      /// ACTION ///
      updateRobotAppearance();
      // Robot stays in place

      /// STATE SWITCH ///
      // if button is pressed again and then 10 seconds pass -> Idle
      if (buttonPressed()) {
        // do things
      }
      break;
    }
    
  }
}

// Xavier
bool buttonPressed() {
  return false;
}

// Xavier
void updateRobotAppearance() {
  switch (currentState) {
    case IDLE_: {
      // TODO set LED colors to OFF
      // TODO set Buzzer to OFF
      break;
    }
      // Invetigating
      // TODO set LED colors to Orange
      // TODO set Buzzer to off or low (test both see what's nice)
      // Hostile
      // TODO set LED to red
      // TODO set Buzzer to "terminator"
      // Friendly
      // TODO set LED to Green
      // TODO set Buzzer to "Happy" somehow? Otherwise off
  }
  // Use value of "currentState" in another switch
  return;
}

//void moveTowardsTarget(int IRValue) {
//  // and the function would look a bit like with for a motor driver that uses servo control
//  leftMotor.write(map(IRValue, IRValuemin, IRValemax, Servocontrolmin, Servocontrolmaz)); //map you can move the wheel in certain angles and degrees
//  rightMotor.write(map(IRValue, IRValuemin, IRValemax, Servocontrolmin, Servocontrolmaz)) //based on the ir value perceived
//}

int lookL(){
  servo1.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo1.write(115);
  return distance;
}

int lookR(){
  servo1.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo1.write(115);
  return distance;
}

int readPing(){
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0){
    cm = 250;
  }
  return cm;
}

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
