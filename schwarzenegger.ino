#include<Servo.h>    // standard library for servo motor
#include<HCSR04.h> // Sketch -> Include Library > Add .ZIP Library > Select the HCSR04_vX.X.X.zip file in the repository folder.

#define ir_right A2
#define ir_left A3

// Motor control
const int MOTOR1_PIN1 = 3;  // Connect this to one terminal of Motor1
const int MOTOR1_PIN2 = 4;  // Connect this to the other terminal of Motor1
const int MOTOR2_PIN1 = 5;  // Connect this to one terminal of Motor2
const int MOTOR2_PIN2 = 6;  // Connect this to the other terminal of Motor2
const int msDelayPer360Turn = 500; // TODO: Test this

// LED and Buzzer 
const int buzzer = 10;
const int LEDorange = 7;
const int LEDred = 8;
const int LEDgreen = 9;

// Touch Sensor 
const int touchSensor = 11;
int touchCount = 0;
bool wasTouched = false;
unsigned long lastTouchTime = 0;
const unsigned long doubleTapWindow = 500; // user has x ms to triple touch

// Sonar & Servo
#define debug true
#define servo_pin 2
#define trig_pin A1
#define echo_pin A0
const int sonarMaxDistance = 250;
const float distanceChangeThreshold = 0.7; // Based on testing, seems to be high enough to prevent accidental triggers with the innacuracy of the sensor
const int servoTotalAngle = 180; // DO NOT CHANGE THIS IT WILL BREAK THE CHASING!
const int servoDirections = 15; // make sure servoTotalAngle/servoDirections is a whole number, and that is an odd number
const int servoDirectionMiddle = floor(servoDirections / 2);
const int servoAngleStepSize = servoTotalAngle / servoDirections;
const int servoChangeAngleDelay = 200;
const int msToAuthenticateWhenInvestigating = 5000;
const int msUntilIdleWhenInvestigating = 20000;
const bool servoSwivel = true; // if false, rotates
int servoDirectionNumber = 0;
HCSR04 sonar = HCSR04(trig_pin, echo_pin); // sensor function
Servo servo = Servo();
bool servoTurnsRight = true;
int sonarDistances[servoDirections];
int investigatingStartTime = 0;

// Robot state
enum State {IDLE_, INVESTIGATING, HOSTILE, FRIENDLY};
int currentState = IDLE_;

void printlnWithState(String message) {
  Serial.print(currentState);
  Serial.print(" | ");
  Serial.println(message);
}

void setup() {
  Serial.begin(9600);
  
  pinMode(ir_right, INPUT);
  pinMode(ir_left, INPUT);

  // Set motor control pins as outputs
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);

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
  pinMode(LED_BUILTIN, OUTPUT);

  updateRobotState(IDLE_);

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
          newServoDirectionNumber = servoDirections - 1; // rotate around to end
        } else if (newServoDirectionNumber >= servoDirections) {
          newServoDirectionNumber = 0;
        }
      }

      // rotate and measure distance
      setServo(newServoDirectionNumber * servoAngleStepSize); // Set direction; this updates servoDirection
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
        
        // rotate robot towards found target
        int turnSteps = servoDirectionNumber - servoDirectionMiddle;
        int turnDistance = abs(turnSteps) * servoAngleStepSize;
        if (turnSteps < 0) {
          turnL();
        } else if (turnSteps > 0) {
          turnR();
        }
        delay(float(msDelayPer360Turn) / 360.0 * abs(turnSteps));
        moveStop();
      }
      break;
    }
    case INVESTIGATING: {
      /// ACTION ///
      // Sjoerd
      // Drive in direction of potential intruder to pick up heat signature. Avoid obstacles where possible
      if (sonar.dist() > 20) {
        moveForward();
      } else {
        moveStop();
      }

      /// STATE SWITCH ///
      // Sjoerd
      // if heat signature detected (so there's a person) & they are not authenticated withing 5 seconds -> Hostile
      if (digitalRead(ir_right) == 1 || digitalRead(ir_left) == 1) {
        Serial.println("Detected IR signature");
        moveStop();
        int start = millis();
        while (millis() - start < msToAuthenticateWhenInvestigating) { // while current time is more than start time + 5000ms
          Serial.println(millis() - start);
          if (touchSensorPressed()) {
            updateRobotState(FRIENDLY);
            return;
          }
        }
        // failed to authenticate -> Hostile
        updateRobotState(HOSTILE);
      }
      
      // If user authenticated -> Friendly
      if (touchSensorPressed()) {
        updateRobotState(FRIENDLY);
      }
      
      // if nothing detected for 20 seconds, drive back to original location -> Idle. Avoid obstacles where possible
      if (millis() > investigatingStartTime + msUntilIdleWhenInvestigating) {
        moveStop();
        updateRobotState(IDLE_);
      }
      break;
    }
    case HOSTILE: { // Tom & Jen
      /// ACTION ///
      delay(50);
      unsigned int distance = sonar.dist();
      // Serial.print("distance");
      // Serial.println(distance);
      int Right_Value = digitalRead(ir_right);
      int Left_Value = digitalRead(ir_left);
      // Serial.print("RIGHT");
      // Serial.println(Right_Value);
      // Serial.print("LEFT");
      // Serial.println(Left_Value);
      if (debug) {
        String message = "Distance: ";
        message = message + distance;
        message = message + " RightIR: ";
        message = message + Right_Value;
        message = message + " LeftIR: ";
        message = message + Left_Value;
        printlnWithState(message);
      }

      // Check if object is detected within a certain distance and both sensors detect an object
      if (Right_Value == 1 && Left_Value == 1 && distance >= 10 && distance <= 30) {
        moveForward();
      } else if (Right_Value == 0 && Left_Value == 1) {
        turnR();
      } else if (Right_Value == 1 && Left_Value == 0) {
        turnL();
      } else {
        moveStop();
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
      break;
    }
    case FRIENDLY: { // Xavier + all the LEDs & Buzzers & Button in the idle mode
      /// ACTION ///
      // Robot stays in place
      moveStop();

      /// STATE SWITCH ///
      // if button is pressed again and then 10 seconds pass -> Idle
      if (touchSensorPressed()) {
        for (int i = 0; i < 5; i++) {
          updateRobotAppearance(INVESTIGATING);
          delay(1000);
          updateRobotAppearance(IDLE_);
          delay(1000);
        }
        updateRobotState(IDLE_);
      }
      break;
    }
  }
}

// Sjoerd
void setServo(int direction) {
  servoDirectionNumber = round(direction / servoAngleStepSize);
  servo.write(direction);
}

// Xavier
bool touchSensorPressed() {
  if (digitalRead(touchSensor) == HIGH) {
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
  currentState = state;
  updateRobotAppearance(state);
  switch (state) {
    case IDLE_: {
      Serial.println("IDLE_");

      // reset measurements
      for (int i = 0; i < servoDirections; i++) {
        sonarDistances[i] = 0;
      }

      // reset servo
      setServo(0);
      delay(servoChangeAngleDelay*2); // wait for servo to move
      break;
    }
    case INVESTIGATING: {
      Serial.println("INVESTIGATING");

      // reset timer
      investigatingStartTime = millis();
      break;
    }
    case HOSTILE: {
      Serial.println("HOSTILE");
      break;
    }
    case FRIENDLY: {
      Serial.println("FRIENDLY");
      break;
    }
    default: {
      Serial.println("UNKNOWN? ERROR?");
      break;
    }
  }
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
      digitalWrite(LED_BUILTIN, HIGH);
      // Set buzzer to OFF
      digitalWrite(buzzer, LOW);
      break;
    }
    case INVESTIGATING: {
      // Set LED to Orange
      digitalWrite(LEDorange, HIGH);
      digitalWrite(LEDred, LOW);
      digitalWrite(LEDgreen, LOW);
      digitalWrite(LED_BUILTIN, LOW);
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
      digitalWrite(LED_BUILTIN, LOW);
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
      digitalWrite(LED_BUILTIN, LOW);
      // Set Buzzer to "Happy"
      tone(buzzer, 300);
      delay(1000);
      noTone(buzzer);
      break;
    }
  }
  return;
}

void moveStop(){
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void moveForward(){
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void turnL(){
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void turnR(){
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH);
}
