#include <Pixy2.h>
#include <math.h>
#define STOP 0              // No rotation
#define FORWARD 1           // Go forward
#define BACKWARD 2          // Go backward
#define TURNRIGHT 3         // Turn right
#define TURNLEFT 4          // Turn left
#define TURNAROUND 5        // Turn around

// Define global variables
int IRleft_initial = 0;       // Reference reading from the left IR sensor
int IRright_initial = 0;      // Reference reading from the right IR sensor
int const sigRight = 2;       // signature for a right turn
int const sigLeft = 1;        // signature for a left turn
int const sigAround = 3;      // signature for turning around
unsigned long pulseduration = 0;
float pingDist;               // Distance to the front wall measured by the PING sensor

// Define pins
int Enable1 = 3;                    // Digital Pin 3: H-bridge Enable 1 Left DC MOTOR
int Enable2 = 9;                    // Digital Pin 9: H-bridge Enable 2 Right DC MOTOR
int Input1 = 4;                     // Digital Pin 4: H-bridge Input 1 Left DC Motor
int Input2 = 6;                     // Digital Pin 6: H-bridge Input 2 Left DC Motor
int Input3 = 8;                     // Digital Pin 8: H-bridge Input 3 Right DC Motor
int Input4 = 5;                     // Digital Pin 5: H-bridge Input 4 Right DC Motor
int const IRleft = A5;               // Digital Pin A5: Left IR sensor
int const IRright = A4;              // Ditigal Pin A4: Right IR sensor

int trigPin = 2;                    // Arduino to pin Trig of HC-SR04
int leftMotorSpeed = 80;
int rightMotorSpeed = 80;


// This is the main Pixy object
Pixy2 pixy;

uint8_t state_machine_state;    // Create global state variable

void setup() {
  Serial.begin(115200);
  // Set up pin modes
  pinMode(IRleft, INPUT);
  pinMode(IRright, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(Enable1, OUTPUT);
  pinMode(Enable2, OUTPUT);
  pinMode(Input1, OUTPUT);
  pinMode(Input2, OUTPUT);
  pinMode(Input3, OUTPUT);
  pinMode(Input4, OUTPUT);

  pixy.init();

  // Set initial state of the state machine to NROTATE
  state_machine_state = STOP;
}

void loop() {
  int i;
  switch (state_machine_state) {
    case STOP:
      Serial.print("Current state is ");
      Serial.println(state_machine_state);
      delay(3000);
      // Record the reference values for left and right IR sensors
      IRleft_initial = analogRead(IRleft);
      IRright_initial = analogRead(IRright);
      state_machine_state = FORWARD;
      break;
    case FORWARD:
      Serial.print("Current state is ");
      Serial.println(state_machine_state);

      // Algorithm for keeping the robot moving along the centerline of the hallway
      // Case 1: both left and right walls exist
      if (analogRead(IRleft) >= 180 && analogRead(IRright) >= 180) {
        if (analogRead(IRleft) - IRleft_initial > 20) {
          // Lean to the left, turn right a bit
          analogWrite(Enable1, leftMotorSpeed + 30);
          analogWrite(Enable2, rightMotorSpeed);
          digitalWrite(Input1, HIGH);
          digitalWrite(Input2, LOW);
          digitalWrite(Input3, LOW);
          digitalWrite(Input4, HIGH);
          delay(50);
        }
        else if (analogRead(IRright) - IRright_initial > 20) {
          // Lean to the right, turn left a bit
          analogWrite(Enable1, leftMotorSpeed);
          analogWrite(Enable2, rightMotorSpeed + 30);
          digitalWrite(Input1, HIGH);
          digitalWrite(Input2, LOW);
          digitalWrite(Input3, LOW);
          digitalWrite(Input4, HIGH);
          delay(50);
        }
        else {
          // Walking along the center line go straight
          analogWrite(Enable1, leftMotorSpeed);
          analogWrite(Enable2, rightMotorSpeed);
          digitalWrite(Input1, HIGH);
          digitalWrite(Input2, LOW);
          digitalWrite(Input3, LOW);
          digitalWrite(Input4, HIGH);
        }
      }
      else if (analogRead(IRright) < 180) {
        // Case 2: No right wall, stick with the left sensor
        if (analogRead(IRleft) - IRleft_initial < -20) {
          // Tilt to the right, turn left a bit
          analogWrite(Enable1, leftMotorSpeed);
          analogWrite(Enable2, rightMotorSpeed + 30);
          digitalWrite(Input1, HIGH);
          digitalWrite(Input2, LOW);
          digitalWrite(Input3, LOW);
          digitalWrite(Input4, HIGH);
          delay(50);
        }
        else if (analogRead(IRleft) - IRleft_initial > 20) {
          // Tilt to the left, turn right a bit
          analogWrite(Enable1, leftMotorSpeed + 30);
          analogWrite(Enable2, rightMotorSpeed);
          digitalWrite(Input1, HIGH);
          digitalWrite(Input2, LOW);
          digitalWrite(Input3, LOW);
          digitalWrite(Input4, HIGH);
          delay(50);
        }
        else {
          // Walking along the center line, go straight
          analogWrite(Enable1, leftMotorSpeed);
          analogWrite(Enable2, rightMotorSpeed);
          digitalWrite(Input1, HIGH);
          digitalWrite(Input2, LOW);
          digitalWrite(Input3, LOW);
          digitalWrite(Input4, HIGH);
        }
      }
      else if (analogRead(IRleft) < 180) {
        // Case 3: No left wall, stick with the right sensor
        if (analogRead(IRright) - IRright_initial < -20) {
          // Tilt to the left, turn right a bit
          analogWrite(Enable1, leftMotorSpeed + 30);
          analogWrite(Enable2, rightMotorSpeed);
          digitalWrite(Input1, HIGH);
          digitalWrite(Input2, LOW);
          digitalWrite(Input3, LOW);
          digitalWrite(Input4, HIGH);
          delay(50);
        }
        else if (analogRead(IRright) - IRright_initial > 20) {
          // Tilt to the right, turn left a bit
          analogWrite(Enable1, leftMotorSpeed);
          analogWrite(Enable2, rightMotorSpeed + 30);
          digitalWrite(Input1, HIGH);
          digitalWrite(Input2, LOW);
          digitalWrite(Input3, LOW);
          digitalWrite(Input4, HIGH);
          delay(50);
        }
        else {
          // Walking along the center line, go straight
          analogWrite(Enable1, leftMotorSpeed);
          analogWrite(Enable2, rightMotorSpeed);
          digitalWrite(Input1, HIGH);
          digitalWrite(Input2, LOW);
          digitalWrite(Input3, LOW);
          digitalWrite(Input4, HIGH);
        }
      }
      else {
        // go straight
        analogWrite(Enable1, leftMotorSpeed);
        analogWrite(Enable2, rightMotorSpeed);
        digitalWrite(Input1, HIGH);
        digitalWrite(Input2, LOW);
        digitalWrite(Input3, LOW);
        digitalWrite(Input4, HIGH);
      }

      // Check the distance between the robot and the front wall
      pingDist = pingResult();
      Serial.println(pingDist);
      delay(50);
      if (pingDist <= 17) {
        // Stop both motors for 2 sec
        analogWrite(Enable1, 0);
        analogWrite(Enable2, 0);
        digitalWrite(Input1, HIGH);
        digitalWrite(Input2, LOW);
        digitalWrite(Input3, HIGH);
        digitalWrite(Input4, LOW);
        delay(2000);

        // Get readings from our Pixy
        pixy.ccc.getBlocks();
        if (pixy.ccc.blocks[i].m_signature == sigRight) {
          // Check turning signiture
          state_machine_state = TURNRIGHT;
        }
        else if (pixy.ccc.blocks[i].m_signature == sigLeft) {
          state_machine_state = TURNLEFT;
        }
        else if (pixy.ccc.blocks[i].m_signature == sigAround) {
          state_machine_state = TURNAROUND;
        }
        else {
          state_machine_state = FORWARD;
        }
      }
      else if (pingDist <= 5)
      {
        state_machine_state = BACKWARD;
      }
      else
      {
        state_machine_state = FORWARD;
      }
      break;

    case BACKWARD:
      // Stop both motors for 2 sec
      analogWrite(Enable1, 0);
      analogWrite(Enable2, 0);
      digitalWrite(Input1, HIGH);
      digitalWrite(Input2, LOW);
      digitalWrite(Input3, HIGH);
      digitalWrite(Input4, LOW);
      delay(2000);

      // Moving Backward for 2 sec
      analogWrite(Enable1, leftMotorSpeed);
      analogWrite(Enable2, rightMotorSpeed);
      digitalWrite(Input1, LOW);
      digitalWrite(Input2, HIGH);
      digitalWrite(Input3, LOW);
      digitalWrite(Input4, HIGH);
      delay(2000);

      // Back to the FORWARD state
      state_machine_state = FORWARD;
      break;




    case TURNRIGHT:
      // Prepare turn right
      analogWrite(Enable1, leftMotorSpeed + 40);
      analogWrite(Enable2, rightMotorSpeed - 20);
      digitalWrite(Input1, HIGH);
      digitalWrite(Input2, LOW);
      digitalWrite(Input3, HIGH);
      digitalWrite(Input4, LOW);
      delay(350);
      while (1 < 2) {
        digitalWrite(Input1, HIGH);
        digitalWrite(Input2, LOW);
        digitalWrite(Input3, HIGH);
        digitalWrite(Input4, LOW);
        delay(20);
        // Keep turning until the stop condition is satisfied.
        if (abs(analogRead(IRleft) - IRleft_initial) <= 30 || IRleft_initial - analogRead(IRleft) > 20) {
          break;
        }
      }
      analogWrite(Enable1, 0);
      analogWrite(Enable2, 0);
      digitalWrite(Input1, HIGH);
      digitalWrite(Input2, LOW);
      digitalWrite(Input3, HIGH);
      digitalWrite(Input4, LOW);
      delay(2000);
      state_machine_state = FORWARD;
      break;










    case TURNLEFT:
      Serial.print("Current state is ");
      Serial.println(state_machine_state);
      Serial.println("Turn Left.");
      analogWrite(Enable1, leftMotorSpeed - 20);
      analogWrite(Enable2, rightMotorSpeed + 20);
      digitalWrite(Input1, LOW);
      digitalWrite(Input2, HIGH);
      digitalWrite(Input3, LOW);
      digitalWrite(Input4, HIGH);
      delay(350);
      while (1 < 2) {
        digitalWrite(Input1, LOW);
        digitalWrite(Input2, HIGH);
        digitalWrite(Input3, LOW);
        digitalWrite(Input4, HIGH);
        delay(10);
        // Keep turning until the stop condition is satisfied.
        if (abs(analogRead(IRright) - IRright_initial) <= 30 || IRright_initial - analogRead(IRright) > 20) {
          break;
        }
      }
      analogWrite(Enable1, 0);
      analogWrite(Enable2, 0);
      digitalWrite(Input1, HIGH);
      digitalWrite(Input2, LOW);
      digitalWrite(Input3, HIGH);
      digitalWrite(Input4, LOW);
      delay(2000);
      state_machine_state = FORWARD;
      break;






    case TURNAROUND:
      Serial.print("Current state is: ");
      Serial.println(state_machine_state);
      analogWrite(Enable1, leftMotorSpeed);
      analogWrite(Enable2, rightMotorSpeed);
      digitalWrite(Input1, HIGH);
      digitalWrite(Input2, LOW);
      digitalWrite(Input3, HIGH);
      digitalWrite(Input4, LOW);
      delay(1450);

      analogWrite(Enable1, 0);
      analogWrite(Enable2, 0);
      digitalWrite(Input1, HIGH);
      digitalWrite(Input2, LOW);
      digitalWrite(Input3, LOW);
      digitalWrite(Input4, HIGH);
      delay(2000);

      state_machine_state = FORWARD;
      break;

  }
}

float pingResult()
{
  // get the raw measurement data from Ping)))
  // set pin as output so we can send a pulse
  pinMode(trigPin, OUTPUT);
  // set output to LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  // now send the 5uS pulse out to activate Ping)))
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);

  // now we need to change the digital pin
  // to input to read the incoming pulse
  pinMode(trigPin, INPUT);

  // finally, measure the length of the incoming pulse
  pulseduration = pulseIn(trigPin, HIGH);
  //************************//
  // FILL IN YOUR CODES HERE
  float distance = 0.5 * pulseduration * 0.034;
  // END OF YOUR CODES
  //************************//
  return distance;
}
