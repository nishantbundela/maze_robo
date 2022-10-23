#include <Pixy2.h>
#include <math.h>
#define STOP 0              // No rotation
#define FORWARD 1           // Go forward
#define BACKWARD 2          // Go backward
#define TURNRIGHT 3         // Turn right
#define TURNLEFT 4          // Turn left
#define TURNAROUND 5        // Turn around

int const turnArea = 10000;
int const sigRight = 5;       // signature for a right turn
int const sigLeft = 1;        // signature for a left turn
int const sigAround = 3;      // signature for turning around
int const minDistance = 5;    // minimum distance between the pixy and the object
int const hallWidth = 30;     // Width of the hall way
float const irThreshold = 2.1;// Voltage across the IR sensor when the robot is going along the center

// Define pins
int Enable1 = 9;                    // Digital Pin 9: H-bridge Enable 1 Left DC MOTOR
int Enable2 = 3;                    // Digital Pin 9: H-bridge Enable 2 Right DC MOTOR
int Input1 = 8;                     // Digital Pin 8: H-bridge Input 1 Left DC Motor
int Input2 = 7;                     // Digital Pin 7: H-bridge Input 2 Left DC Motor
int Input3 = 2;                   // Digital Pin  : H-bridge Input 3 Right DC Motor
int Input4 = 4;                    // Digital Pin  : H-bridge Input 4 Right DC Motor
int const IRleft = A0;              // Analog Pin A0: Left IR sensor
int const IRright = A1;             // Analog Pin A1: Right IR sensor

// This is the main Pixy object
Pixy2 pixy;

uint8_t state_machine_state;    // Create global state variable

void setup() {
  Serial.begin(115200);

  // Set up pin mode
  pinMode(IRleft, INPUT);
  pinMode(IRright, INPUT);

  pixy.init();

  // Set initial state of the state machine to NROTATE
  state_machine_state = STOP;
}

void loop() {
  int i;


  switch (state_machine_state) {
    case STOP:
      // analogWrite(Enable1, 0);
      Serial.print("Current state is ");
      Serial.println(state_machine_state);
      delay(5000);
      state_machine_state = FORWARD;
      break;
    case FORWARD:
      Serial.print("Current state is ");
      Serial.println(state_machine_state);

      // grab blocks!
      pixy.ccc.getBlocks();
      if (!pixy.ccc.numBlocks) {
        state_machine_state = FORWARD;
      }
      else
      {
        int width = pixy.ccc.blocks[i].m_width;
        int height = pixy.ccc.blocks[i].m_height;
        int area = width * height;
        // Check the object distance
        float distance = objDistance(width, height);
        Serial.println(width * height);
        if (area >= 21000 && area <= 23000) {
          Serial.println("working");
          if (pixy.ccc.blocks[i].m_signature == sigRight) {
            // Check turning signiture
            state_machine_state = TURNRIGHT;
          }
          else if (pixy.ccc.blocks[i].m_signature == sigLeft) {
            state_machine_state = TURNLEFT;
          }
          else {
            state_machine_state = TURNAROUND;
          }
        }
        else {
          state_machine_state = FORWARD;
        }
      }
      // Move both motor at full speed
      // analogWrite(Enable1, 255);

      // Check IR sensor reading
      float vLeft = irVoltage(IRleft);
      float vRight = irVoltage(IRright);
      if (vLeft - irThreshold > 0.2 && vRight - irThreshold < -0.2)   // rotate to the left
      {
        // Need to Turn right
        analogWrite(Enable1, 147);
        analogWrite(Enable2, 107);
        delay(500);
      }
      else if (vLeft - irThreshold < -0.2 && vRight - irThreshold > -0.2) // rotate to the right
      {
        // Need to Turn left
        analogWrite(Enable1, 107);
        analogWrite(Enable2, 177);
        delay(500);
      }
      else if ()

      break;
    case TURNRIGHT:
      Serial.println("Turning right by 90 deg.");  
      digitalWrite(Input3, LOW);  // check polarity to turn right
      digitalWrite(Input4, HIGH)
      analogWrite(Enable1, 255);
      analogWrite(Enable2, 255);

      delay(1000);        // make sure the robot finish the right turn
      digitalWrite(Input3, HIGH); // resetting motor direction
      digitalWrite(Input4, LOW);
      state_machine_state = FORWARD;
      break;
    case TURNLEFT:
      Serial.println("Turning left by 90 deg.");
      // turning left
      state_machine_state = FORWARD;
      digitalWrite(Input1, LOW);  // check polarity to turn left
      digitalWrite(Input2, HIGH);
      analogWrite(Enable1, 255);
      analogWrite(Enable2, 255);

      delay(1000);        // make sure the robot finish the right turn
      
      digitalWrite(Input1, HIGH);
      digitalWrite(Input2, LOW);
      state_machine_state = FORWARD;
      break;
    case BACKWARD:
      Serial.println("Moving backward by 2 cm.");
      // moving back
      // Setting both motors to reverse
      digitalWrite(Input1, LOW);
      digitalWrite(Input2, HIGH);
      digitalWrite(Input3, LOW);
      digitalWrite(Input4, HIGH);

      analogWrite(Enable1, 255);
      analogWrite(Enable2, 255);

      delay(1000);

      // Resetting motors to forward
      digitalWrite(Input1, HIGH);
      digitalWrite(Input2, LOW);
      digitalWrite(Input3, HIGH);
      digitalWrite(Input4, LOW);
      
      state_machine_state = FORWARD;
      break;
    case TURNAROUND:
      Serial.println("Turning around.");
      // turn around 
      digitalWrite(Input3, LOW);  // check polarity to turn right
      digitalWrite(Input4, HIGH)
      analogWrite(Enable1, 255);
      analogWrite(Enable2, 255);

      delay(2000);        // make sure the robot finish the right turn
      digitalWrite(input3, HIGH);
      digitalWrite(input4, LOW);
      
      state_machine_state = FORWARD;
      break;
  }
  delay(1000);
}

float objDistance(int width, int height) {
  float obj_width = 5;          // Object's real width in cm
  float obj_height = 5;         // Object's real height in cm

  // calculate the focal length by using width
  float fl_temp1 = (float) obj_width / width;
  // calculate the focal length by using height
  float fl_temp2 = (float) obj_height / height;
  // find the mean value of the two focal lengths above
  float focalLength = 0.5 * (fl_temp1 + fl_temp2);
  // Calculate the object distance by using the equation of
  // the best fit line in the lab report
  float distance = (focalLength + 0.0653) / 0.0553;
  return distance;
}

float irVoltage(int IR_pin) {
  float volts = analogRead(IR_pin) * 0.0048828125;
  //  float d_inv = (volts * 0.0487 - 0.0087);                  // 1/distance
  //  float d = 1 / d_inv;
  return volts;
}
