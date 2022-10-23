# maze_robo

Lab 3: State Machines and Labyrinth

Final Design - Chassis

	The mechanical design was done in Solidworks in order to ensure the final package would be within the required footprint and to make sure there was adequate space for all necessary components and fasteners. In order to make turning in the tight maze as easy as possible, the overall size of the robot was minimized. The two wheels were mounted coaxially at the rear and the caster wheel was mounted at the front of the chassis. The PING sensors were mounted at 45 degree angles facing forwards and the PING sensor was mounted at the front facing forwards. The Arduino UNO and breadboard shield were at the rear of the chassis and the batteries underneath. The Pixy camera was mounted at the front of the chassis so that the navigation cues were close to centered within the view of the camera. 

Figure 1: Isometric view of chassis with component callouts

Figure 2: Side view of chassis with component callouts

	The electrical connections to the DC motors were soldered and then taped for strain relief. The stranded wires from the sensors were crimped to pins so they could be used in the breadboard. All of the motors and sensors were mounted using aluminum brackets with the acrylic chassis base being laser cut. The batteries were wired such that one was being used purely for the Arduino input power and the other for the motor power (only into the H-bridge). 
	The final overall dimensions for the robot were 13.5cm wide x 16.06cm long x 10.14cm tall, which is within the 20cm wide x 20cm tall maximum dimensions. 


Figure 3: Projected views of mechanical design with overall dimensions called out


Figure 4: Final robot



Power Budget

Table 1: Power data and calculations for robot components


The table above summarizes the worst-case current draws and the worst-case voltages that the robot components were operating at. It assumes that the batteries were providing their full voltage (9V) the entire time and that the Arduino was providing 5V. In reality, the voltages were likely lower than this. Additionally, the current data (see sources below) all assume the max possible current draw for each component is sustained for the duration of the robot’s use. 
The total power consumption for all of the robot components is about 5900mW. The two 9V batteries used can provide 550mAh each (see reference below), totalling 9900mWh. Using these assumptions, This means that the two 9V batteries should be able to power the robot for a total of 1.68 hours. 
The maze itself was completed in under a minute, and because all of the above assumptions made in the power budget were conservative, the robot would have likely been able to run for a significantly longer time. 











PING sensor data source: https://www.mouser.com/datasheet/2/321/28015-PING-Sensor-Product-Guide-v2.0- 461050.pdf
Sharp IR sensor data source: https://www.sparkfun.com/datasheets/Sensors/Infrared/gp2y0a02yk_e.pdf
DC motor data source: https://www.amazon.com/1pc-Plastic-Gear-Motor-3V-6V/dp/B07C67LKLP
Arduino UNO data source: https://diyi0t.com/arduino-uno-tutorial/
9V battery data source: https://www.microbattery.com/blog/post/battery-bios:-everything-you-need-to-know-about- th e-9v-battery/
Final Design - Software & Logic
We programmed our robot according to the design requirements listed below. 
The robot shall be able to perform the following maneuvers
Going forward;
Turn Left/Right by 90 degrees;
Turn Around by 180 degrees;
Going backward;
The robot shall be able to recognize cues in the labyrinth and perform corresponding motion.
To achieve the above requirements, we used state machines. Our state machine diagram is shown below. Also, conditions of changing states are listed in Table 1.

States
Entry Condition
Exit Condition
STOP
N/A
After 5 sec delay
FORWARD
Condition 2:
d’ = d (from the left IR)
Condition 4:
d’ = d (from the right IR)
Condition 5:
After 3450 ms delay
Condition 7:
After 4 s delay
Condition 9:
PING reading ≥ 15 cm
Condition 1:
Front distance < 15 cm
Detect turn right signature
Condition 3:
Front distance < 15 cm
Detect turn left signature
Condition 6:
Front distance < 15 cm
Detect turn around signature
Condition 8:
Front distance < 5 cm
TURNRIGHT
Condition 1
Condition 2
TURNLEFT
Condition 3
Condition 4
TURNAROUND
Condition 6
Condition 5
BACKWARDS
Condition 8
Condition 7



State: STOP
After the initialization, we entered a STOP state. At the same time, the robot is placed at the entry of the maze, and the reference distances from both left and right walls are recorded. This state lasts about 5 second, and then it will automatically go to the FORWARD state.

State: FORWARD
This is our main state, where we keep checking the readings from our PING and IR sensors and moving the robot forward. The PING sensor measures the distance between the robot and the front wall. Once the distance is less than 15 cm, we stop the robot by 2 seconds and let the pixy read the cue. Otherwise, we will keep moving our robot forward. We implemented 2 IR sensors and angled at about 45 degrees to the left and right walls. In the FORWARD stage, their readings are used to adjust the speed of our DC motors such that the robot can keep moving along the centerline of the hallway. The detailed algorithm for this part will be introduced later.

State: TURNLEFT/TURNRIGHT
The conditions to enter the TURNLEFT/TURNRIGHT states are
Condition 1 (FORWARD → TURNRIGHT)
Distance to the front wall is less than 15 cm;
Pixy detects a right turn signature (in our case, a green sticky).
Condition 3 (FORWARD → TURNLEFT)
Distance to the front wall is less than 15 cm;
Pixy detects a left turn signature (in our case, an orange sticky).
The logic for turning left by 90 degrees is as following:
Condition 3 must be satisfied;
Using the reading from the right IR sensor as a reference. Here d, the wall distance, measured by the right IR sensor is the one we record during the STOP state;
Rotate the right and the left motors at the speed speed but with the opposite directions;
Keep rotating the robot until the stop condition, Condition 4 (i.e. d’ = d), is achieved;
Stop the motor for 2 sec and then change the state back to FORWARD.
The logic for turning right by 90 degrees is similar to the above with the following exceptions:
Instead of Condition 3, in this case Condition 1 must be satisfied;
The left IR sensor is used to monitor the wall distance.
The stop condition, Condition 2, is d’=d, where d and d’ are measured by the left IR sensor.

State: TURNAROUND
Similar to TURNLEFT/TURNRIGHT states, the condition to enter this state is:
Condition 7:
Distance to the front wall is less than 15 cm;
Pixy detects a turn around signature (in our case, a purple sticky).
The logic to turn the robot 180 degrees is very simple. We directly measured the time required to turn the robot 180 degrees, and let the DC motors rotate this amount of time. The main drawback for this method is the time required to turn the robot around is dependent on the remaining battery power. The more power the battery remains, the shorter the time it takes.

After the robot finishes the turn, it will stop for 2 sec and then go back to the FORWARD state automatically.

State: BACKWARDS
This state is used as a precaution method to prevent the robot hitting the wall while moving forward. The logic is simple. If the reading from our PING sensor was less than 5 cm, the robot would stop and then both motors would rotate backwards for 2 seconds. Then, the robot would go back to the FORWARD state automatically.

Auto-adjust Algorithm
When the robot moves forward, we expect our robot to auto-adjust itself so that it can keep a reasonable distance from side walls. To achieve this, we need to consider several different cases.
Both left and right walls exist. In this case, we monitor the differences between the real time readings from both IR sensors and their reference values that were recorded during the STOP state. Then, we keep the difference inside a range. Once it is outside the range, we adjust our robot’s moving direction by slightly varying both DC motors’ speed.
No right or left wall.
The same logic applies here. Instead of using both IR sensors’ readings, we only monitor one IR sensor to adjust the course. For example, if no right wall exists, we will use the left IR sensor.
No side walls.
In this case, we will not adjust the orientation. In other words, rotate both DC motors at the same speed. 


Discussion
Our final prototype was able to complete all of the required objectives. It was able to recognize colored markers and turn according to their pre-set meanings, it was also able to self-correct its motion between two walls in order to move in a straight line without ever colliding into a wall. Our turning algorithms enabled our robot to successfully make 90 and 180 degree turns even when the battery’s voltage varied. The only situation our robot struggled was in low light conditions which was the requirement to earn extra credits. Even in these low light conditions, our robot was able to navigate through more than half of the maze.
	The reason why our robot was able to do so was because of our robust algorithms that utilized all of the available sensors to accurately figure out what state our robot was in. We used infrared sensors to make sure our robot moved in a straight line and is always at a certain distance from both walls. In situations where there was no wall on one side, we programmed it to track and move along the available wall. We used ultrasonic/ping sensors to make the robot stop 15 cms before a turn. During turning we used the infrared sensor facing the long edge of the wall to record a reference value before turning and then keep turning till the sensor reading becomes equal to that again. For a 180 degree turn, we executed the first half of the turn using the same algorithm of a 90 degree and recorded the time it took to do that. After completing the first half we made the robot turn for the recorded amount of time leading to a 90 + 90 or 180-degree turn.



Lessons Learned:
This project was especially tricky for us and we did struggle a lot. The biggest lessons that we learnt were:
We have to create algorithms that can sense the robots current state and dynamically react to them. We cannot program the robot with static or time based algorithms because the voltage across the batteries falls under use and causes incorrect movements.
We also learnt how to debug code in conjunction with the physical sensors and actuators controlled by it. This was significantly different compared to debugging software algorithms.
We had to check our wiring and components multiple times to make sure each one of them was working. We did face issues with malfunctioning H-bridges and Arduino pins. This gave us a lot of experience dealing with physical parts and we will be much efficient at that in future projects. In all honesty this was a massively frustrating issue and it taught us how to be patient and not get frustrated. 
We learnt how to calibrate the Pixy2 camera’s object recognition system to allow for flexibility in adverse situations.
We also learnt how to create algorithms for individual actions and them integrate them together to successfully execute complex tasks.
We learnt how to deal with edge cases and repeatedly test for them.

Appendix: Arduino Code
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


