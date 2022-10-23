#include <Pixy2.h>
#include <math.h>

// Define pins
int Enable1 = 3;                    // Digital Pin 3: H-bridge Enable 1 Left DC MOTOR
int Enable2 = 9;                    // Digital Pin 9: H-bridge Enable 2 Right DC MOTOR
int Input1 = 2;                     // Digital Pin 2: H-bridge Input 1 Left DC Motor
int Input2 = 6;                     // Digital Pin 6: H-bridge Input 2 Left DC Motor
int Input3 = 8;                     // Digital Pin 8: H-bridge Input 3 Right DC Motor
int Input4 = 5;                     // Digital Pin 5: H-bridge Input 4 Right DC Motor
int const IRleft = A5;               // Digital Pin 0: Left IR sensor
int const IRright = A4;              // Ditigal Pin 1: Right IR sensor

// This is the main Pixy object
Pixy2 pixy;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pixy.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  int i;
  // grab blocks!
  pixy.ccc.getBlocks();
  int width = pixy.ccc.blocks[i].m_width;
  int height = pixy.ccc.blocks[i].m_height;
  float pixy_dist = objDistance(width, height);
  float irr_dist = irDist(IRright);
  //  float irl_dist = irDist(IRleft);
  double angle = acos(pixy_dist / irr_dist);
  Serial.print("Pixy distance = ");
  Serial.print(pixy_dist);
  Serial.print("; IR sensor distance = ");
  Serial.println(irr_dist);
  Serial.print("Angle is ");
  Serial.print(angle);
  Serial.println(" rads.");
  delay(500);
}

float objDistance(int width, int height) {
  float obj_width = 7.5;          // Object's real width in cm
  float obj_height = 7.5;         // Object's real height in cm

  // calculate the focal length by using width
  float fl_temp1 = (float) obj_width / width;
  // calculate the focal length by using height
  float fl_temp2 = (float) obj_height / height;
  // find the mean value of the two focal lengths above
  float focalLength = 0.5 * (fl_temp1 + fl_temp2);
  // Calculate the object distance by using the equation of
  // the best fit line in the lab report
  float distance = 216.17 * focalLength + 0.3195;

  return distance;
}

float irDist(int IR_pin) {
  float volts = analogRead(IR_pin) * 0.0048828125;
  float dist_inv = (volts * 0.0421 - 0.0016);
  float dist = 1 / dist_inv;
  return dist;
}
