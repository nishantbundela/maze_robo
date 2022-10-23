// Define pins
int Enable1 = 3;                    // Digital Pin 3: H-bridge Enable 1 Left DC MOTOR
int Enable2 = 9;                    // Digital Pin 9: H-bridge Enable 2 Right DC MOTOR
int Input1 = 4;                     // Digital Pin 2: H-bridge Input 1 Left DC Motor
int Input2 = 6;                     // Digital Pin 6: H-bridge Input 2 Left DC Motor
int Input3 = 8;                     // Digital Pin 8: H-bridge Input 3 Right DC Motor
int Input4 = 5;                     // Digital Pin 5: H-bridge Input 4 Right DC Motor
int const IRleft = A5;               // Digital Pin 0: Left IR sensor
int const IRright = A4;              // Ditigal Pin 1: Right IR sensor
int rightWall_initial = 0;
int leftWall_initial = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(IRleft, INPUT);
  pinMode(IRright, INPUT);
  Serial.println("Calibrating...");
  delay(3000);
  rightWall_initial = analogRead(IRright);
  leftWall_initial = analogRead(IRleft);

}

void loop() {
  Serial.println("--------------------------------------");
  Serial.println(analogRead(IRleft));
  Serial.println(analogRead(IRright));
  // put your main code here, to run repeatedly:
  if (analogRead(IRleft) - leftWall_initial > 30) {
    // turn right a bit
    analogWrite(Enable1, 147);
    analogWrite(Enable2, 107);
    digitalWrite(Input1, HIGH);
    digitalWrite(Input2, LOW);
    digitalWrite(Input3, LOW);
    digitalWrite(Input4, HIGH);
    delay(50);
  }
  else if (analogRead(IRright) - rightWall_initial > 30) {
    // turn left a bit
    analogWrite(Enable1, 107);
    analogWrite(Enable2, 147);
    digitalWrite(Input1, HIGH);
    digitalWrite(Input2, LOW);
    digitalWrite(Input3, LOW);
    digitalWrite(Input4, HIGH);
    delay(50);
  }
  else if (analogRead(IRright) < 200) {
    // No right wall, stick with the left sensor
    if (analogRead(IRleft) - leftWall_initial < -30) {
      // Tilt to the right, turn left a bit
      analogWrite(Enable1, 107);
      analogWrite(Enable2, 147);
      digitalWrite(Input1, HIGH);
      digitalWrite(Input2, LOW);
      digitalWrite(Input3, LOW);
      digitalWrite(Input4, HIGH);
      delay(50);
    }
  }
  else if (analogRead(IRleft) < 200) {
    // No left wall, stick with the right sensor
    if (analogRead(IRright) - rightWall_initial < -30) {
      // Tilt to the left, turn right a bit
      analogWrite(Enable1, 147);
      analogWrite(Enable2, 107);
      digitalWrite(Input1, HIGH);
      digitalWrite(Input2, LOW);
      digitalWrite(Input3, LOW);
      digitalWrite(Input4, HIGH);
      delay(50);
    }
  }
  else {
    // go straight
    analogWrite(Enable1, 127);
    analogWrite(Enable2, 127);
    digitalWrite(Input1, HIGH);
    digitalWrite(Input2, LOW);
    digitalWrite(Input3, LOW);
    digitalWrite(Input4, HIGH);
  }
//  delay(500);
}
