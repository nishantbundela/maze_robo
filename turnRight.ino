// Define pins
int Enable1 = 3;                    // Digital Pin 3: H-bridge Enable 1 Left DC MOTOR
int Enable2 = 9;                    // Digital Pin 9: H-bridge Enable 2 Right DC MOTOR
int Input1 = 4;                     // Digital Pin 2: H-bridge Input 1 Left DC Motor
int Input2 = 6;                     // Digital Pin 6: H-bridge Input 2 Left DC Motor
int Input3 = 8;                     // Digital Pin 8: H-bridge Input 3 Right DC Motor
int Input4 = 5;                     // Digital Pin 5: H-bridge Input 4 Right DC Motor
int const IRleft = A5;               // Digital Pin 0: Left IR sensor
int const IRright = A4;              // Ditigal Pin 1: Right IR sensor

// Define global variables
unsigned long startTime = 0;
unsigned long endTime = 0;
int IRleft_initial = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(IRleft, INPUT);
  pinMode(IRright, INPUT);
  IRleft_initial = analogRead(IRleft);
  Serial.println(IRleft_initial);
}

void loop() {
  // Prepare turn right
  Serial.println("Turn Right");
  delay(3000);
  analogWrite(Enable1, 0);
  analogWrite(Enable2, 0);
  digitalWrite(Input1, HIGH);
  digitalWrite(Input2, LOW);
  digitalWrite(Input3, HIGH);
  digitalWrite(Input4, LOW);
  delay(500);
  while (1 < 2) {
    digitalWrite(Input1, HIGH);
    digitalWrite(Input2, LOW);
    digitalWrite(Input3, HIGH);
    digitalWrite(Input4, LOW);
    delay(10);
    //    Serial.println("---------------------");
    //    Serial.println(analogRead(IRright));
    //        if (abs(analogRead(IRleft) - IRleft_initial) <= 25 || IRleft_initial - analogRead(IRleft) > 30) {
    if (IRleft_initial - analogRead(IRleft) > 20) {
      break;
    }
  }
  analogWrite(Enable1, 0);
  analogWrite(Enable2, 0);
  delay(20000);
}
