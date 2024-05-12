#include <Servo.h>

#define SHOULDER 5.0
#define ELBOW 5.0

float ws_coord[3];
float d = 0;
float angleA = 0;
float angleC = 0;
float angleB = 0;
float angleD = 0;
float b1 = 0;
float b2 = 0;

Servo base;
Servo shoulder;
Servo elbow;
Servo grip;

int bpin = 3;
int spin = 5;
int epin = 6;
int gpin = 9;

#define PI 3.14159265

void resetServo(){

  base.write(90);
  shoulder.write(90);
  elbow.write(180);
  grip.write(30);
}

void calcIK()
{
  
  
  Serial.println("Enter 2 coords separated by spaces: ");
  while (Serial.available() == 0) {
    // Wait for input
  }

  // Read all four angles from serial input
  for (int i = 0; i < 3; ++i) {
    ws_coord[i] = Serial.parseInt();
    // Skip over any non-numeric characters
    while (Serial.peek() == ' ' || Serial.peek() == '\n' || Serial.peek() == '\r') {
      Serial.read();
    }
  }
  
  d = sqrt(sq(ws_coord[0]) + sq(ws_coord[1]));

  angleC = acos((sq(SHOULDER) + sq(ELBOW) - sq(d))/ (2 * SHOULDER * ELBOW))*(180/PI);

  b1 = atan(ws_coord[1]/ws_coord[0])*(180/PI);
  b2 = (180-angleC)/2;
  angleB = b1+b2;
  angleA = atan(ws_coord[2]/ws_coord[0])*(PI/180);
}

void setup()
{
  Serial.begin(9600);
//  elbow.setPeriodHertz(50);
//  shoulder.setPeriodHertz(50);    
  base.attach(bpin);
  shoulder.attach(spin);
  elbow.attach(epin);
  grip.attach(gpin);

  resetServo();
}

void loop()
{
  calcIK();
  //z = atan(2)*(180/PI);
//  Serial.println(z);
//  delay(10);
  Serial.print("Base angle: ");
  Serial.println(angleA);
  delay(100);
  Serial.print("Shoulder angle: ");
  Serial.println(angleB);
  delay(100);
  Serial.print("elbow angle: ");
  Serial.println(angleC);
  delay(100);
  Serial.print("Gripper angle: ");
  Serial.println(angleD);
  delay(100);

  base.write(angleA);
  shoulder.write(angleB);
  elbow.write(angleC);
  grip.write(angleD);
}
