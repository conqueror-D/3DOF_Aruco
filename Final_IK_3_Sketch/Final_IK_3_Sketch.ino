#include <Servo.h>

#define SHOULDER 5.0
#define ELBOW 5.0

float ws_coord[2];
float d = 0;
float angleC = 0;
float angleB = 0;
float b1 = 0;
float b2 = 0;
float z;
Servo shoulder;
Servo elbow;

int spin = 3;
int epin = 5;

#define PI 3.14159265
void calcIK()
{
  
  
  Serial.println("Enter 2 coords separated by spaces: ");
  while (Serial.available() == 0) {
    // Wait for input
  }

  // Read all four angles from serial input
  for (int i = 0; i < 2; ++i) {
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
}

void setup()
{
  Serial.begin(9600);
//  elbow.setPeriodHertz(50);
//  shoulder.setPeriodHertz(50);    
  elbow.attach(epin);
  shoulder.attach(spin);

  elbow.write(90);
  shoulder.write(90);
}

void loop()
{
  calcIK();
  //z = atan(2)*(180/PI);
//  Serial.println(z);
//  delay(10);
  Serial.println(angleC);
  delay(100);
  Serial.println(angleB);
  delay(100);
  shoulder.write(angleB);
  elbow.write(angleC);
}
