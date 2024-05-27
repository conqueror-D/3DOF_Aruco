#include <Servo.h>
#include <WiFi.h>
#include <WiFiClient.h>

const char *ssid = "Jash";
const char *password = "atmc5679";

WiFiServer server(80);

#define SHOULDER 5.0
#define ELBOW 5.0

float ws_coord[4];
float d = 0;
float angleA = 90;  
float angleC = 180;
float angleB = 90;
float angleD = 90;
float b1 = 0;
float b2 = 0;
float x;
bool haveReceived = false;
bool shouldPick = false;

Servo base;
Servo shoulder;
Servo elbow;
Servo grip;

int bpin = 12;
int spin = 14;
int epin = 27;
int gpin = 26;

#define PI 3.14159265

void resetServo(){

  base.write(0);
  shoulder.write(90);
  elbow.write(180);
  grip.write(90);
}

void calcIK()
{
  
 WiFiClient client = server.available();
  if (!client) {
    return;
  }

  // Wait until the client sends some data
  while (!client.available()) {
    delay(1);
    resetServo();
  }

  // Read the request
 
  for (int i = 0; i <= 3; i++) {
    ws_coord[i] = client.parseInt();
    // Skip spaces between integers
    while (client.peek() == ' ') {
      client.read();
      haveReceived = true;
    }
  }
  
  // Send a response back to the client
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println(""); // do not forget this one
  client.println("<h1>ESP32 Response</h1>");
  client.println("<p>Integers received:</p>");
  client.print("<p>");
  

  d = sqrt(sq(ws_coord[0]) + sq(ws_coord[1]));
  

  angleA = atan(ws_coord[1]/ws_coord[0])*(180/PI);
  angleC = acos((sq(SHOULDER) + sq(ELBOW) - sq(d))/ (2 * SHOULDER * ELBOW))*(180/PI);

  angleB = (180-angleC)/2;
  
  if(d<10){
    Serial.println("recieve or not");
    Serial.print(haveReceived);
    //Jaw opens
    if(haveReceived){
      angleD = 60;
    }
    else
    {
      angleD = 90;
    }
    //when x and y coordinates are in first quadrant
    if(ws_coord[0]>=0 && ws_coord[1]>=0){
      //delay(ws_coord[i].toFloat()*(10.0/6.0));
      base.write(angleA);
      shoulder.write(angleB);
      elbow.write(angleC);
      grip.write(angleD);
    }
    //when x is negative and y  i.e second quadrant
    if(ws_coord[0]<0 && ws_coord[1]>=0){
      x = atan(ws_coord[1]/abs(ws_coord[0]))*(180/PI);
      angleA = 180 - x;
      base.write(angleA);
      shoulder.write(angleB);
      elbow.write(angleC);
      grip.write(angleD);
    }

    if(ws_coord[3] == 1){
       grip.write(90);
    }
  }

}

void printAngles(){
  
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
}

void printCoordinates(){

  Serial.print("Target at: ");
  Serial.println(ws_coord[0]);
  Serial.print(" ");
  Serial.println(ws_coord[1]);
  Serial.print(" ");
  Serial.println(ws_coord[2]);  
}

  void readAngles() {
    Serial.println(base.read());
    Serial.println(shoulder.read());
    Serial.println(elbow.read());
    Serial.println(grip.read());

  }

void setup()
{
  
  Serial.begin(115200);
//  elbow.setPeriodHertz(50);
//  shoulder.setPeriodHertz(50);    
  base.attach(bpin);
  shoulder.attach(spin);
  elbow.attach(epin);
  grip.attach(gpin);

  resetServo();

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");

  // Start the server
  server.begin();
  Serial.println("Server started");

  // Print the IP address
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop()
{
    calcIK();
    printAngles();
    printCoordinates();
    readAngles();
}
