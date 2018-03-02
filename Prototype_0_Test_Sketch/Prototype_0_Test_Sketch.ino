#include <Servo.h>

#define SOF 'h'
#define EOF 'p'
#define M_PI 3.14159265358979323846264338327950288



Servo testServo;

int torque = 0;
int angle = 2;

unsigned long currentTime = millis();
unsigned long previousTime = currentTime;
float thing = 0;
byte samplePeriod = 10;
byte f = 1;
byte offset = 0;
byte amplitude = 4;



void setup()
{
  Serial.begin(115200);

  testServo.attach(10);
}



void loop()
{
  currentTime = millis();
  
  if ((currentTime - previousTime) >= samplePeriod)
  {
    previousTime = currentTime;

    thing = offset + amplitude * sin(2*M_PI*f*millis()/1000); //in degrees

    testServo.write(thing);

    torque = analogRead(0);
    angle = analogRead(2);

    Serial.write(SOF);
    Serial.write((byte)((torque >> 8) & 0xFF));
    Serial.write((byte)(torque & 0xFF));
    Serial.write((byte)((angle >> 8) & 0xFF));
    Serial.write((byte)(angle & 0xFF));
    Serial.write(EOF);
  }
}






