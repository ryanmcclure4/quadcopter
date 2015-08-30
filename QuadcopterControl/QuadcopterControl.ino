#include <Servo.h>
//#include <SoftwareSerial.h>

Servo m1, m2, m3, m4;

int val = 800;
int pos = 800;
String message = "";

void setup()
{
  //Setup usb serial connection to computer
  Serial.begin(115200);
  
  m1.attach(6);
  m2.attach(9);
  m3.attach(10);
  m4.attach(11);
}


void loop()
{
   m1.writeMicroseconds(pos);
   m2.writeMicroseconds(pos);
   m3.writeMicroseconds(pos);
   m4.writeMicroseconds(pos);

  //Read from bluetooth and write to usb serial
  if(Serial.available())
  {
    char toSend = (char)Serial.read();
    if (toSend != '\n') {
      message += toSend;
    } else {
      val = atoi(message.c_str());
      val = map(val, 0, 1000, 800, 1400); 
      if (val >= 800 && val <= 850)
        pos = 800;
      if (val > 850 && val < 1400)
          pos = val;
      
      message = "";
    
    }
  }
  
  //Serial.println(pos);
}
