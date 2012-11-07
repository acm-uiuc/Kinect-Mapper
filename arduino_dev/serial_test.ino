#include <Servo.h>

unsigned long serialdata;
int inbyte;
int servoPose;
int servoPoses[80] = {};
int attachedServos[80] = {};
int servoPin;
int pinNumber;
int sensorVal;
int analogRate;
int digitalState;
Servo myservo[] = {};

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  getSerial();
  switch(serialdata)
  {
  case 1:
    {
      //analog digital write
      getSerial();
      switch (serialdata)
      {
      case 1:
        {
          //analog write
          getSerial();
          pinNumber = serialdata;
          getSerial();
          analogRate = serialdata;
          pinMode(pinNumber, OUTPUT);
          analogWrite(pinNumber, analogRate);
          Serial.print("SET ANALOG PIN ");
          Serial.print(pinNumber);
          Serial.print(" ");
          Serial.println(analogRate);
          pinNumber = 0;
          break;
        }
      case 2:
        {
          //digital write
          getSerial();
          pinNumber = serialdata;
          getSerial();
          digitalState = serialdata;
          pinMode(pinNumber, OUTPUT);
          if (digitalState == 1)
          {
            digitalWrite(pinNumber, LOW);
            Serial.print("SET DIGITAL PIN ");
            Serial.print(pinNumber);
            Serial.println(" OFF");
          }
          if (digitalState == 2)
          {
            digitalWrite(pinNumber, HIGH);
            Serial.print("SET DIGITAL PIN ");
            Serial.print(pinNumber);
            Serial.println(" ON");
          }
          pinNumber = 0;
          break;
         
        }
     }
     break; 
    }
    case 2:
    {
      getSerial();
      switch (serialdata)
      {
      case 1:
        {
          //analog read
          getSerial();
          pinNumber = serialdata;
          pinMode(pinNumber, INPUT);
          sensorVal = analogRead(pinNumber);
          Serial.println(sensorVal);
          sensorVal = 0;
          pinNumber = 0;
          break;
        } 
      case 2:
        {
          //digital read
          getSerial();
          pinNumber = serialdata;
          pinMode(pinNumber, INPUT);
          sensorVal = digitalRead(pinNumber);
          Serial.println(sensorVal);
          sensorVal = 0;
          pinNumber = 0;
          break;
        }
      }
      break;
    }
    case 3:
    {
      getSerial();
      switch (serialdata)
      {
        case 1:
        {
           //servo read
           getSerial();
           servoPin = serialdata;
           Serial.println(servoPoses[servoPin]);
           break;
        }
        case 2:
        {
           //servo write
           getSerial();
           servoPin = serialdata;
           getSerial();
           servoPose = serialdata;
           if (attachedServos[servoPin] == 1)
           {
             myservo[servoPin].write(servoPose);
           }
           if (attachedServos[servoPin] == 0)
           {
             Servo s1;
             myservo[servoPin] = s1;
             myservo[servoPin].attach(servoPin);
             myservo[servoPin].write(servoPose);
             attachedServos[servoPin] = 1;
           }
           servoPoses[servoPin] = servoPose;
           break;
        }
        case 3:
        {
          //detach
          getSerial();
          servoPin = serialdata;
          if (attachedServos[servoPin] == 1)
          {
            myservo[servoPin].detach();
            attachedServos[servoPin] = 0;  
          }
        }
      }
    break;
    }
  }
}

long getSerial()
{
  serialdata = 0;
  while (inbyte != '/')
  {
    inbyte = Serial.read(); 
    if (inbyte > 0 && inbyte != '/')
    {
     
      serialdata = serialdata * 10 + inbyte - '0';
    }
  }
  inbyte = 0;
  return serialdata;
}
