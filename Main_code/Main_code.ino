#include "CytronMotorDriver.h"

//--------------------Blynk Credential-------------------
#define BLYNK_TEMPLATE_ID "TMPL6EyU79w7f"
#define BLYNK_TEMPLATE_NAME "Autodosing"
#define BLYNK_AUTH_TOKEN "qxUcK43AMf-gf261Wgs4qeokXXHH-uxx"
//-------------------------------------------------------

//------------------Library List-------------------------
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <SoftwareSerial.h>
#include "DFRobot_EC.h"
#include <EEPROM.h>
//-------------------------------------------------------

//------------------ EC Sensor Configuration-------------
#define EC_PIN A1
float voltage_ec,ecValue,temperature_ec = 25;
DFRobot_EC ec;
//-------------------------------------------------------

//------------------ pH Sensor configuration-------------
#define ph_sensor_pin 0
unsigned long int average_ph_value;
float b;
int buf[10];
int temp;
float phValue;
//-------------------------------------------------------

//------------------ Motor Driver COnfiguration----------
CytronMD motor1(PWM_DIR, 5, 4); //PWM 1A = pin 5, DIR 1B = Pin 4
CytronMD motor2(PWM_DIR, 6, 7); //PWM 2A = pin 6, DIR 2B = Pin 7
//-------------------------------------------------------

//------------------ To Connect Wifi---------------------
char ssid[] = "Semah Terbang";
char pass[] = "ryernaanDick2@@@";
SoftwareSerial EspSerial(10,11); //(RX, TX), connect rx from esp-01 to pin 11, and tx to pin 10
ESP8266 wifi(&EspSerial);
#define ESP8266_BAUD 115200
//-------------------------------------------------------
//-------------------Send Sensor data to blynk-----------
BlynkTimer timer;

//-------------------------------------------------------

void setup()
{
  Serial.begin(9600);
  ec.begin();
  delay(10);

  EspSerial.begin(ESP8266_BAUD);
  delay(10);

  Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass);
  ec.begin();

  timer.setInterval(1000L, sendSensor);
}

void loop()
{
  Blynk.run();
  ph_sensor();
  ec_sensor();
  sendSensor();
  if(ecValue < 9)
  {
    motor1.setSpeed(125);
    motor2.setSpeed(125);
    delay(1000);
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    delay(500);
  }
  else
  {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
  }
  Serial.print("EC :");
  Serial.print(ecValue);
  Serial.println("pH : ");
  Serial.print(phValue);

} 

void sendSensor()
{
  float ec_send = ecValue;
  float ph_send = phValue;
  
  Blynk.virtualWrite(V1,ec_send); //EC sensor send data to Virtual Port 1 in Blynk
  Blynk.virtualWrite(V0,ph_send); // Ph sensor send data to Virtual Port 0 in Blynk
}

//-------------------- pH Sensor function------------------
void ph_sensor()
{
  for(int i=0; i< 10; i++)
  {
    buf[i]=analogRead(ph_sensor_pin);
    delay(10);
  }
  for(int i=0; i<9; i++)
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  average_ph_value=0;
  for(int i=2; i<8; i++)
  {
    average_ph_value += buf [i];
  }
  phValue = (float)average_ph_value*5.0/1024/6;
  phValue = 3.5*phValue;

  Serial.print(phValue);
}
//-------------------------------------------------------

//-------------------- EC Sensor function----------------
void ec_sensor()
{
  static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U)  //time interval: 1s
    {
      timepoint = millis();
      voltage_ec = analogRead(EC_PIN)/1024.0*5000;   // read the voltage
      //temperature = readTemperature();          // read your temperature sensor to execute temperature compensation
      ecValue =  ec.readEC(voltage_ec,temperature_ec);  // convert voltage to EC with temperature compensation
      Serial.print("temperature:");
      Serial.print(temperature_ec,1);
      Serial.print("^C  EC:");
      Serial.print(ecValue,2);
      Serial.println("ms/cm");
    }
    ec.calibration(voltage_ec,temperature_ec);          // calibration process by Serail CMD
}
//-------------------------------------------------------


void pump(int pump_1_speed, int pump_2_speed)
{
  motor1.setSpeed(pump_1_speed);
  motor2.setSpeed(pump_2_speed);
}




