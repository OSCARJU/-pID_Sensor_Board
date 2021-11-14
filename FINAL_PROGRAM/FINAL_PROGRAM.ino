#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include "MegunoLink.h"
#include "Filter.h"
#include <SimpleKalmanFilter.h>
#include<dht.h>

//  if  you require to change the pin number, Edit the pin with your arduino pin.
#define DHT11_PIN 3 

Adafruit_ADS1115 ads(0x48);
dht DHT;
Adafruit_ADS1115 ads2(0x49);
float Voltage1 = 0.0;
float Voltage2 = 0.0;
float Voltage3 = 0.0;
float Voltage4 = 0.0;
float Voltage5 = 0.0;
float Voltage6 = 0.0;
float Voltage7 = 0.0;
float Voltage8 = 0.0;
// Create a new exponential filter with a weight of 10 and initial value of 0. 
ExponentialFilter<long> ADCFilter(10, 0);

SimpleKalmanFilter simpleKalmanFilter1(100, 100, 10);
SimpleKalmanFilter simpleKalmanFilter2(100, 100, 10);
SimpleKalmanFilter simpleKalmanFilter3(100, 100, 10);
SimpleKalmanFilter simpleKalmanFilter4(100, 100, 10);

// kalman variables
//float varVolt = 1.12184278324081E-05;  // variance determined using excel and reading samples of raw sensor data
//float varProcess = 1e-4;



void setup(void)
{
  Serial.begin(9600);
  ads.begin();
  ads2.begin();
}

void loop(void)
{
  int16_t adc0;  // we read from the ADC, we have a sixteen bit integer as a result
  int16_t adc02;  // we read from the ADC, we have a sixteen bit integer as a result
  int16_t adc03;  // we read from the ADC, we have a sixteen bit integer as a result
  int16_t adc04;  // we read from the ADC, we have a sixteen bit integer as a result
  int16_t adc05;  // we read from the ADC, we have a sixteen bit integer as a result
  int16_t adc06;  // we read from the ADC, we have a sixteen bit integer as a result
  int16_t adc07;  // we read from the ADC, we have a sixteen bit integer as a result
  int16_t adc08;  // we read from the ADC, we have a sixteen bit integer as a result



  adc0 = simpleKalmanFilter1.updateEstimate(ads.readADC_SingleEnded(0));

// kalman process
 
 // Xe = simpleKalmanFilter.updateEstimate(adc0);
//  adc02=simpleKalmanFilter.updateEstimate(adc0);
  Voltage1 = (adc0 * 0.1875) / 1000;
  adc02 = simpleKalmanFilter2.updateEstimate(ads.readADC_SingleEnded(1));
  Voltage2 = (adc02 * 0.1875) / 1000;
  adc03 = simpleKalmanFilter3.updateEstimate(ads.readADC_SingleEnded(2));
  Voltage3 = (adc03 * 0.1875) / 1000;
  adc04 = simpleKalmanFilter4.updateEstimate(ads.readADC_SingleEnded(3));
  Voltage4 = (adc04 * 0.1875) / 1000;
  adc05 = ads2.readADC_SingleEnded(0);
  Voltage5 = (adc05 * 0.1875) / 1000;
  adc06 = ads2.readADC_SingleEnded(1);
  Voltage6 = (adc06 * 0.1875) / 1000;
  adc07 = ads2.readADC_SingleEnded(2);
  Voltage7 = (adc07 * 0.1875) / 1000;
  adc08 = ads2.readADC_SingleEnded(3);
  Voltage8 = (adc08 * 0.1875) / 1000;

  /* Serial.print("AIN0: ");
    Serial.print(adc0);
    Serial.print("\tVoltage1: ");
    Serial.println(Voltage1, 7);
    Serial.println();
     Serial.print("AIN02: ");
    Serial.print(adc02);
    Serial.print("\tVoltage2: ");
    Serial.println(Voltage2, 7);
    Serial.println();
     Serial.print("AIN03: ");
    Serial.print(adc03);
    Serial.print("\tVoltage3: ");
    Serial.println(Voltage3, 7);
    Serial.println();
     Serial.print("AIN04: ");
    Serial.print(adc04);
    Serial.print("\tVoltage4: ");
    Serial.println(Voltage4, 7);
    Serial.println();
     Serial.print("AIN05: ");
    Serial.print(adc05);
    Serial.print("\tVoltage5: ");
    Serial.println(Voltage5, 7);
    Serial.println();
     Serial.print("AIN06: ");
    Serial.print(adc06);
    Serial.print("\tVoltage6: ");
    Serial.println(Voltage6, 7);
    Serial.println();
     Serial.print("AIN07: ");
    Serial.print(adc07);
    Serial.print("\tVoltage7: ");
    Serial.println(Voltage7, 7);
    Serial.println();
     Serial.print("AIN08: ");
    Serial.print(adc08);
    Serial.print("\tVoltage8: ");
    Serial.println(Voltage8, 7);
    Serial.println();


    //sensorValue = analogRead(sensorPin);
  */
  //Serial.print("AIN0: ");
  Serial.print(Voltage1, 7);
  Serial.print("%");
 // Serial.print("AIN02: ");
  Serial.print(Voltage2, 7);
   Serial.print("%");
 // Serial.println(" ");
 // Serial.print("AIN03: ");
  Serial.print(Voltage3, 7);
   Serial.print("%");
//  Serial.println(" ");
//  Serial.print("AIN04: ");
 Serial.print(Voltage4, 7);
  Serial.print("%");
 // Serial.println(" ");

  int chk = DHT.read11(DHT11_PIN); 
//Serial.println(" Humidity " ); 
Serial.print(DHT.humidity, 1); 
 Serial.print("%");
//Serial.println(" Temparature "); 
Serial.print(DHT.temperature, 1);
 Serial.println(" ");
//delay(500); 

  //Serial.print(adc0, 7);
  //Serial.print(" ");
  //Serial.print(adc0, 7);
  //Serial.print(" ");
  //Serial.print(adc0, 7);
  //Serial.print(" ");
  //Serial.println(adc0, 7);
 // Serial.println(" ");

  //delay(25);


  delay(1000);
}
