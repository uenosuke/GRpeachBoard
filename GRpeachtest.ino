/* GR-PEACH Sketch Template V2.05.02 */
#include <Arduino.h>
#include <MsTimer2.h>
#include <SPI.h>

#include "USBHostSerial.h"
#include "phaseCounterPeach.h"
#include "AMT203V.h"

#define INTERVAL 100
#define PIN_CSB 10

phaseCounter enc1(1);
phaseCounter enc2(2);

AMT203V amt203(&SPI, PIN_CSB);

void timer_warikomi(){
  static int count = 0;
  if(count == 50){
    digitalWrite(PIN_LED_USER, !digitalRead(PIN_LED_USER));
    count = 0;
  }
  count++;
  
  Serial.print("abscount ");
  Serial.println(amt203.getEncount());
  /*Serial.print("encount1 ");
  Serial.print(enc1.getCount());
  Serial.print("\tencount2 ");
  Serial.println(enc2.getCount());*/
}

void setup()
{
  Serial.begin(115200);

  pinMode(PIN_LED_RED   , OUTPUT);
  pinMode(PIN_LED_GREEN , OUTPUT);
  pinMode(PIN_LED_BLUE  , OUTPUT);
  pinMode(PIN_LED_USER  , OUTPUT);
  pinMode(PIN_SW        , INPUT);

  digitalWrite(PIN_LED_GREEN, 1);

  SPI.begin(); // ここでSPIをbeginしてあげないとちゃんと動かなかった
  amt203.init();

  digitalWrite(PIN_LED_GREEN, 0);
  digitalWrite(PIN_LED_BLUE, 1);

  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();

  enc1.init();
  enc2.init();
}

void loop()
{

}

