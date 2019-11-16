/* GR-PEACH Sketch Template V2.05.02 */
#include <Arduino.h>
#include <MsTimer2.h>
#include <SPI.h>

#include "USBHostSerial.h"
#include "phaseCounterPeach.h"
#include "AMT203VPeach.h"
#include "lpms_me1Peach.h"

#define INTERVAL 100
#define PIN_CSB 10

phaseCounter enc1(1);
phaseCounter enc2(2);

AMT203V amt203(&SPI, PIN_CSB);
lpms_me1 lpms(&Serial1);

void LEDblink(byte pin, int times){
  analogWrite(pin, 0);
  for(int i = 0; i < times; i++){
    wait(0.25);
    analogWrite(pin, 255);
    wait(0.25);
    analogWrite(pin, 0);
  }
}

void timer_warikomi(){
  // RGB LED を良い感じに光らせるための処理
  static int count = 0;
  if(count < 255){
    analogWrite(PIN_LED_RED, count);
    analogWrite(PIN_LED_BLUE, 255 - count);
  }else if(count < 255 * 2){
    analogWrite(PIN_LED_GREEN, count - 255);
    analogWrite(PIN_LED_RED, 255 * 2 - count);
  }else if(count < 255 * 3){
    analogWrite(PIN_LED_BLUE, count - 255 * 2);
    analogWrite(PIN_LED_GREEN, 255 * 3 - count);
  }else{
    count = 0;
  }
  count += 2; // ここで光る周期を変えられる(はず)

  
  Serial.print("abscount ");
  Serial.print(amt203.getEncount());

  Serial.print("lpms ");
  Serial.println(lpms.get_z_angle());
  /*Serial.print("encount1 ");
  Serial.print(enc1.getCount());
  Serial.print("\tencount2 ");
  Serial.println(enc2.getCount());*/
}

void error_stop(){
  while(1){
    analogWrite(PIN_LED_RED, 255);
    analogWrite(PIN_LED_BLUE, 0);
    wait(0.25);
    analogWrite(PIN_LED_RED, 0);
    analogWrite(PIN_LED_BLUE, 255);
    wait(0.25);
  }
}

void setup()
{
  Serial.begin(115200);

  /*pinMode(PIN_LED_RED   , OUTPUT);
  pinMode(PIN_LED_GREEN , OUTPUT);
  pinMode(PIN_LED_BLUE  , OUTPUT);
  pinMode(PIN_LED_USER  , OUTPUT);*/
  pinMode(PIN_SW        , INPUT);

  SPI.begin(); // ここでSPIをbeginしてあげないとちゃんと動かなかった
  if(amt203.init() != 1) error_stop();
  LEDblink(PIN_LED_GREEN, 2);
  
  if(lpms.init() != 1) error_stop();
  LEDblink(PIN_LED_BLUE, 2);
  
  // SDのカードの処理をここに入れる
  LEDblink(PIN_LED_RED, 2);
  
  enc1.init();
  enc2.init();

  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();
}

void loop()
{

}

