/* GR-PEACH Sketch Template V2.05.02 */
#include <Arduino.h>
#include <MsTimer2.h>
#include "USBHostSerial.h"
#include "phaseCounterPeach.h"

#define INTERVAL 100
#define ENC_INIT_VAL 0x7FFF

phaseCounter enc1(1);
phaseCounter enc2(2);

void timer_warikomi(){
  Serial.print("encount1 ");
  Serial.print(enc1.getCount());
  Serial.print("\tencount2 ");
  Serial.println(enc2.getCount());
}

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_LED_RED   , OUTPUT);
  pinMode(PIN_LED_GREEN , OUTPUT);
  pinMode(PIN_LED_BLUE  , OUTPUT);
  pinMode(PIN_LED_USER  , OUTPUT);
  pinMode(PIN_SW        , INPUT);

  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();

  enc1.init();
  enc2.init();
}

void loop()
{

}

