#ifndef _LCCDCLASS_H_INCLUDED
#define _LCCDCLASS_H_INCLUDED

#define LINE_0 -1
#define LINE_1 0
#define LINE_2 64
#define LINE_3 20
#define LINE_4 84

#include <Arduino.h>

class myLCDclass{
  public:
  myLCDclass(HardwareSerial*);

  void color_red();
  void color_blue();
  void color_green();
  void color_white();
  void clear_display();
  int arrow_change(int);

  void write_line(String, int);
  void write_str(String, int, int);
  void write_int(int, int, int);
  void write_double(double, int, int);

  private:
  HardwareSerial *p_serial;
  int line_num;
};

#endif
