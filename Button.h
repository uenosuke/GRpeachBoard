#ifndef _BUTTON_H_INCLUDED
#define _BUTTON_H_INCLUDED

#include <Arduino.h>

class Button{
  public:
  Button(int);

  bool button_rise();
  bool button_fall();
  bool button_changed();
  bool get_button_state();

  private:
  int pin_num;
  bool pre_button_state;
  bool initial_flag;
};

#endif