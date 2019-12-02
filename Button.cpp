#include "Button.h"

// コンストラクタ
Button::Button(int xpin_num){
    pin_num = xpin_num;
    pinMode(pin_num, INPUT);
    pre_button_state = get_button_state();
    initial_flag = true;
}

bool Button::get_button_state(){
    return digitalRead(pin_num);
}

bool Button::button_rise(){
    bool ret = 0;
    bool button_state = get_button_state();
    if(initial_flag){
        if(button_state == 0){
            pre_button_state = button_state;
            initial_flag = false;
        }
    }else{
        if(button_state && !pre_button_state){
            initial_flag = true;
            ret = 1;
        }
        pre_button_state = button_state;
    }
    return ret;
}

bool Button::button_fall(){
    bool ret = 0;
    bool button_state = get_button_state();
    if(initial_flag){
        if(button_state == 1){
            pre_button_state = button_state;
            initial_flag = false;
        }
    }else{
        if(!button_state && pre_button_state){
            initial_flag = true;
            ret = 1;
        }
        pre_button_state = button_state;
    }
    return ret;
}

bool Button::button_changed(){
    bool ret = 0;
    bool button_state = get_button_state();
    if(initial_flag){
        pre_button_state = button_state;
        initial_flag = false;
    }else{
        if(button_state != pre_button_state){
            initial_flag = true;
            ret = 1;
        }
        pre_button_state = button_state;
    }
    return ret;
}