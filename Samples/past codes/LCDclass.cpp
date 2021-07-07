#include "LCDclass.h"

// コンストラクタ
myLCDclass::myLCDclass(HardwareSerial* xserial){
    p_serial = xserial;
    line_num = LINE_0;
}

// バックライトを赤に
void myLCDclass::color_red(){
    p_serial -> write('|'); //Put LCD into setting mode
    p_serial -> write(128 + 29); //Set white/red backlight amount to 0%
    p_serial -> write('|'); //Put LCD into setting mode
    p_serial -> write(158 + 0); //Set green backlight amount to 0%
    p_serial -> write('|'); //Put LCD into setting mode
    p_serial -> write(188 + 0); //Set blue backlight amount to 0%
    p_serial->flush();
}

// バックライトを青に
void myLCDclass::color_blue(){
    p_serial -> write('|'); //Put LCD into setting mode
    p_serial -> write(128 + 0); //Set white/red backlight amount to 0%
    p_serial -> write('|'); //Put LCD into setting mode
    p_serial -> write(158 + 0); //Set green backlight amount to 0%
    p_serial -> write('|'); //Put LCD into setting mode
    p_serial -> write(188 + 29); //Set blue backlight amount to 0%
    p_serial->flush();
}

// バックライトを緑に
void myLCDclass::color_green(){
    p_serial -> write('|'); //Put LCD into setting mode
    p_serial -> write(128 + 0); //Set white/red backlight amount to 0%
    p_serial -> write('|'); //Put LCD into setting mode
    p_serial -> write(158 + 29); //Set green backlight amount to 0%
    p_serial -> write('|'); //Put LCD into setting mode
    p_serial -> write(188 + 0); //Set blue backlight amount to 0%
    p_serial->flush();
}

// バックライトを白に
void myLCDclass::color_white(){
    p_serial -> write('|'); //Put LCD into setting mode
    p_serial -> write(128 + 29); //Set white/red backlight amount to 0%
    p_serial -> write('|'); //Put LCD into setting mode
    p_serial -> write(158 + 29); //Set green backlight amount to 0%
    p_serial -> write('|'); //Put LCD into setting mode
    p_serial -> write(188 + 29); //Set blue backlight amount to 0%
    p_serial->flush();
}

void myLCDclass::clear_display(){
    p_serial -> write('|'); //Setting character
    p_serial -> write('-'); //Clear display
    p_serial->flush();
}

int myLCDclass::arrow_change(int next_line_num){
    if(line_num != next_line_num){
        if(line_num != LINE_0){
            p_serial -> write(254); //Send command character
            p_serial -> write(128 + line_num + 0); //Change the position (128) of the cursor to 2nd row (64), position 0 (0)
            p_serial -> print(" ");
        }

        if(next_line_num != LINE_0){
            p_serial -> write(254); //Send command character
            p_serial -> write(128 + next_line_num + 0); //Change the position (128) of the cursor to 2nd row (64), position 0 (0)
            p_serial -> print(">");
        }

        line_num = next_line_num;
    }
    p_serial->flush();
    return line_num;
}

void myLCDclass::write_line(String str, int line){
    int i = str.length();
    String blanks = "";
    p_serial -> write(254);
    p_serial -> write(128 + line + 0);
    
    if(i <= 20){
        p_serial->print(str);
        for(; i < 20; i++){
            blanks += " ";
        }
        p_serial->print(blanks);
    }
    p_serial->flush();
}

void myLCDclass::write_str(String str, int line, int column){
    int i = str.length();
    if(column + i < 20){
        p_serial -> write(254);
        p_serial -> write(128 + line + column);
        
        p_serial->print(str);
    }
    p_serial->flush();
}

void myLCDclass::write_int(int data, int line, int column){
    if(column < 20){
        p_serial -> write(254);
        p_serial -> write(128 + line + column);
        
        p_serial->print(data);
    }
    p_serial->flush();
}

void myLCDclass::write_double(double data, int line, int column){
    if(column < 20){
        p_serial -> write(254);
        p_serial -> write(128 + line + column);
        
        p_serial->print(data);
    }
    p_serial->flush();
}