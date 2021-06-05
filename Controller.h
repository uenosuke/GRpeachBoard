#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include "define.h"

struct ControllerData{
    unsigned int ButtonState;
    byte RJoyX, RJoyY, LJoyX, LJoyY;
};

#if CON_TYPE == CON_ADACHI
    #define MASK_BUTTON_UP    0x01
    #define MASK_BUTTON_RIGHT 0x02
    #define MASK_BUTTON_DOWN  0x04
    #define MASK_BUTTON_LEFT  0x08
    #define MASK_BUTTON_R1    0x10
    #define MASK_BUTTON_R2    0x20
    #define MASK_BUTTON_L1    0x40
    #define MASK_BUTTON_L2    0x80

    #define BUTTON_UP    1
    #define BUTTON_RIGHT 2
    #define BUTTON_DOWN  3
    #define BUTTON_LEFT  4
    #define BUTTON_R1    5
    #define BUTTON_R2    6
    #define BUTTON_L1    7
    #define BUTTON_L2    8
#elif CON_ELECOM
    #define MASK_BUTTON_X  0x0001
    #define MASK_BUTTON_Y  0x0002
    #define MASK_BUTTON_A  0x0004
    #define MASK_BUTTON_B  0x0008

    #define MASK_BUTTON_L1     0x0010
    #define MASK_BUTTON_R1     0x0020
    #define MASK_BUTTON_L2     0x0040
    #define MASK_BUTTON_R2     0x0080

    #define MASK_BUTTON_JOY_L   0x0100
    #define MASK_BUTTON_JOY_R   0x0200
    #define MASK_BUTTON_BACK    0x0400
    #define MASK_BUTTON_START   0x0800

    #define MASK_BUTTON_UP     0x1000
    #define MASK_BUTTON_RIGHT  0x2000
    #define MASK_BUTTON_DOWN   0x4000
    #define MASK_BUTTON_LEFT   0x8000

    #define BUTTON_UP    12
    #define BUTTON_RIGHT 13
    #define BUTTON_DOWN  14
    #define BUTTON_LEFT  15
    #define BUTTON_R1    5
    #define BUTTON_R2    7
    #define BUTTON_L1    4
    #define BUTTON_L2    6
#endif

class Controller{
    public:
        Controller();           
        void update();          //受信の処理＋ボタンの情報の更新．　繰り返し処理の中に置いておく必要がある．
        void statePrint();      //受信した情報をprint．基本受信の確認用．
        bool readButton_bin(unsigned int ButtonNum); //押していない時はfalse(0),押してるときはtrue(1)を返す．　ButtonNumはデータの欲しいボタンの名前を
        int  readButton(unsigned int ButtonNum);     //上にプラスして 押した瞬間は2，放した瞬間は-1を返す．    define.hを参考に数字を入力しても良い

        unsigned int getButtonState();  //分解する前のButtonStateの情報をprint 0~255の値をとる
        ControllerData getConData();
    
                                //       X
        double readJoyRX();     //       ^ 
        double readJoyRY();     //       | 
        double readJoyLX();     //  Y<---+----
        double readJoyLY();     //       | 
                                //       | 
                                //  1.0  ~   -1.0

                                //       X
        byte readJoyRXbyte();   //       ^ 
        byte readJoyRYbyte();   //       |
        byte readJoyLXbyte();   //  Y<---+----
        byte readJoyLYbyte();   //       | 
                                //       |
                                //  255  ~    0

    private:
        
        bool comCheck;
        ControllerData conData;
        ControllerData pre_conData;

        byte serial_recieve(){
            char temp;
            do{
                temp =SERIAL_CON.read();
            }
            while(temp==-1);
            //CONTROL.write(temp);    //受け取ったデータをTXピンからそのまま送っている．他のマイコンにも流したいとき用．
            return temp;
        }
};

#endif