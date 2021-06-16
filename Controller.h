#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include "define.h"

struct ControllerData{
    unsigned int ButtonState;
    byte RJoyX, RJoyY, LJoyX, LJoyY;
};

class Controller{
    public:
        Controller();           
        bool update();          //受信の処理＋ボタンの情報の更新．　繰り返し処理の中に置いておく必要がある．
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