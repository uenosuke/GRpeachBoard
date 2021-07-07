#include <Arduino.h>
#include "define.h"
#include "Controller.h"

Controller::Controller(){
    conData.ButtonState = 0;
    conData.RJoyX = 127, conData.RJoyY = 127, conData.LJoyX = 127, conData.LJoyY = 127;
}

bool Controller::update(){
  char receive_data[10];
  unsigned int loop_count=0, checksum = 0x00;
  comCheck = false;

#if CON_TYPE == CON_ADACHI // 安達君開発のコントローラを使う場合の処理(どのコントローラを使うかはdefine.hで設定)
    while(loop_count < 10 && SERIAL_CON.available()){    
        if(serial_recieve() == '\n'){
            for(int i=0;i<8;i++) receive_data[i] = serial_recieve();
            for(int i=0;i<8;i++) receive_data[i] -= 0x20;
            for(int i=0;i<7;i++) checksum ^=receive_data[i];

            if(receive_data[7] == checksum & 0xFF){
                comCheck = true;
                
                pre_conData.ButtonState = conData.ButtonState;
                conData.ButtonState = ((receive_data[0] & 0x3F) << 2) | ((receive_data[1] & 0x30) >> 4);

                conData.RJoyX = ((receive_data[1] & 0x0F) << 4) | ((receive_data[2] & 0x3C) >> 2) ;
                conData.RJoyY = ((receive_data[2] & 0x03) << 6) | ( receive_data[3] & 0x3F);
                conData.LJoyX = ((receive_data[4] & 0x3F) << 2) | ((receive_data[5] & 0x30) >> 4);
                conData.LJoyY = ((receive_data[5] & 0x0F) << 4) | (receive_data[6] & 0x0F);
                    
                break;
            }
        }
        loop_count++;
    }
#elif CON_TYPE == CON_ELECOM // ELECOMのコントローラを使う場合の処理(どのコントローラを使うかはdefine.hで設定)
    // コントローラデータを取得する部分
    static int recv_num = 0;
    char c;
    while(SERIAL_CON.available()){
        c = SERIAL_CON.read();
        if(c == '\n'){
            if(recv_num == 10){// チェックサムは無く，9個受信したら値を格納
                for(int i = 0; i < 9; i++) checksum += (unsigned int)(receive_data[i] - 0x20); // チェックサムの計算
                if((checksum & 0x3F) == (receive_data[9] - 0x20)){ // チェックサムの計算が合っていた場合のみ値を格納
                    comCheck = true;

                    pre_conData.ButtonState = conData.ButtonState;

                    conData.ButtonState = 0, conData.LJoyX = 0, conData.LJoyY = 0, conData.RJoyX = 0, conData.RJoyY = 0;
                    conData.ButtonState |= (unsigned int)(receive_data[0] - 0x20);
                    conData.ButtonState |= (unsigned int)(receive_data[1] - 0x20) << 6;
                    conData.ButtonState |= (unsigned int)(receive_data[2] - 0x20) << 12;
                
                    conData.LJoyX |= (unsigned int)(receive_data[3] - 0x20);
                    conData.LJoyX |= (unsigned int)((receive_data[4] - 0x20) & 0x03) << 6;
                    conData.LJoyX = abs(conData.LJoyX - 0xFF);

                    conData.LJoyY |= (unsigned int)((receive_data[4] - 0x20) & 0x3C) >> 2;
                    conData.LJoyY |= (unsigned int)((receive_data[5] - 0x20) & 0x0F) << 4;
                    conData.LJoyY = abs(conData.LJoyY - 0xFF);

                    conData.RJoyX |= (unsigned int)((receive_data[5] - 0x20) & 0x30) >> 4;
                    conData.RJoyX |= (unsigned int)((receive_data[6] - 0x20) & 0x3F) << 2;
                    conData.RJoyX = abs(conData.RJoyX - 0xFF);

                    conData.RJoyY |= (unsigned int)(receive_data[7] - 0x20);
                    conData.RJoyY |= (unsigned int)((receive_data[8] - 0x20) & 0x03) << 6;
                    conData.RJoyY = abs(conData.RJoyY - 0xFF);
                }
            }
            recv_num = 0;
        }else{
            receive_data[recv_num] = c; 
            recv_num++;
        }
    }
#elif CON_TYPE == CON_DS4 // DualShock4を使う場合の処理(どのコントローラを使うかはdefine.hで設定)
    // コントローラデータを取得する部分
    static int recv_num = 0;
    char c;
    while(SERIAL_CON.available()){
        c = SERIAL_CON.read();
        if(c == '\n'){
            if(recv_num == 10){ // データ数はチェックサム含めて10個(0~9)
                checksum = 0;
                for(int i = 0; i < 9; i++) checksum += (unsigned int)(receive_data[i] - 0x20); // チェックサムの計算
                if((checksum & 0x3F) == (receive_data[9] - 0x20)){ // チェックサムの計算が合っていた場合のみ値を格納
                    comCheck = true;
                    
                    pre_conData.ButtonState = conData.ButtonState; // 立下り，立ち上がりの検知用にも必要

                    conData.ButtonState = 0, conData.LJoyX = 0, conData.LJoyY = 0, conData.RJoyX = 0, conData.RJoyY = 0;
                    conData.ButtonState |= receive_data[0] - 0x20;
                    conData.ButtonState |= (receive_data[1] - 0x20) << 6;
                    conData.ButtonState |= (receive_data[2] - 0x20) << 12;
                
                    conData.LJoyX |= (receive_data[3] - 0x20);
                    conData.LJoyX |= ((receive_data[4] - 0x20) & 0x03) << 6;
                    conData.LJoyX = abs(conData.LJoyX - 0xFF);

                    conData.LJoyY |= ((receive_data[4] - 0x20) & 0x3C) >> 2;
                    conData.LJoyY |= ((receive_data[5] - 0x20) & 0x0F) << 4;
                    conData.LJoyY = abs(conData.LJoyY - 0xFF);

                    conData.RJoyX |= ((receive_data[5] - 0x20) & 0x30) >> 4;
                    conData.RJoyX |= ((receive_data[6] - 0x20) & 0x3F) << 2;
                    conData.RJoyX = abs(conData.RJoyX - 0xFF);

                    conData.RJoyY |= (receive_data[7] - 0x20);
                    conData.RJoyY |= ((receive_data[8] - 0x20) & 0x03) << 6;
                    conData.RJoyY = abs(conData.RJoyY - 0xFF);
                }
            }
            recv_num = 0;
        }else{
            receive_data[recv_num] = c;
            recv_num++;
        }
    }
#endif
    return comCheck;
}

void Controller::statePrint()
{
    Serial.print(comCheck);
    Serial.print("\t");
    Serial.print(conData.ButtonState);
    Serial.print("\t");
    Serial.print(conData.RJoyX);
    Serial.print("\t");
    Serial.print(conData.RJoyY);
    Serial.print("\t");
    Serial.print(conData.LJoyX);
    Serial.print("\t");
    Serial.println(conData.LJoyY);
}

bool Controller::readButton_bin(unsigned int ButtonNum){//放しているときは０，押しているときは１
    return ((conData.ButtonState & (0x0001 << (ButtonNum - 1))) == (0x0001 << (ButtonNum - 1)))? true:false;
    }
    
int Controller::readButton(unsigned int ButtonNum){//放しているときは０，押しているときは１，押した瞬間は２，放した瞬間は－１
    int result = 0;
    if((conData.ButtonState & (0x0001 << (ButtonNum - 1))) == (0x0001 << (ButtonNum - 1))) result += 2;
    if((pre_conData.ButtonState & (0x0001 << (ButtonNum - 1))) == (0x0001 << (ButtonNum - 1)))result -= 1;
    return result;
    }

unsigned int Controller::getButtonState(){
    return conData.ButtonState;
    }

ControllerData Controller::getConData(){
    return conData;
    }

double Controller::readJoyRX()
{   
    if(conData.RJoyX==127)return 0;
    return ((double)conData.RJoyX-127.5)/127.5;
}

double Controller::readJoyRY()
{
    if(conData.RJoyY==127)return 0;
    return ((double)conData.RJoyY-127.5)/127.5;
}

double Controller::readJoyLX()
{
    if(conData.LJoyX==127)return 0;
    return ((double)conData.LJoyX-127.5)/127.5;
}

double Controller::readJoyLY()
{
    if(conData.LJoyY==127)return 0;
    return ((double)conData.LJoyY-127.5)/127.5;
}
    
byte Controller::readJoyRXbyte()
{
    return conData.RJoyX;
}
    
byte Controller::readJoyRYbyte()
{
    return conData.RJoyY;
}

byte Controller::readJoyLXbyte()
{
    return conData.LJoyX;
}
    
byte Controller::readJoyLYbyte()
{
    return conData.LJoyY;
}