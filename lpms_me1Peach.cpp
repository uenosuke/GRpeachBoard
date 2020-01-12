#include "lpms_me1Peach.h"
#include <math.h>

#define PIx2 6.28318530718

// コンストラクタ
lpms_me1::lpms_me1(HardwareSerial* xserial){
  serial = xserial;
  anglex = 0.0;
  angley = 0.0;
  anglez = 0.0;
  pre_rawanglex = 0.0;
  pre_rawangley = 0.0;
  pre_rawanglez = 0.0;

  init_ignore = true;
  init_done = false;
}

// コマンドモードへの移行
void lpms_me1::goto_command_mode(){
  serial->write(0x3A);
  serial->write(0x01);//ID
  serial->write(0x00);
  serial->write(0x06);//command
  serial->write(0x00);
  serial->write(0x00);
  serial->write(0x00);
  serial->write(0x07);//check sum
  serial->write(0x00);
  serial->write(0x0D);
  serial->write(0x0A);
}

// 送信データの変更(角度データのみ)
void lpms_me1::set_transmit_data(){
  serial->write(0x3A);
  serial->write(0x01);//ID
  serial->write(0x00);
  serial->write(0x0A);//command
  serial->write(0x00);
  serial->write(0x04);//32bit=4byte送信
  serial->write(0x00);
  serial->write(0x00);//送信データ１
  serial->write(0x00);//送信データ２
  serial->write(0x02);//17bit目：角度データのみを取得
  serial->write(0x00);//送信データ４
  serial->write(0x11);//checksum
  serial->write(0x00);
  serial->write(0x0D);
  serial->write(0x0A);
}

// フィルタの変更(カルマンフィルタ)
void lpms_me1::set_filter_mode(){
  serial->write(0x3A);
  serial->write(0x01);//ID
  serial->write(0x00);
  serial->write(0x29);//Command
  serial->write(0x00);
  serial->write(0x04);//32bit=4byte送信
  serial->write(0x00);
  serial->write(0x02);//送信データ1 0x02 でカルマンフィルタ
  serial->write(0x00);//送信データ2
  serial->write(0x00);//送信データ3
  serial->write(0x00);//送信データ4
  serial->write(0x30);
  serial->write(0x00);
  serial->write(0x0D);
  serial->write(0x0A);
}

// オフセットの設定
void lpms_me1::set_offset(){
  serial->write(0x3A);
  serial->write(0x01);//ID
  serial->write(0x00);
  serial->write(0x12);//Command
  serial->write(0x00);
  serial->write(0x04);//32bit=4byte送信
  serial->write(0x00);
  serial->write(0x00);//送信データ1 0x00 で object offset
  serial->write(0x00);//送信データ2
  serial->write(0x00);//送信データ3
  serial->write(0x00);//送信データ4
  serial->write(0x17);
  serial->write(0x00);
  serial->write(0x0D);
  serial->write(0x0A);
}

// オフセットのリセット(よくわからないので使ってない)
void lpms_me1::reset_orientation_offset(){
  serial->write(0x3A);
  serial->write(0x01);//ID
  serial->write(0x00);
  serial->write(0x52);//command
  serial->write(0x00);
  serial->write(0x00);
  serial->write(0x00);
  serial->write(0x53);//check sum
  serial->write(0x00);
  serial->write(0x0D);
  serial->write(0x0A);
}

// センサデータの送信要求
void lpms_me1::get_sensor_data(){
  serial->write(0x3A);
  serial->write(0x01);
  serial->write(0x00);
  serial->write(0x09);
  serial->write(0x00);
  serial->write(0x00);
  serial->write(0x00);
  serial->write(0x0A);//checksum
  serial->write(0x00);
  serial->write(0x0D);
  serial->write(0x0A);
}

float lpms_me1::get_z_angle(){
  if(init_done){
    get_sensor_data();
    int response = recv_proc(10);
    // 受信は浮動小数点(符号部，指数部，仮数部)に分かれて送られてくるので，以下でunpacking
    if(response == 16 ){
      unsigned int bits_dataz = (buffer[12] | (buffer[13]<<8)) | ((buffer[14]<<16) | (buffer[15]<<24));
      unsigned int mantissa_part = bits_dataz & 0x7FFFFF; // 仮数部を取得
      unsigned int copy_matissa_part = mantissa_part;
      float k = pow(2.0f, -23);
      float real_mantissa = 0.0f;
      
      for(int i = 0; i < 23; i++){
        real_mantissa += k * (float)(copy_matissa_part & 0x000001); 
        copy_matissa_part = copy_matissa_part >> 1;
        k *= 2.0f;
      }
      
      int exponent = (int)((bits_dataz & 0x7F800000) >> 23) - 127; // 指数部を取得
      float rawanglez = pow(2.0f, exponent) * (1.0f + real_mantissa);
      if(bits_dataz>>31){ // 符号部の値に応じて+-を変更
        rawanglez *= -1.0f;
      }

      // センサは-pi ~ +pi の範囲で取れるので，差分を積算していくことで連続して取れるようにする
      float diff_rawanglez;
      if(init_ignore) {
        diff_rawanglez = 0.0;
      }else{
        diff_rawanglez = rawanglez - pre_rawanglez;
      }

      if(fabs(diff_rawanglez) >= 3.0){ 
        if(rawanglez < 0){ //+から-へ回ったとき
          anglez += PIx2 + diff_rawanglez;
        }else{ // -から+へ回ったとき
          anglez += -PIx2 + diff_rawanglez;
        }
      }else{
        anglez += diff_rawanglez;
      }
      pre_rawanglez = rawanglez;

      return anglez;
    }
  }
  return 0.0; // 初期化が終わっていない場合は，0を返す
}

int lpms_me1::recv_proc(int timeout_num = 10){
  int state = 0, data_counter = 0, timeout_counter = 0;
  short command = 0, data_length = 0, check_sum = 0, recv_sum = 0;
  bool recv_done = false;
  
  // 受信処理
  while(!recv_done){
    if(timeout_counter > timeout_num * 20){
      return -1;
    }
    timeout_counter++;
    
    if (serial->available() > 0) {
      byte data = serial->read();

      switch (state) {
      case 0:
        if (data == 0x3A) state++;
        break;
      case 1:
        if (data == 0x01){
          state++;
          check_sum += data;
        }
        else {
          state = 0;
          check_sum = 0;
        }
        break;
      case 2:
        if (data == 0x00){
          state++;
          check_sum += data;
        }
        else{
          state = 0;
          check_sum = 0;
        }
        break;
      case 3:
        command |= data;
        check_sum += data;
        state++;
        break;
      case 4:
        command |= data<<8;
        check_sum += data;
        state++;
        break;
      case 5:
        data_length |= data;
        check_sum += data;
        state++;
        break;
      case 6:
        data_length |= data<<8;
        check_sum += data;
        if(data_length == 0){
          state += 2;
        }else state++;
        break;
      case 7:
        //Serial.print(data, HEX);
        //Serial.print(" ");
        buffer[data_counter++] = data;
        check_sum += data;
        if (data_counter == data_length) state++;
        break;
      case 8:
        //Serial.println("");
        recv_sum |= data;
        state++;
        break;
      case 9:
        recv_sum |= data<<8;
        if (check_sum == recv_sum) {
            state++;
          }else{
            return -5;
          }
        break;
      case 10:
        if(data == 0x0D) state++;
        else return -6;
        break;
      case 11:
        if(data == 0x0A){
          state++;
          recv_done = true;
        }
        else return -7;
        break;
      default:
        return -8;
      }
    }
  }
  return data_counter;
}

int lpms_me1::init(){
  byte trash;
  int datanum, result[4] = {0};

  if(!init_done){
    // ここは Serial1 をジャイロセンサに使うことを前提に書かれている
    // P2_3 を CTSピンとして使うための設定
    /***** ポートの初期化 *****/
    GPIO.PIBC2 &= ~0x0008; // ポート入力バッファ制御レジスタ 入力バッファ禁止
    GPIO.PBDC2 &= ~0x0008; // ポート双方向制御レジスタ 双方向モードを禁止
    GPIO.PM2 &= ~0x0008; // ポートモードレジスタ 入力モード
    GPIO.PMC2 &= ~0x0008; // ポートモード制御レジスタ ポートモード
    GPIO.PIPC2 &= ~0x0008; // ポート IP 制御レジスタ　入出力はPMn.PMnmビットによって制御されます
      
    /***** 入力機能のポート設定 *****/
    GPIO.PBDC2 |= 0x0008; // ポート双方向制御レジスタ 双方向モードを許可
      
    /***** ポート設定 *****/
    GPIO.PFC2 |= 0x0008;
    GPIO.PFCE2 &= ~0x0008;
    GPIO.PFCAE2 |= 0x0008;

    GPIO.PIPC2 |= 0x0008; // ポート IP 制御レジスタ　入出力はPMn.PMnmビットによって制御されます
    GPIO.PMC2 |= 0x0008; // ポートモード制御レジスタ ポートモード

    // CTSピンを使うための設定
    CPG.STBCR4 &= ~0x40; // FIFO内臓シリアルコミュニケーションインタフェースチャンネル1は動作(これをやらないと以下が書き変わらない)
    SCIF1.SCFCR |= 0x08; // CTS#1 を利用

    serial->begin(115200);

    datanum = serial->available();
    for(int i = 0; i < datanum; i++){
      trash = serial->read(); // ゴミデータを捨てる
    }
    
    do{
      goto_command_mode();
      //Serial.print("goto command mode sent: ");
      result[0] = recv_proc(500);
      //Serial.println(result[0]);
    }while(result[0] != 0);
    
    do{
      set_transmit_data();
      //Serial.print("set transmit data command sent: ");
      result[1] = recv_proc(500);
      //Serial.println(result[1]);
    }while(result[1] != 0);

    //set_filter_mode();
    //Serial.print("set filter mode sent: ");
    //result[2] = recv_proc(1000);
    //Serial.println(result[2]);

    delay(10); // これがないと，データが正常に取れなくなる
    
    do{
      set_offset();
      //Serial.print("set offset sent: ");
      result[3] = recv_proc(500);
      //Serial.println(result[3]);
    }while(result[3] != 0);

    for(int i = 0; i < 5; i++){
      get_z_angle();
      delay(10);
    }

    init_ignore = false;
  }

  //if(result[0] >= 0 && result[1] >= 0) digitalWrite(PIN_LED3, HIGH);
  //if(result[2] >= 0 && result[3] >= 0) digitalWrite(PIN_LED2, HIGH);
  if((result[0] + result[1] + result[3]) == 0){
    init_done = true;
    return 1;
  } 
  else return (result[0] + result[1] + result[3]);
}
