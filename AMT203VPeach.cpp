#include "AMT203VPeach.h"

AMT203V::AMT203V(SPIClass* xSPI, byte xCSBpin){
  CSBpin = xCSBpin;
  pSPI = xSPI;
  init_done = false;
}

// SPI送信部分
int AMT203V::spi_write(int msg){
  int msg_temp = 0;
  digitalWrite(CSBpin,LOW);
  delayMicroseconds(50);
  msg_temp = pSPI->transfer(msg);
  digitalWrite(CSBpin,HIGH);
  delayMicroseconds(50);
  return(msg_temp);
}

int AMT203V::init(){
  pinMode(CSBpin, OUTPUT);
  digitalWrite(CSBpin, HIGH);

  int ret = getRawEncount();
  preABSposition = ABSposition;
  
  if(ret == -1) return -1;
  else init_done = true;
  
  return 1;
}

int AMT203V::getRawEncount(){
  int recieved;
  int recieve_count = 0;
  int error_count = 0;
  bool recieve_done = false;

  while( !recieve_done ){
    spi_write(0x10);
    recieved = spi_write(0x00);

    while ( recieved != 0x10 ){
      recieved = spi_write(0x00);
      recieve_count++;

      if( recieve_count >= 10 ){
        error_count++;
        if( error_count == 5 ) return -1; // 5回最初からトライして正常に受信できなかったら0を返す
        break; // 10回以上正しい返答がなかったら最初からやり直し
      }
    }
    if( recieved == 0x10 ) recieve_done = true;
  }

  temp[0] = spi_write(0x00);    // MSB
  temp[1] = spi_write(0x00);    // LSB

  spi_write(0x00);
  digitalWrite(CSBpin,HIGH);  // 念のためここでもHIGHにしておく

  ABSposition = (temp[0] & 0x0F) << 8;
  ABSposition |= temp[1];
  
  return 1;
}

int AMT203V::getEncount(){
  if(init_done){
    getRawEncount();
    updateCount();
    
    encount = rotation * res + ABSposition;
    preABSposition = ABSposition;
  }
  else{
    return -1;
  }
  return encount;
}

// 2,1,0の次に4095ではなくマイナスの値になるように，4093,4094,4095の次に0にならず大きな値になるようにしてる関数
void AMT203V::updateCount(){
  if(abs(preABSposition - ABSposition) >= 3000){
    if(preABSposition > ABSposition){
      rotation++;
    }else if(preABSposition < ABSposition){
      rotation--;
    }
  }
}

int AMT203V::setZeroPos()
{
  int response;
  int count = 0;

  spi_write(0x70);
  response = spi_write(0x00);

  while(response != 0x10) {
    response = spi_write(0x00);
    count++;

    if(count >= 10) {
      return -1;
    }
  }

  if(response == 0x80) {
    return 1;
  }

  return -1;
}
