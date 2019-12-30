#include "AMT203VPeach.h"

AMT203V::AMT203V(SPIClass* xSPI, byte xCSBpin){
  CSBpin = xCSBpin;
  pSPI = xSPI;

  pinMode(CSBpin, OUTPUT);
  digitalWrite(CSBpin, HIGH);

  //pSPI->begin();
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
  int recieved;
  int recieve_count = 0;
  int error_count = 0;
  boolean recieve_done = false;

  delay(100);

  while( !recieve_done ){
    spi_write(0x10);

    recieved = spi_write(0x00);

    while ( recieved != 0x10 ){
      recieved = spi_write(0x00);
      recieve_count++;
      // 先生のプログラムでは時間が経っても0x10が返ってこなかったら-1を戻すようにしてある

      if( recieve_count >= 10 ){
        error_count++;

        if( error_count == 3 ) return 0;
        break;
      }
    }

    if( recieved == 0x10 ) recieve_done = true;
  }

  temp[0] = spi_write(0x00);    // MSB
  temp[1] = spi_write(0x00);    // LSB

  spi_write(0x00);

  digitalWrite(CSBpin,HIGH);  

  ABSposition = (temp[0] & 0x0F) << 8;
  ABSposition |= temp[1];

  preABSposition = ABSposition;
  encount = ABSposition;
  
  return 1;
}

int AMT203V::getEncount(){
  int recieved;
  int recieve_count = 0;
  int error_count = 0;
  boolean recieve_done = false;

  while( !recieve_done ){
    spi_write(0x10);
    
    recieved = spi_write(0x00);

    while ( recieved != 0x10 ){
      recieved = spi_write(0x00);
      recieve_count++;
      // 先生のプログラムでは時間が経っても0x10が返ってこなかったら-1を戻すようにしてある

      if( recieve_count >= 10 ){
        error_count++;

        if( error_count == 10 ) return preABSposition;
        break;
      }
    }

    if( recieved == 0x10 ) recieve_done = true;
  }

  temp[0] = spi_write(0x00);    // MSB
  temp[1] = spi_write(0x00);    // LSB

  spi_write(0x00);

  digitalWrite(CSBpin,HIGH);  

  ABSposition = (temp[0] & 0x0F) << 8;
  ABSposition |= temp[1];

  updateCount();
  encount = rotation * res + ABSposition;
  
  preABSposition = ABSposition;
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
