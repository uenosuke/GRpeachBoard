#ifndef _AMT203V_H
#define _AMT203V_H

#include <SPI.h>

class AMT203V{
  public:
  AMT203V(SPIClass*, byte);

  int spi_write(int msg);
  int init();
  int getEncount();
  void updateCount();
  int setZeroPos();
  int getRawEncount();

  private:
  bool init_done;
  byte CSBpin;
  const int res = 4096;
  int ABSposition;  // rawdata
  int preABSposition;
  int rotation = 0;
  int encount;        // 渡す値
  int temp[2];
  SPIClass *pSPI;
  
};

#endif
