#ifndef SDCLASS_h
#define SDCLASS_h

#include <Arduino.h>
#include <SD.h>

#define RED 0
#define BLUE 1

class mySDclass{
public:
    mySDclass();
    
    bool SD_enable = false;
    String logFileName;
    char c_logFileName[13];
    
    int init();
    int make_logfile();
    int write_logdata(String);

    int path_read(int, double*, double*, double*, double*, int*, int*, double*);

    double str2double(char*, int);
    int str2uint(char* str, int num);
};

#endif