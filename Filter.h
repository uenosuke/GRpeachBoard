#ifndef FILTER_h
#define FILTER_h

#include "Arduino.h"

class Filter
{
public:
    double T_LPF;
    double Om_n;
    double sq_dt;
    double sq_Om;
    
    double omega;
    double dzeta;

    Filter(double);
    void setLowPassPara(double T, double init_data);
    double LowPassFilter(double input);
    
    void setSecondOrderPara(double xOmega, double xDzeta, double init_data);
    void initPrevData(double init_data);
    double SecondOrderLag(double input);
    
    void setNotchPara(double Omega, double init_data);
    double NotchFilter(double input);
    
private:
    double int_time;
    double preOutput;
    bool set_t;
    
    double prev_output1, prev_output2;
    bool set_secorder;
    
    double n_preOutput[2];
    double n_preInput[2];

};

#endif