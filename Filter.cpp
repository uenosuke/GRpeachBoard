#include "Filter.h"

Filter::Filter(double xint_time)
{
    int_time = xint_time;
    set_t = false;
}

void Filter::setLowPassPara(double T, double init_data)
{
    T_LPF = T;
    preOutput = init_data;
    set_t = true;
}

double Filter::LowPassFilter(double input)
{
    //static double preOutput = input;
    if(!set_t) {
        return input;
    } else {
        double Output = (int_time * input + T_LPF * preOutput)/(T_LPF + int_time);
        preOutput = Output;
        return Output;
    }
}

void Filter::setSecondOrderPara(double xOmega, double xDzeta, double init_data)
{
    omega = xOmega;
    dzeta = xDzeta;
    prev_output2 = prev_output1 = init_data;

    set_secorder = true;
}

void Filter::initPrevData(double init_data)
{
    prev_output2 = prev_output1 = init_data;
}

double Filter::SecondOrderLag(double input)
{
    if(!set_secorder) {
        return input;
    } else {
        double output = (2*prev_output1*(1+dzeta*omega*int_time) - prev_output2 + pow(omega, 2.0)*pow(int_time, 2.0)*input)/(1 + 2*dzeta*omega*int_time + pow(omega, 2.0)*pow(int_time, 2.0));
        prev_output2 = prev_output1;
        prev_output1 = output;
        return output;
    }
}

void Filter::setNotchPara(double Omega, double int_data)
{
    // 落としたい角周波数[rad/s]をOm_nに入れる
    Om_n = Omega;
    sq_Om = pow(Om_n, 2.0); // Om_nの2乗
    sq_dt = pow(int_time, 2.0); // dtの2乗

    n_preOutput[0] = int_data;
    n_preOutput[1] = int_data;

    n_preInput[0] = int_data;
    n_preInput[1] = int_data;
}

double Filter::NotchFilter(double input)
{
    double Output = (2*(1 + Om_n * int_time) * n_preOutput[0]-n_preOutput[1] + (1 + sq_Om * sq_dt) * input -2 * n_preInput[0] + n_preInput[1]) / (1 + 2 * Om_n * int_time + sq_Om * sq_dt);

    n_preInput[1] = n_preInput[0];
    n_preInput[0] = input;

    n_preOutput[1] = n_preOutput[0];
    n_preOutput[0] = Output;

    return Output;
}