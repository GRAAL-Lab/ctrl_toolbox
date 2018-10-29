/*
 * DigitalPID.cc
 *
 *  Created on: March 28, 2018
 *      Author: wanderfra
 */

#include <cmath>

#include "DigitalPID.h"
#include <iostream>

namespace ctb {

DigitalPID::DigitalPID(const PIDGains& gains, double sampleTime, double saturation)
    : Ts_(sampleTime)
    , uMax_(saturation)
    , initialized_(false)
    , TrToBeSetted_(false)
{

    SetGains(gains);
    SetErrorFunction(DifferenceFunctor<double>());
    Reset();
}

DigitalPID::~DigitalPID(){

}

void DigitalPID::SetGains(const PIDGains& g)
{
    Kp_ = g.Kp;
    Ki_ = g.Ki;
    Kd_ = g.Kd;
    Kff_ = g.Kff;
    N_ = g.N;
    Tr_ = g.Tr;
}

/**
 * @brief Sets the control sampling time.
 */
void DigitalPID::SetSampleTime(double Ts)
{
    Ts_ = Ts;
}

/**
 * @brief Sets the output saturation value.
 */
void DigitalPID::SetSaturation(double uMax)
{
    uMax_ = uMax;
}

/**
 * @brief Resets the PID state.
 */
void DigitalPID::Reset(){
    int i;
    for (i = 0; i < 3; i++) {
        e_[i] = 0;
    }
    for (i = 0; i < 2; i++) {
        u_[i] = 0;
        y_[i] = 0;
    }

    D_ = 0;
    I_ = 0;
    initialized_ = false;
}

/**
 * @brief Main function: computes the PID output.
 *
 * @param ref	Current reference
 * @param fbk	Current feedback
 * @return	The PID controlled output
 */
double DigitalPID::Compute(double ref, double fbk){
    int i;
    if (initialized_) {
        for (i = 2; i > 0; i--) {
            e_[i] = e_[i - 1];
        }
        u_[1] = u_[0];
        y_[1] = y_[0];
        y_[0] = fbk;
    } else {
        //ortos::DebugConsole::Write(ortos::LogLevel::info, "DigitalPID::Compute", "Initializing variables");
        e_[0] = e_[1] = e_[2] = ErrorFunction_(ref, fbk);
        u_[1] = u_[0] = 0;
        y_[1] = y_[0] = fbk;
        initialized_ = true;
    }

    double ydiff;
    e_[0] = ErrorFunction_(ref, fbk);
    ydiff = ErrorFunction_(y_[0], y_[1]);

    if (Kd_ != 0.0 && Kp_ != 0.0) {
        double Td = Kd_ / Kp_;
        D_ = Td / (Td + N_ * Ts_) * D_ - Kp_ * Td * N_ / (Td + N_ * Ts_) * (ydiff);
    }

    double v = Kp_ * e_[0] + I_ + D_ + Kff_ * ref;
    if (std::abs(v) > uMax_) {
        u_[0] = v / std::abs(v) * uMax_;
    } else {
        u_[0] = v;
    }

    //ortos::DebugConsole::Write(ortos::LogLevel::info, "DigitalPID::Compute",
    //		"e = %+lf I = %+lf D = %+lf uff = %+lf v = %+lf u = %+lf ydiff = %+lf", e[0], I_, D_, Kff_ * ref, v, u[0], ydiff);
    if (Ki_ != 0.0) {
        if (Tr_ == 0.0) {
            std::cerr << "\r Set Tr different from zero to add integral part " << std::flush;
            TrToBeSetted_ = true;
            I_ = 0;
        } else {
            if (TrToBeSetted_) {
                std::cerr << "\r Added Integral Part                                    " << std::flush;
                TrToBeSetted_ = false;
            }
            I_ = I_ + (Ki_ * Ts_) * e_[0] + (Ts_ / Tr_) * (u_[0] - v);
        }
    } else {
        I_ = 0;
    }

    return u_[0];
}

void DigitalPID::SetErrorFunction(const std::function<double(double, double)>& errorFunction)
{
    ErrorFunction_ = errorFunction;
}

}
