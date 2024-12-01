/*
 * DigitalPID.cc
 *
 *  Created on: March 28, 2018
 *      Author: wanderfra
 */

#include <cmath>

#include "ctrl_toolbox/pid/DigitalPID.h"
#include <iostream>

namespace ctb {

DigitalPID::DigitalPID()
    : PIDInitialized_(false)
    , hasBeenReset_(true)
    , TrToBeSetted_(false)
    , D_(0.0) // Initialize D_ to zero
    , I_(0.0) // Initialize I_ to zero
{
    u_.resize(2, 0.0);
    e_.resize(3, 0.0);
    y_.resize(2, 0.0);
}


DigitalPID::DigitalPID(const PIDGains& gains, double sampleTime, double saturation)
    : Ts_(sampleTime)
    , uMax_(saturation)
    , PIDInitialized_(true)
    , hasBeenReset_(true)
    , TrToBeSetted_(false)
{

    u_.resize(2, 0.0);
    e_.resize(3, 0.0);
    y_.resize(2, 0.0);
    SetGains(gains);
    SetErrorFunction(DifferenceFunctor<double>());
    Reset();
}

DigitalPID::~DigitalPID()
{
}

void DigitalPID::Initialize(const PIDGains& gains, double sampleTime, double saturation)
{
    Ts_ = sampleTime;
    uMax_ = saturation;
    SetGains(gains);
    SetErrorFunction(DifferenceFunctor<double>());
    PIDInitialized_ = true;
    Reset();
}

PIDGains DigitalPID::GetGains() const
{
    return g_;
}

void DigitalPID::SetGains(const PIDGains& gains)
{
    g_ = gains;
}

double DigitalPID::GetRef() const
{
    return ref_;
}

double DigitalPID::GetFbk() const
{
    return y_[0];
}

double DigitalPID::GetOutput() const
{
    return u_[0];
}

void DigitalPID::SetSampleTime(double Ts)
{
    Ts_ = Ts;
}

void DigitalPID::SetSaturation(double uMax)
{
    uMax_ = uMax;
}

/**
 * @brief Resets the PID state.
 */
void DigitalPID::Reset()
{
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
    hasBeenReset_ = true;
}

/**
 * @brief Main function: computes the PID output.
 *
 * @param ref	Current reference
 * @param fbk	Current feedback
 * @return	The PID controlled output
 */
double DigitalPID::Compute(double ref, double fbk)
{
    ref_ = ref;

    if (PIDInitialized_) {
        int i;
        if (hasBeenReset_) {
            //ortos::DebugConsole::Write(ortos::LogLevel::info, "DigitalPID::Compute", "Initializing variables");
            e_[0] = e_[1] = e_[2] = ErrorFunction_(ref, fbk);
            u_[1] = u_[0] = 0;
            y_[1] = y_[0] = fbk;
            hasBeenReset_ = false;
        } else {
            for (i = 2; i > 0; i--) {
                e_[i] = e_[i - 1];
            }
            u_[1] = u_[0];
            y_[1] = y_[0];
            y_[0] = fbk;
        }

        double ydiff;
        e_[0] = ErrorFunction_(ref, fbk);
        ydiff = ErrorFunction_(y_[0], y_[1]);

        if (g_.Kd != 0.0 && g_.Kp != 0.0) {
            double Td = g_.Kd / g_.Kp;
            D_ = Td / (Td + g_.N * Ts_) * D_ - g_.Kp * Td * g_.N / (Td + g_.N * Ts_) * (ydiff);
        } else {
            D_ = 0.0; // Ensure D_ is zero when derivative term is not computed
        }
        double v = g_.Kp * e_[0] + I_ + D_ + g_.Kff * ref;
        if (std::abs(v) > uMax_) {
            u_[0] = v / std::abs(v) * uMax_;
        } else {
            u_[0] = v;
        }

        //ortos::DebugConsole::Write(ortos::LogLevel::info, "DigitalPID::Compute",
        //		"e = %+lf I = %+lf D = %+lf uff = %+lf v = %+lf u = %+lf ydiff = %+lf", e[0], I_, D_, Kff_ * ref, v, u[0], ydiff);
        if (g_.Ki != 0.0) {
            if (g_.Tr == 0.0) {
                std::cerr << "\r Set Tr different from zero to add integral part " << std::flush;
                TrToBeSetted_ = true;
                I_ = 0;
            } else {
                if (TrToBeSetted_) {
                    std::cerr << "\r Added Integral Part                                    " << std::flush;
                    TrToBeSetted_ = false;
                }
                I_ = I_ + (g_.Ki * Ts_) * e_[0] + (Ts_ / g_.Tr) * (u_[0] - v);
            }
        } else {
            I_ = 0;
        }
        return u_[0];
    } else {
        std::cerr << "PID Was not initialized! Returning = 0.0" << std::endl;
        return 0.0;
    }
}

void DigitalPID::SetErrorFunction(const std::function<double(double, double)>& errorFunction)
{
    ErrorFunction_ = errorFunction;
}
}
