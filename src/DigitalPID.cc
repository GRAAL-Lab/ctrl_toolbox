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

DigitalPID::~DigitalPID()
{
    // TODO Auto-generated destructor stub
}

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
    initialized_ = false;
}

double DigitalPID::Compute(double ref, double fbk)
{
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

    if (Kd_ != 0 && Kp_ != 0) {
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
    if (Ki_ != 0) {
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

/*
double DigitalPID::Compute(double error) {
	int i;
	if (initialized_) {
		for (i = 2; i > 0; i--) {
			e[i] = e[i - 1];
		}
		u[1] = u[0];
		e[0] = error;
	} else {
		//ortos::DebugConsole::Write(ortos::LogLevel::info, "DigitalPID::Compute", "Initializing variables");
		e[0] = e[1] = e[2] = error;
		u[1] = u[0] = 0;
		initialized_ = true;
	}

	if (Kd_ != 0 && Kp_ != 0) {
		double Td = Kd_ / Kp_;
		D_ = Td / (Td + N_ * Ts_) * D_ + Kp_ * Td * N_ / (Td + N_ * Ts_) * (e[0] - e[1]);
		//D_ = Kd_ / Ts_ * (e[0] - e[1]);
	}

	double v = Kp_ * e[0] + I_ + D_;
	if (std::abs(v) > uMax_) {
		u[0] = v / std::abs(v) * uMax_;
	} else {
		u[0] = v;
	}

	//ortos::DebugConsole::Write(ortos::LogLevel::info, "DigitalPID::Compute", "e = %+lf I = %+lf D = %+lf v = %+lf u = %+lf",
	//		e[0], I_, D_, v, u[0]);

	if (Ki_ != 0) {
		I_ = I_ + (Ki_ * Ts_) * e[0] + (Ts_ / Tr_) * (u[0] - v);
	} else {
		I_ = 0;
	}

	return u[0];

}
*/
}
