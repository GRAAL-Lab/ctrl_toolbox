/*
 * DigitalPID.h
 *
 *  Created on: Jun 13, 2017
 *      Author: asperi
 */

#ifndef SRC_CTRL_DIGITALPID_H_
#define SRC_CTRL_DIGITALPID_H_

#include <cstdint>
#include <functional>

namespace ctb
{

struct PIDGains {
	double Kp = {0.0};  //!< Proportional gain
	double Ki = {0.0};  //!< Integral gain
	double Kd = {0.0};  //!< Derivative gain
	double Kff = {0.0}; //!< Feed-forward gain
	double N = {0.0};   //!< Maximum gain for derivative part
	double Tr = {0.0};  //!< Tracking time constant for anti-windup

	PIDGains() = default;
};


template <typename T>
struct DifferenceFunctor {
	T operator()(T a, T b){
		return (a - b);
	}
};

/**
 * \class DigitalPID
 *
 * \brief Impletation of a flexible digital PID controller
 *
 * This class implements a PID digital controller, with anti-windup and
 * feedforward control. The function to compute the error can be configured
 * passing an std::function to SetErrorFunction(). The default one is just
 * the difference \a ref-fbk where the output \f$ u \f$ is evaluated using the following
 * formula:
 *
 * \f$
    D_ = Td / (Td + N_ * Ts_) * D_ - Kp_ * Td * N_ / (Td + N_ * Ts_) * (ydiff) \\

    v = Kp_ * e[0] + I_ + D_ + Kff_ * ref;
 *
 * \f$
 *
 */
class DigitalPID {
public:
	DigitalPID(const PIDGains& g, double sampleTime, double saturation);
	virtual ~DigitalPID();

	void SetGains(PIDGains g)	{
		Kp_ = g.Kp;
		Ki_ = g.Ki;
		Kd_ = g.Kd;
		Kff_ = g.Kff;
		N_ = g.N;
		Tr_ = g.Tr;
	}

	void SetSampleTime(double Ts) {
		Ts_ = Ts;
	}

	void SetSaturation(double uMax) {
		uMax_ = uMax;
	}

	void Reset();

	//double Compute(double error);
	double Compute(double ref, double fbk);

	void SetErrorFunction(const std::function<double(double, double)>& errorFunction)
	{
		ErrorFunction_ = errorFunction;
	}

private:
	double Kp_, Ki_, Kd_, Kff_;
	double Ts_;
	double uMax_;
	double u[2];
	double e[3];
	double y[2];
	double D_, I_, N_, Tr_;
	std::function<double(double, double)> ErrorFunction_;
	bool initialized_;
};

}

#endif /* SRC_CTRL_DIGITALPID_H_ */
