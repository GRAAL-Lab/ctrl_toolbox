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
	double Kp;
	double Ki;
	double Kd;
	double Kff;
	double N; // maximum gain for derivative part
	double Tr; // tracking time constant for anti-windup
};

template <typename T>
struct DifferenceFunctor {
	T operator()(T a, T b){
		return (a - b);
	}
};

class DigitalPID {
public:
	DigitalPID();
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

	double Compute(double error);
	double Compute(double ref, double fbk);

	virtual ~DigitalPID();

	void SetErrorFunction(const std::function<double(double, double)>& errorFunction)
	{
		ErrorFunction_ = errorFunction;
		//useExternalErrorFunction_ = true;
	}

private:
	double Kp_, Ki_, Kd_, Kff_;
	double Ts_;
	double uMax_;
	double u[2];
	double e[3];
	double y[2];
	double D_, I_, N_, Tr_;
	//bool useExternalErrorFunction_ = {false};
	std::function<double(double, double)> ErrorFunction_;
	bool initialized_;
};

}

#endif /* SRC_CTRL_DIGITALPID_H_ */
