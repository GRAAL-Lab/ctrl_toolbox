/*
 * DigitalPID.h
 *
 *  Created on: Jun 13, 2017
 *      Author: asperi
 */

#ifndef SRC_CTRL_DIGITALPID_H_
#define SRC_CTRL_DIGITALPID_H_

#include <cstdint>
//#include "Defines.h"
//#include "OM2CtrlDataStructs.h"

typedef float float32_t;
typedef double float64_t;

#define M_PIl_OVER_180        (PI / 180.0)
#define M_180_OVER_M_PIl      (180.0 / PI)

namespace ctb
{

struct PIDGains {
	float64_t Kp;
	float64_t Ki;
	float64_t Kd;
	float64_t Kff;
	float64_t N; // maximum gain for derivative part
	float64_t Tr; // tracking time constant for anti-windup
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

	void SetSampleTime(float64_t Ts) {
		Ts_ = Ts;
	}

	void SetSaturation(float64_t uMax) {
		uMax_ = uMax;
	}

	void SetHeadingController() {
		isHeadingControl_ = true;
	}

	void Reset();

	float64_t Compute(float64_t error);
	float64_t Compute(float64_t ref, float64_t fbk);

	virtual ~DigitalPID();

private:
	float64_t Kp_, Ki_, Kd_, Kff_;
	float64_t Ts_;
	float64_t uMax_;
	float64_t u[2];
	float64_t e[3];
	float64_t y[2];
	float64_t D_, I_, N_, Tr_;
	bool isHeadingControl_;
	bool initialized_;
};

}

#endif /* SRC_CTRL_DIGITALPID_H_ */
