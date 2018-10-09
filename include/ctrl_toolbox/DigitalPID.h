/*
 * DigitalPID.h
 *
 *  Created on: March 28, 2018
 *      Author: wanderfra
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

/**
 * @brief Difference functor
 *
 * An utility templated functor to compute the difference between two objects
 * of the same type.
 */
template <typename T>
struct DifferenceFunctor {
	T operator()(T a, T b) const{
		return (a - b);
	}
};

/**
 * @class DigitalPID
 *
 * @brief Impletation of a flexible 1-D digital PID controller
 *
 * This class implements a PID digital controller, with anti-windup and
 * feedforward control. The function to compute the error \f$ e \f$ can be configured
 * passing an std::function to SetErrorFunction(), function which has to take
 * two parameters \p reference and \p feedback and return the error.
 * The default error function is just the difference \f$ ref-fbk \f$, implemented
 * with the ctb::DifferenceFunctor. The output \f$ u \f$ is evaluated using the following
 * formula:\n
 *
 * \f$ u = K_p \cdot e + I + D + K_{ff} \cdot ref \f$
 *
 * where :\n
 *
 * \f$ I(t_k) = I(t_{k-1}) + (K_i \cdot T_s) \cdot e + \frac{T_s}{T_r} \cdot (u(t_{k}) - Sat(u(t_{k}))) \f$
 *
 * \f$ D(t_k) = \frac{T_d}{T_d + N \cdot T_s} \cdot D(t_{k-1}) - \frac{K_p \cdot T_d \cdot N}{T_d + N \cdot T_s} \cdot (y(t_k) - y(t_{k-1})) \f$
 *
 * The reference scientific paper for all the formulas derivation can be found at this link:
 * <a href="https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf" target="_blank">PID Control - Control and Dynamical Systems</a>
 *
 *
 */
class DigitalPID {
public:
	DigitalPID(const PIDGains& g, double sampleTime, double saturation);
	virtual ~DigitalPID();

	/**
	 * @brief Sets the PID internal Gains.
	 *
	 * @param[in] g The
	 */
	void SetGains(const PIDGains& g)	{
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
	void SetSampleTime(double Ts) {
		Ts_ = Ts;
	}

	/**
	 * @brief Sets the output saturation value.
	 */
	void SetSaturation(double uMax) {
		uMax_ = uMax;
	}

	/**
	 * @brief Resets the PID state.
	 */
	void Reset();

	/**
	 * @brief Main function: computes the PID output.
	 *
	 * @param ref	Current reference
	 * @param fbk	Current feedback
	 * @return	The PID controlled output
	 */
	double Compute(double ref, double fbk);

	void SetErrorFunction(const std::function<double(double, double)>& errorFunction)
	{
		ErrorFunction_ = errorFunction;
	}

private:
	double Kp_;		//!< Proportional gain
	double Ki_;		//!< Integral gain
	double Kd_;		//!< Derivative gain
	double Kff_;	//!< Feed-forward gain
	double N_;		//!< Maximum gain for derivative part
	double Tr_;		//!< Tracking time constant for anti-windup
	double Ts_;		//!< Sample Time
	double uMax_;	//!< Output saturation value
	double u_[2];	//!< PID Output
	double e_[3];	//!< Error
	double y_[2];	//!< Feedback
	double D_;		//!< Derivative Term
	double I_;		//!< Integral Term
	std::function<double(double, double)> ErrorFunction_; //!< Definition of the error function that is used in Compute()
	bool initialized_;	//!< Utility variable to check if the PID has been Reset()
    bool TrToBeSetted_{false};
};

}

#endif /* SRC_CTRL_DIGITALPID_H_ */
