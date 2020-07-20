//
// Created by mar on 21/06/19.
//


//#include "ctrl_toolbox/DigitalSlidingMode.h"

namespace ctb{

    template <class T>
    DigitalSlidingMode<T>::DigitalSlidingMode(
            std::vector<double> getAlphaBeta(const std::vector<double> , T ) ,
            double s(const double,const double , T ), T param, double sampleTime,
            unsigned short int n_state, double saturation)
    : initialized_(false)
    {
        ref_=0;
        feedback_=0;
        c_output_.resize(2, 0.0);
        state_.resize(n_state, 0.0);
        saturation_=saturation;
        sample_time_=sampleTime;
        getAlphaBeta_=getAlphaBeta;
        s_=s;
        parameter_=param;
    };

    template <class T>
    DigitalSlidingMode<T>::~DigitalSlidingMode()
    {
        std::vector<double>().swap(c_output_);
        std::vector<double>().swap(state_);

    }

    template <class T>
    void DigitalSlidingMode<T>::Initialize(double k, double sampleTime, unsigned short int n_state, double saturation)
    {
        initialized_=true;
        ref_=0;
        feedback_=0;
        c_output_.resize(2, 0.0);
        state_.resize(n_state, 0.0);
        saturation_=saturation;
        sample_time_=sampleTime;
        k_=k;

    };

    template <class T>
    double DigitalSlidingMode<T>::compute(double GetRef, double GetFbk)
    {
        if(!initialized_)
            std::cerr << "The controller is not initialized; control gain is set to the default value (1)\n";

        auto alfa_beta=getAlphaBeta_(state_,parameter_);

        // implementation of control law parametrized with
        // the sliding surface (s) and the control gain k

        double um = -k_*s_(GetRef,GetFbk, parameter_)/sample_time_;

        double u = (um - alfa_beta[0])/alfa_beta[1];

        if (saturation_ > 0)
        {
            if (std::abs(u) > saturation_)
                u = u/std::abs(u) * saturation_;
        }
        return u;

    }

    template <class T>
    double DigitalSecOrdSlidingMode<T>::compute(double GetRef, double GetFbk)
    {
        if(!this->initialized_) { std::cerr << "The controller is not initialized; control gain is set to the default value (1)\n";};

        auto alfa_beta=this->getAlphaBeta_(this->state_ , this->parameter_);

        // implementation of control law parametrized with
        // the sliding surface (s) and the control gain k

        std::cout << "alfa_beta[0]" << alfa_beta[0] << std::endl;
	std::cout << "alfa_beta[1]" << alfa_beta[1] << std::endl;

        double u = (sliding_state_/this->sample_time_ - alfa_beta[0])/alfa_beta[1];

	std::cout << "u" << u << std::endl;

        auto s_comp=this->s_(GetRef,GetFbk, this->parameter_);
	std::cout << "s_comp" << s_comp << std::endl;

        sliding_state_=sliding_state_+ (-this->k_*sliding_state_-s_comp)*this->sample_time_;

	std::cout << "sliding state" << sliding_state_ << std::endl;
	std::cout << "this->k_" << this->k_ << std::endl;
	std::cout << "-this->k_*sliding_state_" << -this->k_*sliding_state_ << std::endl;

	std::cout << "(-this->k_*sliding_state_-s_comp)*this->sample_time_" << (-this->k_*sliding_state_-s_comp)*this->sample_time_ << std::endl;




        if (this->saturation_ > 0)
                {
                    if (std::abs(u) > this->saturation_)
                        u = u/std::abs(u) * this->saturation_;
                }

        return u;
    }
}

