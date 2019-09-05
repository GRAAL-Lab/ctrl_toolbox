//
// Created by mar on 20/06/19.
//

#ifndef GROUP_PROJECT_DIGITALSLIDINGMODE_H
#define GROUP_PROJECT_DIGITALSLIDINGMODE_H

#include "ctrl_toolbox/HelperFunctions.h"
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <vector>
#include <functional>
#include <z3.h>
#include <iostream>

namespace ctb {

    /**
    * @class DigitaSlidingMode
    *
    * @brief to compute the control algorithm starting by the states of NonLinearObserver
    *
    * T is a template structure of the system parameters
    */
    template <class T>
    class DigitalSlidingMode{

    public:
        DigitalSlidingMode():initialized_(false){};
        DigitalSlidingMode( std::vector<double> getAlphaBeta(const std::vector<double>,T ) ,
                            double s(const double,const double, T ), T param,
                            double sampleTime=0,unsigned short int n_state=2, double saturation=0);

        ~DigitalSlidingMode();

        void Initialize(double k,double sampleTime, unsigned short int n_state, double saturation=0);

        /**
        * @brief Sets the control sampling time.
	    */
        void setSampleTime( double sampleTime) { sample_time_=sampleTime;};

        void setSaturation( double saturation) { saturation_=saturation;};

        /**
	    * @brief Main function: computes the control output.
	    *
	    * @param ref	Current reference
	    * @param fbk	Current feedback
        * @return	    control output
	    */
        virtual double compute( double GetRef , double GetFbk);

        double GetRef() const {return ref_; };

        double GetFbk() const {return feedback_;};

        double GetOutput() const { return c_output_[0];};

        void setState(const std::vector<double>& state){state_=state;};

    protected:
        T parameter_; //!< system parameter
        double ref_;  //!< reference input
        double feedback_; //!< feedback input
        std::vector<double> c_output_; //!< control output

        double sample_time_;
        double saturation_; //!< saturation of control output
        bool initialized_; //!< the controller is initialized
        std::vector<double> state_; //!< system state

        double k_=1; //!< sliding mode parameter

        //!< parametric function to compute the alpha(state) and beta(state) for the control law
        std::function<const std::vector<double>(const std::vector<double>, T)> getAlphaBeta_;
        //!< sliding surface
        std::function<const double(const double,const double, T)> s_;


    };

    /**
    * @class DigitalSecOrdSlidingMode
    *
    * @brief Second order Sliding mode
    *
    * to compute the control algorithmstarting by the states of NonLinearObserver
    *
    * T is a template structure of the system parameters
    */
    template <class T>
    class DigitalSecOrdSlidingMode: public DigitalSlidingMode<T> {

    public:
        DigitalSecOrdSlidingMode(){this->initialized_ = false;};
        DigitalSecOrdSlidingMode( std::vector<double> getAlphaBeta(const std::vector<double>, T) ,
        double s(const double,const double, T), T param,
        double sampleTime=0,unsigned short int n_state=2, double saturation=0)
        :DigitalSlidingMode<T>(getAlphaBeta,s,param,sampleTime,n_state,saturation){sliding_state_=0;};

        /**
	    * @brief Main function: computes the control output.
	    *
	    * @param ref	Current reference
	    * @param fbk	Current feedback
	    * @return	    control output
	    */
        double compute( double GetRef , double GetFbk);

    private:
        double sliding_state_; //!< sliding state

    };


}

#include "template_class/DigitalSlidingMode.tpp"

#endif //GROUP_PROJECT_DIGITALSLIDINGMODE_H
