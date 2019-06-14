#ifndef EXTENDED_KALMAN_FILTER_H_
#define EXTENDED_KALMAN_FILTER__H_

#include "MeasurmentKalmanFilter.h"
#include "ModelKalmanFilter.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>
#include <rml/RML.h>

namespace ctb {

class ExtendedKalmanFilter {

    // F, note that f must depend on the state and the input and the dimension of the state, the indexAngles indicates
    // which
    // entries of the state are angles to be wrapped;
    ExtendedKalmanFilter(
        int stateDimension, std::vector<int> indexAngles, std::shared_ptr<ModelKalmanFilter> kalmanFilterModel);

    // Method that postpone to H the matrix output of the function, note that h is function of the stateitself
    void AddMeasurment(std::shared_ptr<MeasurmentKalmanFilter> h);

    void Predict(double now, Eigen::VectorXd u);

    void ApplyMeasurements();

    void Reset();

    void Init();

    Eigen::VectorXd GetState();

private:
    double FilterAngularJump(const double& primaryHeading, const double& otherHeading);
    void NormalizeAngle(double& angle);

    Eigen::MatrixXd W_; //input matrix
    Eigen::MatrixXd F_; //state matrix
    Eigen::MatrixXd G_; //measurment matrix
    Eigen::MatrixXd K_; // kalman gain
    Eigen::VectorXd x_; // state
    Eigen::VectorXd y_; // measurements

    Eigen::MatrixXd S_; // measure estimate covariance
    Eigen::MatrixXd Sigma_; // state estimate covariance
    Eigen::MatrixXd Q_; // system noise covariance (error in modelling)
    Eigen::MatrixXd R_; // measurement covariance (error in the modelling of G)


    double lastEstimateTime_;
    double speed_;
    int stateDimension_;
    std::shared_ptr<ModelKalmanFilter> kalmanFilterModel_;
    std::vector<int> indexAngles_;
    Eigen::VectorXd u_;
    bool isFirst_;
    rml::RegularizationData regularizationParameter_;
};
}
#endif
