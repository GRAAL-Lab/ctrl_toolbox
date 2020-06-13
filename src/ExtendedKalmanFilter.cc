#include "ExtendedKalmanFilter.h"
#include "HelperFunctions.h"
#include <rml/RML.h>

namespace ctb {

// F, note that f must depend on the state and the input and the dimension of the state, the indexAngles indicates
// which
// entries of the state are angles to be wrapped;
ExtendedKalmanFilter::ExtendedKalmanFilter(int stateDimension, std::vector<int> indexAngles, std::shared_ptr<ModelKalmanFilter> kalmanFilterModel)
    : stateDimension_{ stateDimension }
    , kalmanFilterModel_{ std::move(kalmanFilterModel) }
    , indexAngles_{ std::move(indexAngles) }
{
    x_.setZero(stateDimension_);
    P_.setZero(stateDimension_, stateDimension_);

    //regulatization parameters to perform speudoinverse
    regularizationParameter_.params.lambda = 0.000001;
    regularizationParameter_.params.threshold = 0.000001;

    isFirst_ = true;
}

// Method that postpone to H the matrix output of the function, note that h is function of the stateitself
void ExtendedKalmanFilter::AddMeasurement(std::shared_ptr<MeasurementKalmanFilter> measurement)
{
    if (isFirst_) {
        H_ = measurement->ComputeJacobian(x_, u_);
        z_ = measurement->MeasureVector();
        predicted_z_ = measurement->ComputeObservationModel(x_);

        if (measurement->IsAngleMeasure()) {
            predicted_z_ = FilterAngularJump(z_, predicted_z_);
        }
        R_ = measurement->Covariance();
        isFirst_ = false;
    } else {

        long int size_new_measure = measurement->MeasureVector().size();

        Eigen::MatrixXd zero_previous_covariance;
        zero_previous_covariance.setZero(size_new_measure, z_.size());

        Eigen::MatrixXd zero_new_covariance;
        zero_new_covariance.setZero(z_.size(), size_new_measure);

        H_ = rml::UnderJuxtapose(H_, measurement->ComputeJacobian(x_, u_));

        Eigen::VectorXd yTemp = measurement->MeasureVector();
        Eigen::VectorXd ypredictTemp = measurement->ComputeObservationModel(x_);

        if (measurement->IsAngleMeasure()) {
            ypredictTemp = FilterAngularJump(yTemp, ypredictTemp);
        }

        z_ = rml::UnderJuxtapose(z_, yTemp);

        predicted_z_ = rml::UnderJuxtapose(predicted_z_, ypredictTemp);

        R_ = rml::UnderJuxtapose(R_, zero_previous_covariance);
        Eigen::MatrixXd R_temp = rml::UnderJuxtapose(zero_new_covariance, measurement->Covariance());
        R_ = rml::RightJuxtapose(R_, R_temp);
    }
}

void ExtendedKalmanFilter::Prediction(const Eigen::VectorXd& u)
{
    u_ = u; //input

    F_ = kalmanFilterModel_->ComputeJacobian(x_, u_); //The state transition Jacobian
    Q_ = kalmanFilterModel_->Covariance(); //Covariance of the process

    x_ = kalmanFilterModel_->ComputeStateTransitionModel(x_, u_); //Predicted state estimate

    for (const auto i : indexAngles_) {
        NormalizeAngle(x_(i));
    }

    P_ = F_ * P_ * F_.transpose() + Q_; //Predicted covariance estimate
}

void ExtendedKalmanFilter::Update()
{
    if (!isFirst_) {

        S_ = H_ * P_ * H_.transpose() + R_; //Innovation (or residual) covariance

        K_ = P_ * H_.transpose() * rml::RegularizedPseudoInverse(S_, regularizationParameter_); // Near-optimal Kalman gain

        x_ = x_ + K_ * (z_ - predicted_z_); // Updated state estimate
        P_ = P_ - K_ * S_ * K_.transpose(); // Updated covariance estimate

        isFirst_ = true;
    }
}

void ExtendedKalmanFilter::Reset() {}

void ExtendedKalmanFilter::Init(const Eigen::VectorXd initialState, const Eigen::MatrixXd P)
{
    x_ = initialState;
    P_ = P;
    isFirst_ = true;
}

Eigen::VectorXd ExtendedKalmanFilter::GetState() { return x_; }
}
