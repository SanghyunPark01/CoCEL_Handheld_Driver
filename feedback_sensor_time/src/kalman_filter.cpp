#include "kalman_filter.h"

KalmanFilter::KalmanFilter(double process_var, double measurement_var, Eigen::Vector2d initial_state)
{
    // Observation matrix H (1x2)
    _H = Eigen::Matrix<double, 1, 2>();
    _H << 1,0;
    
    // Initial state
    _x = initial_state;

    // Covariance matrix P (2x2)
    __P = Eigen::Matrix2d::Identity();

    // Process noise variance
    _mdProcessNoiseVar = process_var;

    // Measurement noise covariance matrix R (1x1)
    _R = Eigen::Matrix<double, 1, 1>();
    _R << measurement_var;
}
void KalmanFilter::updateMatrices(double dt)
{
    _A = Eigen::Matrix2d();
    _A << 1, dt,
          0, 1;
          
    _Q = Eigen::Matrix2d();
    _Q << pow(dt, 4) / 4 * _mdProcessNoiseVar, pow(dt, 3) / 2 * _mdProcessNoiseVar,
        pow(dt, 3) / 2 * _mdProcessNoiseVar, pow(dt, 2) * _mdProcessNoiseVar;

}
void KalmanFilter::predictState(double dt)
{
    // Update matrices for dynamic dt
    updateMatrices(dt);

    // Predict next state
    _x = _A * _x;
    // Update covariance matrix
    __P = _A * __P * _A.transpose() + _Q;
}
void KalmanFilter::updateState(double z)
{
    // Kalman Gain
    Eigen::MatrixXd S = _H * __P * _H.transpose() + _R;
    Eigen::MatrixXd K = __P * _H.transpose() * S.inverse();

    // Update state estimate
    double y = z - (_H * _x)(0);  // Scalar innovation

    _x = _x + K * y;

    // Update covariance matrix
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(2, 2);
    __P = (I - K * _H) * __P; 
}
double KalmanFilter::getCurrentState(void)
{
    return _x[0];
}

double KalmanFilter::predictNextState(void)
{
    return (_A * _x)[0];
}