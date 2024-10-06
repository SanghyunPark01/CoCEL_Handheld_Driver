#ifndef KALMAN_FILTER
#define KALMAN_FILTER

#include "utility.h"

class KalmanFilter
{
private:
    Eigen::Matrix2d _A; // State transition matrix
    Eigen::MatrixXd _H; // Observation matrix
    Eigen::Matrix2d __P; // Covariance matrix
    Eigen::Matrix2d _Q; // Process noise covariance matrix
    Eigen::MatrixXd _R; // Measurement noise covariance matrix
    Eigen::Vector2d _x; // State vector
    double _mdProcessNoiseVar; // Process noise variance

    void updateMatrices(double dt);
public:
    KalmanFilter(double process_var, double measurement_var, Eigen::Vector2d initial_state);
    void predictState(double dt);
    void updateState(double z);
    double getCurrentState(void);
    double predictNextState(void);
};

#endif KALMAN_FILTER