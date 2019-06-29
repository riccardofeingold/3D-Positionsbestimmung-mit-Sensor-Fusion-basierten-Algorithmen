#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(float X0, float P0, float R, int q)
{
    model_state = X0;
    est_error = P0;
    mea_error = R;
    q = q;
}

double KalmanFilter::update(float mea)
{
    //Kalman_gain
    K = est_error/(est_error+mea_error);

    //Estimating the current state 
    current_state = model_state + K * (mea - model_state);

    //Update current estimation error 
    est_error = (1 - K) * est_error; 

    return current_state;
}

double KalmanFilter::predict(float model)
{
    model_state = model;
    est_error = est_error + q;
    return est_error;
}
