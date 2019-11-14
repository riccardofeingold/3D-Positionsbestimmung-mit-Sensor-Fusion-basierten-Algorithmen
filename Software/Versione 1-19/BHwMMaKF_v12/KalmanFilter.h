#include "Arduino.h"
class KalmanFilter
{
    public: 
        KalmanFilter(float X0, float P0, float R, float q);
        double update(float mea);
        double predict(float model);
    private:
        float model_state;
        double current_state;
        double est_error;
        double mea_error;
        float q;
        double K;
};
