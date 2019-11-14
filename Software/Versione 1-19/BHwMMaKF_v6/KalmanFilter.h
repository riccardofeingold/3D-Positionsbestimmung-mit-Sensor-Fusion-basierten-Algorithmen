class KalmanFilter
{
    public: 
        KalmanFilter(float X0, float P0, float R, float q);
        float update(float mea);
        float predict(float model);
    private:
        float model_state;
        float current_state;
        float est_error;
        float mea_error;
        float q;
        float K;
};
