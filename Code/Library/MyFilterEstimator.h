class MyFilterEstimator{
    
    public:
    void begin(float lon, float lat);
    float estimation_lon(void);
    float estimation_lat(void);
    void measurementInput(float lon, float lat);
    void modelInput(float LonIMU, float LatIMU);
    
    private:
    float curr_lon = 0;
    float prev_lon = 0;
    float curr_lat = 0;
    float prev_lat = 0;
    float lonIMU = 0;
    float latIMU = 0;
    float est_lon = 0;
    float est_lat = 0;
};
