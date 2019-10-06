class MyFilterEstimator
{
    public:
      void begin(float lon, float lat);
      void compare(float lon, float lat);
      void measurementInput(float lon, float lat);
      void modelInputLon(float lon);
      void modelInputLat(float lat);
      int max_distance = 2500;
      float est_lon = 0;
      float est_lat = 0;
      
    private:
      bool correction = false;
      float prev_lon = 0;
      float prev_lat = 0;
};
