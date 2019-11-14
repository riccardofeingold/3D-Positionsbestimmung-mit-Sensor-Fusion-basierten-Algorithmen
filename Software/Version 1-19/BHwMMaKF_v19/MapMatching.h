#include <avr/pgmspace.h>
#include <stdio.h>
#include "Arduino.h"
#define MAX 38
typedef struct {
    double lat;
    double lon;
    double h;
} X_Y_Height;

const PROGMEM X_Y_Height data[] = {
    46.90982,7.35846,585.7,
    46.90934,7.35846,586.6,
    46.90862,7.35820,588.1,
    46.90780,7.35766,591.5,
    46.90763,7.35731,596.5,
    46.90804,7.35677,611,
    46.90829,7.35614,622,
    46.90876,7.35521,643.5,
    46.90938,7.35452,643.5,
    46.91005,7.35146,666.2,
    46.90720,7.35695,590.4,
    46.90604,7.35557,586.9,
    46.90620,7.35387,590.6,
    46.90676,7.35427,596.6,
    46.90741,7.35613,610.7,
    46.90797,7.35579,619.3,
    46.90739,7.35949,585.6,
    46.90390,7.35816,611.1,
    46.90456,7.36065,628.2,
    46.90446,7.36355,646.8,
    46.90420,7.36791,671.5,
    46.90352,7.37115,685.2,
    46.90340,7.37349,700.8,
    46.90267,7.37627,727.8,
    46.90315,7.37964,735.1,
    46.90418,7.38298,744.6,
    46.90501,7.38322,747.7,
    46.90515,7.38487,752.8,
    46.90278,7.38668,722.4,
    46.90167,7.38782,689.7,
    46.90076,7.38986,682.1,
    46.89995,7.38673,655.1,
    46.89921,7.38451,639.8,
    46.89888,7.38081,645.2,
    46.89867,7.37629,660.9,
    46.90081,7.37176,675.7,
    46.90204,7.37002,679.5
};

class MapMatching
{
    public:
        void begin(double x, double y, bool calibrate);
        double distance();
        float calibrate_BARO();
        void calibrate_GPS(double x, double y, bool calibrate);
        bool RESET = true;
        double cali_dist = 20;
        float xPos, yPos;
        float compensation_lat=0, compensation_lon=0;
        bool inside_cal_area = false;//so that we only calibrate once for each fix point
    private:
        double dist;
        double latitude, longitude, height; 
};
