#include <avr/pgmspace.h>
#include <stdio.h>
#include "Arduino.h"
#define MAX 16
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
    46.90797,7.35579,619.3
};

class MapMatching
{
    public:
        void begin(double x, double y);
        double distance();
        float calibrate_BARO();
        bool RESET = true;
        double cali_dist = 16;
        float xPos, yPos;
        float compensation_lat=0, compensation_lon=0;
    private:
        double dist;
        void calibrate_GPS(double x, double y);
        double latitude, longitude, height, factor_lat=0, factor_lon=0;
};
