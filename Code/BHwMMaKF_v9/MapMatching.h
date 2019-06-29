#include <avr/pgmspace.h>
#include <stdio.h>
#include "Arduino.h"
#define MAX 10
typedef struct X_Y_Height{
    double lat;
    double lon;
    double h;
};

const PROGMEM struct X_Y_Height data[MAX] = {
    46.90982,7.35846,585.7,
    46.90934,7.35846,586.6,
    46.90862,7.35820,588.1,
    46.90780,7.35766,591.5,
    46.90763,7.35731,596.5,
    46.90804,7.35677,611,
    46.90829,7.35614,622,
    46.90876,7.35521,643.5,
    46.90938,7.35452,643.5,
    46.91005,7.35146,666.2
};

class MapMatching
{
    public:
        double begin(double x, double y, bool lat_or_lon);
        double distance();
        float calibrate_BARO();
        bool RESET = true;
        double cali_dist = 0.00005;
        double dist;
    private:
        double calibrate_GPS(double x, double y, bool lat_or_lon);
        float x, y;
        double latitude, longitude, height, factor_lat, factor_lon, compensation_lat, compensation_lon;
};
