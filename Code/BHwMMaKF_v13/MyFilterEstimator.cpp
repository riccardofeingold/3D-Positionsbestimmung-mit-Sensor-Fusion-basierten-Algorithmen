#include <stdio.h>
#include "Arduino.h"
#include "MyFilterEstimator.h"
#define factor 111172.87

void MyFilterEstimator::begin(float lon, float lat)
{
    prev_lon = lon;
    prev_lat = lat;
}
void MyFilterEstimator::measurementInput(float lon, float lat)
{
    curr_lon = lon;
    curr_lat = lat;
}

void MyFilterEstimator::modelInput_lon(float LonIMU)
{
    lonIMU = LonIMU;
}

void MyFilterEstimator::modelInput_lat(float LatIMU)
{
  latIMU = LatIMU;
}

float MyFilterEstimator::estimation_lon(void)
{
    float distance = sq((prev_lon - curr_lon)*factor) + sq((prev_lat - curr_lat)*factor);
    
    if (distance > max_distance)
    {
        est_lon = lonIMU;
        est_lat = latIMU;
        prev_lon = est_lon;
        prev_lat = est_lat;
    }else
    {
        est_lon = curr_lon;
        est_lat = curr_lat;
        prev_lon = est_lon;
        prev_lat = est_lat;
    }
    return est_lon;
}

float MyFilterEstimator::estimation_lat(void)
{
    return est_lat;
}
