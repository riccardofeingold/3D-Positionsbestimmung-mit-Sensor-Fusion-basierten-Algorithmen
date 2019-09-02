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

void MyFilterEstimator::modelInput(float LonIMU, float LatIMU)
{
    lonIMU = LonIMU;
    latIMU = LatIMU;
}

float MyFilterEstimator::estimation_lon(void)
{
    float distance = ((prev_lon - curr_lon)*factor)**2 + ((prev_lat - curr_lat)*factor)**2;
    
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
