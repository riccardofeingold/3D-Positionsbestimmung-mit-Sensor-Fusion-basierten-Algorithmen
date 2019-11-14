#include <stdio.h>
#include "Arduino.h"
#include "MyFilterEstimator.h"
#define factor 111172.87

void MyFilterEstimator::begin(float lon, float lat)
{
  prev_lon = lon;
  prev_lat = lat;
}

void MyFilterEstimator::compare(float lon, float lat)
{
  float distance = sq((prev_lon-lon)*factor) + sq((prev_lat-lat)*factor);
  
  if (distance >= max_distance) correction = true;
  else 
  {
    correction = false;
    est_lon = lon;
    est_lat = lat;
    prev_lon = lon;
    prev_lat = lat;
  }
}

void MyFilterEstimator::modelInputLon(float lon)
{
  if (correction) 
  {
    est_lon = lon;
    prev_lon = lon;
  }
}
void MyFilterEstimator::modelInputLat(float lat)
{
  if (correction)
  {
    est_lat = lat;
    prev_lat = lat;
  }
}
