#include <stdio.h>
#include "Arduino.h"
#include "MyFilterEstimator.h"
#define factor 111139
int threshold = 10000;
float smallest_lat = 46.88801;//this is the smallest possible longitude in the area where the author made his measurement
float biggest_lat = 46.92414;//this is the biggest possible longitude in the area where the author made his measurement
float smallest_lon = 7.34694;//this is the smallest possible latitude in the area where the author made his measurement
float biggest_lon = 7.45195;//this is the biggest possible latitude in the area where the author made his measurement

void MyFilterEstimator::begin(float lon, float lat)
{
  prev_lon = lon;
  prev_lat = lat;
}

void MyFilterEstimator::compare(float lon, float lat)
{
  distance = sq((prev_lon-lon)*factor) + sq((prev_lat-lat)*factor);//calculate the squared distance 
  
  if (distance >= max_distance) correction = true;
  else 
  { 
    if (smallest_lon <= lon) 
    {
      if (lon <= biggest_lon) 
      {
        prev_lon = lon;
        est_lon = lon;
      }
    }
    
    if (smallest_lat <= lat)
    {
      if (lat <= biggest_lat) 
      {
        est_lat = lat;
        prev_lat = lat;
      }
    }
    correction = false;
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
