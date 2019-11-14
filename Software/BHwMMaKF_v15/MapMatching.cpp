#include "MapMatching.h"
#include "MyFilterEstimator.h"
#define DEG_2_METER 111139
void MapMatching::begin(double x, double y)
{ 
  if (RESET)
  {
    compensation_lat = (double)pgm_read_float_far(&data[0].lon)-x;
    compensation_lon = (double)pgm_read_float_far(&data[0].lat)-y;
    height = (double)pgm_read_float_far(&data[0].h);
    
    yPos = (double)(pgm_read_float_far(&data[0].lat));
    xPos = (double)(pgm_read_float_far(&data[0].lon));
    dist = 0;
  }else 
  {
    MapMatching::calibrate_GPS(x+compensation_lon, y+compensation_lat);
    MapMatching::calibrate_GPS(x+compensation_lon, y+compensation_lat);
  }
}

void MapMatching::calibrate_GPS(double x, double y)
{
  int16_t i;
  double delta, minimum;
  
  delta = sq((y-(double)pgm_read_float_far(&data[0].lat))*DEG_2_METER) + sq((x-(double)pgm_read_float_far(&data[0].lon))*DEG_2_METER);
  minimum = delta;

  for (i=0; i<MAX; i++)
  {
      delta = sq((y-(double)pgm_read_float_far(&data[i].lat))*DEG_2_METER) + sq((x-(double)pgm_read_float_far(&data[i].lon))*DEG_2_METER);
      
      if (delta <= minimum)
      {
          latitude = (double)pgm_read_float_far(&data[i].lat);
          longitude = (double)pgm_read_float_far(&data[i].lon);
          height = (double)pgm_read_float_far(&data[i].h);
          minimum = delta;
      }
  }
  
  dist = sqrt((latitude-y)*DEG_2_METER*(latitude-y)*DEG_2_METER+(longitude-x)*DEG_2_METER*(longitude-x)*DEG_2_METER);
  
  if (dist <= cali_dist) 
  {
    compensation_lat = latitude-y;
    compensation_lon = longitude-x;
    y += compensation_lat;
    x += compensation_lon;
    xPos = x;
    yPos = y;
  }else 
  {
    xPos = x;
    yPos = y;
  }
}

double MapMatching::distance()
{
  return dist;
}

float MapMatching::calibrate_BARO()
{
    return height;
}
