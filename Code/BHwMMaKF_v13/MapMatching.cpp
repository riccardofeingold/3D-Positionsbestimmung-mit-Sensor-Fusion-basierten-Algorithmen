#include "MapMatching.h"
#include "MyFilterEstimator.h"
#define DEG_2_METER 111139
MyFilterEstimator filter;

double MapMatching::begin(double x, double y, bool lat_or_lon)
{
  //Here x is latitude and y is longitude!
  double new_x = x, new_y = y;

  //Setting the inital valus for the filter 
  filter.begin(y, x);
  
  if (RESET)
  {
    compensation_lat = (double)pgm_read_float_far(&data[0].lat)-new_x;
    compensation_lon = (double)pgm_read_float_far(&data[0].lon)-new_y;
    height = (double)pgm_read_float_far(&data[0].h);
    
    switch (lat_or_lon)
    {
      case 1:
        return (double)(pgm_read_float_far(&data[0].lat));
      break;
      case 0:
        return (double)(pgm_read_float_far(&data[0].lon));
      break;
    }
  }else 
  {
    new_x = MapMatching::calibrate_GPS(new_x+compensation_lat, new_y+compensation_lon, true);
    new_y = MapMatching::calibrate_GPS(new_x+compensation_lat, new_y+compensation_lon, false);
    switch (lat_or_lon)
    {
      case 1:
        return new_x;
      break;
      case 0:
        return new_y;
      break;
    }
  }
}

double MapMatching::calibrate_GPS(double x, double y, bool lat_or_lon)
{
  int16_t i;
  double delta, minimum;
  
  delta = sq((x-(double)pgm_read_float_far(&data[0].lat))*DEG_2_METER) + sq((y-(double)pgm_read_float_far(&data[0].lon))*DEG_2_METER);
  minimum = delta;

  for (i=0; i<MAX; i++)
  {
      delta = sq((x-(double)pgm_read_float_far(&data[i].lat))*DEG_2_METER) + sq((y-(double)pgm_read_float_far(&data[i].lon))*DEG_2_METER);
      
      if (delta <= minimum)
      {
          latitude = (double)pgm_read_float_far(&data[i].lat);
          longitude = (double)pgm_read_float_far(&data[i].lon);
          height = (double)pgm_read_float_far(&data[i].h);
          minimum = delta;
      }
  }
  
  dist = sqrt((latitude-x)*DEG_2_METER*(latitude-x)*DEG_2_METER+(longitude-y)*DEG_2_METER*(longitude-y)*DEG_2_METER);
  
  if (dist <= cali_dist) 
  {
    compensation_lat = latitude-x;
    compensation_lon = longitude-y;
    x += compensation_lat;
    y += compensation_lon;

    switch (lat_or_lon)
    {
      case 1:
        return x;
      break;
      case 0:
        return y;
      break;
    }
  }
  else 
  {
    switch (lat_or_lon)
    {
      case 1:
        return x;
      break;
      case 0:
        return y;
      break;
    }
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
