#include "MapMatching.h"

double MapMatching::begin(double x, double y, bool lat_or_lon)
{
  double new_x, new_y;
  new_x = (x-4600)/60 + 46;
  new_y = (y-700)/60 + 7;
  Serial.begin(9600);
  if (RESET)
  {
    compensation_lat = (double)(pgm_read_float_far(&data[0].lat))-new_x;
    compensation_lon = (double)(pgm_read_float_far(&data[0].lon))-new_y;
    height = (double)(pgm_read_float_far(&data[0].h));
    
    switch (lat_or_lon)
    {
      case 1:
        return (double)(pgm_read_float_far(&data[0].lat));
      case 0:
        return (double)(pgm_read_float_far(&data[0].lon));
    }
    RESET = !RESET;
  }else 
  {
    new_x = MapMatching::calibrate_GPS(new_x+compensation_lat, new_y+compensation_lon, true);
    new_y = MapMatching::calibrate_GPS(new_x+compensation_lat, new_y+compensation_lon, false);
    switch (lat_or_lon)
    {
      case 1:
        return new_x;
      case 0:
        return new_y;
    }
  }
}

double MapMatching::calibrate_GPS(double x, double y, bool lat_or_lon)
{
  double cali_dist = 0.00008;
  int16_t i;
  double delta_x, delta_y, minimum_x, minimum_y;
  
  delta_x = abs(x-(double)(pgm_read_float_far(&data[0].lat)));
  delta_y = abs(y-(double)(pgm_read_float_far(&data[0].lon)));
  minimum_x = delta_x;
  minimum_y = delta_y;

  for (i=0; i<MAX; i++)
  {
      delta_x = abs(x-(double)(pgm_read_float_far(&data[i].lat)));
      delta_y = abs(y-(double)(pgm_read_float_far(&data[i].lon)));
      
      if (delta_x <= minimum_x && delta_y <= minimum_x)
      {
          latitude = (double)(pgm_read_float_far(&data[i].lat));
          longitude = (double)(pgm_read_float_far(&data[i].lon));
          height = (double)(pgm_read_float_far(&data[i].h));
      }
  }
  dist = sqrt(pow(latitude-x,2)+pow(longitude-y,2));
  
  if (dist <= cali_dist) 
  {
    factor_lat = latitude-x;
    factor_lon = longitude-y;
    x += factor_lat;
    y += factor_lon;
    switch (lat_or_lon)
    {
      case 1:
        return x;
      case 0:
        return y;
    }
  }
  else 
  {
    switch (lat_or_lon)
    {
      case 1:
        return x;
      case 0:
        return y;
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
