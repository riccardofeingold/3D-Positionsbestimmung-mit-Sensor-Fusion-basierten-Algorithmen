#include "MapMatching.h"

double MapMatching::begin(double x, double y, bool lat_or_lon)
{
  double new_x, new_y;
  new_x = (x-4600)/60 + 46;
  new_y = (y-700)/60 + 7;
  if (RESET)
  {
    compensation_lat = (double)(pgm_read_float_far(&data[0].lat))-new_x;
    compensation_lon = (double)(pgm_read_float_far(&data[0].lon))-new_y;
    height = (double)(pgm_read_float_far(&data[0].h));
    
    switch (lat_or_lon)
    {
      case 1:
        return (double)(pgm_read_float_far(&data[0].lat));
      break;
      case 0:
        return (double)(pgm_read_float_far(&data[0].lon));
      break;
    }
    RESET = !RESET;
  }else 
  {
    compensation_lat += factor_lat;
    compensation_lon += factor_lon;
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
    if (dist <= cali_dist) return height;
}
