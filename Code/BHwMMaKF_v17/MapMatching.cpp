#include "MapMatching.h"
#include "MyFilterEstimator.h"
#define DEG_2_METER 111139

void MapMatching::begin(double x, double y, bool calibrate)
{ 
  int16_t i;
  double delta, minimum;
  inside_cal_area = true;
  
  //Calculating a first distance
  delta = sq((y-pgm_read_float_far(&data[0].lat))*DEG_2_METER) + sq((x-pgm_read_float_far(&data[0].lon))*DEG_2_METER);
  minimum = delta;

  //Searching for the nearest point in the dataset
  for (i=0; i<MAX; i++)
  {
    delta = sq((y-pgm_read_float_far(&data[i].lat))*DEG_2_METER) + sq((x-pgm_read_float_far(&data[i].lon))*DEG_2_METER);
    
    if (delta <= minimum)
    {
        latitude = pgm_read_float_far(&data[i].lat);
        longitude = pgm_read_float_far(&data[i].lon);
        height = pgm_read_float_far(&data[i].h);
        minimum = delta;
    }
  }
  
  if (calibrate)
  {
    compensation_lat = latitude-y;
    compensation_lon = longitude-x;
  }else
  {
    compensation_lat = 0;
    compensation_lon = 0;
  }
  
  yPos = latitude;
  xPos = longitude;
  dist = 0;
}

void MapMatching::calibrate_GPS(double x, double y, bool calibrate)
{
  int16_t i;
  double delta, minimum;

  //Calculating a first distance
  delta = sq((y-pgm_read_float_far(&data[0].lat))*DEG_2_METER) + sq((x-pgm_read_float_far(&data[0].lon))*DEG_2_METER);
  minimum = delta;

  //Searching for the nearest point in the dataset
  for (i=0; i<MAX; i++)
  {
    delta = sq((y-pgm_read_float_far(&data[i].lat))*DEG_2_METER) + sq((x-pgm_read_float_far(&data[i].lon))*DEG_2_METER);
    
    if (delta <= minimum)
    {
        latitude = pgm_read_float_far(&data[i].lat);
        longitude = pgm_read_float_far(&data[i].lon);
        height = pgm_read_float_far(&data[i].h);
        minimum = delta;
    }
  }

  //calculating the exact distance between the nearest point and user's position
  dist = sqrt((latitude-(y+compensation_lat))*DEG_2_METER*(latitude-(y+compensation_lat))*DEG_2_METER+(longitude-(x+compensation_lon))*DEG_2_METER*(longitude-(x+compensation_lon))*DEG_2_METER);
  
  if (dist <= cali_dist && calibrate) 
  {
    if (!inside_cal_area)
    {
      compensation_lat = latitude-y;
      compensation_lon = longitude-x;
      xPos = x+compensation_lon;
      yPos = y+compensation_lat;
      inside_cal_area = true;
    }
  }else if (dist >= cali_dist && calibrate) inside_cal_area = false;
  else 
  {
    xPos = x+compensation_lon;
    yPos = y+compensation_lat;
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
