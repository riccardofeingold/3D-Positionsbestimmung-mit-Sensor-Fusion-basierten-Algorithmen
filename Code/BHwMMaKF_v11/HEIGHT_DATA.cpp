#include "HEIGHT_DATA.h"
#define DEG_2_METER 111139

int HeightModel::search(float x, float y)
{
    //x stands for longitude and y for latitude
    
    //Reset counter
    counter = 0;
    
    int i,j,n=2;
    //stores the previous squared distance
    unsigned long diff_prev = 0;

    //Store the x and y distances in meters
    long x_dist = (float)(x-pgm_read_float_far(&dataset[0].x))*DEG_2_METER;
    long y_dist = (float)(y-pgm_read_float_far(&dataset[0].y))*DEG_2_METER;
    Serial.println(x_dist);

    //StortsvektorVres the current squared distance
    unsigned long diff_current = sq(x_dist) + sq(y_dist);

    //StortsvektorVres the calculated height 
    int height = 0;

    diff_prev = diff_current;

    for (i=0;i<MAX;i++)
    {
        x_dist = (float)(x-pgm_read_float_far(&dataset[i].x))*DEG_2_METER;
        y_dist = (float)(y-pgm_read_float_far(&dataset[i].y))*DEG_2_METER;
        diff_current = x_dist*x_dist + y_dist*y_dist;

        if (diff_current <= diff_prev)
        {
            diff_prev = diff_current;
            Serial.println(pgm_read_float_far(&dataset[i].z));
            Serial.println(pgm_read_float_far(&dataset[i].y),5);
            Serial.println(pgm_read_float_far(&dataset[i].x),5);
            
            //The three points can only be at least 282.8 m away. This further law helps the program to not chose three points on the same line
            if (diff_prev<distance)
            {
              Serial.println(diff_prev);
              switch (counter)
              {
                case 0:
                  height_1 = pgm_read_float_far(&dataset[i].z);
                  lat_1 = pgm_read_float_far(&dataset[i].y);
                  lon_1 = pgm_read_float_far(&dataset[i].x);
                break;
                
                case 1:
                  height_2 = pgm_read_float_far(&dataset[i].z);
                  lat_2 = pgm_read_float_far(&dataset[i].y);
                  lon_2 = pgm_read_float_far(&dataset[i].x);
                break;
  
                case 2:
                  height_3 = pgm_read_float_far(&dataset[i].z);
                  lat_3 = pgm_read_float_far(&dataset[i].y);
                  lon_3 = pgm_read_float_far(&dataset[i].x);
  
                  //Reset cortsvektorVunter
                  counter = -1;
                break;
              }
              counter++;
            }
        }
    }
    
    height = HeightModel::height(x,y);
    return height;
}

int HeightModel::height(float x, float y) 
{
  //to store the calculated height
  int h = 0;

  //to store the ortsvektor
  float ortsvektor[3];

  //to store the Richtungsvektor 1
  float r1[3];

  //to store the Richtungsvektor 2
  float r2[3];

  //to store the two not-nearest reference points
  float p1[3];
  float p2[3];

  switch (counter)
  {
    case 0:
      ortsvektor[0] = lon_1;
      ortsvektor[1] = lat_1;
      ortsvektor[2] = height_1;
      
      p1[0] = lon_2;
      p1[1] = lat_2;
      p1[2] = height_2;
      
      p2[0] = lon_3;
      p2[1] = lat_3;
      p2[2] = height_3;
    break;

    case 1:
      ortsvektor[0] = lon_2;
      ortsvektor[1] = lat_2;
      ortsvektor[2] = height_2;

      p1[0] = lon_1;
      p1[1] = lat_1;
      p1[2] = height_1;
      
      p2[0] = lon_3;
      p2[1] = lat_3;
      p2[2] = height_3;
    break;

    case 2: 
      ortsvektor[0] = lon_3;
      ortsvektor[1] = lat_3;
      ortsvektor[2] = height_3;

      p1[0] = lon_2;
      p1[1] = lat_2;
      p1[2] = height_2;
      
      p2[0] = lon_1;
      p2[1] = lat_1;
      p2[2] = height_1;
    break;
  }
  int i;
  Serial.print("C: ");
  Serial.println(counter);
  Serial.print("LON: ");
  Serial.println(x,8);
  Serial.print("LAT: ");
  Serial.println(y,8);
  
  for (i=0;i<3;i++)
  {
    Serial.println(p1[i],8);
  }
  for (i=0;i<3;i++)
  {
    Serial.println(p2[i],8);
  }
  for (i=0;i<3;i++)
  {
    Serial.println(ortsvektor[i],8);
  }
  //Calculate the Richtungsvektoren
  r1[0] = p1[0] - ortsvektor[0];
  r1[1] = p1[1] - ortsvektor[1];
  r1[2] = p1[2] - ortsvektor[2];

  r2[0] = p2[0] - ortsvektor[0];
  r2[1] = p2[1] - ortsvektor[1];
  r2[2] = p2[2] - ortsvektor[2];

  //Calculate the normal vector
  float n[3];
  n[0] = r1[1]*r2[2] - r2[1]*r1[2];
  n[1] = r1[2]*r2[0] - r2[2]*r1[0];
  n[2] = r1[0]*r2[1] - r2[0]*r1[1];

  //Calculate the height
  h = -((n[0]*(x-ortsvektor[0])+n[1]*(y-ortsvektor[1])-n[2]*ortsvektor[2])/n[2]);
  return h;
}
