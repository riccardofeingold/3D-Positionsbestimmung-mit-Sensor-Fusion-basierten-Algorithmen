#include "HEIGHT_DATA.h"
#define DEG_2_METER 111139
#define max_height 1000
float epsilon_0 = 0.0000000001;

int HeightModel::search(float x, float y)
{
    //x stands for longitude and y for latitude

    //to store the location of the first and second point
    int loc1 = 0;
    int loc2 = 0;
    
    int i,j,n=2;
    //stores the previous squared distance
    unsigned long diff_prev = 0;

    //Store the x and y distances in meters
    long x_dist = (float)(x-dataset[0].x)*DEG_2_METER;
    long y_dist = (float)(y-dataset[0].y)*DEG_2_METER;

    //StortsvektorVres the current squared distance
    unsigned long diff_current = pow(x_dist,2) + pow(y_dist,2);

    //StortsvektorVres the calculated height 
    int height = 0;

    diff_prev = diff_current;

    //Search the nearst point
    for (i=0;i<MAX;i++)
    {
        x_dist = (float)(x-dataset[i].x)*DEG_2_METER;
        y_dist = (float)(y-dataset[i].y)*DEG_2_METER;
        diff_current = x_dist*x_dist + y_dist*y_dist;

        if (diff_current <= diff_prev)
        {
            height_1 = dataset[i].z;
            lat_1 = dataset[i].y;
            lon_1 = dataset[i].x;
            
            diff_prev = diff_current;
            loc1 = i;
        }
    }

    //Reset of diff_current
    x_dist = (float)(x-dataset[0].x)*DEG_2_METER;
    y_dist = (float)(y-dataset[0].y)*DEG_2_METER;
    diff_current = pow(x_dist,2) + pow(y_dist,2);
    diff_prev = diff_current;
    
    //Search the second nearest point
    for (i=0;i<MAX;i++)
    {
       x_dist = (float)(x-dataset[i].x)*DEG_2_METER;
       y_dist = (float)(y-dataset[i].y)*DEG_2_METER;
       diff_current = x_dist*x_dist + y_dist*y_dist;
       
       if (i!=loc1)
       {
          if (diff_current <= diff_prev)
          {
              height_2 = dataset[i].z;
              lat_2 = dataset[i].y;
              lon_2 = dataset[i].x;
              diff_prev = diff_current;
              loc2 = i;
          }
       }
    }

    //Reset of diff_current
    x_dist = (float)(x-dataset[0].x)*DEG_2_METER;
    y_dist = (float)(y-dataset[0].y)*DEG_2_METER;
    diff_current = pow(x_dist,2) + pow(y_dist,2);
    diff_prev = diff_current;

    //Search the third nearest point
    for (i=0;i<MAX;i++)
    {
      x_dist = (float)(x-dataset[i].x)*DEG_2_METER;
      y_dist = (float)(y-dataset[i].y)*DEG_2_METER;
      diff_current = x_dist*x_dist + y_dist*y_dist;
      
      if (i!=loc1 && i!=loc2)
      {
          if (diff_current <= diff_prev)
          {
              height_3 = dataset[i].z;
              lat_3 = dataset[i].y;
              lon_3 = dataset[i].x;
  
              diff_prev = diff_current;
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

  //Assigning the values collected in search() to our vectors
  ortsvektor[0] = lon_1;
  ortsvektor[1] = lat_1;
  ortsvektor[2] = height_1;
  
  p1[0] = lon_2;
  p1[1] = lat_2;
  p1[2] = height_2;
  
  p2[0] = lon_3;
  p2[1] = lat_3;
  p2[2] = height_3;
  
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
  if (abs(n[2])<epsilon_0)
  {
    return ortsvektor[2];
  }else
  {
    h = -((n[0]*(x-ortsvektor[0])+n[1]*(y-ortsvektor[1])-n[2]*ortsvektor[2])/n[2]);
    if (h<0 || h>max_height) return ortsvektor[2];
    else return h;
  }
}

HeightModel model;
float lon,lat;

int main()
{
    while (1)
    {
        cin >> lon;
        cin >> lat;
        float height = model.search(lon,lat);
        cout << height;
    }
    return 0;
}
