#include "HEIGHT_DATA.h"

uint16_t planeEquation(uint16_t p1[], uint16_t p2[], uint16_t p3[], uint16_t x, int y)//x Latitude y longitude
{
  uint16_t r[3] = {p2[0]-p1[0],p2[1]-p1[1],p2[2]-p1[2]};
  uint16_t s[3] = {p3[0]-p1[0],p3[1]-p1[1],p3[2]-p1[2]};
  uint16_t n[3] = {r[1]*s[2] - r[2]*s[1],r[2]*s[0] - r[0]*s[2],r[0]*s[1] - r[1]*s[0]};

  static int HEIGHT;
  HEIGHT = p1[2] - ((n[0]*(y - p1[0]) + n[1]*(x - p1[1]))/n[2]);
  return HEIGHT;
}

uint16_t search(uint16_t x, uint16_t y)
{
  int8_t counter = -1;
  uint16_t longs[41];
  uint16_t hs[41];
  uint16_t i;
  
  for (i=0; i<MAX; i++)
  {
      if (x == (int)(pgm_read_word(&height[i].lat)))
      {
          counter++;
          longs[counter] = (int)(pgm_read_word(&height[i].lon));
          hs[counter] = (int)(pgm_read_word(&height[i].h));
      }
  }
  
  uint8_t c,location=0;
  for (c = 0; c < counter; c++)
  {     
      if (y == longs[c])
      {
        location = c;
        break;
      }
  }
  return hs[location];
}

uint16_t searchPrecise(float x, float y)
{
  int16_t counter = -1;
  uint16_t flag = 0;
  uint16_t latStore = 0;
  static uint16_t new_x;
  static uint16_t new_y;
  new_x = floor((x-4600)/60*1000);
  new_y = floor((y-700)/60*100000);
  
  uint16_t diff1 = 0;
    
  if (924 > new_x) diff1 = 924 - new_x;//924 is the first number in my dataset
  else diff1 = new_x - 924;
  
  uint16_t i, minimum1 = diff1;
  
  for (i=1; i<MAX; i++)
  {
      if ((uint16_t)(pgm_read_word(&height[i].lat)) > new_x) diff1 = (uint16_t)(pgm_read_word(&height[i].lat)) - new_x;
      else diff1 = new_x - (uint16_t)(pgm_read_word(&height[i].lat));

      uint16_t z = i;
      if (diff1 <= minimum1)
      {
          counter++;
          minimum1 = diff1;
          latStore = (uint16_t)(pgm_read_word(&height[i].lat));
          lonStore[counter] = (uint16_t)(pgm_read_word(&height[i].lon));
          hStore[counter] = (uint16_t)(pgm_read_word(&height[i].h));
      }
  }
  
  diff1 = 0;
  
  if (lonStore[0] > new_y) diff1 = lonStore[0] - new_y;
  else diff1 = new_y - lonStore[0];
  
  uint16_t minimum = diff1;
  uint8_t c,location=1;
  
  for (c = 1; c < counter; c++)
  {
      if (lonStore[c] > new_y) diff1 = lonStore[c] - new_y;
      else diff1 = new_y - lonStore[c];
      
      if (diff1 < minimum)
      {
          minimum = diff1;
          location = c+1;
      }
  }

  uint8_t pos;
  uint16_t u, loc;
  
  for (u=0;u<21;u++)
  {
    if (latitude[u] == latStore)
    {
      pos = u;
      break;
    }
  }

  for (u=0;u<MAX;u++)
  {
    if (height[u].lon == lonStore[location-1])
    {
      loc = u;
      break;
    }
  }
  
  uint16_t latStore1 = ((latitude[pos-1] - latStore) > (latStore - latitude[pos+1])) ? latitude[pos+1] : latitude[pos-1];
  uint16_t lonStore1 = ((height[loc+1].lon - lonStore[location-1]) > (lonStore[location-1] - height[loc-1].lon)) ? height[loc-1].lon : height[loc+1].lon;
  uint16_t hStore1 = search(latStore, lonStore1);
  uint16_t hStore2 = search(latStore1, lonStore[location-1]);
  
  static uint16_t point1[3];
  point1[0] = lonStore[location-1];
  point1[1] = latStore;
  point1[2] = hStore[location-1];
  static uint16_t point2[3];
  point2[0] = lonStore[location-1];
  point2[1] = latStore1;
  point2[2] = hStore2;
  static uint16_t point3[3];
  point3[0] = lonStore1;
  point3[1] = latStore;
  point3[2] = hStore1;
  
  uint16_t HEIGHT = planeEquation(point1,point2,point3, new_x, new_y);
  return HEIGHT;
}
