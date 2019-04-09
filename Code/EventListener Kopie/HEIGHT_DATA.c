#include "HEIGHT_DATA.h"

int search(float x, float y)
{
    int counter = -1;
    int new_x = floor((x-4600)/60*1000);
    long new_y = floor((y-700)/60*100000);
    int diff1 = 0;
    
    if (924 > new_x) diff1 = 924 - new_x;//924 is the first number in my dataset
    else diff1 = new_x - 924;
    
    int i, minimum1 = diff1;
    
    for (i=1; i<MAX; i++)
    {
        if ((int)(pgm_read_word(&height[i].lat)) > new_x) diff1 = (int)(pgm_read_word(&height[i].lat)) - new_x;
        else diff1 = new_x - (int)(pgm_read_word(&height[i].lat));

        if (diff1 <= minimum1)
        {
            counter++;
            minimum1 = diff1;
            lonStore[counter] = (int)(pgm_read_word(&height[i].lon));
            hStore[counter] = (int)(pgm_read_word(&height[i].h));
        }
    }
    
    diff1 = 0;
    
    if (lonStore[0] > new_y) diff1 = lonStore[0] - new_y;
    else diff1 = new_y - lonStore[0];
    
    int minimum = diff1;
    int c,location=1;
    
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
    return hStore[location-1];
}
