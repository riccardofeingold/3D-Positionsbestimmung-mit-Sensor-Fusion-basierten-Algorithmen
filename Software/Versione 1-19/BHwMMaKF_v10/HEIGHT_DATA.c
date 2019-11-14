#include "HEIGHT_DATA.h"

float search(double x, double y)
{
    int i,j,n=2,counter = 0;
    double diff_current_x = 0, diff_prev_x = 10, diff_current_y = 0, diff_prev_y = 10;

    double *height1 = (double*) malloc(n*sizeof(double));
    double *height2;

    for (i=0;i<MAX;i++)
    {
        diff_current_x = fabs(x-(double)(pgm_read_float_far(&dataset[i].x)));
        diff_current_y = fabs(y-(double)(pgm_read_float_far(&dataset[i].y)));
        
        if ((diff_prev_x >= diff_current_x) && (diff_prev_y >= diff_current_y))
        {
            diff_prev_x = diff_current_x;
            diff_prev_y = diff_current_y;
            if (counter==n)
            {
                height2 = (double*) malloc(n*2*sizeof(double));
                for (j=0;j<n;j++)
                {
                    height2[i] = height1[i];
                }
                free(height1);
                height1 = height2;
            }
            height1[counter] = (double)(pgm_read_float_far(&dataset[i].z));
            counter++;
        }
    }
    int sum = 0;
    int t = counter;
    counter = 0;
    float height = 0;
    while (t>0)
    {
        if (height1[t]>0)
        {
            sum+=height1[t];
            counter++;
            if (counter==1)
            {
                height = height1[t];
            }
        }
        t--;
    }
    float ave = sum/counter;
    return height;
}
