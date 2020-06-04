#include <stdio.h>

float lat, lon, height;
float lat1,lon1,height1;

int main(void)
{
    FILE *f = fopen("GPSCoordinates.txt","r");
    FILE *h = fopen("DHM200.txt","r");
    FILE *g = fopen("dataset.txt","w");
    
    //ask for users input data
    
    while (!feof(f))
    {
        fscanf(f,"%f %f %f \n",&lat, &lon, &height);
        fscanf(h,"%f %f %f \n",&lat1, &lon1, &height1);
        
        fprintf(g,"%f,%f,%f,\n",lat,lon,height1);
    }
    return 0;
}

