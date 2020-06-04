#include <stdio.h>

double skip, lon, lat, height;
int val4 = 0, val5 = 0;

int main()
{
    FILE *f = fopen("DELTAH26.csv", "r");
    FILE *g = fopen("lon.txt","w");
    FILE *h = fopen("lat.txt","w");
    FILE *i = fopen("height.txt","w");
    
    int r;
    for (r=0;r<2206;r++)
    {
        fscanf(f,"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\n",&skip, &skip,&lon,&lat,&skip,&skip,&skip,&skip,&skip,&skip,&height,&skip,&skip,&skip,&skip,&skip);
        fprintf(g,"%lf,%lf\n",lon,lat);
        fprintf(h,"%lf,",lat);
        fprintf(i,"%lf,",height);
    }
    printf("\n");
    return 0;
}
