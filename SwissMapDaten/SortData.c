#include <stdio.h>

float lat, lon, height;
float lat_max, lat_min, lon_max, lon_min;

int main(void)
{
    FILE *f = fopen("DatasetOfCH.csv","r");
    FILE *g = fopen("dataset.txt","w");
    
    //ask for users input data
    //Geben Sie die Koordinate (lat/lon) der oberen linken Ecke und von Ihrer ausgewählten Umgebung auf Swissmap (siehe Kap. 6.2.4):
    lat_max = 46.90606;
    lon_min = 7.38621;
    
    //Geben Sie die Koordinate (lat/lon) der unteren rechten Ecke von Ihrer ausgewählten Umgebung auf Swissmap:
    lat_min = 46.89707;
    lon_max = 7.39958;
    
    printf("Working...");
    
    while (!feof(f))
    {
        fscanf(f,"%f %f %f\n",&lon, &lat, &height);
        if (lat < lat_max && lat > lat_min)
        {
            if (lon <= lon_max && lon >= lon_min)
            {
                fprintf(g,"%f,%f,%f,\n",lat,lon,height);
            }
        }
    }
    printf("Finished!");
    return 0;
}
