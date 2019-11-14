#include <stdio.h>

float lat, lon, height;
float lat_max, lat_min, lon_max, lon_min;

int main(void)
{
    FILE *f = fopen("DatasetOfCH.txt","r");
    FILE *g = fopen("dataset.txt","w");
    
    //ask for users input data
    printf("Geben Sie die Koordinate (lat/lon) der oberen linken Ecke von Ihrer ausgewählten Umgebung auf Swissmap (siehe Kap. 6.2.4): ");
    scanf("%f, %f", &lat_max, &lon_min);
    printf("Geben Sie die Koordinate (lat/lon) der unteren rechten Ecke von Ihrer ausgewählten Umgebung auf Swissmap: ");
    scanf("%f, %f", &lat_min, &lon_max);
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
