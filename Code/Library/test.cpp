#include "MyFilterEstimator.h"

MyFilterEstimator filter;

float measurement_lon = 0;
float measurement_lat = 0;
float imu_lon = 0;
float imu_lat = 0;
int stop = 1;

int main()
{
    FILE *f = fopen("data_est.txt","w");
    FILE *g = fopen("data_gps.txt","w");
    FILE *h = fopen("data_imu.txt","w");
    
    printf("How many data points? ");
    scnaf("%d", &stop);
    
    printf("Enter intial values: \t");
    scanf("%f %f", &measurement_lon, &measurement_lat);
    filter.begin(measurement_lon, measurement_lat);
    
    int i;
    for (i=0;i<stop;i++)
    {
        printf("Measurement Input: \t");
        scanf("%f %f",&measurement_lon, &measurement_lat);
        printf("Model Input:L \t");
        scanf("%f %f", &imu_lon, &imu_lat);
        
        filter.measurementInput(measurement_lon, measurement_lat);
        filter.modelInput(imu_lon,imu_lat);
        
        float lon = filter.estimation_lon();
        float lat = filter.estimation_lat();
        
        fprintf(f, "%f, %f, \n", lon, lat);
        fprintf(g, "%f, %f, \n", measurement_lon, measurement_lat);
        fprintf(h, "%f, %f, \n", imu_lon, imu_lat);
        
    }
    return 1;
}
