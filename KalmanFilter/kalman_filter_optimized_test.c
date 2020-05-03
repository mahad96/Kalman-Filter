//
//  kalman_filter_optimized_test.c
//  KalmanFilter
//
//  Created by Mahad Khan on 2020-05-01.
//  Copyright © 2020 Mahad Khan. All rights reserved.
//

#include "kalman_filter_optimized_test.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

typedef struct Array{
    float* data;
    int n;
} Array;

//read data from input text
void read_data(FILE *fp, Array* array) {
   int n_char = 0;
   float val = 0.0;
   while(fscanf(fp, "%f", &val) == 1) {
       n_char++;
   }
   printf("Testing with %d input values\n", n_char);

   array->data = malloc(n_char*sizeof(float));
   array->n = n_char;

   rewind(fp);
   for(int i=0; i<n_char; i++) {
       fscanf(fp, "%f", &(array->data[i]));
   }
}



int main() {

    //read test data from values.txt and store in test_data
    Array test_data;
    FILE* fp = fopen("values.txt", "r");
    read_data(fp, &test_data);
    fclose(fp);
    
    float Q = powf(10, -5);
    float R = 0.01;
    
    //store data as float rather than long double
    //save data on stack rather than heap
    float K[test_data.n];
    float P[test_data.n];
    float Pminus[test_data.n];
    float xhatminus[test_data.n];
    float xhat[test_data.n];
    float* z = test_data.data;
    float x = -0.37727;
    
    xhat[0] = 0.0;
    P[0] = 1.0;
    
    clock_t start_t, end_t;
    
    float sum_error_kalman = 0;
    float sum_error_measure = 0;
    
    //create variable to store sum of R and Q to avoid extra addition operation in for loop
    float sum_R_Q = R+Q;
    
    start_t = clock();
    
    for (int i=1;i<test_data.n - 10;i=i+11) {
        
        //unroll loop 10 times
        //swap statement Pminus[i] = P[i-1] + Q; with xhatminus[i] = xhat[i-1]; to avoid data dependency
        
        Pminus[i] = P[i-1] + Q;
        xhatminus[i] = xhat[i-1];
        K[i] = Pminus[i] * (1.0/(P[i-1] + sum_R_Q)); //use variable sum_R_Q
        xhat[i] = xhatminus[i] + K[i] * (z[i] - xhatminus[i]);
        P[i] = (1- K[i]) * Pminus[i];
        sum_error_kalman += fabs(x - xhat[i]);
        sum_error_measure += fabs(x-z[i]);
        
        Pminus[i+1] = P[(i+1)-1] + Q;
        xhatminus[i+1] = xhat[(i+1)-1];
        K[i+1] = Pminus[i+1] * (1.0/(P[(i+1)-1] + sum_R_Q));
        xhat[i+1] = xhatminus[i+1] + K[i+1] * (z[i+1] - xhatminus[i+1]);
        P[i+1] = (1- K[i+1]) * Pminus[i+1];
        sum_error_kalman += fabs(x - xhat[i+1]);
        sum_error_measure += fabs(x-z[i+1]);
        
        Pminus[i+2] = P[(i+2)-1] + Q;
        xhatminus[i+2] = xhat[(i+2)-1];
        K[i+2] = Pminus[i+2] * (1.0/(P[(i+2)-1] + sum_R_Q));
        xhat[i+2] = xhatminus[i+2] + K[i+2] * (z[i+2] - xhatminus[i+2]);
        P[i+2] = (1- K[i+2]) * Pminus[i+2];
        sum_error_kalman += fabs(x - xhat[i+2]);
        sum_error_measure += fabs(x-z[i+2]);
        
        Pminus[i+3] = P[(i+3)-1] + Q;
        xhatminus[i+3] = xhat[(i+3)-1];
        K[i+3] = Pminus[i+3] * (1.0/(P[(i+3)-1] + sum_R_Q));
        xhat[i+3] = xhatminus[i+3] + K[i+3] * (z[i+3] - xhatminus[i+3]);
        P[i+3] = (1- K[i+3]) * Pminus[i+3];
        sum_error_kalman += fabs(x - xhat[i+3]);
        sum_error_measure += fabs(x-z[i+3]);
        
        Pminus[i+4] = P[(i+4)-1] + Q;
        xhatminus[i+4] = xhat[(i+4)-1];
        K[i+4] = Pminus[i+4] * (1.0/(P[(i+4)-1] + sum_R_Q));
        xhat[i+4] = xhatminus[i+4] + K[i+4] * (z[i+4] - xhatminus[i+4]);
        P[i+4] = (1- K[i+4]) * Pminus[i+4];
        sum_error_kalman += fabs(x - xhat[i+4]);
        sum_error_measure += fabs(x-z[i+4]);
        
        Pminus[i+5] = P[(i+5)-1] + Q;
        xhatminus[i+5] = xhat[(i+5)-1];
        K[i+5] = Pminus[i+5] * (1.0/(P[(i+5)-1] + sum_R_Q));
        xhat[i+5] = xhatminus[i+5] + K[i+5] * (z[i+5] - xhatminus[i+5]);
        P[i+5] = (1- K[i+5]) * Pminus[i+5];
        sum_error_kalman += fabs(x - xhat[i+5]);
        sum_error_measure += fabs(x-z[i+5]);
        
        Pminus[i+6] = P[(i+6)-1] + Q;
        xhatminus[i+6] = xhat[(i+6)-1];
        K[i+6] = Pminus[i+6] * (1.0/(P[(i+6)-1] + sum_R_Q));
        xhat[i+6] = xhatminus[i+6] + K[i+6] * (z[i+6] - xhatminus[i+6]);
        P[i+6] = (1- K[i+6]) * Pminus[i+6];
        sum_error_kalman += fabs(x - xhat[i+6]);
        sum_error_measure += fabs(x-z[i+6]);
        
        Pminus[i+7] = P[(i+7)-1] + Q;
        xhatminus[i+7] = xhat[(i+7)-1];
        K[i+7] = Pminus[i+7] * (1.0/(P[(i+7)-1] + sum_R_Q));
        xhat[i+7] = xhatminus[i+7] + K[i+7] * (z[i+7] - xhatminus[i+7]);
        P[i+7] = (1- K[i+7]) * Pminus[i+7];
        sum_error_kalman += fabs(x - xhat[i+7]);
        sum_error_measure += fabs(x-z[i+7]);
        
        Pminus[i+8] = P[(i+8)-1] + Q;
        xhatminus[i+8] = xhat[(i+8)-1];
        K[i+8] = Pminus[i+8] * (1.0/(P[(i+8)-1] + sum_R_Q));
        xhat[i+8] = xhatminus[i+8] + K[i+8] * (z[i+8] - xhatminus[i+8]);
        P[i+8] = (1- K[i+8]) * Pminus[i+8];
        sum_error_kalman += fabs(x - xhat[i+8]);
        sum_error_measure += fabs(x-z[i+8]);
        
        Pminus[i+9] = P[(i+9)-1] + Q;
        xhatminus[i+9] = xhat[(i+9)-1];
        K[i+9] = Pminus[i+9] * (1.0/(P[(i+9)-1] + sum_R_Q));
        xhat[i+9] = xhatminus[i+9] + K[i+9] * (z[i+9] - xhatminus[i+9]);
        P[i+9] = (1- K[i+9]) * Pminus[i+9];
        sum_error_kalman += fabs(x - xhat[i+9]);
        sum_error_measure += fabs(x-z[i+9]);
        
        Pminus[i+10] = P[(i+10)-1] + Q;
        xhatminus[i+10] = xhat[(i+10)-1];
        K[i+10] = Pminus[i+10] * (1.0/(P[(i+10)-1] + sum_R_Q));
        xhat[i+10] = xhatminus[i+10] + K[i+10] * (z[i+10] - xhatminus[i+10]);
        P[i+10] = (1- K[i+10]) * Pminus[i+10];
        sum_error_kalman += fabs(x - xhat[i+10]);
        sum_error_measure += fabs(x-z[i+10]);
        
    }
    end_t = clock();
    printf("Elapsed time %5fs\n", ((double)end_t - (double)start_t)/CLOCKS_PER_SEC);
    
    printf("Total error if using noisy measurements:  %f\n",sum_error_measure);
    printf("Total error if using Kalman Filter: %f\n",sum_error_kalman);
    printf("Reduction in error: %d%% \n",100-(int)((sum_error_kalman/sum_error_measure)*100));
    
    
    return 0;
}
