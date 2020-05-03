//
//  kalman_filter_test.c
//  KalmanFilter
//
//  Created by Mahad Khan on 2020-05-01.
//  Copyright © 2020 Mahad Khan. All rights reserved.
//

#include "kalman_filter_test.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>


typedef struct Array{
    long double* data;
    int n;
} Array;

// function to read input test data from text file
void read_data(FILE *fp, Array* array) {
   int n_char = 0;
   float val = 0.0;
   while(fscanf(fp, "%f", &val) == 1) {
       n_char++;
   }
   printf("Testing with %d input values\n", n_char);

   array->data = malloc(n_char*sizeof(long double));
   array->n = n_char;

   rewind(fp);
   for(int i=0; i<n_char; i++) {
       fscanf(fp, "%Lf", &(array->data[i]));
   }
}

int main() {
    
    //read data from values.txt and store in test_data
    Array test_data;
    FILE* fp = fopen("values.txt", "r");
    read_data(fp, &test_data);
    fclose(fp);
    
    float Q = powf(10, -5);
    float R = 0.01;
    
    //long double has greater precision hence slower
    //pointers store data on heap which is slower
    long double* K = malloc(test_data.n * sizeof(long double));
    long double* P = malloc(test_data.n * sizeof(long double));
    long double* Pminus = malloc(test_data.n * sizeof(long double));
    long double* xhatminus = malloc(test_data.n * sizeof(long double));
    long double* xhat = malloc(test_data.n * sizeof(long double));
    long double* z = test_data.data;
    long double x = -0.37727;
    
    xhat[0] = 0.0;
    P[0] = 1.0;
    
    clock_t start_t, end_t;
    
    long double sum_error_kalman = 0;
    long double sum_error_measure = 0;
    
    start_t = clock();
    
    for (int i=1;i<test_data.n;i++) {
        
        xhatminus[i] = xhat[i-1];
        Pminus[i] = P[i-1] + Q;
        K[i] = Pminus[i] * (1.0/(Pminus[i] + R));
        xhat[i] = xhatminus[i] + K[i] * (z[i] - xhatminus[i]);
        P[i] = (1- K[i]) * Pminus[i];
        sum_error_kalman += fabsl(x - xhat[i]);
        sum_error_measure += fabsl(x-z[i]);
    
    }
    
    end_t = clock();
    
    printf("Elapsed time %5fs\n", ((double)end_t - (double)start_t)/CLOCKS_PER_SEC);
    printf("Total error if using noisy measurements:  %Lf\n",sum_error_measure);
    printf("Total error if using Kalman Filter: %Lf\n",sum_error_kalman);
    printf("Reduction in error: %d%% \n",100-(int)((sum_error_kalman/sum_error_measure)*100));
    
    return 0;
}

