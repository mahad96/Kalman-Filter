//
//  kalman_filter_optimized.c
//  KalmanFilter
//
//  Created by Mahad Khan on 2020-05-01.
//  Copyright © 2020 Mahad Khan. All rights reserved.
//

//NOTE: This file should not be used to test performance.
//This is only to see how the algorithm works.
//The print statements in the loop are not critical to the code but for visual representation of results
//And they slow down the algorithm
//However, if the same random values are generated as those in kalman_filter.c, this will be faster.

#include "kalman_filter_optimized.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

//return a random number between -1 and 1
float frand() {
    return 2*((rand()/(float)RAND_MAX) - 0.5);
}

//this makes observations normal about x and with sigma 0.1
//to optimize we perform this operation in an inline function
static float inline normal_distribution(float x){
    return x + frand()*0.1;
}

int main() {

    //initial guesses
    float xhat_last = 0;
    float P_last = 1;
    
    
    float Q = powf(10, -5); //process variance
    float R = 0.01; //estimate of measurement variance, change to see effect
    
    //initial parameters (we run the algorithm for 50 random values
    int n_iter = 50;
    float x = -0.37727;
    
    float K; //gain or blending factor
    float P; //a poster error estimate
    float Pminus; //a priori error estimate
    float xhatminus; //a priori estimate of x
    float xhat; //a posteri estimate of x
    float z; //observations
    
    clock_t start_t, end_t;
    
    //seed is 0
    srand(0);
  
    float total_error_kalman = 0;
    float total_error_measure = 0;
    
    //create variable to store sum of R and Q to avoid extra addition operation in for loop
    float sum_R_Q = R+Q;
    
    start_t = clock();
    
    for (int i=0;i<n_iter-4;i=i+5) {
        
        //unroll loop 5 times
        //swap statement Pminus = P_last + Q; with xhatminus = xhat_last; to avoid data dependency
        
        Pminus = P_last + Q;
        xhatminus = xhat_last;
        K = Pminus * (1.0/(P_last + sum_R_Q));
        z = normal_distribution(x);
        xhat = xhatminus + K * (z - xhatminus);
        P = (1- K) * Pminus;

        printf("Ideal    position: %6.3f \n",x);
        printf("Mesaured position: %6.3f [diff:%.3f]\n",z,fabs(x-z));
        printf("Kalman   position: %6.3f [diff:%.3f]\n",xhat,fabs(x - xhat));
        
        total_error_kalman += fabs(x - xhat);
        total_error_measure += fabs(x-z);
        
        //update our last's because values are randomly generated and not stored in a data structure
        P_last = P;
        xhat_last = xhat;
        
        Pminus = P_last + Q;
        xhatminus = xhat_last;
        K = Pminus * (1.0/(P_last + sum_R_Q));
        z = normal_distribution(x);
        xhat = xhatminus + K * (z - xhatminus);
        P = (1- K) * Pminus;

        printf("Ideal    position: %6.3f \n",x);
        printf("Mesaured position: %6.3f [diff:%.3f]\n",z,fabs(x-z));
        printf("Kalman   position: %6.3f [diff:%.3f]\n",xhat,fabs(x - xhat));
            
        total_error_kalman += fabs(x - xhat);
        total_error_measure += fabs(x-z);
            
        //update our last's because values are randomly generated and not stored in a data structure
        P_last = P;
        xhat_last = xhat;
        
        Pminus = P_last + Q;
        xhatminus = xhat_last;
        K = Pminus * (1.0/(P_last + sum_R_Q));
        z = normal_distribution(x);
        xhat = xhatminus + K * (z - xhatminus);
        P = (1- K) * Pminus;

        printf("Ideal    position: %6.3f \n",x);
        printf("Mesaured position: %6.3f [diff:%.3f]\n",z,fabs(x-z));
        printf("Kalman   position: %6.3f [diff:%.3f]\n",xhat,fabs(x - xhat));
            
        total_error_kalman += fabs(x - xhat);
        total_error_measure += fabs(x-z);
            
        //update our last's because values are randomly generated and not stored in a data structure
        P_last = P;
        xhat_last = xhat;
        
        Pminus = P_last + Q;
        xhatminus = xhat_last;
        K = Pminus * (1.0/(P_last + sum_R_Q));
        z = normal_distribution(x);
        xhat = xhatminus + K * (z - xhatminus);
        P = (1- K) * Pminus;

        printf("Ideal    position: %6.3f \n",x);
        printf("Mesaured position: %6.3f [diff:%.3f]\n",z,fabs(x-z));
        printf("Kalman   position: %6.3f [diff:%.3f]\n",xhat,fabs(x - xhat));
            
        total_error_kalman += fabs(x - xhat);
        total_error_measure += fabs(x-z);
            
        //update our last's because values are randomly generated and not stored in a data structure
        P_last = P;
        xhat_last = xhat;
        
        Pminus = P_last + Q;
        xhatminus = xhat_last;
        K = Pminus * (1.0/(P_last + sum_R_Q));
        z = normal_distribution(x);
        xhat = xhatminus + K * (z - xhatminus);
        P = (1- K) * Pminus;

        printf("Ideal    position: %6.3f \n",x);
        printf("Mesaured position: %6.3f [diff:%.3f]\n",z,fabs(x-z));
        printf("Kalman   position: %6.3f [diff:%.3f]\n",xhat,fabs(x - xhat));
            
        total_error_kalman += fabs(x - xhat);
        total_error_measure += fabs(x-z);
            
        //update our last's because values are randomly generated and not stored in a data structure
        P_last = P;
        xhat_last = xhat;
        
        
        
    }
    
    end_t = clock();
    
    printf("Elapsed time %5fs\n", ((double)end_t - (double)start_t)/CLOCKS_PER_SEC);
    printf("Total error if using noisy measurements:  %f\n",total_error_measure);
    printf("Total error if using Kalman Filter: %f\n",total_error_kalman);
    printf("Reduction in error: %d%% \n",100-(int)((total_error_kalman/total_error_measure)*100));
    
    return 0;
}
