//
//  kalman_filter.c
//  KalmanFilter
//
//  Created by Mahad Khan on 2020-05-01.
//  Copyright © 2020 Mahad Khan. All rights reserved.
//

//NOTE: This file should not be used to test performance.
//This is only to see how the algorithm works.
//The print statements in the loop are not critical to the code but for visual representation of results
//And they slow down the algorithm

#include "kalman_filter.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

//return a random number between -1 and 1
double frand() {
    return 2*((rand()/(double)RAND_MAX) - 0.5);
}

int main() {

    //initial guesses
    long double xhat_last = 0;
    long double P_last = 1;
    
    
    long double Q = powf(10, -5); //process variance
    long double R = 0.01; //estimate of measurement variance, change to see effect
    
    //initial parameters (we run the algorithm for 50 random values
    int n_iter = 50;
    long double x = -0.37727;
    
    long double K; //gain or blending factor
    long double P; //a poster error estimate
    long double Pminus; //a priori error estimate
    long double xhatminus; //a priori estimate of x
    long double xhat; //a posteri estimate of x
    long double z; //observations
    
    clock_t start_t, end_t;
    
    //seed is 0
    srand(0);
  
    long double total_error_kalman = 0;
    long double total_error_measure = 0;
    
    start_t = clock();
    
    for (int i=0;i<n_iter;i++) {
        
        //time update
        xhatminus = xhat_last;
        Pminus = P_last + Q;
        
        //measurement update
        K = Pminus * (1.0/(Pminus + R));
        z = x + frand()*0.1; //this makes observations normal about x and with sigma 0.1
        xhat = xhatminus + K * (z - xhatminus);
        P = (1- K) * Pminus;
        
        printf("Ideal    position: %6.3Lf \n",x);
        printf("Mesaured position: %6.3Lf [diff:%.3Lf]\n",z,fabsl(x-z));
        printf("Kalman   position: %6.3Lf [diff:%.3Lf]\n",xhat,fabsl(x - xhat));
        
        total_error_kalman += fabsl(x - xhat);
        total_error_measure += fabsl(x-z);
        
        //update our last's because values are randomly generated and not stored in a data structure
        P_last = P;
        xhat_last = xhat;
    }
    
    end_t = clock();
    
    
    printf("Elapsed time %2.5fs\n", ((double)end_t - (double)start_t)/CLOCKS_PER_SEC);
    printf("Total error if using noisy measurements:  %Lf\n",total_error_measure);
    printf("Total error if using Kalman Filter: %Lf\n",total_error_kalman);
    printf("Reduction in error: %d%% \n",100-(int)((total_error_kalman/total_error_measure)*100));
    
    
    return 0;
}
