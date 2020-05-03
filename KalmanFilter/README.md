# Kalman Filters


## To simulate the Kalman filter (not for performance analysis)

### Baseline Kalman Filter
To compile the code, open Linux terminal, change directory to KalmanFilter/KalmanFilter and enter **gcc kalman_filter.c**
To run the code, enter **./a.out **

### Optimized Kalman Filter
To compile the code, open Linux terminal, change directory to KalmanFilter/KalmanFilter and enter **gcc kalman_filter_optimized.c**
To run the code, enter  **./a.out  **

## For performance analysis

### Baseline Kalman Filter
To compile the code, open Linux terminal, change directory to KalmanFilter/KalmanFilter and enter **gcc kalman_filter_test.c**
To run the code, enter **./a.out **

### Optimized Kalman Filter
To compile the code, open Linux terminal, change directory to KalmanFilter/KalmanFilter and enter **gcc kalman_filter_optimized_test.c**
To run the code, enter  **./a.out ** 

## To generate a new set of input test values
Open Linux terminal, change directory to KalmanFilter/KalmanFilter and enter **python generate_test_data.py**
This will save 1000 newly generated normally distributed values with given mean and standard deviation to values.txt

