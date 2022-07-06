# include <stdio.h>
# include "dataset.h"
# include "kalman_filter.h"

int main(){
    // One dimension kalman filter
    Kalman_1D state;
    float *data = NULL;
    int data_length = 0;
    float output = 0;

    // import data from dataset
    data = X;
    data_length = sizeof(X)/sizeof(float);

    // initialize kalman filter
    kalman1_init(&state, data[1], 5e2);
    for(int i=2; i<data_length; i++)
    {
        printf("%.2f", data[i]);
        output = kalman1_filter(&state, data[i]); // kalman filter predict
        printf(", %.2f", output);
    }

    return 0;
}
