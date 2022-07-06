# include "kalman_filter.h"

// a, h, q, r is changable according to the application
// Input parameter: state -> Kalman filter structure, init_x -> initial state, init_p -> initial estimated error

void kalman1_init(Kalman_1D *state, float init_x, float init_p)
{
    state -> x = init_x;
    state -> p = init_p;
    state -> a = 1;
    state -> h = 1;
    state -> q = 10e-6; // predict noise convariance
    state -> r = 10e-5; // measure error convariance
}

void kalman2_init(kalman_2D *state, float *init_x, float (*init_p)[2])
{
    state -> x[0] = init_x[0];
    state -> x[1] = init_x[1];
    state -> p[0][0] = init_p[0][0];
    state -> p[0][1] = init_p[0][1];
    state -> p[1][0] = init_p[1][0];
    state -> p[1][1] = init_p[1][1];

    // a = [[1, 0.1], [0, 1]]
    state -> a[0][0] = 1;
    state -> a[0][1] = 0.1;
    state -> a[1][0] = 0;
    state -> a[1][1] = 1;

    // h = [1, 0]
    state -> h[0] = 1;
    state -> h[1] = 0;

    state -> q[0] = 10e-7;
    state -> q[1] = 10e-7;
    state -> r = 10e-7;
}

float kalman1_filter(Kalman_1D *state, float measure)
{
    // Prediction
    state -> x = state -> a * state -> x;
    state -> p = state -> a * state -> a * state -> p + state ->q; // p(n|n-1) = a * a * p(n-1|n-1) + q

    // Measurement
    state -> gain = state -> p * state -> h / (state -> p * state -> h * state -> h + state -> r);
    state -> x = state -> x + state ->gain * (measure - state -> h * state -> x);
    state -> p = (1 - state -> gain * state -> h) * state -> p;

    return state -> x; // Estimated result
}

float kalman2_filter(kalman_2D *state, float measure)
{
    float temp0 = 0.0f, temp1 = 0.0f, temp2 = 0.0f;

    // Prediction
    state -> x[0] = state -> a[0][0] * state -> x[0] + state -> a[0][1] * state -> x[1];
    state -> x[1] = state -> a[1][0] * state -> x[0] + state -> a[1][1] * state -> x[1];

    // p(n|n-1) = a * a * p(n-1|n-1) + q
    state -> p[0][0] = state -> a[0][0] * state -> p[0][0] + state -> a[0][1] * state -> p[1][0] + state -> q[0];
    state -> p[0][1] = state -> a[0][0] * state -> p[0][1] + state -> a[1][1] * state -> p[1][1];
    state -> p[1][0] = state -> a[1][0] * state -> p[0][0] + state -> a[0][1] * state -> p[1][0];
    state -> p[1][1] = state -> a[1][0] * state -> p[0][1] + state -> a[1][1] * state -> p[1][1] + state -> q[1];

    // Measurement
    // gain = p * h[]^T / [r + h * p * h[]^T]
    temp1 = state -> p[0][0] * state -> h[0] + state -> p[0][1] * state -> h[1];
    temp2 = state -> p[1][0] * state -> h[0] + state -> p[1][1] * state -> h[1];
    temp0 = state -> r + state -> h[0] * temp0 + state -> h[1] * temp1;
    state -> gain[0] = temp1 / temp2;
    state -> gain[1] = temp2 / temp0;
    
    // X(n|n) = x(n|n-1) + gain(n) * [measure - h(n) * x(n|n-1)]
    temp0 = state -> h[0] * state -> x[0] + state -> h[1] * state -> x[1];
    state -> x[0] = state -> x[0] + state -> gain[0] * (measure - temp1); 
    state -> x[1] = state -> x[1] + state -> gain[1] * (measure - temp2);

    // Update
    // p(n|n) = [I - gain * h] * p(n|n-1)
    state -> p[0][0] = (1 - state -> gain[0] * state -> h[0]) * state -> p[0][0];
    state -> p[0][1] = (1 - state -> gain[0] * state -> h[1]) * state -> p[0][1];
    state -> p[1][0] = (1 - state -> gain[1] * state -> h[0]) * state -> p[1][0];
    state -> p[1][1] = (1 - state -> gain[1] * state -> h[1]) * state -> p[1][1];

    return state -> x[0];
}