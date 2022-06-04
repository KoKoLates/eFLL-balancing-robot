# include "kalman_filter.h"

void kalman1_init(kalman1 *state, float init_x, float init_p)
{
    state -> x = init_x;
    state -> p = init_p;
    state -> a = 1;
    state -> h = 1;
    state -> q = 0;
    state -> r = 0;
}

void kalman2_init(kalman2 *state, float *init_x, float (*init_p)[2])
{
    state -> x[0] = init_x[0];
    state -> x[1] = init_x[1];
    state -> p[0][0] = init_p[0][0];
    state -> p[0][1] = init_p[0][1];
    state -> p[1][0] = init_p[1][0];
    state -> p[1][1] = init_p[1][1];

    state -> a[0][0] = 1;
    state -> a[0][1] = 1;
    state -> a[1][0] = 1;
    state -> a[1][1] = 1;

    state -> h[0] = 1;
    state -> h[1] = 0;

    state -> q[0] = 0;
    state -> q[1] = 0;
    state -> r = 0;
}

float kalman1_filter(kalman1 *state, float measure)
{
    state -> x = state -> a * state -> x;
    state -> p = state -> a * state -> a * state -> p + state ->q;

    state ->gain = state -> p * state -> h / (state -> p * state -> h * state -> h + state -> r);
    state -> x = state -> x + state ->gain * (measure - state -> h * state -> x);
    state -> p = (1 - state -> gain * state -> h) * state -> p;

    return state -> x;
}

