# ifndef _KALMAN_FILTER_H__
# define _KALMAN_FILTER_H__

typedef struct{
    float x, a, h;
    float q, r, p;
    float gain;
}kalman1;

typedef struct{
    float x[2], a[2][2], h[2];
    float q[2], p[2][2], r;
    float gain[2];
}kalman2;

extern void kalman1_init(kalman1 *state, float init_x, float init_p);
extern void kalman2_init(kalman2 *state, float *init_x, float (*init_p)[2]);
extern float kalman1_filter(kalman1 *state, float measure);
extern float kalman2_filter(kalman2 *state, float measure);

# endif