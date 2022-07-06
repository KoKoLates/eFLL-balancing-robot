# ifndef _KALMAN_FILTER_H__
# define _KALMAN_FILTER_H__

// The measurement only for one dimension

// Struct defination for one dimension Kalman filter
typedef struct{
    float x; // state
    // x(n) = a * x(n-1) + u(n) : u(n) ~ N(0,q)
    // z(n) = h * x(n) + w(n) : w(n) ~ N(0,r)
    float a, q;
    float h, r;
    // q -> Predict(process) noise convariance
    // r -> Measure noise convariance
    float p; // Estimated error convariance
    float gain;
}Kalman_1D;

// Struct defination for two dimension Kalman filter
typedef struct{
    float x[2]; // state: angle and angle rate
    // x(n) = a * x(n-1) + u(n) : u(n) ~ N(0,q)
    // z(n) = h * x(n) + w(n) : w(n) ~ N(0,r)
    float a[2][2], q[2];
    float h[2], r;
    // q -> predict(process) noise convariance
    // r -> measure noise convariance
    float p[2][2]; // Estimated error convariance [[p0, 01], [p2, p3]]
    float gain[2];
}kalman_2D;

// Two type dimension state Kalman filter initialize
extern void kalman1_init(Kalman_1D *state, float init_x, float init_p);
extern void kalman2_init(kalman_2D *state, float *init_x, float (*init_p)[2]);

// Update and Measurement
extern float kalman1_filter(Kalman_1D *state, float measure);
extern float kalman2_filter(kalman_2D *state, float measure);

# endif // _KALMAN_FILTER_H__