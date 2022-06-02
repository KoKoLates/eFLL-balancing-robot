# include "kalman_filter.h"

void kalman1_init(kalman1 *state, float init_x, float init_p)
{
    state->x = init_x;
    state->p = init_p;
    state->a = 1;
    state->h = 1;
}

void kalman_filter()