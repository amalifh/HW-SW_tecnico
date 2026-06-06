Du kan bruke denne koden:

#include <stdio.h>
#include <stdlib.h>
#include "xiltimer.h"


#define N    256
#define SIZE (N * N)

static float A[SIZE];
static float B[SIZE];
static float C_sw[SIZE];

static void init_matrices(void)
{
    for (int i = 0; i < SIZE; i++) {
        A[i] = (float)rand() / (float)RAND_MAX;
        B[i] = (float)rand() / (float)RAND_MAX;
        C_sw[i] = 0.0f;
    }
}

static void sw_matrix_multiply(void)
{
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {

            float acc = 0.0f;

            for (int k = 0; k < N; k++) {
                acc += A[i * N + k] * B[k * N + j];
            }

            C_sw[i * N + j] = acc;
        }
    }
}

int main(void)
{
    XTime t0, t1;

    printf("Software Floating-Point Matrix Multiplication\n");

    init_matrices();

    XTime_GetTime(&t0);
    sw_matrix_multiply();
    XTime_GetTime(&t1);

    printf("SW execution: %llu clock cycles, %.2f us\n",
           2 * (t1 - t0),
           1.0 * (t1 - t0) * 1000000.0 / COUNTS_PER_SECOND);

    return 0;
}