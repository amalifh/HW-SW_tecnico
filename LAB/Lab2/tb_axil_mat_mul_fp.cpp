#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define N 16
#define SIZE (N * N)
#define EPSILON 0.0001f

static float A[SIZE];
static float B[SIZE];
static float C_sw[SIZE];
static float C_hw[SIZE];

void axil_mat_mul_fp(float A[SIZE], float B[SIZE], float C[SIZE]);

void init_matrices()
{
    for (int i = 0; i < SIZE; i++) {
        A[i] = (float)rand() / (float)RAND_MAX;
        B[i] = (float)rand() / (float)RAND_MAX;
        C_sw[i] = 0.0f;
        C_hw[i] = 0.0f;
    }
}

void sw_matrix_multiply()
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

int compare_results()
{
    int errors = 0;

    for (int i = 0; i < SIZE; i++) {
        float diff = fabs(C_sw[i] - C_hw[i]);

        if (diff > EPSILON) {
            int row = i / N;
            int col = i % N;

            printf("Mismatch at C[%d][%d]: SW = %f, HW = %f, diff = %f\n",
                   row, col, C_sw[i], C_hw[i], diff);
            errors++;
        }
    }

    return errors;
}

void print_matrix(const char *name, float M[SIZE])
{
    printf("%s:\n", name);

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            printf("%8.4f ", M[i * N + j]);
        }
        printf("\n");
    }

    printf("\n");
}

int main()
{
    init_matrices();

    sw_matrix_multiply();

    axil_mat_mul_fp(A, B, C_hw);

    int errors = compare_results();

    print_matrix("C_sw", C_sw);
    print_matrix("C_hw", C_hw);

    if (errors == 0) {
        printf("Test passed.\n");
    } else {
        printf("Test failed with %d errors.\n", errors);
    }

    return errors;
}