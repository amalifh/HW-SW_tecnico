#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define N 16
#define SIZE (N * N)
#define B_BLOCK 4
#define EPSILON 0.0001f

static float A[SIZE];
static float B[SIZE];
static float C_sw[SIZE];
static float C_hw[SIZE];

static float A_row[N];
static float B_cols[B_BLOCK][N];
static float C_out[B_BLOCK];

void axil_mat_mul_cols(float A_row[N],
                       float B_cols[B_BLOCK][N],
                       float C_out[B_BLOCK]);

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

void hw_matrix_multiply_simulated()
{
    for (int jb = 0; jb < N; jb += B_BLOCK) {

        // Copy B column block into local input buffer
        for (int j = 0; j < B_BLOCK; j++) {
            for (int k = 0; k < N; k++) {
                B_cols[j][k] = B[k * N + (jb + j)];
            }
        }

        for (int i = 0; i < N; i++) {

            // Copy one row of A
            for (int k = 0; k < N; k++) {
                A_row[k] = A[i * N + k];
            }

            // Run HLS accelerator
            axil_mat_mul_cols(A_row, B_cols, C_out);

            // Store output block into C_hw
            for (int j = 0; j < B_BLOCK; j++) {
                C_hw[i * N + (jb + j)] = C_out[j];
            }
        }
    }
}

int compare_results()
{
    int errors = 0;

    for (int i = 0; i < SIZE; i++) {
        float diff = fabsf(C_sw[i] - C_hw[i]);

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

    hw_matrix_multiply_simulated();

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