#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define N 1024
#define COL_BLOCK 2
#define EPSILON 0.001f

static float A[N * N];
static float B[N * N];
static float C_sw[N * N];
static float C_hw[N * N];

static float B_cols[N * COL_BLOCK];
static float A_row[N];
static float C_vals[COL_BLOCK];

void axil_opt_mat_mul_fp(float B_cols[N * COL_BLOCK],
                         float A_row[N],
                         float C_vals[COL_BLOCK],
                         int mode);

void init_matrices()
{
    for (int i = 0; i < N * N; i++) {
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

void hw_style_matrix_multiply()
{
    for (int j = 0; j < N; j += COL_BLOCK) {

        // Pack COL_BLOCK columns of B into B_cols
        for (int k = 0; k < N; k++) {
            for (int jj = 0; jj < COL_BLOCK; jj++) {
                B_cols[k * COL_BLOCK + jj] = B[k * N + (j + jj)];
            }
        }

        // mode = 0: load B columns into internal static buffer
        axil_opt_mat_mul_fp(B_cols, A_row, C_vals, 0);

        for (int i = 0; i < N; i++) {

            // Copy row i of A into A_row
            for (int k = 0; k < N; k++) {
                A_row[k] = A[i * N + k];
            }

            // mode = 1: compute this A row against stored B columns
            axil_opt_mat_mul_fp(B_cols, A_row, C_vals, 1);

            for (int jj = 0; jj < COL_BLOCK; jj++) {
                C_hw[i * N + (j + jj)] = C_vals[jj];
            }
        }
    }
}

int compare_results()
{
    int errors = 0;

    for (int i = 0; i < N * N; i++) {
        float diff = fabsf(C_sw[i] - C_hw[i]);

        if (diff > EPSILON) {
            int row = i / N;
            int col = i % N;

            printf("Mismatch C[%d][%d]: SW=%f HW=%f diff=%f\n",
                   row, col, C_sw[i], C_hw[i], diff);

            errors++;

            if (errors > 20) {
                printf("Too many errors, stopping comparison.\n");
                return errors;
            }
        }
    }

    return errors;
}

int main()
{
    int errors;

    init_matrices();

    printf("Running software matrix multiplication...\n");
    sw_matrix_multiply();

    printf("Running HLS-style optimized matrix multiplication...\n");
    hw_style_matrix_multiply();

    errors = compare_results();

    if (errors == 0) {
        printf("Test passed.\n");
    } else {
        printf("Test failed with %d errors.\n", errors);
    }

    return errors;
}