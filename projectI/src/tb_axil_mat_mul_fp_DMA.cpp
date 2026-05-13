#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>

#define N 256
#define SIZE (N * N)
#define COL_BLOCK 4
#define EPSILON 0.0001f

typedef ap_axis<32, 0, 0, 0> axis_t;

static float A[SIZE];
static float B[SIZE];
static float C_sw[SIZE];
static float C_hw[SIZE];

void axis_mat_mul_fp_DMA(
    hls::stream<axis_t> &in_stream,
    hls::stream<axis_t> &out_stream,
    int mode
);

union float_converter {
    float f;
    unsigned int u;
};

static axis_t float_to_axis(float value, bool last)
{
    float_converter conv;
    conv.f = value;

    axis_t word;
    word.data = conv.u;
    word.keep = -1;
    word.strb = -1;
    word.last = last;

    return word;
}

static float axis_to_float(axis_t word)
{
    float_converter conv;
    conv.u = word.data;
    return conv.f;
}

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

void hw_matrix_multiply_stream()
{
    hls::stream<axis_t> in_stream;
    hls::stream<axis_t> out_stream;

    for (int j = 0; j < N; j += COL_BLOCK) {

        /*
         * mode = 0
         * Stream COL_BLOCK columns of B into the accelerator.
         *
         * Equivalent to B_cols[k * COL_BLOCK + jj]
         * in your old AXI-Lite version.
         */
        for (int k = 0; k < N; k++) {
            for (int jj = 0; jj < COL_BLOCK; jj++) {
                int col = j + jj;
                bool last = (k == N - 1) && (jj == COL_BLOCK - 1);

                in_stream.write(float_to_axis(B[k * N + col], last));
            }
        }

        axis_mat_mul_fp_DMA(in_stream, out_stream, 0);

        /*
         * mode = 1
         * For each row of A, stream one full A row into the accelerator.
         * The accelerator returns COL_BLOCK C values.
         */
        for (int i = 0; i < N; i++) {

            for (int k = 0; k < N; k++) {
                bool last = (k == N - 1);
                in_stream.write(float_to_axis(A[i * N + k], last));
            }

            axis_mat_mul_fp_DMA(in_stream, out_stream, 1);

            for (int jj = 0; jj < COL_BLOCK; jj++) {
                axis_t result_word;
                out_stream.read(result_word);

                int col = j + jj;
                C_hw[i * N + col] = axis_to_float(result_word);
            }
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

    hw_matrix_multiply_stream();

    int errors = compare_results();

    // print_matrix("C_sw", C_sw);
    // print_matrix("C_hw", C_hw);

    if (errors == 0) {
        printf("Test passed.\n");
    } else {
        printf("Test failed with %d errors.\n", errors);
    }

    return errors;
}