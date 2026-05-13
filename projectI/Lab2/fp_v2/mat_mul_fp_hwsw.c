#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "xparameters.h"
#include "xaxil_opt_mat_mul_fp.h"
#include "xiltimer.h"

#define N 16
#define SIZE (N * N)
#define EPSILON 0.0001f

static float A[SIZE];
static float B[SIZE];
static float C_sw[SIZE];
static float C_hw[SIZE];

static u32 A_words[SIZE];
static u32 B_words[SIZE];
static u32 C_words[SIZE];

static u32 float_to_u32(float x)
{
    union {
        float f;
        u32 u;
    } converter;

    converter.f = x;
    return converter.u;
}

static float u32_to_float(u32 x)
{
    union {
        float f;
        u32 u;
    } converter;

    converter.u = x;
    return converter.f;
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

void SW_matrix_multiply()
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

int HW_matrix_multiply()
{
    XAxil_opt_mat_mul_fp Instance;
    XAxil_opt_mat_mul_fp_Config *ConfigPtr;
    int status;

    ConfigPtr = XAxil_opt_mat_mul_fp_LookupConfig(XPAR_XAXIL_OPT_MAT_MUL_FP_0_BASEADDR);
    if (ConfigPtr == NULL) {
        printf("LookupConfig failed\n");
        return XST_FAILURE;
    }

    status = XAxil_opt_mat_mul_fp_CfgInitialize(&Instance, ConfigPtr);
    if (status != XST_SUCCESS) {
        printf("CfgInitialize failed\n");
        return XST_FAILURE;
    }

    for (int i = 0; i < SIZE; i++) {
        A_words[i] = float_to_u32(A[i]);
        B_words[i] = float_to_u32(B[i]);
    }

    XAxil_opt_mat_mul_fp_Write_A_row_Words(&Instance, 0, A_words, SIZE);
    XAxil_opt_mat_mul_fp_Write_B_cols_Words(&Instance, 0, B_words, SIZE);

    XAxil_opt_mat_mul_fp_Start(&Instance);

    while (!XAxil_opt_mat_mul_fp_IsDone(&Instance));

    XAxil_opt_mat_mul_fp_Read_C_vals_Words(&Instance, 0, C_words, SIZE);

    for (int i = 0; i < SIZE; i++) {
        C_hw[i] = u32_to_float(C_words[i]);
    }

    return XST_SUCCESS;
}

int compare_results()
{
    int errors = 0;

    for (int i = 0; i < SIZE; i++) {
        float diff = fabsf(C_sw[i] - C_hw[i]);

        if (diff > EPSILON) {
            int row = i / N;
            int col = i % N;

            printf("Mismatch C[%d][%d]: SW=%f HW=%f diff=%f\n",
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
    XTime tStart, tEnd;
    int errors;

    init_matrices();

    XTime_GetTime(&tStart);
    SW_matrix_multiply();
    XTime_GetTime(&tEnd);

    printf("SW matrix multiplication done\n");
    printf("SW execution: %llu clock cycles, %.2f us\n",
           2 * (tEnd - tStart),
           1.0 * (tEnd - tStart) * 1000000 / COUNTS_PER_SECOND);

    XTime_GetTime(&tStart);
    if (HW_matrix_multiply() != XST_SUCCESS) {
        printf("HW matrix multiplication failed\n");
        return 1;
    }
    XTime_GetTime(&tEnd);

    printf("HW matrix multiplication done\n");
    printf("HW execution: %llu clock cycles, %.2f us\n",
           2 * (tEnd - tStart),
           1.0 * (tEnd - tStart) * 1000000 / COUNTS_PER_SECOND);

    errors = compare_results();

    printf("Results match: %s\n", (errors == 0) ? "YES" : "NO");

    if (errors != 0) {
        printf("Number of errors: %d\n", errors);
    }

    return errors;
}