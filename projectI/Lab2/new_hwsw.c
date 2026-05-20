#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "xparameters.h"
#include "xaxil_opt_mat_mul_fp.h"
#include "xiltimer.h"

#define N 1024
#define COL_BLOCK 2
#define SIZE (N * N)
#define EPSILON 0.001f

static float A[SIZE];
static float B[SIZE];
static float C_sw[SIZE];
static float C_hw[SIZE];

static u32 B_cols_words[N * COL_BLOCK];
static u32 A_row_words[N];
static u32 C_vals_words[COL_BLOCK];

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

void print_time(const char *name, XTime start, XTime end)
{
    printf("%s: %llu clock cycles, %.2f us\n",
           name,
           2 * (end - start),
           1.0 * (end - start) * 1000000 / COUNTS_PER_SECOND);
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

int init_ip(XAxil_opt_mat_mul_fp *Instance)
{
    XAxil_opt_mat_mul_fp_Config *ConfigPtr;
    int status;

    ConfigPtr = XAxil_opt_mat_mul_fp_LookupConfig(XPAR_XAXIL_OPT_MAT_MUL_FP_0_BASEADDR);
    if (ConfigPtr == NULL) {
        printf("LookupConfig failed\n");
        return XST_FAILURE;
    }

    status = XAxil_opt_mat_mul_fp_CfgInitialize(Instance, ConfigPtr);
    if (status != XST_SUCCESS) {
        printf("CfgInitialize failed\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

void pack_B_columns(int j)
{
    for (int k = 0; k < N; k++) {
        for (int jj = 0; jj < COL_BLOCK; jj++) {
            B_cols_words[k * COL_BLOCK + jj] =
                float_to_u32(B[k * N + (j + jj)]);
        }
    }
}

void pack_A_row(int i)
{
    for (int k = 0; k < N; k++) {
        A_row_words[k] = float_to_u32(A[i * N + k]);
    }
}

int HW_matrix_multiply()
{
    XAxil_opt_mat_mul_fp Instance;
    int status;

    XTime t0, t1, t2, t3, t4, t5;
    XTime write_B_total = 0;
    XTime write_A_total = 0;
    XTime compute_total = 0;
    XTime read_C_total = 0;

    status = init_ip(&Instance);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    for (int j = 0; j < N; j += COL_BLOCK) {

        pack_B_columns(j);

        XTime_GetTime(&t0);
        XAxil_opt_mat_mul_fp_Write_B_cols_Words(
            &Instance, 0, B_cols_words, N * COL_BLOCK
        );
        XAxil_opt_mat_mul_fp_Set_mode(&Instance, 0);
        XAxil_opt_mat_mul_fp_Start(&Instance);
        while (!XAxil_opt_mat_mul_fp_IsDone(&Instance));
        XTime_GetTime(&t1);

        write_B_total += (t1 - t0);

        for (int i = 0; i < N; i++) {

            pack_A_row(i);

            XTime_GetTime(&t2);
            XAxil_opt_mat_mul_fp_Write_A_row_Words(
                &Instance, 0, A_row_words, N
            );
            XTime_GetTime(&t3);

            XAxil_opt_mat_mul_fp_Set_mode(&Instance, 1);
            XAxil_opt_mat_mul_fp_Start(&Instance);
            while (!XAxil_opt_mat_mul_fp_IsDone(&Instance));
            XTime_GetTime(&t4);

            XAxil_opt_mat_mul_fp_Read_C_vals_Words(
                &Instance, 0, C_vals_words, COL_BLOCK
            );
            XTime_GetTime(&t5);

            write_A_total += (t3 - t2);
            compute_total += (t4 - t3);
            read_C_total += (t5 - t4);

            for (int jj = 0; jj < COL_BLOCK; jj++) {
                C_hw[i * N + (j + jj)] = u32_to_float(C_vals_words[jj]);
            }
        }

        printf("Finished columns %d to %d\n", j, j + COL_BLOCK - 1);
    }

    printf("\nHW/SW timing split:\n");
    print_time("Load B columns total", 0, write_B_total);
    print_time("Write A rows total", 0, write_A_total);
    print_time("HW compute total", 0, compute_total);
    print_time("Read C values total", 0, read_C_total);

    XTime total = write_B_total + write_A_total + compute_total + read_C_total;
    print_time("Total HW/SW path", 0, total);

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
    XTime tStart, tEnd;
    int errors;

    printf("Full 1024x1024 matrix multiplication test\n");
    printf("N = %d, COL_BLOCK = %d\n", N, COL_BLOCK);

    init_matrices();

    XTime_GetTime(&tStart);
    SW_matrix_multiply();
    XTime_GetTime(&tEnd);

    printf("\nSW matrix multiplication done\n");
    print_time("SW compute only", tStart, tEnd);

    if (HW_matrix_multiply() != XST_SUCCESS) {
        printf("HW matrix multiplication failed\n");
        return 1;
    }

    errors = compare_results();

    printf("\nResults match: %s\n", (errors == 0) ? "YES" : "NO");

    if (errors != 0) {
        printf("Number of errors: %d\n", errors);
    }

    return errors;
}