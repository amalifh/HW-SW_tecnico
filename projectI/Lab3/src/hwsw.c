#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "xparameters.h"
#include "xaxidma.h"
#include "xil_cache.h"
#include "xil_types.h"
#include "xstatus.h"
#include "xiltimer.h"

#include "xaxis_mat_mul_fp_dma.h"

#define N 512
#define SIZE (N * N)
#define COL_BLOCK 32
#define EPSILON 0.0001f

#define DMA_DEV_ID 0

static XAxiDma AxiDma;
static XAxis_mat_mul_fp_dma MatMulIp;

static float A[SIZE] __attribute__((aligned(64)));
static float B[SIZE] __attribute__((aligned(64)));
static float C_sw[SIZE] __attribute__((aligned(64)));
static float C_hw[SIZE] __attribute__((aligned(64)));

static float B_cols[N * COL_BLOCK] __attribute__((aligned(64)));
static float A_row[N] __attribute__((aligned(64)));
static float C_vals[COL_BLOCK] __attribute__((aligned(64)));

static int init_dma(void)
{
    XAxiDma_Config *CfgPtr;

    CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);
    if (!CfgPtr) {
        printf("DMA config not found\n");
        return XST_FAILURE;
    }

    if (XAxiDma_CfgInitialize(&AxiDma, CfgPtr) != XST_SUCCESS) {
        printf("DMA initialization failed\n");
        return XST_FAILURE;
    }

    if (XAxiDma_HasSg(&AxiDma)) {
        printf("DMA is in scatter-gather mode, expected simple mode\n");
        return XST_FAILURE;
    }

    XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
    XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

    return XST_SUCCESS;
}

static int init_ip(void)
{
    int status;

#ifdef SDT
    status = XAxis_mat_mul_fp_dma_Initialize(
        &MatMulIp,
        XPAR_AXIS_MAT_MUL_FP_DMA_0_BASEADDR
    );
#else
    status = XAxis_mat_mul_fp_dma_Initialize(
        &MatMulIp,
        XPAR_XAXIS_MAT_MUL_FP_DMA_0_BASEADDR
    );
#endif

    if (status != XST_SUCCESS) {
        printf("HLS IP initialization failed\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

static void init_matrices(void)
{
    for (int i = 0; i < SIZE; i++) {
        A[i] = (float)rand() / (float)RAND_MAX;
        B[i] = (float)rand() / (float)RAND_MAX;
        C_sw[i] = 0.0f;
        C_hw[i] = 0.0f;
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

static int dma_send(void *src, int bytes)
{
    Xil_DCacheFlushRange((UINTPTR)src, bytes);

    int status = XAxiDma_SimpleTransfer(
        &AxiDma,
        (UINTPTR)src,
        bytes,
        XAXIDMA_DMA_TO_DEVICE
    );

    if (status != XST_SUCCESS) {
        printf("MM2S transfer failed\n");
        return XST_FAILURE;
    }

    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DMA_TO_DEVICE)) {
        /* wait */
    }

    return XST_SUCCESS;
}

static int dma_send_recv(void *src, int src_bytes, void *dst, int dst_bytes)
{
    Xil_DCacheFlushRange((UINTPTR)src, src_bytes);
    Xil_DCacheFlushRange((UINTPTR)dst, dst_bytes);

    int status;

    /*
     * Arm receive side first.
     */
    status = XAxiDma_SimpleTransfer(
        &AxiDma,
        (UINTPTR)dst,
        dst_bytes,
        XAXIDMA_DEVICE_TO_DMA
    );

    if (status != XST_SUCCESS) {
        printf("S2MM transfer failed\n");
        return XST_FAILURE;
    }

    status = XAxiDma_SimpleTransfer(
        &AxiDma,
        (UINTPTR)src,
        src_bytes,
        XAXIDMA_DMA_TO_DEVICE
    );

    if (status != XST_SUCCESS) {
        printf("MM2S transfer failed\n");
        return XST_FAILURE;
    }

    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DMA_TO_DEVICE)) {
        /* wait */
    }

    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA)) {
        /* wait */
    }

    Xil_DCacheInvalidateRange((UINTPTR)dst, dst_bytes);

    return XST_SUCCESS;
}

static int hw_matrix_multiply_dma(void)
{
    for (int j = 0; j < N; j += COL_BLOCK) {

        /*
         * mode = 0: send two columns of B to the IP.
         */
        int idx = 0;
        for (int k = 0; k < N; k++) {
            for (int jj = 0; jj < COL_BLOCK; jj++) {
                B_cols[idx++] = B[k * N + (j + jj)];
            }
        }

        XAxis_mat_mul_fp_dma_Set_mode(&MatMulIp, 0);
        XAxis_mat_mul_fp_dma_Start(&MatMulIp);

        if (dma_send(B_cols, sizeof(B_cols)) != XST_SUCCESS) {
            return XST_FAILURE;
        }

        while (!XAxis_mat_mul_fp_dma_IsDone(&MatMulIp)) {
            /* wait */
        }

        /*
         * mode = 1: send one row of A, receive two C values.
         */
        for (int i = 0; i < N; i++) {

            for (int k = 0; k < N; k++) {
                A_row[k] = A[i * N + k];
            }

            C_vals[0] = 0.0f;
            C_vals[1] = 0.0f;

            XAxis_mat_mul_fp_dma_Set_mode(&MatMulIp, 1);
            XAxis_mat_mul_fp_dma_Start(&MatMulIp);

            if (dma_send_recv(A_row, sizeof(A_row),
                              C_vals, sizeof(C_vals)) != XST_SUCCESS) {
                return XST_FAILURE;
            }

            while (!XAxis_mat_mul_fp_dma_IsDone(&MatMulIp)) {
                /* wait */
            }

            for (int jj = 0; jj < COL_BLOCK; jj++) {
                C_hw[i * N + (j + jj)] = C_vals[jj];
            }
        }
    }

    return XST_SUCCESS;
}

static int compare_results(void)
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
                break;
            }
        }
    }

    return errors;
}

int main(void)
{
    printf("Starting floating-point AXI-Stream DMA matrix multiplication\n");

    if (init_dma() != XST_SUCCESS) {
        return XST_FAILURE;
    }

    if (init_ip() != XST_SUCCESS) {
        return XST_FAILURE;
    }

    init_matrices();

    XTime t0, t1;

    XTime_GetTime(&t0);
    sw_matrix_multiply();
    XTime_GetTime(&t1);

    printf("SW execution: %llu clock cycles, %.2f us\n",
           2 * (t1 - t0),
           1.0 * (t1 - t0) * 1000000 / COUNTS_PER_SECOND);

    XTime_GetTime(&t0);
    if (hw_matrix_multiply_dma() != XST_SUCCESS) {
        printf("HW DMA matrix multiplication failed\n");
        return XST_FAILURE;
    }
    XTime_GetTime(&t1);

    printf("HW DMA execution: %llu clock cycles, %.2f us\n",
           2 * (t1 - t0),
           1.0 * (t1 - t0) * 1000000 / COUNTS_PER_SECOND);

    int errors = compare_results();

    if (errors == 0) {
        printf("Test passed.\n");
    } else {
        printf("Test failed with %d errors.\n", errors);
    }

    return errors;
}