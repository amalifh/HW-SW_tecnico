#include <stdio.h>
#include "xparameters.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xaxidma.h"
#include "xiltimer.h"
#include "xgaussian_blur_v2.h"

//#define KSIZE 5
//#define NBUF  4
//#define HALO  4

#define WIDTH  512
#define HEIGHT 512
#define SIZE   (WIDTH * HEIGHT)

#define TRANSFER_BYTES SIZE

#define INPUT_ADDR  0x10000000
#define OUTPUT_ADDR 0x11000000

#define DMA_DEV_ID 0

static XAxiDma AxiDma;
static XGaussian_blur_v2 GaussianIP;

static u8 *input_pixels  = (u8 *)INPUT_ADDR;
static u8 *output_pixels = (u8 *)OUTPUT_ADDR;

static u8 sw_output[SIZE];

static const unsigned int kernel[5][5] = {
    { 1,  4,  6,  4,  1},
    { 4, 16, 24, 16,  4},
    { 6, 24, 36, 24,  6},
    { 4, 16, 24, 16,  4},
    { 1,  4,  6,  4,  1}
};

static int init_dma()
{
    XAxiDma_Config *CfgPtr;
    int status;

#ifdef SDT
    CfgPtr = XAxiDma_LookupConfig(XPAR_XAXIDMA_0_BASEADDR);
#else
    CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);
#endif

    if (!CfgPtr) {
        xil_printf("ERROR: No DMA config found\r\n");
        return XST_FAILURE;
    }

    status = XAxiDma_CfgInitialize(&AxiDma, CfgPtr);
    if (status != XST_SUCCESS) {
        xil_printf("ERROR: DMA init failed\r\n");
        return XST_FAILURE;
    }

    if (XAxiDma_HasSg(&AxiDma)) {
        xil_printf("ERROR: DMA is in SG mode, expected simple mode\r\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

static int init_gaussian_ip()
{
    int status;

#ifdef SDT
    status = XGaussian_blur_v2_Initialize(
        &GaussianIP,
        XPAR_XGAUSSIAN_BLUR_V2_0_BASEADDR
    );
#else
    status = XGaussian_blur_v2_Initialize(&GaussianIP, 0);
#endif

    if (status != XST_SUCCESS) {
        xil_printf("ERROR: Gaussian IP init failed\r\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}
/*
static void sw_streaming_reference(void)
{
    u8  linebuf[NBUF][WIDTH];
    u8  w[KSIZE][KSIZE];
    int b, r, c, i, j;

    for (b = 0; b < NBUF; b++)
        for (c = 0; c < WIDTH; c++)
            linebuf[b][c] = 0;

    for (i = 0; i < KSIZE; i++)
        for (j = 0; j < KSIZE; j++)
            w[i][j] = 0;

    for (r = 0; r < HEIGHT; r++) {
        for (c = 0; c < WIDTH; c++) {

            u8 new_pixel = input_pixels[r * WIDTH + c];

            u8 col_pixels[NBUF];
            for (b = 0; b < NBUF; b++)
                col_pixels[b] = linebuf[b][c];

            for (b = 0; b < NBUF - 1; b++)
                linebuf[b][c] = col_pixels[b + 1];
            linebuf[NBUF - 1][c] = new_pixel;

            for (i = 0; i < KSIZE; i++)
                for (j = 0; j < KSIZE - 1; j++)
                    w[i][j] = w[i][j + 1];
            for (b = 0; b < NBUF; b++)
                w[b][KSIZE - 1] = col_pixels[b];
            w[KSIZE - 1][KSIZE - 1] = new_pixel;

            u8 out_pixel;

            if (r < HALO || c < HALO) {
                out_pixel = new_pixel;
            } else {
                unsigned int sum = 0;

                for (i = 0; i < KSIZE; i++)
                    for (j = 0; j < KSIZE; j++)
                        sum += (unsigned int)w[i][j] * kernel[i][j];

                out_pixel = (u8)(sum >> 8);
            }

            sw_output[r * WIDTH + c] = out_pixel;
        }
    }
}
*/
static void sw_gaussian_5x5_reference()
{
    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {

            if (r < 4 || c < 4) {
                sw_output[r * WIDTH + c] = input_pixels[r * WIDTH + c];
            } else {
                unsigned int sum = 0;

                for (int kr = 0; kr < 5; kr++) {
                    for (int kc = 0; kc < 5; kc++) {
                        int rr = r - 4 + kr;
                        int cc = c - 4 + kc;

                        sum += ((unsigned int)input_pixels[rr * WIDTH + cc]) *
                               kernel[kr][kc];
                    }
                }

                sw_output[r * WIDTH + c] = (u8)(sum >> 8);
            }
        }
    }
}

static int run_hw()
{
    int status;

    /*
    Xil_DCacheFlushRange((UINTPTR)input_pixels, TRANSFER_BYTES);
    Xil_DCacheFlushRange((UINTPTR)output_pixels, TRANSFER_BYTES);
    */
    status = XAxiDma_SimpleTransfer(
        &AxiDma,
        (UINTPTR)output_pixels,
        TRANSFER_BYTES,
        XAXIDMA_DEVICE_TO_DMA
    );

    if (status != XST_SUCCESS) {
        xil_printf("ERROR: S2MM setup failed\r\n");
        return XST_FAILURE;
    }

    status = XAxiDma_SimpleTransfer(
        &AxiDma,
        (UINTPTR)input_pixels,
        TRANSFER_BYTES,
        XAXIDMA_DMA_TO_DEVICE
    );

    if (status != XST_SUCCESS) {
        xil_printf("ERROR: MM2S setup failed\r\n");
        return XST_FAILURE;
    }

    XGaussian_blur_v2_Start(&GaussianIP);

    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DMA_TO_DEVICE));
    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA));

    Xil_DCacheInvalidateRange((UINTPTR)output_pixels, TRANSFER_BYTES);

    return XST_SUCCESS;
}

static int compare_outputs()
{
    int errors = 0;

    for (int i = 0; i < SIZE; i++) {
        if (output_pixels[i] != sw_output[i]) {
            if (errors < 20) {
                xil_printf("Mismatch at index %d: SW=%d HW=%d\r\n",
                           i, sw_output[i], output_pixels[i]);
            }
            errors++;
        }
    }

    return errors;
}

int main()
{
    int status;
    int errors;

    XTime t_start, t_end;
    u64 sw_cycles, hw_cycles;
    u64 sw_time_us, hw_time_us;

    xil_printf("\r\nGaussian Blur V2 5x5 8-PPC DMA Test\r\n");
    xil_printf("Image size: %dx%d\r\n", WIDTH, HEIGHT);
    xil_printf("Transfer size: %d bytes each direction\r\n", TRANSFER_BYTES);
    xil_printf("Input buffer:  0x%08x\r\n", INPUT_ADDR);
    xil_printf("Output buffer: 0x%08x\r\n", OUTPUT_ADDR);

    status = init_dma();
    if (status != XST_SUCCESS) {
        xil_printf("DMA init failed\r\n");
        return XST_FAILURE;
    }

    status = init_gaussian_ip();
    if (status != XST_SUCCESS) {
        xil_printf("Gaussian IP init failed\r\n");
        return XST_FAILURE;
    }

    Xil_DCacheInvalidateRange((UINTPTR)input_pixels, TRANSFER_BYTES);

    xil_printf("Running SW 5x5 reference...\r\n");

    XTime_GetTime(&t_start);
    sw_gaussian_5x5_reference();
    //sw_streaming_reference();
    XTime_GetTime(&t_end);

    sw_cycles = (u64)(t_end - t_start);
    sw_time_us = sw_cycles / (COUNTS_PER_SECOND / 1000000);

    xil_printf("SW cycles: %llu\r\n", sw_cycles);
    xil_printf("SW execution time: %llu us\r\n", sw_time_us);

    xil_printf("Running HW accelerator...\r\n");

    XTime_GetTime(&t_start);
    status = run_hw();
    XTime_GetTime(&t_end);

    if (status != XST_SUCCESS) {
        xil_printf("HW run failed\r\n");
        return XST_FAILURE;
    }

    hw_cycles = (u64)(t_end - t_start);
    hw_time_us = hw_cycles / (COUNTS_PER_SECOND / 1000000);

    xil_printf("HW cycles: %llu\r\n", hw_cycles);
    xil_printf("HW execution time: %llu us\r\n", hw_time_us);

    if (hw_cycles != 0) {
        u64 speedup_int = sw_cycles / hw_cycles;
        u64 speedup_frac = ((sw_cycles % hw_cycles) * 100) / hw_cycles;
        xil_printf("Speedup: %llu.%02llux\r\n", speedup_int, speedup_frac);
    }

    errors = compare_outputs();

    if (errors == 0) {
        xil_printf("TEST PASSED: HW matches SW\r\n");
    } else {
        xil_printf("TEST FAILED: %d mismatches\r\n", errors);
    }

    xil_printf("Done\r\n");

    return 0;
}