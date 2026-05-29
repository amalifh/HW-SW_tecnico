#include <stdio.h>
#include "xparameters.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xaxidma.h"
#include "xiltimer.h"
#include "xgaussian_blur_dma.h"

#define WIDTH  1280
#define HEIGHT 720
#define SIZE   (WIDTH * HEIGHT)

// 8-bit image: 1 byte per pixel
#define TRANSFER_BYTES SIZE

#define INPUT_ADDR  0x10000000
#define OUTPUT_ADDR 0x11000000

#define DMA_DEV_ID 0

#define PRINT_IMAGE_HEX 0

static XAxiDma AxiDma;
static XGaussian_blur_dma GaussianIP;

static u8 *input_pixels  = (u8 *)INPUT_ADDR;
static u8 *output_pixels = (u8 *)OUTPUT_ADDR;

static u8 sw_output[SIZE];

static int init_dma()
{
    XAxiDma_Config *CfgPtr;
    int status;

    CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);
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

    status = XGaussian_blur_dma_Initialize(&GaussianIP,
                                           XPAR_XGAUSSIAN_BLUR_DMA_0_BASEADDR);

    if (status != XST_SUCCESS) {
        xil_printf("ERROR: Gaussian IP init failed\r\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

static void sw_streaming_reference()
{
    u8 linebuf0[WIDTH] = {0};
    u8 linebuf1[WIDTH] = {0};

    u8 w00 = 0, w01 = 0, w02 = 0;
    u8 w10 = 0, w11 = 0, w12 = 0;
    u8 w20 = 0, w21 = 0, w22 = 0;

    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {

            u8 new_pixel = input_pixels[r * WIDTH + c];

            u8 top_pixel = linebuf0[c];
            u8 mid_pixel = linebuf1[c];

            linebuf0[c] = mid_pixel;
            linebuf1[c] = new_pixel;

            w00 = w01; w01 = w02; w02 = top_pixel;
            w10 = w11; w11 = w12; w12 = mid_pixel;
            w20 = w21; w21 = w22; w22 = new_pixel;

            u8 out_pixel;

            if (r < 2 || c < 2) {
                out_pixel = new_pixel;
            } else {
                unsigned int sum = 0;

                sum += (unsigned int)w00;
                sum += (unsigned int)w01 << 1;
                sum += (unsigned int)w02;

                sum += (unsigned int)w10 << 1;
                sum += (unsigned int)w11 << 2;
                sum += (unsigned int)w12 << 1;

                sum += (unsigned int)w20;
                sum += (unsigned int)w21 << 1;
                sum += (unsigned int)w22;

                out_pixel = (u8)(sum >> 4);
            }

            sw_output[r * WIDTH + c] = out_pixel;
        }
    }
}

static int run_hw()
{
    int status;

    Xil_DCacheFlushRange((UINTPTR)input_pixels, TRANSFER_BYTES);
    Xil_DCacheFlushRange((UINTPTR)output_pixels, TRANSFER_BYTES);

    status = XAxiDma_SimpleTransfer(&AxiDma,
                                    (UINTPTR)output_pixels,
                                    TRANSFER_BYTES,
                                    XAXIDMA_DEVICE_TO_DMA);

    if (status != XST_SUCCESS) {
        xil_printf("ERROR: S2MM transfer setup failed\r\n");
        return XST_FAILURE;
    }

    status = XAxiDma_SimpleTransfer(&AxiDma,
                                    (UINTPTR)input_pixels,
                                    TRANSFER_BYTES,
                                    XAXIDMA_DMA_TO_DEVICE);

    if (status != XST_SUCCESS) {
        xil_printf("ERROR: MM2S transfer setup failed\r\n");
        return XST_FAILURE;
    }

    XGaussian_blur_dma_Start(&GaussianIP);

    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DMA_TO_DEVICE));
    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA));

    Xil_DCacheInvalidateRange((UINTPTR)output_pixels, TRANSFER_BYTES);

    return XST_SUCCESS;
}

static int compare_outputs()
{
    int errors = 0;

    for (int i = 0; i < SIZE; i++) {
        u8 hw_pixel = output_pixels[i];

        if (hw_pixel != sw_output[i]) {
            if (errors < 20) {
                xil_printf("Mismatch at index %d: SW=%d HW=%d\r\n",
                           i, sw_output[i], hw_pixel);
            }
            errors++;
        }
    }

    return errors;
}

static void print_output_hex()
{
#if PRINT_IMAGE_HEX
    xil_printf("BEGIN_IMAGE_HEX\r\n");

    for (int i = 0; i < SIZE; i++) {
        xil_printf("%02x", output_pixels[i]);

        if ((i + 1) % 32 == 0) {
            xil_printf("\r\n");
        }
    }

    xil_printf("\r\nEND_IMAGE_HEX\r\n");
#endif
}

int main()
{
    int status;
    int errors;

    XTime t_start, t_end;
    u64 cycles;
    u64 time_us;

    xil_printf("\r\nGaussian Blur DMA Test - 8-bit AXI Stream\r\n");
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

    xil_printf("Running SW reference...\r\n");

    XTime_GetTime(&t_start);
    sw_streaming_reference();
    XTime_GetTime(&t_end);

    cycles = (u64)(t_end - t_start);
    time_us = cycles / (COUNTS_PER_SECOND / 1000000);

    xil_printf("SW cycles: %llu\r\n", cycles);
    xil_printf("SW execution time: %llu us\r\n", time_us);

    xil_printf("Running HW accelerator...\r\n");

    XTime_GetTime(&t_start);
    status = run_hw();
    XTime_GetTime(&t_end);

    if (status != XST_SUCCESS) {
        xil_printf("HW run failed\r\n");
        return XST_FAILURE;
    }

    cycles = (u64)(t_end - t_start);
    time_us = cycles / (COUNTS_PER_SECOND / 1000000);

    xil_printf("HW cycles: %llu\r\n", cycles);
    xil_printf("HW execution time: %llu us\r\n", time_us);

    errors = compare_outputs();

    if (errors == 0) {
        xil_printf("TEST PASSED\r\n");
    } else {
        xil_printf("TEST FAILED: %d mismatches\r\n", errors);
    }

    print_output_hex();

    xil_printf("Done\r\n");

    return 0;
}