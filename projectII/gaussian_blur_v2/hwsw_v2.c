#include <stdio.h>
#include "xparameters.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xaxidma.h"
#include "xiltimer.h"
#include "xgaussian_blur_v2.h"

#define WIDTH  512
#define HEIGHT 512
#define SIZE   (WIDTH * HEIGHT)

// 8-bit image: 1 byte per pixel — unchanged from 3x3 version
#define TRANSFER_BYTES SIZE

// ── Kernel parameters ────────────────────────────────────────────────────────
// Must match gaussian_blur_v2.cpp exactly.
#define KSIZE  5
#define HALO   (KSIZE - 1)   // = 4: rows/cols before valid output
#define NBUF   (KSIZE - 1)   // = 4 line buffers

// 5x5 Gaussian kernel — outer product of [1,4,6,4,1] with itself.
// Sum = 256 = 2^8, so normalise with >> 8.
//
//   1  4  6  4  1
//   4 16 24 16  4
//   6 24 36 24  6
//   4 16 24 16  4
//   1  4  6  4  1
static const unsigned int KERNEL[KSIZE][KSIZE] = {
    { 1,  4,  6,  4,  1},
    { 4, 16, 24, 16,  4},
    { 6, 24, 36, 24,  6},
    { 4, 16, 24, 16,  4},
    { 1,  4,  6,  4,  1}
};

#define INPUT_ADDR  0x10000000
#define OUTPUT_ADDR 0x11000000

#define DMA_DEV_ID 0

#define PRINT_IMAGE_HEX 0

static XAxiDma AxiDma;
static XGaussian_blur_v2 GaussianIP;

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

    status = XGaussian_blur_v2_Initialize(&GaussianIP,
                                           XPAR_XGAUSSIAN_BLUR_V2_0_BASEADDR);

    if (status != XST_SUCCESS) {
        xil_printf("ERROR: Gaussian IP init failed\r\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

// ── SW reference — 5x5 streaming Gaussian blur ───────────────────────────────
// Mirrors the HLS implementation exactly: same line buffer shifts,
// same window construction, same kernel and normalisation.
static void sw_streaming_reference()
{
    // 4 line buffers (oldest to most recent buffered row)
    u8 linebuf[NBUF][WIDTH];
    int b, r, c, i, j;

    for (b = 0; b < NBUF; b++)
        for (c = 0; c < WIDTH; c++)
            linebuf[b][c] = 0;

    // 5x5 sliding window: w[row][col], row 0 = oldest, row 4 = current
    u8 w[KSIZE][KSIZE];
    for (i = 0; i < KSIZE; i++)
        for (j = 0; j < KSIZE; j++)
            w[i][j] = 0;

    for (r = 0; r < HEIGHT; r++) {
        for (c = 0; c < WIDTH; c++) {

            u8 new_pixel = input_pixels[r * WIDTH + c];

            // Read column c from all 4 line buffers
            u8 col_pixels[NBUF];
            for (b = 0; b < NBUF; b++)
                col_pixels[b] = linebuf[b][c];

            // Shift line buffers: oldest drops out, newest row enters at end
            for (b = 0; b < NBUF - 1; b++)
                linebuf[b][c] = col_pixels[b + 1];
            linebuf[NBUF - 1][c] = new_pixel;

            // Shift window left by one column (drop leftmost, make room on right)
            for (i = 0; i < KSIZE; i++)
                for (j = 0; j < KSIZE - 1; j++)
                    w[i][j] = w[i][j + 1];

            // Insert new rightmost column into window
            // col_pixels[0] = row r-4, col_pixels[3] = row r-1, new_pixel = row r
            for (b = 0; b < NBUF; b++)
                w[b][KSIZE - 1] = col_pixels[b];
            w[KSIZE - 1][KSIZE - 1] = new_pixel;

            u8 out_pixel;

            if (r < HALO || c < HALO) {
                // Border: not enough history for a full window, pass through
                out_pixel = new_pixel;
            } else {
                unsigned int sum = 0;

                for (i = 0; i < KSIZE; i++)
                    for (j = 0; j < KSIZE; j++)
                        sum += (unsigned int)w[i][j] * KERNEL[i][j];

                // Normalise: sum of kernel = 256 = 2^8
                out_pixel = (u8)(sum >> 8);
            }

            sw_output[r * WIDTH + c] = out_pixel;
        }
    }
}

static int run_hw()
{
    int status;

    // DMA transfer size is still SIZE bytes — pixel format is unchanged (8-bit)
    Xil_DCacheFlushRange((UINTPTR)input_pixels,  TRANSFER_BYTES);
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

    XGaussian_blur_v2_Start(&GaussianIP);

    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DMA_TO_DEVICE));
    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA));

    Xil_DCacheInvalidateRange((UINTPTR)output_pixels, TRANSFER_BYTES);

    return XST_SUCCESS;
}

static int compare_outputs()
{
    int errors = 0;
    int i;

    for (i = 0; i < SIZE; i++) {
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
        if ((i + 1) % 32 == 0)
            xil_printf("\r\n");
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

    xil_printf("\r\nGaussian Blur DMA Test - gaussian_blur_v2 (5x5 kernel, 8-bit AXI Stream)\r\n");
    xil_printf("Image size: %dx%d\r\n", WIDTH, HEIGHT);
    xil_printf("Kernel: %dx%d (sum=256, normalise >>8)\r\n", KSIZE, KSIZE);
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

    xil_printf("Running SW reference (5x5)...\r\n");

    XTime_GetTime(&t_start);
    sw_streaming_reference();
    XTime_GetTime(&t_end);

    cycles = (u64)(t_end - t_start);
    time_us = cycles / (COUNTS_PER_SECOND / 1000000);

    xil_printf("SW cycles: %llu\r\n", cycles);
    xil_printf("SW execution time: %llu us\r\n", time_us);

    xil_printf("Running HW accelerator (5x5)...\r\n");

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
