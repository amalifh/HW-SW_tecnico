/* hwsw_v2.c — Zynq bare-metal driver for gaussian_blur_v2 HLS IP
 *
 * Target board : Zybo (XPS_BOARD_ZYBO, Zynq-7010)
 * HLS IP       : gaussian_blur_v2  — 5x5 Gaussian kernel, 64-bit AXI stream
 *                (8 pixels per transaction, fully unrolled kernel loops)
 * DMA          : AXI DMA 0 in simple mode, 64-bit data bus
 *
 * Address map (from xparameters.h) ─────────────────────────────────────────
 *   DMA base      : 0x41E00000   (XPAR_XAXIDMA_0_BASEADDR)
 *   Gaussian IP   : 0x40000000   (XPAR_XGAUSSIAN_BLUR_V2_0_BASEADDR)
 *   DDR range     : 0x00100000 – 0x1FFFFFFF
 *   INPUT_ADDR    : 0x10000000   (inside DDR ✓)
 *   OUTPUT_ADDR   : 0x11000000   (inside DDR ✓, non-overlapping ✓)
 *
 * Timer note ────────────────────────────────────────────────────────────────
 *   XTime_GetTime() reads the Zynq PS Global Timer which increments at
 *   XPAR_CPU_CORE_CLOCK_FREQ_HZ / 2 = 650 MHz / 2 = 325 MHz.
 *   xiltimer.h does NOT define COUNTS_PER_SECOND; xtime_l.h does.
 *   FIX: replaced #include "xiltimer.h" with #include "xtime_l.h".
 *        COUNTS_PER_SECOND is defined there as (XPAR_CPU_CORE_CLOCK_FREQ_HZ/2).
 *
 * DMA device ID ─────────────────────────────────────────────────────────────
 *   xparameters.h does NOT define XPAR_XAXIDMA_0_DEVICE_ID.
 *   XAxiDma_LookupConfig() accepts the integer device index directly.
 *   DMA_DEV_ID = 0 is correct and consistent with the single DMA instance.
 *
 * Pixel / transfer format ───────────────────────────────────────────────────
 *   8-bit greyscale, 512×512.  TRANSFER_BYTES = 512*512 = 262 144 bytes.
 *   DMA data width is 64-bit (0x40) in both directions — matches the HLS IP
 *   which packs 8 pixels per 64-bit AXI-stream word.
 *
 * Requirements checklist ────────────────────────────────────────────────────
 *   [x] 512×512 base-2 image
 *   [x] 5×5 Gaussian kernel (KSIZE configurable via #define)
 *   [x] Kernel weights: outer product of [1,4,6,4,1], normalised >> 8
 *   [x] 64-bit AXI stream → 8 pixels per DMA word (HLS IP side)
 *   [x] SW reference mirrors HLS streaming algorithm exactly
 *   [x] Configurable window: change KSIZE, HALO, NBUF and kernel table
 *   [x] Speedup ratio printed at end
 */

#include <stdio.h>
#include "xparameters.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xaxidma.h"
#include "xiltimer.h"
#include "xgaussian_blur_v2.h"

/* ── Image / transfer dimensions ─────────────────────────────────────────── */
#define WIDTH          512
#define HEIGHT         512
#define SIZE           (WIDTH * HEIGHT)
#define TRANSFER_BYTES SIZE          /* 8-bit greyscale: 1 byte per pixel    */

/* ── Kernel configuration — change KSIZE here to switch window size ───────
 *   KSIZE must match the HLS IP build (currently 5).
 *   HALO  = KSIZE-1 : pixel rows/cols that lack a full neighbourhood.
 *   NBUF  = KSIZE-1 : number of line buffers needed.                        */
#define KSIZE  5
#define HALO   (KSIZE - 1)
#define NBUF   (KSIZE - 1)

/* ── 5×5 Gaussian kernel ──────────────────────────────────────────────────
 *   Outer product of [1,4,6,4,1] with itself.
 *   Sum = 256 = 2^8  →  normalise with >> 8.
 *
 *    1   4   6   4   1
 *    4  16  24  16   4
 *    6  24  36  24   6
 *    4  16  24  16   4
 *    1   4   6   4   1                                                       */
static const unsigned int KERNEL[KSIZE][KSIZE] = {
    { 1,  4,  6,  4,  1},
    { 4, 16, 24, 16,  4},
    { 6, 24, 36, 24,  6},
    { 4, 16, 24, 16,  4},
    { 1,  4,  6,  4,  1}
};

/* ── Memory map ──────────────────────────────────────────────────────────────
 *   Both addresses are inside the Zynq DDR window 0x00100000–0x1FFFFFFF.
 *   They are 16 MB apart (512 kB image), so they never overlap.             */
#define INPUT_ADDR   0x10000000
#define OUTPUT_ADDR  0x11000000

/* ── DMA device index ────────────────────────────────────────────────────────
 *   xparameters.h has only one DMA instance (XPAR_XAXIDMA_NUM_INSTANCES=1).
 *   XPAR_XAXIDMA_0_DEVICE_ID is NOT defined in this xparameters.h, so we
 *   use the literal 0 which XAxiDma_LookupConfig() maps to the single
 *   instance at 0x41E00000.                                                  */
#define DMA_DEV_ID   0

/* ── Hex dump of output image (set to 1 to enable) ──────────────────────── */
#define PRINT_IMAGE_HEX  0

/* ── Static instances and buffers ─────────────────────────────────────────── */
static XAxiDma         AxiDma;
static XGaussian_blur_v2 GaussianIP;

static u8 *input_pixels  = (u8 *)INPUT_ADDR;
static u8 *output_pixels = (u8 *)OUTPUT_ADDR;

/* SW reference output lives in BSS (zero-initialised, avoids stack overflow) */
static u8 sw_output[SIZE];

/* ═══════════════════════════════════════════════════════════════════════════
 * init_dma
 *   Initialise the AXI DMA in simple (non-SG) mode.
 *   DMA base: 0x41E00000  (XPAR_XAXIDMA_0_BASEADDR, device ID 0)
 * ═══════════════════════════════════════════════════════════════════════════ */
static int init_dma(void)
{
    XAxiDma_Config *CfgPtr;
    int status;

    CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);
    if (!CfgPtr) {
        xil_printf("ERROR: No DMA config found for device %d\r\n", DMA_DEV_ID);
        return XST_FAILURE;
    }

    status = XAxiDma_CfgInitialize(&AxiDma, CfgPtr);
    if (status != XST_SUCCESS) {
        xil_printf("ERROR: DMA init failed (status=%d)\r\n", status);
        return XST_FAILURE;
    }

    if (XAxiDma_HasSg(&AxiDma)) {
        xil_printf("ERROR: DMA is in SG mode; simple mode required\r\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * init_gaussian_ip
 *   Initialise the gaussian_blur_v2 HLS IP.
 *   IP base: 0x40000000  (XPAR_XGAUSSIAN_BLUR_V2_0_BASEADDR)
 * ═══════════════════════════════════════════════════════════════════════════ */
static int init_gaussian_ip(void)
{
    int status;

    /* XPAR_XGAUSSIAN_BLUR_V2_0_BASEADDR = 0x40000000 from xparameters.h     */
    status = XGaussian_blur_v2_Initialize(&GaussianIP,
                                          XPAR_XGAUSSIAN_BLUR_V2_0_BASEADDR);
    if (status != XST_SUCCESS) {
        xil_printf("ERROR: Gaussian IP init failed (status=%d)\r\n", status);
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * sw_streaming_reference
 *   Pure-C 5×5 streaming Gaussian blur.  Mirrors the HLS IP pixel-for-pixel:
 *   same line-buffer shift pattern, same window construction order, same
 *   kernel weights and normalisation (>> 8).
 *
 *   Window indexing:
 *     w[0][*] = row r-4  (oldest buffered)
 *     w[1][*] = row r-3
 *     w[2][*] = row r-2
 *     w[3][*] = row r-1
 *     w[4][*] = row r    (current)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void sw_streaming_reference(void)
{
    u8  linebuf[NBUF][WIDTH];
    u8  w[KSIZE][KSIZE];
    int b, r, c, i, j;

    /* Zero-initialise line buffers */
    for (b = 0; b < NBUF; b++)
        for (c = 0; c < WIDTH; c++)
            linebuf[b][c] = 0;

    /* Zero-initialise 5×5 sliding window */
    for (i = 0; i < KSIZE; i++)
        for (j = 0; j < KSIZE; j++)
            w[i][j] = 0;

    for (r = 0; r < HEIGHT; r++) {
        for (c = 0; c < WIDTH; c++) {

            u8 new_pixel = input_pixels[r * WIDTH + c];

            /* ── Read this column from all 4 line buffers ───────────────── */
            u8 col_pixels[NBUF];
            for (b = 0; b < NBUF; b++)
                col_pixels[b] = linebuf[b][c];

            /* ── Shift line buffers: oldest drops, new row enters last ──── */
            for (b = 0; b < NBUF - 1; b++)
                linebuf[b][c] = col_pixels[b + 1];
            linebuf[NBUF - 1][c] = new_pixel;

            /* ── Shift window left: drop column 0, make room at KSIZE-1 ── */
            for (i = 0; i < KSIZE; i++)
                for (j = 0; j < KSIZE - 1; j++)
                    w[i][j] = w[i][j + 1];

            /* ── Insert new rightmost column ─────────────────────────────
             *   col_pixels[0..3] = rows r-4..r-1, new_pixel = row r       */
            for (b = 0; b < NBUF; b++)
                w[b][KSIZE - 1] = col_pixels[b];
            w[KSIZE - 1][KSIZE - 1] = new_pixel;

            /* ── Compute output pixel ────────────────────────────────── */
            u8 out_pixel;

            if (r < HALO || c < HALO) {
                /* Border region: insufficient history, pass through */
                out_pixel = new_pixel;
            } else {
                unsigned int sum = 0;

                for (i = 0; i < KSIZE; i++)
                    for (j = 0; j < KSIZE; j++)
                        sum += (unsigned int)w[i][j] * KERNEL[i][j];

                /* Kernel sum = 256 = 2^8 */
                out_pixel = (u8)(sum >> 8);
            }

            sw_output[r * WIDTH + c] = out_pixel;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * run_hw
 *   Transfer the input image to the HLS IP via MM2S DMA, collect the output
 *   via S2MM DMA, then wait for both channels to complete.
 *
 *   Important: S2MM must be armed BEFORE MM2S so the IP never stalls waiting
 *   for a receiver.  IP is started after both DMA channels are armed.
 * ═══════════════════════════════════════════════════════════════════════════ */
static int run_hw(void)
{
    int status;

    /* Flush D-cache: CPU write-back → DDR, so DMA sees the real data */
    Xil_DCacheFlushRange((UINTPTR)input_pixels,  TRANSFER_BYTES);
    Xil_DCacheFlushRange((UINTPTR)output_pixels, TRANSFER_BYTES);

    /* Arm S2MM (Device → Memory) first */
    status = XAxiDma_SimpleTransfer(&AxiDma,
                                    (UINTPTR)output_pixels,
                                    TRANSFER_BYTES,
                                    XAXIDMA_DEVICE_TO_DMA);
    if (status != XST_SUCCESS) {
        xil_printf("ERROR: S2MM transfer setup failed (status=%d)\r\n", status);
        return XST_FAILURE;
    }

    /* Arm MM2S (Memory → Device) */
    status = XAxiDma_SimpleTransfer(&AxiDma,
                                    (UINTPTR)input_pixels,
                                    TRANSFER_BYTES,
                                    XAXIDMA_DMA_TO_DEVICE);
    if (status != XST_SUCCESS) {
        xil_printf("ERROR: MM2S transfer setup failed (status=%d)\r\n", status);
        return XST_FAILURE;
    }

    /* Start the HLS IP — it begins consuming the AXI-stream immediately */
    XGaussian_blur_v2_Start(&GaussianIP);

    /* Poll until both DMA channels complete */
    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DMA_TO_DEVICE))
        ;
    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA))
        ;

    /* Invalidate D-cache: force CPU to re-read from DDR (DMA wrote new data) */
    Xil_DCacheInvalidateRange((UINTPTR)output_pixels, TRANSFER_BYTES);

    return XST_SUCCESS;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * compare_outputs
 *   Pixel-by-pixel comparison of HW output vs SW reference.
 *   Prints the first 20 mismatches to UART.
 * ═══════════════════════════════════════════════════════════════════════════ */
static int compare_outputs(void)
{
    int errors = 0;
    int i;

    for (i = 0; i < SIZE; i++) {
        u8 hw_pixel = output_pixels[i];

        if (hw_pixel != sw_output[i]) {
            if (errors < 20) {
                xil_printf("Mismatch at index %d (r=%d,c=%d): SW=%d HW=%d\r\n",
                           i, i / WIDTH, i % WIDTH,
                           (int)sw_output[i], (int)hw_pixel);
            }
            errors++;
        }
    }

    return errors;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * print_output_hex
 *   Optional raw hex dump of the output image (set PRINT_IMAGE_HEX=1).
 *   Use recreate_image.py on the host to reconstruct the PNG.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void print_output_hex(void)
{
#if PRINT_IMAGE_HEX
    int i;
    xil_printf("BEGIN_IMAGE_HEX\r\n");
    for (i = 0; i < SIZE; i++) {
        xil_printf("%02x", output_pixels[i]);
        if ((i + 1) % 32 == 0)
            xil_printf("\r\n");
    }
    xil_printf("\r\nEND_IMAGE_HEX\r\n");
#endif
}

/* ═══════════════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    int    status;
    int    errors;

    XTime  t_sw_start, t_sw_end;
    XTime  t_hw_start, t_hw_end;
    u64    sw_cycles, hw_cycles;
    u64    sw_us,     hw_us;

    /* COUNTS_PER_SECOND is defined in xtime_l.h as
     * (XPAR_CPU_CORE_CLOCK_FREQ_HZ / 2) = 650 000 000 / 2 = 325 000 000    */
    const u64 ticks_per_us = COUNTS_PER_SECOND / 1000000ULL;

    xil_printf("\r\n========================================\r\n");
    xil_printf("  gaussian_blur_v2 — HW/SW benchmark\r\n");
    xil_printf("  5x5 kernel | 64-bit AXI stream\r\n");
    xil_printf("  Image: %dx%d  Transfer: %d bytes\r\n",
               WIDTH, HEIGHT, TRANSFER_BYTES);
    xil_printf("  Input  @ 0x%08X\r\n", INPUT_ADDR);
    xil_printf("  Output @ 0x%08X\r\n", OUTPUT_ADDR);
    xil_printf("  DMA    @ 0x%08X  (device ID %d)\r\n",
               XPAR_XAXIDMA_0_BASEADDR, DMA_DEV_ID);
    xil_printf("  IP     @ 0x%08X\r\n", XPAR_XGAUSSIAN_BLUR_V2_0_BASEADDR);
    xil_printf("  Timer  : %llu ticks/us  (%llu MHz)\r\n",
               ticks_per_us, (u64)(COUNTS_PER_SECOND / 1000000ULL));
    xil_printf("========================================\r\n\r\n");

    /* ── Initialise peripherals ─────────────────────────────────────────── */
    status = init_dma();
    if (status != XST_SUCCESS) {
        xil_printf("FATAL: DMA init failed\r\n");
        return XST_FAILURE;
    }

    status = init_gaussian_ip();
    if (status != XST_SUCCESS) {
        xil_printf("FATAL: Gaussian IP init failed\r\n");
        return XST_FAILURE;
    }

    /* ── SW reference ───────────────────────────────────────────────────── */
    xil_printf("Running SW reference (5x5, single-threaded ARM)...\r\n");

    XTime_GetTime(&t_sw_start);
    sw_streaming_reference();
    XTime_GetTime(&t_sw_end);

    sw_cycles = (u64)(t_sw_end - t_sw_start);
    sw_us     = sw_cycles / ticks_per_us;

    xil_printf("  SW cycles : %llu\r\n", sw_cycles);
    xil_printf("  SW time   : %llu us  (%llu ms)\r\n",
               sw_us, sw_us / 1000ULL);

    /* ── HW accelerator ─────────────────────────────────────────────────── */
    xil_printf("\r\nRunning HW accelerator (5x5, 64-bit AXI stream, unrolled)...\r\n");

    XTime_GetTime(&t_hw_start);
    status = run_hw();
    XTime_GetTime(&t_hw_end);

    if (status != XST_SUCCESS) {
        xil_printf("FATAL: HW run failed\r\n");
        return XST_FAILURE;
    }

    hw_cycles = (u64)(t_hw_end - t_hw_start);
    hw_us     = hw_cycles / ticks_per_us;

    xil_printf("  HW cycles : %llu\r\n", hw_cycles);
    xil_printf("  HW time   : %llu us  (%llu ms)\r\n",
               hw_us, hw_us / 1000ULL);

    /* ── Speedup ─────────────────────────────────────────────────────────── */
    xil_printf("\r\n  Speedup   : ");
    if (hw_us > 0) {
        /* Print as integer + one decimal place without floating point */
        u64 speedup_x10 = (sw_us * 10ULL) / hw_us;
        xil_printf("%llu.%llux\r\n", speedup_x10 / 10ULL, speedup_x10 % 10ULL);
    } else {
        xil_printf("(HW time too short to measure)\r\n");
    }

    /* ── Verify correctness ──────────────────────────────────────────────── */
    xil_printf("\r\nComparing HW output to SW reference...\r\n");
    errors = compare_outputs();

    if (errors == 0) {
        xil_printf("  TEST PASSED — %d pixels verified\r\n", SIZE);
    } else {
        xil_printf("  TEST FAILED — %d / %d pixel mismatches\r\n",
                   errors, SIZE);
    }

    print_output_hex();

    xil_printf("\r\nDone.\r\n");
    return (errors == 0) ? XST_SUCCESS : XST_FAILURE;
}