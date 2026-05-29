#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>

// ─── Image dimensions ────────────────────────────────────────────────────────
// Keep power-of-2 width for efficient address arithmetic (professor's tip)
#define WIDTH  1280
#define HEIGHT 720
#define SIZE   (WIDTH * HEIGHT)

// ─── Kernel size ─────────────────────────────────────────────────────────────
// Change KSIZE here to switch between 3x3, 5x5, 7x7, etc.
// Must be odd. HALO = pixels before valid output starts per row/col.
#define KSIZE  5
#define HALO   (KSIZE - 1)          // = 4 for 5x5
#define NBUF   (KSIZE - 1)          // number of line buffers needed = 4

// ─── AXI stream type: 8-bit per transfer ─────────────────────────────────────
typedef ap_axis<8,0,0,0> axis_t;
typedef ap_uint<8>        pixel_t;

// ─── 5×5 Gaussian kernel (σ≈1.0, sum=256, so shift by 8) ────────────────────
//
//   1  4  6  4  1
//   4 16 24 16  4
//   6 24 36 24  6
//   4 16 24 16  4
//   1  4  6  4  1
//
// Sum = 256  →  divide by 256  →  right-shift 8 bits.
// All coefficients are powers-of-2 friendly, so we use shifts where possible.
static const ap_uint<6> KERNEL[KSIZE][KSIZE] = {
    { 1,  4,  6,  4,  1},
    { 4, 16, 24, 16,  4},
    { 6, 24, 36, 24,  6},
    { 4, 16, 24, 16,  4},
    { 1,  4,  6,  4,  1}
};

// ─── Helper ──────────────────────────────────────────────────────────────────
static axis_t make_axis_word(pixel_t pix, bool last) {
    axis_t word;
    word.data = pix;
    word.keep = 1;
    word.strb = 1;
    word.last = last;
    return word;
}

// ─── Top-level HLS function ──────────────────────────────────────────────────
void gaussian_blur_dma(hls::stream<axis_t>& in_stream,
                       hls::stream<axis_t>& out_stream) {

#pragma HLS INTERFACE axis      port=in_stream
#pragma HLS INTERFACE axis      port=out_stream
#pragma HLS INTERFACE s_axilite port=return bundle=CTRL

    // ── Line buffers: NBUF rows of WIDTH pixels each ─────────────────────────
    // For 5×5 we need 4 line buffers (the 5th "row" is the live incoming pixel).
    static pixel_t linebuf[NBUF][WIDTH];
#pragma HLS ARRAY_PARTITION variable=linebuf complete dim=1   // partition across rows
#pragma HLS BIND_STORAGE      variable=linebuf type=ram_2p impl=bram

    // ── 5×5 window registers ─────────────────────────────────────────────────
    // w[row][col], fully partitioned so HLS sees them as registers.
    pixel_t w[KSIZE][KSIZE];
#pragma HLS ARRAY_PARTITION variable=w complete dim=0

    // Initialise window to zero (important: static not used here on purpose —
    // we want the function to be re-entrant / resettable for co-simulation).
    for (int i = 0; i < KSIZE; i++)
#pragma HLS UNROLL
        for (int j = 0; j < KSIZE; j++)
#pragma HLS UNROLL
            w[i][j] = 0;

    // ── Main streaming loop ───────────────────────────────────────────────────
    ROW_LOOP:
    for (int r = 0; r < HEIGHT; r++) {
        COL_LOOP:
        for (int c = 0; c < WIDTH; c++) {
#pragma HLS PIPELINE II=1

            // 1. Read one pixel from AXI stream
            axis_t  in_word   = in_stream.read();
            pixel_t new_pixel = in_word.data;

            // 2. Read column c from all line buffers (oldest → newest)
            pixel_t col_pixels[NBUF];
#pragma HLS ARRAY_PARTITION variable=col_pixels complete
            for (int b = 0; b < NBUF; b++)
#pragma HLS UNROLL
                col_pixels[b] = linebuf[b][c];

            // 3. Write into line buffers (shift: buf[0] ← buf[1] ← … ← new)
            for (int b = 0; b < NBUF - 1; b++)
#pragma HLS UNROLL
                linebuf[b][c] = col_pixels[b + 1];
            linebuf[NBUF - 1][c] = new_pixel;

            // 4. Shift the 5×5 window left by one column
            //    row 0 = oldest line, row KSIZE-1 = current incoming row
            for (int i = 0; i < KSIZE; i++) {
#pragma HLS UNROLL
                for (int j = 0; j < KSIZE - 1; j++) {
#pragma HLS UNROLL
                    w[i][j] = w[i][j + 1];
                }
            }

            // 5. Insert new column on the right of the window
            //    col_pixels[0] = oldest buffered row (row r-4)
            //    col_pixels[3] = most recent buffered row (row r-1)
            //    new_pixel     = current row r
            for (int b = 0; b < NBUF; b++)
#pragma HLS UNROLL
                w[b][KSIZE - 1] = col_pixels[b];
            w[KSIZE - 1][KSIZE - 1] = new_pixel;

            // 6. Compute output pixel
            pixel_t out_pixel;

            if (r < HALO || c < HALO) {
                // Border region: pass through (no full window available)
                out_pixel = new_pixel;
            } else {
                // Accumulate 5×5 weighted sum
                // Use ap_uint<20> to avoid overflow:
                // max pixel=255, max coeff=36, 25 cells → max sum = 255*256 = 65280 < 2^17
                ap_uint<20> sum = 0;

                ACCUM_ROW:
                for (int i = 0; i < KSIZE; i++) {
#pragma HLS UNROLL
                    ACCUM_COL:
                    for (int j = 0; j < KSIZE; j++) {
#pragma HLS UNROLL
                        sum += (ap_uint<20>)w[i][j] * KERNEL[i][j];
                    }
                }

                // Normalise: divide by 256 = right-shift 8
                out_pixel = (pixel_t)(sum >> 8);
            }

            // 7. Write output
            bool last = (r == HEIGHT - 1 && c == WIDTH - 1);
            out_stream.write(make_axis_word(out_pixel, last));
        }
    }
}
