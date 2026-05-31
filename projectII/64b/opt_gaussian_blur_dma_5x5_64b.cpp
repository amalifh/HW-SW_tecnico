#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>

// ─── Image dimensions ────────────────────────────────────────────────────────
// WIDTH must be a multiple of PIXELS_PER_WORD (8).
// 512 is fine. 1280 is also fine (1280 / 8 = 160).
#define WIDTH  512
#define HEIGHT 512
#define SIZE   (WIDTH * HEIGHT)

// ─── Kernel ──────────────────────────────────────────────────────────────────
#define KSIZE  5
#define HALO   (KSIZE - 1)   // = 4
#define NBUF   (KSIZE - 1)   // = 4 line buffers


// 5×5 Gaussian — outer product of [1,4,6,4,1] with itself.
// Sum = 256 = 2^8  →  normalise with >> 8.
static const ap_uint<6> KERNEL[KSIZE][KSIZE] = {
    { 1,  4,  6,  4,  1},
    { 4, 16, 24, 16,  4},
    { 6, 24, 36, 24,  6},
    { 4, 16, 24, 16,  4},
    { 1,  4,  6,  4,  1}
};

// ─── AXI stream type: 64-bit per transfer (8 pixels packed) ──────────────────
// Each 64-bit word carries 8 grayscale pixels, byte 0 = leftmost pixel.
// keep/strb = 0xFF = all 8 bytes valid.
#define PIXELS_PER_WORD 8
#define AXI_WIDTH       64          // bits

typedef ap_axis<AXI_WIDTH,0,0,0> axis_t;
typedef ap_uint<8>                pixel_t;
typedef ap_uint<AXI_WIDTH>        word_t;


static_assert(WIDTH % PIXELS_PER_WORD == 0,
    "WIDTH must be a multiple of PIXELS_PER_WORD (8) for 64-bit AXI packing");
    
// ─── Helpers ─────────────────────────────────────────────────────────────────
// Unpack byte p (0=LSB side) from a 64-bit word.
static pixel_t unpack_pixel(word_t word, int p) {
#pragma HLS INLINE
    return word(p*8+7, p*8);
}

// Pack 8 pixels into one 64-bit word.
static word_t pack_pixels(pixel_t px[PIXELS_PER_WORD]) {
#pragma HLS INLINE
    word_t word = 0;
    for (int p = 0; p < PIXELS_PER_WORD; p++) {
#pragma HLS UNROLL
        word(p*8+7, p*8) = px[p];
    }
    return word;
}

static axis_t make_axis_word(word_t data, bool last) {
    axis_t word;
    word.data = data;
    word.keep = 0xFF;   // all 8 bytes valid
    word.strb = 0xFF;
    word.last = last;
    return word;
}

// ─── Top-level HLS function ───────────────────────────────────────────────────
void gaussian_blur_dma(hls::stream<axis_t>& in_stream,
                       hls::stream<axis_t>& out_stream) {

#pragma HLS INTERFACE axis      port=in_stream
#pragma HLS INTERFACE axis      port=out_stream
#pragma HLS INTERFACE s_axilite port=return bundle=CTRL

    // ── Line buffers ─────────────────────────────────────────────────────────
    // Partitioned CYCLIC by PIXELS_PER_WORD along the column dimension.
    // This gives 8 independent BRAM banks per row so all 8 pixels in one
    // 64-bit word can be read/written in the same cycle without bank conflict.
    // BRAM partitioning: linebuf gets a second partition pragma — cyclic factor=8 on dim 2 (columns).
    // This splits each of the 4 row-buffers into 8 independent BRAM banks.
    // Pixel p always hits bank p % 8, so all 8 pixels in one word land on different banks — no port conflict, the whole unrolled pixel loop resolves without stalling.
    static pixel_t linebuf[NBUF][WIDTH];
#pragma HLS ARRAY_PARTITION variable=linebuf complete    dim=1   // all 4 rows independent
#pragma HLS ARRAY_PARTITION variable=linebuf cyclic      dim=2 factor=8  // 8 column banks
#pragma HLS BIND_STORAGE    variable=linebuf type=ram_2p impl=bram

    // ── 5×5 window registers ─────────────────────────────────────────────────
    pixel_t w[KSIZE][KSIZE];
#pragma HLS ARRAY_PARTITION variable=w complete dim=0

    // Initialise window
    for (int i = 0; i < KSIZE; i++)
#pragma HLS UNROLL
        for (int j = 0; j < KSIZE; j++)
#pragma HLS UNROLL
            w[i][j] = 0;

    // ── Main loop ─────────────────────────────────────────────────────────────
    // Outer loops: one AXI word read/write per iteration of WORD_LOOP.
    // PIXEL_LOOP is unrolled, so all 8 pixels are processed in one pipeline stage.
    ROW_LOOP:
    for (int r = 0; r < HEIGHT; r++) {
        WORD_LOOP:
        for (int cw = 0; cw < WIDTH / PIXELS_PER_WORD; cw++) {
#pragma HLS PIPELINE II=1

            // ── 1. Read one 64-bit AXI word (= 8 pixels) ─────────────────
            axis_t in_word   = in_stream.read();
            word_t in_data   = in_word.data;

            // ── 2. Process 8 pixels sequentially inside this word ─────────
            // The window state (w[][]) carries over pixel-to-pixel, so this
            // loop is inherently sequential. With UNROLL and PIPELINE II=1
            // on the outer loop, HLS will schedule it as a single wide stage.
            pixel_t out_px[PIXELS_PER_WORD];
#pragma HLS ARRAY_PARTITION variable=out_px complete

            PIXEL_LOOP:
            for (int p = 0; p < PIXELS_PER_WORD; p++) {
#pragma HLS UNROLL

                int c = cw * PIXELS_PER_WORD + p;  // absolute column index

                pixel_t new_pixel = unpack_pixel(in_data, p);

                // Read column c from all line buffers
                pixel_t col_pixels[NBUF];
#pragma HLS ARRAY_PARTITION variable=col_pixels complete
                for (int b = 0; b < NBUF; b++)
#pragma HLS UNROLL
                    col_pixels[b] = linebuf[b][c];

                // Shift line buffers
                for (int b = 0; b < NBUF - 1; b++)
#pragma HLS UNROLL
                    linebuf[b][c] = col_pixels[b + 1];
                linebuf[NBUF - 1][c] = new_pixel;

                // Shift window left
                for (int i = 0; i < KSIZE; i++) {
#pragma HLS UNROLL
                    for (int j = 0; j < KSIZE - 1; j++) {
#pragma HLS UNROLL
                        w[i][j] = w[i][j + 1];
                    }
                }

                // Insert new right column
                for (int b = 0; b < NBUF; b++)
#pragma HLS UNROLL
                    w[b][KSIZE - 1] = col_pixels[b];
                w[KSIZE - 1][KSIZE - 1] = new_pixel;

                // Compute output
                if (r < HALO || c < HALO) {
                    out_px[p] = new_pixel;
                } else {
                    ap_uint<20> sum = 0;
                    for (int i = 0; i < KSIZE; i++) {
#pragma HLS UNROLL
                        for (int j = 0; j < KSIZE; j++) {
#pragma HLS UNROLL
                            sum += (ap_uint<20>)w[i][j] * KERNEL[i][j];
                        }
                    }
                    out_px[p] = (pixel_t)(sum >> 8);
                }
            }

            // ── 3. Pack 8 output pixels and write one 64-bit AXI word ─────
            word_t out_data = pack_pixels(out_px);

            bool last = (r == HEIGHT - 1 && cw == WIDTH / PIXELS_PER_WORD - 1);
            out_stream.write(make_axis_word(out_data, last));
        }
    }
}
