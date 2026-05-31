#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>

// ─────────────────────────────────────────────────────────────────────────────
// Configuration: 512x512 base-2 image with 5x5 Gaussian kernel
// ─────────────────────────────────────────────────────────────────────────────
#define WIDTH  512
#define HEIGHT 512
#define SIZE   (WIDTH * HEIGHT)

#define KSIZE  5
#define HALO   (KSIZE - 1)     // = 4: rows/cols before valid output
#define NBUF   (KSIZE - 1)     // = 4 line buffers

// 64-bit AXI stream: 8 pixels per transaction (8 bits each)
typedef ap_axis<64, 0, 0, 0> axis_wide_t;

// 8-bit pixel type
typedef ap_uint<8> pixel_t;

// ─────────────────────────────────────────────────────────────────────────────
// 5x5 Gaussian kernel: outer product of [1,4,6,4,1]
// Sum = 256 = 2^8, normalise with >> 8
//
//   1  4  6  4  1
//   4 16 24 16  4
//   6 24 36 24  6
//   4 16 24 16  4
//   1  4  6  4  1
// ─────────────────────────────────────────────────────────────────────────────
static const ap_uint<9> KERNEL[KSIZE][KSIZE] = {
    { 1,  4,  6,  4,  1},
    { 4, 16, 24, 16,  4},
    { 6, 24, 36, 24,  6},
    { 4, 16, 24, 16,  4},
    { 1,  4,  6,  4,  1}
};

// ─────────────────────────────────────────────────────────────────────────────
// Pixel unpacking: extract pixel i from 64-bit word (LSB = pixel 0)
// ─────────────────────────────────────────────────────────────────────────────
inline pixel_t extract_pixel(ap_uint<64> word, int idx) {
#pragma HLS INLINE
    return (pixel_t)(word >> (idx * 8));
}

// ─────────────────────────────────────────────────────────────────────────────
// Pixel packing: insert pixel i into 64-bit word
// ─────────────────────────────────────────────────────────────────────────────
inline ap_uint<64> pack_pixel(ap_uint<64> word, int idx, pixel_t pix) {
#pragma HLS INLINE
    ap_uint<64> mask = ((ap_uint<64>)0xFF) << (idx * 8);
    word = word & ~mask;
    word = word | (((ap_uint<64>)pix) << (idx * 8));
    return word;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main HLS Gaussian Blur IP: 5x5 kernel, 64-bit AXI stream I/O
// ─────────────────────────────────────────────────────────────────────────────
void gaussian_blur_dma_5x5(hls::stream<axis_wide_t>& in_stream,
                            hls::stream<axis_wide_t>& out_stream) {

#pragma HLS INTERFACE axis port=in_stream
#pragma HLS INTERFACE axis port=out_stream
#pragma HLS INTERFACE s_axilite port=return bundle=CTRL

    // ─────────────────────────────────────────────────────────────────────────
    // 4 line buffers for 5x5 kernel: oldest to newest row
    // ─────────────────────────────────────────────────────────────────────────
    static pixel_t linebuf[NBUF][WIDTH];

#pragma HLS BIND_STORAGE variable=linebuf type=ram_2p impl=bram

    // Partition line buffers for parallel access
#pragma HLS ARRAY_PARTITION variable=linebuf dim=1 complete

    // ─────────────────────────────────────────────────────────────────────────
    // 5x5 sliding window: w[row][col], row 0 = oldest, row 4 = newest
    // ─────────────────────────────────────────────────────────────────────────
    pixel_t w[KSIZE][KSIZE];

#pragma HLS ARRAY_PARTITION variable=w complete dim=0

    // Initialize window to zero
    init_window_loop:
    for (int i = 0; i < KSIZE; i++) {
#pragma HLS UNROLL factor=5
        for (int j = 0; j < KSIZE; j++) {
#pragma HLS UNROLL factor=5
            w[i][j] = 0;
        }
    }

    // Initialize line buffers to zero
    init_linebuf_loop:
    for (int b = 0; b < NBUF; b++) {
#pragma HLS UNROLL factor=4
        for (int c = 0; c < WIDTH; c++) {
#pragma HLS PIPELINE II=1
            linebuf[b][c] = 0;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Input/Output buffering for 8-pixel transactions
    // ─────────────────────────────────────────────────────────────────────────
    pixel_t in_buffer[8];
    pixel_t out_buffer[8];
    int in_count = 0;
    int out_count = 0;

    ap_uint<64> out_word = 0;

    int total_pixels = 0;
    int pixels_written = 0;

    // ─────────────────────────────────────────────────────────────────────────
    // Main processing loop: iterate through all pixels
    // ─────────────────────────────────────────────────────────────────────────
    main_row_loop:
    for (int r = 0; r < HEIGHT; r++) {

        main_col_loop:
        for (int c = 0; c < WIDTH; c++) {

#pragma HLS PIPELINE II=1

            // ─────────────────────────────────────────────────────────────────
            // Read pixel from input stream every 8 pixels
            // ─────────────────────────────────────────────────────────────────
            if (in_count == 0) {
                axis_wide_t in_word = in_stream.read();
                read_pixels_unroll:
                for (int p = 0; p < 8; p++) {
#pragma HLS UNROLL
                    in_buffer[p] = extract_pixel(in_word.data, p);
                }
            }

            pixel_t new_pixel = in_buffer[in_count];
            in_count = (in_count == 7) ? 0 : (in_count + 1);

            // ─────────────────────────────────────────────────────────────────
            // Read column from all line buffers
            // ─────────────────────────────────────────────────────────────────
            pixel_t col_pixels[NBUF];

#pragma HLS ARRAY_PARTITION variable=col_pixels complete
#pragma HLS UNROLL factor=4

            read_col_unroll:
            for (int b = 0; b < NBUF; b++) {
#pragma HLS UNROLL
                col_pixels[b] = linebuf[b][c];
            }

            // ─────────────────────────────────────────────────────────────────
            // Shift line buffers: oldest drops, newest row enters
            // ─────────────────────────────────────────────────────────────────
            shift_linebuf_unroll:
            for (int b = 0; b < NBUF - 1; b++) {
#pragma HLS UNROLL
                linebuf[b][c] = col_pixels[b + 1];
            }
            linebuf[NBUF - 1][c] = new_pixel;

            // ─────────────────────────────────────────────────────────────────
            // Shift window left by one column
            // ─────────────────────────────────────────────────────────────────
            shift_window_unroll:
            for (int i = 0; i < KSIZE; i++) {
#pragma HLS UNROLL factor=5
                for (int j = 0; j < KSIZE - 1; j++) {
#pragma HLS UNROLL factor=4
                    w[i][j] = w[i][j + 1];
                }
            }

            // ─────────────────────────────────────────────────────────────────
            // Insert new rightmost column into window
            // col_pixels[0] = row r-4, ..., col_pixels[3] = row r-1
            // new_pixel = row r
            // ─────────────────────────────────────────────────────────────────
            insert_col_unroll:
            for (int b = 0; b < NBUF; b++) {
#pragma HLS UNROLL
                w[b][KSIZE - 1] = col_pixels[b];
            }
            w[KSIZE - 1][KSIZE - 1] = new_pixel;

            // ─────────────────────────────────────────────────────────────────
            // Compute output pixel
            // ─────────────────────────────────────────────────────────────────
            pixel_t out_pixel;

            if (r < HALO || c < HALO) {
                // Border: insufficient history, pass through
                out_pixel = new_pixel;
            } else {
                // Apply 5x5 Gaussian convolution
                ap_uint<17> sum = 0;

                kernel_row_unroll:
                for (int i = 0; i < KSIZE; i++) {
#pragma HLS UNROLL factor=5
                    kernel_col_unroll:
                    for (int j = 0; j < KSIZE; j++) {
#pragma HLS UNROLL factor=5
                        sum += (ap_uint<17>)w[i][j] * KERNEL[i][j];
                    }
                }

                // Normalise: kernel sum = 256 = 2^8
                out_pixel = (pixel_t)(sum >> 8);
            }

            // ─────────────────────────────────────────────────────────────────
            // Buffer output pixel
            // ─────────────────────────────────────────────────────────────────
            out_buffer[out_count] = out_pixel;
            out_count = (out_count == 7) ? 0 : (out_count + 1);

            // ─────────────────────────────────────────────────────────────────
            // Write to output stream every 8 pixels
            // ─────────────────────────────────────────────────────────────────
            if (out_count == 0) {
                pack_pixels_unroll:
                for (int p = 0; p < 8; p++) {
#pragma HLS UNROLL
                    out_word = pack_pixel(out_word, p, out_buffer[p]);
                }

                bool is_last = (r == HEIGHT - 1 && c == WIDTH - 1);
                axis_wide_t out_axis;
                out_axis.data = out_word;
                out_axis.keep = 0xFF;  // All 8 bytes valid
                out_axis.strb = 0xFF;
                out_axis.last = is_last;

                out_stream.write(out_axis);
                out_word = 0;
            }

            total_pixels++;
        }
    }
}
