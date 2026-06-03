#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>

// ─────────────────────────────────────────────────────────────────────────────
// True 8-PPC 5x5 Gaussian Blur
//
// One AXI stream word = 64 bits = 8 grayscale pixels.
// This implementation processes one 64-bit word per loop iteration.
// The inner lane loop is fully unrolled, so the hardware computes 8 output
// pixels in parallel.
// ─────────────────────────────────────────────────────────────────────────────

#define WIDTH  512
#define HEIGHT 512
#define PPC    8
#define COLS   (WIDTH / PPC)

#define KSIZE  5
#define HALO   (KSIZE - 1)     // Streaming version: valid after 4 rows/cols
#define NBUF   (KSIZE - 1)     // 4 previous rows stored in line buffers

typedef ap_axis<64, 0, 0, 0> axis_wide_t;
typedef ap_uint<64> vec_t;
typedef ap_uint<8>  pixel_t;

static const ap_uint<8> KERNEL[KSIZE][KSIZE] = {
    { 1,  4,  6,  4,  1},
    { 4, 16, 24, 16,  4},
    { 6, 24, 36, 24,  6},
    { 4, 16, 24, 16,  4},
    { 1,  4,  6,  4,  1}
};

static pixel_t get_pix(vec_t word, int i)
{
#pragma HLS INLINE
    return word.range(i * 8 + 7, i * 8);
}

static void set_pix(vec_t &word, int i, pixel_t pix)
{
#pragma HLS INLINE
    word.range(i * 8 + 7, i * 8) = pix;
}

/* Get pixel i from current 8-pixel word.
 * If i is negative, fetch from the previous 8-pixel word.
 *
 * For a 5x5 streaming window we only need i-4..i.
 * Since PPC=8, one previous word is enough for the left halo.
 */
static pixel_t get_with_left(vec_t left, vec_t cur, int i)
{
#pragma HLS INLINE
    if (i < 0)
        return get_pix(left, PPC + i);
    else
        return get_pix(cur, i);
}

static pixel_t gaussian5x5_for_lane(
    vec_t left0, vec_t row0,
    vec_t left1, vec_t row1,
    vec_t left2, vec_t row2,
    vec_t left3, vec_t row3,
    vec_t left4, vec_t row4,
    int lane
)
{
#pragma HLS INLINE

    ap_uint<17> sum = 0;

    // Window columns are lane-4, lane-3, lane-2, lane-1, lane.
    kernel_row_loop:
    for (int kr = 0; kr < KSIZE; kr++) {
#pragma HLS UNROLL
        kernel_col_loop:
        for (int kc = 0; kc < KSIZE; kc++) {
#pragma HLS UNROLL
            int offset = lane - (KSIZE - 1) + kc;

            pixel_t pix;

            if (kr == 0)
                pix = get_with_left(left0, row0, offset);
            else if (kr == 1)
                pix = get_with_left(left1, row1, offset);
            else if (kr == 2)
                pix = get_with_left(left2, row2, offset);
            else if (kr == 3)
                pix = get_with_left(left3, row3, offset);
            else
                pix = get_with_left(left4, row4, offset);

            sum += (ap_uint<17>)pix * KERNEL[kr][kc];
        }
    }

    // Kernel sum = 256 = 2^8
    return (pixel_t)(sum >> 8);
}

static axis_wide_t make_axis_word(vec_t data, bool last)
{
#pragma HLS INLINE

    axis_wide_t word;
    word.data = data;
    word.keep = 0xFF;
    word.strb = 0xFF;
    word.last = last;
    return word;
}

void gaussian_blur_v2(hls::stream<axis_wide_t>& in_stream,
                      hls::stream<axis_wide_t>& out_stream)
{
#pragma HLS INTERFACE axis port=in_stream
#pragma HLS INTERFACE axis port=out_stream
#pragma HLS INTERFACE s_axilite port=return bundle=CTRL

    static vec_t linebuf[NBUF][COLS];

#pragma HLS BIND_STORAGE variable=linebuf type=ram_2p impl=bram
#pragma HLS ARRAY_PARTITION variable=linebuf dim=1 complete

    // Clear line buffers at the start of each image.
    init_linebuf:
    for (int b = 0; b < NBUF; b++) {
#pragma HLS UNROLL
        for (int cb = 0; cb < COLS; cb++) {
#pragma HLS PIPELINE II=1
            linebuf[b][cb] = 0;
        }
    }

    row_loop:
    for (int r = 0; r < HEIGHT; r++) {

        // Previous 8-pixel block for each row in the 5x5 window.
        // Needed for pixels near the left edge of each 8-pixel word.
        vec_t left0 = 0;
        vec_t left1 = 0;
        vec_t left2 = 0;
        vec_t left3 = 0;
        vec_t left4 = 0;

        col_block_loop:
        for (int cb = 0; cb < COLS; cb++) {
#pragma HLS PIPELINE II=1

            axis_wide_t in_axis = in_stream.read();
            vec_t row4 = in_axis.data;      // current row r
            vec_t row0 = linebuf[0][cb];    // row r-4
            vec_t row1 = linebuf[1][cb];    // row r-3
            vec_t row2 = linebuf[2][cb];    // row r-2
            vec_t row3 = linebuf[3][cb];    // row r-1

            // Shift line buffers vertically.
            linebuf[0][cb] = row1;
            linebuf[1][cb] = row2;
            linebuf[2][cb] = row3;
            linebuf[3][cb] = row4;

            vec_t out_vec = 0;

            lane_loop:
            for (int lane = 0; lane < PPC; lane++) {
#pragma HLS UNROLL

                int c = cb * PPC + lane;
                pixel_t new_pixel = get_pix(row4, lane);
                pixel_t out_pixel;

                if (r < HALO || c < HALO) {
                    // Streaming border behavior: pass through until full 5x5
                    // history exists.
                    out_pixel = new_pixel;
                } else {
                    out_pixel = gaussian5x5_for_lane(
                        left0, row0,
                        left1, row1,
                        left2, row2,
                        left3, row3,
                        left4, row4,
                        lane
                    );
                }

                set_pix(out_vec, lane, out_pixel);
            }

            // Update left-neighbour vectors for the next 8-pixel block.
            left0 = row0;
            left1 = row1;
            left2 = row2;
            left3 = row3;
            left4 = row4;

            bool last = (r == HEIGHT - 1 && cb == COLS - 1);
            out_stream.write(make_axis_word(out_vec, last));
        }
    }
}
