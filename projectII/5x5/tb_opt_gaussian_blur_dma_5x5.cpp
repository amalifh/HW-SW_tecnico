#include <iostream>
#include <cstdlib>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>

#define WIDTH  512
#define HEIGHT 512
#define SIZE   (WIDTH * HEIGHT)

#define KSIZE  5
#define HALO   (KSIZE - 1)   // = 4

typedef ap_axis<8,0,0,0> axis_t;
typedef ap_uint<8>        pixel_t;

// ── Forward declaration of DUT ───────────────────────────────────────────────
void gaussian_blur_dma(hls::stream<axis_t>& in_stream,
                       hls::stream<axis_t>& out_stream);

// ── 5×5 Gaussian kernel (must match hardware) ────────────────────────────────
static const int KERNEL[KSIZE][KSIZE] = {
    { 1,  4,  6,  4,  1},
    { 4, 16, 24, 16,  4},
    { 6, 24, 36, 24,  6},
    { 4, 16, 24, 16,  4},
    { 1,  4,  6,  4,  1}
};

static axis_t make_input_word(pixel_t pix, bool last) {
    axis_t word;
    word.data = pix;
    word.keep = 1;
    word.strb = 1;
    word.last = last;
    return word;
}

// ── Software reference (mirrors HLS streaming behaviour exactly) ─────────────
void sw_streaming_reference(pixel_t input[HEIGHT][WIDTH],
                            pixel_t output[HEIGHT][WIDTH]) {

    // NBUF = KSIZE-1 = 4 line buffers
    pixel_t linebuf[KSIZE-1][WIDTH];
    for (int b = 0; b < KSIZE-1; b++)
        for (int c = 0; c < WIDTH; c++)
            linebuf[b][c] = 0;

    pixel_t w[KSIZE][KSIZE];
    for (int i = 0; i < KSIZE; i++)
        for (int j = 0; j < KSIZE; j++)
            w[i][j] = 0;

    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {

            pixel_t new_pixel = input[r][c];

            // Read column from line buffers
            pixel_t col_pixels[KSIZE-1];
            for (int b = 0; b < KSIZE-1; b++)
                col_pixels[b] = linebuf[b][c];

            // Shift line buffers
            for (int b = 0; b < KSIZE-2; b++)
                linebuf[b][c] = col_pixels[b+1];
            linebuf[KSIZE-2][c] = new_pixel;

            // Shift window columns left
            for (int i = 0; i < KSIZE; i++)
                for (int j = 0; j < KSIZE-1; j++)
                    w[i][j] = w[i][j+1];

            // Insert new right column
            for (int b = 0; b < KSIZE-1; b++)
                w[b][KSIZE-1] = col_pixels[b];
            w[KSIZE-1][KSIZE-1] = new_pixel;

            pixel_t out_pixel;
            if (r < HALO || c < HALO) {
                out_pixel = new_pixel;
            } else {
                unsigned int sum = 0;
                for (int i = 0; i < KSIZE; i++)
                    for (int j = 0; j < KSIZE; j++)
                        sum += (unsigned int)w[i][j] * KERNEL[i][j];
                out_pixel = (pixel_t)(sum >> 8);
            }

            output[r][c] = out_pixel;
        }
    }
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main() {
    hls::stream<axis_t> in_stream;
    hls::stream<axis_t> out_stream;

    // Allocate on heap
    static pixel_t input [HEIGHT][WIDTH];
    static pixel_t sw_out[HEIGHT][WIDTH];
    static pixel_t hw_out[HEIGHT][WIDTH];

    // ── Generate deterministic test image ────────────────────────────────────
    // Left half dark, right half bright, bright square in top-left quadrant
    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
            input[r][c] = (c < WIDTH / 2) ? 40 : 200;
            if (r > 80 && r < 160 && c > 80 && c < 160)
                input[r][c] = 255;
        }
    }

    // ── SW reference ─────────────────────────────────────────────────────────
    sw_streaming_reference(input, sw_out);

    // ── Push image into HLS stream ───────────────────────────────────────────
    for (int r = 0; r < HEIGHT; r++)
        for (int c = 0; c < WIDTH; c++) {
            bool last = (r == HEIGHT-1 && c == WIDTH-1);
            in_stream.write(make_input_word(input[r][c], last));
        }

    // ── Run HW (HLS simulation) ───────────────────────────────────────────────
    gaussian_blur_dma(in_stream, out_stream);

    // ── Compare ───────────────────────────────────────────────────────────────
    int errors       = 0;
    int tlast_errors = 0;

    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
            axis_t word = out_stream.read();
            hw_out[r][c] = word.data;

            bool expected_last = (r == HEIGHT-1 && c == WIDTH-1);
            if ((bool)word.last != expected_last) {
                std::cout << "TLAST error at (" << r << "," << c << "): expected "
                          << expected_last << " got " << word.last << "\n";
                tlast_errors++;
            }

            if (hw_out[r][c] != sw_out[r][c]) {
                if (errors < 20)
                    std::cout << "Mismatch at (" << r << "," << c << "): SW="
                              << (int)sw_out[r][c]
                              << " HW=" << (int)hw_out[r][c] << "\n";
                errors++;
            }
        }
    }

    std::cout << "\nImage: " << WIDTH << "x" << HEIGHT
              << "  Kernel: " << KSIZE << "x" << KSIZE << "\n";
    std::cout << "Pixel mismatches : " << errors       << "\n";
    std::cout << "TLAST errors     : " << tlast_errors << "\n";

    if (errors == 0 && tlast_errors == 0) {
        std::cout << "TEST PASSED\n";
        return 0;
    } else {
        std::cout << "TEST FAILED\n";
        return 1;
    }
}
