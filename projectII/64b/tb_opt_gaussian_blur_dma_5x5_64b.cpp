#include <iostream>
#include <cstring>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>

#define WIDTH  512
#define HEIGHT 512
#define SIZE   (WIDTH * HEIGHT)

#define KSIZE           5
#define HALO            (KSIZE - 1)
#define PIXELS_PER_WORD 8
#define AXI_WIDTH       64

typedef ap_axis<AXI_WIDTH,0,0,0> axis_t;
typedef ap_uint<8>                pixel_t;
typedef ap_uint<AXI_WIDTH>        word_t;

void gaussian_blur_dma(hls::stream<axis_t>& in_stream,
                       hls::stream<axis_t>& out_stream);

// ── Kernel (must match HLS core) ─────────────────────────────────────────────
static const int KERNEL[KSIZE][KSIZE] = {
    { 1,  4,  6,  4,  1},
    { 4, 16, 24, 16,  4},
    { 6, 24, 36, 24,  6},
    { 4, 16, 24, 16,  4},
    { 1,  4,  6,  4,  1}
};

// ── Pack / unpack helpers ─────────────────────────────────────────────────────
static word_t pack_pixels(pixel_t px[PIXELS_PER_WORD]) {
    word_t w = 0;
    for (int p = 0; p < PIXELS_PER_WORD; p++)
        w(p*8+7, p*8) = px[p];
    return w;
}

static pixel_t unpack_pixel(word_t w, int p) {
    return w(p*8+7, p*8);
}

static axis_t make_input_word(word_t data, bool last) {
    axis_t word;
    word.data = data;
    word.keep = 0xFF;
    word.strb = 0xFF;
    word.last = last;
    return word;
}

// ── SW reference — mirrors HLS exactly ───────────────────────────────────────
void sw_streaming_reference(pixel_t input[HEIGHT][WIDTH],
                            pixel_t output[HEIGHT][WIDTH]) {

    pixel_t linebuf[KSIZE-1][WIDTH];
    memset(linebuf, 0, sizeof(linebuf));

    pixel_t w[KSIZE][KSIZE];
    memset(w, 0, sizeof(w));

    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {

            pixel_t new_pixel = input[r][c];

            pixel_t col_pixels[KSIZE-1];
            for (int b = 0; b < KSIZE-1; b++)
                col_pixels[b] = linebuf[b][c];

            for (int b = 0; b < KSIZE-2; b++)
                linebuf[b][c] = col_pixels[b+1];
            linebuf[KSIZE-2][c] = new_pixel;

            for (int i = 0; i < KSIZE; i++)
                for (int j = 0; j < KSIZE-1; j++)
                    w[i][j] = w[i][j+1];

            for (int b = 0; b < KSIZE-1; b++)
                w[b][KSIZE-1] = col_pixels[b];
            w[KSIZE-1][KSIZE-1] = new_pixel;

            if (r < HALO || c < HALO) {
                output[r][c] = new_pixel;
            } else {
                unsigned int sum = 0;
                for (int i = 0; i < KSIZE; i++)
                    for (int j = 0; j < KSIZE; j++)
                        sum += (unsigned int)w[i][j] * KERNEL[i][j];
                output[r][c] = (pixel_t)(sum >> 8);
            }
        }
    }
}

// ── Test image arrays on heap (768 KB total — too large for stack) ────────────
static pixel_t input [HEIGHT][WIDTH];
static pixel_t sw_out[HEIGHT][WIDTH];
static pixel_t hw_out[HEIGHT][WIDTH];

int main() {

    // ── Generate test image ───────────────────────────────────────────────────
    for (int r = 0; r < HEIGHT; r++)
        for (int c = 0; c < WIDTH; c++) {
            input[r][c] = (c < WIDTH / 2) ? 40 : 200;
            if (r > 80 && r < 160 && c > 80 && c < 160)
                input[r][c] = 255;
        }

    // ── SW reference ─────────────────────────────────────────────────────────
    sw_streaming_reference(input, sw_out);

    // ── Pack image into 64-bit AXI words and push to stream ──────────────────
    hls::stream<axis_t> in_stream;
    hls::stream<axis_t> out_stream;

    for (int r = 0; r < HEIGHT; r++) {
        for (int cw = 0; cw < WIDTH / PIXELS_PER_WORD; cw++) {

            pixel_t px[PIXELS_PER_WORD];
            for (int p = 0; p < PIXELS_PER_WORD; p++)
                px[p] = input[r][cw * PIXELS_PER_WORD + p];

            word_t data = pack_pixels(px);
            bool last = (r == HEIGHT-1 && cw == WIDTH/PIXELS_PER_WORD - 1);
            in_stream.write(make_input_word(data, last));
        }
    }

    // ── Run HLS simulation ────────────────────────────────────────────────────
    gaussian_blur_dma(in_stream, out_stream);

    // ── Unpack output stream and compare ─────────────────────────────────────
    int errors       = 0;
    int tlast_errors = 0;

    for (int r = 0; r < HEIGHT; r++) {
        for (int cw = 0; cw < WIDTH / PIXELS_PER_WORD; cw++) {

            axis_t out_word = out_stream.read();
            word_t out_data = out_word.data;

            bool expected_last = (r == HEIGHT-1 && cw == WIDTH/PIXELS_PER_WORD - 1);
            if ((bool)out_word.last != expected_last) {
                std::cout << "TLAST error at word (" << r << "," << cw
                          << "): expected " << expected_last
                          << " got " << out_word.last << "\n";
                tlast_errors++;
            }

            for (int p = 0; p < PIXELS_PER_WORD; p++) {
                int c = cw * PIXELS_PER_WORD + p;
                hw_out[r][c] = unpack_pixel(out_data, p);

                if (hw_out[r][c] != sw_out[r][c]) {
                    if (errors < 20)
                        std::cout << "Mismatch at (" << r << "," << c
                                  << "): SW=" << (int)sw_out[r][c]
                                  << " HW=" << (int)hw_out[r][c] << "\n";
                    errors++;
                }
            }
        }
    }

    std::cout << "\nImage      : " << WIDTH << "x" << HEIGHT << "\n";
    std::cout << "Kernel     : " << KSIZE << "x" << KSIZE << "\n";
    std::cout << "AXI width  : " << AXI_WIDTH << "-bit ("
              << PIXELS_PER_WORD << " pixels/word)\n";
    std::cout << "AXI words  : " << WIDTH * HEIGHT / PIXELS_PER_WORD
              << " (vs " << WIDTH * HEIGHT << " in 8-bit version)\n";
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
