#include <iostream>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>

#define WIDTH  512
#define HEIGHT 512
#define SIZE   (WIDTH * HEIGHT)
#define PPC    8
#define COLS   (WIDTH / PPC)

#define KSIZE  5
#define HALO   (KSIZE - 1)
#define NBUF   (KSIZE - 1)

typedef ap_axis<64, 0, 0, 0> axis_wide_t;
typedef ap_uint<8>  pixel_t;

void gaussian_blur_v2(hls::stream<axis_wide_t>& in_stream,
                      hls::stream<axis_wide_t>& out_stream);

static axis_wide_t make_input_word(pixel_t p[8], bool last)
{
    axis_wide_t word;
    ap_uint<64> data = 0;

    for (int i = 0; i < 8; i++) {
        data.range(i * 8 + 7, i * 8) = p[i];
    }

    word.data = data;
    word.keep = 0xFF;
    word.strb = 0xFF;
    word.last = last;
    return word;
}

static pixel_t extract_pixel(ap_uint<64> word, int idx)
{
    return word.range(idx * 8 + 7, idx * 8);
}

static const ap_uint<8> KERNEL[KSIZE][KSIZE] = {
    { 1,  4,  6,  4,  1},
    { 4, 16, 24, 16,  4},
    { 6, 24, 36, 24,  6},
    { 4, 16, 24, 16,  4},
    { 1,  4,  6,  4,  1}
};

void sw_streaming_reference(pixel_t input[HEIGHT][WIDTH],
                            pixel_t output[HEIGHT][WIDTH])
{
    pixel_t linebuf[NBUF][WIDTH];
    pixel_t w[KSIZE][KSIZE];

    for (int b = 0; b < NBUF; b++)
        for (int c = 0; c < WIDTH; c++)
            linebuf[b][c] = 0;

    for (int i = 0; i < KSIZE; i++)
        for (int j = 0; j < KSIZE; j++)
            w[i][j] = 0;

    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {

            pixel_t new_pixel = input[r][c];

            pixel_t col_pixels[NBUF];
            for (int b = 0; b < NBUF; b++)
                col_pixels[b] = linebuf[b][c];

            for (int b = 0; b < NBUF - 1; b++)
                linebuf[b][c] = col_pixels[b + 1];
            linebuf[NBUF - 1][c] = new_pixel;

            for (int i = 0; i < KSIZE; i++)
                for (int j = 0; j < KSIZE - 1; j++)
                    w[i][j] = w[i][j + 1];

            for (int b = 0; b < NBUF; b++)
                w[b][KSIZE - 1] = col_pixels[b];
            w[KSIZE - 1][KSIZE - 1] = new_pixel;

            pixel_t out_pixel;

            if (r < HALO || c < HALO) {
                out_pixel = new_pixel;
            } else {
                ap_uint<17> sum = 0;

                for (int i = 0; i < KSIZE; i++)
                    for (int j = 0; j < KSIZE; j++)
                        sum += (ap_uint<17>)w[i][j] * KERNEL[i][j];

                out_pixel = (pixel_t)(sum >> 8);
            }

            output[r][c] = out_pixel;
        }
    }
}

int main()
{
    std::cout << "\nGaussian Blur 5x5 true 8-PPC HLS testbench\n";

    hls::stream<axis_wide_t> in_stream;
    hls::stream<axis_wide_t> out_stream;

    static pixel_t input[HEIGHT][WIDTH];
    static pixel_t sw_out[HEIGHT][WIDTH];
    static pixel_t hw_out[HEIGHT][WIDTH];

    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
            int block_r = r / 64;
            int block_c = c / 64;

            if ((block_r + block_c) % 2 == 0)
                input[r][c] = 50 + (r % 64);
            else
                input[r][c] = 200 - (c % 64);

            if ((r > 200 && r < 312) && (c > 200 && c < 312))
                input[r][c] = 255;
        }
    }

    sw_streaming_reference(input, sw_out);

    int write_count = 0;
    for (int r = 0; r < HEIGHT; r++) {
        for (int cb = 0; cb < COLS; cb++) {
            pixel_t p[8];

            for (int i = 0; i < 8; i++)
                p[i] = input[r][cb * PPC + i];

            bool last = (r == HEIGHT - 1 && cb == COLS - 1);
            in_stream.write(make_input_word(p, last));
            write_count++;
        }
    }

    gaussian_blur_v2(in_stream, out_stream);

    int pixel_errors = 0;
    int tlast_errors = 0;
    int read_count = 0;

    for (int r = 0; r < HEIGHT; r++) {
        for (int cb = 0; cb < COLS; cb++) {
            axis_wide_t out_word = out_stream.read();
            read_count++;

            for (int i = 0; i < 8; i++) {
                int c = cb * PPC + i;
                pixel_t hw_pix = extract_pixel(out_word.data, i);
                hw_out[r][c] = hw_pix;

                pixel_t sw_pix = sw_out[r][c];

                if (hw_pix != sw_pix) {
                    if (pixel_errors < 30) {
                        std::cout << "Pixel mismatch at (" << r << "," << c
                                  << "): SW=" << (int)sw_pix
                                  << " HW=" << (int)hw_pix << "\n";
                    }
                    pixel_errors++;
                }
            }

            bool expected_last = (r == HEIGHT - 1 && cb == COLS - 1);
            if ((bool)out_word.last != expected_last) {
                if (tlast_errors < 10) {
                    std::cout << "TLAST error at r=" << r << " cb=" << cb
                              << ": expected " << expected_last
                              << " got " << (bool)out_word.last << "\n";
                }
                tlast_errors++;
            }
        }
    }

    std::cout << "\nImage size:              " << WIDTH << "x" << HEIGHT << "\n";
    std::cout << "Kernel:                  5x5\n";
    std::cout << "PPC:                     " << PPC << "\n";
    std::cout << "Input transactions:      " << write_count << "\n";
    std::cout << "Output transactions:     " << read_count << "\n";
    std::cout << "Pixel mismatches:        " << pixel_errors << "\n";
    std::cout << "TLAST errors:            " << tlast_errors << "\n";

    if (pixel_errors == 0 && tlast_errors == 0) {
        std::cout << "TEST PASSED\n";
        return 0;
    } else {
        std::cout << "TEST FAILED\n";
        return 1;
    }
}
