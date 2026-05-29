#include <iostream>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>

#define WIDTH  1280
#define HEIGHT 720
#define SIZE   (WIDTH * HEIGHT)

// Changed from 32-bit AXI stream to 8-bit AXI stream
typedef ap_axis<8,0,0,0> axis_t;
typedef ap_uint<8> pixel_t;

void gaussian_blur_dma(hls::stream<axis_t>& in_stream,
                       hls::stream<axis_t>& out_stream);

static axis_t make_input_word(pixel_t pix, bool last) {
    axis_t word;

    word.data = pix;

    // 8-bit AXI stream = 1 valid byte
    word.keep = 1;
    word.strb = 1;

    word.last = last;

    return word;
}

void sw_streaming_reference(pixel_t input[HEIGHT][WIDTH],
                            pixel_t output[HEIGHT][WIDTH]) {
    pixel_t linebuf0[WIDTH] = {0};
    pixel_t linebuf1[WIDTH] = {0};

    pixel_t w00 = 0, w01 = 0, w02 = 0;
    pixel_t w10 = 0, w11 = 0, w12 = 0;
    pixel_t w20 = 0, w21 = 0, w22 = 0;

    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {

            pixel_t new_pixel = input[r][c];

            pixel_t top_pixel = linebuf0[c];
            pixel_t mid_pixel = linebuf1[c];

            linebuf0[c] = mid_pixel;
            linebuf1[c] = new_pixel;

            w00 = w01; w01 = w02; w02 = top_pixel;
            w10 = w11; w11 = w12; w12 = mid_pixel;
            w20 = w21; w21 = w22; w22 = new_pixel;

            pixel_t out_pixel;

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

                out_pixel = (pixel_t)(sum >> 4);
            }

            output[r][c] = out_pixel;
        }
    }
}

int main() {
    hls::stream<axis_t> in_stream;
    hls::stream<axis_t> out_stream;

    pixel_t input[HEIGHT][WIDTH];
    pixel_t sw_out[HEIGHT][WIDTH];
    pixel_t hw_out[HEIGHT][WIDTH];

    // Generate simple deterministic test image
    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
            if (c < WIDTH / 2)
                input[r][c] = 40;
            else
                input[r][c] = 200;

            // Add some variation
            if ((r > 80 && r < 160) && (c > 80 && c < 160))
                input[r][c] = 255;
        }
    }

    sw_streaming_reference(input, sw_out);

    // Send input image to HLS stream
    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
            bool last = (r == HEIGHT - 1 && c == WIDTH - 1);
            in_stream.write(make_input_word(input[r][c], last));
        }
    }

    gaussian_blur_dma(in_stream, out_stream);

    int errors = 0;
    int tlast_errors = 0;

    // Read output stream
    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
            axis_t word = out_stream.read();

            // Entire AXI word is now only 8 bits
            hw_out[r][c] = word.data;

            bool expected_last = (r == HEIGHT - 1 && c == WIDTH - 1);

            if ((bool)word.last != expected_last) {
                std::cout << "TLAST error at (" << r << "," << c << "): expected "
                          << expected_last << ", got " << word.last << "\n";
                tlast_errors++;
            }

            if (hw_out[r][c] != sw_out[r][c]) {
                if (errors < 20) {
                    std::cout << "Mismatch at (" << r << "," << c << "): SW="
                              << (int)sw_out[r][c]
                              << " HW=" << (int)hw_out[r][c] << "\n";
                }
                errors++;
            }
        }
    }

    std::cout << "\nTest image size: " << WIDTH << "x" << HEIGHT << "\n";
    std::cout << "Pixel mismatches: " << errors << "\n";
    std::cout << "TLAST errors: " << tlast_errors << "\n";

    if (errors == 0 && tlast_errors == 0) {
        std::cout << "TEST PASSED\n";
        return 0;
    } else {
        std::cout << "TEST FAILED\n";
        return 1;
    }
}