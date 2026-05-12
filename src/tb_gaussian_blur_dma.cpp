#include <iostream>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>

#define WIDTH  8
#define HEIGHT 8
#define SIZE   (WIDTH * HEIGHT)

typedef ap_axis<32,0,0,0> axis_t;
typedef unsigned char pixel_t;

void gaussian_blur_dma(hls::stream<axis_t>& in_stream,
                       hls::stream<axis_t>& out_stream);

static axis_t make_input_word(pixel_t pix, bool last) {
    axis_t word;
    word.data = pix;
    word.keep = -1;
    word.strb = -1;
    word.last = last;
    return word;
}

void sw_gaussian_blur(pixel_t input[HEIGHT][WIDTH],
                      pixel_t output[HEIGHT][WIDTH]) {
    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {

            if (r == 0 || r == HEIGHT-1 || c == 0 || c == WIDTH-1) {
                output[r][c] = input[r][c];
            } else {
                int sum = 0;

                sum += input[r-1][c-1];
                sum += input[r-1][c] * 2;
                sum += input[r-1][c+1];

                sum += input[r][c-1] * 2;
                sum += input[r][c] * 4;
                sum += input[r][c+1] * 2;

                sum += input[r+1][c-1];
                sum += input[r+1][c] * 2;
                sum += input[r+1][c+1];

                output[r][c] = sum / 16;
            }
        }
    }
}

void print_image(const char* name, pixel_t img[HEIGHT][WIDTH]) {
    std::cout << "\n" << name << ":\n";
    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
            std::cout << (int)img[r][c] << "\t";
        }
        std::cout << "\n";
    }
}

int main() {
    hls::stream<axis_t> in_stream;
    hls::stream<axis_t> out_stream;

    pixel_t input[HEIGHT][WIDTH] = {
        {  0,   0,   0,   0, 255, 255, 255, 255 },
        {  0,   0,   0,   0, 255, 255, 255, 255 },
        {  0,   0,   0,   0, 255, 255, 255, 255 },
        {  0,   0,   0,   0, 255, 255, 255, 255 },
        { 50,  50,  50,  50, 200, 200, 200, 200 },
        { 50,  50,  50,  50, 200, 200, 200, 200 },
        { 50,  50,  50,  50, 200, 200, 200, 200 },
        { 50,  50,  50,  50, 200, 200, 200, 200 }
    };

    pixel_t sw_out[HEIGHT][WIDTH];
    pixel_t hw_out[HEIGHT][WIDTH];

    sw_gaussian_blur(input, sw_out);

    // Send image into input stream
    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
            bool last = (r == HEIGHT-1 && c == WIDTH-1);
            in_stream.write(make_input_word(input[r][c], last));
        }
    }

    // Run HLS function
    gaussian_blur_dma(in_stream, out_stream);

    // Read output stream
    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
            axis_t word = out_stream.read();
            hw_out[r][c] = word.data.range(7,0);

            if (r == HEIGHT-1 && c == WIDTH-1 && word.last != 1) {
                std::cout << "ERROR: TLAST missing on final output word\n";
                return 1;
            }
        }
    }

    print_image("Input image", input);
    print_image("Software Gaussian output", sw_out);
    print_image("Hardware Gaussian output", hw_out);

    int errors = 0;

    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
            if (sw_out[r][c] != hw_out[r][c]) {
                std::cout << "Mismatch at (" << r << "," << c << "): SW="
                          << (int)sw_out[r][c] << " HW="
                          << (int)hw_out[r][c] << "\n";
                errors++;
            }
        }
    }

    if (errors == 0) {
        std::cout << "\nTEST PASSED\n";
    } else {
        std::cout << "\nTEST FAILED with " << errors << " errors\n";
    }

    return errors;
}