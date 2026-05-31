#include <iostream>
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
// HLS function declaration
// ─────────────────────────────────────────────────────────────────────────────
void gaussian_blur_v2(hls::stream<axis_wide_t>& in_stream,
                            hls::stream<axis_wide_t>& out_stream);

// ─────────────────────────────────────────────────────────────────────────────
// Helper: Create input word (pack 8 pixels into 64-bit word, LSB = pixel 0)
// ─────────────────────────────────────────────────────────────────────────────
static axis_wide_t make_input_word(pixel_t pix0, pixel_t pix1, pixel_t pix2,
                                    pixel_t pix3, pixel_t pix4, pixel_t pix5,
                                    pixel_t pix6, pixel_t pix7, bool last) {
    axis_wide_t word;

    ap_uint<64> data = 0;
    data |= ((ap_uint<64>)pix0) << 0;
    data |= ((ap_uint<64>)pix1) << 8;
    data |= ((ap_uint<64>)pix2) << 16;
    data |= ((ap_uint<64>)pix3) << 24;
    data |= ((ap_uint<64>)pix4) << 32;
    data |= ((ap_uint<64>)pix5) << 40;
    data |= ((ap_uint<64>)pix6) << 48;
    data |= ((ap_uint<64>)pix7) << 56;

    word.data = data;
    word.keep = 0xFF;  // All 8 bytes valid
    word.strb = 0xFF;
    word.last = last;

    return word;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: Extract pixel i from 64-bit word
// ─────────────────────────────────────────────────────────────────────────────
static pixel_t extract_pixel(ap_uint<64> word, int idx) {
    return (pixel_t)(word >> (idx * 8));
}

// ─────────────────────────────────────────────────────────────────────────────
// 5x5 Gaussian kernel: outer product of [1,4,6,4,1]
// ─────────────────────────────────────────────────────────────────────────────
static const ap_uint<8> KERNEL[KSIZE][KSIZE] = {
    { 1,  4,  6,  4,  1},
    { 4, 16, 24, 16,  4},
    { 6, 24, 36, 24,  6},
    { 4, 16, 24, 16,  4},
    { 1,  4,  6,  4,  1}
};

// ─────────────────────────────────────────────────────────────────────────────
// Software reference: 5x5 streaming Gaussian blur
// ─────────────────────────────────────────────────────────────────────────────
void sw_streaming_reference(pixel_t input[HEIGHT][WIDTH],
                            pixel_t output[HEIGHT][WIDTH]) {
    // 4 line buffers (oldest to most recent buffered row)
    pixel_t linebuf[NBUF][WIDTH];
    int b, r, c, i, j;

    // Initialize line buffers
    for (b = 0; b < NBUF; b++) {
        for (c = 0; c < WIDTH; c++) {
            linebuf[b][c] = 0;
        }
    }

    // 5x5 sliding window: w[row][col], row 0 = oldest, row 4 = current
    pixel_t w[KSIZE][KSIZE];
    for (i = 0; i < KSIZE; i++) {
        for (j = 0; j < KSIZE; j++) {
            w[i][j] = 0;
        }
    }

    // Process all pixels
    for (r = 0; r < HEIGHT; r++) {
        for (c = 0; c < WIDTH; c++) {

            pixel_t new_pixel = input[r][c];

            // Read column c from all 4 line buffers
            pixel_t col_pixels[NBUF];
            for (b = 0; b < NBUF; b++) {
                col_pixels[b] = linebuf[b][c];
            }

            // Shift line buffers: oldest drops out, newest row enters at end
            for (b = 0; b < NBUF - 1; b++) {
                linebuf[b][c] = col_pixels[b + 1];
            }
            linebuf[NBUF - 1][c] = new_pixel;

            // Shift window left by one column (drop leftmost, make room on right)
            for (i = 0; i < KSIZE; i++) {
                for (j = 0; j < KSIZE - 1; j++) {
                    w[i][j] = w[i][j + 1];
                }
            }

            // Insert new rightmost column into window
            // col_pixels[0] = row r-4, ..., col_pixels[3] = row r-1
            // new_pixel = row r
            for (b = 0; b < NBUF; b++) {
                w[b][KSIZE - 1] = col_pixels[b];
            }
            w[KSIZE - 1][KSIZE - 1] = new_pixel;

            pixel_t out_pixel;

            if (r < HALO || c < HALO) {
                // Border: not enough history for a full window, pass through
                out_pixel = new_pixel;
            } else {
                ap_uint<17> sum = 0;

                for (i = 0; i < KSIZE; i++) {
                    for (j = 0; j < KSIZE; j++) {
                        sum += (ap_uint<17>)w[i][j] * KERNEL[i][j];
                    }
                }

                // Normalise: sum of kernel = 256 = 2^8
                out_pixel = (pixel_t)(sum >> 8);
            }

            output[r][c] = out_pixel;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Main testbench
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  Gaussian Blur DMA 5x5 - HLS Testbench                     ║\n";
    std::cout << "║  Config: 512x512 image, 5x5 kernel, 64-bit AXI Stream      ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    hls::stream<axis_wide_t> in_stream;
    hls::stream<axis_wide_t> out_stream;

    pixel_t input[HEIGHT][WIDTH];
    pixel_t sw_out[HEIGHT][WIDTH];
    pixel_t hw_out[HEIGHT][WIDTH];

    // ─────────────────────────────────────────────────────────────────────────
    // Generate test image: checkerboard pattern with gradient
    // ─────────────────────────────────────────────────────────────────────────
    std::cout << "Generating test image (512x512)...\n";

    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
            // Checkerboard: 64x64 blocks
            int block_r = r / 64;
            int block_c = c / 64;

            if ((block_r + block_c) % 2 == 0) {
                input[r][c] = 50 + (r % 64);  // Gradient in one direction
            } else {
                input[r][c] = 200 - (c % 64);  // Gradient in other direction
            }

            // Add a bright square in the center
            if ((r > 200 && r < 312) && (c > 200 && c < 312)) {
                input[r][c] = 255;
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Run software reference
    // ─────────────────────────────────────────────────────────────────────────
    std::cout << "Running software reference (5x5 kernel)...\n";
    sw_streaming_reference(input, sw_out);

    // ─────────────────────────────────────────────────────────────────────────
    // Send input image to HLS stream (64-bit words = 8 pixels per transaction)
    // ─────────────────────────────────────────────────────────────────────────
    std::cout << "Feeding input to HLS accelerator (64-bit stream)...\n";

    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c += 8) {
            pixel_t p[8];
            for (int i = 0; i < 8; i++) {
                p[i] = input[r][c + i];
            }

            bool is_last = (r == HEIGHT - 1 && c == WIDTH - 8);
            in_stream.write(make_input_word(p[0], p[1], p[2], p[3],
                                             p[4], p[5], p[6], p[7],
                                             is_last));
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Call HLS function
    // ─────────────────────────────────────────────────────────────────────────
    std::cout << "Running HLS Gaussian blur accelerator...\n";
    gaussian_blur_v2(in_stream, out_stream);

    // ─────────────────────────────────────────────────────────────────────────
    // Read output stream and compare with software reference
    // ─────────────────────────────────────────────────────────────────────────
    std::cout << "Reading output from HLS accelerator (64-bit stream)...\n";

    int pixel_errors = 0;
    int tlast_errors = 0;
    int read_count = 0;

    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c += 8) {
            axis_wide_t out_word = out_stream.read();
            read_count++;

            // Extract 8 pixels from output word
            for (int i = 0; i < 8; i++) {
                pixel_t hw_pix = extract_pixel(out_word.data, i);
                hw_out[r][c + i] = hw_pix;

                pixel_t sw_pix = sw_out[r][c + i];

                if (hw_pix != sw_pix) {
                    if (pixel_errors < 20) {
                        std::cout << "Pixel mismatch at (" << r << "," << (c + i)
                                  << "): SW=" << (int)sw_pix
                                  << " HW=" << (int)hw_pix << "\n";
                    }
                    pixel_errors++;
                }
            }

            bool expected_last = (r == HEIGHT - 1 && c == WIDTH - 8);
            if ((bool)out_word.last != expected_last) {
                if (tlast_errors < 5) {
                    std::cout << "TLAST error at (" << r << "," << c << "): expected "
                              << expected_last << ", got " << (bool)out_word.last << "\n";
                }
                tlast_errors++;
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Print summary
    // ─────────────────────────────────────────────────────────────────────────
    std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                      TEST SUMMARY                          ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    std::cout << "Image Size:              " << WIDTH << "x" << HEIGHT << " pixels\n";
    std::cout << "Kernel Size:             " << KSIZE << "x" << KSIZE << "\n";
    std::cout << "Total Pixels:            " << SIZE << "\n";
    std::cout << "AXI Stream Transactions: " << read_count << " (8 pixels each)\n";
    std::cout << "Border (HALO):           " << HALO << " pixels\n\n";

    std::cout << "Pixel Mismatches:        " << pixel_errors << "\n";
    std::cout << "TLAST Errors:            " << tlast_errors << "\n\n";

    if (pixel_errors == 0 && tlast_errors == 0) {
        std::cout << "✓ TEST PASSED\n";
        return 0;
    } else {
        std::cout << "✗ TEST FAILED\n";
        return 1;
    }
}
