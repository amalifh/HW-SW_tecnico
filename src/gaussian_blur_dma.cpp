#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>

#define WIDTH  8
#define HEIGHT 8
#define SIZE   (WIDTH * HEIGHT)

typedef ap_axis<32,0,0,0> axis_t;
typedef ap_uint<8> pixel_t;

static axis_t make_axis_word(pixel_t pix, bool last) {
    axis_t word;
    word.data = pix;
    word.keep = -1;
    word.strb = -1;
    word.last = last;
    return word;
}

void gaussian_blur_dma(hls::stream<axis_t>& in_stream,
                       hls::stream<axis_t>& out_stream) {
#pragma HLS INTERFACE axis port=in_stream
#pragma HLS INTERFACE axis port=out_stream
#pragma HLS INTERFACE ap_ctrl_none port=return

    pixel_t img[HEIGHT][WIDTH];
    pixel_t out[HEIGHT][WIDTH];

#pragma HLS ARRAY_PARTITION variable=img complete dim=2
#pragma HLS ARRAY_PARTITION variable=out complete dim=2

    // Read full image from AXI stream
    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
#pragma HLS PIPELINE II=1
            axis_t word = in_stream.read();
            img[r][c] = word.data.range(7,0);
        }
    }

    // Apply 3x3 Gaussian blur
    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
#pragma HLS PIPELINE II=1

            // Simple border handling: copy border pixels unchanged
            if (r == 0 || r == HEIGHT-1 || c == 0 || c == WIDTH-1) {
                out[r][c] = img[r][c];
            } else {
                ap_uint<12> sum = 0;

                ap_uint<12> p00 = img[r-1][c-1];
                ap_uint<12> p01 = img[r-1][c];
                ap_uint<12> p02 = img[r-1][c+1];

                ap_uint<12> p10 = img[r][c-1];
                ap_uint<12> p11 = img[r][c];
                ap_uint<12> p12 = img[r][c+1];

                ap_uint<12> p20 = img[r+1][c-1];
                ap_uint<12> p21 = img[r+1][c];
                ap_uint<12> p22 = img[r+1][c+1];

                sum += p00;
                sum += p01 << 1;
                sum += p02;

                sum += p10 << 1;
                sum += p11 << 2;
                sum += p12 << 1;

                sum += p20;
                sum += p21 << 1;
                sum += p22;

                out[r][c] = sum >> 4;
            }
        }
    }

    // Write blurred image to AXI stream
    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {
#pragma HLS PIPELINE II=1
            bool last = (r == HEIGHT-1 && c == WIDTH-1);
            out_stream.write(make_axis_word(out[r][c], last));
        }
    }
}