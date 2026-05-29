#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>

#define WIDTH  1280
#define HEIGHT 720
#define SIZE   (WIDTH * HEIGHT)

// Changed from 32-bit AXI stream to 8-bit AXI stream
typedef ap_axis<8,0,0,0> axis_t;

typedef ap_uint<8> pixel_t;

static axis_t make_axis_word(pixel_t pix, bool last) {
    axis_t word;

    word.data = pix;

    // For 8-bit stream, only 1 byte is valid
    word.keep = 1;
    word.strb = 1;

    word.last = last;

    return word;
}

void gaussian_blur_dma(hls::stream<axis_t>& in_stream,
                       hls::stream<axis_t>& out_stream) {

#pragma HLS INTERFACE axis port=in_stream
#pragma HLS INTERFACE axis port=out_stream
#pragma HLS INTERFACE s_axilite port=return bundle=CTRL

    static pixel_t linebuf0[WIDTH];
    static pixel_t linebuf1[WIDTH];

#pragma HLS BIND_STORAGE variable=linebuf0 type=ram_2p impl=bram
#pragma HLS BIND_STORAGE variable=linebuf1 type=ram_2p impl=bram

    pixel_t w00 = 0, w01 = 0, w02 = 0;
    pixel_t w10 = 0, w11 = 0, w12 = 0;
    pixel_t w20 = 0, w21 = 0, w22 = 0;

#pragma HLS ARRAY_PARTITION variable=w00 complete
#pragma HLS ARRAY_PARTITION variable=w01 complete
#pragma HLS ARRAY_PARTITION variable=w02 complete
#pragma HLS ARRAY_PARTITION variable=w10 complete
#pragma HLS ARRAY_PARTITION variable=w11 complete
#pragma HLS ARRAY_PARTITION variable=w12 complete
#pragma HLS ARRAY_PARTITION variable=w20 complete
#pragma HLS ARRAY_PARTITION variable=w21 complete
#pragma HLS ARRAY_PARTITION variable=w22 complete

    for (int r = 0; r < HEIGHT; r++) {
        for (int c = 0; c < WIDTH; c++) {

#pragma HLS PIPELINE II=1

            axis_t in_word = in_stream.read();

            // Entire AXI word is now just 8 bits
            pixel_t new_pixel = in_word.data;

            pixel_t top_pixel = linebuf0[c];
            pixel_t mid_pixel = linebuf1[c];

            linebuf0[c] = mid_pixel;
            linebuf1[c] = new_pixel;

            // Shift window left
            w00 = w01; w01 = w02; w02 = top_pixel;
            w10 = w11; w11 = w12; w12 = mid_pixel;
            w20 = w21; w21 = w22; w22 = new_pixel;

            pixel_t out_pixel;

            if (r < 2 || c < 2) {

                out_pixel = new_pixel;

            } else {

                ap_uint<12> p00 = w00;
                ap_uint<12> p01 = w01;
                ap_uint<12> p02 = w02;

                ap_uint<12> p10 = w10;
                ap_uint<12> p11 = w11;
                ap_uint<12> p12 = w12;

                ap_uint<12> p20 = w20;
                ap_uint<12> p21 = w21;
                ap_uint<12> p22 = w22;

                ap_uint<12> sum = 0;

                sum += p00;
                sum += p01 << 1;
                sum += p02;

                sum += p10 << 1;
                sum += p11 << 2;
                sum += p12 << 1;

                sum += p20;
                sum += p21 << 1;
                sum += p22;

                out_pixel = sum >> 4;
            }

            bool last = (r == HEIGHT - 1 && c == WIDTH - 1);

            out_stream.write(make_axis_word(out_pixel, last));
        }
    }
}