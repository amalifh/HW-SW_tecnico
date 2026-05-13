#include <hls_stream.h>
#include <ap_axi_sdata.h>

#define N 64
#define COL_BLOCK 32

typedef ap_axis<32, 0, 0, 0> axis_t;

union float_converter {
    float f;
    unsigned int u;
};

static float axis_to_float(axis_t word) {
    float_converter conv;
    conv.u = word.data;
    return conv.f;
}

static axis_t float_to_axis(float value, bool last) {
    float_converter conv;
    conv.f = value;

    axis_t word;
    word.data = conv.u;
    word.keep = -1;
    word.strb = -1;
    word.last = last;

    return word;
}

void axis_mat_mul_fp_DMA(
    hls::stream<axis_t> &in_stream,
    hls::stream<axis_t> &out_stream,
    int mode
) {
#pragma HLS INTERFACE axis port=in_stream
#pragma HLS INTERFACE axis port=out_stream
#pragma HLS INTERFACE s_axilite port=mode bundle=CTRL
#pragma HLS INTERFACE s_axilite port=return bundle=CTRL

    static float B_buf[N][COL_BLOCK];

#pragma HLS BIND_STORAGE variable=B_buf type=ram_2p impl=bram

    if (mode == 0) {
        // Load COL_BLOCK columns of B into internal BRAM
        for (int k = 0; k < N; k++) {
            for (int jj = 0; jj < COL_BLOCK; jj++) {
#pragma HLS PIPELINE II=1
                axis_t word = in_stream.read();
                B_buf[k][jj] = axis_to_float(word);
            }
        }
    } else {
        // Multiply one row of A with the stored B columns
        float acc[COL_BLOCK];

#pragma HLS ARRAY_PARTITION variable=acc complete

        for (int jj = 0; jj < COL_BLOCK; jj++) {
#pragma HLS UNROLL
            acc[jj] = 0.0f;
        }

        for (int k = 0; k < N; k++) {
#pragma HLS PIPELINE II=4
            axis_t word = in_stream.read();
            float a_val = axis_to_float(word);

            for (int jj = 0; jj < COL_BLOCK; jj++) {
#pragma HLS UNROLL
                acc[jj] += a_val * B_buf[k][jj];
            }
        }

        for (int jj = 0; jj < COL_BLOCK; jj++) {
#pragma HLS PIPELINE II=1
            bool last = (jj == COL_BLOCK - 1);
            out_stream.write(float_to_axis(acc[jj], last));
        }
    }
}
