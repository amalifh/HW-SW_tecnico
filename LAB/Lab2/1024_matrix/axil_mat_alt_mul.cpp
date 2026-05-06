#define N_max 1024
#define B_BLOCK 4

void axil_mat_mul_cols(
    int N,
    float A_row[N_max],
    float B_cols[B_BLOCK][N_max],
    float C_out[B_BLOCK]
)
{
#pragma HLS INTERFACE s_axilite port=return bundle=BUS1
#pragma HLS INTERFACE s_axilite port=A_row bundle=BUS1
#pragma HLS INTERFACE s_axilite port=B_cols bundle=BUS1
#pragma HLS INTERFACE s_axilite port=C_out bundle=BUS1

    float acc[B_BLOCK];

#pragma HLS ARRAY_PARTITION variable=acc complete

    for (int j = 0; j < B_BLOCK; j++) {
        acc[j] = 0.0f;
    }

    for (int k = 0; k < N; k++) {
#pragma HLS PIPELINE
        for (int j = 0; j < B_BLOCK; j++) {
#pragma HLS UNROLL
            acc[j] += A_row[k] * B_cols[j][k];
        }
    }

    for (int j = 0; j < B_BLOCK; j++) {
        C_out[j] = acc[j];
    }
}