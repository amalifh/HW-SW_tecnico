#define N 1024
#define COL_BLOCK 2

void axil_opt_mat_mul_fp(float B_cols[N * COL_BLOCK],
                     float A_row[N],
                     float C_vals[COL_BLOCK],
                     int mode)
{
#pragma HLS INTERFACE s_axilite port=return  bundle=BUS1
#pragma HLS INTERFACE s_axilite port=B_cols  bundle=BUS1
#pragma HLS INTERFACE s_axilite port=A_row   bundle=BUS1
#pragma HLS INTERFACE s_axilite port=C_vals  bundle=BUS1
#pragma HLS INTERFACE s_axilite port=mode    bundle=BUS1

    static float B_buf[N][COL_BLOCK];

#pragma HLS BIND_STORAGE variable=B_buf type=ram_2p impl=bram

    if (mode == 0) {
        // Load COL_BLOCK columns of B into internal BRAM
        for (int k = 0; k < N; k++) {
            for (int jj = 0; jj < COL_BLOCK; jj++) {
                B_buf[k][jj] = B_cols[k * COL_BLOCK + jj];
            }
        }
    } else {
        // Multiply one row of A by the stored B columns
        float acc[COL_BLOCK];

#pragma HLS ARRAY_PARTITION variable=acc complete

        for (int jj = 0; jj < COL_BLOCK; jj++) {
#pragma HLS UNROLL
            acc[jj] = 0.0f;
        }

        for (int k = 0; k < N; k++) {
            float a_val = A_row[k];

            for (int jj = 0; jj < COL_BLOCK; jj++) {
#pragma HLS UNROLL
                acc[jj] += a_val * B_buf[k][jj];
            }
        }

        for (int jj = 0; jj < COL_BLOCK; jj++) {
#pragma HLS UNROLL
            C_vals[jj] = acc[jj];
        }
    }
}