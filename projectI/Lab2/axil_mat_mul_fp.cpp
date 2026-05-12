#define N 16
#define SIZE (N * N)

void axil_mat_mul_fp(float A[SIZE], float B[SIZE], float C[SIZE])
{
#pragma HLS INTERFACE s_axilite port=return bundle=BUS1
#pragma HLS INTERFACE s_axilite port=A      bundle=BUS1
#pragma HLS INTERFACE s_axilite port=B      bundle=BUS1
#pragma HLS INTERFACE s_axilite port=C      bundle=BUS1

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {

            float acc = 0.0f;

            for (int k = 0; k < N; k++) {
                acc += A[i * N + k] * B[k * N + j];
            }

            C[i * N + j] = acc;
        }
    }
}