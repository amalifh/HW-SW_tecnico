#include <stdio.h>
#include <stdlib.h>

#include "xparameters.h"
#include "xaxil_macc.h"
#include "xiltimer.h"

#define VEC_SIZE 10

static int v1[VEC_SIZE];
static int v2[VEC_SIZE];
static int vdotp1, vdotp2;

void init_vecs()
{
	int i;

	for (i=0; i<VEC_SIZE; i++) {
		v1[i] = ((rand() % 0xFF) - 0x80);
		v2[i] = ((rand() % 0xFF) - 0x80);
	}
}

void print_vec(int *x)
{
	int i;
	for (i=0; i<VEC_SIZE; i++) {
		printf("%5d ", x[i]);
	}
	printf("\n");
}

void SW_dot_product()
{
	int i;
	for (vdotp1=0, i=0; i<VEC_SIZE; i++) {
		vdotp1 += v1[i]*v2[i];
	}
}

void HW_SW_dot_product()
{
	int i;

	XAxil_macc Instance;
	XAxil_macc_Config *ConfigPtr;
	int status;

	ConfigPtr = XAxil_macc_LookupConfig(XPAR_AXIL_MACC_0_BASEADDR);
	if (ConfigPtr == NULL) {
		printf("LookupConfig failed\n");
		return;
	}

	status = XAxil_macc_CfgInitialize(&Instance, ConfigPtr);
	if (status != XST_SUCCESS) {
		printf("CfgInitialize failed\n");
		return;
	}

	// initialise (i=0): clear accumulator
	XAxil_macc_Set_a(&Instance, v1[0]);
	XAxil_macc_Set_b(&Instance, v2[0]);
	XAxil_macc_Set_instr(&Instance, 0);
	XAxil_macc_Start(&Instance);

	XAxil_macc_Set_instr(&Instance, 1);
	for (i=1; i<VEC_SIZE; i++) {
		XAxil_macc_Set_a(&Instance, v1[i]);
		XAxil_macc_Set_b(&Instance, v2[i]);
		XAxil_macc_Start(&Instance);
	}
	vdotp2 = XAxil_macc_Get_c(&Instance);
}

int main()
{
	XTime tStart, tEnd;

	init_vecs();
	print_vec(v1);
	print_vec(v2);

	// SW-only measurement
	XTime_GetTime(&tStart);
	SW_dot_product();
	XTime_GetTime(&tEnd);
	printf("   SW dot product: %d\n", vdotp1);
	printf("   SW execution: %llu clock cycles, %.2f us\n",
		2*(tEnd - tStart),
		1.0*(tEnd - tStart) * 1000000 / COUNTS_PER_SECOND);

	// HW/SW measurement
	XTime_GetTime(&tStart);
	HW_SW_dot_product();
	XTime_GetTime(&tEnd);
	printf("HW/SW dot product: %d\n", vdotp2);
	printf("HW/SW execution: %llu clock cycles, %.2f us\n",
		2*(tEnd - tStart),
		1.0*(tEnd - tStart) * 1000000 / COUNTS_PER_SECOND);

	printf("Results match: %s\n", (vdotp1 == vdotp2) ? "YES" : "NO");

	return 0;
}