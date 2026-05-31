// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.1 (64-bit)
// Tool Version Limit: 2025.05
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef XGAUSSIAN_BLUR_V2_H
#define XGAUSSIAN_BLUR_V2_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "xgaussian_blur_v2_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
#ifdef SDT
    char *Name;
#else
    u16 DeviceId;
#endif
    u64 Ctrl_BaseAddress;
} XGaussian_blur_v2_Config;
#endif

typedef struct {
    u64 Ctrl_BaseAddress;
    u32 IsReady;
} XGaussian_blur_v2;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XGaussian_blur_v2_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XGaussian_blur_v2_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XGaussian_blur_v2_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XGaussian_blur_v2_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
#ifdef SDT
int XGaussian_blur_v2_Initialize(XGaussian_blur_v2 *InstancePtr, UINTPTR BaseAddress);
XGaussian_blur_v2_Config* XGaussian_blur_v2_LookupConfig(UINTPTR BaseAddress);
#else
int XGaussian_blur_v2_Initialize(XGaussian_blur_v2 *InstancePtr, u16 DeviceId);
XGaussian_blur_v2_Config* XGaussian_blur_v2_LookupConfig(u16 DeviceId);
#endif
int XGaussian_blur_v2_CfgInitialize(XGaussian_blur_v2 *InstancePtr, XGaussian_blur_v2_Config *ConfigPtr);
#else
int XGaussian_blur_v2_Initialize(XGaussian_blur_v2 *InstancePtr, const char* InstanceName);
int XGaussian_blur_v2_Release(XGaussian_blur_v2 *InstancePtr);
#endif

void XGaussian_blur_v2_Start(XGaussian_blur_v2 *InstancePtr);
u32 XGaussian_blur_v2_IsDone(XGaussian_blur_v2 *InstancePtr);
u32 XGaussian_blur_v2_IsIdle(XGaussian_blur_v2 *InstancePtr);
u32 XGaussian_blur_v2_IsReady(XGaussian_blur_v2 *InstancePtr);
void XGaussian_blur_v2_EnableAutoRestart(XGaussian_blur_v2 *InstancePtr);
void XGaussian_blur_v2_DisableAutoRestart(XGaussian_blur_v2 *InstancePtr);


void XGaussian_blur_v2_InterruptGlobalEnable(XGaussian_blur_v2 *InstancePtr);
void XGaussian_blur_v2_InterruptGlobalDisable(XGaussian_blur_v2 *InstancePtr);
void XGaussian_blur_v2_InterruptEnable(XGaussian_blur_v2 *InstancePtr, u32 Mask);
void XGaussian_blur_v2_InterruptDisable(XGaussian_blur_v2 *InstancePtr, u32 Mask);
void XGaussian_blur_v2_InterruptClear(XGaussian_blur_v2 *InstancePtr, u32 Mask);
u32 XGaussian_blur_v2_InterruptGetEnabled(XGaussian_blur_v2 *InstancePtr);
u32 XGaussian_blur_v2_InterruptGetStatus(XGaussian_blur_v2 *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
