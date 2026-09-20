/*
 * MvCameraControl.h —— 海康机器人 MVS SDK 的「编译验证替身」头文件（Fake/Stub）
 *
 * 用途：在没有安装真实 MVS SDK 的机器上完成 RP-26BeamLock 的编译验证，
 *       由 CMake 选项 USE_FAKE_MVS 启用（见 Detect/MVSCamera/CMakeLists.txt）。
 * 范围：只声明 Detect/MVSCamera 实际用到的类型、常量与函数，不做完整 SDK 复刻。
 * 注意：
 *   - 本文件不是海康官方头文件，不可用于生产、不可用于控制真实相机；
 *   - 真实部署请安装官方 MVS SDK，并通过 -DMVS_ROOT=<安装路径> 指向它；
 *   - 结构体布局按官方 SDK 的字段顺序与类型书写，但仅供编译/链接使用；
 *   - 头文件与实现必须成对使用：MV_CC_IsStubSDK() 用于让「假头文件 + 真库」
 *     这类混搭在链接期立刻报错，而不是运行时踩内存。
 */
#ifndef FAKE_MV_CAMERA_CONTROL_H
#define FAKE_MV_CAMERA_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- 版本与句柄 */

#define MV_SDK_VERSION "0.0.0-stub"

/* 替身 SDK 标识：仅本替身实现提供，链接期即可发现头文件与库不匹配 */
int MV_CC_IsStubSDK(void);

typedef void *MV_CC_HANDLE; /* 真实 SDK 中 void* 即为设备句柄 */

/* ------------------------------------------------------------ 返回码与常量 */

#define MV_OK 0x00000000 /* 成功，无错误 */

#define MV_TRIGGER_MODE_OFF 0 /* 触发模式：关 */

/* 传输层类型（可作为位标志与 MV_CC_EnumDevices 的入参） */
#define MV_GIGE_DEVICE 0x00000001
#define MV_USB_DEVICE 0x00000004

/* 设备访问权限 */
#define MV_ACCESS_Exclusive 1
#define MV_ACCESS_ExclusiveWithSwitch 2
#define MV_ACCESS_Control 3
#define MV_ACCESS_ControlWithSwitch 4
#define MV_ACCESS_ControlSwitchEnable 5
#define MV_ACCESS_ControlSwitchEnableWithKey 6
#define MV_ACCESS_Monitor 7

/* 像素格式：仅保留 MVSCamera 用到的 BayerRG8 */
#define PixelType_Gvsp_BayerRG8 0x01080009

/* ------------------------------------------------------------ 设备信息结构 */

typedef struct _MV_GIGE_DEVICE_INFO_
{
    unsigned int nIpCfgOption;
    unsigned int nIpCfgCurrent;
    unsigned int nCurrentIp;
    unsigned int nCurrentSubNetMask;
    unsigned int nDefultGateWay;
    unsigned char chManufacturerName[32];
    unsigned char chModelName[32];
    unsigned char chDeviceVersion[32];
    unsigned char chManufacturerSpecificInfo[48];
    unsigned char chSerialNumber[16];
    unsigned char chUserDefinedName[16];
    unsigned int nNetExport;
    unsigned int nReserved[4];
} MV_GIGE_DEVICE_INFO;

typedef struct _MV_USB3_DEVICE_INFO_
{
    unsigned int CrtlInEndPoint;
    unsigned int CrtlOutEndPoint;
    unsigned int StreamEndPoint;
    unsigned int EventEndPoint;
    unsigned short idVendor;
    unsigned short idProduct;
    unsigned int nDeviceNumber;
    unsigned char chDeviceGUID[64];
    unsigned char chVendorName[64];
    unsigned char chModelName[64];
    unsigned char chFamilyName[64];
    unsigned char chDeviceVersion[64];
    unsigned char chManufacturerName[64];
    unsigned char chSerialNumber[64];
    unsigned char chUserDefinedName[64];
    unsigned int nDriverType;
    unsigned int nDeviceVersion;
    unsigned int nReserved[4];
} MV_USB3_DEVICE_INFO;

typedef struct _MV_CC_DEVICE_INFO_
{
    unsigned short nMajorVer;
    unsigned short nMinorVer;
    unsigned int nTLayerType;
    unsigned int nDevTypeInfo;
    unsigned int nReserved[2];
    union
    {
        MV_GIGE_DEVICE_INFO stGigEInfo;
        MV_USB3_DEVICE_INFO stUsb3VInfo;
    } SpecialInfo;
} MV_CC_DEVICE_INFO;

#define MV_MAX_DEVICE_NUM 256

typedef struct _MV_CC_DEVICE_INFO_LIST_
{
    unsigned int nDeviceNum;
    MV_CC_DEVICE_INFO *pDeviceInfo[MV_MAX_DEVICE_NUM];
} MV_CC_DEVICE_INFO_LIST;

/* ------------------------------------------------------------ 取流/图像结构 */

typedef struct _MV_IMAGE_BASIC_INFO_
{
    unsigned short nWidthValue;
    unsigned short nWidthMin;
    unsigned int nWidthMax;
    unsigned int nWidthInc;
    unsigned int nHeightValue;
    unsigned int nHeightMin;
    unsigned int nHeightMax;
    unsigned int nHeightInc;
    float fFrameRateValue;
    float fFrameRateMin;
    float fFrameRateMax;
    unsigned int enPixelType;
    unsigned int nReserved[4];
} MV_IMAGE_BASIC_INFO;

typedef struct _MV_FRAME_OUT_INFO_EX_
{
    unsigned short nWidth;
    unsigned short nHeight;
    unsigned int enPixelType;
    unsigned int nFrameNum;
    unsigned int nDevTimeStampHigh;
    unsigned int nDevTimeStampLow;
    unsigned int nReserved0;
    int64_t nHostTimeStamp;
    unsigned int nFrameLen;
    unsigned int nSecondCount;
    unsigned int nCycleCount;
    unsigned int nCycleOffset;
    float fGain;
    float fExposureTime;
    unsigned int nAverageBrightness;
    unsigned int nRed;
    unsigned int nGreen;
    unsigned int nBlue;
    unsigned int nFrameCounter;
    unsigned int nTriggerIndex;
    unsigned int nInput;
    unsigned int nOutput;
    unsigned int nOffsetX;
    unsigned int nOffsetY;
    unsigned int nChunkWidth;
    unsigned int nChunkHeight;
    unsigned int nLostPacket;
    unsigned int nUnparsedChunkNum;
    unsigned int nReserved[35];
} MV_FRAME_OUT_INFO_EX;

typedef struct _MV_FRAME_OUT_
{
    unsigned char *pBufAddr;
    unsigned int nBufSize;
    MV_FRAME_OUT_INFO_EX stFrameInfo;
    unsigned int nReserved[16];
} MV_FRAME_OUT;

/* ------------------------------------------------------------ 通用属性结构 */

typedef struct _MVCC_INTVALUE_EX_
{
    unsigned int nCurValue;
    unsigned int nMax;
    unsigned int nMin;
    unsigned int nInc;
    unsigned int nReserved[16];
} MVCC_INTVALUE_EX;

typedef struct _MVCC_FLOATVALUE_
{
    float fCurValue;
    float fMax;
    float fMin;
    unsigned int nReserved[4];
} MVCC_FLOATVALUE;

typedef struct _MVCC_ENUMVALUE_EX_
{
    unsigned int nCurValue;
    unsigned int nSupportedNum;
    unsigned int nSupportValue[64];
    unsigned int nReserved[4];
} MVCC_ENUMVALUE_EX;

typedef struct _MVCC_STRINGVALUE_
{
    char chCurValue[256];
    int64_t nMaxLength;
    int64_t nReserved[2];
} MVCC_STRINGVALUE;

/* ---------------------------------------------------------------- 接口声明 */

/* 初始化/反初始化 SDK（本替身无副作用） */
int MV_CC_Initialize(void);
int MV_CC_Finalize(void);

/* 枚举设备 / 句柄管理 */
int MV_CC_EnumDevices(unsigned int nTLayerType, MV_CC_DEVICE_INFO_LIST *pstDevList);
int MV_CC_CreateHandle(void **handle, const MV_CC_DEVICE_INFO *pstDevInfo);
int MV_CC_DestroyHandle(void *handle);
int MV_CC_OpenDevice(void *handle);
int MV_CC_CloseDevice(void *handle);

/* 取流 */
int MV_CC_SetImageNodeNum(void *handle, unsigned int nNum);
int MV_CC_StartGrabbing(void *handle);
int MV_CC_StopGrabbing(void *handle);
int MV_CC_GetImageBuffer(void *handle, MV_FRAME_OUT *pstFrame, unsigned int nMsec);
int MV_CC_FreeImageBuffer(void *handle, MV_FRAME_OUT *pstFrame);

/* 属性读写 */
int MV_CC_GetIntValueEx(void *handle, const char *strKey, MVCC_INTVALUE_EX *pstIntValue);
int MV_CC_SetIntValueEx(void *handle, const char *strKey, unsigned int nValue);
int MV_CC_GetEnumValueEx(void *handle, const char *strKey, MVCC_ENUMVALUE_EX *pstEnumValue);
int MV_CC_SetEnumValue(void *handle, const char *strKey, unsigned int nValue);
int MV_CC_GetBoolValue(void *handle, const char *strKey, bool *pbValue);
int MV_CC_SetBoolValue(void *handle, const char *strKey, bool bValue);
int MV_CC_GetFloatValue(void *handle, const char *strKey, MVCC_FLOATVALUE *pstFloatValue);
int MV_CC_SetFloatValue(void *handle, const char *strKey, float fValue);
int MV_CC_GetStringValue(void *handle, const char *strKey, MVCC_STRINGVALUE *pstStringValue);
int MV_CC_SetStringValue(void *handle, const char *strKey, const char *strValue);
int MV_CC_SetCommandValue(void *handle, const char *strKey);

#ifdef __cplusplus
}
#endif

#endif /* FAKE_MV_CAMERA_CONTROL_H */
