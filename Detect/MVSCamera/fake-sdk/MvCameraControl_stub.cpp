/*
 * MvCameraControl_stub.cpp —— 海康 MVS SDK 的空实现（编译验证用，见 USE_FAKE_MVS）
 *
 * 所有接口都是空实现：指针参数仅做「非空则清零」处理，返回值统一为 MV_OK。
 * 目的只有一个 —— 让 MVSCamera 能通过编译与链接，不模拟任何相机行为。
 * 因此运行时会得到「枚举不到设备」的结果，这是预期行为。
 */
#include <MvCameraControl.h>

#include <cstdlib>
#include <cstring>

namespace
{
/* 每个句柄指向这样一小块内存，用来让 handle 非空、可释放 */
struct StubHandle
{
    unsigned int magic;
};

inline bool valid(const void *p) { return p != nullptr; }
} // namespace

extern "C" {

int MV_CC_IsStubSDK(void) { return 1; }

int MV_CC_Initialize(void) { return MV_OK; }
int MV_CC_Finalize(void) { return MV_OK; }

int MV_CC_EnumDevices(unsigned int nTLayerType, MV_CC_DEVICE_INFO_LIST *pstDevList)
{
    (void)nTLayerType;
    /* 空实现：一个设备都找不到，但按「成功」返回 */
    if (valid(pstDevList))
        std::memset(pstDevList, 0, sizeof(*pstDevList));
    return MV_OK;
}

int MV_CC_CreateHandle(void **handle, const MV_CC_DEVICE_INFO *pstDevInfo)
{
    (void)pstDevInfo;
    if (!valid(handle))
        return MV_OK;
    StubHandle *h = static_cast<StubHandle *>(std::calloc(1, sizeof(StubHandle)));
    if (h != nullptr)
        h->magic = 0x4D565354u; /* "MVST" */
    *handle = h;
    return MV_OK;
}

int MV_CC_DestroyHandle(void *handle)
{
    std::free(handle);
    return MV_OK;
}

int MV_CC_OpenDevice(void *handle) { return valid(handle) ? MV_OK : MV_OK; }
int MV_CC_CloseDevice(void *handle) { return valid(handle) ? MV_OK : MV_OK; }

int MV_CC_SetImageNodeNum(void *handle, unsigned int nNum)
{
    (void)handle;
    (void)nNum;
    return MV_OK;
}

int MV_CC_StartGrabbing(void *handle) { return valid(handle) ? MV_OK : MV_OK; }
int MV_CC_StopGrabbing(void *handle) { return valid(handle) ? MV_OK : MV_OK; }

int MV_CC_GetImageBuffer(void *handle, MV_FRAME_OUT *pstFrame, unsigned int nMsec)
{
    (void)handle;
    (void)nMsec;
    /* 空实现：不产生图像，只把结构体清零，避免调用方读到脏数据 */
    if (valid(pstFrame))
        std::memset(pstFrame, 0, sizeof(*pstFrame));
    return MV_OK;
}

int MV_CC_FreeImageBuffer(void *handle, MV_FRAME_OUT *pstFrame)
{
    (void)handle;
    (void)pstFrame;
    return MV_OK;
}

int MV_CC_GetIntValueEx(void *handle, const char *strKey, MVCC_INTVALUE_EX *pstIntValue)
{
    (void)handle;
    (void)strKey;
    if (valid(pstIntValue))
        std::memset(pstIntValue, 0, sizeof(*pstIntValue));
    return MV_OK;
}

int MV_CC_SetIntValueEx(void *handle, const char *strKey, unsigned int nValue)
{
    (void)handle;
    (void)strKey;
    (void)nValue;
    return MV_OK;
}

int MV_CC_GetEnumValueEx(void *handle, const char *strKey, MVCC_ENUMVALUE_EX *pstEnumValue)
{
    (void)handle;
    (void)strKey;
    if (valid(pstEnumValue))
        std::memset(pstEnumValue, 0, sizeof(*pstEnumValue));
    return MV_OK;
}

int MV_CC_SetEnumValue(void *handle, const char *strKey, unsigned int nValue)
{
    (void)handle;
    (void)strKey;
    (void)nValue;
    return MV_OK;
}

int MV_CC_GetBoolValue(void *handle, const char *strKey, bool *pbValue)
{
    (void)handle;
    (void)strKey;
    if (valid(pbValue))
        *pbValue = false;
    return MV_OK;
}

int MV_CC_SetBoolValue(void *handle, const char *strKey, bool bValue)
{
    (void)handle;
    (void)strKey;
    (void)bValue;
    return MV_OK;
}

int MV_CC_GetFloatValue(void *handle, const char *strKey, MVCC_FLOATVALUE *pstFloatValue)
{
    (void)handle;
    (void)strKey;
    if (valid(pstFloatValue))
        std::memset(pstFloatValue, 0, sizeof(*pstFloatValue));
    return MV_OK;
}

int MV_CC_SetFloatValue(void *handle, const char *strKey, float fValue)
{
    (void)handle;
    (void)strKey;
    (void)fValue;
    return MV_OK;
}

int MV_CC_GetStringValue(void *handle, const char *strKey, MVCC_STRINGVALUE *pstStringValue)
{
    (void)handle;
    (void)strKey;
    if (valid(pstStringValue))
        std::memset(pstStringValue, 0, sizeof(*pstStringValue));
    return MV_OK;
}

int MV_CC_SetStringValue(void *handle, const char *strKey, const char *strValue)
{
    (void)handle;
    (void)strKey;
    (void)strValue;
    return MV_OK;
}

int MV_CC_SetCommandValue(void *handle, const char *strKey)
{
    (void)handle;
    (void)strKey;
    return MV_OK;
}

} /* extern "C" */
