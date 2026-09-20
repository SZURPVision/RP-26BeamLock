#ifndef __CRC_H_
#define __CRC_H_

#include <cstdint>

// CRC8
const uint8_t CRC8_INIT = 0xff;
void Append_CRC8_Check_Sum(uint8_t* pchMessage, uint16_t dwLength);
uint32_t Verify_CRC8_Check_Sum(uint8_t* pchMessage, uint16_t dwLength);
uint8_t Get_CRC8_Check_Sum(uint8_t* pchMessage, uint16_t dwLength, uint8_t ucCRC8);

// CRC16
const uint16_t CRC_INIT = 0xffff;
void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);

#endif
