#ifndef IMU_FUNCTIONS_H
#define IMU_FUNCTIONS_H

#include <Arduino.h>

void CopeCmdData(unsigned char ucData);
void ShowHelp(void);
void CmdProcess(void);
void AutoScanSensor(void);
void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
void Delayms(uint16_t ucMs);

#endif
