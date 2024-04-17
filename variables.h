#ifndef VARIABLE_H
#define VARIABLE_H

#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[8] = { 0, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };

#endif
