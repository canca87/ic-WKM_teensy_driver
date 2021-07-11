#ifndef __HARDWARE_DEF_H_
#define __HARDWARE_DEF_H_

#include "Arduino.h"

/// defining IO pins for the teensy ///
#define temperature_sensor_pin 23
#define power_bank_A_address 0b0101110
#define power_bank_B_address 0b0101111

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

extern byte laser_PAC_address[6];
extern byte laser_TTL_pin[6];
extern byte laser_EEPROM_pin[6];
extern byte laser_DIGIPOT_pin[6];
extern byte laser_DIGIPOT_value[6];

extern long watchdog_timeout_microseconds;

#endif /* __HARDWARE_DEF_H_ */