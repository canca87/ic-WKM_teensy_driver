/*-
 * BSD 2-Clause License
 *
 * Copyright (c) 2012-2018, Jan Breuer
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file   scpi-def.c
 * @date   Thu Nov 15 10:58:45 UTC 2012
 *
 * @brief  SCPI parser test
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "scpi/scpi.h"
#include "scpi-def.h"

//------- macro's --------------
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C //the address for the internal reset register
#define CPU_RESTART_VAL 0x5FA0004 //the value for the reset register that triggrs a system reset
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL); //the macro to reset the teensy program

OneWire  ds(temperature_sensor_pin);  // setup the onewire sensor
float system_temperature = -88; //default un-read temperature value
byte psu_value = 0; //stores the current PSU setting
float PAC_voltage_multiplier[6];
float PAC_current_multiplier[6];
uint16_t wavelength[6];
float max_power_mw[6];
uint32_t serial_number[6];

// ************** NON SCPI functions ----------------
//reads a byte from the PAC sensor.
byte readPAC(int deviceaddress, unsigned int regaddress, int wire_bus) 
{
  byte rdata = 0x2D; //default is '-' character if read fails.
  if (wire_bus == 2) {
    Wire1.beginTransmission(deviceaddress);
    Wire1.send((int)(regaddress & 0xFF)); // address of register to read
    Wire1.endTransmission();
    Wire1.requestFrom(deviceaddress,1);
    
    if (Wire1.available()) {
        rdata = Wire1.receive();
    }
  }
  else {
    Wire.beginTransmission(deviceaddress);
    Wire.send((int)(regaddress & 0xFF)); // address of register to read
    Wire.endTransmission();
    Wire.requestFrom(deviceaddress,1);
    
    if (Wire.available())  {
        rdata = Wire.receive();
    }
  }
  
  return rdata;
}
//reads the 10-bit current ADC value
uint16_t readCurrent(byte index) {
  int wire_bank = 1;
  if (index >= 3) {
      wire_bank = 2;
  }  
  byte addr = laser_PAC_address[index];
  uint16_t result = 0;
  
  result = readPAC(addr,0x12,wire_bank) << 2;
  result |= (readPAC(addr,0x13,wire_bank) >> 6) & 0x0003;
  
  return result;
}
//reads the 10-bit voltage ADC value
uint16_t readVoltage(byte index) {
  int wire_bank = 1;
  if (index >= 3) {
      wire_bank = 2;
  }  
  byte addr = laser_PAC_address[index];
  uint16_t result = 0;
  
  result = readPAC(addr,0x10,wire_bank) << 2;
  result |= (readPAC(addr,0x11,wire_bank) >> 6) & 0x0003;
  
  return result;
}
//reads the 10-bit power ADC value
uint16_t readPower(byte index) {
  int wire_bank = 1;
  if (index >= 3) {
      wire_bank = 2;
  }  
  byte addr = laser_PAC_address[index];
  uint16_t result = 0;
  
    result = readPAC(addr,0x1d,wire_bank) << 2;

    result |= (readPAC(addr,0x1e,wire_bank) >> 6) & 0x0003;

  return result;
}

/*
 * update_power_bank
 * 
 * initalises one power banks to a digital value
 * 
 * Returns VOID
 */
void update_power_bank(byte address, byte val) {
    // All power banks are on SPI0 (Wire):
    Wire.beginTransmission(address);
    Wire.write(0);
    Wire.write(val);
    Wire.endTransmission();
}

/*
 * update_power_banks
 * 
 * initalises both power banks to the same digital value
 * 
 * Returns VOID
 */
void update_power_banks(byte val) {
    update_power_bank(power_bank_A_address,val);
    update_power_bank(power_bank_B_address,val);
}

byte read_eeprom_byte(byte module, uint16_t address) {
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    byte return_value = 0xFF;
    digitalWriteFast(laser_EEPROM_pin[module-1],LOW);
    SPI.transfer(0b00000011); //read instruction
    SPI.transfer(0xFF & (byte)(address>>8)); //data address to read
    SPI.transfer(0xFF & (byte)address); //data address to read
    return_value = SPI.transfer(0);
    digitalWriteFast(laser_EEPROM_pin[module-1],HIGH);
    SPI.endTransaction();
    return(return_value);
}

void write_eeprom_byte(byte module, uint16_t address, byte value) {
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    // Setup to allow writing to the EEPROM:
    digitalWriteFast(laser_EEPROM_pin[module-1],LOW);
    SPI.transfer(0b00000110); //write enable instruction
    digitalWriteFast(laser_EEPROM_pin[module-1],HIGH);
    delay(1);
    // Write to the EEPROM a single byte:
    digitalWriteFast(laser_EEPROM_pin[module-1],LOW);
    SPI.transfer(0b00000010); //write instruction
    SPI.transfer(0xFF & (byte)(address>>8)); //data address to wrute
    SPI.transfer(0xFF & (byte)address); //data address to write
    SPI.transfer(value);
    digitalWriteFast(laser_EEPROM_pin[module-1],HIGH);
    delay(1);
    // Wait for the write to complete:
    digitalWriteFast(laser_EEPROM_pin[module-1],LOW);
    SPI.transfer(0b00000101); //read status register
    byte status = 1;
    byte timeout = 100; //100ms timeout for writes
    while (status && (timeout > 0)) {
        status = SPI.transfer(0); // get the value from the status register
        status = status & 1;
        delay(1);
        timeout --;
    }
    digitalWriteFast(laser_EEPROM_pin[module-1],HIGH);
    SPI.endTransaction();
    return;
}

int save_laser_module_eeprom(byte module) {
    // EEPROM map is found in the electronics bringup folder
    // First set the inital byte to 0xAA to indicate the EEPROM is initalised and functional
    write_eeprom_byte(module, 0, 0xAA);
    // Check that the first byte is correct (and there is actually a working EEPROM!)
    if (read_eeprom_byte(module, 0) == 0xAA) {
        // set the current sensor I2C address from the device
        write_eeprom_byte(module, 1, laser_PAC_address[module-1]);
        // set the PAC sensors current multiplier from the device
        long PAC_current_multiplier_long = PAC_current_multiplier[module-1] * 100000000;
        write_eeprom_byte(module, 2, PAC_current_multiplier_long & 0xFF);
        write_eeprom_byte(module, 3, (PAC_current_multiplier_long >> 8) & 0xFF);
        write_eeprom_byte(module, 4, (PAC_current_multiplier_long >> 16) & 0xFF);
        write_eeprom_byte(module, 5, (PAC_current_multiplier_long >> 24) & 0xFF);
        // set the PAC sensors voltage multiplier from the device
        long PAC_voltage_multiplier_long = PAC_voltage_multiplier[module-1] * 100000000;
        write_eeprom_byte(module, 6, PAC_voltage_multiplier_long & 0xFF);
        write_eeprom_byte(module, 7, (PAC_voltage_multiplier_long >> 8) & 0xFF);
        write_eeprom_byte(module, 8, (PAC_voltage_multiplier_long >> 16) & 0xFF);
        write_eeprom_byte(module, 9, (PAC_voltage_multiplier_long >> 24) & 0xFF);
        // set the diode wavelength
        write_eeprom_byte(module, 10, wavelength[module-1] & 0xFF);
        write_eeprom_byte(module, 11, (wavelength[module-1] >> 8) & 0xFF);
        // set the diode optical power level
        uint16_t max_power_bytes = max_power_mw[module-1] * 10;
        write_eeprom_byte(module, 12, max_power_bytes & 0xFF);
        write_eeprom_byte(module, 13, (max_power_bytes >> 8) & 0xFF);
        // set the serial number of the diode
        write_eeprom_byte(module, 14, serial_number[module-1] & 0xFF);
        write_eeprom_byte(module, 15, (serial_number[module-1] >> 8) & 0xFF);
        write_eeprom_byte(module, 16, (serial_number[module-1] >> 16) & 0xFF);
        return 0;
    }
    else return -1;
    
}

int save_system_eeprom(){
    /* Save to the teensy EEPROM:
    * address 0 is reserved.
    * address 1 is the system PSU voltage setpoint.
    */
    unsigned int eeAddress = 1;
    uint16_t count = 0;
    EEPROM.put(eeAddress, psu_value);
    count = count + sizeof(psu_value);
    return 0;
}

int save_parameters_to_eeprom(void){
    /* EEPROM map:
    * 
    * Undefined...
    */
    int ret_val = 0;
    ret_val += save_system_eeprom();
    for (int i = 1; i<=6; i++){
        ret_val += save_laser_module_eeprom(i);
    }
    //done.
    return ret_val;
}

int recall_laser_module_eeprom(byte module) {
    // EEPROM map is found in the electronics bringup folder
    // First byte is a check byte, it must be 0xAA for a valid initalised module to function:
    if (read_eeprom_byte(module,0) == 0xAA){
        // get the current sensor I2C address from the device
        laser_PAC_address[module-1] = read_eeprom_byte(module,1);
        // get the PAC sensors current multiplier from the device
        long PAC_current_multiplier_long = 0;
        PAC_current_multiplier_long |= 0xFF & read_eeprom_byte(module,2);
        PAC_current_multiplier_long |= 0xFF00 & (read_eeprom_byte(module,3) << 8);
        PAC_current_multiplier_long |= 0xFF0000 & (read_eeprom_byte(module,4) << 16);
        PAC_current_multiplier_long |= 0xFF000000 & (read_eeprom_byte(module,5) << 24);
        PAC_current_multiplier[module-1] = (float)PAC_current_multiplier_long / 100000000.0;
        // get the PAC sensors voltage multiplier from the device
        long PAC_voltage_multiplier_long = 0;
        PAC_voltage_multiplier_long |= 0xFF & read_eeprom_byte(module,6);
        PAC_voltage_multiplier_long |= 0xFF00 & (read_eeprom_byte(module,7) << 8);
        PAC_voltage_multiplier_long |= 0xFF0000 & (read_eeprom_byte(module,8) << 16);
        PAC_voltage_multiplier_long |= 0xFF000000 & (read_eeprom_byte(module,9) << 24);
        PAC_voltage_multiplier[module-1] = (float)PAC_voltage_multiplier_long / 100000000.0;
        // get the diode wavelength
        wavelength[module-1] = 0;
        wavelength[module-1] |= 0xFF & read_eeprom_byte(module,10);
        wavelength[module-1] |= 0xFF00 & (read_eeprom_byte(module,11) << 8);
        // get the diode optical power level
        uint16_t max_power_bytes = 0;
        max_power_bytes |= 0xFF & read_eeprom_byte(module,12);
        max_power_bytes |= 0xFF00 & (read_eeprom_byte(module,13) << 8);
        max_power_mw[module-1] = (float)max_power_bytes / 10.0;
        // get the serial number of the diode
        serial_number[module-1] = 0;
        serial_number[module-1] |= 0xFF & read_eeprom_byte(module,14);
        serial_number[module-1] |= 0xFF00 & (read_eeprom_byte(module,15) << 8);
        serial_number[module-1] |= 0xFF0000 & (read_eeprom_byte(module,16) << 16);
        return 0;
    }
    else {
        //make everything nonsensical:
        laser_PAC_address[module-1] = 1;
        PAC_voltage_multiplier[module-1] = 0;
        PAC_current_multiplier[module-1] = 0;
        wavelength[module-1] = 0;
        max_power_mw[module-1] = 0;
        serial_number[module-1] = 0;
        return -1;
    }
}

int recall_system_eeprom(){
    /* Recall from the teensy EEPROM:
    * address 0 is reserved.
    * address 1 is the system PSU voltage setpoint.
    */
    unsigned int eeAddress = 1;
    uint16_t count = 0;
    EEPROM.get(eeAddress, psu_value);
    count = count + sizeof(psu_value);
    return 0;
}
int recall_parameters_from_eeprom(void){
    /* EEPROM map:
    * 
    * Undefined...
    */
   recall_system_eeprom();
    
    for (int i = 1; i<=6; i++){
        recall_laser_module_eeprom(i);
    }

    //done.
    return 0;
}

void temperature_update(void){
    static byte i;
    static byte type_s;
    static byte data[12];
    static byte addr[8];
    ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = ds.read();
    }
    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - data[6];
        }
    } 
    else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
    }
    system_temperature = (float)raw / 16.0;
    //Serial.print("  Temperature = ");
    //Serial.println(system_temperature);
    
    if ( !ds.search(addr)) {
        ds.reset_search();
        delay(250);
        if ( !ds.search(addr)) {
            ds.reset_search();
            return;
        }
    }
    if (OneWire::crc8(addr, 7) != addr[7]) {
        return;
    }
    
    // the first ROM byte indicates which chip
    switch (addr[0]) {
        case 0x10:
            //Serial.println("  Chip = DS18S20");  // or old DS1820
            type_s = 1;
            break;
        case 0x28:
            //Serial.println("  Chip = DS18B20");
            type_s = 0;
            break;
        case 0x22:
            //Serial.println("  Chip = DS1822");
            type_s = 0;
        break;
        default:
            //Serial.println("Device is not a DS18x20 family device.");
        return;
    } 

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end 
}

// ************ SCPI functions --------------------

/** *TST?
 * Reimplement IEEE488.2 *TST?
 *
 * Result should be 0 if everything is ok
 * Result should be 1 if something goes wrong
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t My_CoreTstQ(scpi_t * context) {

    SCPI_ResultInt32(context, 0);

    return SCPI_RES_OK;
}

/** *SAV
 * SAVE data from ram to EEPROM
 *
 * Result should be 0 if everything is ok
 * Result should be 1 if something goes wrong
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t My_CoreSav(scpi_t * context) {

    SCPI_ResultInt32(context, save_parameters_to_eeprom());

    return SCPI_RES_OK;
}

/** *RCL
 * RECALL data from EEPROM to ram
 *
 * Result should be 0 if everything is ok
 * Result should be 1 if something goes wrong
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t My_CoreRcl(scpi_t * context) {

    SCPI_ResultInt32(context, recall_parameters_from_eeprom());
    
    return SCPI_RES_OK;
}

/** *RST
 * Restarts the CPU
 *
 * Result should never return (restart)
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t My_CoreRst(scpi_t * context) {

    CPU_RESTART;
    
    return SCPI_RES_OK;
}

/** SYSTem:TEMPerature?
 * Get the system temperature
 *
 * Result should be float
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t GetSystemTemperature(scpi_t * context) {
    temperature_update();
    SCPI_ResultFloat(context, system_temperature);

    return SCPI_RES_OK;
}

/** SYSTem:TEMPerature
 * Update the system temperature
 *
 * Result should be NULL
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t SetSystemTemperature(scpi_t * context) {
    
    temperature_update();

    return SCPI_RES_OK;
}

/** SYSTem:VOLTage
 * Update the system voltage
 *
 * Result should be NULL
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t SetSystemVoltage(scpi_t * context) {
    int32_t param1;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    // if is a byte.
    if (param1 >= 0 && param1 <=255) {
        psu_value = (byte)param1;
        update_power_banks(psu_value);
    }

    return SCPI_RES_OK;
}

/** SYSTem:VOLTage?
 * Get the system voltage
 *
 * Result should be byte (system voltage)
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t GetSystemVoltage(scpi_t * context) {
    // make sure the power banks are actually what you say they are:
    update_power_banks(psu_value);
    // return that value:
    SCPI_ResultInt32(context, psu_value);
    return SCPI_RES_OK;
}

/** SYSTem:TRIGger
 * Update the system trigger
 *
 * Result should be NULL
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t SetSystemTrigger(scpi_t * context) {
    int32_t param1;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    // if is == 0 turn off (if external trigger enabled).
    if (param1 == 0) {
        
    }
    // if is == 1 turn on (if external trigger enabled).
    if (param1 == 1) {
        
    }
    
    return SCPI_RES_OK;
}

/** LASer:POWer
 * Set laser digipot
 *
 * Result should be digpot setting in uint8
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t SetLaserPower(scpi_t * context) {
    int32_t param1;
    int32_t param2;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    /* read second parameter if present */
    if (!SCPI_ParamInt32(context, &param2, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= -1 && param1 <= 254) {
        byte module = (uint8_t)param1;
        if (param2 >= 0 && param2 <= 254) {
            laser_DIGIPOT_value[module-1] = (uint8_t)param2;
            // gain control of the SPI port
            // and configure settings
            SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
            // take the SS pin low to select the chip:
            digitalWriteFast(laser_DIGIPOT_pin[module-1],LOW);
            //  send in the address and value via SPI:
            SPI.transfer(0b00010001); //write to wiper 0
            SPI.transfer(laser_DIGIPOT_value[module-1]);
            // take the SS pin high to de-select the chip:
            digitalWriteFast(laser_DIGIPOT_pin[module-1],HIGH);
            // release control of the SPI port
            SPI.endTransaction();
            SCPI_ResultInt32(context, laser_DIGIPOT_value[module-1]);
        }
        else {
            SCPI_ResultInt32(context, -1);
        }
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}

/** LASer:POWer?
 * Get laser digpot
 *
 * Result should be in uint8
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t GetLaserPower(scpi_t * context) {
    int32_t param1;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= -1 && param1 <= 254) {
        byte module = (uint8_t)param1;
        SCPI_ResultInt32(context, laser_DIGIPOT_value[module-1]); 
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}

/** LASer:CURRent?
 * Get laser current
 *
 * Result should be in float
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t GetLaserCurrent(scpi_t * context) {
    int32_t param1;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= -1 && param1 <= 254) {
        byte module = (uint8_t)param1;
        float val = 0.488758553 * (float)readCurrent(module-1); //send the module # to the function (zero indexed)
        SCPI_ResultFloat(context, val);
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}

/** LASer:VOLTage?
 * Get laser PSU voltage
 *
 * Result should be in float
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t GetLaserVoltage(scpi_t * context) {
    int32_t param1;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= -1 && param1 <= 254) {
        byte module = (uint8_t)param1;
        float val = 0.01564027 * (float)readVoltage(module-1); //send the module # to the function (zero indexed)
        SCPI_ResultFloat(context, val);
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}

/** LASer:STATe
 * Set laser state
 *
 * Result should be bool on/off
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t SetLaserState(scpi_t * context) {
    int32_t param1;
    int32_t param2;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    /* read second parameter if present */
    if (!SCPI_ParamInt32(context, &param2, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        if (param2 >= 0 && param2 <= 1) {
            byte state = (uint8_t)param2;
            digitalWriteFast(laser_TTL_pin[module-1],state);
            if (digitalReadFast(laser_TTL_pin[module-1]) == 0) {
                SCPI_ResultInt32(context, 0);
            }
            else{
                SCPI_ResultInt32(context, 1);
            }
        }
    }
    else if (param1 == 0) {
        //if zero module is sent, this is a global command to toggle:
        if (param2 >= 0 && param2 <= 1) {
            byte state = (uint8_t)param2;
            int ret_val = 0;
            for (int i = 0; i<6; i++){
                digitalWriteFast(laser_TTL_pin[i],state);
                if (digitalReadFast(laser_TTL_pin[i]) == 1) {
                    ret_val ++;
                }
            }
            SCPI_ResultInt32(context, ret_val);
        }
        else {
            SCPI_ResultInt32(context, -1);
        }
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}

/** LASer:STATe?
 * Get laser state
 *
 * Result should be bool on/off
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t GetLaserState(scpi_t * context) {
    int32_t param1;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        if (digitalReadFast(laser_TTL_pin[module-1]) == 0) {
            SCPI_ResultInt32(context, 0);
        }
        else{
            SCPI_ResultInt32(context, 1);
        }
    }
    else if (param1 == 0) {
        int ret_val = 0;
        for (int i = 0; i<6; i++) {
            if (digitalReadFast(laser_TTL_pin[i]) == 1) {
                ret_val ++;
            }
        }
        SCPI_ResultInt32(context, ret_val);
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}

// get wavelength
static scpi_result_t GetWavelength(scpi_t * context) {
    int32_t param1;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        SCPI_ResultInt32(context, wavelength[module-1]);
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}
// set waveleggth
static scpi_result_t SetWavelength(scpi_t * context) {
    int32_t param1;
    int32_t param2;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    /* read second parameter if present */
    if (!SCPI_ParamInt32(context, &param2, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        if (param2 >= 0 && param2 <= 5000) {
            wavelength[module-1] = (uint16_t)param2;
            SCPI_ResultInt32(context, 0);
        }
        else {
            SCPI_ResultInt32(context, -1);
        }
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}
// get serial number
static scpi_result_t GetSerialNumber(scpi_t * context) {
    int32_t param1;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        SCPI_ResultInt32(context, serial_number[module-1]);
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}
// set serial number
static scpi_result_t SetSerialNumber(scpi_t * context) {
    int32_t param1;
    int32_t param2;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    /* read second parameter if present */
    if (!SCPI_ParamInt32(context, &param2, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        if (param2 >= 0 && param2 <= 999999) {
            serial_number[module-1] = (uint32_t)param2;
            SCPI_ResultInt32(context, 0);
        }
        else {
            SCPI_ResultInt32(context, -1);
        }
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}
// get PACXV
static scpi_result_t GetPACXV(scpi_t * context) {
    int32_t param1;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        SCPI_ResultFloat(context, PAC_voltage_multiplier[module-1]);
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}
// set PACXV
static scpi_result_t SetPACXV(scpi_t * context) {
    int32_t param1;
    float param2;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    /* read second parameter if present */
    if (!SCPI_ParamFloat(context, &param2, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        if (param2 >= 0 && param2 <= 1023) {
            PAC_voltage_multiplier[module-1] = param2;
            SCPI_ResultInt32(context, 0);
        }
        else {
            SCPI_ResultInt32(context, -1);
        }
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}
// get PACXI
static scpi_result_t GetPACXI(scpi_t * context) {
    int32_t param1;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        SCPI_ResultFloat(context, PAC_current_multiplier[module-1]);
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}
// set PACXI
static scpi_result_t SetPACXI(scpi_t * context) {
    int32_t param1;
    float param2;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    /* read second parameter if present */
    if (!SCPI_ParamFloat(context, &param2, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        if (param2 >= 0 && param2 <= 1023) {
            PAC_current_multiplier[module-1] = param2;
            SCPI_ResultInt32(context, 0);
        }
        else {
            SCPI_ResultInt32(context, -1);
        }
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}
// get PACADDR
static scpi_result_t GetPACADDR(scpi_t * context) {
    int32_t param1;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        SCPI_ResultInt32(context, laser_PAC_address[module-1]);
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}
// set PACADDR
static scpi_result_t SetPACADDR(scpi_t * context) {
    int32_t param1;
    int32_t param2;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    /* read second parameter if present */
    if (!SCPI_ParamInt32(context, &param2, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        if (param2 >= 0 && param2 <= 1023) {
            laser_PAC_address[module-1] = (byte)param2;
            SCPI_ResultInt32(context, 0);
        }
        else {
            SCPI_ResultInt32(context, -1);
        }
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}
// get PMAX
static scpi_result_t GetPMAX(scpi_t * context) {
    int32_t param1;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        SCPI_ResultFloat(context, max_power_mw[module-1]);
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}
// set PMAX
static scpi_result_t SetPMAX(scpi_t * context) {
    int32_t param1;
    float param2;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    /* read second parameter if present */
    if (!SCPI_ParamFloat(context, &param2, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= 1 && param1 <= 6) {
        byte module = (uint8_t)param1;
        if (param2 >= 0 && param2 <= 1023) {
            max_power_mw[module-1] = param2;
            SCPI_ResultInt32(context, 0);
        }
        else {
            SCPI_ResultInt32(context, -1);
        }
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}

/** LASer:EEProm?
 * Get laser eeprom dump
 *
 * Result should be serial dump of eeprom
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t GetEEPROM_dump(scpi_t * context) {
    int32_t param1;

    /* read first parameter if present */
    if (!SCPI_ParamInt32(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if Module is >= 1 and <=6
    if (param1 >= -1 && param1 <= 254) {
        byte module = (uint8_t)param1;
        uint16_t address = 0x0000;
        Serial.print("Module ");
        Serial.print(module);
        Serial.println(" EEPROM:");
        for (int row = 0; row <10; row ++){
            for (int col = 0; col <10; col++){
                Serial.print(read_eeprom_byte(module,address),16);
                Serial.print(" ");
                address++;
            }
            Serial.println("");
        }
        Serial.println("------------------");
    }
    else {
        SCPI_ResultInt32(context, -1);
    }
    
    return SCPI_RES_OK;
}

/** SYSTem:I2C?
 * Get scan of the systems I2C BUs
 *
 * Result should be list of i2c bus address
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t GetI2C(scpi_t * context) {
    int found = 0;
    //Serial.print("Starting scan...\n");
    for(byte target = 1; target <= 0x7F; target++) // sweep addr, skip general call
    {
        Wire.beginTransmission(target);       // slave addr
        Wire.endTransmission();               // no data, just addr
        switch(Wire.status()){
            case I2C_WAITING:  
                Serial.printf("0x%02X,", target);
                found = 1;
                break;
            case I2C_ADDR_NAK: 
                break;
            default:
                break;
        }
    }
    if(!found) Serial.print("No devices found.");

    return SCPI_RES_OK;
}

/** SYSTem:TIMEout?
 * Get the timeout time in microseconds
 *
 * Result should be time in microseconds
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t GetTimeout(scpi_t * context) {
    
    SCPI_ResultInt64(context, watchdog_timeout_microseconds);

    return SCPI_RES_OK;
}

/** SYSTem:TIMEout
 * Set the timeout time in microseconds
 *
 * Result should be time in microseconds
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t SetTimeout(scpi_t * context) {
    
    int64_t param1;
    
    /* read first parameter if present */
    if (!SCPI_ParamInt64(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    
    // if time is >= 1
    if (param1 >= 1) {
        watchdog_timeout_microseconds = (long)param1;
    }
    else {
        watchdog_timeout_microseconds = 0;
    }
    

    SCPI_ResultInt64(context, watchdog_timeout_microseconds);

    return SCPI_RES_OK;
}

const scpi_command_t scpi_commands[] = {
    /* IEEE Mandated Commands (SCPI std V1999.0 4.1.1) */
    { .pattern = "*CLS", .callback = SCPI_CoreCls,},
    { .pattern = "*ESE", .callback = SCPI_CoreEse,},
    { .pattern = "*ESE?", .callback = SCPI_CoreEseQ,},
    { .pattern = "*ESR?", .callback = SCPI_CoreEsrQ,},
    { .pattern = "*IDN?", .callback = SCPI_CoreIdnQ,},
    { .pattern = "*OPC", .callback = SCPI_CoreOpc,},
    { .pattern = "*OPC?", .callback = SCPI_CoreOpcQ,},
    { .pattern = "*RST", .callback = My_CoreRst,},
    { .pattern = "*SRE", .callback = SCPI_CoreSre,},
    { .pattern = "*SRE?", .callback = SCPI_CoreSreQ,},
    { .pattern = "*STB?", .callback = SCPI_CoreStbQ,},
    { .pattern = "*TST?", .callback = My_CoreTstQ,},
    { .pattern = "*SAV", .callback = My_CoreSav,},
    { .pattern = "*RCL", .callback = My_CoreRcl,},
    { .pattern = "*WAI", .callback = SCPI_CoreWai,},

    /* Required SCPI commands (SCPI std V1999.0 4.2.1) */
    {.pattern = "SYSTem:ERRor[:NEXT]?", .callback = SCPI_SystemErrorNextQ,},
    {.pattern = "SYSTem:ERRor:COUNt?", .callback = SCPI_SystemErrorCountQ,},
    {.pattern = "SYSTem:VERSion?", .callback = SCPI_SystemVersionQ,},

    /* {.pattern = "STATus:OPERation?", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:EVENt?", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:CONDition?", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:ENABle", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:ENABle?", .callback = scpi_stub_callback,}, */

    {.pattern = "STATus:QUEStionable[:EVENt]?", .callback = SCPI_StatusQuestionableEventQ,},
    /* {.pattern = "STATus:QUEStionable:CONDition?", .callback = scpi_stub_callback,}, */
    {.pattern = "STATus:QUEStionable:ENABle", .callback = SCPI_StatusQuestionableEnable,},
    {.pattern = "STATus:QUEStionable:ENABle?", .callback = SCPI_StatusQuestionableEnableQ,},

    {.pattern = "STATus:PRESet", .callback = SCPI_StatusPreset,},

    {.pattern = "SYSTem:TEMPerature?", .callback = GetSystemTemperature,},
    {.pattern = "SYSTem:TEMPerature", .callback = SetSystemTemperature,},
    {.pattern = "SYSTem:TIMEout?", .callback = GetTimeout,},
    {.pattern = "SYSTem:TIMEout", .callback = SetTimeout,},
    {.pattern = "SYSTem:VOLTage?", .callback = GetSystemVoltage,},
    {.pattern = "SYSTem:VOLTage", .callback = SetSystemVoltage,},
    {.pattern = "SYSTem:I2C?", .callback = GetI2C,},
    //{.pattern = "SYSTem:TRIGger", .callback = SetSystemTrigger,},
    {.pattern = "LASer:POWer", .callback = SetLaserPower,},
    {.pattern = "LASer:POWer?", .callback = GetLaserPower,},
    {.pattern = "LASer:CURRent?", .callback = GetLaserCurrent,},
    {.pattern = "LASer:VOLTage?", .callback = GetLaserVoltage,},
    {.pattern = "LASer:STATe", .callback = SetLaserState,},
    {.pattern = "LASer:STATe?", .callback = GetLaserState,},
    {.pattern = "LASer:EEProm?", .callback = GetEEPROM_dump,},
    {.pattern = "LASer:WAVelength?", .callback = GetWavelength,},
    {.pattern = "LASer:WAVelength", .callback = SetWavelength,},
    {.pattern = "LASer:SERial?", .callback = GetSerialNumber,},
    {.pattern = "LASer:SERial", .callback = SetSerialNumber,},
    {.pattern = "LASer:PMAX?", .callback = GetPMAX,},
    {.pattern = "LASer:PMAX", .callback = SetPMAX,},
    {.pattern = "LASer:PACADdr?", .callback = GetPACADDR,},
    {.pattern = "LASer:PACADdr", .callback = SetPACADDR,},
    {.pattern = "LASer:PACXI?", .callback = GetPACXI,},
    {.pattern = "LASer:PACXI", .callback = SetPACXI,},
    {.pattern = "LASer:PACXV?", .callback = GetPACXV,},
    {.pattern = "LASer:PACXV", .callback = SetPACXV,},
    //{.pattern = "LASer:TST", .callback = SetLaserState,}, //TODO

    SCPI_CMD_LIST_END
};


size_t SCPI_Write(scpi_t * context, const char * data, size_t len) {
    (void) context;
    return fwrite(data, 1, len, stdout);
}

scpi_result_t SCPI_Flush(scpi_t * context) {
    (void) context;

    return SCPI_RES_OK;
}

int SCPI_Error(scpi_t * context, int_fast16_t err) {
    (void) context;

    fprintf(stderr, "**ERROR: %d, \"%s\"\r\n", (int16_t) err, SCPI_ErrorTranslate(err));
    return 0;
}

scpi_result_t SCPI_Control(scpi_t * context, scpi_ctrl_name_t ctrl, scpi_reg_val_t val) {
    (void) context;

    if (SCPI_CTRL_SRQ == ctrl) {
        fprintf(stderr, "**SRQ: 0x%X (%d)\r\n", val, val);
    } else {
        fprintf(stderr, "**CTRL %02x: 0x%X (%d)\r\n", ctrl, val, val);
    }
    return SCPI_RES_OK;
}

scpi_result_t SCPI_Reset(scpi_t * context) {
    (void) context;

    fprintf(stderr, "**Reset\r\n");
    delayMicroseconds(100000);
    CPU_RESTART;
    return SCPI_RES_OK;
}

scpi_result_t SCPI_SystemCommTcpipControlQ(scpi_t * context) {
    (void) context;

    return SCPI_RES_ERR;
}


scpi_interface_t scpi_interface = {
    .error = SCPI_Error,
    .write = SCPI_Write,
    .control = SCPI_Control,
    .flush = SCPI_Flush,
    .reset = SCPI_Reset,
};

char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];
scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];

scpi_t scpi_context;
