#include "Arduino.h"
#include "scpi/scpi.h"
#include "scpi-def.h"

//define hardware IOs
byte laser_PAC_address[6];
byte laser_TTL_pin[6];
byte laser_EEPROM_pin[6];
byte laser_DIGIPOT_pin[6];
byte laser_DIGIPOT_value[6];
long watchdog_timeout_microseconds = 30000000; //default is 30 seconds

//Timers and functions
IntervalTimer updateTemperature, watchdogTimer;
void test_scpi(void);
void serial_scpi(void);
void LEDtoggle_function(void);
void temperature_update_isr(void);
void recall_parameters(void);
void update_power(void);
void init_PAC_sensor_voltage(int);
void watchdog_timeout(void);

// Macros and functions for testing SCPI comms:
#define TEST_SCPI_INPUT(cmd) result = SCPI_Input(&scpi_context, cmd, strlen(cmd))

//redirect printf functions to serial port:
extern "C" {
int _write(int file, char *ptr, int len) {
  int todo;

  for (todo = 0; todo < len; todo++) {
      Serial.print(ptr[todo]);
    }
    return len;
  }
}

void setup()
{
  // laser bank A hardware IO configuration
  laser_DIGIPOT_pin[0] = 8;
  laser_EEPROM_pin[0] = 6;
  laser_TTL_pin[0] = 0;
  laser_PAC_address[0] = 0x18; //this will come from the EEPROM on each channel... eventually....
  // laser bank B hardware IO configuration
  laser_DIGIPOT_pin[1] = 14;
  laser_EEPROM_pin[1] = 9;
  laser_TTL_pin[1] = 1;
  laser_PAC_address[1] = 0x18; //this will come from the EEPROM on each channel... eventually....
  // laser bank C hardware IO configuration
  laser_DIGIPOT_pin[2] = 17;
  laser_EEPROM_pin[2] = 15;
  laser_TTL_pin[2] = 2;
  laser_PAC_address[2] = 0x18; //this will come from the EEPROM on each channel... eventually....
  // laser bank D hardware IO configuration
  laser_DIGIPOT_pin[3] = 22;
  laser_EEPROM_pin[3] = 20;
  laser_TTL_pin[3] = 3;
  laser_PAC_address[3] = 0x18; //this will come from the EEPROM on each channel... eventually....
  // laser bank E hardware IO configuration
  laser_DIGIPOT_pin[4] = 25;
  laser_EEPROM_pin[4] = 31;
  laser_TTL_pin[4] = 4;
  laser_PAC_address[4] = 0x18; //this will come from the EEPROM on each channel... eventually....
  // laser bank F hardware IO configuration
  laser_DIGIPOT_pin[5] = 28;
  laser_EEPROM_pin[5] = 26;
  laser_TTL_pin[5] = 5;
  laser_PAC_address[5] = 0x18; //this will come from the EEPROM on each channel... eventually....

   for (int i = 0; i<6; i++) {
    pinMode(laser_DIGIPOT_pin[i],OUTPUT);
    digitalWriteFast(laser_DIGIPOT_pin[i],HIGH); //active low IOs
    pinMode(laser_EEPROM_pin[i],OUTPUT);
    digitalWriteFast(laser_EEPROM_pin[i],HIGH); //active low IOs
    pinMode(laser_TTL_pin[i],OUTPUT);
    digitalWriteFast(laser_TTL_pin[i],LOW); //active high IOs
    laser_DIGIPOT_value[i] = 0;
  }

  pinMode(6,OUTPUT);
  digitalWriteFast(6,HIGH);
  // Setup for Master mode, pins 18/19, external pullups, 400kHz, 10ms default timeout
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(10000); // 10ms
  // Setup for Master mode, pins 29/30, external pullups, 400kHz, 10ms default timeout
  Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_29_30, I2C_PULLUP_EXT, 400000);
  Wire1.setDefaultTimeout(10000); // 10ms
  
  //turn this on for the eeprom
  SPI.begin();

  Serial.begin(9600); //start the serial port (baud is irrelevant for USB serial)
  //initalise the SCPI interface: scpi-def.h has all the commands...
  SCPI_Init(&scpi_context,
          scpi_commands,
          &scpi_interface,
          scpi_units_def,
          SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
          scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH,
          scpi_error_queue_data, SCPI_ERROR_QUEUE_SIZE);

  //get the PAC sensor details from the EEPROMs
  recall_parameters();

  for (int i = 0; i<6; i++) {
    init_PAC_sensor_voltage(i);
  }

  pinMode(temperature_sensor_pin,INPUT_PULLUP);
  analogReadResolution(15); //15-bit ADC

  update_power(); //set up the main PSU for a fixed voltage
  //updateTemperature.begin(temperature_update_isr,1000000); //every second as per datasheet
  
  watchdogTimer.begin(watchdog_timeout,watchdog_timeout_microseconds);
}

void loop()
{
  //test_scpi();
  serial_scpi();
}

void temperature_update_isr(void){
  int result; //used by the TEST_SCPI_INPUT macro...
  (void)result; //this is to silence the unused variable warning from the compiler. Its used by the macro below:
  //test the *IDN? input. Should print itentity to the serial port:
  TEST_SCPI_INPUT("SYSTEM:TEMPERATURE\r\n");
}

void recall_parameters(void){
  int result; //used by the TEST_SCPI_INPUT macro...
  (void)result; //this is to silence the unused variable warning from the compiler. Its used by the macro below:
  //test the *IDN? input. Should print itentity to the serial port:
  TEST_SCPI_INPUT("*RCL\r\n");
}

void update_power(void){
  int result; //used by the TEST_SCPI_INPUT macro...
  (void)result; //this is to silence the unused variable warning from the compiler. Its used by the macro below:
  //test the *IDN? input. Should print itentity to the serial port:
  TEST_SCPI_INPUT("SYSTEM:VOLTAGE?\r\n");//asking for the power value sets the value (usefull on power-up)
}
void serial_scpi(void) {
  int result;
  (void)result; //this is to silence the unused variable warning from the compiler. Its used by the macro below:
  static String cmd = "";
  static String scpi_cmd = "";
  if (Serial.available()) //if ther is data avaliable
  {
    //digitalWriteFast(LED_BUILTIN,!digitalReadFast(LED_BUILTIN));
    char ch = Serial.read(); //read the character
    cmd = ch;
    scpi_cmd += ch;
    //Serial.println(ch); //reprint the number/letter to the screen
    if(ch >= '\n') // is it a new line?
    {
      char _cmd[sizeof(cmd)];
      cmd.toCharArray(_cmd,sizeof(_cmd));
      //Serial.print(_cmd);
      
      TEST_SCPI_INPUT(_cmd);
      cmd = "";

      //reset the watchdog timer on a new line character
      if (watchdog_timeout_microseconds != 0) {
        watchdogTimer.begin(watchdog_timeout,watchdog_timeout_microseconds);
      }
      else {
        watchdogTimer.end();
      }
      
    }
    if(ch == '\n'){
      scpi_cmd = "";
    }
  }
}

void test_scpi(void) {
  int result; //used by the TEST_SCPI_INPUT macro...
  (void)result; //this is to silence the unused variable warning from the compiler. Its used by the macro below:
  //test the *IDN? input. Should print itentity to the serial port:
  TEST_SCPI_INPUT("*IDN?\r\n");
  //test the *TST? input. Should print 0 to the serial port:
  TEST_SCPI_INPUT("*TST?\r\n");
  //clear all queue errors:
  TEST_SCPI_INPUT("*CLS\r\n");
}

/*
 * init_PAC_sensor_current
 * 
 * initalises the PAC sensor to a current reading state
 * 
 * Returns VOID
 */
void init_PAC_sensor_voltage(int idx) {
    byte addr = laser_PAC_address[idx];
    if (idx >= 3){
      // need to put the sensor in READ mode first:
      Wire1.beginTransmission(addr);
      Wire1.send(1); //integration configuration register
      Wire1.send(0b00001110); //SMPL = 1 sample; VSFEN = filter enabled; VBFEN = filter enabled; RIOV = Override enabled; INTEN = read state.
      Wire1.endTransmission();
      //set gain to known gain level: (default is zero)
      Wire1.beginTransmission(addr);
      Wire1.send(0); // Gain configuration register
      Wire1.send(0b00001001); //I_RES = 14-bit; V_RES = 14-bit; DI_GAIN = 2; DV_GAIN = 2
      Wire1.endTransmission();
      //sets the measurement of Vsense in free-run mode
      Wire1.beginTransmission(addr);
      Wire1.send(2); //control register
      Wire1.send(0b11000000); // MXSL = V-bus; OFSR = 3v; TOUT = disabled; Sleep = normal; SLPOV = normal; RDAC = 0
      Wire1.endTransmission();
      // put the device into run mode:
      Wire1.beginTransmission(addr);
      Wire1.send(1); //integration configuration register
      Wire1.send(15); //SMPL = 1 sample; VSFEN = filter enabled; VBFEN = filter enabled; RIOV = Override enabled; INTEN = integrate state.
      Wire1.endTransmission();
    }
    else {
      // need to put the sensor in READ mode first:
      Wire.beginTransmission(addr);
      Wire.send(1); //integration configuration register
      Wire.send(0b00001110); //SMPL = 1 sample; VSFEN = filter enabled; VBFEN = filter enabled; RIOV = Override enabled; INTEN = read state.
      Wire.endTransmission();
      //set gain to known gain level: (default is zero)
      Wire.beginTransmission(addr);
      Wire.send(0); // Gain configuration register
      Wire.send(0b00001001); //I_RES = 14-bit; V_RES = 14-bit; DI_GAIN = 2; DV_GAIN = 2
      Wire.endTransmission();
      //sets the measurement of Vsense in free-run mode
      Wire.beginTransmission(addr);
      Wire.send(2); //control register
      Wire.send(0b11000000); // MXSL = V-bus; OFSR = 3v; TOUT = disabled; Sleep = normal; SLPOV = normal; RDAC = 0
      Wire.endTransmission();
      // put the device into run mode:
      Wire.beginTransmission(addr);
      Wire.send(1); //integration configuration register
      Wire.send(15); //SMPL = 1 sample; VSFEN = filter enabled; VBFEN = filter enabled; RIOV = Override enabled; INTEN = integrate state.
      Wire.endTransmission();
    }
    
}

void watchdog_timeout(void){
  for (int i = 0; i<6; i++) {
    digitalWriteFast(laser_TTL_pin[i],LOW); //active high IOs
  }
}