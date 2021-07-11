# iC-WKM based laser driver
 A SCPI based system for controlling a laser driver using the iC-Haus WKM drive IC. This software is compatible with PCB 210609 rev1.

# How to set up a module:
1. Connect USB power to the board and connect via PUTTY (or other serial terminal).
2. Use the serial command ***IDN?** To check the device you are connecting to is the correct one. The reply should be a 4-element CSV string. *"OptoTech,AFC.10.07.2021,LaserDriver,1-channel"*
3. Use the serial command **SYST:I2C?** to get a list of the connected I2C bus devices. There will always be one devices 0x18, which is the PAC current sense IC. If this is not present the board is defective (sensor does not respond on the bus).
4. Set the PAC address using the serial command **LASER:PACADDR 1,24** where 24 is the decimal value of the PAC sensors address (ie 0x18 = 24d)
5. Set the serial number of the laser modules using the serial command **LASER:SERIAL 1,XXXXXX** where XXXXXX is the serial number of the laser module
6. Set the wavelength of the laser modules using the serial command **LASER:WAVELENGTH 1,XXX** where XXX is the wavelength of the laser module in nm
7. Set the max power of the of the laser modules using the serial command **LASER:PMAX 1,XXXXXX** where XXXXXX is the maximum power of the laser module in mW.
8. Save the setting using the serial command ***SAV** and observe the response of *"-5"*. If any other number is returned the board has a defective memory module. 
9. Restart the devide to apply the new settings using the serial command ***RST** or by power cycling the unit.
10. Test the features:
- Temperature sensor can be tested by using the serial command **SYST:TEMP?** twice. The first time will return *"-0.0625"* and the second time should return the room temperature in degrees Celcius. 
- Test the voltage supply measurement function using the serial command **LASER:VOLT? 1** which should read voltage supplied to the iC-WKM driver.
- Test the driver current monitor functin using the serial command **LASER:CURRENT? 1** which should return ~5mA when the laser is in the off state (default).
- Test the laser enable signal **LASER:STAT 1,x** where x is either 0 for off or 1 for on. To test this, you will need to set the laser power to mid-range using the command **LASER:POWER 1,128**. Then toggle the laser on/off to observe the indicator LED.
- Test the ability to set the power by trying **LAS:POWER 1,xxx** with different values of x between 0 and 254. Use the **LASER:CURRENT? 1** command to read the current and observe the current changing with the power level.