/******************************************************************************
 *
 * ORIGINAL : 
 * 
 * SparkFunCCS811.cpp
 * CCS811 Arduino library
 * 
 * Marshall Taylor @ SparkFun Electronics
 * Nathan Seidle @ SparkFun Electronics
 * 
 * April 4, 2017
 * 
 * https://github.com/sparkfun/CCS811_Air_Quality_Breakout
 * https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library
 * 
 * Resources:
 * Uses Wire.h for i2c operation
 * 
 * Development environment specifics:
 * Arduino IDE 1.8.1
 * 
 * This code is released under the [MIT License](http://opensource.org/licenses/MIT).
 * 
 * Please review the LICENSE.md file included with this example. 
 * If you have any questions or concerns with licensing, 
 * please contact techsupport@sparkfun.com.
 * 
 * Distributed as-is; no warranty is given.
 *****************************************************************************
 * 
 * Modified for Arduino CCS811 (instead of Sparkfun Combo) and Raspberry Pi
 * October 2018 Paul van Haastrecht
 * 
 * components side pin 1 - left
 * CCS811
 * Description             GPIO           PIN
 * 1 3.3V or 5V            Power           1 or 2
 * 2 3V3 output 
 * 3 GND                   GND             6 or 14 or 20 or 30
 * 4 SDA                   SDA1/i2C/GPIO2  3      *1
 * 5 SCL to                SCL1/i2C/GPIO3  5      *1
 * 6 WAKE                  GPIO 6          31     *3    
 * 7 INT                   GPIO --         --     *2 (optional)
 * 8 RESET                 GPIO 5          29     *3
 * 
 * NOTES :
 *       *1 : soft_I2C could be any pin other than the default (set in hw_init)
 *       *2 : if interrupt GPIO is not set (0), polling is performed
 *       *3 : these are default. can be overruled by user program or the CCSS811.h
 * 
 * Development environment specifics:
 * Raspberry Pi / linux Jessie release
 * 
 * Resources / dependencies:
 * BCM2835 library (http://www.airspayce.com/mikem/bcm2835/)
 * twowire library (https://github.com/paulvha/twowire)
 * 
 * *****************************************************************
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

# include "CCS811.h"
# include <math.h>

/* Store the values obtained from the sensor */
uint16_t tVOC;
uint16_t CO2;
uint32_t vrefCounts = 0;
uint32_t ntcCounts = 0;

/* calculated values */
uint32_t resistance;
double temperature;

/* used as part of p_printf() to skip colors */
bool NoColor = false;

/* global constructor for I2C (hardware of software) */ 
TwoWire TWI;

/**********************************************************
 * constructor CCS811 core (low level sensor communication)
 **********************************************************/
 
CCS811Core::CCS811Core() {
    
    /* I2C is not initialized */
    settings.hw_initialized = false ;
    
    /* set for soft i2C by default */
    settings.I2C_interface = soft_I2C;
    
    /* set default SDA/SCL */
    settings.sda = DEF_SDA;           // default SDA line for soft_I2C
    settings.scl = DEF_SCL;           // SCL GPIO for soft_I2C
    
    /* default reset GPIO */
    settings.reset_gpio = RESET_GPIO;
    
    /* default wake GPIO */
    settings.wake_gpio = WAKE_GPIO ;
    
    /* no driver messages */
    settings.drmess = false;
}

/*****************************************************
 *  perform hard-reset on CCS811
 *****************************************************/

void CCS811Core::hw_reset() {

    /* if not initialized */
    if (settings.hw_initialized == false) return;
        
    bcm2835_gpio_write(settings.reset_gpio, LOW);
    bcm2835_delay(1000);

    bcm2835_gpio_write(settings.reset_gpio, HIGH);
    bcm2835_delay(1000);
}

/*****************************************************
 * wake_up or sleep the CCS811
 ***************************************************/
CCS811Core::status CCS811Core::wake(bool action) {

    /* if not initialized */
    if (settings.hw_initialized == false) return(SENSOR_SETTING_ERROR);

    if (action == WAKE_UP)
        bcm2835_gpio_write(settings.wake_gpio, LOW);
        
    else if (action == SLEEP)
        bcm2835_gpio_write(settings.wake_gpio, HIGH);
    
    else
    {
        if (settings.drmess) 
          p_printf(RED, (char *) "incorrect wake request %d\n", action);
        return(SENSOR_INTERNAL_ERROR);
    }

    return(SENSOR_SUCCESS);
}

/**********************************************************
 * check that the CSS811 is connected to the I2C 
 *********************************************************/
CCS811Core::status CCS811Core::beginCore() {
    
    uint8_t readCheck = 0;
    CCS811Core::status returnError = SENSOR_SUCCESS;
    
    /* if not initialized */
    if (settings.hw_initialized == false) return(SENSOR_SETTING_ERROR);
        
    returnError = readRegister(CSS811_HW_ID, &readCheck);

    if( returnError == SENSOR_SUCCESS )
    {
        // get Hardware ID, MUST be 0x81
        if( readCheck != 0x81 ) returnError = SENSOR_ID_ERROR;
    }
    
    return(returnError);
}

/*********************************************************************
 * Initialized the hardware / BCM2835 on the Raspberry Pi
 * 
 * interface : 
 *  hard_I2C / 1 = hardware I2C
 *  soft_I2C / 0 = software I2C (default in CCS811)
 *********************************************************************/

CCS811Core::status CCS811Core::hw_init(bool interface, uint8_t verbose) {
    
    if (settings.hw_initialized == true) return(SENSOR_SUCCESS);
    
    /* enable driver messages in case of level 3 verbose */
    if (verbose == 3) {
        settings.drmess = true;
        TWI.setDebug(true);
    }
    
    /* initialize the I2C software */
    if (TWI.begin(interface, settings.sda,settings.scl) != TW_SUCCESS){
        if (settings.drmess) p_printf(RED, (char *) "Can't setup i2c pin!\n");
        return(SENSOR_INTERNAL_ERROR);
    }
    
    /* set baudrate */
    TWI.setClock(settings.baudrate);
    
    /* setup the IO pins */
    bcm2835_gpio_fsel(settings.reset_gpio, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(settings.wake_gpio, BCM2835_GPIO_FSEL_OUTP);

    /* indicate initialized hardware */
    settings.hw_initialized = true;

    return(SENSOR_SUCCESS);
} 

/********************************************************************
 * close Hardware correctly on the Raspberry Pi
 ********************************************************************/
void CCS811Core::hw_close() {
    
    /* if not initialized */
    if (settings.hw_initialized == false) return;
    
    TWI.close();
    
    settings.hw_initialized = false;
}

/*********************************************************************
 *  ReadRegister
 *
 *  Parameters:
 *    offset -- register to read
 *    *outputPointer -- Pass &variable (address of) to save read data to
 *********************************************************************/
CCS811Core::status CCS811Core::readRegister(uint8_t offset, uint8_t* outputPointer) {
    return(multiReadRegister(offset, outputPointer,1));
}

/**********************************************************************
 *  multiReadRegister
 *
 *  Parameters:
 *    offset -- register to read
 *    *outputPointer -- Pass &variable (base address of) to save read data to
 *    length -- number of bytes to read
 *
 *  Note:  Does not know if the target memory space is an array or not, or
 *    if there is the array is big enough.  if the variable passed is only
 *    two bytes long and 3 bytes are requested, this will over-write some
 *    other memory!
 **********************************************************************/
CCS811Core::status CCS811Core::multiReadRegister(uint8_t offset, uint8_t *outputPointer, uint8_t length){
    
    int     result, retry = 3;  // retry count
    status  returnError;

    /* if not initialized */
    if (settings.hw_initialized == false) return(SENSOR_SETTING_ERROR);
    
    /* set slave address for CSS811 */
    TWI.setSlave(settings.I2CAddress);

    while(1)
    {
        /* first write the register we want to read */
        returnError = multiWriteRegister(offset, 0,0);
      
        if( returnError != SENSOR_SUCCESS ) 
        {
            if (settings.drmess) p_printf(RED,(char *) "Error during reading register %d, code %x\n",offset, returnError);
            return(returnError);
        }
       
        /* read results */
        result = TWI.i2c_read((char *) outputPointer, length);
       
        /* if failure, then retry as long as retrycount has not been reached */
        if (result != I2C_OK) if (retry-- > 0) continue;
                
        // process result
        switch(result)
        {
            case I2C_SDA_NACK :
                if (settings.drmess) p_printf(RED, (char *) "NACK error during read\n");
                return(SENSOR_I2C_ERROR);
    
            case I2C_SCL_CLKSTR :
                if (settings.drmess) p_printf(RED,(char *)"Clock stretch error during read\n");
                return(SENSOR_I2C_ERROR);
                
            case I2C_SDA_DATA :
                if (settings.drmess) p_printf(RED,(char *)"not all data has been read\n");
                return(SENSOR_I2C_ERROR);
                
            case I2C_OK:
                return(SENSOR_SUCCESS);
                
            default:
                if (settings.drmess) p_printf(RED,(char *) "unkown return code 0x%x during read\n", result);
                return(SENSOR_I2C_ERROR);
        }
    }
}

 /********************************************************************
 *  writeRegister
 *
 *  Parameters:
 *    offset -- register to write
 *    dataToWrite -- 8 bit data to write to register
 *********************************************************************/
CCS811Core::status CCS811Core::writeRegister(uint8_t offset, uint8_t dataToWrite) {
    return(multiWriteRegister(offset, &dataToWrite, 1)); 
}

/**********************************************************************
 *  multiWriteRegister
 *
 *  Parameters:
 *    offset -- register to write
 *    *inputPointer -- Pass &variable (base address of) to save read data to
 *    length -- number of bytes to write
 *
 * Note:  Does not know if the target memory space is an array or not, or
 *    if there is the array is big enough.  if the variable passed is only
 *    two bytes long and 3 bytes are requested, this will over-write some
 *    other memory!
 *
 * Note: the CCS811 was very unstable on the raspberryPi. Often resulting in
 *     NACK errors. To improve stability a retrycount has been implemented
 *
 *********************************************************************/
CCS811Core::status CCS811Core::multiWriteRegister(uint8_t offset, uint8_t *inputPointer, uint8_t length){
    
    int     i;
    char    buff[10];
    int     retry=3;
    status  result = SENSOR_SUCCESS;
    
    if (length > 9)
    {
        if (settings.drmess) p_printf(RED, (char *) "can handle up to 9 registers. Request is for %d\n", length);
        return(SENSOR_INTERNAL_ERROR);
    }
    
    /* if not initialized */
    if (settings.hw_initialized == false) return(SENSOR_SETTING_ERROR);
    
    /* set register */
    buff[0]=offset;
    
    /* copy data */
    for (i = 0 ; i < length; i++)  buff[i+1] = inputPointer[i];
    
    /* set slave address for CSS811 */
    TWI.setSlave(settings.I2CAddress);
    
    while (1)
    {
        /* perform a write of data
         * if error, perform retry (if not exceeded) */
        if (TWI.i2c_write(buff, length+1) != I2C_OK)
        {
            if (settings.drmess) p_printf(YELLOW, (char *) " retrying %d\n", result);
            if (retry-- > 0) continue;
            result = SENSOR_I2C_ERROR;
        }
        
        return(result);
     }
}

/*************************************************************
 *  Main user class -- wrapper for the core class + maths
 *
 *  Construct with same rules as the core
 *************************************************************/
 
CCS811::CCS811() : CCS811Core() {
    
    settings.refResistance = 100000;    // 100K on board resistor
    settings.tempOffset = 0;            // no compensation
}

/*************************************************************
 *  Begin
 *
 *  This starts the lower level begin, then applies settings
 *************************************************************/

CCS811Core::status CCS811::begin() {
    
    CCS811Core::status returnError = SENSOR_SUCCESS;    //Default return state
    uint8_t value;
    int retry = 5;
    
    /* hw_init should have been called before... but just in case */
    if (settings.hw_initialized == false)
    {
         returnError = hw_init(soft_I2C,0);
         if( returnError == SENSOR_SUCCESS ) return(returnError);
    }

    /* reset CCS811values */
    resistance = 0;
    temperature = 0;
    tVOC = 0;
    CO2 = 0;
    
    /* try to read Hardware ID */
    while (1)
    {
        // wake up the CCS811
        returnError = wake(WAKE_UP);
        if( returnError != SENSOR_SUCCESS ) return(returnError);
   
        bcm2835_delay(200);

        // check for hardware_ID
        returnError = beginCore();

        if( returnError == SENSOR_SUCCESS ) break;
        
        if (retry-- == 0)  return(returnError);
        else
        {
            hw_reset();
            
            returnError = wake(SLEEP);
            if( returnError != SENSOR_SUCCESS ) return(returnError);
            
            bcm2835_delay(500);             
        }
     }
    
    /* perform soft reset */
    if( soft_reset() != SENSOR_SUCCESS ) return (SENSOR_INTERNAL_ERROR);
    
    /* check  for NO error */
    if( checkForStatusError() == true ) 
    {
        printf(" Status error found \n");
        return(SENSOR_INTERNAL_ERROR);
    }

    /* check that application is available */
    if (appValid() == false ) 
    {
        printf("error application validation\n");
        return(SENSOR_INTERNAL_ERROR);
    }
    
    /* start application */
    returnError  = multiWriteRegister(CSS811_APP_START, 0,0);
    if (returnError != SENSOR_SUCCESS)  return(returnError);
    
    /* check that application has started */
    retry = 5;
    while(1)
    {
        bcm2835_delay(100); 
        
        returnError = readRegister(CSS811_STATUS, &value);
        
        if (returnError != SENSOR_SUCCESS)  return(returnError);
        
        // application started and status register can be read correctly
        if (value == 0x90) break;
        
        if (retry-- == 0)
        {
            if (settings.drmess) p_printf(RED, (char*) "can not start application\n");
            return(SENSOR_INTERNAL_ERROR);          
        }
    }
    
    /* check for NO error */
    if( checkForStatusError() == true ) 
    {
        if (settings.drmess) p_printf(RED, (char*) "Status error found after start application\n");
        return(SENSOR_INTERNAL_ERROR);
    }
       
    /* set drive mode */
    if (setDriveMode(settings.mode) != SENSOR_SUCCESS)
    {
        if (settings.drmess) if (settings.drmess) p_printf(RED, (char*) "can not set drive mode to %d\n",settings.mode);
        return(SENSOR_INTERNAL_ERROR);
    }

    /* set baseline (if provided) */
    if (settings.baseline != 0)
    {
        if (setBaseline(settings.baseline) != SENSOR_SUCCESS)
        {
            if (settings.drmess) p_printf(RED, (char*) "can not set baseline to %lx\n",settings.baseline);
            return(SENSOR_INTERNAL_ERROR);
        }
    }
    
    /* enable interrupts in the CCS811 if needed */
    if (settings.interrupt_gpio > 0) enableInterrupts();

    /* there for NO error */
    if( checkForStatusError() == true ) 
    {
        if (settings.drmess) p_printf(RED, (char*) " Status error found after enabling interrupts \n");
        return(SENSOR_INTERNAL_ERROR);
    }
    
    return(returnError);
}

/*********************************************************************
 *   soft_reset on CCS811
 ********************************************************************/
CCS811Core::status CCS811::soft_reset() {
    
    uint8_t data[4] = {0x11,0xE5,0x72,0x8A};            //software Reset key

    /* Reset the device */
    CCS811Core::status returnError = multiWriteRegister(CSS811_SW_RESET, data, 4);
    bcm2835_delay(20);
    
    return(returnError);
}

/**********************************************************************
 * Sensor functions
 * Updates the total voltatile organic compounds (TVOC) in 
 * parts per billion (PPB) and the CO2 value
 * Returns no reading
 *
 ********************************************************************/
CCS811Core::status CCS811::readAlgorithmResults(){
    
    uint8_t data[4];
    
    CCS811Core::status returnError = multiReadRegister(CSS811_ALG_RESULT_DATA, data, 4);
    
    if( returnError != SENSOR_SUCCESS ) return(returnError);
    
    // Data order:  co2MSB, co2LSB, tvocMSB, tvocLSB
    // printf("%s   1 : %x 2: %x 3 %x 4 %x\n",__func__, data[0], data[1], data[2], data[3]);
    CO2 = ((uint16_t)data[0] << 8) | data[1];
    tVOC = ((uint16_t)data[2] << 8) | data[3];
    
    /* during testing on the raspberry Pi it did happen that due to 
     * clock stretch issues with hardware BCM2835 I2C, the first bit 
     * of the CO2MSB is read as 1. 
     * As the CCS811 works within 400 ppm and 8192 PPM anything above 
     * 0x2000 (8192) is filtered out 
     * 
     * ACTUALLY the CCS811 should only be run with soft_I2C (although 
     * hard_I2C with 35Khz works most of the time */
     CO2 = CO2 & 0x2fff;
    
    return(SENSOR_SUCCESS);
}

/*********************************************************************
 *  Checks to see if error bit is set 
 ********************************************************************/
bool CCS811::checkForStatusError() {

    uint8_t value;
    
    /* return the status bit */
    CCS811Core::status returnError = readRegister(CSS811_STATUS, &value );

    if( returnError != SENSOR_SUCCESS ) return(1);

    /* return databit 0 */
    return (value & 1 << 0);
}

/********************************************************************
 *  Checks to see if DATA_READ flag is set in the status register 
 ********************************************************************/
bool CCS811::dataAvailable() {
    
    uint8_t value;
    CCS811Core::status returnError = readRegister(CSS811_STATUS, &value );
  
    if( returnError != SENSOR_SUCCESS ) 
        return 0;
    else
    {   // 3rd bit in status
        return (value & 1 << 3);
    }
}

/********************************************************************
 * Checks to see if APP_VALID flag is set in the status register 
 * making sure there is an application program to execute 
 ********************************************************************/
bool CCS811::appValid() {
    
    uint8_t value;
    CCS811Core::status returnError = readRegister( CSS811_STATUS, &value );
    
    if( returnError != SENSOR_SUCCESS ) 
        return 0;
    else
        return (value & 1 << 4);
}

/*********************************************************************
 * read error register
 *********************************************************************/
uint8_t CCS811::getErrorRegister() {
    uint8_t value;
    
    CCS811Core::status returnError = readRegister( CSS811_ERROR_ID, &value );
    
    if( returnError != SENSOR_SUCCESS )  return(0xFF);
    else
        return value;  //Send all errors in the event of communication error
}

/*********************************************************************
 * read version registers
 *
 * return as :
 * version[0] = HW_version
 * version[1] = Firmware bootloader major / minor
 * version[2] = Firmware bootloader trivial
 * version[3] = Firmware application major / minor
 * version[4] = Firmware application trivial
 *********************************************************************/
CCS811Core::status CCS811::getversionregisters( uint8_t *version ) {
    
    CCS811Core::status returnError;
    
    returnError = readRegister( CSS811_HW_VERSION, &version[0]);
    if( returnError != SENSOR_SUCCESS )   return(SENSOR_I2C_ERROR);

    returnError = multiReadRegister( CSS811_FW_BOOT_VERSION, &version[1],2);
    if( returnError != SENSOR_SUCCESS )   return(SENSOR_I2C_ERROR);
    
    returnError = multiReadRegister( CSS811_FW_APP_VERSION, &version[3],2);
    if( returnError != SENSOR_SUCCESS )   return(SENSOR_I2C_ERROR);
    
    return(SENSOR_SUCCESS);
}

/******************************************************************
 * Returns the baseline value
 * Used for telling sensor what 'clean' air is
 * You must put the sensor in clean air and record this value
 *******************************************************************/
uint16_t CCS811::getBaseline() {
    
    uint8_t data[2];

    CCS811Core::status returnError = multiReadRegister(CSS811_BASELINE, data, 2);

    if( returnError != SENSOR_SUCCESS ) return SENSOR_I2C_ERROR;
    else
    {
        uint16_t  baseline = ((uint16_t)data[0] << 8) | data[1];
        return (baseline);
    }
}

/******************************************************************
 * set the baseline value
 * Used for telling sensor what 'clean' air is
 *******************************************************************/
CCS811Core::status CCS811::setBaseline(uint16_t input) {
    
    uint8_t data[2];
    data[0] = (input >> 8) & 0x00FF;
    data[1] = input & 0x00FF;
    
    return(multiWriteRegister(CSS811_BASELINE, data, 2));
}

/********************************************************************
 * Enable or disable the nINT signal
 ********************************************************************/
CCS811Core::status CCS811::enableInterrupts() {
    
    uint8_t value;
    
    //Read what's currently there
    CCS811Core::status returnError = readRegister( CSS811_MEAS_MODE, &value ); 
    if(returnError != SENSOR_SUCCESS) return returnError;
        
    value |= (1 << 3); //Set INTERRUPT bit
    
    return(writeRegister(CSS811_MEAS_MODE, value));
}

/*! Disable the nINT signal */
CCS811Core::status CCS811::disableInterrupts() {
    
    uint8_t value;
    CCS811Core::status returnError = readRegister( CSS811_MEAS_MODE, &value ); //Read what's currently there
    if( returnError != SENSOR_SUCCESS ) return(returnError);
    
    value &= ~(1 << 3); //Clear INTERRUPT bit
    
    return(writeRegister(CSS811_MEAS_MODE, value));
}

/********************************************************************
 * Mode 0 = Idle
 * Mode 1 = read every 1s
 * Mode 2 = every 10s
 * Mode 3 = every 60s
 * Mode 4 = RAW mode
 ********************************************************************/
CCS811Core::status CCS811::setDriveMode(uint8_t mode) {
    
    if (mode > 4) mode = 4; //sanitize input
 
    uint8_t value;
    CCS811Core::status returnError = readRegister(CSS811_MEAS_MODE, &value ); //Read what's currently there
    if( returnError != SENSOR_SUCCESS ) return returnError;

    value &= ~(0b00001100 << 4); // Clear DRIVE_MODE bits
    value |= (mode << 4);        // Mask in mode
  
    returnError = writeRegister(CSS811_MEAS_MODE, value);
    if( returnError != SENSOR_SUCCESS ) return returnError;
  
    settings.mode = mode;        // save new mode
    
    return(SENSOR_SUCCESS);
}

/**********************************************************************
 * Given a temp and humidity, write this data to the CSS811 for better compensation
 * This function expects the humidity and temp to come in as floats 
 **********************************************************************/
CCS811Core::status CCS811::setEnvironmentalData( float relativeHumidity, float temperature ){

    /* Check for invalid temperatures */
    if((temperature < -25)||(temperature > 50)) return SENSOR_GENERIC_ERROR;
    
    /* Check for invalid humidity */
    if((relativeHumidity < 0)||(relativeHumidity > 100)) return SENSOR_GENERIC_ERROR;
    
    uint32_t rH = relativeHumidity * 1000; //42.348 becomes 42348
    uint32_t temp = temperature * 1000;    //23.2 becomes 23200

    char envData[4];

    /* Split value into 7-bit integer and 9-bit fractional */
    envData[0] = ((rH % 1000) / 100) > 7 ? (rH / 1000 + 1) << 1 : (rH / 1000) << 1;
    envData[1] = 0; //CCS811 only supports increments of 0.5 so bits 7-0 will always be zero
    
    if (((rH % 1000) / 100) > 2 && (((rH % 1000) / 100) < 8))
    {
        envData[0] |= 1; //Set 9th bit of fractional to indicate 0.5%
    }

    temp += 25000; //Add the 25C offset:  0 masp to -25C
    
    /* Split value into 7-bit integer and 9-bit fractional */
    envData[2] = ((temp % 1000) / 100) > 7 ? (temp / 1000 + 1) << 1 : (temp / 1000) << 1;
    envData[3] = 0;
    if (((temp % 1000) / 100) > 2 && (((temp % 1000) / 100) < 8))
    {
        envData[2] |= 1;  //Set 9th bit of fractional to indicate 0.5C
    }

    return(multiWriteRegister(CSS811_ENV_DATA, (uint8_t *) envData, 4));

}

/*********************************************************************
 * overrule the reference resistor value (100K is default) to support 
 * Temperature / NTC readings
 * 
 * On the Arduino CCS811 the NTC is 10K, while the reference resistor 
 * is 100K (some internet pages claim it to be 10K as well)
 *********************************************************************/
void CCS811::setRefResistance(uint32_t input) {
    settings.refResistance = input;
}

/********************************************************************
 * allow for compensation of in-circuit measurements. Often the NTC is 
 * warmer than air temp, due to CCS811 heating and other components. 
 * The offset is SUBTRACTED in readNTC() from the temperature reading 
 ********************************************************************/
void CCS811::setTemperatureOffset (uint32_t offset) {
    settings.tempOffset = offset;
}

/********************************************************************
 *  get the NTC resistance, just in case you are interested 
 ********************************************************************/
uint32_t CCS811::getResistance() {
    return(resistance);
}

/********************************************************************
 *  obtain the temperature calculated with readNTC() 
 *******************************************************************/
double CCS811::getTemperature() {
    return(temperature);
}

/*******************************************************************
 * read the NTC values and calculate the temperature 
 * An offset could have been provided (setTemperatureOffset). This value
 * will be subtracted from the calculated temperature to compensate for
 * to high reading caused by heat coming from components on the board.
 * 
 * The resulted temperature can be obtained with a getTemperature() call
 ********************************************************************/
CCS811Core::status CCS811::readNTC() {
    
    uint8_t data[4];
    
    CCS811Core::status returnError = multiReadRegister(CSS811_NTC, data, 4);
    if( returnError != SENSOR_SUCCESS ) return (returnError);
    
    vrefCounts = ((uint32_t) data[0] << 8) | data[1];
    ntcCounts = ((uint32_t) data[2] << 8) | data[3];

    /*! based on the ccs811 Thermistor interface AN000372.pdf
     * 12 feb 2016 Cambridge Cmos Sensors limited */

    resistance = (ntcCounts * settings.refResistance / vrefCounts);

    temperature = log ((double) resistance / 10000); //1
    temperature /= 3380;             //2
    temperature += 1 / (25 +273.15); //3
    temperature = 1 / temperature;   //4
    temperature -= 273.15 ;          //5

    /* compensate for higher readings due to warm components  */
    temperature -= settings.tempOffset;
    
    return returnError;
}

/****************************************************************
 * get I2C statistics for clock Stretch
 ****************************************************************/
void CCS811::getClckStat(clock_stretch *stat) {
    TWI.GetStretchStat(stat);
}

/****************************************************************
 * clear I2C statistics for clock Stretch
 ****************************************************************/
void CCS811::ClrClckStat() {
    TWI.ClrStretchStat();
}

/****************************************
 * Display the clock stretch info
 *****************************************/
void CCS811::DispClockStretch() {
    TWI.DispClockStretch();
}

/********************************************************************
 * Obtain values that have been set with readAlgorithmResults())
 ********************************************************************/
uint16_t CCS811::getTVOC(){
    return(tVOC);
}

uint16_t CCS811::getCO2() {
    return(CO2);
}

/*********************************************************************
 *  Display in color
 * @param format : Message to display and optional arguments
 *                 same as printf
 * @param level :  1 = RED, 2 = GREEN, 3 = YELLOW 4 = BLUE 5 = WHITE
 * 
 *  if NoColor was set, output is always WHITE.
 *********************************************************************/
void p_printf(int level, char *format, ...) {
    
    char    *col;
    int     coll=level;
    va_list arg;
    
    //allocate memory
    col = (char *) malloc(strlen(format) + 20);
    
    if (NoColor) coll = WHITE;
                
    switch(coll)
    {
    case RED:
        sprintf(col,REDSTR, format);
        break;
    case GREEN:
        sprintf(col,GRNSTR, format);
        break;      
    case YELLOW:
        sprintf(col,YLWSTR, format);
        break;      
    case BLUE:
        sprintf(col,BLUSTR, format);
        break;
    default:
        sprintf(col,"%s",format);
    }

    va_start (arg, format);
    vfprintf (stdout, col, arg);
    va_end (arg);

    fflush(stdout);

    // release memory
    free(col);
}
