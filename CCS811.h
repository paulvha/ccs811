/*********************************************************************
 *
 * ORIGINAL version: 
 *
 * SparkFunCCS811.h
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
 * If you have any questions or concerns with licensing, please 
 * contact techsupport@sparkfun.com.
 * 
 * Distributed as-is; no warranty is given.
 * ********************************************************************
 * Modified, enhanced and extended for the Raspberry Pi
 * October 2018 Paul van Haastrecht
 * 
 * Resources:
 * 
 * BCM2835 library for i2C (http://www.airspayce.com/mikem/bcm2835/)
 * twowire library (https://github.com/paulvha/twowire)
 *
 * Development environment specifics:
 * Raspberry Pi / linux Jessie release
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
 *********************************************************************/


#ifndef __CCS811_H__
#define __CCS811_H__

# include <bcm2835.h>
# include <getopt.h>
# include <signal.h>
# include <stdint.h>
# include <stdarg.h>
# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include <unistd.h>
# include <time.h>
# include <twowire.h>

//Register addresses
#define CSS811_STATUS           0x00
#define CSS811_MEAS_MODE        0x01
#define CSS811_ALG_RESULT_DATA  0x02
#define CSS811_RAW_DATA         0x03
#define CSS811_ENV_DATA         0x05
#define CSS811_NTC              0x06
#define CSS811_THRESHOLDS       0x10
#define CSS811_BASELINE         0x11
#define CSS811_HW_ID            0x20
#define CSS811_HW_VERSION       0x21
#define CSS811_FW_BOOT_VERSION  0x23
#define CSS811_FW_APP_VERSION   0x24
#define CSS811_ERROR_ID         0xE0
#define CSS811_APP_START        0xF4
#define CSS811_SW_RESET         0xFF


// define wake_up actions
#define SLEEP   0
#define WAKE_UP 1

#define SUCCESS 0
#define ERROR   1

/*! default GPIO for SOFT_I2C */
#define DEF_SDA 2
#define DEF_SCL 3

#define  CCS811_ADDR        0x5a    // Default i2C Address
#define  INT_GPIO           0       // 0 = disabled / not using interrupt
#define  WAKE_GPIO          6       // GPIO wake
#define  RESET_GPIO         5       // GPIO hard reset
#define  CCS_MODE           2       // default every 10 seconds
#define  CONDITION_PERIOD   20      // default 20 min
#define  MAXBUF             200     // maximum different buffers

struct ccs811_p
{
  public:

    /*! driver information */
    uint8_t     reset_gpio;         // GPIO reset
    uint8_t     wake_gpio;          // GPIO Wake
    uint8_t     interrupt_gpio;     // GPIO INT (optional)
    bool         hw_initialized;     // initialized or not
    bool         drmess;         // show driver messages (true) 
    
    /*! I2C  information */
    bool         I2C_interface;      // hard_I2C or soft_I2C
    uint8_t     I2CAddress;         // slave address
    uint16_t    baudrate;           // speed
    int          sda;                // SDA GPIO (soft_I2C only)
    int          scl;                // SCL GPIO (soft_I2C only)
    
    /*! CCS811 information */
    uint8_t     mode;               // mode 1 - 4
    uint8_t     conditionPeriod;    // warmup period
    uint16_t    baseline;           // CCS811 baseline
    uint32_t    refResistance;      // temperature reference
    uint32_t    tempOffset;         // temperature reading offset
    
};

/***************************************************************
 *  This is the core operational class of the driver.
 *  CCS811Core contains only read and write operations towards the sensor.\
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *  !!!!!!!! DO NOT Use from user program                   !!!!!!  
 *  !!!!!!!! use the class CCS811 which inherits this class.!!!!!!
 ***************************************************************/
class CCS811Core
{
public:
    /* Return values */
    typedef enum
    {
        SENSOR_SUCCESS,
        SENSOR_ID_ERROR,
        SENSOR_I2C_ERROR,
        SENSOR_INTERNAL_ERROR,
        SENSOR_GENERIC_ERROR,
        SENSOR_SETTING_ERROR
        //...
    } status;

    /* structure with values */
    ccs811_p settings;

    /* constructor */
    CCS811Core( void );
    
    /* */
    status beginCore( void );
    
    /*! ROUTINES FOR i2C HANDLING ON RASPBERRY PI */
    
    /* start initialize */
    status hw_init(bool interface, uint8_t verbose);  
    
    /* close down correctly */
    void hw_close();
    
    /* perform a reset */
    void hw_reset(void);
    
    /* set wake status */
    status wake(bool action);
      
    /*! Reading functions */
    
    /* readRegister reads one 8-bit register */
    status readRegister( uint8_t offset, uint8_t* outputPointer);
    
    /* multiReadRegister takes a uint8 array address as input and performs
     * a number of consecutive reads */
    status multiReadRegister(uint8_t offset, uint8_t *outputPointer, uint8_t length);

    /***Writing functions***/
    
    /* Writes an 8-bit byte; */
    status writeRegister(uint8_t offset, uint8_t dataToWrite);
    
    /* multiWriteRegister takes a uint8 array address as input and performs
     * a number of consecutive writes*/
    status multiWriteRegister(uint8_t offset, uint8_t *inputPointer, uint8_t length);

protected:
 
};

/********************************************************************
 * This is the highest level class of the driver.
 *
 * class CCS811 inherits the CCS811Core and makes use of the beginCore()
 * method through its own begin() method.  It also contains user settings/values.
 ********************************************************************/
class CCS811 : public CCS811Core 
{
public:
    /* constructor */
    CCS811(void);
    
    /* Call to check for errors, start app, and set default mode */
    status begin( void );
    
    /* sent a soft reset sequence to the CCS811 */
    status soft_reset( void );
    
    /* get the hardware and software version information */
    status getversionregisters( uint8_t *version );
    
    /* trigger to obtain the results from the CCS811 */
    status readAlgorithmResults( void );
    
    /* get the values from the recent readAlgorithmResults */
    uint16_t getTVOC( void );
    uint16_t getCO2( void );    
    
    /* check for error on the CCS811 */
    bool checkForStatusError( void );

    /* obtain the error information details from the CCS811 */
    uint8_t getErrorRegister( void );
        
    /* check that new results are available from the CCS811 */
    bool dataAvailable( void );
    
    /* check whether an application is available on the CCS811 */
    bool appValid( void );
    
    /* get or set the calculated baseline value */
    uint16_t getBaseline( void );
    status setBaseline( uint16_t );
    
    /* enable interrupts from the CCS811 */
    status enableInterrupts( void );
    
    /* disable interrupts from the CCS811 */
    status disableInterrupts( void );
 
    /* set operation mode */
    status setDriveMode( uint8_t mode );
 
    /* performs temperature and humidity compensation */
    status setEnvironmentalData( float relativeHumidity, float temperature );
    
    /*! handle the temperature measurement options */
    
    /* set reference resistor ( default 100K) */
    void setRefResistance( uint32_t  );
 
    /* read values and calculate temperature */
    status readNTC( void );
    
    /* subtract the offset due to higher measurement than
     * real temperature caused by warmer components */ 
    void setTemperatureOffset (uint32_t offset);
    
    /* get NTC resistance (nice to know... ) */
    uint32_t getResistance( void );
    
    /* get the temperature (adjusted with offset) */
    double getTemperature( void );
        
    /*! clockstretch statistics */
    
    /* get I2C clock stretch statistics */ 
    void getClckStat( clock_stretch *stat );
    
    /* clear I2C clock stretch statistics */
    void ClrClckStat();
    
    /* display clockstretch statistics */
    void DispClockStretch();

private:

};

/** display in color  */
void p_printf (int level, char *format, ...);

/* color display colors */
#define RED     1
#define GREEN   2
#define YELLOW  3
#define BLUE    4
#define WHITE   5

#define REDSTR "\e[1;31m%s\e[00m"
#define GRNSTR "\e[1;92m%s\e[00m"
#define YLWSTR "\e[1;93m%s\e[00m"
#define BLUSTR "\e[1;34m%s\e[00m"

/* disable color output */
extern bool NoColor;

#endif  // End of definition check
