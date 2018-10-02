/******************************************************************************
 *  some information is based on BasicReadings.ino
 *
 *  Marshall Taylor @ SparkFun Electronics
 *  Nathan Seidle @ SparkFun Electronics
 *
 *  April 4, 2017
 *
 *  https://github.com/sparkfun/CCS811_Air_Quality_Breakout
 *  https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library
 *
 *  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
 *
 *  Please review the LICENSE.md file included with this example. If you have any questions
 *  or concerns with licensing, please contact techsupport@sparkfun.com.
 *
 *  Distributed as-is; no warranty is given.  
 * 
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
#include "CCS811.h"

#define  VERSION 2

/* indicate these are C-programs and not to be linked */
extern "C" {
    void close_dylos();
    int read_dylos (char * buf, int len, int wait, int verbose);
    int open_dylos(char * device, int verbose);
}

typedef struct dylos
{
    char     port[MAXBUF];  // connected port (like /dev/ttyUSB0)
    bool     include;        // 1 = include
    uint16_t value_pm10;      // measured value PM10 DC1700
    uint16_t value_pm1;       // Measured value PM1  DC1700
} dylos;

/* holds all the measurement directions and values */
typedef struct measure
{
    uint8_t     verbose;        // extra information
    uint16_t    loop;           // # of measurement loops
    uint16_t    loop_delay;     // force loop delay (sec)
    char        format[MAXBUF]; // output format
    time_t      b_time_start;    // last baseline save
    char        b_save_file[MAXBUF];   // baseline savefile
    char        v_save_file[MAXBUF];   // value savefule
    uint16_t    TVOC;           // TVOC value
    uint16_t    CO2;            // CO2 value
    double      temperature;    // hold the temperature

    float       env_tempC;       // temperature for environment compensation
    float       env_humid;       // humidity for enviroment compensation
    struct dylos dylos;
} measure ;

int parameter_from_file(char * file, struct measure *mm);

/* global constructers */ 
CCS811 MySensor;

/* PID for child process handling interrupts */
pid_t ch_pid =1;

/* indicate whether any data is pending in CCS811 */
int data_ready = 0;

/*file name to save all settings */
char save_file[MAXBUF];

char progname[20];

/*********************************
*  generate timestamp
*
* @param buf : buffer to write
**********************************/  
void time_stamp(char *buf)
{
    time_t ltime;
    struct tm *tm ;
    
    ltime = time(NULL);
    tm = localtime(&ltime);
    
    static const char wday_name[][4] = {
    "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
    
    static const char mon_name[][4] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

    sprintf(buf, "%.3s %.3s%3d %.2d:%.2d:%.2d %d",
    wday_name[tm->tm_wday],  mon_name[tm->tm_mon],
    tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec,
    1900 + tm->tm_year);
}

/******************************************
*  close hardware and program correctly
*******************************************/
void closeout()
{
    /* request for closure child if created for interrupt handling */
    if (ch_pid > 1)    kill (ch_pid,SIGHUP);

    /* stop CSS811 sensor */
    MySensor.soft_reset();
    MySensor.setDriveMode(0); 
    MySensor.wake(SLEEP);
          
    /* close dylos */
    close_dylos();
     
    /* reset pins in Raspberry Pi */
    MySensor.hw_close();
    
    exit(EXIT_SUCCESS);
}

/*************************************************
* catch signal from child that data is available
* 
* @param sign_num : signal number received
**************************************************/
void handle_interrupt(int sig_num)
{
    /* check that child was running */
    if (ch_pid == 1 )  return;
    
    /* if this is child : this is request to exit */
    if (ch_pid == 0)
    {
        printf("closing interrupt child\n");
        exit(0);
    }
    
    /* this is executed by parent as child raised the interrupt
     * check for data available / trigger main_loop
     */
    
    if (MySensor.dataAvailable()) data_ready = 2;
}

/*****************************************
** catch signals to close out correctly
* 
* @param sign_num : signal number received 
******************************************/
void signal_handler(int sig_num)
{
    switch(sig_num)
    {
        
        case SIGINT:
        case SIGKILL:
        case SIGABRT:
        case SIGTERM:
        default:
            if (ch_pid == 0)
            {
                printf("stopping interrupt handler\n");
                exit(0);
            }
            printf("\nStopping CCS811 monitor\n");
            closeout();
            break;
    }
}

/**********************************
 * setup signals 
 * 
 * @param sign_num : signal number received
 **********************************/
void set_signals()
{
    struct sigaction act;
    
    memset(&act, 0x0,sizeof(act));
    act.sa_handler = &signal_handler;
    sigemptyset(&act.sa_mask);
    
    sigaction(SIGTERM,&act, NULL);
    sigaction(SIGINT,&act, NULL);
    sigaction(SIGABRT,&act, NULL);
    sigaction(SIGSEGV,&act, NULL);
    sigaction(SIGKILL,&act, NULL);
}

/*********************************************
 * set_child_signal for CCS811 interupt
 * 
 * @param action : 1 = set, 0 = to default 
 *********************************************/
void set_child_signal(bool action)
{
    struct sigaction act;
    
    memset(&act, 0x0,sizeof(act));
    
    if (action) act.sa_handler = &handle_interrupt;
    else act.sa_handler = SIG_DFL;
    
    sigemptyset(&act.sa_mask);

    sigaction(SIGHUP,&act, NULL);
}

/**********************************************
 *  usage information  
 **********************************************/

void usage()
{
    p_printf(YELLOW, (char *)
    "%s [options] \n\n"
    
    "CCS811: \n\n"
    "-a #       i2C address              (default 0x%02x)\n"
    "-r #       reset GPIO               (default %d)\n"
    "-w #       wake GPIO                (default %d)\n"
    "-i #       interrupt GPIO           (default %d)\n"
    "-m #       mode (1-4)               (default %d)\n"
    "-b #       set baseline from commandline\n"
    "-S file    save baseline to file\n"
    "-G file    get baseline from file\n"
    "-o #       offset temperature       (default %d)\n"
    "-t #       compensate for temperature\n"
    "-u #       compensate for humidity\n"

    "\nDylos DC1700: \n\n"
    "-D port    Enable Dylos input from port\n"
    
    "\nGeneral: \n\n"
    "-B         no colored output\n"
    "-C #       conditioning period      (default %d min)\n"
    
    "-L #       loop count               (default 0 = endless)\n"
    "-U #       Loop delay (default depending on CCS811 mode\n"
    "-O string  output format string\n"
    "-V #       verbose level (1 - 3)\n"
    "-W file    save formatted output to file\n"
    
    "\nI2C settings: \n\n"
    "-H         set for hardware I2c\n"
    "-I #       i2C speed 1 - 400Khz     (default %d Khz)\n"
    "-s #       SOFT I2C GPIO # for SDA  (default GPIO %d)\n"
    "-d #       SOFT I2C GPIO # for SCL  (default GPIO %d)\n"  
    
    "\nSpecial: \n\n"
    "-E file    save all settings to file\n"
    "-K file    load all settings from file\n"
    ,progname,CCS811_ADDR ,RESET_GPIO,WAKE_GPIO,INT_GPIO,CCS_MODE,
    (int) MySensor.settings.tempOffset, CONDITION_PERIOD, 
    MySensor.settings.baudrate, MySensor.settings.sda, MySensor.settings.scl);
}

/**********************************************************
 * read baseline from file 
 * 
 * There should be a baseline for each mode. This routine
 * will provide a warning if not the same
 * 
 * @param file : baseline input file
 * @param mm : measurement structure
 **********************************************************/
void get_baseline(char * file, struct measure *mm)
{
    FILE    *fp = NULL ;
    char    line[MAXBUF], *p;
    uint8_t  mode =0;
    
    /* open inputfile for reading */
    fp = fopen (file, "r");
   
    /* Checks if file is empty or none existing */
    if( fp == NULL ) { 
        p_printf(RED,(char *) "Issue with opening/reading baseline file : %s\n", file);                      
        exit(EXIT_FAILURE);
    }

    /* read line by line */
    while(fgets(line,MAXBUF,fp))
    {
        p = strstr(line,"BASELINE=");
        
        if (p != NULL)
        {
            p += strlen("BASELINE=");
            MySensor.settings.baseline = (uint16_t) strtod(p,NULL);
            break;
        }
        
        p=strstr(line,"MODE=");
        
        if (p != NULL)
        {
            p += strlen("MODE=");
            mode = (uint8_t) strtod(p,NULL);   
            
            if (mode != MySensor.settings.mode)
            {
                p_printf(RED, (char *) "WARNING: This baseline is for CCS811 mode %d. Current setting is %d\n",
                mode, MySensor.settings.mode);
            }
         }
    }
    
    fclose(fp);
      
    if (MySensor.settings.baseline == 0)
    {
        p_printf(RED,(char *) "Could not get baseline from file %s\n", file);
        exit(EXIT_FAILURE);
    }
    else if (mode != 0)
    {
        if (mm->verbose) printf("baseline for mode %d will be set to %d\n"
        ,mode, MySensor.settings.baseline);
    }
    else
    {
        if (mm->verbose) printf("baseline will be set to %d\n", MySensor.settings.baseline);
    }       
}

/*******************************************************************
 * Validate whether a valid GPIO has been provided on the commandline 
 * and check it is not in use already
 *******************************************************************/
 int check_gpio()
{
   if (MySensor.settings.wake_gpio < 2 || MySensor.settings.wake_gpio > 27 ||
   MySensor.settings.reset_gpio < 2 || MySensor.settings.reset_gpio > 27 )
   {
       p_printf(RED,(char *) "Invalid GPIO. Must be between 2 and 27\n");
       return(ERROR);
   }
      
  /* if interrupt GPIO = 0, it is disabled */  
  if ( MySensor.settings.interrupt_gpio != 0)
  {
    if(MySensor.settings.interrupt_gpio < 2 || MySensor.settings.interrupt_gpio > 27)
    {
        p_printf(RED,(char *) "Invalid GPIO. Must be between 2 and 27\n");
        return(ERROR);
    }   
  }
  
  if (MySensor.settings.wake_gpio == MySensor.settings.reset_gpio)
  {
      p_printf(RED,(char *) "GPIO %d is already in use \n",MySensor.settings.wake_gpio);
      return(ERROR);
  }

  if (MySensor.settings.wake_gpio == MySensor.settings.interrupt_gpio)
  {
      p_printf(RED,(char *) "GPIO %d is already in use \n",MySensor.settings.wake_gpio);
      return(ERROR);
  }

  if (MySensor.settings.reset_gpio == MySensor.settings.interrupt_gpio)
  {
      p_printf(RED,(char *) "GPIO %d is already in use \n",MySensor.settings.reset_gpio);
      return(ERROR);
  }
  
  return(SUCCESS);
}

/************************************************************
 * set up child process forinterrupt handling from the CCS811
 * 
 * @param  mm : measurement structure
 ************************************************************/
 int configure_CCS_interrupt(struct measure *mm)
{
    pid_t parent;

    /* set interrupt signal */
    set_child_signal(1);
    
    /* create new child program to detect interrupt */
    ch_pid = fork();

    if (ch_pid < 0)
    {
        printf(REDSTR,"could not create child process to handle interrupts\n");
        return(ERROR);
    }
    
    /* if parent:  save the PID and return */
    else if (ch_pid > 0)
    {
       
        if (mm->verbose) 
            printf("child process to handle interrupt has been created (pid: %d)\n", ch_pid);
        
        return(SUCCESS);
    }

    /**************************************************************
     * for child process to execute that watches the interrupt line
     ***************************************************************/
    
    /* disable falling edge (just in case it was set earlier) */
    bcm2835_gpio_clr_afen(MySensor.settings.interrupt_gpio);

    /* disable rising edge (just in case it was set earlier) */
    bcm2835_gpio_clr_aren(MySensor.settings.interrupt_gpio);

    /* set as GPIO on Raspberry input */
    bcm2835_gpio_fsel(MySensor.settings.interrupt_gpio, BCM2835_GPIO_FSEL_INPT);

    /* pull up (trigger is active low) */
    bcm2835_gpio_set_pud(MySensor.settings.interrupt_gpio,BCM2835_GPIO_PUD_UP);

    /* enable falling edge detection */
    bcm2835_gpio_afen(MySensor.settings.interrupt_gpio);

    /* get parent PPID to signal */
    parent = getppid();
    
    while(1)
    {
        /* if level changed is detected */
        if (bcm2835_gpio_eds(MySensor.settings.interrupt_gpio))
        {
            /* signal parent */
            kill (parent,SIGHUP);

            /* clear the eds flag in Raspberry by setting it to 1*/
            bcm2835_gpio_set_eds(MySensor.settings.interrupt_gpio);
        }
                         
        sleep(2);
    }
}

/***********************************************************
 *  Read and display version information about the CCS811
 ***********************************************************/
 int disp_versions()
{
    uint8_t version[5];
    
    CCS811Core::status ret = MySensor.getversionregisters(version);

    if (ret != CCS811Core::SENSOR_SUCCESS)
    {
        p_printf(RED,(char *)"error during reading CCS811 version informmation\n");
        return(ERROR);
    }
    
    p_printf(YELLOW, (char*) "\tHardware version    0x%02x\n", version[0]);
    p_printf(YELLOW, (char*) "\tBootloader version  0x%02x, trivial 0x%02x\n", version[1], version[2]);
    p_printf(YELLOW, (char*) "\tApplication version 0x%02x, trivial 0x%02x\n", version[3],version[4]);

    return(SUCCESS);
}

/*********************************************************
 *  perform init (take in account commandline option) 
 *  GPIO pins to use, speed of i2C, i2C addresses etc.
 
 * @param  mm : measurement structure
 *********************************************************/
void init_hardware(struct measure *mm)
{
    /* hard_I2C requires  root permission */    
    if (MySensor.settings.I2C_interface == hard_I2C)
    {
        if (geteuid() != 0)
        {
            p_printf(RED,(char *)"You must be super user\n");
            exit(EXIT_FAILURE);
        }
    }
  
    if(mm->verbose) printf((char *)"initialize hardware\n");

    /* set raspberry Pi hardware for communications
     * 
     * option 
     *  hard_I2C : use hardware I2C
     *  soft_I2C : use software I2C (default)
     * 
     * sent verbose (if 3 it will enable driver messages)
     */
  
    CCS811Core::status ret = MySensor.hw_init(MySensor.settings.I2C_interface, mm->verbose);
    
    if (ret != CCS811Core::SENSOR_SUCCESS)
    {
        p_printf(RED,(char *)"error during init\n");
        exit(EXIT_FAILURE);
    }
    
    if(mm->verbose) printf((char *)"initialize CCS811\n");
    
    /* if interupt GPIO was enabled */
    if (MySensor.settings.interrupt_gpio > 0) 
    {
        if(mm->verbose) printf((char *)"initialize interrupts\n");
        if (configure_CCS_interrupt(mm) == ERROR)   closeout();
    }

    /* set check for CCS811 and setup  */
    ret = MySensor.begin();

    if (ret != CCS811Core::SENSOR_SUCCESS)
    {
        p_printf(RED,(char *)"error during starting CSS811\n");
        closeout();
    }
   
    /* init Dylos DC1700 port */ 
    if (mm->dylos.include)
    {
        if(mm->verbose) printf((char *)"initialize Dylos\n");
        
        if (open_dylos(mm->dylos.port, mm->verbose) != 0)   closeout();
    }
     
    /* display CCS811 information */
    if (mm->verbose == 3)
        if (disp_versions() == ERROR)   exit(EXIT_FAILURE); 
    
    if (MySensor.settings.conditionPeriod == 0) return;
    
    /* wait warming up */
    do
    {
        if (MySensor.settings.conditionPeriod > 1)
            p_printf(GREEN,(char *)"\rwarming up. %d minutes to go  ",MySensor.settings.conditionPeriod);
        else
            p_printf(GREEN,(char *)"\rwarming up. 1 minute to go  \n");
        sleep(60);
    } while (MySensor.settings.conditionPeriod--);  
}

/*********************************************************************
 *  set default variable values
 * 
 * @param mm : measurement structure
 *********************************************************************/
void init_variables(struct measure *mm)
{
    /* I2C values */
    MySensor.settings.baudrate = 100;           // 100khz
    MySensor.settings.I2CAddress = CCS811_ADDR;
    MySensor.settings.I2C_interface = soft_I2C; // default soft I2C
    MySensor.settings.sda = DEF_SDA;            // default SDA line for soft_I2C
    MySensor.settings.scl = DEF_SCL;            // SCL GPIO for soft_I2C
        
    /* CSS811 values */
    MySensor.settings.reset_gpio = RESET_GPIO;
    MySensor.settings.wake_gpio = WAKE_GPIO;
    MySensor.settings.interrupt_gpio = INT_GPIO;
    MySensor.settings.mode = CCS_MODE;
    MySensor.settings.baseline = 0;
    MySensor.settings.conditionPeriod = CONDITION_PERIOD; 
    MySensor.settings.refResistance = 100000;      // 100K reference resistor  
  
    /* Dylos values */
    mm->dylos.include = 0;
    mm->dylos.value_pm1 = 0;
    mm->dylos.value_pm10 = 0;
    
    /* measurement instructions & results */
    mm->verbose = 0;
    mm->loop = 0;
    mm->b_time_start = 0;
    mm->b_save_file[0] = 0x0;
    mm->v_save_file[0] = 0x0;
    mm->format[0] = 0x0;
    mm->env_tempC = 0;              // reset enviroment variables
    mm->env_humid = 0;              // reset enviroment variables
    
    //other
    NoColor = false;
}

/*********************************************
 * output format can be defined
 * 
 * @param mm : measurement structure
 * @param buf : result of creating output
 * CCS811 results:
 *  c = CO2
 *  t = TVOC
 *  T = temperature
 * 
 * DYLOS results:
 *  D = PM1
 *  E = PM10
 * 
 * Markup: 
 *  \l = local time
 *  \t = tab
 *  \s = space
 *  \, = comma
 *  \; = semicolon
 *  \\x = character x is included (x can be any)
 *  \n = new line
 *****************************************************************/
void format_output(struct measure *mm, char *buf)
{
    char    *p,tm[30];
    
    buf[0] = 0x0;
    
    /* use default output  if no specific format was requested */
    if (strlen(mm->format) == 0 )
    {
        sprintf(buf, "CO2: %d, TVOC: %d, Temp %2.2f\n", mm->CO2, mm->TVOC,mm->temperature);
        return;
    }

    p = mm->format;
    
    while (*p != 0x0 && strlen(buf) < MAXBUF)
    {
        
        //CCS811
        if (*p == 'c')  sprintf(buf, "%s CO2: %d",buf,mm->CO2);
        else if (*p == 't') sprintf(buf, "%s TVOC: %d",buf,mm->TVOC);
        else if (*p == 'T') sprintf(buf, "%s Temp: %2.2f",buf,mm->temperature);
       
        //Dylos
        else if (*p == 'D') sprintf(buf, "%s PM1: %d",buf,mm->dylos.value_pm1);
        else if (*p == 'E') sprintf(buf, "%s PM10: %d",buf,mm->dylos.value_pm10);
        
        //markup
        else if (*p == '\\')
        {
            p++;

            if (*p == 't') sprintf(buf, "%s\t",buf);
            else if (*p == 's') sprintf(buf, "%s ",buf);
            else if (*p == 'n') sprintf(buf, "%s\n",buf);
            else if (*p == ',') sprintf(buf, "%s,",buf);
            else if (*p == ';') sprintf(buf, "%s;",buf);
            else if (*p == 'l')
            {
                // get timestamp
                time_stamp(tm);
                sprintf(buf, "%s %s",buf,tm);
            }
            else if (*p == '\\')
            {
                p++; 
                sprintf(buf, "%s%c",buf,*p);
            }
        }
        
        // trouble ...
        else
        {
            printf("Illegal character %c in output format string: %s\n", *p, mm->format);
            sprintf(buf, "CO2: %d, TVOC: %d, Temp %2.2f\n", mm->CO2, mm->TVOC,mm->temperature);
            return;
        }
   
        p++;    
 
    }
    
    sprintf(buf,"%s\n",buf);
}

/**********************************************************
 * output values to screen and file (if requested) 
 * 
 * @param mm : measurement structure
 *  
 **********************************************************/

int do_output_values(struct measure *mm)
{
    FILE    *fp = NULL ;
    char    buf[MAXBUF];

    /* get results */
    mm->CO2  = MySensor.getCO2();
    mm->TVOC = MySensor.getTVOC();
    mm->temperature = MySensor.getTemperature();

    /* create output string (including Dylos, BME 280 etc. ) */
    format_output(mm, buf);
        
    /* display output */
    p_printf(YELLOW,(char *) "%s",buf);
     
    /* append output to a save_file ? */
    if (strlen (mm->v_save_file) > 0)
    {
        if(mm->verbose >1 ) printf("Appending data to file %s\n",mm->v_save_file);
        
        /* save baseline to file */
        fp = fopen (mm->v_save_file, "a");
           
        /* Checks if file is open */
        if (fp == NULL )
        { 
            p_printf(RED,(char *) "Issue with opening output file: %s\n", mm->v_save_file);                      
            return(ERROR);
        }
        
        /* write output */
        if (fwrite(buf, sizeof(char), strlen(buf),fp) != strlen(buf))
        { 
            p_printf(RED,(char *) "Issue during writing output file: %s\n", mm->v_save_file);                      
            fclose(fp);
            return(ERROR);
        }

        fclose(fp);
    }
    
    return(SUCCESS);
}

/*********************************************************************
 * compensate CCS811 for temperature and humidity
 * 
 * if temperature is below zero, the current temperature from the CCS811
 * will be read (which MIGHT be adjusted with temperature offset in the
 * drive withsetTemperatureOffset())
 * 
 * @param mm : measurment structure
 *********************************************************************/
int do_compensation(struct measure *mm)
{
    CCS811Core::status ret;
    
    if (mm-> env_humid == 0 || mm->env_tempC == 0)
    {
        p_printf(RED,(char *)"humidity or temp for Environmental data not set\n");
        return(ERROR);
    }
    
    /* this will trigger to read temperature from CCS811 */
    if (mm->env_tempC < 0)
        ret = MySensor.setEnvironmentalData(mm->env_humid, (float) MySensor.getTemperature());

    else
        /* This sends the provided temperature & humidity data to the CCS811 */
        ret = MySensor.setEnvironmentalData(mm->env_humid, mm->env_tempC);

    if (ret != CCS811Core::SENSOR_SUCCESS)
    {
        p_printf(RED,(char *)"can not set Environmental data\n");
        return(ERROR);      
    }
    
    return(SUCCESS);
}

/*************************************
 * Try to read from Dylos DC1700 monitor
 * 
 * @param mm : measurement structure
 *************************************/
void do_dylos(struct measure *mm)
{
    char    buf[MAXBUF], t_buf[MAXBUF];
    int     ret, i, offset =0 ;
    
    if(mm->verbose >1 ) printf("Reading Dylos data\n");
    
    /* reset values */
    mm->dylos.value_pm1 = mm->dylos.value_pm10 = 0;
    
    /* try to read from Dylos and wait max 2 seconds */
    ret = read_dylos(buf, MAXBUF, 2, mm->verbose);

    /* if data received : parse it */
    for(i = 0; i < ret; i++)
    {
        /* if last byte on line */
        if (buf[i] == 0xa) 
        {
            /* terminate & get PM10 */
            t_buf[offset] = 0x0;
            mm->dylos.value_pm10 = (uint16_t)strtod(t_buf, NULL);
            return;
        }
        
        /* skip carriage return and any carbage below 'space' */
        else if (buf[i] != 0xd && buf[i] > 0x1f) 
        {
            t_buf[offset] = buf[i];
        
            /* get PM1 */
            if (t_buf[offset] == ',')
            {
                t_buf[offset] = 0x0;
                mm->dylos.value_pm1 = (uint16_t)strtod(t_buf, NULL);
                offset=0;
            }
            else
                offset++;
        }
    }
}

/*************************************
 * parse & display error message  
 * 
 * @param  error register value
 *************************************/
void display_error(int error)
{
    if (error == 0) return;
   
    p_printf(RED, (char *) "error 0x%02x has occured:\n", error);
    
    if (error == 0xff) p_printf(RED, (char *) "\ti2C commnication\n");
    else
    {
       if (error & 0x01) p_printf(RED, (char *) "\tinvalid register to write\n");
       if (error & 0x02) p_printf(RED, (char *) "\tinvalid register to read\n");
       if (error & 0x04) p_printf(RED, (char *) "\tinvalid MEAS_MODE requested\n");
       if (error & 0x08) p_printf(RED, (char *) "\tMAX_RESISTANCE reached\n");
       if (error & 0x10) p_printf(RED, (char *) "\tHEATER_FAULT:current not in range\n");
       if (error & 0x20) p_printf(RED, (char *) "\tHEATER_FAULT:voltage not in range\n");
    }
}

/*********************************************************
 * Save baseline information every 5 min 
 * 
 * This routine will add the current running mode to the output
 * so it can be checked during reading.
 *************************************************************/  
int do_save_baseline(struct measure *mm) 
{
    char           buf[MAXBUF], tm[30];
    FILE           *fp = NULL ;
    uint16_t       baseline;
    
    /* check on 5 minutes (300 seconds) passed */
    if (mm->b_time_start != 0)
    {
        if ( 300 > time(NULL) - mm->b_time_start)
        {
            return(SUCCESS);
        }
    }
    
    if(mm->verbose) printf((char *)"saving baseline data to %s\n", mm->b_save_file);
    
    /* (re)set baseline time */
    mm->b_time_start = time(NULL);
    
    /* get current baseline */
    baseline = MySensor.getBaseline();
    
    /* get timestamp */
    time_stamp(tm);
    
    /* create string */
    sprintf(buf,"CCS811 baseline value\n%s\nMODE=%d\nBASELINE=%d\n\n",
    tm, MySensor.settings.mode, baseline);
    
    /* save baseline to file */
    fp = fopen (mm->b_save_file, "w");
   
    /* Checks if file can be created */
    if( fp == NULL )
    { 
        p_printf(RED,(char *) "Issue with opening write baseline file : %s\n", mm->b_save_file);                      
        return(ERROR);
    }

    /* write baseline */
    if (fwrite(buf, sizeof(char), strlen(buf),fp) != strlen(buf))
    { 
        p_printf(RED,(char *) "Issue during writing baseline file : %s\n", mm->b_save_file);                      
        fclose(fp);
        return(ERROR);
    }
    
    fclose(fp);
    
    return(SUCCESS); 
}

/*****************************************
 * main loop to read & display output
 * 
 * @param mm : measurement structure
 *****************************************/
void main_loop(struct measure *mm)
{
    uint16_t lloop = mm->loop, sec, i;
    int ret1;
    CCS811Core::status ret;
    
    if(mm->verbose) printf((char *)"starting mainloop\n");
    
    do
    {
        /* check for data available 
         * ch_pid > 1 it is interrupt driven and data_ready is 
         * set in handle_interrupt(). Otherwise we are polling 
         * status here. 
         */
        
        if (ch_pid == 1)
        {
            if (MySensor.dataAvailable()) data_ready = 1;
        }
        
        
        if(data_ready)
        {
            if(mm->verbose > 1 ) 
            {
                if (data_ready == 1) 
                    printf("Polling CSS811 data\n");
                else if (data_ready == 2) 
                    printf("Interrupt received. Reading CSS811 data\n");
            }
            
            /* reset */
            data_ready = 0;
            
            /* read and calculate values */
            ret = MySensor.readAlgorithmResults();
        
            if (ret != CCS811Core::SENSOR_SUCCESS)
            {
                p_printf(RED,(char *)"error during reading CSS811\n");
                closeout();
            }
            
            /* read and calculate temperature */
            ret = MySensor.readNTC();
        
            if (ret != CCS811Core::SENSOR_SUCCESS)
            {
                p_printf(RED,(char *)"error during reading temperature CSS811\n");
                closeout();
            }
            
            /* include to Dylos DC1700 ? */
            if (mm->dylos.include) do_dylos(mm);
            
            /* display (and save ?) values (depending on specific format) */
            do_output_values(mm);
    
            /* save baseline values */
            if (strlen(mm->b_save_file) > 0)
            {
                if (do_save_baseline(mm) != SUCCESS) closeout();
            }
            
            /* if compensation for temperature and humidity was requested */
            if (mm->env_humid != 0 && mm->env_tempC != 0)
                        do_compensation(mm);
        }
        
        else if (MySensor.checkForStatusError())
        {
            if (mm->verbose > 1) printf("No CCS811data and error\n");
            ret1 = MySensor.getErrorRegister();
            display_error(ret1);
            closeout();
        }
        
        else if (mm->verbose > 1) printf("No CCS811 data\n");
        
        /* display for I2C clock stretch statistics */
        if (mm->verbose == 3 ) MySensor.DispClockStretch();
        
        /* delay (not to overload i2C) depending on CCS811 mode */
        if (MySensor.settings.mode == 1) sec=5;
        else if (MySensor.settings.mode == 2) sec=15;
        else if (MySensor.settings.mode == 3) sec=65;
        else sec=30;
    
        /* forced seconds have been set */
        if (mm->loop_delay > 0) sec =  mm->loop_delay;
       
        if (mm->verbose > 1) printf((char *)"sleep (max) %d seconds\n",sec);
        
        /* this loop is to check every second for (potential) interrupt */
        for (i = 0 ; i < sec ; i++)
        {
            if (data_ready) break;
            sleep(1);     
        }
        
        /* loop until done */
        if (mm->loop > 0 ) lloop--;
 
    } while (mm->loop == 0 || lloop > 0);
    
    /* force write baseline info before stopping */
    if (strlen(mm->b_save_file) > 0)
    {
        mm->b_time_start = 0;
        if (do_save_baseline(mm) != SUCCESS) closeout();
    }
}

/*********************************************
 * Write entry in parameter file
 * 
 * @param buf : buffer to write
 * @param fp: opened file pointer
 ********************************************/

int parameter_write_entry(char *buf, FILE *fp) 
{
    if (fwrite(buf, 1, strlen(buf),fp) != strlen(buf))
    {
        printf("Error during writing parameters\n");
        fclose(fp);
        return(ERROR);
    } 
    
    return(SUCCESS);
}
/********************************************************************
 * save all parameters to a file, so it can be imported later on with
 * parameter_from_file()
 * 
 * @param file : filename to save
 * @param mm :   pointer to measurement parameters
 ********************************************************************/
int parameter_to_file(char *file, struct measure *mm)
{
    FILE    *fp = NULL ;
    char    c;
    char    buf[MAXBUF], tm[30];
    
    /* check that file exists */
    if (access (file, F_OK) != -1)
    {
        p_printf(RED, (char *)"file for saving parameters exists\nOverwrite ? ");
    
        c= getchar();
        
        if (c != 'Y' && c != 'y') return(SUCCESS);
    }

    /* open inputfile for writing */
    fp = fopen (file, "w");
    
    /* Checks if file is empty */
    if( fp == NULL ) { 
        p_printf(RED,(char *) "Issue with opening parameter file for writting : %s\n", file);                      
        return(ERROR);
    }

    /* create top line  with version */
    time_stamp(tm);
    sprintf(buf,"# Parameter file. created %s\nVERSION:%d\n", tm, VERSION);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);
  
    /* save all CCS811 */    
    sprintf(buf,"# CCS811 information\n");
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);    
       
    sprintf(buf,"CCS811 i2C ADDRESS: -a 0x%02x\n", MySensor.settings.I2CAddress);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);  
  
    sprintf(buf,"RESET-GPIO: -r %d\n", MySensor.settings.reset_gpio);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);
    
    sprintf(buf,"WAKE-GPIO: -w %d\n", MySensor.settings.wake_gpio);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);     
    
    sprintf(buf,"INTERRUPT-GPIO: -i %d\n", MySensor.settings.interrupt_gpio);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);  
    
    sprintf(buf,"RUN-MODE: -m %d\n", MySensor.settings.mode);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);  

    sprintf(buf,"BASELINE: -b %d\n", MySensor.settings.baseline);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR); 

    if ( MySensor.settings.tempOffset > 0)
    {
        sprintf(buf,"temperature offset: -o %d\n", MySensor.settings.tempOffset);
        if (parameter_write_entry(buf, fp) == ERROR) return(ERROR); 
    }
    
    if (mm->env_tempC > 0)
    {
        sprintf(buf,"temperature compensation: -t %f\n", mm->env_tempC);
        if (parameter_write_entry(buf, fp) == ERROR) return(ERROR); 
    } 
     
    if (mm->env_humid > 0)
    {
        sprintf(buf,"humidity compensation: -u %f\n", mm->env_humid);
        if (parameter_write_entry(buf, fp) == ERROR) return(ERROR); 
    }    
     
    if (mm->dylos.include)
    {
        sprintf(buf,"\n# DYLOS DC1700 information\n");
        if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);   
        
        /* save DYLOS */ 
        sprintf(buf,"DYLOS PORT: -D \"%s\"\n",mm->dylos.port);
        if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);
    }
    
    /* save general information */ 
    sprintf(buf,"\n# general information\n");
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);   
    
    sprintf(buf,"CONDITION_PERIOD: -C %d\n", MySensor.settings.conditionPeriod);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR); 
   
    sprintf(buf,"LOOP COUNT: -L %d\n", mm->loop);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR); 
    
    sprintf(buf,"FORCE LOOP DELAY: -U %d\n", mm->loop_delay);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR); 
    
    sprintf(buf,"BASELINE SAVEFILE: -S %s\n", mm->b_save_file);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR); 
    
    sprintf(buf,"VALUE SAVEFILE: -W %s\n", mm->b_save_file);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR); 
    
    sprintf(buf,"OUTPUT FORMAT STRING: -O %s\n", mm->format);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR); 

    if (NoColor)
    {
        sprintf(buf,"NO COLOR OUTPUT: -B = true\n");
        if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);
    }
    
    if (mm->verbose > 0)
    {
        sprintf(buf,"VERBOSE: -V %d\n", mm->verbose);
        if (parameter_write_entry(buf, fp) == ERROR) return(ERROR); 
    }
 
     /* save I2C information */ 
    sprintf(buf,"\n# I2C information\n");
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);  
    
    sprintf(buf,"I2C BAUDRATE: -I %d\n", MySensor.settings.baudrate);
    if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);
    
    if (MySensor.settings.I2C_interface == hard_I2C)
    {
        sprintf(buf,"hard_I2c: -H true\n");
        if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);
    }
    else
    {
        sprintf(buf,"SOFT_I2C SDA: -s %d\n", MySensor.settings.sda);
        if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);
    
        sprintf(buf,"SOFT_I2C SCL: -d %d\n", MySensor.settings.scl);
        if (parameter_write_entry(buf, fp) == ERROR) return(ERROR);
    }
    
    fclose(fp);
    
    p_printf(GREEN, (char *) "parameters saved to file\n");
    return(SUCCESS);    
}

/******************************************************
 * Parse parameter input (either commandline or file)
 * 
 * @param opt   : command line optional parameter
 * @param option : command line optional addition to paramter
 * @param  mm    : measurement structure
 * 
 ******************************************************/ 

void parse_cmdline(int opt, char *option, struct measure *mm)
{
    switch (opt) {

    case 'a':   // CCS811 i2C address
      MySensor.settings.I2CAddress = (int)strtod(option, NULL);
      
      if (MySensor.settings.I2CAddress != 0x5a && MySensor.settings.I2CAddress != 0x5b)
      {
          p_printf(RED,(char *) "incorrect CCS811 i2C address 0x%x\n",MySensor.settings.I2CAddress);
          exit(EXIT_FAILURE);
      }
      break;
    
    case 'b':   // get baseline from command line
      MySensor.settings.baseline = (uint16_t) strtod(option, NULL);
      break;
    
    case 'G':   // get baseline from file
      get_baseline(option,mm);
      break;
    
    case 'i':   // interrupt_gpio
      MySensor.settings.interrupt_gpio = (uint8_t) strtod(option, NULL);
      
      if (check_gpio() != SUCCESS)
      {
          p_printf(RED,(char *) "invalid interrupt GPIO %d\n",MySensor.settings.interrupt_gpio);
          exit(EXIT_FAILURE);
      }
      break; 
        
    case 'm':   // drive mode (will not support mode 0 = idle)
      MySensor.settings.mode = (uint8_t) strtod(option, NULL);
      
      if (MySensor.settings.mode < 1 || MySensor.settings.mode > 4)
      {
          p_printf(RED,(char *) "incorrect mode: %d (1 -4)\n",MySensor.settings.mode);
          exit(EXIT_FAILURE);
      }
      break;
    
    case 'r':   // reset_gpio
      MySensor.settings.reset_gpio = (uint8_t) strtod(option, NULL);
      
      if (check_gpio() != SUCCESS)
      {
          p_printf(RED,(char *) "invalid reset GPIO %d\n",MySensor.settings.reset_gpio);
          exit(EXIT_FAILURE);
      }
      break;

    case 'S':   // save file
      strncpy(mm->b_save_file, option, MAXBUF);
      break;

    case 'o':   // temperature offset
      MySensor.settings.tempOffset = (uint32_t) strtod(option, NULL);
   
      if (MySensor.settings.tempOffset > 10)
      {
          p_printf(RED,(char *) "invalid temperature offset %d\n",(int) MySensor.settings.tempOffset);
          exit(EXIT_FAILURE);
      }
      break;
      
    case 't':   // compensate temperature offset
      mm->env_tempC = (float) strtod(option, NULL);
      
      if (mm->env_tempC > 50 || mm->env_tempC < -1 )
      {
          p_printf(RED,(char *) "invalid temperature compensation %2.2f\n", mm->env_tempC);
          exit(EXIT_FAILURE);
      }
      break;

    case 'u':   // compensate humidity offset
      mm->env_humid = (float) strtod(option, NULL);

      if (mm->env_humid > 100 || mm->env_humid < 0 )
      {
          p_printf(RED,(char *) "invalid humidity compensation %2.2f\n", mm->env_humid);
          exit(EXIT_FAILURE);
      }
      break;
           
    case 'w':   // wake-gpio
      MySensor.settings.wake_gpio = (int)strtod(option, NULL);
      
      if (check_gpio() != SUCCESS)
      {
          p_printf(RED,(char *) "invalid wake GPIO %d\n",MySensor.settings.wake_gpio);
          exit(EXIT_FAILURE);
      }
      break;
     
    case 'B':   // set NO color output
      NoColor = true;
      break;
        
    case 'C':   // set condition /warm-up period (min)
      MySensor.settings.conditionPeriod = (uint8_t) strtod(option, NULL);
      break;

    case 'D':   // include Dylos read
      strncpy(mm->dylos.port, option, MAXBUF);
      mm->dylos.include = 1;
      break;

    case 'E':    // save current settings to file
      strncpy(save_file,option,MAXBUF);
      break;  
  
    case 'K':   // load all setting from file
      parameter_from_file(option, mm);
      break;
          
    case 'H':   // i2C interface 
      MySensor.settings.I2C_interface = hard_I2C; 
      break;
        
    case 'I':   // i2C Speed
      MySensor.settings.baudrate = (uint32_t) strtod(option, NULL);
     
      if (MySensor.settings.baudrate  < 1 || MySensor.settings.baudrate  > 400)
      {
          p_printf(RED,(char *) "Invalid i2C speed option %d\n",MySensor.settings.baudrate );
          exit(EXIT_FAILURE);
      }
      break;
    
    case 'L':   // loop count
      mm->loop = (int) strtod(option, NULL);
      break;

          
    case 'O':   // output string
      strncpy(mm->format,option,MAXBUF);
      break;
    
    case 'U':   // force loop delay
      mm->loop_delay = (int) strtod(option, NULL);
      break;
      
    case 'V':   // verbose level ( 1 = flow, 2 = details, 3 = more details)
      mm->verbose = (int) strtod(option, NULL);
   
      if (mm->verbose < 1 || mm->verbose > 3)
      {
          p_printf(RED,(char *) "incorrect verbose level :%d\n",mm->verbose);
          exit(EXIT_FAILURE);
      }
      break;
     
    case 'W':   // save file
      strncpy(mm->v_save_file, option, MAXBUF);
      break;

    case 'd':   // change default SCL line for soft_I2C
      MySensor.settings.scl = (int)strtod(option, NULL);
      
      if (MySensor.settings.scl < 2 || MySensor.settings.scl == 4 || 
      MySensor.settings.scl > 27 || MySensor.settings.sda == MySensor.settings.scl)
      {
          p_printf(RED,(char *) "invalid GPIO for SCL :  %d\n",MySensor.settings.scl);
          exit(EXIT_FAILURE);
      }   
      break; 

    case 's':   // change default SDA line for soft_I2C
      MySensor.settings.sda = (int)strtod(option, NULL);
      
      if (MySensor.settings.sda < 2 || MySensor.settings.sda == 4 || 
      MySensor.settings.sda > 27 || MySensor.settings.sda == MySensor.settings.scl)
      {
          p_printf(RED,(char *) "invalid GPIO for SDA :  %d\n", MySensor.settings.sda);
          exit(EXIT_FAILURE);
      }   
      break;
     
    case 'h':   // help     
    default: /* '?' */
        usage();
        exit(EXIT_FAILURE);
    }
}

/****************************************************************
 * read the options / parameters from file previous created with
 * parameter_to_file()
 * 
 * @param file : input file to read from
 * @param  mm : measurement structure
 ****************************************************************/
 
int parameter_from_file(char * file, struct measure *mm)
{
    FILE    *fp = NULL ;
    char    line[MAXBUF], *p, opt_arg[20];
    int     version, opt,i, j;
    
    if (mm->verbose) printf("reading parameter file\n");
    
    /* open inputfile for reading */
    fp = fopen (file, "rb");
   
    /* Checks if file is empty */
    if( fp == NULL ) { 
        p_printf(RED,(char *) "Issue with opening/reading parameter file : %s\n", file);                      
        exit(EXIT_FAILURE);
    }

    /* read line to version */
    while(fgets(line,MAXBUF,fp))
    {
        p = strstr(line,"VERSION:");
        
        if (p != NULL)
        {
            p += strlen("VERSION:");
            
            /* get version with which it was created */
            version = (uint16_t) strtod(p,NULL);
            
            if (version != VERSION)
            {
                p_printf(RED, (char *) 
                "WARNING: current version %d, file created with version %d)\n"
                "         you might get errors or not using full capacity.\n"
                , VERSION, version);
            }
            
            break;
        }
    }
 
    while(fgets(line,MAXBUF,fp))
    {
       
       p = line;
       
       for (i=0, j=0 ; i <  (int) strlen(line); i++,p++)
       {
            /* comment start found */
            if (*p == '#')
            {   
                i = MAXBUF;
                continue;
            }
    
            /* find begin opt and opt-arg */
            else if (*p == ':')
            {               
                /* find start of option */
                while (*p++ != '-');
                
                /* save opt */
                opt = *p++;
                
                /* skip spaces/tabs etc to find start of argument */
                while (*p < 0x21)
                {
               
                    /* end of line? */
                    if (*p == 0x0)
                    {   
                        i = MAXBUF;
                        break;
                    }
                    
                    p++;
                } 
                
                /* copy optarg (must have been provided) */
                while(*p != 0x0)
                {
                    if (*p > 0x20)  opt_arg[j++] = *p++;
                    else
                    {
                        opt_arg[j] = 0x0;
                        if (mm ->verbose > 0)
                            printf("parsed from file :  opt %c optarg %s\n", opt, opt_arg);
                        parse_cmdline(opt, opt_arg, mm);
                    
                        i = MAXBUF;
                        break;
                    }
                }
            }
       }
    }
    
    fclose(fp);   
        
    p_printf(GREEN, (char *) "Parameters obtained from file\n");
    
    return(SUCCESS);
}

/**************************
 *  program starts here
 **************************/
int main(int argc, char *argv[])
{
    int opt;
    struct measure mm;
  
    /* initialize default setting */
    init_variables(&mm);
    
    /* set signals */
    set_signals();
    
    /* save name for (potential) usage display */
    strncpy(progname,argv[0],20);
    
    /* parse commandline */
    while ((opt = getopt(argc, argv, "m:a:r:i:w:b:S:t:o:u:G:I:C:D:E:K:BH L:U:O:V:W:")) != -1)
    {
        parse_cmdline(opt, optarg, &mm);
    }
    
    /* save the current parameters */
    if (strlen(save_file) > 0)  
        if (parameter_to_file(save_file, &mm) == ERROR) exit(EXIT_FAILURE);
    
    /* initialize the hardware (BCM2835, CCS811, Dylos)*/
    init_hardware(&mm);

    /* main loop (include command line options)  */
    main_loop(&mm);
    
    closeout();
    exit(EXIT_SUCCESS);
}
