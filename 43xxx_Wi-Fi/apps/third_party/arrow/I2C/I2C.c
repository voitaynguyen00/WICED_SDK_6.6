/** @file
 *
 * I2C Application
 *
 * This application tests I2C capable board features.
 *
 *
 */

#include "I2C.h"
#include <math.h>
#include "wiced.h"
#include "resources.h"
#include "sntp.h"
#include "command_console.h"
#include "wiced_management.h"
#include "command_console_ping.h"


/******************************************************
 *                      Macros
 ******************************************************/


/******************************************************
 *                    Constants
 ******************************************************/
#define MAX_LINE_LENGTH  (128)
#define MAX_HISTORY_LENGTH (20)
#define MAX_NUM_COMMAND_TABLE  (8)

#define HTS221_SLAVE_ADDR        (0x5F)
#define HTS221_WOAMI_REG         (0x0F | 0x80)
#define HTS221_CTRL_REG1         (0x20 | 0x80)
#define HTS221_TEMP_OUT_L        (0x2A | 0x80)
#define HTS221_TEMP_OUT_H        (0x2B | 0x80)
#define HTS221_T0_DEGC_X8        (0x32 | 0x80)
#define HTS221_T1_DEGC_X8        (0x33 | 0x80)
#define HTS221_T1_T0_MSB         (0x35 | 0x80)
#define HTS221_T0_OUT_L          (0x3C | 0x80)
#define HTS221_T0_OUT_H          (0x3D | 0x80)
#define HTS221_T1_OUT_L          (0x3E | 0x80)
#define HTS221_T1_OUT_H          (0x3F | 0x80)

#define HTS221_CTRL1_PD          (0x80)
#define HTS221_CTRL1_BDU         (0x02)

#define LIS2DH12_SLAVE_ADDR      (0x19)
#define LIS2DH12_WOAMI_REG       (0x0F | 0x80)
#define LIS2DH12_CTRL_REG0       (0x1E | 0x80)
#define LIS2DH12_CTRL_REG1       (0x20 | 0x80)
#define LIS2DH12_CTRL_REG2       (0x21 | 0x80)
#define LIS2DH12_CTRL_REG3       (0x22 | 0x80)
#define LIS2DH12_CTRL_REG4       (0x23 | 0x80)
#define LIS2DH12_CTRL_REG5       (0x24 | 0x80)
#define LIS2DH12_CTRL_REG6       (0x25 | 0x80)
#define LIS2DH12_STATUS_REG      (0x27 | 0x80)
#define LIS2DH12_OUT_X_L         (0x28 | 0x80)
#define LIS2DH12_OUT_X_H         (0x29 | 0x80)
#define LIS2DH12_OUT_Y_L         (0x2A | 0x80)
#define LIS2DH12_OUT_Y_H         (0x2B | 0x80)
#define LIS2DH12_OUT_Z_L         (0x2C | 0x80)
#define LIS2DH12_OUT_Z_H         (0x2D | 0x80)

#define LIS2DH12_CTRL1_ODR_400    (0x07<<4) // 400 Hz
#define LIS2DH12_CTRL1_ODR_25     (0x03<<4) // 25 Hz
#define LIS2DH12_CTRL1_ODR_10     (0x02<<4) // 10 Hz
#define LIS2DH12_CTRL1_ODR_1      (0x01<<4) // 1 Hz
#define LIS2DH12_CTRL1_ZEN        (0x01<<2)
#define LIS2DH12_CTRL1_YEN        (0x01<<1)
#define LIS2DH12_CTRL1_XEN        (0x01)

#define LIS2DH12_CTRL4_BDU        (0x80)

#define LIS2DH12_STAT_ZYXDA       (0x01<<3)

#define NUM_I2C_MESSAGE_RETRIES   (3)




/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/


wiced_result_t temperature_init( void );
wiced_result_t accelerometer_init( void );


/******************************************************
 *               Variable Definitions
 ******************************************************/
static char line_buffer[MAX_LINE_LENGTH];
static char history_buffer_storage[MAX_LINE_LENGTH * MAX_HISTORY_LENGTH];

static wiced_i2c_device_t i2c_device_temperature =
{
        .port = WICED_I2C_2,  //I2C_1
        .address = HTS221_SLAVE_ADDR,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE,
};

static wiced_i2c_device_t i2c_device_accelerometer =
{
        .port = WICED_I2C_2,  //I2C_1
        .address = LIS2DH12_SLAVE_ADDR,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE,
};


#define DIAGNOSTICS_COMMANDS \
{ "temp",  temperature_get,  0, NULL, NULL,"",  "get HTS221 temperature" }, \
{ "accel",  accelerometer_get,  0, NULL, NULL,"",  "get LIS2DH12 accelerometer" }, \


/******************************************************
 *               Function Definitions
 ******************************************************/

/**************************Holder function to get HTS221 temperature**************************/
int temperature_get(int argc, char *argv[]){
    uint8_t wbuf[8];
    uint8_t rbuf[8];
    int16_t T0, T1, T2, T3, raw;
    uint8_t val[4];
    int32_t temperature;
    float tempC, tempF;

    // Temperature Calibration values
    // Read 1 byte of data from address 0x32(50)
    wbuf[0] = HTS221_T0_DEGC_X8;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    T0 = rbuf[0];

    // Read 1 byte of data from address 0x33(51)
    wbuf[0] = HTS221_T1_DEGC_X8;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    T1 = rbuf[0];

    // Read 1 byte of data from address 0x35(53)
    wbuf[0] = HTS221_T1_T0_MSB;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    raw = rbuf[0];

    // Convert the temperature Calibration values to 10-bits
    T0 = ((raw & 0x03) * 256) + T0;
    T1 = ((raw & 0x0C) * 64) + T1;

    // Read 1 byte of data from address 0x3C(60)
    wbuf[0] = HTS221_T0_OUT_L;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[0] = rbuf[0];

    // Read 1 byte of data from address 0x3D(61)
    wbuf[0] = HTS221_T0_OUT_H;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[1] = rbuf[0];

    T2 = ((val[1] & 0xFF) * 256) + (val[0] & 0xFF);

    // Read 1 byte of data from address 0x3E(62)
    wbuf[0] = HTS221_T1_OUT_L;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[0] = rbuf[0];

    // Read 1 byte of data from address 0x3F(63)
    wbuf[0] = HTS221_T1_OUT_H;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[1] = rbuf[0];

    T3 = ((val[1] & 0xFF) * 256) + (val[0] & 0xFF);

    // Read 2 bytes of data; temperature msb and lsb
    wbuf[0] = HTS221_TEMP_OUT_L;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[0] = rbuf[0];

    wbuf[0] = HTS221_TEMP_OUT_H;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[1] = rbuf[0];

    temperature = ((val[1] & 0xFF) * 256) + (val[0] & 0xFF);
    if(temperature > 32767)
    {
        temperature -= 65536;
    }

    tempC = ((T1 - T0) / 8.0) * (temperature - T2) / (T3 - T2) + (T0 / 8.0);
    tempF = (tempC * 1.8 ) + 32;

    WPRINT_APP_INFO( ( "HTS221 temperature %.1f°C, %.1f°F\n", tempC, tempF ) );

    return 0;
}

/********************Converts raw accelerometer data to mg.***************************/
void convert_accel_data(int16_t* x, int16_t* y, int16_t* z) {
    uint16_t lx, ly, lz;

    lx = *x;
    ly= *y;
    lz= *z;

    *x = (int32_t)lx*1000/(1024*16); // transform data to millig, for 2g scale axis*1000/(1024*16),
    *y = (int32_t)ly*1000/(1024*16); // for 4g scale axis*1000/(1024*8),
    *z = (int32_t)lz*1000/(1024*16); // for 8g scale axis*1000/(1024*4)

    return;
}

/************************Holder function to get LIS2DH12 accelerometer*****************************/
int accelerometer_get(int argc, char *argv[]){
    uint8_t wbuf[8];
    uint8_t rbuf[8];
    uint8_t val[8];
    int16_t xdata, ydata, zdata;
    float xdataf, ydataf, zdataf;

    // Read status register from address 0x27
    wbuf[0] = LIS2DH12_STATUS_REG;
    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    val[0] = rbuf[0];

    if( val[0] & LIS2DH12_STAT_ZYXDA ) {

        // Read 1 byte of data from address 0x29
        wbuf[0] = LIS2DH12_OUT_X_H;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[1] = rbuf[0];

        // Read 1 byte of data from address 0x28
        wbuf[0] = LIS2DH12_OUT_X_L;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[0] = rbuf[0];

        // Read 1 byte of data from address 0x2B
        wbuf[0] = LIS2DH12_OUT_Y_H;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[3] = rbuf[0];

        // Read 1 byte of data from address 0x2A
        wbuf[0] = LIS2DH12_OUT_Y_L;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[2] = rbuf[0];

        // Read 1 byte of data from address 0x2D
        wbuf[0] = LIS2DH12_OUT_Z_H;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[5] = rbuf[0];

        // Read 1 byte of data from address 0x2C
        wbuf[0] = LIS2DH12_OUT_Z_L;
        wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
        wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
        val[4] = rbuf[0];

        xdata =(val[1]<<8 | val[0]);
        ydata =(val[3]<<8 | val[2]);
        zdata =(val[5]<<8 | val[4]);

        // Convert to mg
        convert_accel_data(&xdata, &ydata, &zdata);

        // Check for negative
        if(xdata&0x800) {
            xdata = (((~xdata&0x7ff)+1) * -1);
        }
        if(ydata&0x800) {
            ydata = (((~ydata&0x7ff)+1) * -1);
        }
        if(zdata&0x800) {
            zdata = (((~zdata&0x7ff)+1) * -1);
        }

        // Convert to g and round
        xdataf = roundf(xdata * 0.001);
        ydataf = roundf(ydata * 0.001);
        zdataf = roundf(zdata * 0.001);

        WPRINT_APP_INFO(("x: %.fg\t", xdataf));
        WPRINT_APP_INFO(("y: %.fg\t", ydataf));
        WPRINT_APP_INFO(("z: %.fg\n", zdataf));

    } else {
        WPRINT_APP_INFO(("No new XYZ data\n"));
        return 0;
    }

    return 0;
}

static const command_t init_commands[] = {
        DIAGNOSTICS_COMMANDS
        CMD_TABLE_END
};

/******************************Application start/main function.**********************************/
void application_start( void )
{
    //wiced_result_t result;

    wiced_init( );

    /* Initialize the device */
    wiced_core_init( );

    /* probe for temperature device */
    temperature_init();

    /* probe for accelerometer device */
    accelerometer_init();

    /* enable command line interface */
    WPRINT_APP_INFO( ( "\r\nType help to know more about commands ...\r\n" ) );
    command_console_init( STDIO_UART, MAX_LINE_LENGTH, line_buffer, MAX_HISTORY_LENGTH, history_buffer_storage, " " );
    console_add_cmd_table( init_commands );


}

/*********************************Initializes I2C, probes for temperature device******************************************/
wiced_result_t temperature_init( void )
{
    uint8_t wbuf[8];
    uint8_t rbuf[8];

    /* Initialize I2C for the temp sensor*/
    if ( wiced_i2c_init( &i2c_device_temperature ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "I2C Initialization Failed\n" ) );
        return WICED_ERROR;
    }

    /* Probe I2C bus for temperature sensor */
    if( wiced_i2c_probe_device( &i2c_device_temperature, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE )
    {
        WPRINT_APP_INFO( ( "Failed to connect to temperature device; addr 0x%x\n", i2c_device_temperature.address ) );
        return WICED_ERROR;
    }

    wbuf[0] = HTS221_WOAMI_REG;
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );


    if( rbuf[0] != 0xbc )
    {
        WPRINT_APP_INFO( ( "Failed to read WHOAMI from temperature device; addr 0x%x\n", i2c_device_temperature.address ) );
        return WICED_ERROR;
    }
    WPRINT_APP_INFO( ( "HTS221 device (0x%x) at address 0x%x\n", rbuf[0], i2c_device_temperature.address ) );

    /* Power-up the device */
    wbuf[0] = HTS221_CTRL_REG1;
    wbuf[1] = (HTS221_CTRL1_PD | HTS221_CTRL1_BDU);
    wiced_i2c_write( &i2c_device_temperature, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2 );

    return WICED_SUCCESS;
}

/********************************Initializes I2C, probes for accelerometer device***************************************/
wiced_result_t accelerometer_init( void )
{
    uint8_t wbuf[8];
    uint8_t rbuf[8];

    /* Initialize I2C */
    if ( wiced_i2c_init( &i2c_device_accelerometer ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "I2C Initialization Failed\n" ) );
        return WICED_ERROR;
    }

    /* Probe I2C bus for accelerometer */
    if(  wiced_i2c_probe_device( &i2c_device_accelerometer, NUM_I2C_MESSAGE_RETRIES )  != WICED_TRUE )
    {
        WPRINT_APP_INFO( ( "Failed to connect to accelerometer device; addr 0x%x\n", i2c_device_accelerometer.address ) );
        return WICED_ERROR;
    }

    wbuf[0] = LIS2DH12_WOAMI_REG;
    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );

    if( rbuf[0] != 0x33 )
    {
        WPRINT_APP_INFO( ( "Failed to read WHOAMI from accelerometer device; addr 0x%x\n", i2c_device_accelerometer.address ) );
        return WICED_ERROR;
    }
    WPRINT_APP_INFO( ( "LIS2DH12 device (0x%x) at address 0x%x\n", rbuf[0], i2c_device_accelerometer.address ) );

    /* Power-up the device */
    wbuf[0] = LIS2DH12_CTRL_REG1;
    wbuf[1] = 0;
    wbuf[1] = (LIS2DH12_CTRL1_ODR_400 | LIS2DH12_CTRL1_ZEN | LIS2DH12_CTRL1_YEN | LIS2DH12_CTRL1_XEN);
    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2 );

    /* Set normal mode */
    wbuf[0] = LIS2DH12_CTRL_REG4;
    wbuf[1] = LIS2DH12_CTRL4_BDU;
    wiced_i2c_write( &i2c_device_accelerometer, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2 );

    return WICED_SUCCESS;
}
