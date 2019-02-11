
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "lp55231.h"

/* Control Register Map */

#define ENABLE_ENGINE_CNTRL1    0x00
#define ENGINE_CNTRL2           0x01
#define OUTPUT_DIRECT_MSB       0x02
#define OUTPUT_DIRECT_LB        0x03
#define OUTPUT_ON_OFF_CNTL_MSB  0x04
#define OUTPUT_ON_OFF_CNTL_LSB  0x05
#define D1_CONTROL              0x06
#define D2_CONTROL              0x07
#define D3_CONTROL              0x08
#define D4_CONTROL              0x09
#define D5_CONTROL              0x0A
#define D6_CONTROL              0x0B
#define D7_CONTROL              0x0C
#define D8_CONTROL              0x0D
#define D9_CONTROL              0x0E

// 0x0F to 0x15 :- Reserved

#define D1_PWM                  0x16     
#define D2_PWM                  0x17     
#define D3_PWM                  0x18     
#define D4_PWM                  0x19     
#define D5_PWM                  0x1A     
#define D6_PWM                  0x1B     
#define D7_PWM                  0x1C     
#define D8_PWM                  0x1D     
#define D9_PWM                  0x1E

// 0x1F TO 0x25 :- RESERVED

#define D1_CURRENT_CONTROL      0x26
#define D2_CURRENT_CONTROL      0x27
#define D3_CURRENT_CONTROL      0x28
#define D4_CURRENT_CONTROL      0x29
#define D5_CURRENT_CONTROL      0x2A
#define D6_CURRENT_CONTROL      0x2B
#define D7_CURRENT_CONTROL      0x2C
#define D8_CURRENT_CONTROL      0x2D
#define D9_CURRENT_CONTROL      0x2E

// 0x2F TO 0x35 :- RESERVED

#define MISC                    0x36
#define ENGINE1PC               0x37
#define ENGINE2PC               0x38
#define ENGINE3PC               0x39
#define STATUS_INTERRUPT        0x3A
#define INT_GPO                 0x3B
#define VARIABLE                0x3C
#define RESET                   0x3D
#define TEMP_ADC_CONTROL        0x3E
#define TEMPERATURE_READ        0x3F
#define TEMPERATURE_WRITE       0x40
#define LED_TEST_CONTROL        0x41
#define LED_TEST_ADC            0x42

#define ENGINE1_VARIABLE_A      0x45
#define ENGINE2_VARIABLE_A      0x46
#define ENGINE3_VARIABLE_A      0x47
#define MASTER_FADER_1          0x48
#define MASTER_FADER_2          0x49
#define MASTER_FADER_3          0x4A

// 0x4B :- RESERVED

#define ENG1_PROG_START_ADDR    0x4C
#define ENG2_PROG_START_ADDR    0x4D
#define ENG3_PROG_START_ADDR    0x4E
#define PROG_MEM_PAGE_SEL       0x4F

#define PROG_MEM_START          0x50 // 16-bit instructions, two registers per instruction, 96-instructions

#define ENG1_MAPPING_MSB        0x70
#define ENG1_MAPPING_LSB        0x71
#define ENG2_MAPPING_MSB        0x72
#define ENG2_MAPPING_LSB        0x73
#define ENG3_MAPPING_MSB        0x74
#define ENG3_MAPPING_LSB        0x75

#define GAIN_CHANGE_CTRL        0x76

#define LP55231_Address         0x32   // Address of the LP55231

// map leds to LP55231 pin numbers
#define R1 D7_PWM  
#define R2 D8_PWM
#define R3 D9_PWM
#define G1 D4_PWM
#define G2 D5_PWM
#define G3 D2_PWM
#define B1 D1_PWM
#define B2 D6_PWM
#define B3 D3_PWM

// define current for all leds
#define CURRENT 0x0A  // eight bits in increments of 100 microAmps, A = 1 mA per led = 9 mA if all on

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

static const bool invert = false; // set true if common anode, false if common cathode

static uint8_t color = 0;        // a value from 0 to 255 representing the hue
static uint8_t R, G, B;          // the Red Green and Blue color components
static uint8_t brightness = 255; // 255 is maximum brightness

// I2C read/write functions  

static void writeByte(const nrf_drv_twi_t * m_twi, uint8_t address, uint8_t subAddress, uint8_t data)
{
	ret_code_t err_code;
	uint8_t buf[2] = {subAddress, data};
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(m_twi, address, buf, sizeof(buf), false);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
}

// I2C read function (single byte) for the CaliPile sensor
static uint8_t readByte(const nrf_drv_twi_t * m_twi, uint8_t address, uint8_t subAddress)
{
	ret_code_t err_code;
    uint8_t data; // `data` will store the register data   
	
	m_xfer_done = false;
    err_code = nrf_drv_twi_tx(m_twi, address, &subAddress, sizeof(subAddress), true);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(m_twi, address, &data, sizeof(data));
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
	
	return data;                             // Return data read from slave register
}

static void readBytes(const nrf_drv_twi_t * m_twi, uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	ret_code_t err_code;
  
	m_xfer_done = false;
    err_code = nrf_drv_twi_tx(m_twi, address, &subAddress, sizeof(subAddress), true);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(m_twi, address, dest, count);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
}

// Courtesy http://www.instructables.com/id/How-to-Use-an-RGB-LED/?ALLSTEPS
// function to convert a color to its Red, Green, and Blue components.

static void hueToRGB(uint8_t hue, uint8_t brightness)
{
    uint16_t scaledHue = (hue * 6);
    uint8_t segment = scaledHue / 256; // segment 0 to 5 around the
                                            // color wheel
    uint16_t segmentOffset =
      scaledHue - (segment * 256); // position within the segment

    uint8_t complement = 0;
    uint16_t prev = (brightness * ( 255 -  segmentOffset)) / 256;
    uint16_t next = (brightness *  segmentOffset) / 256;

    if(invert)
    {
      brightness = 255 - brightness;
      complement = 255;
      prev = 255 - prev;
      next = 255 - next;
    }

    switch(segment ) {
    case 0:      // red
        R = brightness;
        G = next;
        B = complement;
    break;
    case 1:     // yellow
        R = prev;
        G = brightness;
        B = complement;
    break;
    case 2:     // green
        R = complement;
        G = brightness;
        B = next;
    break;
    case 3:    // cyan
        R = complement;
        G = prev;
        B = brightness;
    break;
    case 4:    // blue
        R = next;
        G = complement;
        B = brightness;
    break;
   case 5:      // magenta
    default:
        R = brightness;
        G = complement;
        B = prev;
    break;
    }
}

// Function to check temperature and turn off leds for 10 sec if they are too hot
static void checkTemp(const nrf_drv_twi_t * m_twi)
{
  // read temperature
    int8_t temp = readByte(m_twi, LP55231_Address, TEMPERATURE_READ);
    NRF_LOG_DEBUG("Temperature of LP55231 is %d degC", temp);
	
    if(temp > 38) 
	{
      // if temperature too high, turn all leds off!
         writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_MSB, 0x00); 
         writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0x00);
         NRF_LOG_DEBUG("Toohot, shutting down for 10 seconds!");
         nrf_delay_ms(10000);
    }
}

void lp55231_config(const nrf_drv_twi_t * m_twi)
{
  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
  //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  //Serial.begin(38400);
  //delay(5000);

  //I2Cscan(); // should detect LP55231 at 32 

  // enable the chip
  writeByte(m_twi, LP55231_Address, ENABLE_ENGINE_CNTRL1, 0x40);
  
// Enable auto increment (bit 6), auto charge pump (bits 4;3), and set the clock to internal mode bits 1:0
  writeByte(m_twi, LP55231_Address, MISC, 0x40 | 0x18 | 0x03);
  

  // Set PWM output
   writeByte(m_twi, LP55231_Address, R1, 0x20);  // 255 steps of PWM from 0 % (0x00) to 100% (0xFF)
   writeByte(m_twi, LP55231_Address, G1, 0x20);
   writeByte(m_twi, LP55231_Address, B1, 0xC0);
   writeByte(m_twi, LP55231_Address, R2, 0x20);
   writeByte(m_twi, LP55231_Address, G2, 0x20);
   writeByte(m_twi, LP55231_Address, B2, 0xC0);
   writeByte(m_twi, LP55231_Address, R3, 0x20);
   writeByte(m_twi, LP55231_Address, G3, 0x20);
   writeByte(m_twi, LP55231_Address, B3, 0xC0);

  // Set current output
   writeByte(m_twi, LP55231_Address, D1_CURRENT_CONTROL, CURRENT);
   writeByte(m_twi, LP55231_Address, D2_CURRENT_CONTROL, CURRENT);
   writeByte(m_twi, LP55231_Address, D3_CURRENT_CONTROL, CURRENT);
   writeByte(m_twi, LP55231_Address, D4_CURRENT_CONTROL, CURRENT);
   writeByte(m_twi, LP55231_Address, D5_CURRENT_CONTROL, CURRENT);
   writeByte(m_twi, LP55231_Address, D6_CURRENT_CONTROL, CURRENT);
   writeByte(m_twi, LP55231_Address, D7_CURRENT_CONTROL, CURRENT);
   writeByte(m_twi, LP55231_Address, D8_CURRENT_CONTROL, CURRENT);
   writeByte(m_twi, LP55231_Address, D9_CURRENT_CONTROL, CURRENT);

  // configure temperature sensor
  writeByte(m_twi, LP55231_Address, TEMP_ADC_CONTROL, 0x06); // continuous conversion 

  // start with all leds off
  writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_MSB, 0x00); // D9 led enabled by bit 0 here
  writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0x00); // D1 - D8 enabled by eight bits here
  nrf_delay_ms(1000);

  // test of led function, sweep through pins D1 - D9 one at a time
  for (int ii = 0; ii < 8; ii++)
  {
  writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 1 << ii);
  nrf_delay_ms(1000);
  writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0x00);
  }
 
  writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_MSB, 0x01);
  nrf_delay_ms(1000);
  writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_MSB, 0x00);
 
 // start with all leds off
  writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_MSB, 0x00); 
  writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0x00);
  nrf_delay_ms(1000);
}


void test_lp55231(const nrf_drv_twi_t * m_twi)
{  
    checkTemp(m_twi);

    writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0x49);
    for (color = 0; color < 255; color++) { // Slew through the color spectrum

    hueToRGB(color, brightness);  // call function to convert hue to RGB

    // write the RGB values to the pins
    writeByte(m_twi, LP55231_Address, R1, R);
    writeByte(m_twi, LP55231_Address, G1, G);
    writeByte(m_twi, LP55231_Address, B1, B);
  
    nrf_delay_ms(100);
    }

    checkTemp(m_twi);

    writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0xB0);
    for (color = 0; color < 255; color++) { // Slew through the color spectrum

    hueToRGB(color, brightness);  // call function to convert hue to RGB

    // write the RGB values to the pins
    writeByte(m_twi, LP55231_Address, R2, R);
    writeByte(m_twi, LP55231_Address, G2, G);
    writeByte(m_twi, LP55231_Address, B2, B);
  
    nrf_delay_ms(100);
    }

    checkTemp(m_twi);

    writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0x06);
    writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_MSB, 0x01);
    for (color = 0; color < 255; color++) { // Slew through the color spectrum

    hueToRGB(color, brightness);  // call function to convert hue to RGB

    // write the RGB values to the pins
    writeByte(m_twi, LP55231_Address, R3, R);
    writeByte(m_twi, LP55231_Address, G3, G);
    writeByte(m_twi, LP55231_Address, B3, B);
  
    nrf_delay_ms(100);
    }

    // turn off all leds
    writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0x00);
    writeByte(m_twi, LP55231_Address, OUTPUT_ON_OFF_CNTL_MSB, 0x00);
}
