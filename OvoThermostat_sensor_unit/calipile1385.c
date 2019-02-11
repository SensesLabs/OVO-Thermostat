/******************************************************************************
		INCLUDES
 *****************************************************************************/

/*library*/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/*nordic*/
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/*ovo thermostat*/
#include "ovo_thermostat_config.h"
#include "calipile1385.h"
#include "sd.h"

/******************************************************************************
		CONSTANTS
 *****************************************************************************/

// CaliPile Registers
#define CALIPILE_TPOBJECT           1
#define CALIPILE_TPAMBIENT          3
#define CALIPILE_TPOBJLP1           5
#define CALIPILE_TPOBJLP2           7
#define CALIPILE_TPAMBLP3           10
#define CALIPILE_TPOBJLP2_FRZN      12
#define CALIPILE_TPPRESENCE         15
#define CALIPILE_TPMOTION           16
#define CALIPILE_TPAMB_SHOCK        17
#define CALIPILE_INTERRUPT_STATUS   18
#define CALIPILE_CHIP_STATUS        19
#define CALIPILE_SLP12              20
#define CALIPILE_SLP3               21
#define CALIPILE_TP_PRES_THLD       22
#define CALIPILE_TP_MOT_THLD        23
#define CALIPILE_TP_AMB_SHOCK_THLD  24
#define CALIPILE_INT_MASK           25
#define CALIPILE_SRC_SELECT         26
#define CALIPILE_TMR_INT            27
#define CALIPILE_TPOT_THR           28

// Calipile EEPROM Registers
#define CALIPILE_EEPROM_CONTROL     31
#define CALIPILE_EEPROM_PROTOCOL    32
#define CALIPILE_EEPROM_CHECKSUM    33
#define CALIPILE_EEPROM_LOOKUPNUM   41
#define CALIPILE_EEPROM_PTAT25      42
#define CALIPILE_EEPROM_M           44
#define CALIPILE_EEPROM_U0          46
#define CALIPILE_EEPROM_UOUT1       48
#define CALIPILE_EEPROM_TOBJ1       50
#define CALIPILE_SLAVE_ADDRESS      63

#define CALIPILE_VALUES_NUM  1
#define MAX_CALIPILE         9

/******************************************************************************
		GLOBALS
 *****************************************************************************/

// Indicator if Serial output is needed for debugging
static bool serialDebug = false;

// IR sensor registers read variables

typedef struct {
	uint16_t PTAT25;
	uint16_t M;
	uint16_t U0;
	uint32_t UOUT1;
	uint8_t  TOBJ1;
	float    k;
} calipile_config_t;

calipile_config_t calipile_config_info[MAX_CALIPILE];

//static float Tobj;
/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

typedef enum {
	STATUS_DISABLED = 0,
	STATUS_ENABLED,
} sensor_status_t;

typedef enum {
	CHANNEL_0 = 0,
	CHANNEL_1,
	CHANNEL_2,
	CHANNEL_3,
	CHANNEL_4,
	CHANNEL_5,
	CHANNEL_6,
	CHANNEL_7,
	CHANNEL_INVALID = 0xFF
} mux_channel_t;

typedef struct {
	uint8_t mux_addr;
	mux_channel_t channel;
} mux_info_t;

typedef struct {
	uint8_t             address;
	mux_info_t          mux_info; //mux address + channel
	sensor_status_t     enabled;
	uint8_t             samples;
	calipile_config_t * calipile_config;
	//void             (*fault_handler)(void);
} calipile_info_t;

static calipile_info_t calipile_info[] = {
	//address           //mux_info             //status          //samples

#ifdef TCA_I2C_MUX_PRESENT
	{CALIPILE_ADDR_0C,  {TCA_ADDR, CHANNEL_0}, STATUS_ENABLED,  NUM_OF_SAMPLES, &calipile_config_info[0]},
	{CALIPILE_ADDR_0C,  {TCA_ADDR, CHANNEL_1}, STATUS_DISABLED,  NUM_OF_SAMPLES, &calipile_config_info[1]},
	{CALIPILE_ADDR_0C,  {TCA_ADDR, CHANNEL_2}, STATUS_DISABLED,  NUM_OF_SAMPLES, &calipile_config_info[2]},
	{CALIPILE_ADDR_0C,  {TCA_ADDR, CHANNEL_3}, STATUS_DISABLED,  NUM_OF_SAMPLES, &calipile_config_info[3]},
	{CALIPILE_ADDR_0C,  {TCA_ADDR, CHANNEL_4}, STATUS_DISABLED,  NUM_OF_SAMPLES, &calipile_config_info[4]},
	{CALIPILE_ADDR_0C,  {TCA_ADDR, CHANNEL_5}, STATUS_DISABLED,  NUM_OF_SAMPLES, &calipile_config_info[5]},
	{CALIPILE_ADDR_0C,  {TCA_ADDR, CHANNEL_6}, STATUS_DISABLED,  NUM_OF_SAMPLES, &calipile_config_info[6]},
	{CALIPILE_ADDR_0C,  {TCA_ADDR, CHANNEL_7}, STATUS_DISABLED,  NUM_OF_SAMPLES, &calipile_config_info[7]},
#endif //TCA_I2C_MUX_PRESENT
	//TODO: Replace address with CALIPILE_ADDR_0D when working with PCB
	{CALIPILE_ADDR_0D,  {0,CHANNEL_INVALID}, STATUS_ENABLED,  NUM_OF_SAMPLES, &calipile_config_info[8]},
};

/******************************************************************************
		FUNCTIONS
 *****************************************************************************/

#ifdef TCA_I2C_MUX_PRESENT
static void tca_i2c_select(const nrf_drv_twi_t * m_twi, uint8_t channel)
{
  if (channel >= TCA_MAX_CHANNEL) return;
  
  ret_code_t err_code;
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(m_twi, TCA_ADDR, &channel, sizeof(channel), true);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
}
#endif //TCA_I2C_MUX_PRESENT

__STATIC_INLINE void data_handler(uint8_t * data)
{
		// Do nothing
}

/**
 * @brief TWI events handler.
 */
void calipile1385_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
		if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
		{
				data_handler(NULL);
		}
		m_xfer_done = true;
}

// I2C write function for the CaliPile sensor
static void writeByte(const nrf_drv_twi_t * m_twi, uint8_t mux_addr, uint8_t calipile_addr, uint8_t sub_addr, uint8_t data)
{
	ret_code_t err_code;
	
	m_xfer_done = false;
	
	if(mux_addr!=0)
	{
		uint8_t buf[3] = {calipile_addr, sub_addr, data};
		err_code = nrf_drv_twi_tx(m_twi, mux_addr, buf, sizeof(buf), false);
	}
	else
	{
		uint8_t buf[2] = {sub_addr, data};
		err_code = nrf_drv_twi_tx(m_twi, calipile_addr, buf, sizeof(buf), false);
	}
	
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
}

// I2C read function (single byte) for the CaliPile sensor
static uint8_t readByte(const nrf_drv_twi_t * m_twi, uint8_t mux_addr, uint8_t calipile_addr, uint8_t sub_addr)
{
	ret_code_t err_code;
  uint8_t data; // `data` will store the register data   
	
	m_xfer_done = false;
	if (mux_addr!=0)
	{
		uint8_t buff[] = {calipile_addr, sub_addr};
		err_code = nrf_drv_twi_tx(m_twi, mux_addr, buff, sizeof(buff), false);
	}
	else
	{
		err_code = nrf_drv_twi_tx(m_twi, calipile_addr, &sub_addr, sizeof(sub_addr), false);
	}
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
	
	m_xfer_done = false;
	if (mux_addr!=0)
	{
		err_code = nrf_drv_twi_rx(m_twi, mux_addr, &data, sizeof(data));
	}
	else
	{
		err_code = nrf_drv_twi_rx(m_twi, calipile_addr, &data, sizeof(data));
	}
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
	
	return data;                             // Return data read from slave register
}

// I2C read function (multiple bytes) for the CaliPile sensor
static void readBytes(const nrf_drv_twi_t * m_twi, uint8_t mux_addr, uint8_t calipile_addr, uint8_t sub_addr, uint8_t count, uint8_t * dest)
{
	ret_code_t err_code;
  
	m_xfer_done = false;
	if (mux_addr!=0)
	{
		uint8_t buff[] = {calipile_addr, sub_addr};
		err_code = nrf_drv_twi_tx(m_twi, mux_addr, buff, sizeof(buff), false);
	}
	else
	{
		err_code = nrf_drv_twi_tx(m_twi, calipile_addr, &sub_addr, sizeof(sub_addr), false);
	}
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
	
	m_xfer_done = false;
	
	if (mux_addr!=0)
	{
		err_code = nrf_drv_twi_rx(m_twi, mux_addr, dest, count);
	}
	else
	{
		err_code = nrf_drv_twi_rx(m_twi, calipile_addr, dest, count);
	}
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
}

// Initialization function
void calipile1385_setup(const nrf_drv_twi_t * m_twi) 
{
	int recs = sizeof (calipile_info) / sizeof (calipile_info_t);
	for (int i=0; i<recs; i++)
	{
		ret_code_t err_code;
		uint8_t lookUp;
		uint16_t CHECKSUM;
		
		uint8_t subAddress = 0x04;
		
		// Buffer for reading data from registers on IR sensor
		uint8_t rawData[3] = {0, 0, 0};
		
		if(calipile_info[i].enabled!=STATUS_ENABLED)
			continue;
		
		//Initiate I2C transcations with general call/reload command
		m_xfer_done = false;
		err_code = nrf_drv_twi_tx(m_twi, 0, &subAddress, sizeof(subAddress), false);
		APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);
		
		nrf_delay_ms(10);

	 /* Start of EEPROM operations, just have to do once *************************************************** */
	 // Check EEPROM protocol number and slave address as a test of I2C communication 
		writeByte(m_twi, calipile_info[i].mux_info.mux_addr, calipile_info[i].address, CALIPILE_EEPROM_CONTROL, 0x80); // enable EEPROM read
		
		uint8_t c = readByte(m_twi, calipile_info[i].mux_info.mux_addr, calipile_info[i].address, CALIPILE_EEPROM_PROTOCOL);
		NRF_LOG_DEBUG("CaliPile EEPROM protocol number is %d", c); 
		NRF_LOG_DEBUG("CaliPile EEPROM protocol number should be 3"); 

		uint8_t d = readByte(m_twi, calipile_info[i].mux_info.mux_addr, calipile_info[i].address, CALIPILE_SLAVE_ADDRESS);
		NRF_LOG_DEBUG("CaliPile EEPROM slave address is %d", d); 
		NRF_LOG_DEBUG("CaliPile EEPROM slave address should be 140"); 

		// Read the EEPROM calibration constants

		lookUp = readByte(m_twi, calipile_info[i].mux_info.mux_addr, calipile_info[i].address, CALIPILE_EEPROM_LOOKUPNUM);
		NRF_LOG_DEBUG("CaliPile LookUpNumber is %d",lookUp);

		readBytes(m_twi, calipile_info[i].mux_info.mux_addr, calipile_info[i].address, CALIPILE_EEPROM_PTAT25, 2, &rawData[0]);
		calipile_info[i].calipile_config->PTAT25 = ( (uint16_t) (rawData[0] & 0x7F) << 8) | rawData[1];
		NRF_LOG_DEBUG("CaliPile PTAT25 is %d", calipile_info[i].calipile_config->PTAT25);

		readBytes(m_twi, calipile_info[i].mux_info.mux_addr, calipile_info[i].address, CALIPILE_EEPROM_M, 2, &rawData[0]);
		calipile_info[i].calipile_config->M = ( (uint16_t) rawData[0] << 8) | rawData[1];
		calipile_info[i].calipile_config->M /= 100;
		NRF_LOG_DEBUG("CaliPile M is %d", calipile_info[i].calipile_config->M);

		readBytes(m_twi, calipile_info[i].mux_info.mux_addr, calipile_info[i].address, CALIPILE_EEPROM_U0, 2, &rawData[0]);
		calipile_info[i].calipile_config->U0 = ( (uint16_t) rawData[0] << 8) | rawData[1];
		calipile_info[i].calipile_config->U0 += 32768;
		NRF_LOG_DEBUG("CaliPile U0 is %d", calipile_info[i].calipile_config->U0);

		readBytes(m_twi, calipile_info[i].mux_info.mux_addr, calipile_info[i].address, CALIPILE_EEPROM_UOUT1, 2, &rawData[0]);
		calipile_info[i].calipile_config->UOUT1 = ( (uint16_t) rawData[0] << 8) | rawData[1];
		calipile_info[i].calipile_config->UOUT1 *= 2;
		NRF_LOG_DEBUG("CaliPile UOUT1 is %d", calipile_info[i].calipile_config->UOUT1);

		calipile_info[i].calipile_config->TOBJ1 = readByte(m_twi, calipile_info[i].mux_info.mux_addr, calipile_info[i].address, CALIPILE_EEPROM_TOBJ1);
		NRF_LOG_DEBUG("CaliPile TOBJ1 is %d", calipile_info[i].calipile_config->TOBJ1);

		readBytes(m_twi, calipile_info[i].mux_info.mux_addr, calipile_info[i].address, CALIPILE_EEPROM_CHECKSUM, 2, &rawData[0]);
		CHECKSUM = ( (uint16_t) rawData[0] << 8) | rawData[1];
		NRF_LOG_DEBUG("CaliPile CHECKSUM is supposed to be %d", CHECKSUM);

		// Calculate the checksum
		uint16_t sum = 0;
		for(int ii = 35; ii < 64; ii++)
		{
		 sum += readByte(m_twi, calipile_info[i].mux_info.mux_addr, calipile_info[i].address, ii);
		}
		NRF_LOG_DEBUG("CaliPile CHECKSUM is %d", sum + c);

		writeByte(m_twi, calipile_info[i].mux_info.mux_addr, calipile_info[i].address, CALIPILE_EEPROM_CONTROL, 0x00); // disable EEPROM read
		/* End of EEPROM operations, just have to do once *************************************************** */

		// Construct needed calibration constant "k" used to calculate Object temperature
		calipile_info[i].calipile_config->k = ( (float) (calipile_info[i].calipile_config->UOUT1 - calipile_info[i].calipile_config->U0) )/(powf((float)(calipile_info[i].calipile_config->TOBJ1 + 273.15f), 3.8f) - powf(25.0f + 273.15f, 3.8f) );
		
		NRF_LOG_FLUSH();
	}
}

// Function that returns the object temperature measured by the IR sensor
static float read_calipile1835(const nrf_drv_twi_t * m_twi, calipile_info_t * calipile_rec, float * sensor_data)
{
	float temp_samples[NUM_OF_SAMPLES];
	uint32_t TPOBJ;
	uint16_t TPAMB;
	// Temperature read by the IR sensor
	float Tamb;
	
	// Buffer for reading data from registers on IR sensor
	uint8_t rawData[3] = {0, 0, 0};
	
	for (int i=0; i<NUM_OF_SAMPLES; i++)
	{
		// read the Ambient temperature
		readBytes(m_twi, calipile_rec->mux_info.mux_addr, calipile_rec->address, CALIPILE_TPAMBIENT, 2, &rawData[0]);
		TPAMB = ( (uint16_t)(rawData[0] & 0x7F) << 8) | rawData[1] ; 
		// Calculate Ambient temperature to be used in Object temperature calculatatio later
		Tamb = 298.15f + ((float)TPAMB - (float)calipile_rec->calipile_config->PTAT25) * (1.0f/(float) calipile_rec->calipile_config->M);

		// read the Object temperature
		readBytes(m_twi, calipile_rec->mux_info.mux_addr, calipile_rec->address, CALIPILE_TPOBJECT, 3, &rawData[0]);
		TPOBJ = ( (uint32_t) ( (uint32_t)rawData[0] << 24) | ( (uint32_t)rawData[1] << 16) | ( (uint32_t)rawData[2] & 0x80) << 8) >> 15; 

		// calculate Object temperature "tempObj"
		float temp0 = powf(Tamb, 3.8f);
		float temp1 = ( ((float) TPOBJ) - ((float) calipile_rec->calipile_config->U0)  ) / calipile_rec->calipile_config->k ;
		float tempObj = powf( (temp0 + temp1), 0.2631578947f );

		if(serialDebug) 
		{
			NRF_LOG_DEBUG("-------------------------")
			NRF_LOG_DEBUG("Tambient = "NRF_LOG_FLOAT_MARKER" K", NRF_LOG_FLOAT(Tamb));
			NRF_LOG_DEBUG("TPAMB = %d", TPAMB);   
			NRF_LOG_DEBUG("Tobj = "NRF_LOG_FLOAT_MARKER" K", NRF_LOG_FLOAT(tempObj));
			NRF_LOG_DEBUG("Tobj Celsius = "NRF_LOG_FLOAT_MARKER" C", NRF_LOG_FLOAT(tempObj - 273.15f));
			NRF_LOG_DEBUG("TPOBJ = %d", TPOBJ);
			NRF_LOG_FLUSH();
		}
		
		// In celsius
		temp_samples[i] = tempObj - 273.15f;
	}
	
	sensor_data[0] = calculateSensorValue(temp_samples, NUM_OF_SAMPLES);
	NRF_LOG_DEBUG("Tobj Celsius = "NRF_LOG_FLOAT_MARKER" C", NRF_LOG_FLOAT(sensor_data[0]));
	
  return sensor_data[0];
}

int read_all_calipile(const nrf_drv_twi_t * m_twi, float * sensor_data)
{
	int count = 0;
	
	int recs = sizeof(calipile_info)/sizeof (calipile_info_t);
	
	for (int i=0; i<recs; i++)
	{
		// check next if disabled
		if(calipile_info[i].enabled!=STATUS_ENABLED)
		{
			// TODO: Remove forced success read data.
			count += CALIPILE_VALUES_NUM;
			sensor_data += CALIPILE_VALUES_NUM;
			continue;
		}
		
		// Read sht3x
		read_calipile1835(m_twi, &calipile_info[i], sensor_data);
		
		sensor_data += CALIPILE_VALUES_NUM;
		count += CALIPILE_VALUES_NUM;
	}
	
	return count;
}
