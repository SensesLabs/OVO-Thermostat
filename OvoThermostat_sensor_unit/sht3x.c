#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ovo_thermostat_config.h"
#include "sht3x.h"
#include "sd.h"

/* Common addresses definition for temperature sensor. */
//#define SHT3X_ADDR          (0x44) //ADDR (pin 2) connected to VSS
#define SHT3X_CONFIG1       (0x2C)
#define SHT3X_CONFIG2       (0x06)

typedef enum {
	STATUS_DISABLED = 0,
	STATUS_ENABLED,
} sensor_status_t;

typedef struct {
	uint8_t          address;
	sensor_status_t  enabled;
	uint8_t          samples;
	//void             (*fault_handler)(void);
} sht3x_info_t;

static sht3x_info_t sht3x_info [] = {
	{SHT3X_ADDR_44,  STATUS_ENABLED,  NUM_OF_SAMPLES},
	{SHT3X_ADDR_45,  STATUS_ENABLED,  NUM_OF_SAMPLES},
};

/* Indicates if operation on TWI has ended. */
/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

__STATIC_INLINE void data_handler(uint8_t * data)
{
		// Do nothing
}

/**
 * @brief TWI events handler.
 */
void sht3x_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
                if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
		{
				data_handler(NULL);
		}
		m_xfer_done = true;
}

static void SHT3X_set_mode(const nrf_drv_twi_t * m_twi, uint8_t address)
{
    ret_code_t err_code;

    /* High repeatibility */
    uint8_t reg[2] = {SHT3X_CONFIG1, SHT3X_CONFIG2};
    err_code = nrf_drv_twi_tx(m_twi, address, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

bool CRC8(uint8_t MSB, uint8_t LSB, uint8_t CRC)
{
	/*
	*	Name  : CRC-8
	*	Poly  : 0x31	x^8 + x^5 + x^4 + 1
	*	Init  : 0xFF
	*	Revert: false
	*	XorOut: 0x00
	*	Check : for 0xBE,0xEF CRC is 0x92
	*/
	uint8_t crc = 0xFF;
	uint8_t i;
	crc ^=MSB;

		for (i = 0; i < 8; i++)
			crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;

	crc ^= LSB;
	for (i = 0; i < 8; i++)
				crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;

	if (crc==CRC)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * @brief Function for reading data from temperature/humidity sensor.
 */
static int read_sht3x(const nrf_drv_twi_t * m_twi, sht3x_info_t * sht3x_rec, float * sensor_buf)
{
	uint8_t rx_buf[6];
	
	// TODO: Allocate memory to sample buffers as per parameter 'samples' size
	float temp_samples[NUM_OF_SAMPLES];
	float humi_samples[NUM_OF_SAMPLES];
	
	for(uint8_t i=0; i<NUM_OF_SAMPLES; i++)
	{
		// reset event flag
		m_xfer_done = false;

		// set mode
		SHT3X_set_mode(m_twi, sht3x_rec->address);
		nrf_delay_ms(50);
	        
		// reset event flag
		m_xfer_done = false;
		
		// read sht3x data
		ret_code_t err_code = nrf_drv_twi_rx(m_twi, sht3x_rec->address, rx_buf, sizeof(rx_buf));
		APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);
		
		// convert to real values
		int temp = (rx_buf[0] * 256 + rx_buf[1]);
		
		temp_samples[i] = -45 + (175 * temp / 65535.0);
		humi_samples[i] = 100 * (rx_buf[3] * 256 + rx_buf[4]) / 65535.0;
	}
	
	sensor_buf[0] = calculateSensorValue(temp_samples, NUM_OF_SAMPLES);
	sensor_buf[1] = calculateSensorValue(humi_samples, NUM_OF_SAMPLES);
	NRF_LOG_DEBUG("temp="NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(sensor_buf[0]));
	NRF_LOG_DEBUG("humi="NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(sensor_buf[1]));
	
	return SHT3X_VALUES_NUM;
}

int read_all_sht3x(const nrf_drv_twi_t * m_twi, float * sensor_data)
{
	int count = 0;
	
	int recs = sizeof(sht3x_info)/sizeof (sht3x_info_t);
	
	for (int i=0; i<recs; i++)
	{
		// check next if disabled
		if(sht3x_info[i].enabled!=STATUS_ENABLED)
		{
			// TODO: Remove forced success read data.
			count += SHT3X_VALUES_NUM;
			sensor_data += SHT3X_VALUES_NUM;
			continue;
		}
		
		// Read sht3x
		read_sht3x(m_twi, &sht3x_info[i], sensor_data);
		
		sensor_data += SHT3X_VALUES_NUM;
		count += SHT3X_VALUES_NUM;
	}
	
	return count;
}
