#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ovo_thermostat_config.h"
#include "sd.h"
#include "sts30.h"

/* Common addresses definition for temperature sensor. */
#define STS3X_ADDR          (0x4A) //ADDR (pin 2) connected to VSS
#define STS3X_CONFIG1       (0x2C) //STS3X_ACCURACY_HIGH
#define STS3X_CONFIG2       (0x06)
#define EXPECTED_DATA_SIZE  (3)
#define STS31_VALUES_NUM    (1)

typedef enum {
	STATUS_DISABLED = 0,
	STATUS_ENABLED,
} sensor_status_t;

typedef struct {
	uint8_t          address;
	sensor_status_t  enabled;
	uint8_t          samples;
	//void             (*fault_handler)(void);
} sts31_info_t;

static sts31_info_t sts31_info [] = {
	{STS3X_ADDR,  STATUS_DISABLED,   NUM_OF_SAMPLES},
};

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;


/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */

__STATIC_INLINE void data_handler(uint8_t * data)
{
	// Do nothing
}

/**
 * @brief TWI events handler.
 */
void sts3x_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
		if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
		{
				data_handler(NULL);
		}
		m_xfer_done = true;
}

static void STS3X_set_mode(const nrf_drv_twi_t * m_twi, uint8_t sts31_addr)
{
    ret_code_t err_code;

    /* High repeatibility */
    uint8_t reg[2] = {STS3X_CONFIG1, STS3X_CONFIG2};
    err_code = nrf_drv_twi_tx(m_twi, sts31_addr, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

/**
 * @brief Function for reading data from temperature/humidity sensor.
 */
static void read_sts3x(const nrf_drv_twi_t * m_twi, sts31_info_t * p_sts31_info, float * sensor_data)
{
	uint8_t data[EXPECTED_DATA_SIZE];
	float   temp_samples[NUM_OF_SAMPLES];
	
	for (int i=0; i<NUM_OF_SAMPLES; i++)
	{
		m_xfer_done = false;
		STS3X_set_mode(m_twi, p_sts31_info->address);
		
		// TODO: Remove delay
		nrf_delay_ms(50);
		
		m_xfer_done = false;
		ret_code_t err_code = nrf_drv_twi_rx(m_twi, p_sts31_info->address, data, sizeof(data));
		APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);

		temp_samples[i] = -45 + (175 * ((data[0] * 256 + data[1])) / 65535.0);
	}

	sensor_data[0] = calculateSensorValue(temp_samples, NUM_OF_SAMPLES);
	
	// Output data to screen
	//NRF_LOG_DEBUG("Temperature in Celsius : "NRF_LOG_FLOAT_MARKER" C \n", NRF_LOG_FLOAT(sensor_data[0]));
	//NRF_LOG_FLUSH();
}

int read_all_sts31(const nrf_drv_twi_t * m_twi, float * sensor_data)
{
	int count = 0;
	
	int recs = sizeof(sts31_info)/sizeof (sts31_info_t);
	
	for (int i=0; i<recs; i++)
	{
		// check next if disabled
		if(sts31_info[i].enabled!=STATUS_ENABLED)
		{
			// TODO: Remove forced success read data.
			count += STS31_VALUES_NUM;
			sensor_data += STS31_VALUES_NUM;
			continue;
		}
		
		// Read sht3x
		read_sts3x(m_twi, &sts31_info[i], sensor_data);
		
		sensor_data += STS31_VALUES_NUM;
		count += STS31_VALUES_NUM;
	}
	
	return count;
}

