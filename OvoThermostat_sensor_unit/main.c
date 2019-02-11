/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Ovo Thermostat sensor unit main file.
 *
 * This file contains the source code for an sensor unit application.
 * Ovo thermostat's sensor unit acts as a beacon and transmits sensor data
 * every interval.
 */

/******************************************************************************
		INCLUDES
 *****************************************************************************/

/*library*/
#include <stdbool.h>
#include <stdint.h>

/*nordic*/
#include "nordic_common.h"
#include "bsp.h"
#include "mem_manager.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "nrf_serial.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "app_error.h"
#include "app_util.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/*ovo thermostat*/
#include "ovo_thermostat_config.h"
#include "sht3x.h"
#include "sts30.h"
#include "calipile1385.h"
#include "lp55231.h"

/******************************************************************************
		CONSTANTs
 *****************************************************************************/

/**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_CONN_CFG_TAG            1

/**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(2500, UNIT_0_625_MS)

/**< Total length of information advertised by the Beacon. */
#define APP_BEACON_INFO_LENGTH          180 //0x1F //0x17

/**< Length of manufacturer specific data in the advertisement. */
#define APP_ADV_DATA_LENGTH             0x1D //0x15

/**< 0x02 refers to Beacon. */
#define APP_DEVICE_TYPE                 0x02

/**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_MEASURED_RSSI               0xC3

/**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_COMPANY_IDENTIFIER          0x0059

/**< Major value used to identify Beacons. */
#define APP_MAJOR_VALUE                 0x01, 0x02

/**< Minor value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04

/**< Proprietary UUID for Beacon. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0, \
																				0x11, 0x22, 0x33, 0x44, \
																				0x55, 0x66, 0x77, 0x88

/**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define DEAD_BEEF                       0xDEADBEEF

/**< Interval at which all sensors to be read. */
#define BEACON_INTERVAL                 APP_TIMER_TICKS(3000)

/**< All sensor data buffer size */
#define SENSOR_DATA_BUF_SIZE            15
#define SENSOR_DATA_MAX_SIZE_PER_SENSOR 5

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
/**< Position of the MSB of the Major Value in m_beacon_info array. */
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18
/**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#define UICR_ADDRESS                    0x10001080
#endif

/**< TWI instance ID. */
#define TWI_INSTANCE_ID                 0

/**< SPI instance index. */
#define SPI_INSTANCE                    1

/******************************************************************************
		USER DEFINED
 *****************************************************************************/

typedef enum {
	SENSOR_NONE = 0,
	SENSOR_SHT3X,
	SENSOR_STS31,
	SENSOR_CALIPILE_1385,
	
	// Add new sensor above
	SENSOR_MAX
} sensor_t;


/******************************************************************************
		GLOBALS
 *****************************************************************************/

//static volatile sensor_t current_sensor = SENSOR_NONE;
static volatile sensor_t current_sensor = SENSOR_SHT3X;
static volatile bool g_setAdvData = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* SPI instance */
static const nrf_drv_spi_t m_spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
//static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED];
/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        //.len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
				.len    = BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0
    }
};

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};

/**< Flag used to indicate that SPI instance completed the transfer. */
static volatile bool spi_xfer_done;

#define TEST_STRING "Nordic"
//static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
//static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

/******************************************************************************
		FUNCTIONS
 *****************************************************************************/

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

void load_beacon_adv_data(float * sensor_buf)
{
    memcpy (&m_beacon_info[2], sensor_buf, SENSOR_DATA_BUF_SIZE*sizeof(float));
//    memcpy (&m_beacon_info[2], sensor_buf, SENSOR_DATA_BUF_SIZE*sizeof(uint32_t));
//    NRF_LOG_DEBUG("==================================");
//    NRF_LOG_HEXDUMP_DEBUG(&m_beacon_info[2], SENSOR_DATA_BUF_SIZE*sizeof(uint32_t));
//    NRF_LOG_DEBUG("==================================");
}

APP_TIMER_DEF(m_beacon_timer_id);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for starting application timers.
 */
static void application_timer_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_beacon_timer_id, BEACON_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}
	
void set_adv_sensor_data(bool sensor_init)
{
		uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    float         sensor_data[SENSOR_DATA_BUF_SIZE];
    uint8_t       index = 0;

    ble_advdata_manuf_data_t manuf_specific_data;

    // Clear buffer
    index=0;
    memset(sensor_data, 0, sizeof sensor_data);
    
    // All sensor initialized?
    if (sensor_init)
    {
            current_sensor=SENSOR_SHT3X;
            index += read_all_sht3x(&m_twi, &sensor_data);
            
 //           current_sensor=SENSOR_STS31;
 //           index += read_all_sts31(&m_twi, sensor_data);
            
            current_sensor=SENSOR_CALIPILE_1385;
            index += read_all_calipile(&m_twi, &sensor_data[index]);

        float testdata = 10.0;
        for(uint8_t cntr=index; cntr<SENSOR_DATA_BUF_SIZE; cntr++)
        {
            sensor_data[cntr] = testdata;
            testdata += 1.0f;
        }
    }
    
    NRF_LOG_DEBUG("--- SU: sensor data ---");
    NRF_LOG_FLUSH();
    for (uint8_t log_index=0; log_index<SENSOR_DATA_BUF_SIZE; log_index++)
    {
//      NRF_LOG_HEXDUMP_DEBUG(sensor_data[log_index], sizeof(uint32_t));
//      NRF_LOG_DEBUG("["NRF_LOG_FLOAT_MARKER"]", NRF_LOG_FLOAT(sensor_data[log_index]));
    }
    NRF_LOG_FLUSH();
    
    // load beacon info
    load_beacon_adv_data(sensor_data);
		
    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;	
    manuf_specific_data.data.p_data = (uint8_t *)&m_beacon_info;
    manuf_specific_data.data.size   = 80;//APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

//    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.include_appearance    = false;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    NRF_LOG_DEBUG("==================================");
    NRF_LOG_HEXDUMP_DEBUG(m_adv_data.adv_data.p_data, m_adv_data.adv_data.len);
    NRF_LOG_DEBUG("==================================");

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    //ble_advdata_t advdata;
    //uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    //ble_advdata_manuf_data_t manuf_specific_data;
    //manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    //manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    //manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    //memset(&advdata, 0, sizeof(advdata));

    //advdata.name_type             = BLE_ADVDATA_NO_NAME;
    //advdata.flags                 = flags;
    //advdata.p_manuf_specific_data = &manuf_specific_data;

    set_adv_sensor_data(false);
		
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    //m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    m_adv_params.secondary_phy   = BLE_GAP_PHY_2MBPS;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = 0;       // Never time out.

    //err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    //APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();  
	
		APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}

static void beacon_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    // set update data flag
    g_setAdvData = true;
}

/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
	
	  // Create timer to periodically update sensor advertisement packet.
    err_code = app_timer_create(&m_beacon_timer_id, APP_TIMER_MODE_REPEATED, beacon_timeout_handler);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{

    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
						if (*(sensor_t*)p_context==SENSOR_SHT3X)
						{	
							sht3x_handler(p_event, p_context);
						}
						else if(*(sensor_t*)p_context==SENSOR_STS31)
						{
							sts3x_handler(p_event, p_context);
						}
						else if(*(sensor_t*)p_context==SENSOR_CALIPILE_1385)
						{
							calipile1385_handler(p_event, p_context);
						}
        default:
						//NRF_LOG_INFO("INV_EVT\n");
            break;
    }
}

void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, (void*)&current_sensor);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void spi_init (void)
{
		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    //spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&m_spi, &spi_config, spi_event_handler, NULL));
}

/**
 * @brief Function for application main entry.
 */

int main(void)
{
        nrf_gpio_cfg_output(POWER_GATE_BOOST_PIN);
        nrf_gpio_cfg_output(POWER_GATE_SHT30_PIN);
        nrf_gpio_cfg_output(POWER_GATE_SENSOR_TOP_PIN);
        
        nrf_gpio_pin_set(POWER_GATE_BOOST_PIN);
        nrf_gpio_pin_set(POWER_GATE_SHT30_PIN);
        nrf_gpio_pin_set(POWER_GATE_SENSOR_TOP_PIN);

        //ret_code_t ret;
	
	// Initialize.
	log_init();
	timers_init();
	leds_init();
	power_management_init();
	ble_stack_init();
	advertising_init();
                
	twi_init();
//	spi_init();

	// Fixed memory allocation init
	// TODO: Fixed memory allocation does not work. Need to study more on this issue.
	//nrf_mem_init ();
	
	// Device setup
	current_sensor=SENSOR_CALIPILE_1385;
	calipile1385_setup(&m_twi);
	
	// Start Timer
	application_timer_start();
	
	// Start execution.
	NRF_LOG_INFO("OvoThermostat: sensor unit advertisement started");
        NRF_LOG_FLUSH();
	advertising_start();
	
	// Delay after all setup.
	//TODO: Remove if systems works without delay
	nrf_delay_ms(100);

	//Test LP55231
	//lp55231_config(&m_twi);
	//test_lp55231(&m_twi);
	
	// Enter main loop.
	for (;;)
	{
#ifdef SPI_TEST
		// Reset rx buffer and transfer done flag
		memset(m_rx_buf, 0, m_length);
		spi_xfer_done = false;

		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));

		while (!spi_xfer_done)
		{
				__WFE();
		}

		NRF_LOG_FLUSH();

		bsp_board_led_invert(BSP_BOARD_LED_0);
		nrf_delay_ms(200);
#endif //SPI_TEST
		
		// one-shot flag to read all sensor and set adv data
		if(g_setAdvData)
		{
			set_adv_sensor_data(true);
			g_setAdvData = false;
		}

                idle_state_handle();
	}

}

/**
 * @}
 */
