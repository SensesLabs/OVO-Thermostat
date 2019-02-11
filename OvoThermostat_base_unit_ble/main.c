/***************************************************************************************/
/*
 * beacon_scanner
 * Created by Manuel Montenegro, Sep 7, 2018.
 *
 *  This is a Bluetooth 5 scanner. This code reads every advertisement from beacons
 *  and sends its data through serial port.
 *
 *  This code has been developed for Nordic Semiconductor nRF52840 PDK.
*/
/***************************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_serial.h"
#include "nrf_pwr_mgmt.h"
#include "app_util.h"
#include "app_error.h"
#include "ble_dis_c.h"
#include "ble_rscs_c.h"
#include "app_util.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define APP_BLE_CONN_CFG_TAG        1                                   /**< Tag that identifies the BLE configuration of the SoftDevice. */
#define APP_BLE_OBSERVER_PRIO       3                                   /**< BLE observer priority of the application. There is no need to modify this value. */
#define APP_SOC_OBSERVER_PRIO       1                                   /**< SoC observer priority of the application. There is no need to modify this value. */

#define SCAN_INTERVAL               0x0640                              /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 0x0640                              /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION           	0x0000                              /**< Duration of the scanning in units of 10 milliseconds. If set to 0x0000, scanning continues until it is explicitly disabled. */

/**< All sensor data buffer size */
#define SENSOR_DATA_BUF_SIZE        15

#define SENSOR_VALUES_START_BYTE    9

NRF_BLE_SCAN_DEF(m_scan);                                   /**< Scanning Module instance. */

static bool                  m_memory_access_in_progress;   /**< Flag to keep track of ongoing operations on persistent memory. */

static ble_gap_scan_params_t m_scan_param =                 /**< Scan parameters requested for scanning and connection. */
{
    .active        = 0x00,
    .interval      = SCAN_INTERVAL,
    .window        = SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .timeout       = SCAN_DURATION,
//    .scan_phys     = BLE_GAP_PHY_CODED,                                 // Choose only one of the following scan_phys
    .scan_phys     = BLE_GAP_PHY_1MBPS,
//    .scan_phys     = BLE_GAP_PHY_2MBPS,
    .extended      = 1,
};

static void scan_start(void);

// UARTE1 Functions

static void sleep_handler(void)
{
    __WFE();
    __SEV();
    __WFE();
}

/* UARTE1 Instance */
NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uarte1_drv_config,
                      UARTE1_RX_PIN_NUMBER/*Rx*/, UARTE1_TX_PIN_NUMBER/*Tx*/,
                      RTS_PIN_NUMBER, CTS_PIN_NUMBER,
                      NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_115200,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);
											
#define SERIAL_FIFO_TX_SIZE 256
#define SERIAL_FIFO_RX_SIZE 256

#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 1

NRF_SERIAL_QUEUES_DEF(serial1_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);

NRF_SERIAL_BUFFERS_DEF(serial1_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

NRF_SERIAL_CONFIG_DEF(serial1_config, NRF_SERIAL_MODE_DMA,
                      &serial1_queues, &serial1_buffs, NULL, sleep_handler);

NRF_SERIAL_UART_DEF(serial1_uarte, 1);

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
float sensor_data[SENSOR_DATA_BUF_SIZE];
bool update_sensor_data = false;

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            if(m_scan.scan_buffer.p_data[2] == 0x04)
            {
              memcpy(sensor_data, &m_scan.scan_buffer.p_data[SENSOR_VALUES_START_BYTE], SENSOR_DATA_BUF_SIZE*sizeof(uint32_t));
              update_sensor_data = true;
            
  //            NRF_LOG_RAW_HEXDUMP_INFO (m_scan.scan_buffer_data, NRF_BLE_SCAN_BUFFER);
              NRF_LOG_RAW_HEXDUMP_INFO (m_scan.scan_buffer.p_data, m_scan.scan_buffer.len);
              NRF_LOG_RAW_INFO ("----------------------------\r\n");
            }
/*              if(m_scan.scan_buffer.p_data[2] ==  0x04)
              {
                  NRF_LOG_RAW_INFO ("Above Data is sensor data----------------------------------\r\n");
              }
              else
              {
                  NRF_LOG_RAW_INFO ("Other Device Data ----------------------------------\r\n");
              }
*/
  //          }
  //          else
  //          {
  //            memset(m_scan.scan_buffer.p_data, 0, m_scan.scan_buffer.p_data);
  //          }
        }
	break;
				
        default:
            break;
    }
}


/**
 * @brief SoftDevice SoC event handler.
 *
 * @param[in] evt_id    SoC event.
 * @param[in] p_context Context.
 */
static void soc_evt_handler(uint32_t evt_id, void * p_context)
{
    switch (evt_id)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        /* fall through */
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
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

    // Register handlers for BLE and SoC events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);
}

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            NRF_LOG_INFO("Scan timed out.");
            scan_start();
        } break;

        default:
          break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
    init_scan.p_scan_param     = &m_scan_param;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    // If there is any pending write to flash, defer scanning until it completes.
    if (nrf_fstorage_is_busy(NULL))
    {
        m_memory_access_in_progress = true;
        return;
    }

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
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
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


int main(void)
{
		ret_code_t ret;
	
    // Initialize.
    log_init();
    timer_init();
    power_management_init();
    ble_stack_init();
    scan_init();

    // Start execution.
    NRF_LOG_RAW_INFO("OvoThermostat: Base unit scan started\r\n");

    // Uart1 Init
    ret = nrf_serial_init(&serial1_uarte, &m_uarte1_drv_config, &serial1_config);
    APP_ERROR_CHECK(ret);
		
    //char welcome[] = "welcome\n";
    //ret = nrf_serial_write(&serial1_uarte, welcome, sizeof(welcome), NULL, 0);
    //ret = nrf_serial_flush(&serial1_uarte, 0);
	
    scan_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
				
        if (update_sensor_data)
        {
            (void)nrf_serial_write(&serial1_uarte, sensor_data, sizeof(sensor_data), NULL, 0);
            (void)nrf_serial_flush(&serial1_uarte, 0);

            NRF_LOG_RAW_INFO("---sensor data---\r\n");
            for (int log_index=0; log_index<SENSOR_DATA_BUF_SIZE; log_index++)
            {
              NRF_LOG_RAW_INFO("["NRF_LOG_FLOAT_MARKER"]\r\n", NRF_LOG_FLOAT(sensor_data[log_index]));
            }

            //NRF_LOG_RAW_INFO("sht3x_temp = "NRF_LOG_FLOAT_MARKER"\r\n", NRF_LOG_FLOAT(sensor_data[0]));
            //NRF_LOG_RAW_INFO("sht3x_humi = "NRF_LOG_FLOAT_MARKER"\r\n", NRF_LOG_FLOAT(sensor_data[1]));
            //NRF_LOG_RAW_INFO("calip_temp = "NRF_LOG_FLOAT_MARKER"\r\n", NRF_LOG_FLOAT(sensor_data[2]));

            update_sensor_data=false;
        }
    }
}
