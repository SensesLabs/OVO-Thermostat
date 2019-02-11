#ifndef __CALIPILE1385_H__
#define __CALIPILE1385_H__

//#ifdef TCA_I2C_MUX_PRESENT
#define TCA_ADDR                    0x70
#define TCA_MAX_CHANNEL             8
//#endif //TCA_I2C_MUX_PRESENT

#define CALIPILE_ADDR_0C          0x0C
#define CALIPILE_ADDR_0D          0x0D

void calipile1385_setup(const nrf_drv_twi_t * m_twi);
int read_all_calipile(const nrf_drv_twi_t * m_twi, float * sensor_data);
void calipile1385_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);

#endif //__CALIPILE1385_H__
