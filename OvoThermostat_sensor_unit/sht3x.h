#ifndef __SHT3X_H__
#define __SHT3X_H__

#define SHT3X_ADDR_44  0x44
#define SHT3X_ADDR_45  0x45

#define SHT3X_VALUES_NUM 2

void sht3x_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
int read_all_sht3x(const nrf_drv_twi_t * m_twi, float * sensor_buf);
int read_Humidity(void);
int read_Temp(void);

#endif //__SHT3X_H__
