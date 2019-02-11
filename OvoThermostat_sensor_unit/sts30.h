#ifndef __STS3X_H__
#define __STS3X_H__

void sts3x_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
int read_all_sts31(const nrf_drv_twi_t * m_twi, float * sensor_data);

#endif //__SHT3X_H__
