
#ifndef __SD_H__
#define __SD_H__

/* Calculate the average of the sensor readings that are inside the standard deviation */
float calculateSensorValue(float data[], uint8_t samples);

#endif //__SD_H__
