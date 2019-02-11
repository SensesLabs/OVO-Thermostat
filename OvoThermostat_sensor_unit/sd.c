
#include <math.h>

#include "bsp.h"

#include "ovo_thermostat_config.h"
#include "sd.h"

/* Calculate mean value of an array of floats */
static float calculateAverage(float data[], uint8_t samples)
{
  float sum = 0.0;
  
  /* Calculate sum of values */
  for(int i=0; i < NUM_OF_SAMPLES; ++i) 
	{
    sum += data[i];
  }
	
  /* Calculate mean */
  return(sum / NUM_OF_SAMPLES);
}

/* Calculate the standard deviation of an array of floats */
static float calculateStandardDeviation(float data[], uint8_t samples)
{
  float sum = 0.0, mean, standardDeviation = 0.0;

  /* Calculate sum of values */
  for(int i=0; i < NUM_OF_SAMPLES; ++i) 
	{
    sum += data[i];
  }
  /* Calculate mean */
  mean = sum / NUM_OF_SAMPLES;

  /* Calculate and return standard deviation */
  for(int i=0; i < NUM_OF_SAMPLES; ++i) 
	{
    standardDeviation += pow(data[i] - mean, 2);
  }
  return sqrt(standardDeviation / NUM_OF_SAMPLES);
}

/* Calculate the average of the sensor readings that are inside the standard deviation */
float calculateSensorValue(float data[], uint8_t samples)
{
  int i;
  int numberOfGoodValues = 0; // Number of values that are inside the standard deviation and are kept 
  float sum = 0.0;  // Sum of the good values (inside the standard devition)
  float mean = calculateAverage(data, NUM_OF_SAMPLES);  // Mean value of all sensor readings
  float standardDeviation = calculateStandardDeviation(data, NUM_OF_SAMPLES);  // Standard deviation from all sensor reading values
	
  /* Calculate the sum of the good values (inside the standard devition) */
  for(i=0; i < NUM_OF_SAMPLES; ++i) 
	{
    /* If the sensor value is closer to the mean value by less than the standard deviation, we keep this value */
    if (fabs(data[i] - mean) <= standardDeviation)
		{
      sum += data[i];
      /* Increase the counter indicating how many values are considered good */
      numberOfGoodValues++;  
    }
  }
  return (sum / numberOfGoodValues);
}
