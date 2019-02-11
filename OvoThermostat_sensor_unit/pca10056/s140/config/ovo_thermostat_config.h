#ifndef __OVO_THERMOSTAT_CONFIG_H__
#define __OVO_THERMOSTAT_CONFIG_H__

// This macro makes code compatible to run on ovo thermostat board
//#undef OVO_THERMOSTAT_BOARD
#ifndef OVO_THERMOSTAT_BOARD
#define OVO_THERMOSTAT_BOARD
#endif

// TODO: Do not change this value until dynamic memory
// allocation is implemented
#define NUM_OF_SAMPLES 5

// If OVO_THERMOSTAT_BOARD is enabled, also enable supporting macro switch
#ifdef OVO_THERMOSTAT_BOARD

// Enable/Disable I2C mux
#define TCA_I2C_MUX_PRESENT

#endif //OVO_THERMOSTAT_BOARD

#endif //__OVO_THERMOSTAT_CONFIG_H__
