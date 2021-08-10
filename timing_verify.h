#pragma once

#ifndef _TIMING_VERIFY_H_INCLUDED_
#define _TIMING_VERIFY_H_INCLUDED_

#define FREQUENCY_SAMPLES  10
#define FREQUENCY_SETS      5

#include <stdint.h>
#include <stdbool.h>

// 
// This is the beginning of an API to measure timing so that we can lower our dependency 
// on the lab oscilloscope.
//
// You will need to figure how to feed the funcion and how to calibrate it.  I want you 
// to do it empirically for now but we’ll use some high resolution timers in the future
//

void timing_init(void);

// 
// Will drop a timing datapoint for set 0-(FREQUENCY_SETS-1)
//
void timing_freq_datapoint(uint32_t set, uint32_t val);

// 
// Will return the average frequency of the prior FREQUENCY_SAMPLES
// data points for a frequency set
//
uint32_t timing_freq_measurement_hz(uint32_t set);

//
// Used for easily testing that a fewquency is in range 
// return true if frequency of set 'set' is with Hi and Low parameters
//
int timing_verify_frequency_in_range(uint32_t set, uint32_t low_mhz_inclusive,uint32_t high_mhz_inclusive);


#endif // _TIMING_VERIFY_H_INCLUDED_
