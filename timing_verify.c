#include <stdint.h>
#include <stdlib.h>
#include "timing_verify.h"
#define TRUE 1
#define FALSE 0

uint32_t frequency_table[FREQUENCY_SETS][FREQUENCY_SAMPLES];
int last_visited_sample[FREQUENCY_SETS];

void timing_init(void) {
    for ( int i=0; i < FREQUENCY_SETS; i++) {
        last_visited_sample[i] = FREQUENCY_SAMPLES - 1;
        for ( int j=0; j < FREQUENCY_SAMPLES; j++) {
            frequency_table[i][j] = 0;
        }
    }
}

//// Will drop a timing datapoint for set 0-(FREQUENCY_SETS-1)//
void timing_freq_datapoint(uint32_t set, uint32_t val) {
    int current_sample = (last_visited_sample[set] + 1) % FREQUENCY_SAMPLES;
    frequency_table[ set ][ current_sample ] = val;
    last_visited_sample[set] = current_sample;
}

//
// Will return the average frequency of the prior FREQUENCY_SAMPLES
// data points for a frequency set
//
uint32_t timing_freq_measurement_hz(uint32_t set) {
    int sum = 0;
    int count = 0;
    for ( int i = last_visited_sample[set]; i > 0; i-- ) {
        int current = frequency_table[ set ][ i ];
        sum += current;
        count++;
    }
    return (uint32_t)( sum / count );
}

//
// Used for easily testing that a fewquency is in range
// return true if frequency of set 'set' is with Hi and Low parameters
//
int timing_verify_frequency_in_range(uint32_t set, uint32_t low_mhz_inclusive,uint32_t high_mhz_inclusive){
    uint32_t freq = timing_freq_measurement_hz(set);
    // Condition if no frequencies are recorded
    if ( freq <= 0 ) {
        return 0;}
    else if ( freq <= low_mhz_inclusive ) {
        return 1;
    }
    // Light blue if high
    else if ( freq >= high_mhz_inclusive ) {
        return 2;
    }
    return 0;
}
