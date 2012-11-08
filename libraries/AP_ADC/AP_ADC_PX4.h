/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_ADC_PX4_H__
#define __AP_ADC_PX4_H__


#include <inttypes.h>
#include "AP_ADC.h"

class AP_ADC_PX4 : public AP_ADC
{
public:
    AP_ADC_PX4();      // Constructor
    void                Init();

    // Read 1 sensor value
    float               Ch(unsigned char ch_num);

    // Read 6 sensors at once
    uint32_t            Ch6(const uint8_t *channel_numbers, float *result);

    // check if Ch6 would block
    bool                new_data_available(const uint8_t *channel_numbers);

    // Get minimum number of samples read from the sensors
    uint16_t            num_samples_available(const uint8_t *channel_numbers);

private:
    static void         read(uint32_t);

};

#endif
