
#include "AP_ADC_AnalogSource.h"

float AP_ADC_AnalogSource::read() {
    float fullscale = _adc->Ch(_ch);
    float scaled = _prescale * fullscale;
    return scaled;
}
