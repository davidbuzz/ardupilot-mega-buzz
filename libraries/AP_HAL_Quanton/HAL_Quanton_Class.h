
#ifndef __AP_HAL_Quanton_CLASS_H__
#define __AP_HAL_Quanton_CLASS_H__

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_Quanton

#include <AP_HAL_Quanton.h>
#include "AP_HAL_Quanton_Namespace.h"
#include <systemlib/visibility.h>
#include <systemlib/perf_counter.h>

class HAL_Quanton : public AP_HAL::HAL {
public:
    HAL_Quanton();    
    void init(int argc, char * const argv[]) const;
};

extern const HAL_Quanton AP_HAL_Quanton;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_Quanton
#endif // __AP_HAL_Quanton_CLASS_H__
