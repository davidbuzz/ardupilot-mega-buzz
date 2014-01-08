

#ifndef __AP_HAL_Quanton_STORAGE_H__
#define __AP_HAL_Quanton_STORAGE_H__

#include <AP_HAL.h>
#include "AP_HAL_Quanton_Namespace.h"
#include <systemlib/perf_counter.h>

#define Quanton_STORAGE_SIZE 4096
#define Quanton_STORAGE_MAX_WRITE 512
#define Quanton_STORAGE_LINE_SHIFT 9
#define Quanton_STORAGE_LINE_SIZE (1<<Quanton_STORAGE_LINE_SHIFT)
#define Quanton_STORAGE_NUM_LINES (Quanton_STORAGE_SIZE/Quanton_STORAGE_LINE_SIZE)

class Quanton::QuantonStorage : public AP_HAL::Storage {
public:
    QuantonStorage() :
	_fd(-1),
	_dirty_mask(0),
	_perf_storage(perf_alloc(PC_ELAPSED, "APM_storage")),
	_perf_errors(perf_alloc(PC_COUNT, "APM_storage_errors"))
	{}
    void init(void* machtnichts) {}
    uint8_t  read_byte(uint16_t loc);
    uint16_t read_word(uint16_t loc);
    uint32_t read_dword(uint16_t loc);
    void     read_block(void *dst, uint16_t src, size_t n);

    void write_byte(uint16_t loc, uint8_t value);
    void write_word(uint16_t loc, uint16_t value);
    void write_dword(uint16_t loc, uint32_t value);
    void write_block(uint16_t dst, const void* src, size_t n);

    void _timer_tick(void);

private:
    int _fd;
    volatile bool _initialised;
    void _storage_create(void);
    void _storage_open(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    uint8_t _buffer[Quanton_STORAGE_SIZE] __attribute__((aligned(4)));
    volatile uint32_t _dirty_mask;
    perf_counter_t  _perf_storage;
    perf_counter_t  _perf_errors;
};

#endif // __AP_HAL_Quanton_STORAGE_H__
