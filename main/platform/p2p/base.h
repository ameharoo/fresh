#pragma once

#include "types.h"

#include <cstddef>


class BaseSerial
{
public:
    virtual size_t read_block(void* dst, size_t size) = 0;

    virtual void write(const void* data, size_t size) = 0;

    virtual void write_byte(ubyte data) {
        write(&data, 1);
    }

    virtual void start_writing() {}

    virtual void end_writing() {}

    virtual ~BaseSerial() = default;
};
