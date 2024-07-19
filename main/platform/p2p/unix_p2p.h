#pragma once

#include "base.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

class UnixSerial : public BaseSerial
{
public:
    int fd;
    struct termios serial_old;

    UnixSerial(const char* name, uint baud);

    size_t read_block(void* dst, size_t size) override;

    void write(const void* data, size_t size) override;

    ~UnixSerial() override;
};
