#include <cstdio>
#include <cassert>
#include "unix_p2p.h"


static int get_baud(int baud)
{
    switch (baud) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        case 1152000:
            return B1152000;
        case 1500000:
            return B1500000;
        case 2000000:
            return B2000000;
        case 2500000:
            return B2500000;
        case 3000000:
            return B3000000;
        case 3500000:
            return B3500000;
        case 4000000:
            return B4000000;
        default:
            assert(0);
            return -1;
    }
}

UnixSerial::UnixSerial(const char* name, uint baud) {
    fd = open(name, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd < 0) {
        printf("Opening port (%s) failed: %d\n", name, fd);
        return;
    }

    struct termios serial;

    if (tcgetattr(fd, &serial) < 0) {
        printf("Error: getting configuration\n");
        return;
    }

    serial_old = serial;

    // SERIAL CONFIGURATION
    /* Set Baud Rate */
    cfsetospeed(&serial, get_baud(baud));
    cfsetispeed(&serial, get_baud(baud));

    // Todo: make read from uart more purify (exclude from current reader trash from uart)

    // Setting other Port Stuff
    //serial.c_cflag = 0;
    //serial.c_iflag = 0;
    serial.c_cflag = (serial.c_cflag & ~CSIZE) | CS8;
    serial.c_iflag &= ~IGNBRK;
    serial.c_oflag = 0;
    serial.c_lflag = 0;
    serial.c_cc[VMIN] = 0;
    serial.c_cc[VTIME] = 1;

    serial.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    serial.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    serial.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    serial.c_cflag |= 0;
    serial.c_cflag &= ~CSTOPB;
    serial.c_cflag &= ~CRTSCTS;

    // Make raw
    //cfmakeraw(&serial);

    // Flush Port, then applies attributes
    tcflush(fd, TCIFLUSH);

    // Set attributes to port
    if (tcsetattr(fd, TCSANOW, &serial) < 0) {
        printf("Error: set attributes\n");
        return;
    }

    printf("Port opened\n");
}

void UnixSerial::read_block(void* dst, size_t size) {
    uint written = 0;
    while (written != size)
        written += read_nonblock((ubyte *) dst + written, size - written);
}

size_t UnixSerial::read_nonblock(void* dst, size_t size) {
    auto bytes = read(fd, dst, size);
    if(bytes == -1)
        return 0;

    return bytes;
}

void UnixSerial::write(const void* data, size_t size) {
    //attempt to send
    if (::write(fd, data, size) < 0) {
        perror("serial write");
    }
}

UnixSerial::~UnixSerial() {
    tcsetattr(fd, TCSANOW, &serial_old); // Set old settings
    close(fd);
}
