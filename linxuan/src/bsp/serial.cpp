#include "bsp/serial.hpp"
#include "mbed.h"

BufferedSerial *serial_port = nullptr;
Mutex *serial_mutex = nullptr;

FileHandle *mbed::mbed_override_console(int) {
    return serial_port;
}

bool serial_init() {
    serial_port = new BufferedSerial(USBTX, USBRX, 115200);
    serial_mutex = new Mutex();
    return true;
}

void serial_send(const char *data) {
    if (serial_port != nullptr) {
        serial_port->write(data, strlen(data));
    }
}

void serial_lock() {
    if (serial_mutex != nullptr) {
        serial_mutex->lock();
    }
}

void serial_unlock() {
    if (serial_mutex != nullptr) {
        serial_mutex->unlock();
    }
}
