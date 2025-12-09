#include "bsp/serial.hpp"
#include "mbed.h"

BufferedSerial *serial_port = nullptr;

FileHandle *mbed::mbed_override_console(int) {
    return serial_port;
}

bool serial_init() {
    serial_port = new BufferedSerial(USBTX, USBRX, 115200);
    return true;
}

void serial_send(const char *data) {
    if (serial_port != nullptr) {
        serial_port->write(data, strlen(data));
    }
}