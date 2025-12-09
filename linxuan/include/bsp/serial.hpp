#pragma once

bool serial_init();
void serial_send(const char *data);
void serial_lock();
void serial_unlock();