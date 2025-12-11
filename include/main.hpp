#pragma once

#define BUILD_VERSION "1.0.1"

#include "mbed.h"

extern EventFlags *program_fatal_error_flag;

void trigger_fatal_error();