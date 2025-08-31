#include <stdio.h>
#include <xdc/runtime/System.h>
#include "util/inv_mpu.h" // Needed for the int_param_s struct definition

// Provide an implementation for the logging function
void _MLPrintLog(int priority, const char *tag, const char *fmt, ...) {
    // For this project, we can just route it to the TI-RTOS System_printf
    // You could add more complex logic here if you needed different log levels.
    va_list args;
    va_start(args, fmt);
    System_vprintf(fmt, args);
    va_end(args);
}

// Provide an implementation for the min function
// (This may not be strictly necessary if a standard library has it,
// but defining it ensures the linker finds it).
int min(int a, int b) {
    return (a < b) ? a : b;
}

// Provide a stub for the interrupt callback registration
// Since you are using polling, this function can be empty.
int reg_int_cb(struct int_param_s *int_param) {
    // Polling is used, so no interrupt callback is needed.
    return 0;
}
