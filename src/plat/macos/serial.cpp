#include <HENRY/serial.hpp>

namespace serial {
    Device open(const char *const filepath, const unsigned int baud) { return {}; }
    void close(Device *const device) {}

    void flush(const Device *const device) {}
    void read(const Device *const device, void *const buffer, const unsigned int size) {}
    void write(const Device *const device, void *const buffer, const unsigned int size) {}
} // namespace serial