#pragma once

namespace serial {
    struct Device {
        int file_descriptor;
        bool is_open;
        const char *filepath;
        unsigned int baudrate;
    };

    Device open(const char *const filepath, const unsigned int baud);
    void close(Device *const device);

    void flush(const Device *const device);
    void read(const Device *const device, void *const buffer, const unsigned int size);
    void write(const Device *const device, void *const buffer, const unsigned int size);
} // namespace serial
