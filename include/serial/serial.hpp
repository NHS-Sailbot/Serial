#pragma once

namespace serial {
    struct Device {
        int file_descriptor;
        bool is_open;
        const char *filepath;
        unsigned int baudrate;
    };

    bool open(Device &device, const char *const filepath, const unsigned int baud);
    static inline Device open(const char *const filepath, const unsigned int baud) {
        Device result;
        open(result, filepath, baud);
        return result;
    }
    void close(Device &device);

    void flush(const Device &device);
    void read(const Device &device, void *const buffer, const unsigned int size);
    void write(const Device &device, void *const buffer, const unsigned int size);
} // namespace serial
