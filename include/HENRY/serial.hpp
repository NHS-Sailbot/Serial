#pragma once

namespace HENRY {
    class Serial {
        int file_descriptor;

      public:
        bool is_open;
        unsigned int baudrate;
        const char *m_filepath;

        Serial() = default;
        Serial(const char *const filepath, const unsigned int baud);
        ~Serial();

        bool open(const char *const filepath, const unsigned int baud);
        void close();
        void flush();
        void read(void *const buffer, const unsigned int size);
        void write(void *const buffer, const unsigned int size);
    };
} // namespace HENRY
