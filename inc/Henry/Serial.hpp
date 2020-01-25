#pragma once

#include <memory>

namespace Henry {
    class SerialDevice {
      public:
        virtual ~SerialDevice() = 0;

        virtual bool open(const char *const deviceFilepath, const unsigned int baudrate) = 0;
        virtual bool isOpen() const = 0;
        virtual bool isValid() const = 0;
        virtual void close() = 0;

        virtual void flushBuffer() const = 0;
        virtual void readBuffer(void *const, const unsigned int) = 0;
        virtual void writeBuffer(void *const, const unsigned int) = 0;
    };

    std::shared_ptr<SerialDevice> createSerialDevice(const char *const deviceFilepath, const unsigned int baudrate);
} // namespace Henry
