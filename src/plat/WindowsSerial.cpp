#include <Henry/Serial.hpp>

namespace Henry {
    SerialDevice::SerialDevice() : mBaudrate(0), mId(0), mFlags(NONE), mFilepath(nullptr) {}
    SerialDevice::SerialDevice(const char *const deviceFilepath, const unsigned int baudrate)
        : mBaudrate(baudrate), mId(0), mFlags(NONE), mFilepath(deviceFilepath) {
        open(deviceFilepath, baudrate);
    }
    SerialDevice::~SerialDevice() { close(); }

    bool SerialDevice::open(const char *const deviceFilepath, const unsigned int baudrate) { 
        return false;
    }

    void SerialDevice::close() {
    }

    void SerialDevice::flushBuffer() const {
    }

    void SerialDevice::readBuffer(void *const buffer, const unsigned int size) {
    }

    void SerialDevice::writeBuffer(void *const buffer, const unsigned int size) {
    }
} // namespace Henry
