#include <Henry/Serial.hpp>

namespace Henry {
    namespace Windows {
        class SerialDevice : public Henry::SerialDevice {
          public:
            SerialDevice(const char *const deviceFilepath, const unsigned int baudrate) {}
            ~SerialDevice() {}

            bool open(const char *const deviceFilepath, const unsigned int baudrate) { return false; }
            bool isOpen() const override { return false; }
            bool isValid() const override { return false; }
            void close() override {}

            void flushBuffer() const override {}
            void readBuffer(void *const, const unsigned int) override {}
            void writeBuffer(void *const, const unsigned int) override {}
        };
    } // namespace Windows

    std::shared_ptr<SerialDevice> createSerialDevice(const char *const deviceFilepath, const unsigned int baudrate) {
        return std::make_shared<Windows::SerialDevice>(deviceFilepath, baudrate);
    }
} // namespace Henry
