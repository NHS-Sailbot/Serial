#include <Henry/Serial.hpp>

#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <sys/file.h>
#include <errno.h>

// clang-format off
#define BDC(x) case x: return B##x;
static inline constexpr unsigned int convertBaudrate(const unsigned int baudrate) {
    switch (baudrate) { 
        BDC(50); BDC(75); BDC(110); BDC(134); BDC(150);
		BDC(200); BDC(300); BDC(600); BDC(1200); BDC(1800); 
		BDC(2400); BDC(4800); BDC(9600); BDC(19200); BDC(38400); 
		BDC(57600); BDC(115200); BDC(230400); BDC(460800); 
		BDC(500000); BDC(576000); BDC(921600); BDC(1000000);
		BDC(1152000); BDC(1500000); BDC(2000000); BDC(2500000); 
		BDC(3000000); BDC(3500000); BDC(4000000);
    default: return 0;
    } // clang-format on
}
#undef BDCASE

namespace Henry {
    namespace Linux {
        class SerialDevice : public Henry::SerialDevice {
            enum Flags { NONE = 0, OPEN_STATUS = 1, VALID_STATUS = 2 };
            speed_t mBaudrate;
            int mFileDescriptor;
            unsigned int mFlags;

          public:
            SerialDevice(const char *const deviceFilepath, const unsigned int baudrate) { open(deviceFilepath, baudrate); }
            ~SerialDevice() { close(); }

            virtual bool open(const char *const deviceFilepath, const unsigned int baudrate) override {
                mFlags = NONE;
                mBaudrate = convertBaudrate(mBaudrate);
                mFileDescriptor = ::open(deviceFilepath, O_RDWR | O_NOCTTY | O_NDELAY | O_SYNC);
                if (mFileDescriptor == -1) {
                    return false;
                } else {
                    fcntl(mFileDescriptor, F_SETFL, 0);
                    mFlags |= OPEN_STATUS;
                }
                if (flock(mFileDescriptor, LOCK_EX | LOCK_NB) != 0) {
                    close();
                    return false;
                }
                struct termios tPortSettings, tPrevPortSettings;
                tcgetattr(mFileDescriptor, &tPortSettings);
                tPrevPortSettings = tPortSettings;
                cfsetispeed(&tPortSettings, mBaudrate);
                cfsetospeed(&tPortSettings, mBaudrate);
                tPortSettings.c_cflag |= (CLOCAL | CREAD);
                tPortSettings.c_cflag |= CS8;
                tPortSettings.c_iflag |= (INPCK | ISTRIP);
                tPortSettings.c_oflag = 0;
                tPortSettings.c_lflag = 0;
                tPortSettings.c_cc[VMIN] = 0;
                tPortSettings.c_cc[VTIME] = 0;
                int tResult = tcsetattr(mFileDescriptor, TCSANOW, &tPortSettings);
                if (tResult == -1) {
                    tcsetattr(mFileDescriptor, TCSANOW, &tPrevPortSettings);
                    close();
                    return false;
                }
                if (ioctl(mFileDescriptor, TIOCMGET, &tResult) == -1) {
                    tcsetattr(mFileDescriptor, TCSANOW, &tPrevPortSettings);
                    close();
                    return false;
                }
                tResult |= TIOCM_DTR;
                tResult |= TIOCM_RTS;
                if (ioctl(mFileDescriptor, TIOCMSET, &tResult) == -1) {
                    tcsetattr(mFileDescriptor, TCSANOW, &tPrevPortSettings);
                    close();
                    return false;
                }
                tcflush(mFileDescriptor, TCIOFLUSH);
                return true;
            }
            virtual bool isOpen() const override;
            virtual bool isValid() const override;
            virtual void close() override {
                if (isOpen()) {
                    flock(mFileDescriptor, LOCK_UN);
                    ::close(mFileDescriptor);
                    mFlags &= ~OPEN_STATUS;
                }
            }

            virtual void flushBuffer() const override {
                if (isOpen()) tcflush(mFileDescriptor, TCIOFLUSH);
            }
            virtual void readBuffer(void *const buffer, const unsigned int size) override {
                if (isOpen()) {
                    auto tBytesRead = read(mFileDescriptor, buffer, size);
                    if (tBytesRead < 0)
                        mFlags &= ~VALID_STATUS;
                    else
                        mFlags |= VALID_STATUS;
                }
            }
            virtual void writeBuffer(void *const buffer, const unsigned int size) override {
                if (isOpen()) {
                    auto tBytesWritten = write(mFileDescriptor, buffer, size);
                    if (tBytesWritten < 0)
                        mFlags &= ~VALID_STATUS;
                    else
                        mFlags |= VALID_STATUS;
                }
            }
        };

    } // namespace Linux

    std::shared_ptr<SerialDevice> createSerialDevice(const char *const deviceFilepath, const unsigned int baudrate) {
        return std::make_shared<Linux::SerialDevice>(deviceFilepath, baudrate);
    }
} // namespace Henry
