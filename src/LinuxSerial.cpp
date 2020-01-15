#include "Serial.hpp"

#include <errno.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#define DEBUG_ENABLE_TIMING
#define DEBUG_ENABLE_LOGGING
#include <Debug/Debug.hpp>

static inline constexpr unsigned int convertBaudrate(const unsigned int aBaudrate) {
    switch (aBaudrate) {
    case 50: return B50;
    case 75: return B75;
    case 110: return B110;
    case 134: return B134;
    case 150: return B150;
    case 200: return B200;
    case 300: return B300;
    case 600: return B600;
    case 1200: return B1200;
    case 1800: return B1800;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 3500000: return B3500000;
    case 4000000: return B4000000;
    default: return -1;
    }
}

namespace Henry {
    struct LinuxSerialDevice {
        int mFileDescriptor = -1;
    };
    static LinuxSerialDevice sLinuxDevices[32];

    Serial::Serial() : mBaudrate(0), mID(0), mFlags(NONE), mFilepath(nullptr) {}
    Serial::Serial(const char *const aFilepath, const unsigned int aBaudrate)
        : mBaudrate(aBaudrate), mID(0), mFlags(NONE), mFilepath(aFilepath) {
        Open(aFilepath, aBaudrate);
    }
    Serial::~Serial() { Close(); }

    bool Serial::Open(const char *const aFilepath, const unsigned int aBaudrate) {
        DEBUG_BEGIN_FUNC_PROFILE;

        mFilepath = aFilepath;
        mBaudrate = convertBaudrate(aBaudrate);

        DEBUG_BEGIN_PROFILE(dSelectIndex);
        unsigned int tIndex = 0;
        for (; tIndex < 32; ++tIndex)
            if (sLinuxDevices[tIndex].mFileDescriptor < 0) break;
        if (tIndex > 31) {
            Debug::Log::error("Too many devices requested!");
            mFlags &= !OPEN_STATUS;
            return false;
        }
        DEBUG_END_PROFILE(dSelectIndex);

        auto &tLinuxDevice = sLinuxDevices[tIndex];

        // More information about serial communication can be found at
        // https://www.cmrr.umn.edu/~strupp/serial.html#2_1. This is where
        // I based this code off of.

        // Attempt to open the file.
        tLinuxDevice.mFileDescriptor = open(mFilepath, O_RDWR | O_NOCTTY | O_NDELAY | O_SYNC);
        if (tLinuxDevice.mFileDescriptor == -1) {
            Debug::Log::message("Serial::Open(): Unable to open %s: %s", mFilepath, strerror(errno));
            return false;
        } else {
            fcntl(tLinuxDevice.mFileDescriptor, F_SETFL, 0);
            mFlags |= OPEN_STATUS;
        }

        // Lock access so that another process can't also use the port.
        if (flock(tLinuxDevice.mFileDescriptor, LOCK_EX | LOCK_NB) != 0) {
            Close();
            return false;
        }

        // If one requires no blocking, uncomment
        // fcntl(file_descriptor, F_SETFL, FNDELAY);

        // Setting the port options
        struct termios tPortSettings, tPrevPortSettings;

        // First get the current options
        tcgetattr(tLinuxDevice.mFileDescriptor, &tPortSettings);
        tPrevPortSettings = tPortSettings;

        // Set the baudrate into the options (in and out)
        cfsetispeed(&tPortSettings, mBaudrate);
        cfsetospeed(&tPortSettings, mBaudrate);

        // CLOCAL restricts the operating system from changing the owner of the
        // port to be this program, and CREAD enables the reciever.
        // Both of these should always be set.
        tPortSettings.c_cflag |= (CLOCAL | CREAD);

        // Disable parity bits
        // options.c_cflag &= ~PARENB; // disable parity
        // options.c_cflag &= ~CSTOPB; // disable 2 stop bits

        // Enable hardware flow control. This is not supported on all platforms.
        // Some systems may define it as CNEW_RTSCTS, where others define it as CRTSCTS.
        // options.c_cflag |= CRTSCTS;

        tPortSettings.c_cflag &= ~CSIZE; // Mask the character size bits
        tPortSettings.c_cflag |= CS8;    // Select 8 data bits

        // Enable parity checking on the incoming stream as well as strip the parity bits.
        tPortSettings.c_iflag |= (INPCK | ISTRIP);

        // TODO: Read more up on these port settings.
        tPortSettings.c_iflag = IGNPAR;
        tPortSettings.c_oflag = 0;
        tPortSettings.c_lflag = 0;
        tPortSettings.c_cc[VMIN] = 0;  // block untill n bytes are received
        tPortSettings.c_cc[VTIME] = 0; // block untill a timer expires (n * 100 mSec.)

        int result = tcsetattr(tLinuxDevice.mFileDescriptor, TCSANOW, &tPortSettings);

        if (result == -1) {
            Debug::Log::message("Serial::Open(): Unable to open %s: %s", mFilepath, strerror(errno));
            tcsetattr(tLinuxDevice.mFileDescriptor, TCSANOW, &tPrevPortSettings);
            Close();
            return false;
        }

        int status;
        if (ioctl(tLinuxDevice.mFileDescriptor, TIOCMGET, &status) == -1) {
            Debug::Log::message("Serial::Open(): Unable to open %s: %s", mFilepath, strerror(errno));
            tcsetattr(tLinuxDevice.mFileDescriptor, TCSANOW, &tPrevPortSettings);
            Close();
            return false;
        }

        status |= TIOCM_DTR; // turn on DTR
        status |= TIOCM_RTS; // turn on RTS

        if (ioctl(tLinuxDevice.mFileDescriptor, TIOCMSET, &status) == -1) {
            Debug::Log::message("Serial::Open(): Unable to open %s: %s", mFilepath, strerror(errno));
            tcsetattr(tLinuxDevice.mFileDescriptor, TCSANOW, &tPrevPortSettings);
            Close();
            return false;
        }

        Debug::Timer::sleep(2);

        tcflush(tLinuxDevice.mFileDescriptor, TCIOFLUSH);

        mID = tIndex;
        return true;
    }

    void Serial::Close() {
        DEBUG_BEGIN_FUNC_PROFILE;

        if (IsOpen()) {
            auto &tLinuxDevice = sLinuxDevices[mID];
            flock(tLinuxDevice.mFileDescriptor, LOCK_UN); // Free the port so that others can use it.
            close(tLinuxDevice.mFileDescriptor);          // Close the device file.
            mFlags &= ~OPEN_STATUS;
        }
    }

    void Serial::Flush() {
        DEBUG_BEGIN_FUNC_PROFILE;

        if (IsOpen()) {
            auto &tLinuxDevice = sLinuxDevices[mID];
            // Flush the io buffers
            tcflush(tLinuxDevice.mFileDescriptor, TCIOFLUSH);
        }
    }

    void Serial::Read(void *const buffer, const unsigned int size) {
        DEBUG_BEGIN_FUNC_PROFILE;

        // Before reading from the device, make sure the device has been opened.
        if (IsOpen()) {
            auto &tLinuxDevice = sLinuxDevices[mID];
            // read directly from the device file.
            int tBytesRead = read(tLinuxDevice.mFileDescriptor, buffer, size);
            if (tBytesRead < 0) {
                Debug::Log::error("Serial::Read(): read() of %d bytes failed!", size);
                mFlags &= ~VALID_STATUS;
            } else
                mFlags |= VALID_STATUS;
        }
    }

    void Serial::Write(void *const buffer, const unsigned int size) {
        DEBUG_BEGIN_FUNC_PROFILE;

        // Before writing to the device, make sure the device has been opened.
        if (IsOpen()) {
            auto &tLinuxDevice = sLinuxDevices[mID];
            // Write directly to the device file.
            int tBytesWritten = write(tLinuxDevice.mFileDescriptor, buffer, size);
            // Make sure that the number of bytes written isn't less than 0.
            // This is true when it fails to write to the file.
            if (tBytesWritten < 0) {
                Debug::Log::error("Serial::Write(): write() of %d bytes failed!", size);
                mFlags &= ~VALID_STATUS;
            } else
                mFlags |= VALID_STATUS;
        }
    }
} // namespace Henry
