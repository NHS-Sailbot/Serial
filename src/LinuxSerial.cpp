#include "Serial.hpp"

#include <errno.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

static inline constexpr unsigned int convertBaudrate(
const unsigned int baudrate) {
	switch (baudrate) {
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
	default: return 0;
	}
}

namespace Henry {
	struct LinuxSerialDevice {
		int mFileDescriptor;
		speed_t mBaudrate;
	};
	static LinuxSerialDevice sLinuxDevices[32];

	SerialDevice::SerialDevice() :
	mBaudrate(0), mId(0), mFlags(NONE), mFilepath(nullptr) {}
	SerialDevice::SerialDevice(
	const char *const deviceFilepath, const unsigned int baudrate) :
	mBaudrate(baudrate),
	mId(0), mFlags(NONE), mFilepath(deviceFilepath) {
		open(deviceFilepath, baudrate);
	}
	SerialDevice::~SerialDevice() { close(); }

	bool SerialDevice::open(const char *const deviceFilepath, const unsigned int baudrate) {
		mFilepath = deviceFilepath;
		mBaudrate = baudrate;
		unsigned short tIndex = 0;
		for (; tIndex < 32; ++tIndex)
			if (sLinuxDevices[tIndex].mFileDescriptor < 0)
				break;
		if (tIndex > 31) {
			mFlags &= ~OPEN_STATUS;
			return false;
		}
		auto &tLinuxDevice = sLinuxDevices[tIndex];
		tLinuxDevice.mBaudrate = convertBaudrate(mBaudrate);
		tLinuxDevice.mFileDescriptor = ::open(mFilepath, O_RDWR | O_NOCTTY | O_NDELAY | O_SYNC);
		if (tLinuxDevice.mFileDescriptor == -1) {
			return false;
		} else {
			fcntl(tLinuxDevice.mFileDescriptor, F_SETFL, 0);
			mFlags |= OPEN_STATUS;
		}
		if (flock(tLinuxDevice.mFileDescriptor, LOCK_EX | LOCK_NB) != 0) {
			close();
			return false;
		}
		struct termios tPortSettings, tPrevPortSettings;
		tcgetattr(tLinuxDevice.mFileDescriptor, &tPortSettings);
		tPrevPortSettings = tPortSettings;
		cfsetispeed(&tPortSettings, tLinuxDevice.mBaudrate);
		cfsetospeed(&tPortSettings, tLinuxDevice.mBaudrate);
		tPortSettings.c_cflag |= (CLOCAL | CREAD);
		tPortSettings.c_cflag |= CS8;
		tPortSettings.c_iflag |= (INPCK | ISTRIP);
		tPortSettings.c_oflag = 0;
		tPortSettings.c_lflag = 0;
		tPortSettings.c_cc[VMIN] = 0;
		tPortSettings.c_cc[VTIME] = 0;
		int tResult = tcsetattr(tLinuxDevice.mFileDescriptor, TCSANOW, &tPortSettings);
		if (tResult == -1) {
			tcsetattr(tLinuxDevice.mFileDescriptor, TCSANOW, &tPrevPortSettings);
			close();
			return false;
		}
		if (ioctl(tLinuxDevice.mFileDescriptor, TIOCMGET, &tResult) == -1) {
			tcsetattr(tLinuxDevice.mFileDescriptor, TCSANOW, &tPrevPortSettings);
			close();
			return false;
		}
		tResult |= TIOCM_DTR;
		tResult |= TIOCM_RTS;
		if (ioctl(tLinuxDevice.mFileDescriptor, TIOCMSET, &tResult) == -1) {
			tcsetattr(tLinuxDevice.mFileDescriptor, TCSANOW, &tPrevPortSettings);
			close();
			return false;
		}
		tcflush(tLinuxDevice.mFileDescriptor, TCIOFLUSH);
		mId = tIndex;
		return true;
	}

	void SerialDevice::close() {
		if (isOpen()) {
			auto &tLinuxDevice = sLinuxDevices[mId];
			flock(tLinuxDevice.mFileDescriptor, LOCK_UN);
			::close(tLinuxDevice.mFileDescriptor);
			mFlags &= ~OPEN_STATUS;
		}
	}

	void SerialDevice::flushBuffer() const {
		if (isOpen()) {
			auto &tLinuxDevice = sLinuxDevices[mId];
			tcflush(tLinuxDevice.mFileDescriptor, TCIOFLUSH);
		}
	}

	void SerialDevice::readBuffer(void *const buffer, const unsigned int size) {
		if (isOpen()) {
			auto &tLinuxDevice = sLinuxDevices[mId];
			auto tBytesRead = read(tLinuxDevice.mFileDescriptor, buffer, size);
			if (tBytesRead < 0)
				mFlags &= ~VALID_STATUS;
			else
				mFlags |= VALID_STATUS;
		}
	}

	void SerialDevice::writeBuffer(void *const buffer, const unsigned int size) {
		if (isOpen()) {
			auto &tLinuxDevice = sLinuxDevices[mId];
			auto tBytesWritten = write(tLinuxDevice.mFileDescriptor, buffer, size);
			if (tBytesWritten < 0)
				mFlags &= ~VALID_STATUS;
			else
				mFlags |= VALID_STATUS;
		}
	}
} // namespace Henry
