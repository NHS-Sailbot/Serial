#pragma once

namespace Henry {
	class SerialDevice {
	  private:
		enum Flags { NONE = 0,
			OPEN_STATUS = 1,
			VALID_STATUS = 2 };
		unsigned int mBaudrate;
		unsigned short mId, mFlags;
		const char *mFilepath;

	  public:
		SerialDevice();
		SerialDevice(const char *const deviceFilepath, const unsigned int baudrate);
		~SerialDevice();

		bool open(const char *const deviceFilepath, const unsigned int baudrate);
		void close();
		void flushBuffer() const;
		void readBuffer(void *const, const unsigned int);
		void writeBuffer(void *const, const unsigned int);
		bool isOpen() const { return mFlags & OPEN_STATUS; }
		bool isValid() const { return mFlags & VALID_STATUS; }
	};
} // namespace Henry
