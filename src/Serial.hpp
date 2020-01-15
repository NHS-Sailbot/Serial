#pragma once

namespace Henry {
    class Serial {
      private:
        enum Flags { NONE = 0, OPEN_STATUS = 1, VALID_STATUS = 2 };
        unsigned int mBaudrate;
        unsigned short mID, mFlags;
        const char *mFilepath;

      public:
        Serial();
        Serial(const char *const, const unsigned int);
        ~Serial();

        bool Open(const char *const, const unsigned int);
        void Close();
        void Flush();
        void Read(void *const, const unsigned int);
        void Write(void *const, const unsigned int);
        bool IsOpen() const { return mFlags & OPEN_STATUS; }
        bool IsValid() const { return mFlags & VALID_STATUS; }
    };
} // namespace Henry
