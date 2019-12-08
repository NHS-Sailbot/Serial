#include <serial/serial.hpp>

int main() {
    auto dev = serial::open("/dev/ttyUSB0", 9600);
    if (!dev.is_open) return -1;
    return 0;
}
