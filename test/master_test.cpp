#include <HENRY/serial.hpp>
#include <debug/debug.hpp>

int main() {
    constexpr const char *const filepath = "/dev/ttyACM0";
    HENRY::Serial device(filepath, 9600);
    if (!device.is_open) {
        debug::log::error("Failed to open device: %s", filepath);
        return 0;
    }
    debug::log::success("Opened device");
    return 0;
}
