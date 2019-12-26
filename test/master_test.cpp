#include <HENRY/serial.hpp>
#include <debug/debug.hpp>

int main() {
    constexpr const char *const filepath = "/dev/ttyACM0";
    auto dev = serial::open(filepath, 9600);
    if (!dev.is_open) {
        debug::log::error("Failed to open device: %s", filepath);
        return -1;
    }
    debug::log::success("Opened device");
    return 0;
}
