#include "capture.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <cstdint>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

class linux_serial_device final : public mt::serial_device
{
public:
  linux_serial_device(const char* port_name, int baud_rate)
  {
    fd_ = open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      std::cerr << "Error: Unable to open port " << port_name << ", error: " << strerror(errno) << std::endl;
      throw std::runtime_error("Failed to open serial port");
    }

    configure_port(baud_rate);
  }

  ~linux_serial_device() override
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  auto read(uint8_t* c) -> bool override
  {
    if (!c)
      return false;
    ssize_t result = ::read(fd_, c, 1);
    return result == 1;
  }

  auto write(uint8_t c) -> bool override
  {
    ssize_t result = ::write(fd_, &c, 1);
    return result == 1;
  }

private:
  int fd_;

  void configure_port(int baud_rate)
  {
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
      std::cerr << "Error: tcgetattr failed, error: " << strerror(errno) << std::endl;
      throw std::runtime_error("Failed to get port attributes");
    }

    cfsetospeed(&tty, baud_rate_to_constant(baud_rate));
    cfsetispeed(&tty, baud_rate_to_constant(baud_rate));

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN] = 1;                         // read one byte at a time
    tty.c_cc[VTIME] = 1;                        // timeout in tenths of a second

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // disable parity
    tty.c_cflag &= ~CSTOPB;                 // one stop bit
    tty.c_cflag &= ~CRTSCTS;                // disable RTS/CTS hardware flow control

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      std::cerr << "Error: tcsetattr failed, error: " << strerror(errno) << std::endl;
      throw std::runtime_error("Failed to set port attributes");
    }

    tcflush(fd_, TCIOFLUSH);
  }

  speed_t baud_rate_to_constant(int baud_rate)
  {
    switch (baud_rate) {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      default:
        std::cerr << "Error: Unsupported baud rate " << baud_rate << std::endl;
        throw std::invalid_argument("Unsupported baud rate");
    }
  }
};

class clock_impl final : public mt::clock
{
public:
  auto read() -> uint32_t override
  {
    const auto t1 = std::chrono::high_resolution_clock::now();

    const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0_).count();

    return static_cast<uint32_t>(dt);
  }

private:
  std::chrono::high_resolution_clock::time_point t0_{ std::chrono::high_resolution_clock::now() };
};

} // namespace

auto
main() -> int
{
  linux_serial_device dev("/dev/ttyACM1", 115200);

  const auto num_datasets = 128;

  for (auto i = 0; i < num_datasets; i++) {

    dev.write('r');

    std::this_thread::sleep_for(std::chrono::milliseconds(256 * 10));

    dev.write('w');

    std::vector<std::string> lines;

    std::string line;

    while (true) {
      uint8_t c = 0;
      if (!dev.read(&c)) {
        break;
      }

      if (c == '\r') {
        continue;
      }

      if (c == '\n') {
        lines.emplace_back(line);
        line.clear();
        if (lines.size() == 256) {
          break;
        }
      } else {
        line.push_back(static_cast<char>(c));
      }
    }

    if (!line.empty()) {
      lines.emplace_back(std::move(line));
    }

    std::ostringstream path_stream;
    path_stream << "samples/" << std::setw(2) << std::setfill('0') << i << ".csv";
    const auto path = path_stream.str();

    std::ofstream outfile(path);

    outfile << "motor_state,magnetometer\n";

    for (size_t j = 0; j < lines.size(); j++) {
      const auto motor_state = (j / (256 / 4)) % 2;
      outfile << (motor_state ? '1' : '0') << "," << lines[j] << '\n';
    }
  }

  return 0;
}
