#include <motor_test/common.h>

#include "capture.h"

#include <iostream>
#include <random>
#include <vector>

#include <cstdlib>

namespace {

class simulated_platform final : public mt::platform
{
public:
  explicit simulated_platform(int seed)
    : rng_(seed)
  {
  }

  auto read_magnetometer(uint32_t* xyz) -> bool override
  {
    log() << "read magnetometer" << std::endl;
    xyz[0] = rand_int(32, 4);
    xyz[1] = rand_int(32, 4);
    // This isn't really accurate, since it depends on other factors, but that's
    // fine for now.
    const auto z_bias = motor_state_ ? 1024 : 64;
    xyz[2] = rand_int(z_bias, z_bias + 4);
    time_ += rand_int(1, 2);
    return true;
  }

  auto get_time(uint32_t* t) -> bool override
  {
    *t = time_;
    return true;
  }

  auto set_motor_state(bool state) -> bool override
  {
    log() << "set motor state to " << (state ? "on" : "off") << std::endl;
    time_ += rand_int(0, 1);
    motor_state_ = state;
    return true;
  }

  auto serial_read(uint8_t* c) -> bool override
  {
    if (host_buffer_.empty()) {
      return false;
    }
    *c = host_buffer_.front();
    host_buffer_.erase(host_buffer_.begin());
    return true;
  }

  auto serial_write(uint8_t c) -> bool override
  {
    if (device_buffer_.size() < 64) {
      device_buffer_.emplace_back(randomly_corrupt(c, 0.02F));
      return true;
    }
    return false;
  }

  auto host_read(uint8_t* c) -> bool
  {
    if (device_buffer_.empty()) {
      return false;
    }
    *c = device_buffer_.front();
    device_buffer_.erase(device_buffer_.begin());
    return true;
  }

  auto host_write(uint8_t c) -> bool
  {
    if (host_buffer_.size() < 64) {
      host_buffer_.emplace_back(randomly_corrupt(c, 0.02F));
      return true;
    }
    return false;
  }

protected:
  [[nodiscard]] auto log() -> std::ostream& { return std::cout << "[" << static_cast<int>(time_) << "]: "; }

  [[nodiscard]] auto randomly_corrupt(uint8_t data, float probability) -> uint8_t
  {
    std::uniform_real_distribution<float> dist(0, 1);
    if (dist(rng_) < probability) {
      return data + 1;
    }
    return data;
  }

  [[nodiscard]] auto rand_int(uint32_t min_v, uint32_t max_v) -> uint32_t
  {
    std::uniform_int_distribution<uint32_t> dist(min_v, max_v);
    return dist(rng_);
  }

private:
  std::mt19937 rng_;

  bool motor_state_{};

  uint32_t time_{};

  std::vector<std::uint8_t> device_buffer_;

  std::vector<std::uint8_t> host_buffer_;
};

class simulated_serial_device final : public mt::serial_device
{
public:
  simulated_serial_device(simulated_platform* platform)
    : platform_(platform)
  {
  }

  auto read(uint8_t* c) -> bool override { return platform_->host_read(c); }

  auto write(uint8_t c) -> bool override { return platform_->host_write(c); }

private:
  simulated_platform* platform_{};
};

} // namespace

auto
main() -> int
{
  simulated_platform plt(/*seed=*/0);

  mt::program program;

  program.setup(plt);

  simulated_serial_device host_dev(&plt);

  mt::capture_program capture;

  while (!capture.done()) {

    program.loop(plt);

    capture.run(host_dev);
  }

  return EXIT_SUCCESS;
}
