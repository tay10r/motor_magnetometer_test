#pragma once

#include <motor_test/common.h>

#include <vector>

#include <cstdint>

namespace mt {

class serial_device
{
public:
  virtual ~serial_device() = default;

  virtual auto write(uint8_t c) -> bool = 0;

  virtual auto read(uint8_t* c) -> bool = 0;
};

enum class capture_state
{
  initial,
  wait_reset_ack,
  start,
  wait_start_ack,
  status,
  wait_status,
  read,
  read_response,
  done
};

constexpr uint16_t
max_read_size()
{
  return 1024 * sizeof(uint32_t) * 3;
}

class clock
{
public:
  virtual ~clock() = default;

  virtual auto read() -> uint32_t = 0;
};

inline auto
elapsed(const uint32_t t0, clock& clk) -> uint32_t
{
  const auto t1 = clk.read();
  if (t1 < t0) {
    // integer overflow occurred
    return (0xffff'ffffUL - t0) + t1;
  } else {
    return t1 - t0;
  }
}

constexpr uint32_t
timeout()
{
  return 100;
}

class capture_program final
{
public:
  void run(serial_device& dev, clock& clk);

  [[nodiscard]] auto done() const -> bool { return state_ == capture_state::done; }

protected:
  void next_state() { state_ = static_cast<capture_state>(static_cast<int>(state_) + 1); }

  void read_status(serial_device& dev, clock& clk);

  void request_next_byte(serial_device& dev, clock& clk);

  void read_next_byte(serial_device& dev, clock& clk);

  [[nodiscard]] static auto send(serial_device& dev, message_id id, uint8_t op1 = 0, uint8_t op2 = 0) -> bool;

  [[nodiscard]] static auto recv(serial_device& dev, message_packet* p) -> bool;

  [[nodiscard]] static auto wait_ack(serial_device& dev) -> bool;

private:
  capture_state state_{ capture_state::initial };

  std::vector<uint8_t> read_buffer_;

  /**
   * @brief Any time we're waiting on a response, we use this to check if we've been waiting a long time.
   *        If so, we issue a new request.
   */
  uint32_t timeout_start_{};
};

} // namespace mt
