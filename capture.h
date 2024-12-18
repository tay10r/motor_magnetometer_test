#pragma once

#include <motor_test/common.h>

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
  done
};

class capture_program final
{
public:
  void run(serial_device& dev);

  [[nodiscard]] auto done() const -> bool { return state_ == capture_state::done; }

protected:
  void next_state() { state_ = static_cast<capture_state>(static_cast<int>(state_) + 1); }

  void read_status(serial_device& dev);

  [[nodiscard]] static auto send(serial_device& dev, message_id id, uint8_t op1 = 0, uint8_t op2 = 0) -> bool;

  [[nodiscard]] static auto recv(serial_device& dev, message_packet* p) -> bool;

  [[nodiscard]] static auto wait_ack(serial_device& dev) -> bool;

private:
  capture_state state_{ capture_state::initial };
};

} // namespace mt
