#pragma once

#include <stdint.h>

namespace mt {

class timer final
{
public:
  /**
   * @brief Constructs a new timer object.
   *
   * @param interval How many milliseconds apart should the timer emit a signal.
   * */
  explicit timer(const uint32_t interval);

  /**
   * @brief Moves the timer forward.
   *
   * @param t The new system time stamp, in terms of milliseconds.
   *
   * @return Whether or not a timer interval has elapsed.
   * */
  auto step(uint32_t t) -> bool;

  void reset();

private:
  uint32_t interval_{};

  uint32_t last_timestamp_{};

  uint32_t remainder_{};
};

class platform
{
public:
  virtual ~platform() = default;

  [[nodiscard]] virtual auto read_magnetometer(uint32_t* xyz) -> bool = 0;

  [[nodiscard]] virtual auto get_time(uint32_t* t) -> bool = 0;

  [[nodiscard]] virtual auto set_motor_state(bool state) -> bool = 0;

  [[nodiscard]] virtual auto serial_read(uint8_t* c) -> bool = 0;

  [[nodiscard]] virtual auto serial_write(uint8_t c) -> bool = 0;
};

enum class status : uint8_t
{
  wait,
  running_ack,
  acq_failed,
  acq_complete
};

enum class message_id : uint8_t
{
  // host messages
  reset_state = 0x00,
  start_acq = 0x01,
  get_status = 0x02,
  read_request = 0x03,
  // device messages
  ack = 0x80,
  status = 0x81,
  invalid_msg = 0x82,
  read_response = 0x83
};

struct message_packet final
{
  message_id id{ message_id::reset_state };
  uint8_t operands[2]{ 0, 0 };
  uint8_t checksum{};

  auto compute_checksum() const -> uint8_t;
};

enum class program_state : uint8_t
{
  wait = 0,
  acq = 1,
  failed = 2,
  done = 3
};

constexpr uint32_t
num_samples()
{
  return 1024;
}

class program final
{
public:
  void setup(platform& plt);

  void loop(platform& plt);

protected:
  [[nodiscard]] auto read_message(platform& plt, message_packet* p) -> bool;

  void send_message(platform& plt, message_id id, uint8_t op1 = 0, uint8_t op2 = 0);

  void loop_wait(platform& plt);

  void loop_acq(platform& plt);

  void loop_failed(platform& plt);

  void loop_done(platform& plt);

  void handle_read_request(platform& plt, const message_packet& pkt);

private:
  program_state state_{ program_state::wait };

  uint32_t last_sample_time_{};

  uint32_t sample_offset_{};

  timer sample_timer_{ 10 /* interval in milliseconds */ };

  uint32_t sample_buffer_[num_samples() * 3];
};

} // namespace mt
