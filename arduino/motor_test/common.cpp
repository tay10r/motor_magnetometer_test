#include "common.h"

namespace mt {

timer::timer(const uint32_t interval)
  : interval_(interval)
{
}

auto
timer::step(uint32_t t) -> bool
{
  const auto dt = t - last_timestamp_;

  remainder_ += dt;

  const auto steps = remainder_ / interval_;

  remainder_ -= steps * interval_;

  return steps > 0;
}

void
timer::reset()
{
  last_timestamp_ = 0;
  remainder_ = 0;
}

auto
message_packet::compute_checksum() const -> uint8_t
{
  static_assert(sizeof(message_packet) == 4, "Message packet size is not correct.");
  uint8_t buf[3]{ static_cast<uint8_t>(id), operands[0], operands[1] };
  uint8_t result{ 0 };
  for (auto i = 0; i < 3; i++) {
    result ^= buf[i];
  }
  return result;
}

void
program::setup(platform& plt)
{
}

void
program::loop(platform& plt)
{
  switch (state_) {
    case program_state::wait:
      loop_wait(plt);
      break;
    case program_state::acq:
      loop_acq(plt);
      break;
    case program_state::failed:
      loop_failed(plt);
      break;
    case program_state::done:
      loop_done(plt);
      break;
  }
}

void
program::loop_wait(platform& plt)
{
  message_packet pkt;

  if (!read_message(plt, &pkt)) {
    return;
  }

  switch (pkt.id) {
    case message_id::get_status:
      send_message(plt, message_id::status, static_cast<uint8_t>(status::wait));
      break;
    case message_id::reset_state:
      state_ = program_state::wait;
      send_message(plt, message_id::ack);
      break;
    case message_id::start_acq:
      state_ = program_state::acq;
      sample_offset_ = 0;
      send_message(plt, message_id::ack);
      break;
    default:
      send_message(plt, message_id::invalid_msg);
      break;
  }
}

void
program::loop_acq(platform& plt)
{
  message_packet pkt;

  if (read_message(plt, &pkt)) {
    switch (pkt.id) {
      case message_id::get_status:
        send_message(plt, message_id::status, static_cast<uint8_t>(status::running_ack));
        break;
      default:
        send_message(plt, message_id::invalid_msg);
        break;
    }
  }

  const auto motor_state = (sample_offset_ / (num_samples() / 4)) % 2;

  if (!plt.set_motor_state(!!motor_state)) {
    state_ = program_state::failed;
    return;
  }

  uint32_t t = 0;

  if (!plt.get_time(&t)) {
    state_ = program_state::failed;
    return;
  }

  if (!sample_timer_.step(t)) {
    // nothing left to do
    return;
  }

  if (!plt.read_magnetometer(&sample_buffer_[sample_offset_ * 3])) {
    state_ = program_state::failed;
    return;
  }

  sample_offset_++;

  if (sample_offset_ == num_samples()) {
    state_ = program_state::done;
  }
}

void
program::loop_failed(platform& plt)
{
  message_packet pkt;

  if (!read_message(plt, &pkt)) {
    return;
  }

  switch (pkt.id) {
    case message_id::reset_state:
      state_ = program_state::wait;
      break;
    case message_id::get_status:
      send_message(plt, message_id::status, static_cast<uint8_t>(status::acq_failed));
      break;
    default:
      send_message(plt, message_id::invalid_msg);
      break;
  }
}

void
program::loop_done(platform& plt)
{
  message_packet pkt;

  if (!read_message(plt, &pkt)) {
    return;
  }

  switch (pkt.id) {
    case message_id::reset_state:
      state_ = program_state::wait;
      break;
    case message_id::get_status:
      send_message(plt, message_id::status, static_cast<uint8_t>(status::acq_complete));
      break;
    case message_id::read_request:
      handle_read_request(plt, pkt);
      break;
    default:
      send_message(plt, message_id::invalid_msg);
      break;
  }
}

void
program::handle_read_request(platform& plt, const message_packet& pkt)
{
  const uint16_t offset_lo = static_cast<uint16_t>(pkt.operands[0]);
  const uint16_t offset_hi = static_cast<uint16_t>(pkt.operands[1]);
  const uint16_t offset = (offset_hi << 8) | offset_lo;
  if (offset >= sizeof(sample_buffer_)) {
    return;
  }
  const auto* ptr = reinterpret_cast<const uint8_t*>(&sample_buffer_[0]);
  const auto data = ptr[offset];
  send_message(plt, message_id::read_response, data);
}

auto
program::read_message(platform& plt, message_packet* p) -> bool
{
  uint8_t* buffer = reinterpret_cast<uint8_t*>(p);
  for (auto i = 0; i < sizeof(message_packet); i++) {
    if (!plt.serial_read(&buffer[i])) {
      return false;
    }
  }
  if (p->compute_checksum() != p->checksum) {
    return false;
  }
  return true;
}

void
program::send_message(platform& plt, message_id id, uint8_t op1, uint8_t op2)
{
  message_packet pkt;
  pkt.id = id;
  pkt.operands[0] = op1;
  pkt.operands[1] = op2;
  pkt.checksum = pkt.compute_checksum();

  uint8_t* buffer = reinterpret_cast<uint8_t*>(&pkt);
  for (auto i = 0; i < sizeof(message_packet); i++) {
    if (!plt.serial_write(buffer[i])) {
      return;
    }
  }
}

} // namespace mt
