#include "capture.h"

#include <iostream>

namespace mt {

void
log_error(const char* what)
{
  std::cerr << "[error]: " << what << std::endl;
}

void
log_info(const char* what)
{
  std::cerr << "[info]: " << what << std::endl;
}

void
capture_program::run(serial_device& dev)
{
  switch (state_) {
    case capture_state::initial:
      if (!send(dev, message_id::reset_state)) {
        return;
      }
      next_state();
      break;
    case capture_state::wait_reset_ack:
      if (wait_ack(dev)) {
        log_info("received reset ack");
        next_state();
      }
      break;
    case capture_state::start:
      if (!send(dev, message_id::start_acq)) {
        return;
      }
      next_state();
      break;
    case capture_state::wait_start_ack:
      if (wait_ack(dev)) {
        log_info("received start ack");
        next_state();
      }
      break;
    case capture_state::status:
      if (!send(dev, message_id::get_status)) {
        return;
      }
      next_state();
      break;
    case capture_state::wait_status:
      read_status(dev);
      break;
    case capture_state::done:
      break;
  }
}

void
capture_program::read_status(serial_device& dev)
{
  message_packet status_pkt;
  if (!recv(dev, &status_pkt)) {
    return;
  }

  if (status_pkt.id != message_id::status) {
    return;
  }

  log_info("received status");

  const auto s = static_cast<status>(status_pkt.operands[0]);

  switch (s) {
    case status::wait:
    case status::acq_failed:
      log_error("data acquisition failed");
      state_ = capture_state::initial;
      break;
    case status::running_ack:
      break;
    case status::acq_complete:
      log_info("acquisition done");
      next_state();
      break;
  }
}

auto
capture_program::send(serial_device& dev, message_id id, uint8_t op1, uint8_t op2) -> bool
{
  uint8_t buffer[3]{ static_cast<uint8_t>(id), op1, op2 };

  for (auto i = 0; i < 3; i++) {
    if (!dev.write(buffer[i])) {
      return false;
    }
  }

  return true;
}

auto
capture_program::recv(serial_device& dev, message_packet* p) -> bool
{
  auto* buf = reinterpret_cast<uint8_t*>(&p);

  for (auto i = 0; i < sizeof(message_packet); i++) {
    if (!dev.read(&buf[i])) {
      return false;
    }
  }

  if (p->compute_checksum() != p->checksum) {
    return false;
  }

  return true;
}

auto
capture_program::wait_ack(serial_device& dev) -> bool
{
  message_packet p;

  if (!recv(dev, &p)) {
    return false;
  }

  return p.id == message_id::ack;
}

} // namespace mt
