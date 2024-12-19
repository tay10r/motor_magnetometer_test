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
log_debug(const char* what)
{
  std::cout << "[debug]: " << what << std::endl;
}

void
capture_program::run(serial_device& dev, clock& clk)
{
  switch (state_) {
    case capture_state::initial:
      log_info("Sending reset message.");
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
      log_info("Sending start message.");
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
      timeout_start_ = clk.read();
      next_state();
      break;
    case capture_state::wait_status:
      read_status(dev, clk);
      break;
    case capture_state::read:
      request_next_byte(dev, clk);
      break;
    case capture_state::read_response:
      read_next_byte(dev, clk);
      break;
    case capture_state::done:
      break;
  }
}

void
capture_program::read_status(serial_device& dev, clock& clk)
{
  message_packet status_pkt;
  if (!recv(dev, &status_pkt)) {
    if (elapsed(timeout_start_, clk) > timeout()) {
      // timeout occurred, try to read the state again
      state_ = capture_state::status;
    }
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
      // request status again
      state_ = capture_state::status;
      break;
    case status::acq_complete:
      log_info("acquisition done");
      next_state();
      break;
    default:
      break;
  }
}

void
capture_program::request_next_byte(serial_device& dev, clock& clk)
{
  const uint16_t read_offset = static_cast<uint16_t>(read_buffer_.size());

  if (!send(dev, message_id::read_request, read_offset & 0xff, (read_offset >> 8) & 0xff)) {
    return;
  }

  next_state();

  timeout_start_ = clk.read();
}

void
capture_program::read_next_byte(serial_device& dev, clock& clk)
{
  message_packet pkt;

  if (!recv(dev, &pkt) || pkt.id != message_id::read_response) {
    if (elapsed(timeout_start_, clk) > timeout()) {
      state_ = capture_state::read;
    }
    return;
  }

  log_info("received data from sample buffer");

  read_buffer_.emplace_back(pkt.operands[0]);

  if (read_buffer_.size() >= max_read_size()) {
    next_state();
  } else {
    state_ = capture_state::read;
  }
}

auto
capture_program::send(serial_device& dev, message_id id, uint8_t op1, uint8_t op2) -> bool
{
  message_packet pkt;
  pkt.id = id;
  pkt.operands[0] = op1;
  pkt.operands[1] = op2;
  pkt.checksum = pkt.compute_checksum();

  const auto* buffer = reinterpret_cast<const uint8_t*>(&pkt);

  for (auto i = 0; i < sizeof(message_packet); i++) {
    if (!dev.write(buffer[i])) {
      return false;
    }
  }

  return true;
}

auto
capture_program::recv(serial_device& dev, message_packet* p) -> bool
{
  auto* buf = reinterpret_cast<uint8_t*>(p);

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
