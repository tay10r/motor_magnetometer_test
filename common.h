#pragma once

#include <stdint.h>

struct __attribute__((packed)) packet final {
  uint8_t magic[2];
  uint8_t checksum;
  uint8_t motor_state;
  uint32_t xyz[3];
  uint32_t time;

  packet() {
    static_assert(sizeof(packet) == 20, "Size of packet is not correct.");
  }

  [[nodiscard]] auto compute_checksum() const -> uint8_t {
    auto* ptr = reinterpret_cast<const uint8_t*>(this);
    uint8_t result = 0;
    for (uint32_t i = 3; i < sizeof(packet); i++) {
      result ^= ptr[i];
    }
    return result;
  }
};
