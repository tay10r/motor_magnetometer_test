#include <Wire.h>

#include <SparkFun_MMC5983MA_Arduino_Library.h>

#include "common.h"

#define MOTOR_PIN 6
#define NUM_SAMPLES 256

namespace {

SFE_MMC5983MA g_magnetometer;

uint32_t g_sample_buffer[NUM_SAMPLES];

} // namespace

void setup()
{
  SerialUSB.begin(115200);
  Wire.begin();
  if (g_magnetometer.begin() == false)
  {
    // TODO : report this with the program interface.
    while (true)
    {}
  }
  g_magnetometer.softReset();
  pinMode(MOTOR_PIN, OUTPUT);
}

namespace {

auto elapsed(uint32_t t0, uint32_t t1) -> uint32_t
{
  if (t1 < t0) {
    return (0xffffffffUL - t1) + t0;
  } else {
    return t1 - t0;
  }
}

bool readSamples()
{
  const int period = 10;

  auto success = true;

  for (int i = 0; i < NUM_SAMPLES; i++) {

    auto t0 = millis();

    const auto motor_state = (i / (NUM_SAMPLES / 4)) % 2;

    digitalWrite(MOTOR_PIN, motor_state ? HIGH : LOW);

    g_sample_buffer[i] = g_magnetometer.getMeasurementZ();

    auto t1 = millis();
    auto dt = elapsed(t0, t1);

    if (dt < period) {
      delay(period - dt);
    } else if (dt > period) {
      success = false;
      break;
    }
  }

  digitalWrite(MOTOR_PIN, LOW);

  return success;
}

void writeSamples()
{
  for (auto i = 0; i < NUM_SAMPLES; i++) {
    SerialUSB.println(g_sample_buffer[i]);
  }
}

} // namespace

void loop()
{
  digitalWrite(MOTOR_PIN, LOW);

  if (SerialUSB.available() < 1) {
    delay(10);
    return;
  }
  auto c = SerialUSB.read();
  switch (c) {
    case 'r':
      readSamples();
      break;
    case 'w':
      writeSamples();
      break;
    default:
      break;
  }
}
