/*
  X/Y/Z magnetic field and raw readings from the MMC5983MA
  By: Nathan Seidle and Ricardo Ramos
  SparkFun Electronics
  Date: April 14th, 2022
  License: SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/19034

  This example demonstrates how to read the basic X/Y/Z readings from the sensor over Qwiic

  Hardware Connections:
  Plug a Qwiic cable into the sensor and a RedBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper
  (https://www.sparkfun.com/products/17912) Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h>

#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA

namespace {

SFE_MMC5983MA myMag;

class Timer
{
public:
  unsigned long int step(unsigned long int dt, unsigned long int period)
  {
    remainder_ += dt;
    const auto ticks = remainder_ / period;
    remainder_ -= period * ticks;
    return ticks;
  }

private:
  unsigned long int remainder_{};
};
const int measureInterval = 10;
unsigned long int lastMeasureTime = 0;
Timer measureTimer;

const int motorPin = 4;
int motorState = 0;
int motorStateChangeInterval = 1000;
Timer motorStateTimer;
unsigned long int lastMotorTime = 0;

int numSamples = 0;

} // namespace

char dataBuffer[20 * 1024];
void setup()
{
    Serial.begin(115200);

    Wire.begin();

    if (myMag.begin() == false)
    {
        Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
        while (true)
            ;
    }

    myMag.softReset();

    pinMode(motorPin, OUTPUT);
}

void reportMeasurement(uint32_t x, uint32_t y, uint32_t z, unsigned long int measurementTime)
{
    dataBuffer[numSamples % sizeof(dataBuffer)] = x;

    static_assert(sizeof(uint32_t) == sizeof(unsigned long int), "The size of ulong is not right.");

    uint8_t sendBuffer[sizeof(uint32_t) * 4 + 4];

    sendBuffer[0] = 0xff; // magic number
    sendBuffer[1] = 0xfd; // magic number
    sendBuffer[2] = 0; // crc
    sendBuffer[3] = motorState;
    memcpy(&sendBuffer[4], &x, sizeof(x));
    memcpy(&sendBuffer[8], &y, sizeof(y));
    memcpy(&sendBuffer[12], &z, sizeof(z));
    memcpy(&sendBuffer[16], &measurementTime, sizeof(measurementTime));

    uint8_t checksum = 0;
    for (auto i = 3; i < sizeof(sendBuffer); i++) {
      checksum ^= sendBuffer[i];
    }
    sendBuffer[2] = checksum;

    Serial.write(sendBuffer, sizeof(sendBuffer));
}

void doMeasurement(uint32_t* x, uint32_t* y, uint32_t* z, unsigned long int* t)
{
    myMag.getMeasurementXYZ(x, y, z);
    *t = millis();
}

void loop()
{
    const auto measureT = millis();
    const auto measureDt = measureT - lastMeasureTime;
    lastMeasureTime = measureT;
    if (measureTimer.step(measureDt, measureInterval) > 0) {
      uint32_t x = 0;
      uint32_t y = 0;
      uint32_t z = 0;
      unsigned long int t = 0;
      doMeasurement(&x, &y, &z, &t);
      reportMeasurement(x, y, z, t);
    }

    const auto t = millis();
    const auto dt = t - lastMotorTime;
    lastMotorTime = t;

    if (motorStateTimer.step(static_cast<unsigned long int>(dt), motorStateChangeInterval)) {
      motorState = !motorState;
      //digitalWrite(motorPin, motorState ? HIGH : LOW);
      //digitalWrite(motorPin, HIGH);
      digitalWrite(motorPin, LOW);
    }
}

/*

void setup() {
  pinMode(pin, OUTPUT);
  // put your setup code here, to run once:

}

void loop() {
  digitalWrite(pin, HIGH);
  delay(onInterval);
  digitalWrite(pin, LOW);
  delay(offInterval);
  // put your main code here, to run repeatedly:

}
*/
