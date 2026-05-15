#include "ADS131M0x.h"
#include "Adafruit_MAX31856.h"
#include "CommandRouter.h"
#include "CommsSerial.h"
#include "SPI.h"
#include "SPI_Demux.h"
#include "SPI_Fixed.h"
#include "Thermocouples.h"

CommsSerial_t<HardwareSerial> HW_CommsSerial(PIN_SERIAL_RX, PIN_SERIAL_TX);
CommsSerial_t<USBSerial> USB_CommsSerial;

void ping(const char *args) {
  CommsSerial.println("pong");
  CommsSerial.print("args: ");
  CommsSerial.println(args == nullptr ? "null" : args);
}

void setup() {

  delay(3000);
  pinMode(PE2, OUTPUT); // cs for pt1
  digitalWrite(PE2, HIGH);
  pinMode(PG10, OUTPUT); // sync line
  digitalWrite(PG10, HIGH);

  SPI.begin(); // spi is a shared interface, so we always begin here
  HW_CommsSerial.begin(57600);
  USB_CommsSerial.begin(57600);
  CommsSerial.println("Controller started.");

  ADS131M0x adc(PE2); // for pt1
  adc.resetDevice();

  while (true) {
    adcOutput out = adc.readADC();

    if (!out.crc_ok) {
      CommsSerial.println("CRC Error - check signal integrity/wiring");
    } else {
      // 1. Get raw value from Channel 1 (where the swap occurred)
      int32_t raw_val = out.ch1;

      // 2. Sign-extend 24-bit to 32-bit
      // This ensures negative numbers are handled correctly by the MCU
      if (raw_val & 0x800000) {
        raw_val |= 0xFF000000;
      }

      // 3. Correct for the PCB Swap (AIN1P grounded, AIN1N is signal)
      // Since (0 - Signal) = -Signal, we multiply by -1 to get the true voltage
      int32_t corrected_val = -raw_val;

      // 4. Convert to Volts using 3.3V Reference
      // Formula: (Corrected_Value / Max_Positive_24bit_Int) * Vref
      // Max 24-bit positive is 2^23 - 1 = 8,388,607
      float voltage = (float)corrected_val * (3.3f / 8388607.0f) * 1.1667;

      // 5. Output results
      CommsSerial.print("Raw: ");
      CommsSerial.print(corrected_val);
      CommsSerial.print(" | Volts: ");
      CommsSerial.println(voltage, 4);
    }

    delay(100);
  }
  // TODO - configure CS somewhere else!
  digitalWrite(PB7, HIGH);
  digitalWrite(PC12, HIGH);

  digitalWrite(PE7, HIGH); // MAG CS 2
  digitalWrite(PE4, HIGH); // MAG CS 3

  digitalWrite(PB3, HIGH); // IMU CS 3
  digitalWrite(PB4, HIGH); // IMU CS 2
  digitalWrite(PB5, HIGH); // IMU CS 1

  // TODO - configure CS somewhere else!
  pinMode(PB7, OUTPUT);
  pinMode(PC12, OUTPUT);
  pinMode(PE7, OUTPUT);
  pinMode(PE4, OUTPUT);

  pinMode(PB3, OUTPUT);
  pinMode(PB4, OUTPUT);
  pinMode(PB5, OUTPUT);

  pinMode(PB6, OUTPUT);
  pinMode(PC4, OUTPUT);
  pinMode(PC5, OUTPUT);

  while (1) {
    digitalWrite(PC4, HIGH);
    digitalWrite(PC5, HIGH);
    delay(500);
    digitalWrite(PC4, LOW);
    digitalWrite(PC5, LOW);
    delay(500);
    CommsSerial.printf("Loop...");
  }

  CommandRouter::begin();
  CommandRouter::add(ping, "ping"); // example registration
}

void loop() {
  while (CommsSerial.available()) {
    CommandRouter::receive_byte(CommsSerial.read());
  }
}