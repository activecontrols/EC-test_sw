#include "CommandRouter.h"
#include "CommsSerial.h"
#include "SPI.h"
#include <Arduino.h>

// ==========================================
// PIN DEFINITIONS & PERIPHERAL CONFIGURATION
// ==========================================
#define SPI1_MOSI PD7
#define SPI1_MISO PG9
#define SPI1_SCK PG11

#define ADC_CS0_PIN PD0
#define ADC_CS1_PIN PD1

CommsSerial_t<HardwareSerial> HW_CommsSerial(PIN_SERIAL_RX, PIN_SERIAL_TX);
CommsSerial_t<USBSerial> USB_CommsSerial;

// ==========================================
// MAX31856 DIRECT REGISTER WRITER / READER
// ==========================================

void max31856_WriteReg(uint8_t csPin, uint8_t reg, uint8_t value) {
  // To write to a register on the MAX31856, the MSB of the address must be 1 (Reg | 0x80)
  digitalWrite(csPin, LOW);
  delayMicroseconds(5);
  SPI.transfer(reg | 0x80);
  SPI.transfer(value);
  digitalWrite(csPin, HIGH);
  delayMicroseconds(5);
}

uint8_t max31856_ReadReg(uint8_t csPin, uint8_t reg) {
  // To read a register, the MSB of the address must be 0
  digitalWrite(csPin, LOW);
  delayMicroseconds(5);
  SPI.transfer(reg & 0x7F);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  delayMicroseconds(5);
  return val;
}

// Configures the MAX31856 for Continuous Conversion & K-Type Thermocouple
void initMAX31856(uint8_t csPin) {
  // Register 0x00 (CR0): Continuous Conversion Mode = 1, Cutoff Filter = 60Hz -> 0x80
  max31856_WriteReg(csPin, 0x00, 0x80);

  // Register 0x01 (CR1): Averaging = 1 sample, Thermocouple Type = K-Type (0x03) -> 0x03
  max31856_WriteReg(csPin, 0x01, 0x03);
}

// Reads the 19-bit Temperature register and converts it to true Celsius
float readThermocoupleTemp(uint8_t csPin) {
  digitalWrite(csPin, LOW);
  delayMicroseconds(5);

  SPI.transfer(0x0C); // Start reading from the Thermocouple Temperature Byte 2
  uint8_t byte2 = SPI.transfer(0x00);
  uint8_t byte1 = SPI.transfer(0x00);
  uint8_t byte0 = SPI.transfer(0x00);

  digitalWrite(csPin, HIGH);
  delayMicroseconds(5);

  // Combine the bytes into a single signed 32-bit integer
  int32_t rawTemp = ((int32_t)byte2 << 16) | ((int32_t)byte1 << 8) | byte0;

  // Shift right by 5 bits because data is aligned to the MSB of the 24-bit structure
  rawTemp >>= 5;

  // Check for negative sign extension on the 19-bit value
  if (byte2 & 0x80) {
    rawTemp |= 0xFFF80000;
  }

  // LSB multiplier for MAX31856 Thermocouple temperature is 0.0078125°C
  return (float)rawTemp * 0.0078125;
}


// ==========================================
// MAIN SETUP
// ==========================================
void setup() {
  delay(3000);

  // Lock out other SPI slaves sharing the bus
  pinMode(PB7, OUTPUT);
  digitalWrite(PB7, HIGH);
  pinMode(PC12, OUTPUT);
  digitalWrite(PC12, HIGH);
  pinMode(PE7, OUTPUT);
  digitalWrite(PE7, HIGH);
  pinMode(PE4, OUTPUT);
  digitalWrite(PE4, HIGH);
  pinMode(PB3, OUTPUT);
  digitalWrite(PB3, HIGH);
  pinMode(PB4, OUTPUT);
  digitalWrite(PB4, HIGH);
  pinMode(PB5, OUTPUT);
  digitalWrite(PB5, HIGH);

  // Initialize Chip Select lines
  pinMode(ADC_CS0_PIN, OUTPUT);
  digitalWrite(ADC_CS0_PIN, HIGH);
  pinMode(ADC_CS1_PIN, OUTPUT);
  digitalWrite(ADC_CS1_PIN, HIGH);

  // Setup STM32H7 hardware pins for SPI1
  SPI.begin();
  SPI.setMOSI(SPI1_MOSI);
  SPI.setMISO(SPI1_MISO);
  SPI.setSCLK(SPI1_SCK);

  // MAX31856 supports SPI Mode 1 or Mode 3 up to 5MHz. Locking Mode 3 at 1MHz.
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

  HW_CommsSerial.begin(57600);
  USB_CommsSerial.begin(57600);
  CommsSerial.println("SPI Bus Initialized for MAX31856.");

  // Initialize both MAX31856 converters
  initMAX31856(ADC_CS0_PIN);
  initMAX31856(ADC_CS1_PIN);

  CommsSerial.println("MAX31856 Configured for Continuous K-Type Capture.");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  static uint32_t lastSampleTime = 0;
  uint32_t currentMillis = millis();

  if (currentMillis - lastSampleTime >= 500) {
    lastSampleTime = currentMillis;

    // --- Read Device 0 ---
    uint8_t fault0 = max31856_ReadReg(ADC_CS0_PIN, 0x0F); // Read Fault Status Register
    float temp0 = readThermocoupleTemp(ADC_CS0_PIN);

    CommsSerial.print("MAX0 Temp: ");
    CommsSerial.print(temp0);
    CommsSerial.print(" C | Fault Status: 0x");
    CommsSerial.println(fault0, HEX);

    // --- Read Device 1 ---
    uint8_t fault1 = max31856_ReadReg(ADC_CS1_PIN, 0x0F);
    float temp1 = readThermocoupleTemp(ADC_CS1_PIN);

    CommsSerial.print("MAX1 Temp: ");
    CommsSerial.print(temp1);
    CommsSerial.print(" C | Fault Status: 0x");
    CommsSerial.println(fault1, HEX);
    CommsSerial.println("------------------------------------");
  }
}