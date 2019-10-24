#include <Arduino.h>
#include <SPI.h>

#define PRESS_OUT_XL 0x28
#define PRESS_OUT_L 0x29
#define PRESS_OUT_H 0x2a
#define TEMP_OUT_L 0x2b
#define TEMP_OUT_H 0x2c

const int SensorSS = 2;

/**
 * Read bytes from a register
 * @param addr The register address
 * @param byteCount The number of bytes to read
 * @param buffer* A pointer to a byte buffer
 */
void readRegister(uint8_t addr, uint8_t byteCount, uint8_t* buffer) {
  // Select slave
  digitalWrite(SensorSS, LOW);
  // Set READWRITE bit to 1 (READ)
  uint8_t sendByte = addr | (1 << 7);
  // Transfer address
  SPI.transfer(sendByte);
  // Read {byteCount} number of bytes and write them to the buffer
  uint8_t index = 0;
  while (index < byteCount) {
    // Read a single byte
    uint8_t inByte = SPI.transfer(0x00);
    buffer[index] = inByte;
    index++;
  }
  // Release slave
  digitalWrite(SensorSS, HIGH);
}

/**
 * Read a single byte from the register
 * @param addr The register address
 */
uint8_t readRegisterSingle(uint8_t addr) {
  uint8_t response;
  readRegister(addr, 1, &response);
  return response;
}

/**
 * Write a byte to a register
 * @param addr The register address
 * @param data The data byte to write
 */
uint8_t writeRegister(uint8_t addr, uint8_t data) {
  // Select slave
  digitalWrite(SensorSS, LOW);
  // Set READWRITE bit to 0 (WRITE)
  uint8_t sendByte = (addr << 1) >> 1;
  // Transfer write bytes
  SPI.transfer(sendByte);
  SPI.transfer(data);
  // Release slave
  digitalWrite(SensorSS, HIGH);
}

/**
 * Disable FIFO, Disable address increase (IF_ADD_INC), trigger ONE_SHOT
 */
void doMeasurement() {
  // Write 1 to ONE_SHOT
  writeRegister(0x11, 0b00000001);
}

/**
 * Read temperature from device
 */
float readTemperature() {
  doMeasurement();
  uint8_t high = readRegisterSingle(TEMP_OUT_H);
  uint8_t low = readRegisterSingle(TEMP_OUT_L);
  uint16_t temperature = low | (high << 8);
  return (float)temperature / 100.0f;
}

/**
 * Read pressure in hPa from device
 */
float readPressure() {
  doMeasurement();
  uint8_t byte_XL = readRegisterSingle(PRESS_OUT_XL);
  uint8_t byte_L = readRegisterSingle(PRESS_OUT_L);
  uint8_t byte_H = readRegisterSingle(PRESS_OUT_H);
  uint32_t pressureLSB = (((uint32_t)byte_H << 16) | ((uint32_t)byte_L << 8) | ((uint32_t)byte_XL));
  return pressureLSB / 4096.0f;
}

void setup() {
  Serial.begin(9600);
  //
  pinMode(SensorSS, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  //
  digitalWrite(SensorSS, HIGH);
  //
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  // SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  //
  // writeRegister(0x11, 0b00000000);
}

void loop() {
  float temp = readTemperature();
  float pressure = readPressure();
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print("\t\tPressure: ");
  Serial.println(pressure);
  delay(1000);
}