#include <SPI.h>
#include <Adafruit_MAX31865.h>

// --- Pin Definitions ---
#define MAX_CS 10
#define MAX_MISO 12
#define MAX_MOSI 11
#define MAX_CLK 13

// --- MAX31865 Configuration ---
// Initialize MAX31865 with Chip Select pin
Adafruit_MAX31865 max31865(MAX_CS);
const uint8_t RTD_WIRES = MAX31865_3WIRE;  // Or MAX31865_2WIRE or MAX31865_4WIRE
const float RTD_RESISTANCE = 100.0;   // Resistance of your RTD at 0°C (e.g., 100.0 for PT100)
const float REF_RESISTANCE = 430.0;   // Value of your reference resistor

void setup() {
  Serial.begin(115200);
  Serial.println("MAX31865 Test");

  // --- SPI Initialization ---
  // Initialize SPI with defined pins
  SPI.begin(MAX_CLK, MAX_MISO, MAX_MOSI, MAX_CS);

  // Initialize MAX31865
  if (!max31865.begin((max31865_numwires_t)RTD_WIRES)) {
    Serial.println("Failed to initialize MAX31865. Check wiring!");
    while (1)
      ; // Stop here if initialization fails
  }
}

void loop() {
  // --- Read RTD Data ---
  uint16_t rtd_raw = max31865.readRTD(); // Read raw RTD value

  // --- Check for Faults ---
  uint8_t fault = max31865.readFault();
  if (fault) {
    Serial.print("MAX31865 Fault: ");

    // --- Replace these with the CORRECT fault constants from your library ---
    if (fault & 0x01) // Example: Replace with actual constant if different
      Serial.println("RTD High or Low");
    if (fault & 0x02) // Example: Replace with actual constant if different
      Serial.println("RTD was not connected");
    if (fault & 0x04) // Example: Replace with actual constant if different
      Serial.println("Under Voltage");
    if (fault & 0x08) // Example: Replace with actual constant if different
      Serial.println("Over Voltage");

    max31865.clearFault();
    delay(1000);
    return; // Skip temperature calculation if there's a fault
  }

  // --- Calculate Temperature ---
  float temperature = max31865.temperature(RTD_RESISTANCE, REF_RESISTANCE);

  // --- Display Results ---
  Serial.print("Raw RTD Value: ");
  Serial.println(rtd_raw);
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  delay(1000);
}