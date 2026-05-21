// Flash this to each T-Display S3 board to find its MAC address.
// Open Serial Monitor at 115200 baud to read the output.
// Update Context/mac_addresses.h and platformio.ini with the results.

#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);

  Serial.println("==========================");
  Serial.print("Board MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("==========================");
  Serial.println("Copy this into Context/mac_addresses.h");

  // Also print in byte array format ready to paste
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.print("Byte format: {");
  for (int i = 0; i < 6; i++) {
    Serial.print("0x");
    if (mac[i] < 0x10) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(", ");
  }
  Serial.println("}");
}

void loop() {}
