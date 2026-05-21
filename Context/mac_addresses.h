// ===================================================
// MAC Addresses — mech_bot ESP-NOW network
//
// Both T-Display S3 boards communicate over ESP-NOW.
// These addresses are board-specific hardware identifiers.
// Update if boards are swapped.
//
// To find a board's MAC: flash any sketch and read Serial output,
// or use: WiFi.macAddress() in setup().
// ===================================================

#pragma once

// Master T-Display S3
#define MASTER_MAC_ARRAY  { 0x30, 0x30, 0xF9, 0x59, 0x31, 0x78 }

// Slave T-Display S3
#define SLAVE_MAC_ARRAY   { 0x30, 0x30, 0xF9, 0x59, 0xCE, 0xE4 }
