#include <Arduino.h>
#include <stdint.h>

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

// Structure to handle the various outputs to the joystick
typedef struct TXDataPacket {
  // Boolean data
  uint8_t state;
} __attribute__((__packed__));

// Structure to handle the various inputs from the joystick
typedef struct RXDataPacket {
  // Arm (safety switch)
  uint8_t global_arm;

  // Steering angle
  uint8_t steering;

  // Throttle data
  uint8_t throttle;
  uint8_t direction;
} __attribute__((__packed__));

extern TXDataPacket tx_packet;
extern RXDataPacket rx_packet;

void send_data();
bool recieve_data();
void clear_buffer();

#endif  /* CONTROLLER_H_ */
