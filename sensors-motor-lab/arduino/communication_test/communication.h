#include <Arduino.h>
#include <stdint.h>

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

// Structure to handle the various outputs to the GUI
typedef struct TXDataPacket {
    // Boolean data
    uint8_t slot_encoder;

    // Encoder data
    int32_t encoder_count;
    float encoder_velocity;

    // Temperature sensor
    uint8_t temperature;

    // Ultrasonic sensor
    uint16_t ultrasonic_distance;

    // Flex sensor
    uint16_t flex_sensor;

    // Ouput servo angle
    uint8_t servo_angle;
} __attribute__((__packed__));

// Structure to handle the various inputs from the GUI
typedef struct RXDataPacket {
    // Enable for demo
    uint8_t global_switch;

    // State select
    uint8_t state;

    // Input servo angle
    uint8_t servo_angle;

    // Input motor angle
    int16_t motor_angle;
    int16_t motor_velocity;

    // Stepper motor
    uint16_t stepper_value;
    uint8_t stepper_dir;
    uint8_t stepper_flag;
} __attribute__((__packed__));

extern TXDataPacket tx_packet;
extern RXDataPacket rx_packet;

void send_data();
bool recieve_data();
void clear_buffer();

#endif
