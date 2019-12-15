#include "communication.h"

void setup() {
    // Setup serial communication   
    Serial.begin(115200);
}

void loop() {
    // tx_packet.button1 = true;
    // tx_packet.button2 = false;
    // tx_packet.slot_encoder = true;
    // tx_packet.encoder_count = -10000;
    // tx_packet.encoder_velocity = 45.6;
    // tx_packet.temperature = 24;
    // tx_packet.sharp_distance = 40000;
    // tx_packet.ultrasonic_distance = 50000;
    // tx_packet.roll_angle = 0.1;
    // tx_packet.pitch_angle = 0.2;
    // tx_packet.yaw_angle = 0.2;
    // tx_packet.accel_x = 10;
    // tx_packet.accel_y = 20;
    // tx_packet.accel_z = 30;
    // tx_packet.gyro_x = 40;
    // tx_packet.gyro_y = 50;
    // tx_packet.gyro_z = 60;

    // send_data();

    if (recieve_data()) {
        Serial.println(rx_packet.global_switch);
        Serial.println(rx_packet.state);
        Serial.println(rx_packet.servo_angle);
        Serial.println(rx_packet.motor_angle);
        Serial.println(rx_packet.motor_velocity);
        Serial.println(rx_packet.stepper_value);
        Serial.println(rx_packet.stepper_dir);
        Serial.println(rx_packet.stepper_flag);
        Serial.println();
    }

    // Serial.println(ret);
    // delay(50);
}
