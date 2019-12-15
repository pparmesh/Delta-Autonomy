#include "communication.h"

TXDataPacket tx_packet;
RXDataPacket rx_packet;

void send_data()
{
    // Packet metadata
    uint8_t size = sizeof(TXDataPacket);
    uint8_t checksum = 0;

    // Payload buffer
    uint8_t buff[size];

    // Convert struct to bytes
    memcpy(buff, &tx_packet, size);

    // Send header
    Serial.print("$>");

    // Send data length
    Serial.write(size);

    // Send byte array and update checksum
    for (int i = 0; i < size; i++) {
        Serial.write(buff[i]);
        checksum ^= buff[i];
    }

    // Send checksum
    Serial.write(checksum);
}

void clear_buffer()
{
    while (Serial.available()) {
        Serial.read();
    }
    Serial.flush();
}

bool recieve_data()
{
    // Packet metadata
    uint8_t size = sizeof(RXDataPacket);
    uint8_t checksum = 0;

    // Payload buffer
    uint8_t buff[size];

    // Check (frame + header) length
    if (Serial.available() >= size + 4) {
        // Check header of data frame
        if (Serial.read() != '$') {
            return;
        }
        if (Serial.read() != '<') {
            return;
        }

        // Data length byte
        Serial.read();

        // Read data bytes
        for (int i = 0; i < size; i++) {
            buff[i] = Serial.read();
            checksum ^= buff[i];
        }

        // Discard frame if checksum does not match
        if (Serial.read() != checksum) {
            return;
        }

        // Convert bytes to struct
        memcpy(&rx_packet, buff, size);

        // Success
        return true;
    }

    // Failure
    return false;
}
