/**
  Delta Autonomy RC Car Firmware
  @author Heethesh Vhavle
  @version 1.0.0
  @date 31-OCT-2019
*/

#include <Servo.h>
#include "communication.h"

/************************** MACROS **************************/

#define DEFAULT_LED 13
#define LEDR 9
#define LEDG 11
#define LEDB 10
#define BLDC 11
#define SERVO 10
#define MIN_THROTTLE 76 //76
#define MAX_THROTTLE 110 //90
#define MIN_STEER 70
#define MID_STEER 98
#define MAX_STEER 130
#define SERVO_STEP 10


/************************** GLOBALS **************************/

// Colors
enum class Color { red, yellow, green, cyan, blue, magenta, on, off };
Servo steer_servo, motor_esc;
uint8_t last_servo_signal = 0;
/************************** RGB LED **************************/

void set_rgb_digital(int r, int g, int b) {
  digitalWrite(LEDR, !r);
  digitalWrite(LEDG, !g);
  digitalWrite(LEDB, !b);
}

void set_rgb_analog(int r, int g, int b) {
  analogWrite(LEDR, 255 - r);
  analogWrite(LEDG, 255 - g);
  analogWrite(LEDB, 255 - b);
}

void set_rgb_color(Color color) {
  switch (color) {
    case Color::red:
      set_rgb_digital(1, 0, 0); break;
    case Color::yellow:
      set_rgb_digital(1, 1, 0); break;
    case Color::green:
      set_rgb_digital(0, 1, 0); break;
    case Color::cyan:
      set_rgb_digital(0, 1, 1); break;
    case Color::blue:
      set_rgb_digital(0, 0, 1); break;
    case Color::magenta:
      set_rgb_digital(1, 0, 1); break;
    case Color::on:
      set_rgb_digital(1, 1, 1); break;
    case Color::off:
      set_rgb_digital(0, 0, 0); break;
  }
}

void rgb_init() {
  // Set pin mode
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // Set default state
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}

/************************* CONTROLS *************************/

void write_servo(uint8_t value) {
  // TODO: Implement slow servo sweep
  uint8_t servo_signal;
  if (value > 127)
    servo_signal = map(value, 127, 255, MID_STEER, MAX_STEER);
  else
    servo_signal = map(value, 0, 127, MIN_STEER, MID_STEER);
  servo_signal = constrain((servo_signal), last_servo_signal-SERVO_STEP, last_servo_signal+SERVO_STEP);

  steer_servo.write(servo_signal);
  last_servo_signal = servo_signal;
}

void write_bldc(uint8_t value) {
  // TODO: Implement slow sweep
  uint8_t motor_signal = map(value, 0, 255, MIN_THROTTLE, MAX_THROTTLE);
  motor_esc.write(motor_signal);
}

void actuator_reset() {
  steer_servo.write(MID_STEER);
  motor_esc.write(MIN_THROTTLE);
}

void actuator_init() {
  // Set pin mode
  steer_servo.attach(SERVO);
  motor_esc.attach(BLDC);

  // Set default state
  actuator_reset();
}

/************************* ROUTINES *************************/

void main_routine() {
  // Recieve data from joystick
  if (!recieve_data()) return;

  if (rx_packet.global_arm) {
//    analogWrite(LEDR, 255 - rx_packet.throttle);
//    analogWrite(LEDB, 255 - rx_packet.steering);
//    digitalWrite(LEDG, !rx_packet.direction);
    write_bldc(rx_packet.throttle);
    write_servo(rx_packet.steering);
  } else {
//    set_rgb_color(Color::off);
    actuator_reset();
    
  }
}

void display() {
  Serial.print("ARM: "); Serial.print(rx_packet.global_arm);
  Serial.print(" | THR: "); Serial.print(rx_packet.throttle);
  Serial.print(" | DIR: "); Serial.print(rx_packet.direction);
  Serial.print(" | STR: "); Serial.println(rx_packet.steering);
}

/*************************** MAIN ***************************/

void setup() {
//  rgb_init();
  actuator_init();
  pinMode(DEFAULT_LED, OUTPUT);
  Serial.begin(9600);
  Serial.println("\n**** Delta Autonomy RC Car Firmware ****\n");
}

void loop() {
  main_routine();
  // set_rgb_color(Color::yellow);
}
