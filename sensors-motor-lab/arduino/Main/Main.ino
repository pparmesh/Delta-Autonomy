/******************************************************************************
  Team Delta Autonomy - Sensors and Motors Assignment
******************************************************************************/

/************************** Headers **************************/

#include "TimerFour.h"
#include "communication.h"
#include <PID_v1.h>
#include <Servo.h>

/************************** Macros **************************/

// States
#define STATE_RESERVED 0
#define STATE_DC_POS_GUI 1
#define STATE_DC_VEL_GUI 2
#define STATE_DC_POS_SENS 3
#define STATE_DC_VEL_SENS 4
#define STATE_STEPPER_GUI 5
#define STATE_STEPPER_SENS 6
#define STATE_SERVO_GUI 7
#define STATE_SERVO_SENS 8

// Stepper motorservo
#define step_stp 4
#define step_dir 5
#define step_MS1 11 // 3
#define step_MS2 48
#define step_EN 46

// DC Motor
#define ENC_CH1 19 // Quadrature encoder A pin
#define ENC_CH2 20 // Quadrature encoder B pin
#define MOTOR_EN 3 // PWM outputs to L298N H-Bridge motor driver module
#define MOTOR_A1 51
#define MOTOR_A2 53
#define MIN_RPM 55
#define ROT_FACTOR 1

// Servo
#define SERVO_PIN 9

// Sensors
#define POTENTIOMETER_PIN A15
#define flexpin A4
#define photo_in 8
#define photo_out A0
#define TMP36 A1
#define lv_Sonar A3

// Debug
#define buttonPin 2
#define DEFAULT_LED 32 

/************************** Globals **************************/

bool circuitState = LOW;
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 50; // the debounce time; increase if the output flickers
unsigned long last_send_time = 0;

// This is the reading of the SONAR sensor sensor
int min_SONAR = 6;
int max_SONAR = 24;

// ISR variables
volatile double encoder_count = 0;
volatile double Motor_prev_count = 0;
volatile double RPM;

// Position Control Initialization
double kp_pos = 0.8, ki_pos = 0, kd_pos = 0; // modify for optimal performance
double input_pos = 0, output_pos = 0, setpoint_pos = 0;
PID position_PID(&input_pos, &output_pos, &setpoint_pos, kp_pos, ki_pos, kd_pos, DIRECT);

// Velocity Control Initialization
double kp_vel = 4.9, ki_vel = 0, kd_vel = 0.0; // modify for optimal performance
double input_vel = 0, output_vel = 0, setpoint_vel = 0;
PID velocity_PID(&input_vel, &output_vel, &setpoint_vel, kp_vel, ki_vel, kd_vel, DIRECT);

struct Button {
    int state;
    int last_state = LOW;
    int time = millis();
};

volatile Button button;
Servo servo_motor;

/************************** Stepper **************************/

//Reset Stepper pins to default states
void resetStepperPins()
{
    digitalWrite(step_stp, LOW);
    digitalWrite(step_dir, LOW);
    digitalWrite(step_MS1, LOW);
    digitalWrite(step_MS2, LOW);
    digitalWrite(step_EN, HIGH);
}

//Microstep function for stepper
void StepperStep()
{
    for (int x = 1; x < 50; x++) // Loop the forward stepping enough times for motion to be visible
    {
        digitalWrite(step_stp, HIGH); // Trigger one step forward
        delay(1);
        digitalWrite(step_stp, LOW); // Pull step pin low so it can be triggered again
        delay(1);
    }
}

uint8_t read_slot_encoder()
{
    digitalWrite(photo_in, HIGH);
    uint8_t val = constrain(analogRead(photo_out), 0, 100);
    tx_packet.slot_encoder = val;
    return val;
}

// Stepper Function with Slot Encoder
void StepperMain()
{
    uint8_t val = read_slot_encoder();

    if (val > 5) {
        digitalWrite(step_EN, LOW); // Pull enable pin low to allow motor control
        StepperStep();
    } else {
        digitalWrite(step_EN, HIGH);
    }
    resetStepperPins();
}

// Microstep function for stepper
void StepperPosStep(uint16_t angle, uint8_t dir)
{
    if (!rx_packet.stepper_flag) return;

    if (dir == 0) {
        digitalWrite(step_dir, HIGH); // Pull direction pin low to move "forward" and high to move "reverse"
    } else {
        digitalWrite(step_dir, LOW);
    }

    int t = map(angle, 0, 360, 1, 200);
    for (int x = 1; x < t; x++) // Loop the forward stepping enough times for motion to be visible
    {
        digitalWrite(step_stp, HIGH); // Trigger one step forward
        delay(1);
        digitalWrite(step_stp, LOW); // Pull step pin low so it can be triggered again
        delay(1);
    }

    rx_packet.stepper_flag = 0;
}

// Stepper function for position control
void StepperPos()
{
    uint16_t angle = rx_packet.stepper_value;
    uint8_t dir = rx_packet.stepper_dir;
    read_slot_encoder();

    digitalWrite(step_EN, LOW); // Pull enable pin low to allow motor control
    StepperPosStep(angle, dir);
    resetStepperPins();
}

/************************** Button **************************/

void button_isr()
{
    noInterrupts();
    button.state = digitalRead(buttonPin);

    if (button.state == button.last_state) {
        button.time = millis();
        return;
    }

    if (!button.state) {
        button.time = millis();
    }

    if (button.state && (millis() - button.time > debounceDelay)) {
        button.time = millis();
        circuitState = !circuitState;
        digitalWrite(DEFAULT_LED, circuitState);
    }

    button.last_state = button.state;
    delay(5);
    interrupts();
}

/************************** Servo **************************/

void servo_motor_control()
{   
    int servo_position;
    if (rx_packet.state == STATE_SERVO_GUI) {
        servo_position = rx_packet.servo_angle;
    } else if (rx_packet.state == STATE_SERVO_SENS) {
        // Read flex sensor
        uint16_t flex_position = analogRead(flexpin);
        tx_packet.flex_sensor = 1023 - flex_position;

        servo_position = map(flex_position, 0, 1023, 90, 0);
        servo_position = constrain(servo_position, 0, 90);
    }

    // Actuate servo and wait
    tx_packet.servo_angle = (uint8_t)servo_position;
    servo_motor.write(servo_position);
}

/************************** DC Motor **************************/

void PID_Setup_Position()
{
    position_PID.SetMode(AUTOMATIC);
    position_PID.SetSampleTime(20);
    position_PID.SetOutputLimits(-255, 255);
}

void PID_Setup_Velocity()
{
    velocity_PID.SetMode(AUTOMATIC);
    velocity_PID.SetSampleTime(20);
    velocity_PID.SetOutputLimits(0, 255);
}

void pwmOut(int out)
{
    if (abs(out) < 40) {
        analogWrite(MOTOR_EN, 0);
        return;
    }
    if (out > 0) {
        Set_Motor_Direction(0, 1);
    } else {
        Set_Motor_Direction(1, 0);
    }

    out = map(abs(out), 0, 255, MIN_RPM, 255);
    analogWrite(MOTOR_EN, out);
}

void Set_Motor_Direction(int A, int B)
{
    //  Update new direction
    digitalWrite(MOTOR_A1, A);
    digitalWrite(MOTOR_A2, B);
}

void Encoder_ISR()
{
    int state = digitalRead(ENC_CH1);
    if (digitalRead(ENC_CH2))
        state ? encoder_count-- : encoder_count++;
    else
        state ? encoder_count++ : encoder_count--;
}

void PID_Loop()
{
    // If state is to control position using Main OR GUI
    if (rx_packet.state == STATE_DC_POS_SENS || rx_packet.state == STATE_DC_POS_GUI) {
        // Get setpoint
        if (rx_packet.state == STATE_DC_POS_SENS) {
            // Modify to fit motor and encoder characteristics, potmeter connected to A15
            int RawValue = analogRead(TMP36);
            float Voltage = map(RawValue, 0, 1023, 0, 5000); // 5000 to get millivots.
            float tempC = (Voltage - 500) * 0.1; // 500 is the offset
            tempC = constrain(tempC, 20, 120);
            tx_packet.temperature = (uint8_t)tempC;

            // Pot
            // setpoint_pos = analogRead(POTENTIOMETER_PIN); // modify to fit motor and encoder characteristics, potmeter connected to A0
            // setpoint_pos = map(setpoint_pos, 0, 1024, -360 * ROT_FACTOR, 360 * ROT_FACTOR);
            // setpoint_pos = map(setpoint_pos, -360 * ROT_FACTOR, 360 * ROT_FACTOR, -378 * ROT_FACTOR, 378 * ROT_FACTOR);

            // Setpoint
            setpoint_pos = map(tempC, 20, 120, -378 * ROT_FACTOR, 378 * ROT_FACTOR);
        }

        if (rx_packet.state == STATE_DC_POS_GUI) {
            // Get setpoint from GUI
            setpoint_pos = (double)rx_packet.motor_angle;
            setpoint_pos = map(setpoint_pos, -360 * ROT_FACTOR, 360 * ROT_FACTOR, -378 * ROT_FACTOR, 378 * ROT_FACTOR);
        }

        // Get input
        input_pos = encoder_count;

        // PID loop
        position_PID.Compute();

        // Set output
        pwmOut(output_pos);
    }

    else if (rx_packet.state == STATE_DC_VEL_SENS || rx_packet.state == STATE_DC_VEL_GUI) {
        // Get setpoint
        if (rx_packet.state == STATE_DC_VEL_SENS) {
            // Read SONAR
            uint16_t ultrasonic_input = analogRead(lv_Sonar);
            tx_packet.ultrasonic_distance = ultrasonic_input;

            // Setpoint
            setpoint_vel = constrain(ultrasonic_input, min_SONAR, max_SONAR);
            setpoint_vel = (double) map(setpoint_vel, min_SONAR, max_SONAR, 0, 125);
        }

        if (rx_packet.state == STATE_DC_VEL_GUI) {
            // Get setpoint from GUI
            setpoint_vel = (double)rx_packet.motor_velocity;
        }

        // Get input
        input_vel = RPM;

        // PID loop
        velocity_PID.Compute();

        // Set output
        pwmOut(output_vel);
    }
}

/*
    (Change in encoder count) * (60 sec/1 min)
RPM = __________________________________________
    (Change in time --> 20ms) * (PPR --> 378/2)
*/
void timer4_callback()
{
    // Make a local copy of the global encoder count
    volatile double Motor_current_count = encoder_count;
    RPM = (double)(((Motor_current_count - Motor_prev_count) * 60) / (0.02 * 378));

    // Store current encoder count for next iteration
    Motor_prev_count = Motor_current_count;

    // Update TX packet
    tx_packet.encoder_count = (int32_t)Motor_current_count;
    tx_packet.encoder_velocity = (float)RPM;
}

/************************** Setup **************************/

void stepper_setup()
{
    // stepper code
    pinMode(step_stp, OUTPUT);
    pinMode(step_dir, OUTPUT);
    pinMode(step_MS1, OUTPUT);
    pinMode(step_MS2, OUTPUT);
    pinMode(step_EN, OUTPUT);
}

void slot_encoder_setup()
{
    // slot encoder code
    pinMode(photo_in, OUTPUT);
    pinMode(photo_out, INPUT);

    //Set step, direction, microstep and enable pins to default states
    resetStepperPins();
}

void misc_setup()
{
    pinMode(DEFAULT_LED, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(buttonPin), button_isr, CHANGE);
}

void sensor_setup()
{
    pinMode(lv_Sonar, INPUT);
    pinMode(TMP36, INPUT);
}

void motor_setup()
{
    pinMode(ENC_CH1, INPUT_PULLUP);
    pinMode(ENC_CH2, INPUT_PULLUP);
    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);
    pinMode(MOTOR_EN, OUTPUT);
    pinMode(POTENTIOMETER_PIN, INPUT);

    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, LOW);
    analogWrite(MOTOR_EN, 0);

    // Initialize timer 5 PWM interrupt for enable
    attachInterrupt(digitalPinToInterrupt(ENC_CH1), Encoder_ISR, CHANGE);
    // Set 31KHz PWM to prevent motor noise
    TCCR5C = TCCR5C & 0b11111000 | 1;

    // Initialize timer 4 interrupt at 20ms
    Timer4.initialize(20000);
    Timer4.attachInterrupt(timer4_callback);

    PID_Setup_Position();
    PID_Setup_Velocity();
}

void setup()
{
    stepper_setup();
    slot_encoder_setup();
    misc_setup();
    motor_setup();
    sensor_setup();
    servo_motor.attach(SERVO_PIN);

    Serial.begin(115200);
    delay(3000);
}

//Main loop
void loop()
{
    // Recieve data from GUI
    recieve_data();

    if (rx_packet.global_switch && circuitState) {
        switch (rx_packet.state) {
        case STATE_RESERVED:
            pwmOut(0);
            break;

        case STATE_DC_POS_GUI:
        case STATE_DC_VEL_GUI:
        case STATE_DC_POS_SENS:
        case STATE_DC_VEL_SENS:
            PID_Loop();
            break;

        case STATE_STEPPER_GUI:
            pwmOut(0);
            StepperPos();
            break;

        case STATE_STEPPER_SENS:
            pwmOut(0);
            StepperMain();
            break;

        case STATE_SERVO_GUI:
        case STATE_SERVO_SENS:
            pwmOut(0);
            servo_motor_control();
            break;

        default:
            break;
        }
    } else {
        pwmOut(0);
    }

    // Send data to GUI
    if (millis() - last_send_time > 50) {
        last_send_time = millis();
        send_data();
    }
}
