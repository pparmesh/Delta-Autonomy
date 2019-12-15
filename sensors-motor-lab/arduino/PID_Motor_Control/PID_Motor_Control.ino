#include "TimerFour.h"
#include <PID_v1.h>

// DC Motor
#define encoder_ch1 19 // Quadrature encoder A pin
#define encoder_ch2 20 // Quadrature encoder B pin
#define Motor1_enable 3 // PWM outputs to L298N H-Bridge motor driver module
#define Motor1_A1 51
#define Motor1_A2 53
#define MIN_RPM 55
#define ROT_FACTOR 1

#define SensorPin_DC A15
#define DEFAULT_LED 13

// TODO: Clean
#define PositionGUI 1
#define VelocityGUI 2
#define PositionMain 3
#define VelocityMain 4

// ISR variables
volatile double encoder_count = 0;
volatile double Motor_prev_count = 0;
volatile double RPM;

unsigned long last_time = 0;
long SpeedControl_Temp;

// TODO: Replace with RX packet
int state = PositionMain;

// Position Control Initialization
double kp_pos = 0.8, ki_pos = 0, kd_pos = 0; // modify for optimal performance
double input_pos = 0, output_pos = 0, setpoint_pos = 0;
PID position_PID(&input_pos, &output_pos, &setpoint_pos, kp_pos, ki_pos, kd_pos, DIRECT);

// Velocity Control Initialization
double kp_vel = 4.9, ki_vel = 0, kd_vel = 0.0; // modify for optimal performance
double input_vel = 0, output_vel = 0, setpoint_vel = 0;
PID velocity_PID(&input_vel, &output_vel, &setpoint_vel, kp_vel, ki_vel, kd_vel, DIRECT);

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
{ // to H-Bridge board
    if (out > 0) {
        Set_Motor_Direction(0, 1);
    } else {
        Set_Motor_Direction(1, 0);
    }

    out = map(abs(out), 0, 255, MIN_RPM, 255);
    analogWrite(Motor1_enable, out);
}

void Set_Motor_Direction(int A, int B)
{
    //  Update new direction
    digitalWrite(Motor1_A1, A);
    digitalWrite(Motor1_A2, B);
}

void Encoder_ISR()
{
    //  encoder_count++;
    int state = digitalRead(encoder_ch1);
    if (digitalRead(encoder_ch2))
        state ? encoder_count-- : encoder_count++;
    else
        state ? encoder_count++ : encoder_count--;
}

void PID_Loop()
{
    // If state is to control position using Main OR GUI
    if (state == PositionMain || state == PositionGUI) {
        // Get setpoint
        if (state == PositionMain) {
            setpoint_pos = analogRead(SensorPin_DC); // modify to fit motor and encoder characteristics, potmeter connected to A0
            setpoint_pos = map(setpoint_pos, 0, 1024, -360 * ROT_FACTOR, 360 * ROT_FACTOR);
            setpoint_pos = map(setpoint_pos, -360 * ROT_FACTOR, 360 * ROT_FACTOR, -378 * ROT_FACTOR, 378 * ROT_FACTOR);
        }

        if (state == PositionGUI) {
            // TODO: Replace with RX packets
            // Get setpoint from GUI
            // setpoint_pos = (double) rx_packet.motor_angle;
        }

        // Get input
        input_pos = encoder_count;

        // PID loop
        position_PID.Compute();

        // Set output
        pwmOut(output_pos);
    }

    else if (state == VelocityMain || state == VelocityGUI) {
        // Get setpoint
        if (state == VelocityMain) {
            setpoint_vel = analogRead(SensorPin_DC);
            setpoint_vel = map(setpoint_vel, 0, 1024, 0, 140);
        }

        if (state == VelocityGUI) {
            // Get setpoint from GUI
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
}

void Motor_Setup()
{
    pinMode(encoder_ch1, INPUT_PULLUP); // quadrature encoder input A
    pinMode(encoder_ch2, INPUT_PULLUP);
    pinMode(Motor1_A1, OUTPUT);
    pinMode(Motor1_A2, OUTPUT);
    pinMode(Motor1_enable, OUTPUT);
    pinMode(SensorPin_DC, INPUT);

    digitalWrite(Motor1_A1, LOW);
    digitalWrite(Motor1_A2, LOW);
    analogWrite(Motor1_enable, 0);

    // Initialize timer 5 PWM interrupt for enable
    attachInterrupt(digitalPinToInterrupt(encoder_ch1), Encoder_ISR, CHANGE);
    TCCR5C = TCCR5C & 0b11111000 | 1; // set 31KHz PWM to prevent motor noise

    // Initialize timer 4 interrupt at 20ms
    Timer4.initialize(20000);
    Timer4.attachInterrupt(timer4_callback);

    PID_Setup_Position();
    PID_Setup_Velocity();
}

void setup()
{
    Motor_Setup();

    pinMode(DEFAULT_LED, OUTPUT);
    Serial.begin(115200);

    delay(2000);
    Serial.println("Starting now");
}

void loop()
{
    if (millis() - last_time > 200) {
        last_time = millis();
        Serial.print(encoder_count);
        // Serial.print(RPM);
        Serial.print('\t');
        // Serial.print(output_vel);
        Serial.print(output_pos);
        Serial.print('\t');
        // Serial.print(abs(setpoint_vel - output_vel));
        Serial.print(abs(setpoint_pos - output_pos));
        Serial.print('\t');
        // Serial.println(setpoint_vel);
        Serial.println(setpoint_pos);
    }
    PID_Loop();
}
