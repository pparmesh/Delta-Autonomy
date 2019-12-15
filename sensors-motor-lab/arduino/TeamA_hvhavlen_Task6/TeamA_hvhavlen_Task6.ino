/**
    16-650 Systems Engineering
    Task 6: Microcontroller Familiarization

    @author Heethesh Vhavle
    @version 1.1 
    @date 11-NOV-2018

    ------------ STATE MACHINE DESCRIPTION ------------

    State 1: The colour of the LED cycles as follows:
             Red-Yellow-Green-Cyan-Blue-Magenta

    State 2: The hue angle of the LED changes

    State 3: Please send commands from the terminal.
             Valid commands: "r255" "g0" "b127"
             Invalid command cases:
             1. Value greater out of range: "r500" --> Limits within [0, 255]
             2. Value not a number: "rxy" --> Resets LED value to 0
             3. Invalid channel: "y100" --> Ignored
*/

/************************** MACROS **************************/

#define DEFAULT_LED 13
#define BTN0 2
#define BTN1 3
#define LEDR 9
#define LEDG 10
#define LEDB 11
#define POT A0

#define DEBOUNCE_DELAY 50

/************************** GLOBALS **************************/

// Colors
enum class Color { red, yellow, green, cyan, blue, magenta, on, off };

// State machine
struct State
{
    int current = 0;
    int color = 0;
    int rgb[3] = { 0, 0, 0 };
};

volatile State state;
volatile bool STATE_UPDATED = false;

// Button states
struct Button
{
    int state;
    int last_state = LOW;
    int time = millis();
}; 

volatile Button button0;
volatile Button button1;

/************************** RGB LED **************************/

void hue2rgb(float hue, int* rgb)
{
    hue = hue / 60.0;
    float hue_flr = floor(hue);
    int hue_mod = (int) hue_flr % 6;
    float diff = hue - hue_flr;
    int a = (int) ((1 - diff) * 255.0);
    int b = (int) (diff * 255.0);

    if      (hue_mod == 0) { rgb[0] = 255; rgb[1] = b; rgb[2] = 0; }
    else if (hue_mod == 1) { rgb[0] = a; rgb[1] = 255; rgb[2] = 0; }
    else if (hue_mod == 2) { rgb[0] = 0; rgb[1] = 255; rgb[2] = b; }
    else if (hue_mod == 3) { rgb[0] = 0; rgb[1] = a; rgb[2] = 255; }
    else if (hue_mod == 4) { rgb[0] = b; rgb[1] = 0; rgb[2] = 255; }
    else if (hue_mod == 5) { rgb[0] = 255; rgb[1] = 0; rgb[2] = a; }
}

void set_rgb_digital(int r, int g, int b)
{
    digitalWrite(LEDR, !r);
    digitalWrite(LEDG, !g);
    digitalWrite(LEDB, !b);
}

void set_rgb_analog(int r, int g, int b)
{
    analogWrite(LEDR, 255 - r);
    analogWrite(LEDG, 255 - g);
    analogWrite(LEDB, 255 - b);
}

void set_rgb_color(Color color)
{
    switch (color)
    {
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

void rgb_init()
{
    // Set pin mode
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);

    // Set default state
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
}

/************************** BUTTONS **************************/

void button0_isr()
{
    // Debounce handling
    noInterrupts();
    button0.state = digitalRead(BTN0);
    if (button0.state == button0.last_state) { button0.time = millis(); return; }
    if (!button0.state) button0.time = millis();
    if (button0.state && (millis()-button0.time > DEBOUNCE_DELAY))
    {
        button0.time = millis();
        
        // Update state
        STATE_UPDATED = true;
    }

    button0.last_state = button0.state;
    delay(5);
    interrupts();
}

void button1_isr()
{
    // Debounce handling
    noInterrupts();
    button1.state = digitalRead(BTN1);
    if (button1.state == button1.last_state) { button1.time = millis(); return; }
    if (!button1.state) button1.time = millis();
    if (button1.state && (millis()-button1.time > DEBOUNCE_DELAY))
    {
        button1.time = millis();

        // Update mode
        if (state.current == 0)
        {
            state.color++;
            if (state.color > 5) state.color = 0;

            // Display state
            Serial.print("Color: "); Serial.println(state.color);
        }
    }

    button1.last_state = button1.state;
    delay(5);
    interrupts();
}

void button_init()
{
    // Set pin mode
    pinMode(BTN0, INPUT_PULLUP);
    pinMode(BTN1, INPUT_PULLUP);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(BTN0), button0_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BTN1), button1_isr, CHANGE);
}

/************************** STATE MACHINE **************************/

void state0_handler()
{
    set_rgb_color((Color) state.color);
}

void state1_handler(bool debug=false)
{
    int rgb[3] = { 0, 0, 0 };
    int hue = analogRead(POT);
    
    // Convert analog value to RGB
    hue = map(hue, 0, 1023, 0, 360);
    hue2rgb((float) hue, rgb);

    // Set PWM values for RGB
    set_rgb_analog(rgb[0], rgb[1], rgb[2]);
    
    if (debug)
    {
        Serial.print("Hue Angle: "); Serial.print(hue); Serial.print('\t');
        Serial.print("RGB: "); Serial.print(rgb[0]); Serial.print('\t');
        Serial.print(rgb[1]); Serial.print('\t'); Serial.println(rgb[2]);
        delay(50);
    }
}

void state2_handler()
{
    // Load previous state
    set_rgb_analog(state.rgb[0], state.rgb[1], state.rgb[2]);

    // Wait for new command
    Serial.println("Enter New Command >>>");
    while (Serial.available() == 0)
    {
        if (state.current != 2 || STATE_UPDATED) return;
    }
    delay(10); // Wait for entire string to be received
    
    // Read string
    String command = Serial.readString();
    Serial.flush();
    command.trim();
    // Serial.println(command);
    
    String val = command.substring(1, command.length());

    if (command.startsWith("r")) state.rgb[0] = constrain(val.toInt(), 0, 255);
    if (command.startsWith("g")) state.rgb[1] = constrain(val.toInt(), 0, 255);
    if (command.startsWith("b")) state.rgb[2] = constrain(val.toInt(), 0, 255);
    
    // Debug
    Serial.print("RGB: "); Serial.print(state.rgb[0]); Serial.print('\t');
    Serial.print(state.rgb[1]); Serial.print('\t'); Serial.println(state.rgb[2]);

    set_rgb_analog(state.rgb[0], state.rgb[1], state.rgb[2]);
}

void state_update()
{
    // Reset flag
    STATE_UPDATED = false;

    // Update state
    state.current++;
    if (state.current > 2) state.current = 0;

    // Reset LED
    set_rgb_color(Color::off);

    // Display state
    Serial.println(""); Serial.print("**** STATE "); 
    Serial.print(state.current); Serial.println(" ****");

    // Display state
    if (state.current == 0)
    {
        Serial.print("Color: "); Serial.println(state.color);
    }
    if (state.current == 1)
    {
        state1_handler(true);
    }
    if (state.current == 2)
    {
        Serial.print("RGB: "); Serial.print(state.rgb[0]); Serial.print('\t');
        Serial.print(state.rgb[1]); Serial.print('\t'); Serial.println(state.rgb[2]);
    }
}

void state_handler()
{
    if (state.current == 0) state0_handler();
    if (state.current == 1) state1_handler();
    if (state.current == 2) state2_handler();
}

/************************** MAIN **************************/

void setup()
{
    rgb_init();
    button_init();
    pinMode(DEFAULT_LED, OUTPUT);
    Serial.begin(9600);

    // Display state
    Serial.println(""); Serial.print("**** STATE "); 
    Serial.print(state.current); Serial.println(" ****");
}

void loop()
{
    if (STATE_UPDATED) state_update();
    state_handler();
}
