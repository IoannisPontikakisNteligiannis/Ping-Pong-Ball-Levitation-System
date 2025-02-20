#include <LiquidCrystal.h>  // Include the LiquidCrystal library for LCD display

// Pin definitions for the system hardware components
struct Pins {
    static const int LCD_RS = 12, LCD_EN = 11, LCD_D4 = 5, LCD_D3 = 4, LCD_D2 = 3, LCD_D1 = 2;  // LCD pin connections
    static const int BUTTON = 7;   // Push button pin
    static const int BUZZER = 13;  // Buzzer pin
    static const int POT = A0;     // Potentiometer pin (analog input)
    static const int MOTOR = 9;    // Motor pin (PWM output)
    static const int TRIG = A1;    // Ultrasonic sensor trigger pin
    static const int ECHO = A2;    // Ultrasonic sensor echo pin
};

// PID (Proportional, Integral, Derivative) control configuration
struct PIDConfig {
    static constexpr float KP = 4;  // Proportional constant
    static constexpr float KI = 1.5;  // Integral constant
    static constexpr float KD =1;  // Derivative constant
    static constexpr float SAMPLE_TIME = 0.05;  // Time interval for PID calculation
    static constexpr float MAX_INTEGRAL = 255.0;  // Maximum allowable integral value
    static constexpr int OUTPUT_MIN = 50;  // Minimum output value
    static constexpr int OUTPUT_MAX = 255;  // Maximum output value
};

// Timing constants for debounce, display updates, and control intervals
struct Timing {
    static const unsigned long DEBOUNCE_DELAY = 50;  // Button debounce delay (milliseconds)
    static const unsigned long DISPLAY_UPDATE = 320;  // LCD display update interval (milliseconds)
    static const unsigned long CONTROL_INTERVAL = 1;  // Control update interval (milliseconds)
    static const unsigned long SERIAL_UPDATE = 50;  // Serial update interval (milliseconds)
};

// Enumeration for different system modes
enum class Mode { INIT, SET_POSITION, CONTROL };  // Mode for the system (INIT, SET_POSITION, CONTROL)

/***** class PositionController
* Purpose
*   Controls and maintains the position of a motor-driven system using PID control
*   and ultrasonic distance sensing. Provides user interface through LCD display
*   and button inputs for mode selection and target position setting.
*
* Specifiers
*   - LCD display interface for user feedback and system status
*   - PID control parameters (KP, KI, KD) for position regulation
*   - Timing constants for debounce, display updates, and control intervals
*   - Mode selection (INIT, SET_POSITION, CONTROL) for system operation states
*
* Methods
*   - begin() initializes system hardware and displays welcome message
*   - update() handles main control loop, mode switching, and display updates
*
* Hardware
*   - LCD Display (16x2)
*   - Ultrasonic sensor (HC-SR04)
*   - DC motor with PWM control
*   - Push button for mode selection
*   - Potentiometer for setpoint input
*   - Buzzer for audio feedback
*
* Software
*   - Uses Arduino LiquidCrystal library
*   - Implements PID control algorithm
*   - Includes debounce handling for button input
*   - Features serial communication for debugging
********/

class PositionController {
private:
    LiquidCrystal lcd;  // LCD object
    Mode currentMode;  // Current mode of the system
    float setpoint, lastError, integral, currentError, motorOutput;  // PID control variables
    bool firstTimeInMode, buttonPressed, buttonWasPressed;  // Control flags for button and mode transitions
    unsigned long lastButtonTime, lastDisplayTime, lastControlTime, lastSerialTime;  // Timing variables for various updates

    // Update button state with debounce handling
    void updateButton(unsigned long currentTime) {
        bool currentButtonState = (digitalRead(Pins::BUTTON) == LOW);  // Check if button is pressed (LOW means pressed)
        if (currentTime - lastButtonTime >= Timing::DEBOUNCE_DELAY) {  // Check if enough time has passed for debounce
            if (currentButtonState && !buttonPressed && !buttonWasPressed) {
                buttonPressed = true;
                buttonWasPressed = true;
                lastButtonTime = currentTime;
                switchMode();  // Switch to the next mode
            } else if (!currentButtonState && buttonWasPressed) {
                buttonPressed = false;
                buttonWasPressed = false;
                lastButtonTime = currentTime;
            }
        }
    }

    // PID control function to calculate motor output based on the current position
    float calculatePID(float currentPosition) {
        float error = currentPosition - setpoint;  // Error between setpoint and current position
        currentError = error;
        float proportional = PIDConfig::KP * error;  // Proportional term
        integral += PIDConfig::KI * error * PIDConfig::SAMPLE_TIME;  // Integral term
        integral = constrain(integral, -PIDConfig::MAX_INTEGRAL, PIDConfig::MAX_INTEGRAL);  // Limit integral to prevent wind-up
        float derivative = PIDConfig::KD * (error - lastError) / PIDConfig::SAMPLE_TIME;  // Derivative term
        lastError = error;  // Update last error for the next calculation
        float output = proportional + integral + derivative;  // Calculate the total motor output
        motorOutput = constrain(output, PIDConfig::OUTPUT_MIN, PIDConfig::OUTPUT_MAX);  // Ensure output is within allowable range
        return motorOutput;
    }

    // Read the position from the ultrasonic sensor
    float readPosition() {
        digitalWrite(Pins::TRIG, LOW);  // Set trigger low
        delayMicroseconds(2);  // Wait for 2 microseconds
        digitalWrite(Pins::TRIG, HIGH);  // Send trigger pulse
        delayMicroseconds(10);  // Wait for 10 microseconds
        digitalWrite(Pins::TRIG, LOW);  // Set trigger low again
        long duration = pulseIn(Pins::ECHO, HIGH, 30000);  // Measure echo pulse duration
        if (duration == 0 || duration > 30000) return -1;  // If no valid duration, return -1
        return constrain(duration * 0.0343 / 2, 2, 400);  // Convert duration to distance in cm
    }

    // Read potentiometer value and map it to a desired range (0-55cm)
    float readPotentiometer() {
        long sum = 0;
        for (int i = 0; i < 5; i++) {
            sum += analogRead(Pins::POT);  // Read analog value from potentiometer
            delay(1);  // Short delay to avoid noise
        }
        return map(sum / 5, 0, 1023, 0, 56);  // Average the readings and map to range 0-55 cm
    }

    // Send serial data for debugging (position, setpoint, error, and motor output)
    void sendSerialData(float currentPosition) {
        Serial.print(currentPosition);
        Serial.print(",");
        Serial.print(setpoint);
        Serial.print(",");
        Serial.print(currentError);
        Serial.print(",");
        Serial.println(motorOutput);
    }

    // Switch between different system modes (INIT -> SET_POSITION -> CONTROL ->SET_POSITION)
    void switchMode() {
        firstTimeInMode = true;
        switch (currentMode) {
            case Mode::INIT: currentMode = Mode::SET_POSITION; shortBeep(); break;  // Switch to SET_POSITION mode
            case Mode::SET_POSITION:
                currentMode = Mode::CONTROL;
                setpoint = readPotentiometer();  // Set the setpoint from potentiometer
                resetPIDVariables();  // Reset PID variables
                longBeep();  // Long beep to signal transition
                break;
            case Mode::CONTROL: currentMode = Mode::SET_POSITION; analogWrite(Pins::MOTOR, 0); doubleBeep(); break;  // Stop motor and switch back to SET_POSITION mode
        }
        lcd.clear();  // Clear the LCD after switching modes
    }

    // Reset all PID variables to their initial state
    void resetPIDVariables() {
        lastError = 0;
        integral = 0;
        currentError = 0;
        motorOutput = 0;
    }

    // Show the initial welcome message on the LCD
    void showWelcomeMessage() {
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print("SYSTEM ON!");  // Display system status
        lcd.setCursor(1, 1);
        lcd.print("WAITING START");  // Display message prompting to start
    }

    // Play a short beep using the buzzer
    void shortBeep() { tone(Pins::BUZZER, 2000, 100); }
    // Play a long beep using the buzzer
    void longBeep() { tone(Pins::BUZZER, 1500, 1000); }
    // Play a double beep using the buzzer
    void doubleBeep() {
        tone(Pins::BUZZER, 2000, 100);
        delay(300);
        tone(Pins::BUZZER, 2000, 100);
    }

public:
    // Constructor to initialize variables
    PositionController()
        : lcd(Pins::LCD_RS, Pins::LCD_EN, Pins::LCD_D4, Pins::LCD_D3, Pins::LCD_D2, Pins::LCD_D1),
          currentMode(Mode::INIT), setpoint(0), lastError(0), integral(0), currentError(0), motorOutput(0),
          firstTimeInMode(true), buttonPressed(false), buttonWasPressed(false) {
    }

    // Initialize the system (pins, LCD, and serial communication)
    void begin() {
        pinMode(Pins::TRIG, OUTPUT);  // Set the trigger pin as output
        pinMode(Pins::ECHO, INPUT);   // Set the echo pin as input
        pinMode(Pins::BUTTON, INPUT_PULLUP);  // Enable internal pull-up for the button
        pinMode(Pins::BUZZER, OUTPUT);  // Set the buzzer pin as output
        pinMode(Pins::MOTOR, OUTPUT);  // Set the motor pin as output
        lcd.begin(16, 2);  // Initialize the LCD with 16x2 dimensions
        Serial.begin(9600);  // Start serial communication for debugging
        showWelcomeMessage();  // Display the welcome message on the LCD
    }

    // Main update loop to handle the system logic
    void update() {
        unsigned long currentTime = millis();  // Get the current time in milliseconds
        updateButton(currentTime);  // Check for button presses and handle mode switching

        switch (currentMode) {
            case Mode::INIT:
                if (currentTime - lastDisplayTime >= Timing::DISPLAY_UPDATE) {
                    lcd.setCursor(1, 0);
                    lcd.print("SYSTEM ON!");  // Display system status
                    lcd.setCursor(1, 1);
                    lcd.print("WAITING START");  // Prompt user to start
                    lcd.scrollDisplayLeft();  // Scroll display to indicate waiting state
                    lastDisplayTime = currentTime;
                }
                break;

            case Mode::SET_POSITION:
                if (firstTimeInMode) {
                    lcd.clear();
                    lcd.print("SET POSITION:");  // Prompt for setting the position
                    firstTimeInMode = false;
                }
                if (currentTime - lastDisplayTime >= Timing::DISPLAY_UPDATE) {
                    lcd.setCursor(0, 1);
                    lcd.print("Target: ");
                    lcd.print(readPotentiometer(), 1);  // Display target position from potentiometer
                    lcd.print("cm    ");
                    lastDisplayTime = currentTime;
                }
                break;

            case Mode::CONTROL:
                if (firstTimeInMode) {
                    lcd.clear();
                    lcd.print("CONTROL MODE");  // Display control mode
                    firstTimeInMode = false;
                }
                if (currentTime - lastControlTime >= Timing::CONTROL_INTERVAL) {
                    float currentPosition = readPosition();  // Get current position
                    if (currentPosition > 0) {
                        analogWrite(Pins::MOTOR, calculatePID(currentPosition));  // Adjust motor speed using PID
                    }
                    lastControlTime = currentTime;

                    if (currentTime - lastDisplayTime >= Timing::DISPLAY_UPDATE) {
                        lcd.setCursor(0, 1);
                        lcd.print("Pos:");
                        lcd.print(currentPosition, 1);  // Display current position
                        lcd.print(" Tgt:");
                        lcd.print(setpoint, 1);  // Display target position
                        lcd.print("cm    ");
                        lastDisplayTime = currentTime;
                    }

                    if (currentTime - lastSerialTime >= Timing::SERIAL_UPDATE) {
                        sendSerialData(currentPosition);  // Send data to serial monitor
                        lastSerialTime = currentTime;
                    }
                }
                break;
        }
    }
};


// Create a PositionController object to manage the system
PositionController controller;

void setup() {
    controller.begin();  // Initialize the system
}

void loop() {
    controller.update();  // Continuously update the system state
}
