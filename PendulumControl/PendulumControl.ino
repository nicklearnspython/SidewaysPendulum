/** 
 * Pendulum Control 
 * author nicklearnspython <nicklearnspython@gmail.com>
 * Nov. 2019
 */

/**
 *
 * New Functions Game Plan
 *      3. Arming throttle (1140 PWM) 
 *      4. Listen to IMU and print
 *      5. Listen to RC controller
 *      6. Listen to both RC cont. and IMU and print
 *      7. P controller
 *      8. PI controller
 *      9. PID controller
 *      10. State Space controller
 * 
 */
 
// ---------------------------------------------------------------------------
#include <Servo.h>
// ---------------------------------------------------------------------------
// Constants
#define MIN_PULSE_LENGTH 1100 // Minimum pulse length in µs
#define ARM_PULSE_LENGTH 1180 // Arming  pulse length in µs
#define MAX_PULSE_LENGTH 1900 // Maximum pulse length in µs

#define ARM_DELAY 3000      // [ms] 3 seconds

// ---------------------------------------------------------------------------
// Variables
Servo motor;
char data;
// ---------------------------------------------------------------------------

/**
 * Initialization routine
 */
void setup() {
    Serial.begin(9600);
    motor.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motor.writeMicroseconds(MIN_PULSE_LENGTH); // Initialize motor to off
    displayInstructions();
}

/**
 * Main function
 */
void loop() {
    if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            // 0
            case 48 : Serial.println("Sending minimum throttle");
                      motor.writeMicroseconds(MIN_PULSE_LENGTH);
            break;

            // 1
            case 49 : Serial.println("Sending maximum throttle");
                      motor.writeMicroseconds(MAX_PULSE_LENGTH);
            break;

            // 2
            case 50 : Serial.print("Running test");
                      countdown();
                      test();
            break;
            
            // 3
            case 51 : Serial.print("Running test number 2");
                      countdown();
                      arm();
                      disarm();
                      
            break;
        }
    }
}


// ---------------------------------------------------------------------------
// Functions

/**
 * Test function: send min throttle to max throttle to each ESC.
 */
void test()
{
    for (int i = MIN_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        motor.writeMicroseconds(i);
        
        delay(20);
    }

    disarm();
}

/**
 * Arms the motor setting the output to a low spinning PWM
 */
void arm()
{
    Serial.println("Arming...");
    motor.writeMicroseconds(ARM_PULSE_LENGTH);
    delay(ARM_DELAY);
}

/**
 * Disarms the motor setting the output to lowest PWM
 */
void disarm()
{
    Serial.println("Disarming.");
    motor.writeMicroseconds(MIN_PULSE_LENGTH);
}

/**
 * Displays instructions to user
 */
void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t1 : Send max throttle");
    Serial.println("\t2 : Run test function\n");
    Serial.println("\t3 : Run arm function\n");
}

/**
 * Displays countdown from 3 seconds
 */
void countdown()
{  
    Serial.print("Running in 3");
    delay(1000);
    Serial.print(" 2");
    delay(1000);
    Serial.println(" 1...");
    delay(1000);
}
