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
 
 
 *      7. P controller
 *      8. PI controller
 *      9. PID controller
 *      10. State Space controller
 * 
 */
 
// ---------------------------------------------------------------------------
#include <Servo.h>
#include <Wire.h>
// ---------------------------------------------------------------------------
// Constants
#define MIN_PULSE_LENGTH 1100 // Minimum pulse length in µs
#define ARM_PULSE_LENGTH 1180 // Arming  pulse length in µs
#define HOV_PULSE_LENGTH 1320 // Hover   pulse length in µs 1341
#define MAX_PULSE_LENGTH 1900 // Maximum pulse length in µs

#define ARM_DELAY 3000      // [ms] 3 seconds

#define MPU9250_ADDRESS           0x68
#define MAG_ADDRESS               0x0C

#define GYRO_FULL_SCALE_250_DPS   0x00  
#define GYRO_FULL_SCALE_500_DPS   0x08
#define GYRO_FULL_SCALE_1000_DPS  0x10
#define GYRO_FULL_SCALE_2000_DPS  0x18

#define ACC_FULL_SCALE_2_G        0x00  
#define ACC_FULL_SCALE_4_G        0x08
#define ACC_FULL_SCALE_8_G        0x10
#define ACC_FULL_SCALE_16_G       0x18

#define mx     0.00119597
#define my     0.00119597
#define mz     0.00119269
#define bx     0.08292081
#define by     0.30098593
#define bz    -0.02623918

#define pi     3.1415926
#define alpha  0.8
#define beta   0.2
#define dt     0.01

#define Kp        1
#define Ki        0.01
#define Kd        0.1
#define ANGLE_REF 0

// ---------------------------------------------------------------------------
// Variables
Servo motor;
char data;

float angle;
float angle_accel;

float ax;
float az;
float gy;

int input_pulse_length;
int integrator = 0;
int error;
int prev_error = 0;
int control_effort;
int angle_ref;
// ---------------------------------------------------------------------------

/**
 * Initialization routine
 */
void setup() {
    Wire.begin();
    Serial.begin(115200);
    motor.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motor.writeMicroseconds(MIN_PULSE_LENGTH); // Initialize motor to off
    initializeIMU();
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
            case 51 : Serial.print("Running arming test");
                      countdown();
                      arm();
                      disarm();
            break;

            // 4
            case 52 : Serial.print("Printing accelerometer data");
                      for (int i = 0; i < 1200; i++){
                        readAndCalculate();
                        Serial.println(angle);
                        delay(20);
                      }
            break;

            // 5
            
            // 6
            case 54 : Serial.println("P-Controller");
                      Serial.println("Stand back ladies and gentlemen!");
                      countdown();
                      arm();
                      for (int i = 0; i < 1200; i++){
                        readAndCalculate();
                        input_pulse_length = p_controller();
                        motor.writeMicroseconds(input_pulse_length);
                        
                        Serial.print("angle: ");
                        Serial.print(angle);
                        Serial.print(" | input: ");
                        Serial.println(input_pulse_length);
                        delay(20);
                      }
                      disarm();
            break; 

            // 7
            case 55: Serial.println("PI-Controller");
                     Serial.println("Stand back ladies and gentlemen!");
                     countdown();
                     arm();
                     integrator = 0;
                     control_effort = 0;
                     
                     for (int i = 0; i < 1200; i++){
                        readAndCalculate();
                        input_pulse_length = pi_controller();
                        motor.writeMicroseconds(input_pulse_length);
                        
                        Serial.print("angle: ");
                        Serial.print(angle);
                        Serial.print(" | control: ");
                        Serial.println(control_effort);
                        delay(20);
                      }
                      disarm();
            break;

            // 8
            case 56: Serial.println("PID-Controller");
                     Serial.println("Stand back ladies and gentlemen!");
                     countdown();
                     arm();
                     integrator = 0;
                     control_effort = 0;
                     prev_error = 0;
                     
                     for (int i = 0; i < 1200; i++){
                        readAndCalculate();
                        input_pulse_length = pid_controller();
                        motor.writeMicroseconds(input_pulse_length);
                        
                        Serial.print("angle: ");
                        Serial.print(angle);
                        Serial.print(" | control: ");
                        Serial.println(control_effort);
                        delay(20);
                      }
                      disarm();
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// Functions

/* 
 *  P Controller
 */
int p_controller()
{
  return (ANGLE_REF - angle) * Kp + HOV_PULSE_LENGTH;
}


/*
 * PI Controller
 */
int pi_controller()
{
  error = (ANGLE_REF - angle);
  integrator += error;
  control_effort = Kp * error + Ki * integrator;
  return control_effort + HOV_PULSE_LENGTH;
}

/*
 * PID Controller
 */
int pid_controller()
{
  error = (ANGLE_REF - angle);
  integrator += error;
  control_effort = Kp * error + Ki * integrator - Kd * (error - prev_error)/2;
  prev_error = error;
  return control_effort + HOV_PULSE_LENGTH;
}


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
    countdown(); //delay(ARM_DELAY);
}


/**
 * Disarms the motor setting the output to lowest PWM
 */
void disarm()
{
    Serial.println("Disarming.");
    motor.writeMicroseconds(MIN_PULSE_LENGTH);
}


/*
 * This function read Nbytes bytes from I2C device at address Address. 
 * Put read bytes starting at register Register in the Data array. 
 */
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


/*
 * Write a byte (Data) in device (Address) at register (Register)
 */
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}


/*
 * Calculates output based on prior calibration
 */
float calibration(float input, float m, float b)
{
  // Using known calibration values calibrate sensors
  return (m*input)+b;
}


/*
 * Initialize IMU
 */
void initializeIMU()
{
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
 
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
}

/*
 * Read relavent IMU data and calibrate the accelerometer values
 * Calculate the estimated angles
 */
void readAndCalculate()
{
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  // Accelerometer and Gyroscope
  ax=-(Buf[0]<<8 | Buf[1]);
  az=Buf[4]<<8 | Buf[5];
  gy=(Buf[10]<<8 | Buf[11]);

  // Calibrate
  ax = calibration(ax, mx, bx);
  az = calibration(az, mz, bz);
  gy -= 25;

  // Calculate accel estimated angle
  angle_accel = atan2(ax, az) * 180 / pi;
  // Calculate Complementary Filter
  angle = beta * (angle + gy * dt) + alpha * angle_accel;
}


/**
 * Displays instructions to user
 */
void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t1 : Send max throttle");
    Serial.println("\t2 : Run test function");
    Serial.println("\t3 : Run arm function");
    Serial.println("\t4 : Print IMU Data");
    Serial.println("\t5 : "); // Print RC Data");
    Serial.println("\t6 : P Controller");
    Serial.println("\t7 : PI Controller");
    Serial.println("\t8 : PID Controller");
    Serial.println("\t9 : ");
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
