#ifndef MOTOR_MODULE_MAIN_H
#define MOTOR_MODULE_MAIN_H

#include <HardwareSerial.h>
#include <Wire.h>

HardwareSerial SystemUART(1);  // UART1
const int SystemUARTRX = 21; //7;
const int SystemUARTTX = 20; //8;
#define DEVICE_ID "1"

// Pins and PWM channels
#define RPWM_PIN 5
#define LPWM_PIN 7
#define RPWM_CHANNEL 0
#define LPWM_CHANNEL 1



static TaskHandle_t EncoderReading_Task = NULL;
static TaskHandle_t Motor_Task = NULL;

hw_timer_t* timer = NULL;
uint8_t timer_id = 0;
uint16_t prescaler = 80;  // Between 0 and 65 535
int threshold = 50000;    // 64 bits value (limited to int size of 32bits) -- Set to 100ms

const uint8_t AS5600_ADDR = 0x36;
const uint8_t ANGLE_MSB = 0x0E; // MSB register
const uint8_t ANGLE_LSB = 0x0F; // LSB register
long totalTurns = 0;
int16_t lastAngle = -1;


portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;

const float ticks_per_rev = 2000.0;  // ✅ Encoder ticks per motor shaft revolution
const float ticksToRadians = 2 * PI / ticks_per_rev;
int delta_Ticks = 0;
float Current_speed = 0;

double radsSP = 0;  //ROS compliant.

float Kp = 1.1;
float Ki = 0;
float Kd = 0.04;
double motorpwm = 0;

float integral = 0;
float previous_error = 0;
unsigned long last_time_MOTOR = 0;

double LinearDistanceSP = 0;  //ROS compliant.
const float wheel_diameter_mm = 97.0;
const float wheel_circumference_mm = PI * wheel_diameter_mm;
const float ticks_per_mm = ticks_per_rev / wheel_circumference_mm;

double displacementKp = 2.0, displacementKi = 0.5, displacementKd = 0.1;
double displacement_integral = 0, last_displacement_error = 0;
double target_position = 0;  // In encoder ticks, or convert to radians

// forward declarations for ISRs (must exist before timerAttachInterrupt)
void IRAM_ATTR encoderISRTimer();
void IRAM_ATTR MotorTaskTimer();

void TimerTask(void* parameters);

void UART_SEND(String message);
void checkUSBInput();

#include "BTS7960.h"
#include "Device_Networking.h"
#include "Movement_Control.h"
#include "BTS7960.H"

#endif