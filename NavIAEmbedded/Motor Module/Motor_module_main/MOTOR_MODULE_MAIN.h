#ifndef MOTOR_MODULE_MAIN_H
#define MOTOR_MODULE_MAIN_H

#include <HardwareSerial.h>
#include <Wire.h>
#include "BTS7960.h"

HardwareSerial SystemUART(1);  // UART1
const int SystemUARTRX = 20;   //7;
const int SystemUARTTX = 21;   //8;

// Pins and PWM channels
#define RPWM_PIN 5
#define LPWM_PIN 7
#define RPWM_CHANNEL 0
#define LPWM_CHANNEL 1
BTS7960Motor motor(RPWM_PIN, LPWM_PIN, RPWM_CHANNEL, LPWM_CHANNEL);

#define SNS_SDA  8       // Your AS5600 pins
#define SNS_SCL  9


static TaskHandle_t EncoderReading_Task = NULL;
static TaskHandle_t Motor_Task = NULL;

hw_timer_t* timer = NULL;
uint8_t timer_id = 0;
uint16_t prescaler = 80;  // Between 0 and 65 535
int threshold = 50000;    // 64 bits value (limited to int size of 32bits) -- Set to 100ms

const uint8_t AS5600_ADDR = 0x36;
const uint8_t ANGLE_MSB = 0x0E;  // MSB register
const uint8_t ANGLE_LSB = 0x0F;  // LSB register
long totalTurns = 0;
int16_t lastAngle = -1;


portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;

const float ticks_per_rev = 2000.0;  // ✅ Encoder ticks per motor shaft revolution
const float ticksToRadians = 2 * PI / ticks_per_rev;
int delta_Ticks = 0;
float Current_speed = 0;

double radsSP = 0;  //ROS compliant.

// ===================================================================
// ==          COMPILE-TIME DEVICE CONFIGURATION                  ==
// ===================================================================
// ===================================================================
// ==          COMPILE-TIME DEVICE CONFIGURATION                  ==
// ===================================================================

// << CHANGE THIS VALUE FOR EACH DEVICE AND RECOMPILE/UPLOAD >>
#define DEBUG_PRINT_INTERVAL 20 //-1 for no print out mode
#define DEVICE_ID 4
/*
    m1: float  # front-left
    m2: float  # front-right
    m3: float  # rear-left
    m4: float  # rear-right
*/
// --- Define INITIAL values based on DEVICE_ID ---
#if DEVICE_ID == 1
#define INITIAL_KP_POS 1.6
#define INITIAL_KI_POS 0.0
#define INITIAL_KD_POS 0.0
#define INITIAL_KP_SPD 1.8
#define INITIAL_KI_SPD 3.0
#define INITIAL_KD_SPD 0.002
#define INITIAL_UPDATE_DELAY 0

#elif DEVICE_ID == 2
#define INITIAL_KP_POS 1.6
#define INITIAL_KI_POS 0.0
#define INITIAL_KD_POS 0.0
#define INITIAL_KP_SPD 1.8
#define INITIAL_KI_SPD 3.0
#define INITIAL_KD_SPD 0.002
#define INITIAL_UPDATE_DELAY 3

#elif DEVICE_ID == 3
#define INITIAL_KP_POS 1.6
#define INITIAL_KI_POS 0.0
#define INITIAL_KD_POS 0.0
#define INITIAL_KP_SPD 1.8
#define INITIAL_KI_SPD 2.5
#define INITIAL_KD_SPD 0.001
#define INITIAL_UPDATE_DELAY 7

#elif DEVICE_ID == 4
#define INITIAL_KP_POS 1.6
#define INITIAL_KI_POS 0.0
#define INITIAL_KD_POS 0.0
#define INITIAL_KP_SPD 1.6
#define INITIAL_KI_SPD 2.5
#define INITIAL_KD_SPD 0.0005
#define INITIAL_UPDATE_DELAY 11

#else
#error "Invalid DEVICE_ID specified. Must be 1, 2, 3, or 4."
#endif

// --- Now, create the actual variables using the initial values ---
// These are NOT const, so they can be changed by your code at runtime.
float Kp_pos = INITIAL_KP_POS;
float Ki_pos = INITIAL_KI_POS;
float Kd_pos = INITIAL_KD_POS;
float Kp_spd = INITIAL_KP_SPD;
float Ki_spd = INITIAL_KI_SPD;
float Kd_spd = INITIAL_KD_SPD;
int Updatedelay = INITIAL_UPDATE_DELAY;

// ===================================================================

// ===================================================================


// ===== POSITION PID STATE (outer loop) =====
static double prev_error_pos = 0;
static double integral_pos = 0;
static double derivative_pos = 0;
double currentPositionRad = 0.0f;
double error_pos = 0;

// ===== SPEED PID STATE (inner loop) =====
static double prev_error_spd = 0;
static double integral_spd = 0;
static double derivative_spd = 0;

double motorpwm = 0;

double integral = 0;
double previous_error = 0;
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


const float GEAR_RATIO = 71.0;
bool debug_on = 0;
float targetDistance = 0;        // mm or degrees, outer ref
float targetSpeed = 0;           // deg/s, set by outer PID
float lastAbsMotorAngleDeg = 0;  // for speed calc

bool plotterMode = false;

typedef struct {
  int16_t raw_angle;  // 0..4095
  long turns;
} encoder_sample_t;

// ===== GLOBAL VARIABLES FOR POSITION CONTROL =====
double targetPosition = 0.0f;                                    // Target position in radians
bool positionControlMode = false;                               // false = speed control, true = position control
double positionKp = 2.0f, positionKi = 0.0f, positionKd = 0.0f;  // Position PID gains (if used)

// Fuzzy controller variables
double prev_position_error = 0.0f;
unsigned long last_position_time = 0;

// timings
#define ENCODER_US 200UL  // encoder sample period (microseconds)
#define MOTOR_US 5000UL   // motor task period (microseconds) -> 200Hz (5ms)
// FreeRTOS handles
QueueHandle_t sampleQueue = NULL;
// Queue size for buffered readings
#define SAMPLE_QUEUE_LEN 256
hw_timer_t *encoderTimer = NULL;
hw_timer_t *motorTimer = NULL;

int16_t readRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(ANGLE_MSB);
  if (Wire.endTransmission(false) != 0) return -1;  // comms error
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return -1;
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  int16_t raw = ((msb << 8) | lsb) & 0x0FFF;  // 12-bit value
  return raw;
}


#include "Device_Networking.h"
#include "Movement_Control.h"
#include "BTS7960.H"

#endif