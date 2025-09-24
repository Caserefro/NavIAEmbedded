#include "MOTOR_MODULE_MAIN.h"

BTS7960Motor motor(RPWM_PIN, LPWM_PIN, RPWM_CHANNEL, LPWM_CHANNEL);

typedef struct {
  float kp, ki, kd;
  float prevError;
  float integral;
} PIDController;  // Changed from pid_t to PIDController

PIDController speedPID = { 0.5, 0.1, 0.01, 0, 0 };  // Use standard initializer syntax
PIDController distPID = { 0.3, 0.05, 0.0, 0, 0 };   // Use standard initializer syntax


float targetDistance = 0;   // mm or degrees, outer ref
float targetSpeed = 0;      // deg/s, set by outer PID
float lastAbsAngleDeg = 0;  // for speed calc

typedef struct {
  int16_t raw_angle;  // 0..4095
  long turns;
} encoder_sample_t;

// timings
#define ENCODER_US 500UL  // encoder sample period (microseconds)
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

void setup() {
  Serial.begin(115200);  // USB Serial
  SystemUART.begin(115200, SERIAL_8N1, SystemUARTRX, SystemUARTTX);
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  motor.begin();
  //motorSetup();
  Serial.println("I CAN PRINT ran succesfull --------------------------");

  // Initialize I²C on custom pins (SDA=8, SCL=9) for as5600 module
  Wire.begin(8, 9);
  delay(50);
  lastAngle = readRawAngle();
  if (lastAngle < 0) {
    Serial.println("AS5600 not found!");
    while (1) delay(1000);
  }
  Serial.println("AS5600 detected");

  xTaskCreate(EncoderReadingTask,
              "Timer Task",
              4096,
              NULL,
              1,
              &EncoderReading_Task);

  xTaskCreate(MotorTask,
              "Motor Task",
              4096,
              NULL,
              1,
              &Motor_Task);

  // create queue
  sampleQueue = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(encoder_sample_t));
  if (!sampleQueue) {
    Serial.println("Queue creation failed!");
    while (1) delay(1000);
  }

  // encoder hw-timer -> notify encoder task
  encoderTimer = timerBegin(1, 80, true);  // timer 1, prescaler 80 -> 1 tick = 1 us
  timerAttachInterrupt(encoderTimer, &encoderISRTimer, true);
  timerAlarmWrite(encoderTimer, ENCODER_US, true);
  timerAlarmEnable(encoderTimer);

  // motor hw-timer -> notify motor task
  motorTimer = timerBegin(0, 80, true);  // timer 0
  timerAttachInterrupt(motorTimer, &MotorTaskTimer, true);
  timerAlarmWrite(motorTimer, MOTOR_US, true);
  timerAlarmEnable(motorTimer);

  if (lastAbsAngleDeg < 0) {
    encoder_sample_t s;
    if (xQueueReceive(sampleQueue, &s, pdMS_TO_TICKS(100)) == pdTRUE) {
      lastAbsAngleDeg = (s.turns * 4096 + s.raw_angle) * 360.0 / 4096.0;
    }
  }


  Serial.println("Timers and tasks started");
}
void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // read until newline
    command.trim();                                 // remove whitespace
    command.toLowerCase();

    // --- SPEED TEST: "speed <value> [kp <val> ki <val> kd <val>]" ---
    if (command.startsWith("speed")) {
      float tempSP = 0, tempKp = speedPID.kp, tempKi = speedPID.ki, tempKd = speedPID.kd;
      char *token = strtok((char *)command.c_str(), " ");
      while (token != nullptr) {
        if (strcmp(token, "speed") == 0) {
          token = strtok(nullptr, " ");
          if (token) tempSP = atof(token);
        } else if (strcmp(token, "kp") == 0) {
          token = strtok(nullptr, " ");
          if (token) tempKp = atof(token);
        } else if (strcmp(token, "ki") == 0) {
          token = strtok(nullptr, " ");
          if (token) tempKi = atof(token);
        } else if (strcmp(token, "kd") == 0) {
          token = strtok(nullptr, " ");
          if (token) tempKd = atof(token);
        } else {
          token = strtok(nullptr, " ");
        }
      }
      targetSpeed = tempSP;
      speedPID.kp = tempKp;
      speedPID.ki = tempKi;
      speedPID.kd = tempKd;

      Serial.printf("[SPEED MODE] SP: %.2f deg/s | Kp=%.3f Ki=%.3f Kd=%.3f\n",
                    targetSpeed, speedPID.kp, speedPID.ki, speedPID.kd);
    }

    // --- DISTANCE TEST: "dist <deg> [kp <val> ki <val> kd <val>]" ---
    else if (command.startsWith("dist")) {
      float tempDist = 0, tempKp = distPID.kp, tempKi = distPID.ki, tempKd = distPID.kd;
      char *token = strtok((char *)command.c_str(), " ");
      while (token != nullptr) {
        if (strcmp(token, "dist") == 0) {
          token = strtok(nullptr, " ");
          if (token) tempDist = atof(token);
        } else if (strcmp(token, "kp") == 0) {
          token = strtok(nullptr, " ");
          if (token) tempKp = atof(token);
        } else if (strcmp(token, "ki") == 0) {
          token = strtok(nullptr, " ");
          if (token) tempKi = atof(token);
        } else if (strcmp(token, "kd") == 0) {
          token = strtok(nullptr, " ");
          if (token) tempKd = atof(token);
        } else {
          token = strtok(nullptr, " ");
        }
      }
      targetDistance = tempDist;
      distPID.kp = tempKp;
      distPID.ki = tempKi;
      distPID.kd = tempKd;

      Serial.printf("[DISTANCE MODE] Target=%.2f deg | Kp=%.3f Ki=%.3f Kd=%.3f\n",
                    targetDistance, distPID.kp, distPID.ki, distPID.kd);
    }

    // --- STOP COMMAND ---
    else if (command.startsWith("stop")) {
      targetSpeed = 0;
      targetDistance = 0;
      Serial.println("[STOP] Targets cleared.");
    }

    else {
      Serial.println("Unknown command. Use:");
      Serial.println("  speed <deg/s> [kp <val> ki <val> kd <val>]");
      Serial.println("  dist <deg> [kp <val> ki <val> kd <val>]");
      Serial.println("  stop");
    }
  }
}


void IRAM_ATTR encoderISRTimer() {
  if (EncoderReading_Task != NULL) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(EncoderReading_Task, &woken);
    if (woken) portYIELD_FROM_ISR();
  }
}

void EncoderReadingTask(void *parameters) {
  static int16_t lastAngle = 0;
  static long totalTurns = 0;

  for (;;) {
    // read raw angle
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(ANGLE_MSB);
    if (Wire.endTransmission(false) != 0) {
      continue;
    }

    Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
    if (Wire.available() < 2) {
      continue;
    }

    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    int16_t angle = ((msb << 8) | lsb) & 0x0FFF;  // 12-bit value

    // handle wrap-around (forward/backward rotation)
    int diff = angle - lastAngle;
    if (diff > 2048) diff -= 4096;
    else if (diff < -2048) diff += 4096;

    if (angle < 512 && lastAngle > 4096 - 512) totalTurns++;
    else if (angle > 4096 - 512 && lastAngle < 512) totalTurns--;

    lastAngle = angle;

    // fill struct
    encoder_sample_t sample;
    sample.raw_angle = angle;
    sample.turns = totalTurns;

    // send to queue (overwrite oldest if full)
    if (xQueueSend(sampleQueue, &sample, 0) != pdPASS) {
      // queue full, optional: handle overflow
    }

    // block until next notification (from timer ISR)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
}

void IRAM_ATTR MotorTaskTimer() {
  xTaskResumeFromISR(Motor_Task);
  portYIELD_FROM_ISR();  // Not required if task priority is 0
}


// Corrected function signature
float runPID(PIDController *pid, float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;
  pid->integral += error * dt;
  float derivative = (error - pid->prevError) / dt;
  pid->prevError = error;
  return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}


void MotorTask(void *parameters) {
  const float Ts = MOTOR_US / 1e6;  // 0.005 s (5 ms)
// 💡 --- DEBUGGING SETUP ---
#define DEBUG_PRINT_INTERVAL 50  // Print every 50 loops (50 * 5ms = 250ms)
  int debug_counter = 0;
  // -------------------------

  // Note: 'lastAbsAngleDeg' should be declared outside this task
  // if it's used elsewhere, or static float if only used here.

  for (;;) {
    int count = 0;
    long sumAbsCounts = 0;
    encoder_sample_t s;
    float absAngleDeg = lastAbsAngleDeg;  // Start with the last known angle
    float speed = 0;

    while (xQueueReceive(sampleQueue, &s, 0) == pdTRUE) {
      long absCounts = (long)s.turns * 4096L + s.raw_angle;
      sumAbsCounts += absCounts;
      count++;
    }

    if (count > 0) {
      float avgCounts = (float)sumAbsCounts / count;
      absAngleDeg = (avgCounts / 4096.0) * 360.0;
      speed = (absAngleDeg - lastAbsAngleDeg) / Ts;
      lastAbsAngleDeg = absAngleDeg;
    } else {
      // If no new samples, assume speed is zero.
      speed = 0;
    }

    // --- Outer loop (Position control) ---
    static int outerCount = 0;
    static unsigned long last_time_outer = millis();
    if (++outerCount >= 10) {
      outerCount = 0;
      float distance = absAngleDeg;
      unsigned long now = millis();
      float dt = (now - last_time_outer) / 1000.0f;
      // Prevent division by zero on the first run
      if (dt <= 0) dt = 0.05;

      // The output of this PID is the new target speed
      targetSpeed = runPID(&distPID, targetDistance, distance, dt);
      last_time_outer = now;

      // 💡 DEBUG PRINT for Outer Loop
      if (debug_counter % DEBUG_PRINT_INTERVAL == 0) {
        Serial.printf("OUTER -> TgtDist: %.2f, CurDist: %.2f, NewTgtSpd: %.2f\n",
                      targetDistance, distance, targetSpeed);
      }
    }

    // --- Inner loop (Speed control) ---
    float pwmCmd = runPID(&speedPID, targetSpeed, speed, Ts);

    // Apply PWM
    pwmCmd = constrain(pwmCmd, -100, 100);
    motor.move(pwmCmd);

    // 💡 DEBUG PRINT for Inner Loop and Final Output
    if (debug_counter % DEBUG_PRINT_INTERVAL == 0) {
      Serial.printf("INNER -> TgtSpd: %.2f, CurSpd: %.2f, PWM: %.2f\n\n",
                    targetSpeed, speed, pwmCmd);
    }
    debug_counter++;

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
}



/* ----------- Deprecated ------------------------

void IRAM_ATTR encoderISR() {
  uint8_t curr_state = (digitalRead(C1_ENCODER_PIN) << 1) | digitalRead(C2_ENCODER_PIN);
  uint8_t transition = (last_encoder_state << 2) | curr_state;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      encoder_ticks++;
      break;
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      encoder_ticks--;
      break;
    default:
      // Invalid or bounce — ignore
      break;
  }
  last_encoder_state = curr_state;

}

void SpeedCalculationTask(void *parameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);  // Run every 10ms

  while (1) {
    unsigned long now = millis();

    // Use FreeRTOS critical section (no mutex needed)
    portENTER_CRITICAL(&mux);
    delta_Ticks = encoder_ticks - last_encoder_ticks;
    last_encoder_ticks = encoder_ticks;
    portEXIT_CRITICAL(&mux);

    // Calculate time difference
    static unsigned long last_time = 0;  // Static to preserve value
    unsigned long dt = now - last_time;
    last_time = now;

    if (dt > 0) {
      portENTER_CRITICAL(&motorMux);
      Current_speed = (delta_Ticks * ticksToRadians) / (dt / 1000.0);
      portEXIT_CRITICAL(&motorMux);
    }
    // Periodic execution (replaces vTaskSuspend)
    Serial.println(encoder_ticks);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


void DisplacementTask(void *parameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(30);

  while (1) {
    unsigned long now = millis();
    static unsigned long last_time = 0;
    unsigned long dt = now - last_time;
    last_time = now;

    double current_position;
    portENTER_CRITICAL(&mux);
    current_position = encoder_ticks;
    portEXIT_CRITICAL(&mux);

    double error = target_position - current_position;
    displacement_integral += error * (dt / 1000.0);
    double derivative = (error - last_displacement_error) / (dt / 1000.0);
    last_displacement_error = error;

    double output_speed = displacementKp * error + displacementKi * displacement_integral + displacementKd * derivative;

    // Stop when within 1 mm worth of ticks (you can adjust this tolerance)
    if (fabs(error) < (ticks_per_mm * 1.0)) {
      portENTER_CRITICAL(&motorMux);
      radsSP = 0;
      portEXIT_CRITICAL(&motorMux);

      displacement_integral = 0;
      last_displacement_error = 0;

      vTaskSuspend(NULL);  // Suspend this task until resumed
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

*/
