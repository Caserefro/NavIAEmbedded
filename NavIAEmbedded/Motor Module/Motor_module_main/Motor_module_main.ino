#include "MOTOR_MODULE_MAIN.h"

void setup() {
  Serial.begin(115200);  // USB Serial
  SystemUART.begin(115200, SERIAL_8N1, SystemUARTRX, SystemUARTTX);
  Wire.begin(SNS_SDA, SNS_SCL, 400000);  // 400 kHz or lower
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  motor.begin();
  //motorSetup();
  Serial.println("I CAN PRINT ran succesfull --------------------------");
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

  Serial.println("Timers and tasks started");
}

void loop() {
  checkSystemUART();
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();

    // --- PLOTTER MODE COMMANDS ---
    if (command.startsWith("plotter")) {
      if (command.indexOf("on") != -1) {
        plotterMode = true;
        Serial.println("Plotter mode ON - sending CSV data for graphing");
      } else if (command.indexOf("off") != -1) {
        plotterMode = false;
        Serial.println("Plotter mode OFF - returning to detailed debug prints");
      } else {
        Serial.println("Use: 'plotter on' or 'plotter off'");
      }
    }
    // --- POSITION CONTROL: "position <value> [kp <val> ki <val> kd <val>]" ---
    else if (command.startsWith("position") || command.startsWith("pos")) {
      float tempPos = targetPosition;
      float tempKp = Kp_pos;
      float tempKi = Ki_pos;
      float tempKd = Kd_pos;

      // Reset position PID state when switching
      prev_error_pos = 0;
      integral_pos = 0;

      char *token = strtok((char *)command.c_str(), " ");
      while (token != nullptr) {
        if (strcmp(token, "position") == 0 || strcmp(token, "pos") == 0) {
          token = strtok(nullptr, " ");
          if (token) tempPos = atof(token);
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
      // Apply changes
      targetPosition = tempPos;
      Kp_pos = tempKp;
      Ki_pos = tempKi;
      Kd_pos = tempKd;
      positionControlMode = true;

      if (!plotterMode) {
        Serial.printf("[POSITION MODE] Target: %.3f rad | Kp=%.3f Ki=%.3f Kd=%.3f\n",
                      targetPosition, Kp_pos, Ki_pos, Kd_pos);
      }
    }

    else if (command.startsWith("speed")) {
      float tempSP = targetSpeed;
      float tempKp = Kp_spd;
      float tempKi = Ki_spd;
      float tempKd = Kd_spd;
      integral_spd = 0;
      prev_error_spd = 0;

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

      // Apply changes
      targetSpeed = tempSP;
      Kp_spd = tempKp;
      Ki_spd = tempKi;
      Kd_spd = tempKd;
      positionControlMode = false;

      if (!plotterMode) {
        Serial.printf("[SPEED MODE] SP: %.2f rad/s | Kp=%.3f Ki=%.3f Kd=%.3f\n",
                      targetSpeed, Kp_spd, Ki_spd, Kd_spd);
      }
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

    encoder_sample_t sample;
    sample.raw_angle = angle;
    sample.turns = totalTurns;

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


void MotorTask(void *parameters) {
  const float Ts = MOTOR_US / 1e6f;
  float derivative = 0;
  float integral = 0;
  static bool lastErrorPositive = true;
  float lastShaftAngleDeg = 0;  // Track shaft angle, not motor angle
  float shaftSpeed = 0;         // Shaft speed in rad/s
  int debug_counter = 0;
  // Position control variables
  float positionError = 0.0f;
  float deltaPositionError = 0.0f;
  float motorAngleDeg = 0;
  const float MAX_TARGET_SPEED = 12;


  for (;;) {
    int count = 0;
    long sumAbsCounts = 0;
    encoder_sample_t s;

    while (xQueueReceive(sampleQueue, &s, 0) == pdTRUE) {
      long absCounts = (long)s.turns * 4096L + s.raw_angle;
      sumAbsCounts += absCounts;
      count++;
    }

    float avgCounts = 0;
    float deltaMotorAngleDeg = 0;

    if (count > 0) {
      avgCounts = (float)sumAbsCounts / count;

      // Calculate motor angle in degrees
      motorAngleDeg = (avgCounts / 4096.0f) * 360.0f;
      // Convert motor angle to SHAFT angle
      float shaftAngleDeg = motorAngleDeg / GEAR_RATIO;
      // Calculate shaft speed (output speed) in rad/s
      deltaMotorAngleDeg = motorAngleDeg - lastAbsMotorAngleDeg;
      shaftSpeed = ((deltaMotorAngleDeg * DEG_TO_RAD) / Ts) / GEAR_RATIO;  // rad/s at output shaft

      // Update position in radians (shaft position)
      currentPositionRad = shaftAngleDeg * (PI / 180.0f);

      // Store for next iteration
      lastAbsMotorAngleDeg = motorAngleDeg;  // Store motor angle for speed calculation
      lastShaftAngleDeg = shaftAngleDeg;     // Store shaft angle for position reference
    }

    // ===== POSITION CONTROL MODE =====
    float targetSpeed_cmd = targetSpeed;  // default (speed mode)

    if (positionControlMode) {
      error_pos = targetPosition - currentPositionRad;

      // Position PID only active if error is larger than threshold
      const float POSITION_DEADBAND = 0.005f;  // rad, ~0.6°
      if (fabs(error_pos) > POSITION_DEADBAND) {
        integral_pos += error_pos * Ts;
        float derivative_pos = (error_pos - prev_error_pos) / Ts;
        prev_error_pos = error_pos;

        // Position PID outputs target speed in rad/s
        targetSpeed_cmd = Kp_pos * error_pos + Ki_pos * integral_pos + Kd_pos * derivative_pos;

        // Limit so the position loop doesn’t demand crazy speeds
        targetSpeed_cmd = constrain(targetSpeed_cmd, -MAX_TARGET_SPEED, MAX_TARGET_SPEED);
      } else {
        // Small error → stop sending speed command
        targetSpeed_cmd = 0.0f;

        // Optional: reset integrator when inside deadband to avoid windup
        integral_pos = 0.0f;
        integral_spd = 0.0f;
        prev_error_pos = 0.0f;
      }
    }


    // ===== SPEED PID CONTROLLER (always runs) =====
    // Both targetSpeed_cmd and shaftSpeed are in SHAFT rad/s
    float error_spd = targetSpeed_cmd - shaftSpeed;

    //Conditional integrator
    bool errorPositive = (error_spd >= 0.03f);
    if (errorPositive == lastErrorPositive) {
      integral_spd += error_spd * Ts;
    }
    lastErrorPositive = errorPositive;

    derivative_spd = (error_spd - prev_error_spd) / Ts;
    float controlOutput = Kp_spd * error_spd + Ki_spd * integral_spd + Kd_spd * derivative_spd;
    prev_error_spd = error_spd;

    float pwmCmd = (controlOutput / 11.4) * 255.0f;
    pwmCmd = constrain(pwmCmd, -255, 255);

    motor.move(pwmCmd);
    if (debug_on) {
      if (++debug_counter >= DEBUG_PRINT_INTERVAL && DEBUG_PRINT_INTERVAL > 0) {
        debug_counter = 0;

        if (plotterMode) {
          if (positionControlMode) {
            // Plotter mode: show position error, current pos, outer-loop target speed, and PWM
            Serial.print(error_pos, 4);
            Serial.print(",");
            Serial.print(currentPositionRad, 4);
            Serial.print(",");
            Serial.print(targetSpeed_cmd, 4);  // inner loop target
            Serial.print(",");
            Serial.println(pwmCmd, 1);
          } else {
            // Plotter mode: show speed PID signals
            Serial.print(targetSpeed, 4);
            Serial.print(",");
            Serial.print(shaftSpeed, 4);
            Serial.print(",");
            Serial.print(error_spd, 4);
            Serial.print(",");
            Serial.println(pwmCmd, 1);
          }
        } else {
          // ===== DETAILED TEXT MODE =====
          if (positionControlMode) {
            Serial.println(F("===== POSITION CONTROL MODE ====="));
            Serial.printf("TargetPos: %.3f rad | CurrentPos: %.3f rad | ErrorPos: %.3f rad\n",
                          targetPosition, currentPositionRad, error_pos);
            Serial.printf("PosPID -> P: %.3f | I: %.3f | D: %.3f | Out(TargetSpeed): %.3f rad/s\n",
                          Kp_pos * error_pos,
                          Ki_pos * integral_pos,
                          Kd_pos * ((error_pos - prev_error_pos) / Ts),
                          targetSpeed_cmd);

            Serial.println(F("----- Inner Speed PID -----"));
            Serial.printf("TargetSpeed: %.3f rad/s | ShaftSpeed: %.3f rad/s | ErrorSpd: %.3f rad/s\n",
                          targetSpeed_cmd, shaftSpeed, error_spd);
            Serial.printf("SpdPID -> P: %.3f | I: %.3f | D: %.3f | Out: %.3f\n",
                          Kp_spd * error_spd,
                          Ki_spd * integral_spd,
                          Kd_spd * derivative_spd,  // Use the already calculated derivative
                          controlOutput);
            Serial.printf("PWM Cmd: %.2f\n", pwmCmd);
            Serial.println(F("===============================\n"));

          } else {
            Serial.println(F("===== SPEED CONTROL MODE ====="));
            Serial.printf("Samples: %d | AvgCounts: %.2f | absAngleDeg: %.2f | Δangle: %.2f\n",
                          count, avgCounts, motorAngleDeg, deltaMotorAngleDeg);
            Serial.printf("TargetSpeed: %.3f rad/s | ShaftSpeed: %.3f rad/s | ErrorSpd: %.3f\n",
                          targetSpeed, shaftSpeed, error_spd);
            Serial.printf("SpdPID -> Kp_spd: %.3f P: %.3f | Ki_spd: %.3f I: %.3f | Kd_spd: %.3f D: %.3f | Out: %.3f\n",
                          Kp_spd, Kp_spd * error_spd,
                          Ki_spd, Ki_spd * integral_spd,
                          Kd_spd, Kd_spd * ((error_spd - prev_error_spd) / Ts),
                          controlOutput);
            Serial.printf("PWM Cmd: %.2f\n", pwmCmd);
            Serial.println(F("===============================\n"));
          }
        }
      }
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
}

void checkSystemUART() {
  if (SystemUART.available()) {
    String command = readUARTLine(SystemUART);
    command.trim();  // Remove any newline/whitespace characters

    // --- Debug toggle ---
    if (command.equalsIgnoreCase("D")) {
      debug_on = !debug_on;
      Serial.printf("[DEBUG] Debug mode is now: %s\n", debug_on ? "ON" : "OFF");
      return;
    }

    // Odometry polling
    if (command.equalsIgnoreCase("O")) {
      vTaskDelay(pdMS_TO_TICKS(INITIAL_UPDATE_DELAY));
      String odom_message =
        String(DEVICE_ID) + "," + String(currentPositionRad, 4) + "," + String(error_pos, 4) + "," + String(targetPosition, 4);
      UART_SEND(odom_message);
      return;
    }

    // --- Multi-value command parser ---
    if (command.length() > 1) {

      int firstComma = command.indexOf(',');
      int secondComma = command.indexOf(',', firstComma + 1);
      int thirdComma = command.indexOf(',', secondComma + 1);
      int fourthComma = command.indexOf(',', thirdComma + 1);

      if (firstComma == -1 || secondComma == -1 || thirdComma == -1 || fourthComma == -1) {

        Serial.println("[UART ERROR] Expected format: M,val1,val2,val3,val4");
        return;
      }

      char mode = command.charAt(0);

      float motor1 = command.substring(firstComma + 1, secondComma).toFloat();
      float motor2 = command.substring(secondComma + 1, thirdComma).toFloat();
      float motor3 = command.substring(thirdComma + 1, fourthComma).toFloat();
      float motor4 = command.substring(fourthComma + 1).toFloat();

      float value = 0;
      if (DEVICE_ID == 1) value = motor1;
      else if (DEVICE_ID == 2) value = motor2;
      else if (DEVICE_ID == 3) value = motor3;
      else if (DEVICE_ID == 4) value = motor4;

      if (mode == 'S' || mode == 's') {
        positionControlMode = false;
        targetSpeed = value;
        integral_spd = 0;
        prev_error_spd = 0;

        Serial.printf("[MODE] Speed: %.2f rad/s\n", value);

      } else if (mode == 'P' || mode == 'p') {
        positionControlMode = true;
        targetPosition = value;
        prev_error_pos = 0;
        integral_pos = 0;

        Serial.printf("[MODE] Position: %.2f rad\n", value);
      }
    }
  }
}