#ifndef DEVICE_NETWORKING.h
#define DEVICE_NETWORKING_H

void checkUSBInput() {
  static String inputBuffer = "";

  while (Serial.available()) {
    char incomingByte = Serial.read();

    if (incomingByte == '\n') {
      UART_SEND(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += incomingByte;
    }
  }
}


void UART_SEND(String message) {
  Serial.print("Sent to UART1: ");
  Serial.println(message);
  for (int i = 0; i < message.length() + 1; i++)
    SystemUART.write(message[i]);
}



String readUARTLine(HardwareSerial &uart) {
  String result = "";
  while (uart.available()) {
    char c = uart.read();
    if (c == '\n') break;  // end of message
    result += c;
  }
  return result;
}
/*
void checkSystemUART() { //TOP RIGHT
  if (SystemUART.available()) {
    String command = readUARTLine(SystemUART);

    Serial.print("Received: ");
    Serial.println(command);

    // Skip 'CMD:' prefix if present
    int colonIndex = command.indexOf(':');
    String cmdPart = (colonIndex == -1) ? command : command.substring(colonIndex + 1);

    if (cmdPart.length() < 3) return;  // Not enough characters

    char mode = cmdPart.charAt(0);       // L / C / Z
    char dir = cmdPart.charAt(1);        // F / B / L / R / G / H / I / J
    float value = cmdPart.substring(2).toFloat();

    float Vy = 0, Vx = 0, W = 0; // forward/backward, strafe, rotation

    // Geometry of robot
    const float L = 0.275;   // in meters
    const float W_ = 0.22;
    const float R = L + W_;  // Rotation coefficient

    if (mode == 'L') {
      // Linear motion (finite distance)
      switch (dir) {
        case 'F': Vy = value; break;
        case 'B': Vy = -value; break;
        case 'L': Vx = -value; break;
        case 'R': Vx = value; break;
        case 'H': Vy = value; Vx = value; break;   // Forward-right
        case 'G': Vy = value; Vx = -value; break;  // Forward-left
        case 'I': Vy = -value; Vx = value; break;  // Backward-right
        case 'J': Vy = -value; Vx = -value; break; // Backward-left
        default: break;
      }

      // Convert mm to wheel angular distance in rads for top-right wheel
      float wheelLinearDist_mm = Vy - Vx + W * R;
      float wheelCircumference_mm = 97.0 * PI;
      float revs = wheelLinearDist_mm / wheelCircumference_mm;
      float ticks_per_rev = 2000.0;
      long ticks = revs * ticks_per_rev;

      // Set the linear distance target (in ticks or rads based on your motor driver)
      LinearDistanceSP = ticks;  // or set distance in mm if working in mm internally

      Serial.print("Linear movement command — wheel ticks target: ");
      Serial.println(ticks);

    } else if (mode == 'C') {
      // Continuous motion (in rad/s)
      switch (dir) {
        case 'F': Vy = value; break;
        case 'B': Vy = -value; break;
        case 'L': Vx = -value; break;
        case 'R': Vx = value; break;
        case 'H': Vy = value; Vx = value; break;
        case 'G': Vy = value; Vx = -value; break;
        case 'I': Vy = -value; Vx = value; break;
        case 'J': Vy = -value; Vx = -value; break;
        default: break;
      }

      float wheelSpeed = Vy - Vx + W * R;
      radsSP = wheelSpeed;

      Serial.print("Continuous motion — target rad/s: ");
      Serial.println(radsSP);

    } else if (mode == 'Z') {
      // Rotation-only mode
      if (dir == 'L') W = -value;
      else if (dir == 'R') W = value;

      float wheelSpeed = Vy - Vx + W * R;
      radsSP = wheelSpeed;

      Serial.print("Rotation command — wheel rad/s: ");
      Serial.println(radsSP);
    }
  }
}
*/
/*
void checkSystemUART() { //TOP LEFT
  if (SystemUART.available()) {
    String command = readUARTLine(SystemUART);

    Serial.print("Received: ");
    Serial.println(command);

    // Skip "CMD:" prefix if present
    int colonIndex = command.indexOf(':');
    String cmdPart = (colonIndex == -1) ? command : command.substring(colonIndex + 1);

    if (cmdPart.length() < 3) return;

    char mode = cmdPart.charAt(0);       // L / C / Z
    char dir = cmdPart.charAt(1);        // F / B / L / R / G / H / I / J
    float value = cmdPart.substring(2).toFloat();

    float Vy = 0, Vx = 0, W = 0; // forward/backward, strafe, rotation

    // Robot geometry
    const float L = 0.275;   // Length in meters (front-back)
    const float W_ = 0.22;   // Width in meters (left-right)
    const float R = L + W_;  // Combined turning radius

    if (mode == 'L') {
      // Linear movement (finite distance)
      switch (dir) {
        case 'F': Vy = value; break;
        case 'B': Vy = -value; break;
        case 'L': Vx = -value; break;
        case 'R': Vx = value; break;
        case 'H': Vy = value; Vx = value; break;   // Forward-right
        case 'G': Vy = value; Vx = -value; break;  // Forward-left
        case 'I': Vy = -value; Vx = value; break;  // Backward-right
        case 'J': Vy = -value; Vx = -value; break; // Backward-left
        default: break;
      }

      // Top-Left wheel kinematics: Vy + Vx - W*R
      float wheelLinearDist_mm = Vy + Vx - W * R;
      float wheelCircumference_mm = 97.0 * PI;
      float revs = wheelLinearDist_mm / wheelCircumference_mm;
      float ticks_per_rev = 2000.0;
      long ticks = revs * ticks_per_rev;

      LinearDistanceSP = ticks;

      Serial.print("Linear movement (Top-Left) — wheel ticks: ");
      Serial.println(ticks);

    } else if (mode == 'C') {
      // Continuous motion (speed in rad/s)
      switch (dir) {
        case 'F': Vy = value; break;
        case 'B': Vy = -value; break;
        case 'L': Vx = -value; break;
        case 'R': Vx = value; break;
        case 'H': Vy = value; Vx = value; break;
        case 'G': Vy = value; Vx = -value; break;
        case 'I': Vy = -value; Vx = value; break;
        case 'J': Vy = -value; Vx = -value; break;
        default: break;
      }

      // Top-Left wheel speed: Vy + Vx - W*R
      float wheelSpeed = Vy + Vx - W * R;
      radsSP = wheelSpeed;

      Serial.print("Continuous motion (Top-Left) — rad/s: ");
      Serial.println(radsSP);

    } else if (mode == 'Z') {
      // Rotation-only
      if (dir == 'L') W = -value;
      else if (dir == 'R') W = value;

      float wheelSpeed = Vy + Vx - W * R;
      radsSP = wheelSpeed;

      Serial.print("Rotation (Top-Left) — rad/s: ");
      Serial.println(radsSP);
    }
  }
}
*/
/* 
void checkSystemUART() {//Bottom RIGHT
  if (SystemUART.available()) {
    String command = readUARTLine(SystemUART);

    Serial.print("Received: ");
    Serial.println(command);

    // Allow optional "CMD:" prefix
    int colonIndex = command.indexOf(':');
    String cmdPart = (colonIndex == -1) ? command : command.substring(colonIndex + 1);

    if (cmdPart.length() < 3) return;

    char mode = cmdPart.charAt(0);       // L / C / Z
    char dir = cmdPart.charAt(1);        // F / B / L / R / G / H / I / J
    float value = cmdPart.substring(2).toFloat();

    float Vy = 0, Vx = 0, W = 0; // forward/backward, strafe, rotation

    // Robot geometry (meters)
    const float L = 0.275;   // front-back
    const float W_ = 0.22;   // left-right
    const float R = L + W_;  // turning radius

    if (mode == 'L') {
      // Linear movement
      switch (dir) {
        case 'F': Vy = value; break;
        case 'B': Vy = -value; break;
        case 'L': Vx = -value; break;
        case 'R': Vx = value; break;
        case 'H': Vy = value; Vx = value; break;   // Forward-right
        case 'G': Vy = value; Vx = -value; break;  // Forward-left
        case 'I': Vy = -value; Vx = value; break;  // Backward-right
        case 'J': Vy = -value; Vx = -value; break; // Backward-left
        default: break;
      }

      // Bottom-Right wheel: Vy + Vx + W * R
      float wheelLinearDist_mm = Vy + Vx + W * R;
      float wheelCircumference_mm = 97.0 * PI;
      float revs = wheelLinearDist_mm / wheelCircumference_mm;
      float ticks_per_rev = 2000.0;
      long ticks = revs * ticks_per_rev;

      LinearDistanceSP = ticks;

      Serial.print("Linear movement (Bottom-Right) — wheel ticks: ");
      Serial.println(ticks);

    } else if (mode == 'C') {
      // Continuous motion (speed)
      switch (dir) {
        case 'F': Vy = value; break;
        case 'B': Vy = -value; break;
        case 'L': Vx = -value; break;
        case 'R': Vx = value; break;
        case 'H': Vy = value; Vx = value; break;
        case 'G': Vy = value; Vx = -value; break;
        case 'I': Vy = -value; Vx = value; break;
        case 'J': Vy = -value; Vx = -value; break;
        default: break;
      }

      float wheelSpeed = Vy + Vx + W * R;
      radsSP = wheelSpeed;

      Serial.print("Continuous motion (Bottom-Right) — rad/s: ");
      Serial.println(radsSP);

    } else if (mode == 'Z') {
      // Rotation only
      if (dir == 'L') W = -value;
      else if (dir == 'R') W = value;

      float wheelSpeed = Vy + Vx + W * R;
      radsSP = wheelSpeed;

      Serial.print("Rotation (Bottom-Right) — rad/s: ");
      Serial.println(radsSP);
    }
  }
}
*/
/*
void checkSystemUART() {
  if (SystemUART.available()) {
    String command = readUARTLine(SystemUART);

    Serial.print("Received: ");
    Serial.println(command);

    // Allow optional "CMD:" prefix
    int colonIndex = command.indexOf(':');
    String cmdPart = (colonIndex == -1) ? command : command.substring(colonIndex + 1);

    if (cmdPart.length() < 3) return;

    char mode = cmdPart.charAt(0);       // L / C / Z
    char dir = cmdPart.charAt(1);        // F / B / L / R / G / H / I / J
    float value = cmdPart.substring(2).toFloat();

    float Vy = 0, Vx = 0, W = 0; // forward/backward, strafe, rotation

    // Robot geometry (meters)
    const float L = 0.275;
    const float W_ = 0.22;
    const float R = L + W_;

    if (mode == 'L') {
      // Linear movement
      switch (dir) {
        case 'F': Vy = value; break;
        case 'B': Vy = -value; break;
        case 'L': Vx = -value; break;
        case 'R': Vx = value; break;
        case 'H': Vy = value; Vx = value; break;   // Forward-right
        case 'G': Vy = value; Vx = -value; break;  // Forward-left
        case 'I': Vy = -value; Vx = value; break;  // Backward-right
        case 'J': Vy = -value; Vx = -value; break; // Backward-left
        default: break;
      }

      // Bottom-Left wheel: Vy - Vx - W * R
      float wheelLinearDist_mm = Vy - Vx - W * R;
      float wheelCircumference_mm = 97.0 * PI;
      float revs = wheelLinearDist_mm / wheelCircumference_mm;
      float ticks_per_rev = 2000.0;
      long ticks = revs * ticks_per_rev;

      // Update displacement control target
      portENTER_CRITICAL(&mux);
      target_position = encoder_ticks + LinearDistanceSP;
      portEXIT_CRITICAL(&mux);

      // Activate DisplacementTask
      vTaskResume(Displacement_Task);

      Serial.print("Linear movement (Bottom-Left) — wheel ticks: ");
      Serial.println(ticks);

    } else if (mode == 'C') {
      // Continuous motion
      switch (dir) {
        case 'F': Vy = value; break;
        case 'B': Vy = -value; break;
        case 'L': Vx = -value; break;
        case 'R': Vx = value; break;
        case 'H': Vy = value; Vx = value; break;
        case 'G': Vy = value; Vx = -value; break;
        case 'I': Vy = -value; Vx = value; break;
        case 'J': Vy = -value; Vx = -value; break;
        default: break;
      }

      float wheelSpeed = Vy - Vx - W * R;
      radsSP = wheelSpeed;

      Serial.print("Continuous motion (Bottom-Left) — rad/s: ");
      Serial.println(radsSP);

    } else if (mode == 'Z') {
      // Rotation only
      if (dir == 'L') W = -value;
      else if (dir == 'R') W = value;

      float wheelSpeed = Vy - Vx - W * R;
      radsSP = wheelSpeed;

      Serial.print("Rotation (Bottom-Left) — rad/s: ");
      Serial.println(radsSP);
    }
  }
}

*/
#endif