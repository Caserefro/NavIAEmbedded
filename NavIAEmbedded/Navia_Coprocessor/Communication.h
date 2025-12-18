#ifndef Communication.h
#define Communication_h


void USBSERIAL_Handler() {
  static char cmdBuf[64];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();

    // Build command string
    if (c == '\n' || c == '\r') {
      cmdBuf[idx] = '\0';
      idx = 0;
      handleCommand(cmdBuf);  // process full command
    } else {
      if (idx < sizeof(cmdBuf) - 1)
        cmdBuf[idx++] = c;
    }
  }
}

void handleCommand(const char* cmd) {

  // ---------------------------
  //  SERVER CONTROL: W,start / W,stop
  // ---------------------------
  if (cmd[0] == 'W' && cmd[1] == ',') {

    // CASE A: STOP SERVER
    if (strcmp(cmd, "W,stop") == 0) {
      Serial.println("[CMD] Stopping WiFi/Server...");
      wifiEnabled = false;

      // Optional: Drop to IDLE if you want the robot to stop moving when WiFi cuts
      // currentMode = MODE_IDLE;
      return;
    }

    // CASE B: START SERVER (Any other W command, e.g., "W,start")
    else {
      Serial.println("[CMD] Enabling Server Mode...");
      wifiEnabled = true;  // Signals NetworkManagerTask to start AP/Station & Listen

      // Default to Manual Mode so the supervisor is ready for Joystick data
      currentMode = MODE_MANUAL;
      return;
    }
  }

  // ---------------------------
  //  POSITION MODE: P,x,y,theta
  // ---------------------------
  if (cmd[0] == 'P' && cmd[1] == ',') {
    float x, y, th;
    if (sscanf(cmd, "P,%f,%f,%f", &x, &y, &th) == 3) {
      Serial.printf("[CMD] Position Target: %.2f %.2f %.2f\n", x, y, th);

      // 1. Add to Queue
      PositionTarget_t pt = { x, y, th };
      xQueueSend(PositionBuffer, &pt, 0);
      //      Serial2.write("O");
      // 2. Change Mode
      currentMode = MODE_POSITION;
    } else {
      Serial.println("[ERR] Invalid P Command Format");
    }
    return;
  }

  // ---------------------------
  //  DIRECT SPEED DEBUG: S,m1,m2,m3,m4
  //  (Useful for testing motors via USB without WiFi)
  // ---------------------------
  if (cmd[0] == 'S' && cmd[1] == ',') {
    // Just pass-through to motors (Dangerous! Be careful!)
    if (xSemaphoreTake(SerialOutputMutex, (TickType_t)10) == pdTRUE) {
      Serial2.printf("%s\n", cmd);  // Send raw string: "S,10,10,10,10"
      xSemaphoreGive(SerialOutputMutex);
    }
    return;
  }

  // ---------------------------
  //  EMERGENCY STOP
  // ---------------------------
  if (strcmp(cmd, "STOP") == 0) {
    Serial.println("[CMD] EMERGENCY STOP");
    currentMode = MODE_IDLE;  // Tasks will see this and stop logic

    // Optional: Kill WiFi too?
    // wifiEnabled = false;

    // Send immediate 0 command (Safety override)
    if (xSemaphoreTake(SerialOutputMutex, (TickType_t)10) == pdTRUE) {
      Serial2.println("S,0,0,0,0");
      xSemaphoreGive(SerialOutputMutex);
    }
    return;
  }

  Serial.printf("[ERR] Unknown Command: %s\n", cmd);
}

// Helper: Extracts numbers from string
bool parseWheelPacketHelper(const char* buf, int& id, float& cur, float& err, float& tgt) {
  return sscanf(buf, "%d,%f,%f,%f", &id, &cur, &err, &tgt) == 4;
}


#endif