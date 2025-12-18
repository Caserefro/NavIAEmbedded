#ifndef Manual_Control.h
#define Manual_Control_h

bool connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return true;

  Serial.println("[WiFi] Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  const uint8_t maxAttempts = 20;  // ~10 seconds
  uint8_t attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Connected. IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("[WiFi] FAILED to connect.");
  return false;
}

void disconnectWifi() {
  Serial.println("[WiFi] Disconnecting...");

  WiFi.disconnect(true);  // drop connection + erase config
  delay(100);

  Serial.println("[WiFi] Disconnected and radio turned OFF.");
}

void onEvent(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("[WS] Connected!");
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("[WS] Disconnected!");
  } else if (event == WebsocketsEvent::GotPing) {
    Serial.println("[WS] Got ping");
  } else if (event == WebsocketsEvent::GotPong) {
    Serial.println("[WS] Got pong");
  }
}

void onMessageCallback(WebsocketsMessage message) {
  static char cmd[64];  // persistent buffer
  memset(cmd, 0, sizeof(cmd));

  String msg = message.data();        // keep String alive
  msg.trim();                         // removes \r \n \t spaces
  msg.toCharArray(cmd, sizeof(cmd));  // SAFE copy

  Serial.printf("[WS RX] '%s'\n", cmd);

  float x, y, th;
  //Serial.println(message.data());
  // ==========================================
  // CASE 1: POSITION COMMAND -> "P,1.5,0.0,1.57"
  // ==========================================
  if (cmd[0] == 'P') {
    if (sscanf(cmd, "P,%f,%f,%f", &x, &y, &th) == 3) {

      Serial.printf("[WS] New Waypoint: %.2f %.2f %.2f\n", x, y, th);

      // 1. Push to Queue
      PositionTarget_t pt = { x, y, th };
      xQueueSend(PositionBuffer, &pt, 0);

      // 2. FORCE odometry re-sync (CRITICAL)
      isOdometryInitialized = false;
      for (int i = 0; i < 4; i++) {
        w[i].seen = false;
        w[i].updated = false;
      }

      // 3. Kick encoder feedback immediately
      if (xSemaphoreTake(SerialOutputMutex, 5)) {
        Serial2.write("O");
        xSemaphoreGive(SerialOutputMutex);
      }

      // 4. Switch mode
      if (currentMode != MODE_POSITION) {
        Serial.println("[WS] Switching to POSITION MODE");
        currentMode = MODE_POSITION;
      }

    } else {
      Serial.println("[ERR] Invalid P format");
    }
    return;
  }


  // ==========================================
  // CASE 2: SPEED COMMAND -> "S,0.5,0.0,0.0"
  // ==========================================
  if (cmd[0] == 'S') {
    if (sscanf(cmd, "S,%f,%f,%f", &x, &y, &th) == 3) {

      // 1. Update Globals
      manual_req_x = x;
      manual_req_y = y;
      manual_req_th = th;

      // 2. Switch Mode (if not already)
      if (currentMode != MODE_MANUAL) {
        Serial.println("[WS] Switching to MANUAL MODE");
        currentMode = MODE_MANUAL;
      }

      // Debug (Optional, can be spammy)
      //Serial.printf("[WS MAN] %.2f %.2f %.2f\n", x, y, th);

    } else {
      Serial.println("[ERR] Invalid S format");
    }
    return;
  }

  // ==========================================
  // CASE 3: LEGACY FALLBACK (Raw Numbers)
  // "0.5,0.0,0.0" (If your joystick app can't send 'S')
  // ==========================================
  if (sscanf(cmd, "%f,%f,%f", &x, &y, &th) == 3) {
    manual_req_x = x;
    manual_req_y = y;
    manual_req_th = th;

    if (currentMode != MODE_MANUAL) {
      currentMode = MODE_MANUAL;
    }
    return;
  }

  Serial.printf("[ERR] Unknown WS Packet: %s\n", cmd);
}

/*
void onMessageCallback(WebsocketsMessage message) {
  float x, y, th;

  // Parse "-0.38,-0.08,0.00"
  if (sscanf(message.data().c_str(), "%f,%f,%f", &x, &y, &th) != 3) {
    Serial.println("[ERR] Invalid manual message");
    return;
  }

  // ----- Run Mecanum IK -----
  WheelRotations w = mecanum_ik(
    x,   // dx normalized input
    y,   // dy normalized input
    th,  // dtheta normalized input
    WHEEL_RADIUS,
    HALF_LENGTH,
    HALF_WIDTH);

  // ----- NORMALIZE so direction is preserved -----
  float maxVal = max(
    max(fabs(w.m1), fabs(w.m2)),
    max(fabs(w.m3), fabs(w.m4)));

  if (maxVal > 1.0f) {
    w.m1 /= maxVal;
    w.m2 /= maxVal;
    w.m3 /= maxVal;
    w.m4 /= maxVal;
  }

  float scale = 10.0f;  // rad/s max motor speed

  float m1 = w.m1 * scale;
  float m2 = w.m2 * scale;
  float m3 = w.m3 * scale;
  float m4 = w.m4 * scale;

  // ----- Wheel signs (3 and 4 inverted on your robot) -----
  m3 *= -1;
  m4 *= -1;

  // Debug
  Serial.printf("[MAN] x=%.2f y=%.2f th=%.2f -> %.2f %.2f %.2f %.2f\n",
                x, y, th, m1, m2, m3, m4);

  // ----- Output to motors via your serial command -----
  xSemaphoreTake(SerialOutputMutex, portMAX_DELAY);
  Serial2.printf("S,%.2f,%.2f,%.2f,%.2f\n", m1, m2, m3, m4);
  xSemaphoreGive(SerialOutputMutex);
}
*/
#endif