#include "NAVIA_INTERMEDIARY.h"
// EXAMPLE SPEED COMMAND S,-2,2,-2,2
void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- 1. INIT OBJECTS FIRST (Critical) ---
  SerialOutputMutex = xSemaphoreCreateMutex();
  PositionBuffer = xQueueCreate(10, sizeof(PositionTarget));  // Fixed size constant

  // --- 2. INIT HARDWARE ---
  Serial2.begin(115200, SERIAL_8N1, 17, 16);  // Motor Serial
  Serial1.begin(115200, SERIAL_8N1, 5, -1);   // Aux Serial?

  Serial.println("[SYS] Objects & Hardware Initialized");

  // --- 3. START TASKS ---
  // Note: No InverseKinematic task. No Suspend calls.

  xTaskCreate(MotionSupervisor, "Motion", 4096, NULL, 5, NULL);
  xTaskCreate(WSPollingTask, "WSPoll", 4096, NULL, 4, NULL);
  xTaskCreate(NetworkManagerTask, "NetMgr", 4096, NULL, 1, NULL);

  Serial.println("[SYS] Tasks Started. Robot IDLE.");
  wifiEnabled = true;  // Signals NetworkManagerTask to start AP/Station & Listen

  // Default to Manual Mode so the supervisor is ready for Joystick data
  currentMode = MODE_MANUAL;
}

// Flag to coordinate the two tasks safely
volatile bool isNetworkReady = false;
void NetworkManagerTask(void *parameters) {
  bool isServerListening = false;

  while (1) {

    // 1. SAFETY CHECK: Is the system enabled?
    // We use a flag instead of IP string since we ARE the server now.
    if (!wifiEnabled) {
      isNetworkReady = false;
      isServerListening = false;

      if (WiFi.status() == WL_CONNECTED) {
        WiFi.disconnect();
        Serial.println("[NET] Manual Mode OFF. WiFi Disconnecting.");
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // 2. Check WiFi
    if (WiFi.status() != WL_CONNECTED) {
      isNetworkReady = false;
      isServerListening = false;

      Serial.println("[NET] Starting WiFi Access Point...");
      // Option A: Connect to Router (Station Mode)
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);

      // Wait loop
      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        vTaskDelay(pdMS_TO_TICKS(500));
        attempts++;
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[NET] WiFi Active. IP: ");
        Serial.println(WiFi.localIP());
      }
    }

    // 3. Manage Server & Clients (Only if WiFi is good)
    if (WiFi.status() == WL_CONNECTED) {

      // A. Start Listening (Once)
      if (!isServerListening) {
        server.listen(serverPort);  // Port 8000
        isServerListening = true;
        Serial.println("[NET] Server Listening on Port 8000");
      }

      // B. Check for New Connections
      // If we don't have a client, or the current one died...
      if (!isNetworkReady || !client.available()) {

        // Non-blocking check for new clients
        if (server.poll()) {
          WebsocketsClient newClient = server.accept();

          if (newClient.available()) {
            Serial.println("[NET] New Client Connected!");

            // 1. Assign to global holder
            client = newClient;

            // 2. Setup Callbacks on the CLIENT object
            client.onMessage(onMessageCallback);
            client.onEvent(onEvent);

            // 3. Enable the Poller Task
            isNetworkReady = true;
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
  }
}

void WSPollingTask(void *parameters) {
  while (1) {
    if (isNetworkReady) {
      // 1. Poll the connected client
      client.poll();

      // 2. Check health
      // If client disconnects, turn off the flag so NetworkManager searches for a new one
      if (!client.available()) {
        Serial.println("[NET] Client Disconnected.");
        isNetworkReady = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));  // Run fast
  }
}

void MotionSupervisor(void *parameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 50 Hz

  while (true) {
    // =====================================================
    // 1) PREPARE TARGETS
    // =====================================================
    WheelRotations targetW = { 0, 0, 0, 0 };
    bool shouldDrive = false;

    // We only proceed if ALL motors have reported in.
    bool allMotorsUpdated =
      (w[0].updated && w[1].updated && w[2].updated && w[3].updated);

    // =====================================================
    // MODE 1: MANUAL
    // =====================================================
    if (currentMode == MODE_MANUAL) {
      const float max_speed = 10.0f;

      targetW = mecanum_ik(
        manual_req_x * max_speed,
        manual_req_y * max_speed,
        manual_req_th * max_speed,
        WHEEL_RADIUS, HALF_LENGTH, HALF_WIDTH);
      shouldDrive = true;

      if (isOdometryInitialized) {
        Serial.println("[MAN] Clearing Odometry Sync");
        isOdometryInitialized = false;
      }
    }

    // =====================================================
    // MODE 2: POSITION CONTROL
    // =====================================================
    else if (currentMode == MODE_POSITION) {
      // ------ SYNC PHASE ------
      if (!isOdometryInitialized) {
        Serial.printf("SEEN FLAGS: %d %d %d %d\n", w[0].seen, w[1].seen, w[2].seen, w[3].seen);
        if (w[0].seen && w[1].seen && w[2].seen && w[3].seen) {
          for (int i = 0; i < 4; i++) {
            prev_w[i] = w[i].current;
            w[i].updated = false;
            w[i].seen = false;  // reset latch
          }

          isOdometryInitialized = true;
          Serial.println("[SYS] Odom Synced");
        }
      }
      // ------ CONTROL PHASE ------
      else {
        if (allMotorsUpdated) {
          updateOdometry();
          Serial.printf("[ODO] X:%.2f Y:%.2f Th:%.2f Target x:%.2f Target y:%.2f Target th:%.2f\n",
                        current_x, current_y, current_th, PositionTarget.x, PositionTarget.y, PositionTarget.th);
          // Consume flags immediately after using them
          for (int i = 0; i < 4; i++) w[i].updated = false;

          // Errors
          float dist_err = hypot(PositionTarget.x - current_x, PositionTarget.y - current_y);
          float ang_err = fabs(PositionTarget.th - current_th);

          bool isArrived = (dist_err < 0.05f && ang_err < 0.1f);

          // Waypoint dequeue
          if (isArrived) {
            PositionTarget_t nextPt;
            if (xQueueReceive(PositionBuffer, &nextPt, 0) == pdTRUE) {
              PositionTarget = nextPt;
              isArrived = false;
              Serial.printf("[SYS] Next Waypoint: %.2f %.2f\n", nextPt.x, nextPt.y);
            }
          }

          // Speed commands
          if (isArrived) {
            targetW = { 0, 0, 0, 0 };
            shouldDrive = true;
          } else {
            targetW = mecanum_to_pose(
              current_x, current_y, current_th,
              PositionTarget.x, PositionTarget.y, PositionTarget.th,
              WHEEL_RADIUS, HALF_LENGTH, HALF_WIDTH);
            shouldDrive = true;
          }
        }
      }
    }


    // =====================================================
    // 3) OUTPUT MOTOR COMMANDS (CRITICAL FIX)
    // =====================================================
    if (shouldDrive) {
      // Sign adjustments (Check your hardware wiring for this!) S,-5,5,-5,5
      targetW.m3 *= -1.0f;
      targetW.m4 *= -1.0f;

      // Normalize
      const float LIMIT = 7.0f;
      float maxVal = max(max(fabs(targetW.m1), fabs(targetW.m2)),
                         max(fabs(targetW.m3), fabs(targetW.m4)));

      if (maxVal > LIMIT) {
        float s = LIMIT / maxVal;
        targetW.m1 *= s;
        targetW.m2 *= s;
        targetW.m3 *= s;
        targetW.m4 *= s;
      }
      Serial2.write("O");
      // --- FIX: SEND TO BOTH PORTS ---
      if (xSemaphoreTake(SerialOutputMutex, 5)) {
        Serial.printf("S,%.2f,%.2f,%.2f,%.2f\n",
                      targetW.m1, targetW.m2, targetW.m3, targetW.m4);
        // Send to Bank A
        Serial1.printf("S,%.2f,%.2f,%.2f,%.2f\n",
                       targetW.m1, targetW.m2, targetW.m3, targetW.m4);
        // Send to Bank B
        Serial2.printf("S,%.2f,%.2f,%.2f,%.2f\n",
                       targetW.m1, targetW.m2, targetW.m3, targetW.m4);

        xSemaphoreGive(SerialOutputMutex);
      }
    }


    // =====================================================
    // 4) PIPELINE REQUEST FOR NEXT TICK (CRITICAL FIX)
    // =====================================================
    if (currentMode == MODE_POSITION) {
      if (xSemaphoreTake(SerialOutputMutex, 5)) {
        // Ask Bank B for data
        Serial2.write("O");
        xSemaphoreGive(SerialOutputMutex);
      }
    }

    // =====================================================
    // 5) TIMING
    // =====================================================
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void handlePacket(char *buffer, int sourcePort) {
  int id;
  float current, error, target;

  if (!isdigit(buffer[0]) && buffer[0] != '-') return;

  if (parseWheelPacketHelper(buffer, id, current, error, target)) {
    if (id >= 1 && id <= 4) {

      // --- HARDWARE ABSTRACTION ---
      // Fix the wiring signs here ONCE.
      // If Wheels 1 & 3 count NEGATIVE when moving FORWARD, invert them here.
      // (Change this line if it's actually 1 & 2 for your specific robot)
      if (id == 1 || id == 3) { 
        current = -current;
      } 

      int index = id - 1;
      w[index].current = current;
      w[index].updated = true;
      w[index].seen = true;

      Serial.printf("[FB] Port:%d ID:%d Pos: %.4f\n", sourcePort, id, current);
    }
  }
}


void loop() {
  if (Serial.available()) USBSERIAL_Handler();

  // PROCESS SERIAL 1
  while (Serial1.available()) {
    char ch = Serial1.read();
    // Accept Newline OR Null Byte
    if (ch == '\n' || ch == '\r' || ch == '\0') {
      buf1[idx1] = 0;
      if (idx1 > 0) handlePacket(buf1, 1);
      idx1 = 0;
    }
    // Prevent buffer overflow
    else if (idx1 < MAX_BUF - 1) {
      // Only append if it's a valid character (filter binary garbage)
      if (isPrintable(ch)) buf1[idx1++] = ch;
    }
  }

  // PROCESS SERIAL 2
  while (Serial2.available()) {
    char ch = Serial2.read();
    if (ch == '\n' || ch == '\r' || ch == '\0') {
      buf2[idx2] = 0;
      if (idx2 > 0) handlePacket(buf2, 2);
      idx2 = 0;
    } else if (idx2 < MAX_BUF - 1) {
      if (isPrintable(ch)) buf2[idx2++] = ch;
    }
  }
  vTaskDelay(1);
}
