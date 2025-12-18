#ifndef NAVIA_INTERMEDIARY.h
#define NAVIA_INTERMEDIARY_h

char echoBuf[128];
int echoIdx = 0;
/*
const char* ssid = "mikerasp";   //Enter SSID
const char* password = "abc123456";  //Enter Password
*/
const char* ssid = "INFINITUMD0E4";   //Enter SSID
const char* password = "fY5cb5dANU";  //Enter Password
/*
const char *ssid = "TP-LINK_A07C";
const char *password = "59033950";
*/
//WEBSOCKETS THINGS
#include <WiFi.h>
#include <ArduinoWebsockets.h>
using namespace websockets;
//WebsocketsClient ws;
WebsocketsServer server;
WebsocketsClient client; // This holds the currently connected controller
bool wifiEnabled = false; // Logic flag to turn radio on/off
// Configuración del servidor WebSocket
const char* serverAddress = "10.106.180.121";  // Dirección IP del servidor Flask
const int serverPort = 8000;

static TaskHandle_t Manual_Control = NULL;
static TaskHandle_t Motion_Supervisor = NULL;
static TaskHandle_t Inverse_Kinematic = NULL;

// --- Global State ---
enum RobotMode { MODE_IDLE, MODE_MANUAL, MODE_POSITION };
volatile RobotMode currentMode = MODE_IDLE;

// Buffers for passing data between tasks
String wsServerIP = "";

static QueueHandle_t PositionBuffer;
enum { MSG_QUEUE_LEN = 10 };  // Number of slots in message queue
SemaphoreHandle_t SerialOutputMutex;

// Define the TYPE first
typedef struct {
  float x;
  float y;
  float th;
} PositionTarget_t;

// Then create the GLOBAL INSTANCE
PositionTarget_t PositionTarget;

// Required by MecanumKinematics updateOdometry
float prev_w[4] = {0,0,0,0};

// Required by Manual_Control callback
unsigned long lastManualPacketTime = 0;

// --- Globals for Odometry (Where am I?) ---
// We use double for better precision over time
double current_x = 0.0;
double current_y = 0.0;
double current_th = 0.0;


// --- Shared Globals ---
// These hold the raw normalized values (-1.0 to 1.0) from the Joystick/Websocket
volatile float manual_req_x = 0.0;
volatile float manual_req_y = 0.0;
volatile float manual_req_th = 0.0;

// Add this global flag near the top of your file
bool isOdometryInitialized = false;

float WHEEL_RADIUS = 0.0485;  // radio de rueda (m)
float HALF_LENGTH = 1.75;   // mitad de la longitud del robot (m)
float HALF_WIDTH = 2.25;   // mitad del ancho del robot (m)

// PROTOTYPES
void USBSERIAL_Handler();
bool connectWifi();
void disconnectWifi();
void handleCommand(const char* cmd);
void Serial1Handler();
void Serial2Handler();
void onMessageCallback(WebsocketsMessage message);

// Serial Buffers
#define MAX_BUF 64
char buf1[MAX_BUF]; // Buffer for Serial1
int idx1 = 0;

char buf2[MAX_BUF]; // Buffer for Serial2
int idx2 = 0;


struct WheelData {
  float current;
  float error;
  float target;
  bool updated;   // per-tick update
  bool seen;      // latched since reset
};

WheelData w[4];

#include "MecanumKinematics.h"
#include "Communication.h"
#include "ManualControl.h"
#endif