#include <FreeRTOS.h>
#include <queue.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_timer.h>

// Pin declarations
const volatile int TRIG_PIN = 12;      // Trigger pin for Ultrasonic sensor
const volatile int ECHO_PIN = 13;      // Echo pin for Ultrasonic sensor
const volatile int LED_OBSTACLE = 2;   // Red LED for obstacle detection
const volatile int LED_FORWARD = 4;    // Green LED for forward motion
const volatile int LED_AVOIDANCE = 5;  // Yellow LED for obstacle avoidance
const volatile int CS_PIN = 15;        // Chip Select (CS) for SD Card module
const volatile int SCK_PIN = 18;       // SPI Clock for SD Card module
const volatile int MOSI_PIN = 26;      // SPI Data (MOSI) for SD Card module
const volatile int MISO_PIN = 19;      // SPI Data (MISO) for SD Card module
const volatile int SDA_PIN = 36;       // I2C Data (SDA) for MPU6050
const volatile int SCL_PIN = 35;       // I2C Clock (SCL) for MPU6050

// FreeRTOS Queue
QueueHandle_t obstacleQueue;

// MPU6050 Sensor
Adafruit_MPU6050 mpu;

// Ultrasonic measurement variables
volatile long pulseDuration = 0;
volatile int distance = 0;

// Interrupt Service Routine (ISR) for Echo pin
void IRAM_ATTR handleEchoPulse() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    // Start timing the pulse
    pulseDuration = micros();
  } else {
    // Calculate pulse duration
    pulseDuration = micros() - pulseDuration;
    distance = pulseDuration * 0.034 / 2; // Convert to cm
    if (distance < 50) { // Send to queue if obstacle detected
      xQueueSendFromISR(obstacleQueue, (const void*)&distance, NULL); // Fixed type casting
    }
  }
}

// Timer callback for TRIG_PIN pulse
void trigPulseCallback(void* arg) {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
}

// Data logging task
void dataLoggingTask(void *pvParameters) {
  while (true) {
    File logFile = SD.open("/log.txt", FILE_WRITE);
    if (logFile) {
      logFile.println("Logging data...");
      logFile.close();
    } else {
      Serial.println("Failed to open log file!");
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // Log data every 500ms
  }
}

// Navigation task
void navigationTask(void *pvParameters) {
  int obstacleDistance;
  while (true) {
    if (xQueueReceive(obstacleQueue, &obstacleDistance, portMAX_DELAY)) {
      // Obstacle detected, simulate avoidance
      digitalWrite(LED_AVOIDANCE, HIGH);
      delay(500);
      digitalWrite(LED_AVOIDANCE, LOW);
    } else {
      // No obstacle, move forward
      digitalWrite(LED_FORWARD, HIGH);
      delay(1000);
      digitalWrite(LED_FORWARD, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Check queue every 100ms
  }
}

// Setup function
void setup() {
  Serial.begin(115200);

  // Pin setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_OBSTACLE, OUTPUT);
  pinMode(LED_FORWARD, OUTPUT);
  pinMode(LED_AVOIDANCE, OUTPUT);

  // Attach interrupt to Echo pin
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), handleEchoPulse, CHANGE);

  // Initialize SD card
  if (!SD.begin(CS_PIN)) {
    Serial.println("SD Card initialization failed! Check wiring, format, or module voltage.");
    while (true); // Halt execution
  }
  Serial.println("SD Card initialized.");

  // Initialize MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (true); // Halt if MPU6050 fails
  }
  Serial.println("MPU6050 initialized.");

  // Create FreeRTOS queue
  obstacleQueue = xQueueCreate(10, sizeof(int));

  // Create FreeRTOS tasks
  xTaskCreate(navigationTask, "Navigation", 1024, NULL, 1, NULL);
  xTaskCreate(dataLoggingTask, "DataLogging", 1024, NULL, 1, NULL);

  // Create periodic timer for TRIG_PIN pulses
  esp_timer_create_args_t trigTimerArgs = {
    .callback = &trigPulseCallback,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "TRIG Pulse Timer"
  };
  esp_timer_handle_t trigTimer;
  esp_timer_create(&trigTimerArgs, &trigTimer);
  esp_timer_start_periodic(trigTimer, 100000); // Trigger every 100ms
}

// Loop function (unused with FreeRTOS)
void loop() {
  // Empty
}
