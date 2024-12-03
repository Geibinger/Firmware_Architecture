/*
  FreeRTOS-Based Implementation for ESP32
  - Control Task: Time-critical, high-priority FreeRTOS task
  - Communication Task: Non-critical, lower-priority FreeRTOS task
  - GPIO Toggling: Marks the start and end of each task for oscilloscope monitoring
*/

// Include FreeRTOS library (already included in ESP32 Arduino core)
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define GPIO pins for task indicators
const int CONTROL_PIN = 2;    // GPIO 2 for Control Task
const int COMM_PIN = 4;       // GPIO 4 for Communication Task

// Define task periods in milliseconds (modifiable for testing)
const unsigned long CONTROL_TASK_PERIOD_MS = 10;      // Control Task period (e.g., 10 ms)
const unsigned long COMM_TASK_PERIOD_MS = 50;         // Communication Task period (e.g., 50 ms)

// Task Handles (optional, for monitoring or future enhancements)
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t commTaskHandle = NULL;

// Function Prototypes
void controlTask(void *pvParameters);
void commTask(void *pvParameters);
void initializePeripherals();
void performControlOperations();
void performCommunication();

void setup() {
  // Initialize serial communication for debugging (optional)
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (only necessary for some boards)
  }

  // Initialize GPIO pins as OUTPUT and set them LOW
  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, LOW);

  pinMode(COMM_PIN, OUTPUT);
  digitalWrite(COMM_PIN, LOW);

  // Initialize peripherals (placeholders)
  initializePeripherals();

  // Create FreeRTOS tasks with appropriate priorities
  // Priority levels: Higher number -> Higher priority
  xTaskCreate(
    controlTask,                // Function that implements the task
    "Control Task",             // Text name for the task
    2048,                       // Stack size in words, not bytes
    NULL,                       // Parameter passed into the task
    2,                          // Priority at which the task is created
    &controlTaskHandle          // Pointer to the task handle
  );

  xTaskCreate(
    commTask,                   // Function that implements the task
    "Communication Task",       // Text name for the task
    2048,                       // Stack size in words, not bytes
    NULL,                       // Parameter passed into the task
    1,                          // Priority at which the task is created
    &commTaskHandle             // Pointer to the task handle
  );

  // Start the FreeRTOS scheduler (automatically called in Arduino)
  // No need to call vTaskStartScheduler() in Arduino framework
  Serial.println("FreeRTOS-Based Implementation Started");
}

void loop() {
  // Empty. FreeRTOS manages tasks independently.
}

/*
  Function: initializePeripherals
  Description: Placeholder for initializing control and communication peripherals.
*/
void initializePeripherals() {
  // Example: Initialize PWM, sensors, actuators, UART, SPI, I2C, Wi-Fi, Bluetooth, etc.
  // For this placeholder, no actual initialization is performed.
  Serial.println("Peripherals initialized.");
}

/*
  Function: controlTask
  Description: High-priority FreeRTOS task that handles the Control Task.
*/
void controlTask(void *pvParameters) {
  // Cast parameters if needed
  (void) pvParameters;

  while (1) {
    // Mark the start of the Control Task
    digitalWrite(CONTROL_PIN, HIGH);

    // Perform Control Task operations (simulated with a short delay)
    performControlOperations();

    // Mark the end of the Control Task
    digitalWrite(CONTROL_PIN, LOW);

    // Delay to maintain task period
    vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
  }
}

/*
  Function: commTask
  Description: Lower-priority FreeRTOS task that handles the Communication Task.
*/
void commTask(void *pvParameters) {
  // Cast parameters if needed
  (void) pvParameters;

  while (1) {
    // Mark the start of the Communication Task
    digitalWrite(COMM_PIN, HIGH);

    // Perform Communication Task operations (simulated with a blocking delay)
    performCommunication();

    // Mark the end of the Communication Task
    digitalWrite(COMM_PIN, LOW);

    // Delay to maintain task period
    vTaskDelay(pdMS_TO_TICKS(COMM_TASK_PERIOD_MS));
  }
}

/*
  Function: performControlOperations
  Description: Simulates control task operations.
*/
void performControlOperations() {
  // Simulate a fast control task (e.g., 1 ms)
  // For more precise timing, use delayMicroseconds() if necessary
  delayMicroseconds(1000); // 1 ms
}

/*
  Function: performCommunication
  Description: Simulates communication task operations.
*/
void performCommunication() {
  // Simulate a blocking communication task (e.g., 10 ms)
  delay(10); // 10 ms
}
