/*
  Enhanced FreeRTOS-Based Implementation for ESP32
  - Control Task: Time-critical, high-priority FreeRTOS task
  - Communication Task: Non-critical, lower-priority FreeRTOS task
  - GPIO Toggling: Marks the start and end of each task for oscilloscope monitoring
  - Boolean Flag: Indicates if Communication Task is running to manage COMM_PIN state
*/

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define GPIO pins for task indicators
const int CONTROL_PIN = 2;    // GPIO 2 for Control Task
const int COMM_PIN = 4;       // GPIO 4 for Communication Task

// Define task periods in milliseconds
const unsigned long CONTROL_TASK_PERIOD_MS = 10;    // Control Task period (10 ms)
const unsigned long COMM_TASK_PERIOD_MS = 50;       // Communication Task period (50 ms)

// Control Task Duration
const unsigned long CONTROL_TASK_DURATION_MS = 2;    // Simulated Control Task duration (2 ms)

// Communication Task Duration
const unsigned long COMM_TASK_DURATION_MS = 20;      // Communication Task duration (20 ms)

// Communication Task Loop Count
const unsigned long COMM_LOOP_COUNT = COMM_TASK_DURATION_MS * 40000; // 20 ms * 40,000 iterations/ms = 800,000

// Control Task Loop Count
const unsigned long CONTROL_LOOP_COUNT = CONTROL_TASK_DURATION_MS * 40000; // 2 ms * 40,000 iterations/ms = 80,000

// Task Handles (optional, for monitoring or future enhancements)
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t commTaskHandle = NULL;

// Function Prototypes
void controlTask(void *pvParameters);
void commTask(void *pvParameters);
void initializePeripherals();

// Volatile boolean indicating if Communication Task is running
volatile bool commTaskRunning = false;

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
    2048,                       // Stack size in words (~8 KB)
    NULL,                       // Parameter passed into the task
    2,                          // Priority at which the task is created
    &controlTaskHandle          // Pointer to the task handle
  );

  xTaskCreate(
    commTask,                   // Function that implements the task
    "Communication Task",       // Text name for the task
    4096,                       // Stack size in words (~16 KB) - increased for longer loop
    NULL,                       // Parameter passed into the task
    1,                          // Priority at which the task is created
    &commTaskHandle             // Pointer to the task handle
  );

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

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    // Check if Communication Task is running
    if (commTaskRunning) {
      // Set COMM_PIN low to indicate Control Task is using processing power
      digitalWrite(COMM_PIN, LOW);
    }

    // Mark the start of the Control Task
    digitalWrite(CONTROL_PIN, HIGH);

    // Perform Control Task operations (simulated with a calibrated for-loop)
    for (unsigned long i = 0; i < CONTROL_LOOP_COUNT; i++) {
      // Simulate a minimal operation to prevent compiler optimization
      asm("nop");
    }

    // Mark the end of the Control Task
    digitalWrite(CONTROL_PIN, LOW);

    if (commTaskRunning) {
      // Restore COMM_PIN high if Communication Task is still running
      digitalWrite(COMM_PIN, HIGH);
    }

    // Delay to maintain task period
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
  }
}

/*
  Function: commTask
  Description: Lower-priority FreeRTOS task that handles the Communication Task.
*/
void commTask(void *pvParameters) {
  // Cast parameters if needed
  (void) pvParameters;

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    // Mark the start of the Communication Task
    commTaskRunning = true;
    digitalWrite(COMM_PIN, HIGH);

    // Perform Communication Task operations (simulated with a calibrated for-loop)
    for (unsigned long i = 0; i < COMM_LOOP_COUNT; i++) {
      // Simulate a minimal operation to prevent compiler optimization
      asm("nop");
    }

    // Mark the end of the Communication Task
    digitalWrite(COMM_PIN, LOW);
    commTaskRunning = false;

    // Delay to maintain task period
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(COMM_TASK_PERIOD_MS));
  }
}
