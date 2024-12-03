/*
  Super Loop Implementation for ESP32
  - Control Task: Time-critical, triggered every 10 ms
  - Communication Task: Non-critical, triggered every 50 ms
  - GPIO Toggling: Marks the start and end of each task for oscilloscope monitoring
*/

#include <Arduino.h>

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
// Based on empirical testing: 40,000 iterations ≈ 1 ms
// Therefore, for 20 ms: 40,000 * 20 = 800,000 iterations
const unsigned long COMM_LOOP_COUNT = COMM_TASK_DURATION_MS * 40000; // 20 ms * 40,000 iterations/ms = 800,000

// Control Task Loop Count
// Based on empirical testing: 40,000 iterations ≈ 1 ms
// Therefore, for 2 ms: 40,000 * 2 = 80,000 iterations
const unsigned long CONTROL_LOOP_COUNT = CONTROL_TASK_DURATION_MS * 40000; // 2 ms * 40,000 iterations/ms = 80,000

// Variables to track task timing
unsigned long previousControlMillis = 0;
unsigned long previousCommMillis = 0;

// Flag to indicate Communication Task is running (optional, for synchronization if needed)
bool commTaskRunning = false;

// Function Prototypes
void initializeControl();
void initializeCommunication();
void performControlOperations();
void performCommunication();

void setup() {
  setCpuFrequencyMhz(240);

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
  initializeControl();
  initializeCommunication();

  Serial.println("Super Loop Implementation Started");
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if it's time to execute the Control Task
  if (currentMillis - previousControlMillis >= CONTROL_TASK_PERIOD_MS) {
    previousControlMillis = currentMillis;

    // Execute Control Task
    digitalWrite(CONTROL_PIN, HIGH);
    performControlOperations();
    digitalWrite(CONTROL_PIN, LOW);
  }

  // Check if it's time to execute the Communication Task
  if (currentMillis - previousCommMillis >= COMM_TASK_PERIOD_MS) {
    previousCommMillis = currentMillis;
    commTaskRunning = true;

    // Start Communication Task
    digitalWrite(COMM_PIN, HIGH);

    // Perform Communication Task operations
    performCommunication();

    // End Communication Task
    digitalWrite(COMM_PIN, LOW);
    commTaskRunning = false;
  }

  // (Optional) Other non-blocking operations can be added here
}

/*
  Function: initializeControl
  Description: Placeholder for initializing control-related peripherals.
*/
void initializeControl() {
  // Example: Initialize PWM, sensors, actuators, etc.
  // For this placeholder, no actual initialization is performed.
  Serial.println("Control peripherals initialized.");
}

/*
  Function: initializeCommunication
  Description: Placeholder for initializing communication-related peripherals.
*/
void initializeCommunication() {
  // Example: Initialize UART, SPI, I2C, Wi-Fi, Bluetooth, etc.
  // For this placeholder, no actual initialization is performed.
  Serial.println("Communication peripherals initialized.");
}

/*
  Function: performControlOperations
  Description: Simulates the Control Task operations by executing a calibrated for-loop.
*/
void performControlOperations() {
  // Simulate a fast control task (e.g., 2 ms)
  // Use a calibrated for-loop based on empirical testing

  for (unsigned long i = 0; i < CONTROL_LOOP_COUNT; i++) {
    // Simulate a minimal operation to prevent compiler optimization
    asm("nop");
  }
}

/*
  Function: performCommunication
  Description: Simulates the Communication Task operations by executing a calibrated for-loop.
*/
void performCommunication() {
  // Simulate a communication task operation using a calibrated for-loop

  for (unsigned long i = 0; i < 960000; i++) {
    // Simulate a minimal operation to prevent compiler optimization
    asm("nop");
  }
}
