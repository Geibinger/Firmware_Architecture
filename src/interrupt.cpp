/*
  Interrupt-Driven Implementation for ESP32
  - Control Task: Time-critical, handled by a high-priority ISR
  - Communication Task: Non-critical, handled in the main loop
  - GPIO Toggling: Marks the start and end of each task for oscilloscope monitoring
*/
#include <Arduino.h>

// Define GPIO pins for task indicators
const int CONTROL_PIN = 2;    // GPIO 2 for Control Task
const int COMM_PIN = 4;       // GPIO 4 for Communication Task

// Define task periods in milliseconds (modifiable for testing)
const unsigned long CONTROL_ISR_PERIOD_MS = 10;   // ISR triggers every 10 ms
const unsigned long COMM_TASK_PERIOD_MS = 50;      // Communication Task period (e.g., 50 ms)

// Hardware Timer Variables
hw_timer_t * controlTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Variables to track task execution timing
volatile bool commTaskFlag = false;

// Function Prototypes
void IRAM_ATTR controlISR();
void setupHardwareTimer();
void initializeControl();
void initializeCommunication();
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
  initializeControl();
  initializeCommunication();

  // Setup Hardware Timer to trigger ISR
  setupHardwareTimer();

  Serial.println("Interrupt-Driven Implementation Started");
}

void loop() {
  // Execute Communication Task when flag is set by timer interrupt
  if (commTaskFlag) {
    commTaskFlag = false; // Reset the flag

    // Mark the start of the Communication Task
    digitalWrite(COMM_PIN, HIGH);

    // Perform Communication Task operations (simulated with a delay)
    performCommunication();

    // Mark the end of the Communication Task
    digitalWrite(COMM_PIN, LOW);
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
  Function: setupHardwareTimer
  Description: Configures the ESP32 hardware timer to trigger the Control ISR at a defined frequency.
*/
void setupHardwareTimer() {
  // Create a hardware timer (Timer 0)
  controlTimer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1 tick = 1 µs)

  // Attach the Control ISR to the timer
  timerAttachInterrupt(controlTimer, &controlISR, true);

  // Set the timer to trigger every CONTROL_ISR_PERIOD_MS milliseconds
  timerAlarmWrite(controlTimer, CONTROL_ISR_PERIOD_MS * 1000, true); // Convert ms to µs

  // Start the timer
  timerAlarmEnable(controlTimer);

  Serial.println("Hardware Timer configured for Control ISR.");
}

/*
  Function: controlISR
  Description: Interrupt Service Routine for the Control Task.
               Executes every CONTROL_ISR_PERIOD_MS milliseconds.
*/
void IRAM_ATTR controlISR() {
  // Mark the start of the Control Task
  digitalWrite(CONTROL_PIN, HIGH);

  // Perform Control Task operations (simulated with a short delay)
  // Note: Avoid using delay() or other blocking functions in ISRs
  performControlOperations();

  // Mark the end of the Control Task
  digitalWrite(CONTROL_PIN, LOW);
}

/*
  Function: performControlOperations
  Description: Placeholder for control task operations within the ISR.
*/
void performControlOperations() {
  // Simulate a fast control task (e.g., 1 ms)
  // Since delay() is not allowed in ISRs, perform minimal operations
  // For simulation, use a busy-wait loop
  unsigned long start = micros();
  while (micros() - start < 1000) {
    // Busy-wait for 1 ms
  }
}

/*
  Function: performCommunication
  Description: Placeholder for communication task operations.
*/
void performCommunication() {
  // Simulate a blocking communication task (e.g., 10 ms)
  delay(10); // 10 ms
}
