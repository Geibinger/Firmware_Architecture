/*
  Adjusted Interrupt-Driven Implementation for ESP32
  - Control Task: Time-critical, handled by a high-priority ISR
  - Communication Task: Non-critical, handled in the main loop with precise timing
  - GPIO Toggling: Marks the start and end of each task for oscilloscope monitoring

? Verbose compilation info:
CONFIGURATION: https://docs.platformio.org/page/boards/espressif32/esp32dev.html
PLATFORM: Espressif 32 (6.6.0) > Espressif ESP32 Dev Module
HARDWARE: ESP32 240MHz, 320KB RAM, 4MB Flash
DEBUG: Current (cmsis-dap) External (cmsis-dap, esp-bridge, esp-prog, iot-bus-jtag, jlink, minimodule, olimex-arm-usb-ocd, olimex-arm-usb-ocd-h, olimex-arm-usb-tiny-h, olimex-jtag-tiny, tumpa)
PACKAGES: 
 - framework-arduinoespressif32 @ 3.20014.231204 (2.0.14) 
 - tool-esptoolpy @ 1.40501.0 (4.5.1) 
 - toolchain-xtensa-esp32 @ 8.4.0+2021r2-patch5
LDF: Library Dependency Finder -> https://bit.ly/configure-pio-ldf
LDF Modes: Finder ~ chain, Compatibility ~ soft
Found 33 compatible libraries
Scanning dependencies...
No dependencies
Building in release mode
<lambda>(["checkprogsize"], [".pio/build/interrupt/firmware.elf"])
MethodWrapper(["checkprogsize"], [".pio/build/interrupt/firmware.elf"])
Advanced Memory Usage is available via "PlatformIO Home > Project Inspect"
RAM:   [=         ]   6.5% (used 21456 bytes from 327680 bytes)
Flash: [==        ]  20.7% (used 271909 bytes from 1310720 bytes)
.pio/build/interrupt/firmware.elf  :
section                                                                                size         addr
.rtc.text                                                                                 0   1074528256
.rtc.dummy                                                                                0   1073217536
.rtc.force_fast                                                                           0   1073217536
.rtc_noinit                                                                              16   1342177792
.rtc.force_slow                                                                           0   1342177808
.iram0.vectors                                                                         1027   1074266112
.iram0.text                                                                           56847   1074267140
.dram0.data                                                                           16496   1073470304
.ext_ram_noinit                                                                           0   1065353216
.noinit                                                                                   0   1073486800
.ext_ram.bss                                                                              0   1065353216
.dram0.bss                                                                             4960   1073486800
.flash.appdesc                                                                          256   1061158944
.flash.rodata                                                                         54440   1061159200
.flash.rodata_noload                                                                      0   1061213640
.flash.text                                                                          143099   1074593824
.iram0.text_end                                                                           1   1074323987
.iram0.data                                                                               0   1074323988
.iram0.bss                                                                                0   1074323988
.dram0.heap_start                                                                         0   1073491760
.xt.prop                                                                             161604            0
.xt.lit                                                                                4960            0
.xtensa.info                                                                             56            0
.comment                                                                                166            0
.xt.prop._ZN14HardwareSerial9readBytesEPcj                                               36            0
.xt.prop._ZNSt14_Function_baseD2Ev                                                       36            0
.xt.prop._ZTV14HardwareSerial                                                            12            0
.xt.lit._ZN5Print5writeEPKc                                                               0            0
.xt.prop._ZN5Print5writeEPKc                                                             48            0
.xt.lit._ZN6String4initEv                                                                 0            0
.xt.prop._ZN6String4initEv                                                               36            0
.xt.prop._ZNK6String3lenEv                                                               60            0
.xt.prop._ZN6String6setLenEi                                                             72            0
.xt.lit._ZN3nvs4Lock4initEv                                                               0            0
.xt.prop._ZN3nvs4Lock4initEv                                                             72            0
.xt.lit._ZN14intrusive_listIN3nvs7Storage14NamespaceEntryEE17clearAndFreeNodesEv          0            0
.xt.prop._ZN14intrusive_listIN3nvs7Storage14NamespaceEntryEE17clearAndFreeNodesEv       108            0
.xt.prop._ZN14intrusive_listIN3nvs7Storage14NamespaceEntryEE9push_backEPS2_              60            0
.xt.lit._ZN3nvs12NVSPartitionD5Ev                                                         0            0
.xt.prop._ZN3nvs12NVSPartitionD5Ev                                                        0            0
.xt.prop._ZN3nvs12NVSPartitionD2Ev                                                       36            0
.xt.prop._ZN3nvs12NVSPartitionD0Ev                                                       36            0
.xt.prop._ZTVN3nvs12NVSPartitionE                                                        12            0
.xt.lit._ZN3nvs19NVSPartitionManagerD5Ev                                                  0            0
.xt.prop._ZN3nvs19NVSPartitionManagerD5Ev                                                 0            0
.xt.prop._ZN3nvs19NVSPartitionManagerD2Ev                                                36            0
.xt.prop._ZN3nvs19NVSPartitionManagerD0Ev                                                36            0
.xt.prop._ZN14intrusive_listIN3nvs12NVSPartitionEE5eraseENS2_8iteratorE                  84            0
.xt.prop._ZTVN3nvs19NVSPartitionManagerE                                                 12            0
.xt.prop._ZN14intrusive_listIN3nvs8HashList13HashListBlockEE5eraseENS3_8iteratorE        84            0
.xt.lit._ZN3nvs4ItemC5EhNS_8ItemTypeEhPKch                                                0            0
.xt.prop._ZN3nvs4ItemC5EhNS_8ItemTypeEhPKch                                               0            0
.xt.prop._ZN3nvs20isVariableLengthTypeENS_8ItemTypeE                                     48            0
.xt.prop._ZN3nvs4ItemC2EhNS_8ItemTypeEhPKch                                              60            0
.xt.prop._ZNK19CompressedEnumTableIN3nvs4Page10EntryStateELj2ELj126EE3getEjPS2_          48            0
.xt.lit._ZN14intrusive_listIN3nvs4PageEE5clearEv                                          0            0
.xt.prop._ZN14intrusive_listIN3nvs4PageEE5clearEv                                        60            0
.xt.prop._ZN14intrusive_listIN3nvs4PageEE9push_backEPS1_                                 60            0
.xt.prop._ZN14intrusive_listIN3nvs4PageEE5eraseENS2_8iteratorE                           84            0
.xt.prop._ZTISt9exception                                                                12            0
.xt.prop._ZTISt9bad_alloc                                                                12            0
.xt.prop._ZTVN10__cxxabiv120__si_class_type_infoE                                        12            0
.xt.prop._ZTISt17bad_function_call                                                       12            0
.xt.prop._ZTVSt17bad_function_call                                                       12            0
.xt.prop._ZTVN10__cxxabiv117__class_type_infoE                                           12            0
.xt.lit._ZNK9__gnu_cxx24__concurrence_lock_error4whatEv                                   8            0
.xt.lit._ZNK9__gnu_cxx26__concurrence_unlock_error4whatEv                                 8            0
.xt.lit._ZN9__gnu_cxx24__concurrence_lock_errorD5Ev                                       8            0
.xt.lit._ZN9__gnu_cxx26__concurrence_unlock_errorD5Ev                                     8            0
.xt.lit._ZN9__gnu_cxx7__mutex4lockEv                                                      8            0
.xt.lit._ZN9__gnu_cxx13__scoped_lockD5Ev                                                  8            0
.xt.prop._ZNK9__gnu_cxx24__concurrence_lock_error4whatEv                                 48            0
.xt.prop._ZNK9__gnu_cxx26__concurrence_unlock_error4whatEv                               48            0
.xt.prop._ZN9__gnu_cxx24__concurrence_lock_errorD5Ev                                     12            0
.xt.prop._ZN9__gnu_cxx26__concurrence_unlock_errorD5Ev                                   12            0
.xt.prop._ZN9__gnu_cxx7__mutex4lockEv                                                    48            0
.xt.prop._ZN9__gnu_cxx13__scoped_lockD5Ev                                                12            0
.xt.prop._ZN9__gnu_cxx24__concurrence_lock_errorD2Ev                                     36            0
.xt.prop._ZN9__gnu_cxx26__concurrence_unlock_errorD2Ev                                   36            0
.xt.prop._ZN9__gnu_cxx24__concurrence_lock_errorD0Ev                                     36            0
.xt.prop._ZN9__gnu_cxx26__concurrence_unlock_errorD0Ev                                   36            0
.xt.prop._ZN9__gnu_cxx13__scoped_lockD2Ev                                                72            0
.xt.prop._ZTIN9__gnu_cxx24__concurrence_lock_errorE                                      12            0
.xt.prop._ZTIN9__gnu_cxx26__concurrence_unlock_errorE                                    12            0
.xt.prop._ZTVN9__gnu_cxx24__concurrence_lock_errorE                                      12            0
.xt.prop._ZTVN9__gnu_cxx26__concurrence_unlock_errorE                                    12            0
.debug_frame                                                                          65920            0
.debug_info                                                                         3159205            0
.debug_abbrev                                                                        271014            0
.debug_loc                                                                           487754            0
.debug_aranges                                                                        26296            0
.debug_ranges                                                                         69920            0
.debug_line                                                                         1195228            0
.debug_str                                                                           255118            0
Total                                                                               5976183
*/

#include <Arduino.h>

// Define GPIO pins for task indicators
const int CONTROL_PIN = 2;    // GPIO 2 for Control Task
const int COMM_PIN = 4;       // GPIO 4 for Communication Task

// Define task periods in milliseconds
const unsigned long CONTROL_ISR_PERIOD_MS = 10;   // ISR triggers every 10 ms
const unsigned long COMM_TASK_PERIOD_MS = 50;      // Communication Task period (50 ms)

const unsigned long CONTROL_LOOP_COUNT = 79458;                 // ~79,500 loops for 2 ms 
const unsigned long COMM_LOOP_COUNT = CONTROL_LOOP_COUNT * 10;  // ~795,000 loops for 20 ms

// Hardware Timer Variables
hw_timer_t * controlTimer = NULL;

// Function Prototypes
void IRAM_ATTR controlISR();
void setupHardwareTimer();
void initializeControl();
void initializeCommunication();
void performControlOperations();
void performCommunication();

// Tracking variables
unsigned long previousCommMillis = 0;
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
  initializeControl();
  initializeCommunication();

  // Setup Hardware Timer to trigger ISR
  setupHardwareTimer();

  Serial.println("Interrupt-Driven Implementation Started");
}

void loop() {
  unsigned long currentMillis = millis();

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
  // If Communication Task is active, set COMM_PIN low temporarily
  if (commTaskRunning) {
    digitalWrite(COMM_PIN, LOW);
  }

  // Mark the start of the Control Task
  digitalWrite(CONTROL_PIN, HIGH);

  // Perform Control Task operations (simulated with a short delay)
  performControlOperations();

  // Mark the end of the Control Task
  digitalWrite(CONTROL_PIN, LOW);

  // If Communication Task is still active, set COMM_PIN high again
  if (commTaskRunning) {
    digitalWrite(COMM_PIN, HIGH);
  }
}

/*
  Function: performControlOperations
  Description: Placeholder for control task operations within the ISR.
*/
void performControlOperations() {
  // Simulate a fast control task (e.g., 2 ms)
  // Use a calibrated for-loop based on calibration

  for (unsigned long i = 0; i < CONTROL_LOOP_COUNT; i++) {
    // Simulate a minimal operation to prevent compiler optimization
    asm("nop");
  }
}

/*
  Function: performCommunication
  Description: Placeholder for communication task operations.
*/
void performCommunication() {
  // Simulate a communication task operation using a calibrated for-loop

  for (unsigned long i = 0; i < COMM_LOOP_COUNT; i++) {
    // Simulate a minimal operation to prevent compiler optimization
    asm("nop");
  }
}
