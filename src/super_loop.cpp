/*
  Super Loop Implementation for ESP32
  - Control Task: Time-critical, triggered every 10 ms
  - Communication Task: Non-critical, triggered every 50 ms
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
<lambda>(["checkprogsize"], [".pio/build/super_loop/firmware.elf"])
MethodWrapper(["checkprogsize"], [".pio/build/super_loop/firmware.elf"])
Advanced Memory Usage is available via "PlatformIO Home > Project Inspect"
RAM:   [=         ]   6.5% (used 21424 bytes from 327680 bytes)
Flash: [==        ]  20.3% (used 265981 bytes from 1310720 bytes)
.pio/build/super_loop/firmware.elf  :
section                                                                                size         addr
.rtc.text                                                                                 0   1074528256
.rtc.dummy                                                                                0   1073217536
.rtc.force_fast                                                                           0   1073217536
.rtc_noinit                                                                              16   1342177792
.rtc.force_slow                                                                           0   1342177808
.iram0.vectors                                                                         1027   1074266112
.iram0.text                                                                           56559   1074267140
.dram0.data                                                                           16480   1073470304
.ext_ram_noinit                                                                           0   1065353216
.noinit                                                                                   0   1073486784
.ext_ram.bss                                                                              0   1065353216
.dram0.bss                                                                             4944   1073486784
.flash.appdesc                                                                          256   1061158944
.flash.rodata                                                                         53272   1061159200
.flash.rodata_noload                                                                      0   1061212472
.flash.text                                                                          138643   1074593824
.iram0.text_end                                                                           1   1074323699
.iram0.data                                                                               0   1074323700
.iram0.bss                                                                                0   1074323700
.dram0.heap_start                                                                         0   1073491728
.xt.prop                                                                             158484            0
.xt.lit                                                                                4776            0
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
.debug_frame                                                                          64144            0
.debug_info                                                                         3095375            0
.debug_abbrev                                                                        266805            0
.debug_loc                                                                           473926            0
.debug_aranges                                                                        25624            0
.debug_ranges                                                                         66848            0
.debug_line                                                                         1164870            0
.debug_str                                                                           251986            0
Total                                                                               5846058
*/

#include <Arduino.h>

// Define GPIO pins for task indicators
const int CONTROL_PIN = 2;    // GPIO 2 for Control Task
const int COMM_PIN = 4;       // GPIO 4 for Communication Task

// Define task periods in milliseconds
const unsigned long CONTROL_TASK_PERIOD_MS = 10;    // Control Task period (10 ms)
const unsigned long COMM_TASK_PERIOD_MS = 50;       // Communication Task period (50 ms)

// Calculated Loop Counts based on calibration
const unsigned long CONTROL_LOOP_COUNT = 79458;                 // ~79,500 loops for 2 ms 
const unsigned long COMM_LOOP_COUNT = CONTROL_LOOP_COUNT * 10;  // ~795,000 loops for 20 ms

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
  setCpuFrequencyMhz(240); // Set CPU frequency to 240 MHz

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
  // Use a calibrated for-loop based on calibration

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

  for (unsigned long i = 0; i < COMM_LOOP_COUNT; i++) {
    // Simulate a minimal operation to prevent compiler optimization
    asm("nop");
  }
}
