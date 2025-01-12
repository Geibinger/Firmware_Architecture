#include <Arduino.h>

void setup() {
  setCpuFrequencyMhz(240); // Set CPU frequency to 240 MHz
  Serial.begin(115200);
  delay(1000); // Allow time for serial monitor to initialize

  const unsigned long test_iterations = 1000000;
  unsigned long CONTROL_LOOP_COUNT = test_iterations;
  volatile unsigned long i; // Volatile to prevent optimization

  // Record start time
  uint64_t start_time = esp_timer_get_time(); // Returns time in microseconds

  // Execute the loop
  for (i = 0; i < CONTROL_LOOP_COUNT; i++) {
    asm volatile ("nop");
  }

  // Record end time
  uint64_t end_time = esp_timer_get_time();

  // Calculate elapsed time in microseconds
  uint64_t elapsed_time = end_time - start_time;

  Serial.printf("Executed %lu iterations in %llu microseconds.\n", CONTROL_LOOP_COUNT, elapsed_time);

  // Calculate cycles per iteration
  const double cpu_freq_mhz = 240.0;
  double cycles_per_second = cpu_freq_mhz * 1000000.0;
  double elapsed_cycles = (elapsed_time * cycles_per_second) / 1000000.0;
  double cycles_per_iteration = elapsed_cycles / (double)CONTROL_LOOP_COUNT;

  Serial.printf("Elapsed Cycles: %.2f\n", elapsed_cycles);
  Serial.printf("Cycles per Iteration: %.2f\n", cycles_per_iteration);

  // Use this value to calculate CONTROL_LOOP_COUNT for 20 ms
  double desired_delay_sec = 0.020; // 20 ms
  double total_cycles_needed = desired_delay_sec * cycles_per_second;
  unsigned long calculated_control_loop = (unsigned long)(total_cycles_needed / cycles_per_iteration);

  Serial.printf("Calculated CONTROL_LOOP_COUNT for 20 ms: %lu\n", calculated_control_loop);

  // Blink LED to indicate completion
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(1000);
  digitalWrite(2, LOW);
  while (1); // Halt further execution
}

void loop() {
  // Empty loop
}