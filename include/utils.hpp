
#ifndef UTILS_HPP
  #define UTILS_HPP

  #include <Arduino.h>

  void checkMemoryUsage() {
    UBaseType_t controlHighWaterMark = uxTaskGetStackHighWaterMark(controlTaskHandle);
    UBaseType_t commHighWaterMark = uxTaskGetStackHighWaterMark(commTaskHandle);
    size_t freeHeap = xPortGetFreeHeapSize();
    
    Serial.print("Control Task Stack High Water Mark: ");
    Serial.println(controlHighWaterMark);
    Serial.print("Communication Task Stack High Water Mark: ");
    Serial.println(commHighWaterMark);
    Serial.print("Free Heap Size: ");
    Serial.println(freeHeap);
  }

#endif // UTILS_HPP