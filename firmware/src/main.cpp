#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    // Vent på at Serial er klar (vigtigt på Teensy)
    while (!Serial && millis() < 3000);
    Serial.println("Hello World fra Teensy 4.1!");
}

void loop() {
    Serial.println("Kører...");
    delay(1000);
}