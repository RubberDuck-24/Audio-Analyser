#include <Arduino.h>
#include <Audio.h>

// ─────────────────────────────────────────────
//  PCM4222EVM → Teensy 4.1  (I2S Slave RX)
//  v2 — streamer alle samples til Python UI
//
//  Teensy pins:
//    Pin 8  = DATA  ← EVM J6 pin 7
//    Pin 20 = LRCK  ← EVM J6 pin 5
//    Pin 21 = BCK   ← EVM J6 pin 3
//    GND           ← EVM J6 pin 2/4/6/8
// ─────────────────────────────────────────────

AudioInputI2Sslave   i2s_in;
AudioRecordQueue     queueLeft;
AudioRecordQueue     queueRight;

AudioConnection patchLeft (i2s_in, 0, queueLeft,  0);
AudioConnection patchRight(i2s_in, 1, queueRight, 0);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  AudioMemory(16);
  queueLeft.begin();
  queueRight.begin();
}

void loop() {
  if (queueLeft.available() > 0 && queueRight.available() > 0) {
    int16_t* L = queueLeft.readBuffer();
    int16_t* R = queueRight.readBuffer();

    for (int i = 0; i < 128; i++) {
      // Format: "L,R\n" — nemt at parse i Python
      Serial.print(L[i]);
      Serial.print(",");
      Serial.println(R[i]);
    }

    queueLeft.freeBuffer();
    queueRight.freeBuffer();
  }
}