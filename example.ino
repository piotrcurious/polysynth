#include "PolySynthESP32.h"

PolySynthESP32 synth;

void setup() {
  Serial.begin(115200);

  synth.begin();
  synth.setADSR(0.01, 0.2, 0.6, 0.4);

  delay(500);

  // C major chord
  synth.noteOn(261.63); // C4
  synth.noteOn(329.63); // E4
  synth.noteOn(392.00); // G4
}

void loop() {
  delay(2000);

  synth.noteOff(261.63);
  synth.noteOff(329.63);
  synth.noteOff(392.00);

  delay(1000);

  synth.noteOn(440.00); // A4
}
