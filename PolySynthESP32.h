#pragma once
#include <Arduino.h>
#include "driver/i2s.h"

// ================= CONFIG =================
#define SYNTH_MAX_VOICES     8
#define WAVETABLE_SIZE      256
#define SAMPLE_RATE         48000
#define CHUNK_FRAMES        256
#define DITHER_BITS         5
#define SCALE_MICROHZ       1000000ULL
// =========================================

// ========== Envelope States ==========
enum EnvStage {
  ENV_OFF = 0,
  ENV_ATTACK,
  ENV_DECAY,
  ENV_SUSTAIN,
  ENV_RELEASE
};

// ========== ADSR Envelope ==========
struct ADSR {
  float attack  = 0.01f;
  float decay   = 0.10f;
  float sustain = 0.70f;
  float release = 0.20f;
};

// ========== Voice ==========
struct Voice {
  bool active = false;

  float freqHz = 440.0f;
  float velocity = 1.0f;

  // Bresenham phase stepping
  uint32_t phase_idx = 0;
  uint64_t err = 0;

  uint64_t step_numer = 0;
  uint64_t q = 0;
  uint64_t rem = 0;

  // Envelope
  EnvStage stage = ENV_OFF;
  float envLevel = 0.0f;
  uint32_t envSamples = 0;

  ADSR adsr;
};

// ================= Synth Class =================
class PolySynthESP32 {
public:
  void begin();
  void noteOn(float freqHz, float velocity = 1.0f);
  void noteOff(float freqHz);
  void setADSR(float A, float D, float S, float R);

private:
  static int16_t wavetable[WAVETABLE_SIZE];
  static Voice voices[SYNTH_MAX_VOICES];

  static TaskHandle_t streamTask;
  static uint32_t rngState;

  static void buildWavetable();
  static void configureI2S();
  static void streamTaskFn(void *arg);

  static inline uint32_t xorshift32();
  static inline uint16_t waveToDAC(int32_t sample);

  static void updateEnvelope(Voice &v);
  static Voice* allocateVoice();
};
