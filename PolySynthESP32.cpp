#include "PolySynthESP32.h"

// ===== Static Storage =====
int16_t PolySynthESP32::wavetable[WAVETABLE_SIZE];
Voice   PolySynthESP32::voices[SYNTH_MAX_VOICES];

TaskHandle_t PolySynthESP32::streamTask = nullptr;
uint32_t PolySynthESP32::rngState = 0x12345678;

// ================= Utility =================
inline uint32_t PolySynthESP32::xorshift32() {
  uint32_t x = rngState;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  rngState = x;
  return x;
}

inline uint16_t PolySynthESP32::waveToDAC(int32_t sample16) {
  uint32_t u = (uint32_t)(sample16 + 32768);
  uint8_t dac8 = u >> 8;
  return (uint16_t)(dac8 << 8);
}

// ================= Wavetable =================
void PolySynthESP32::buildWavetable() {
  for (int i = 0; i < WAVETABLE_SIZE; i++) {
    float s = sinf(2.0f * PI * i / WAVETABLE_SIZE);
    wavetable[i] = (int16_t)(s * 32767);
  }
}

// ================= I2S Setup =================
void PolySynthESP32::configureI2S() {
  i2s_driver_uninstall(I2S_NUM_0);

  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER |
                         I2S_MODE_TX |
                         I2S_MODE_DAC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);

  i2s_pin_config_t pins{};
  i2s_set_pin(I2S_NUM_0, &pins);

  i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

// ================= Envelope Update =================
void PolySynthESP32::updateEnvelope(Voice &v) {
  float sr = SAMPLE_RATE;

  switch (v.stage) {

    case ENV_ATTACK:
      v.envLevel += 1.0f / (v.adsr.attack * sr);
      if (v.envLevel >= 1.0f) {
        v.envLevel = 1.0f;
        v.stage = ENV_DECAY;
      }
      break;

    case ENV_DECAY:
      v.envLevel -= (1.0f - v.adsr.sustain) / (v.adsr.decay * sr);
      if (v.envLevel <= v.adsr.sustain) {
        v.envLevel = v.adsr.sustain;
        v.stage = ENV_SUSTAIN;
      }
      break;

    case ENV_SUSTAIN:
      // hold
      break;

    case ENV_RELEASE:
      v.envLevel -= v.adsr.sustain / (v.adsr.release * sr);
      if (v.envLevel <= 0.0f) {
        v.envLevel = 0.0f;
        v.stage = ENV_OFF;
        v.active = false;
      }
      break;

    default:
      break;
  }
}

// ================= Voice Allocation =================
Voice* PolySynthESP32::allocateVoice() {
  for (auto &v : voices) {
    if (!v.active) return &v;
  }
  return &voices[0]; // steal oldest
}

// ================= Public API =================
void PolySynthESP32::begin() {
  buildWavetable();
  configureI2S();

  xTaskCreatePinnedToCore(
    streamTaskFn,
    "synth_stream",
    8192,
    nullptr,
    5,
    &streamTask,
    1
  );
}

void PolySynthESP32::setADSR(float A,float D,float S,float R) {
  for (auto &v : voices) {
    v.adsr.attack = A;
    v.adsr.decay = D;
    v.adsr.sustain = S;
    v.adsr.release = R;
  }
}

void PolySynthESP32::noteOn(float freqHz, float vel) {
  Voice *v = allocateVoice();
  v->freqHz = freqHz;
  v->velocity = vel;

  v->active = true;
  v->stage = ENV_ATTACK;
  v->envLevel = 0.0f;

  uint64_t denom = SAMPLE_RATE * SCALE_MICROHZ;
  uint64_t freq_micro = (uint64_t)(freqHz * SCALE_MICROHZ);

  v->step_numer = WAVETABLE_SIZE * freq_micro;
  v->q = v->step_numer / denom;
  v->rem = v->step_numer % denom;
}

void PolySynthESP32::noteOff(float freqHz) {
  for (auto &v : voices) {
    if (v.active && fabs(v.freqHz - freqHz) < 0.1f) {
      v.stage = ENV_RELEASE;
    }
  }
}

// ================= Audio Streaming =================
void PolySynthESP32::streamTaskFn(void *arg) {

  uint16_t buffer[CHUNK_FRAMES * 2];
  uint64_t denom = SAMPLE_RATE * SCALE_MICROHZ;
  uint32_t mask = (1u << DITHER_BITS) - 1;

  while (true) {

    for (int n = 0; n < CHUNK_FRAMES; n++) {

      int32_t mix = 0;

      for (auto &v : voices) {
        if (!v.active) continue;

        // Bresenham stepping
        v.phase_idx += v.q;
        v.err += v.rem;

        if (v.err >= denom) {
          v.phase_idx++;
          v.err -= denom;
        }

        v.phase_idx %= WAVETABLE_SIZE;

        // Envelope update
        updateEnvelope(v);

        // Sample
        int32_t s = wavetable[v.phase_idx];
        s = (int32_t)(s * v.envLevel * v.velocity);

        mix += s;
      }

      // Clamp mix
      if (mix > 32767) mix = 32767;
      if (mix < -32767) mix = -32767;

      uint16_t out = waveToDAC(mix);
      buffer[2*n]   = out;
      buffer[2*n+1] = out;
    }

    size_t written;
    i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &written, portMAX_DELAY);
  }
}
