/*
  Music Box: Type-1 MIDI player (non-blocking) + polyphonic wavetable synth
  + per-channel pitch bend + channel-aware voice allocation

  - Drop /song.mid on SD root
  - Serial commands:
      p : pause/resume
      s : stop
      r : restart
  Notes:
  - Default pitch-bend range = +/- 2 semitones (per channel)
  - Voices are assigned to the MIDI channel they were started on.
  - Pitch-bend updates retune active voices of that channel in-place (phase preserved).
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "driver/i2s.h"
#include "esp_timer.h" // for esp_timer_get_time()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------------- CONFIG ----------------
#define SD_CS_PIN       5
#define MIDI_FILENAME   "/song.mid"

#define SAMPLE_RATE     48000
#define WAVETABLE_SIZE  256
#define MAX_VOICES      8
#define CHUNK_FRAMES    256
#define DITHER_BITS     5
#define SCALE_MICROHZ   1000000ULL
// -----------------------------------------

// ---------------- Simple PolySynth (wavetable + ADSR) ----------------
struct ADSR { float a=0.005f, d=0.08f, s=0.6f, r=0.25f; };

struct Voice {
  bool active = false;
  uint8_t channel = 0;     // MIDI channel that owns this voice
  float baseFreqHz = 0.0f; // original note frequency (without pitch bend)
  float vel = 1.0f;

  // Bresenham stepping state
  uint32_t phase_idx = 0;
  uint64_t err = 0;
  uint64_t step_numer = 0;
  uint64_t q = 0;
  uint64_t rem = 0;

  // envelope
  uint8_t stage = 0; // 0=off,1=attack,2=decay,3=sustain,4=release
  float level = 0.0f;
  ADSR adsr;

  // for voice-stealing policy
  uint32_t ageCounter = 0;
};

static int16_t wavetable[WAVETABLE_SIZE];
static Voice voices[MAX_VOICES];
static uint32_t rng_state = 0x12345678;
static TaskHandle_t synthTaskHandle = NULL;

// per-channel pitch bend multiplier and range
static float channelPitchMultiplier[16];
static float channelPitchRangeSemis[16]; // semitone range (+/-) default 2.0

// synchronization for voice table modifications
static portMUX_TYPE synthMux = portMUX_INITIALIZER_UNLOCKED;

// global denom used by both synth and scheduler
static const uint64_t STEP_DENOM = (uint64_t)SAMPLE_RATE * (uint64_t)SCALE_MICROHZ;

// age counter
static uint32_t globalAgeCounter = 1;

static inline uint32_t xorshift32() {
  uint32_t x = rng_state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  rng_state = x;
  return x;
}

static inline uint16_t waveToI2SWord_int16(int32_t v_signed16) {
  uint32_t u = (uint32_t)(v_signed16 + 32768); // 0..65535
  uint8_t dac8 = (uint8_t)(u >> 8);
  return (uint16_t)(dac8 << 8);
}

void buildWavetable() {
  for (int i = 0; i < WAVETABLE_SIZE; ++i) {
    float f = sinf((2.0f * PI * (float)i) / (float)WAVETABLE_SIZE);
    int32_t s = (int32_t)roundf(f * 32767.0f);
    if (s > 32767) s = 32767;
    if (s < -32767) s = -32767;
    wavetable[i] = (int16_t)s;
  }
}

bool configureI2S_fixedRate() {
  i2s_driver_uninstall(I2S_NUM_0);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512 / 2,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("i2s_driver_install failed: %d\n", (int)err);
    return false;
  }

  i2s_pin_config_t pin_config;
  memset(&pin_config, 0, sizeof(pin_config));
  i2s_set_pin(I2S_NUM_0, &pin_config);

  i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN);
  i2s_zero_dma_buffer(I2S_NUM_0);
  return true;
}

// compute step numerators q/rem from frequency (preserve phase_idx and err)
static void setVoiceFrequency(Voice &v, float newFreqHz) {
  // must run inside critical section
  uint64_t freq_micro = (uint64_t)(newFreqHz * (double)SCALE_MICROHZ + 0.5);
  uint64_t step_numer = (uint64_t)WAVETABLE_SIZE * freq_micro;
  v.step_numer = step_numer;
  v.q = step_numer / STEP_DENOM;
  v.rem = step_numer % STEP_DENOM;
}

// convert MIDI note number to frequency (Hz)
float midiNoteToFreq(uint8_t note) {
  return 440.0f * powf(2.0f, ((float)note - 69.0f) / 12.0f);
}

// allocate a voice for a given channel: prefer unused, then oldest on same channel, then global oldest
Voice* allocateVoiceForChannel(uint8_t channel) {
  Voice *victim = nullptr;

  // 1) find inactive
  for (int i = 0; i < MAX_VOICES; ++i)
    if (!voices[i].active) {
      victim = &voices[i];
      break;
    }

  if (!victim) {
    // 2) find oldest voice on the same channel
    uint32_t oldestAge = UINT32_MAX;
    for (int i = 0; i < MAX_VOICES; ++i) {
      if (voices[i].channel == channel && voices[i].ageCounter < oldestAge) {
        oldestAge = voices[i].ageCounter;
        victim = &voices[i];
      }
    }
  }

  if (!victim) {
    // 3) global oldest
    uint32_t oldestAge = UINT32_MAX;
    for (int i = 0; i < MAX_VOICES; ++i) {
      if (voices[i].ageCounter < oldestAge) {
        oldestAge = voices[i].ageCounter;
        victim = &voices[i];
      }
    }
  }

  return victim;
}

// Public API used by the MIDI scheduler
void synthNoteOn(uint8_t channel, float freqHz, float velocity) {
  portENTER_CRITICAL(&synthMux);

  Voice *v = allocateVoiceForChannel(channel);
  if (!v) { portEXIT_CRITICAL(&synthMux); return; }

  v->active = true;
  v->channel = channel;
  v->baseFreqHz = freqHz;
  v->vel = velocity;
  v->stage = 1; // attack
  v->level = 0.0f;
  v->ageCounter = globalAgeCounter++;
  v->phase_idx = 0; // start at 0 for simplicity; could be randomized
  v->err = 0;

  // apply current channel pitch multiplier
  float mult = channelPitchMultiplier[channel];
  float tunedFreq = freqHz * mult;

  setVoiceFrequency(*v, tunedFreq);

  portEXIT_CRITICAL(&synthMux);
}

void synthNoteOff(uint8_t channel, uint8_t note) {
  // Stop voices on that channel with matching note frequency (close match)
  float baseFreq = midiNoteToFreq(note);
  portENTER_CRITICAL(&synthMux);
  for (int i = 0; i < MAX_VOICES; ++i) {
    Voice &v = voices[i];
    if (!v.active) continue;
    if (v.channel != channel) continue;
    if (fabs(v.baseFreqHz - baseFreq) < 0.5f) {
      if (v.stage != 4) v.stage = 4; // release
    }
  }
  portEXIT_CRITICAL(&synthMux);
}

// set pitch-bend multiplier for a channel (and retune all active voices for that channel)
void setChannelPitchMultiplier(uint8_t channel, float multiplier) {
  portENTER_CRITICAL(&synthMux);
  channelPitchMultiplier[channel] = multiplier;
  // retune active voices of this channel
  for (int i = 0; i < MAX_VOICES; ++i) {
    Voice &v = voices[i];
    if (!v.active) continue;
    if (v.channel != channel) continue;
    float newFreq = v.baseFreqHz * multiplier;
    // preserve phase_idx and err, only update step numer/q/rem
    setVoiceFrequency(v, newFreq);
  }
  portEXIT_CRITICAL(&synthMux);
}

void synthSetADSR(float A,float D,float S,float R) {
  for (int i = 0; i < MAX_VOICES; ++i) {
    voices[i].adsr.a = A;
    voices[i].adsr.d = D;
    voices[i].adsr.s = S;
    voices[i].adsr.r = R;
  }
}

// envelope update (called inside synth thread)
static inline void updateEnvelope(Voice &v) {
  float sr = (float)SAMPLE_RATE;
  switch (v.stage) {
    case 1: // attack
      if (v.adsr.a <= 0.00001f) { v.level = 1.0f; v.stage = 2; break; }
      v.level += 1.0f / (v.adsr.a * sr);
      if (v.level >= 1.0f) { v.level = 1.0f; v.stage = 2; }
      break;
    case 2: // decay
      if (v.adsr.d <= 0.00001f) { v.level = v.adsr.s; v.stage = 3; break; }
      v.level -= (1.0f - v.adsr.s) / (v.adsr.d * sr);
      if (v.level <= v.adsr.s) { v.level = v.adsr.s; v.stage = 3; }
      break;
    case 3: // sustain
      // hold
      break;
    case 4: // release
      if (v.adsr.r <= 0.00001f) { v.level = 0.0f; v.stage = 0; v.active = false; break; }
      v.level -= (v.adsr.s) / (v.adsr.r * sr);
      if (v.level <= 0.0f) { v.level = 0.0f; v.stage = 0; v.active = false; }
      break;
    default:
      break;
  }
}

// synth streaming task: Bresenham stepping + integer triangular dither
void synthStreamTask(void *arg) {
  Serial.println("Synth stream task started");
  const size_t wordsPerFrame = 2;
  const size_t chunkWords = (size_t)CHUNK_FRAMES * wordsPerFrame;
  uint16_t *buf = (uint16_t*)malloc(chunkWords * sizeof(uint16_t));
  if (!buf) { Serial.println("alloc fail"); vTaskDelete(NULL); return; }

  const uint64_t denom = STEP_DENOM;
  const uint32_t dmask = (DITHER_BITS > 0) ? ((1u << DITHER_BITS) - 1u) : 0;

  while (true) {
    // fill chunk
    for (int n = 0; n < CHUNK_FRAMES; ++n) {
      int32_t mix = 0;

      // We'll take a quick critical section to snapshot necessary voice state,
      // then operate on the snapshot to minimize lock hold time.
      portENTER_CRITICAL(&synthMux);
      Voice snapshot[MAX_VOICES];
      for (int i = 0; i < MAX_VOICES; ++i) snapshot[i] = voices[i];
      portEXIT_CRITICAL(&synthMux);

      for (int i = 0; i < MAX_VOICES; ++i) {
        Voice &v = snapshot[i];
        if (!v.active) continue;

        // step
        uint32_t phase_idx_local = v.phase_idx;
        uint64_t err_local = v.err;
        uint64_t q_local = v.q;
        uint64_t rem_local = v.rem;

        phase_idx_local += (uint32_t)q_local;
        err_local += rem_local;

        if (DITHER_BITS > 0) {
          // apply triangular dither to err/carry decision
          uint32_t r1 = xorshift32();
          uint32_t r2 = xorshift32();
          int32_t tri = (int32_t)((r1 & dmask) - (r2 & dmask));
          int64_t cmp = (int64_t)err_local + (int64_t)tri;
          if (cmp >= (int64_t)denom) {
            phase_idx_local++;
            err_local = (uint64_t)(cmp - (int64_t)denom);
          } else if (cmp < 0) {
            err_local = 0;
          } else {
            err_local = (uint64_t)cmp;
          }
        } else {
          if (err_local >= denom) { phase_idx_local++; err_local -= denom; }
        }

        if (phase_idx_local >= (uint32_t)WAVETABLE_SIZE) phase_idx_local %= (uint32_t)WAVETABLE_SIZE;

        // envelope update - operate on snapshot
        updateEnvelope(v);

        int32_t s = wavetable[phase_idx_local];
        float scaled = (float)s * v.level * v.vel;
        mix += (int32_t)scaled;

        // write back step and phase to real voices under lock
        portENTER_CRITICAL(&synthMux);
        // Only update phase_idx and err of the real voice to preserve sync with stream
        voices[i].phase_idx = phase_idx_local;
        voices[i].err = err_local;
        // Also copy envelope level and stage to the real voice
        voices[i].level = v.level;
        voices[i].stage = v.stage;
        // If release finished in snapshot, deactivate
        if (voices[i].stage == 0 && voices[i].level <= 0.0f) voices[i].active = false;
        portEXIT_CRITICAL(&synthMux);
      }

      // clamp
      if (mix > 32767) mix = 32767;
      if (mix < -32767) mix = -32767;

      uint16_t w = waveToI2SWord_int16(mix);
      buf[2*n + 0] = w;
      buf[2*n + 1] = w;
    }

    size_t bytesToWrite = chunkWords * sizeof(uint16_t);
    size_t bytesWritten = 0;
    esp_err_t e = i2s_write(I2S_NUM_0, (const char*)buf, bytesToWrite, &bytesWritten, portMAX_DELAY);
    if (e != ESP_OK) {
      Serial.printf("i2s_write err %d\n", (int)e);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  free(buf);
  vTaskDelete(NULL);
}

void synthBegin() {
  buildWavetable();
  if (!configureI2S_fixedRate()) {
    Serial.println("I2S config failed");
    while (1) delay(1000);
  }

  // initialize channel multipliers & ranges
  for (int ch = 0; ch < 16; ++ch) {
    channelPitchMultiplier[ch] = 1.0f;
    channelPitchRangeSemis[ch] = 2.0f; // default +/-2 semitones
  }

  // create task
  xTaskCreatePinnedToCore(synthStreamTask, "synth", 8192, NULL, 5, &synthTaskHandle, 1);
}
// ---------------- End PolySynth ----------------


// ---------------- MIDI parsing + scheduler ----------------
#include <vector>
struct MidiEvent {
  uint64_t tick;    // MIDI tick absolute
  uint64_t usec;    // absolute microseconds (filled after conversion)
  uint8_t type;     // 0=NoteOn,1=NoteOff,2=Tempo,3=PitchBend
  uint8_t channel;
  uint8_t note;
  uint8_t vel;
  uint32_t tempo;   // for tempo events
  uint16_t bend;    // for pitch bend 14-bit (0..16383), center=8192
};

std::vector<MidiEvent> events;
uint16_t ticksPerQuarter = 480;
bool playbackPaused = false;
bool playbackStopped = true;
uint64_t startUsec = 0;
size_t eventCursor = 0;

// MIDI helpers
uint32_t readBE32(File &f) {
  uint32_t v = 0;
  for (int i = 0; i < 4; ++i) {
    int b = f.read(); if (b < 0) return 0;
    v = (v << 8) | (uint8_t)b;
  }
  return v;
}
uint16_t readBE16(File &f) {
  int hi = f.read(); int lo = f.read();
  if (hi < 0 || lo < 0) return 0;
  return (uint16_t)((hi << 8) | lo);
}
uint32_t readVLQ(File &f) {
  uint32_t value = 0;
  while (true) {
    int b = f.read();
    if (b < 0) return value;
    value = (value << 7) | (uint32_t)(b & 0x7F);
    if ((b & 0x80) == 0) break;
  }
  return value;
}

// parse one track into per-track event list
bool parseTrackToEvents(File &f, uint32_t trackLen, std::vector<MidiEvent> &outEvents) {
  uint32_t trackEnd = f.position() + trackLen;
  uint64_t tickPos = 0;
  int runningStatus = -1;

  while ((uint32_t)f.position() < trackEnd) {
    uint32_t delta = readVLQ(f);
    tickPos += delta;

    int b = f.read();
    if (b < 0) return false;

    if (b == 0xFF) {
      int metaType = f.read();
      uint32_t len = readVLQ(f);
      if (metaType == 0x2F) {
        for (uint32_t i = 0; i < len; ++i) f.read();
        break;
      } else if (metaType == 0x51 && len == 3) {
        int t0 = f.read(); int t1 = f.read(); int t2 = f.read();
        uint32_t usPerQuarter = ((uint32_t)t0 << 16) | ((uint32_t)t1 << 8) | (uint32_t)t2;
        MidiEvent me;
        me.tick = tickPos; me.type = 2; me.tempo = usPerQuarter;
        outEvents.push_back(me);
      } else {
        for (uint32_t i = 0; i < len; ++i) f.read();
      }
      runningStatus = -1;
    } else if ((b & 0xF0) == 0xF0) {
      // SysEx
      if (b == 0xF0 || b == 0xF7) {
        uint32_t len = readVLQ(f);
        for (uint32_t i = 0; i < len; ++i) f.read();
      }
      runningStatus = -1;
    } else {
      int statusByte;
      int data1;
      if (b & 0x80) {
        statusByte = b;
        data1 = f.read(); if (data1 < 0) data1 = 0;
        runningStatus = statusByte;
      } else {
        if (runningStatus < 0) {
          Serial.println("Running status used but not set");
          return false;
        }
        statusByte = runningStatus;
        data1 = b;
      }

      uint8_t statusHigh = statusByte & 0xF0;
      uint8_t channel = statusByte & 0x0F;

      if (statusHigh == 0x80) {
        int data2 = f.read(); if (data2 < 0) data2 = 0;
        MidiEvent me; me.tick = tickPos; me.type = 1; me.channel = channel; me.note = (uint8_t)data1; me.vel = (uint8_t)data2;
        outEvents.push_back(me);
      } else if (statusHigh == 0x90) {
        int data2 = f.read(); if (data2 < 0) data2 = 0;
        if (data2 == 0) {
          MidiEvent me; me.tick = tickPos; me.type = 1; me.channel = channel; me.note = (uint8_t)data1; me.vel = 0;
          outEvents.push_back(me);
        } else {
          MidiEvent me; me.tick = tickPos; me.type = 0; me.channel = channel; me.note = (uint8_t)data1; me.vel = (uint8_t)data2;
          outEvents.push_back(me);
        }
      } else if (statusHigh == 0xE0) {
        // pitch bend: two bytes LSB then MSB
        int lsb = f.read(); if (lsb < 0) lsb = 0;
        int msb = f.read(); if (msb < 0) msb = 0;
        uint16_t val = (uint16_t)((msb << 7) | (lsb & 0x7F)); // 0..16383, center 8192
        MidiEvent me; me.tick = tickPos; me.type = 3; me.channel = channel; me.bend = val;
        outEvents.push_back(me);
      } else if (statusHigh == 0xA0 || statusHigh == 0xB0) {
        // two-data but ignore actual controller for now
        f.read();
        f.read();
      } else if (statusHigh == 0xC0 || statusHigh == 0xD0) {
        // single-data messages
        f.read();
      } else if (statusHigh == 0xE0) {
        // already handled above
      } else {
        // unknown: attempt to consume one more byte safely
        f.read();
      }
    }
  }

  return true;
}

void mergeAndSort(std::vector<std::vector<MidiEvent>> &perTrack, std::vector<MidiEvent> &outMerged) {
  outMerged.clear();
  for (auto &v : perTrack) for (auto &e : v) outMerged.push_back(e);
  std::stable_sort(outMerged.begin(), outMerged.end(), [](const MidiEvent &a, const MidiEvent &b){
    if (a.tick != b.tick) return a.tick < b.tick;
    // ensure tempo/pitchbend first? keep stable
    return a.type < b.type;
  });
}

// convert ticks->absolute microseconds honoring tempo events in order
void ticksToUsec(std::vector<MidiEvent> &merged, uint16_t tpq) {
  uint64_t curUsec = 0;
  uint64_t lastTick = 0;
  uint32_t usPerQuarter = 500000u; // default

  for (size_t i = 0; i < merged.size(); ++i) {
    MidiEvent &e = merged[i];
    uint64_t deltaTicks = e.tick - lastTick;
    if (deltaTicks) {
      curUsec += (uint64_t)deltaTicks * (uint64_t)usPerQuarter / (uint64_t)tpq;
    }
    e.usec = curUsec;
    if (e.type == 2) {
      usPerQuarter = e.tempo;
    }
    lastTick = e.tick;
  }
}

// load MIDI (Type 0 or 1), parse tracks into events vector with usec timestamps
bool loadMidiFile(const char *path, std::vector<MidiEvent> &outEvents) {
  outEvents.clear();
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.printf("Failed to open %s\n", path); return false; }

  char hdr[5] = {0};
  if (f.readBytes(hdr, 4) != 4 || strncmp(hdr, "MThd", 4) != 0) {
    Serial.println("Not a MIDI file (missing MThd)");
    f.close(); return false;
  }
  uint32_t headerLen = readBE32(f);
  uint16_t format = readBE16(f);
  uint16_t ntrks = readBE16(f);
  ticksPerQuarter = readBE16(f);
  Serial.printf("MIDI header: format=%u tracks=%u tpq=%u\n", format, ntrks, ticksPerQuarter);

  std::vector<std::vector<MidiEvent>> perTrack;
  perTrack.resize(ntrks);

  for (int t = 0; t < ntrks; ++t) {
    char ttag[5] = {0};
    if (f.readBytes(ttag, 4) != 4) { Serial.println("Track header read fail"); f.close(); return false; }
    if (strncmp(ttag, "MTrk", 4) != 0) { Serial.printf("Expected MTrk, got '%s'\n", ttag); f.close(); return false; }
    uint32_t trackLen = readBE32(f);
    uint32_t posBefore = f.position();
    if (!parseTrackToEvents(f, trackLen, perTrack[t])) { Serial.printf("Parse track %d failed\n", t); f.close(); return false; }
    uint32_t posAfter = f.position();
    uint32_t consumed = posAfter - posBefore;
    if (consumed < trackLen) f.seek(posBefore + trackLen); // skip padding if any
    Serial.printf("Track %d events: %u\n", t, (unsigned)perTrack[t].size());
  }
  f.close();

  mergeAndSort(perTrack, outEvents);
  Serial.printf("Merged events: %u\n", (unsigned)outEvents.size());
  ticksToUsec(outEvents, ticksPerQuarter);
  if (!outEvents.empty()) {
    Serial.printf("Event times: start %llu us, end %llu us\n", (unsigned long long)outEvents.front().usec, (unsigned long long)outEvents.back().usec);
  }
  return true;
}

// ---------------- Playback control & scheduler ----------------
void startPlayback() {
  if (events.empty()) { Serial.println("No events loaded"); return; }
  playbackPaused = false;
  playbackStopped = false;
  eventCursor = 0;
  startUsec = (uint64_t)esp_timer_get_time();
  Serial.println("Playback started");
}

void stopPlayback() {
  playbackStopped = true;
  playbackPaused = false;
  // force release all voices
  portENTER_CRITICAL(&synthMux);
  for (int i = 0; i < MAX_VOICES; ++i) {
    if (voices[i].active) voices[i].stage = 4;
  }
  portEXIT_CRITICAL(&synthMux);
  Serial.println("Playback stopped");
}

void pauseResume() {
  if (playbackStopped) return;
  playbackPaused = !playbackPaused;
  if (!playbackPaused) {
    // resuming: shift startUsec so next event is scheduled correctly
    uint64_t now = (uint64_t)esp_timer_get_time();
    if (eventCursor < events.size()) {
      uint64_t nextEventUsec = events[eventCursor].usec;
      startUsec = now - nextEventUsec;
    } else {
      startUsec = now;
    }
    Serial.println("Resumed");
  } else {
    Serial.println("Paused");
  }
}

void processScheduler() {
  if (playbackStopped || playbackPaused) return;
  uint64_t now = (uint64_t)esp_timer_get_time();
  // elapsed absolute usec since start
  uint64_t elapsed = now - startUsec;

  while (eventCursor < events.size()) {
    MidiEvent &e = events[eventCursor];
    if (e.usec <= elapsed) {
      if (e.type == 0) {
        // Note on
        float freq = midiNoteToFreq(e.note);
        float vel = (float)e.vel / 127.0f;
        // multiply by current channel pitch multiplier
        float mult = channelPitchMultiplier[e.channel];
        synthNoteOn(e.channel, freq, vel);
      } else if (e.type == 1) {
        // Note off
        synthNoteOff(e.channel, e.note);
      } else if (e.type == 2) {
        // Tempo meta: already applied in ticksToUsec; nothing at runtime
      } else if (e.type == 3) {
        // Pitch bend - update channel multiplier and retune voices
        // e.bend is 0..16383, center=8192
        int32_t val = (int32_t)e.bend;
        float center = 8192.0f;
        float frac = ((float)val - center) / center; // -1..+~1
        float rangeSemis = channelPitchRangeSemis[e.channel];
        float semis = frac * rangeSemis;
        float multiplier = powf(2.0f, semis / 12.0f);
        setChannelPitchMultiplier(e.channel, multiplier);
      }
      eventCursor++;
    } else break;
  }

  if (eventCursor >= events.size()) {
    Serial.println("Playback finished");
    playbackStopped = true;
  }
}

// ---------------- Arduino setup/loop ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nMusic Box with Pitch Bend & Channel-aware Voices");

  // init synth
  synthBegin();
  synthSetADSR(0.005f, 0.08f, 0.6f, 0.25f);

  // init SD
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed");
    while (1) delay(500);
  }

  Serial.printf("Loading '%s' ...\n", MIDI_FILENAME);
  if (!loadMidiFile(MIDI_FILENAME, events)) {
    Serial.println("Failed to load MIDI file");
    while (1) delay(500);
  }

  // initialize channel ranges & multipliers (already done in synthBegin, but ensure here)
  for (int ch=0; ch<16; ++ch) {
    channelPitchRangeSemis[ch] = 2.0f; // +/- 2 semitones
    channelPitchMultiplier[ch] = 1.0f;
  }

  startPlayback();
}

void loop() {
  processScheduler();

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'p' || c == 'P') pauseResume();
    if (c == 's' || c == 'S') stopPlayback();
    if (c == 'r' || c == 'R') startPlayback();
  }

  delay(1);
}
