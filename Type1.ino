/*
  Music Box: Type-1 MIDI player + non-blocking scheduler + polyphonic wavetable synth (ESP32)
  - Read /song.mid from SD card
  - Supports MIDI Type 0 and Type 1
  - Handles Note On/Note Off (NoteOn with vel=0 -> off) and Tempo meta (0x51)
  - Merges tracks, maps ticks->microseconds using tempo map, schedules events non-blocking (micros)
  - Uses a small PolySynth (Bresenham integer stepping + per-voice ADSR)
  - Serial commands:
      'p' -> pause/resume
      's' -> stop
      'r' -> restart playback from beginning
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// ---------------- CONFIG ----------------
#define SD_CS_PIN       5        // change if needed
#define MIDI_FILENAME   "/song.mid"

#define SAMPLE_RATE     48000
#define WAVETABLE_SIZE  256
#define MAX_VOICES      8
#define CHUNK_FRAMES    256
#define DITHER_BITS     5
#define SCALE_MICROHZ   1000000ULL
// -----------------------------------------

// ---------------- Simple PolySynth ----------------
// Very compact polyphonic wavetable synth with ADSR and integer stepping.
// Public API used by MIDI player: synthBegin(), synthSetADSR(...), noteOn(freq, vel), noteOff(freq)

struct ADSR { float a=0.01f, d=0.05f, s=0.7f, r=0.2f; };

struct Voice {
  bool active = false;
  float freqHz = 0.0f;
  float vel = 1.0f;
  uint32_t phase_idx = 0;
  uint64_t err = 0;
  uint64_t step_numer = 0;
  uint64_t q = 0;
  uint64_t rem = 0;

  // envelope
  uint8_t stage = 0; // 0=off,1=attack,2=decay,3=sustain,4=release
  float level = 0.0f;
  ADSR adsr;
};

static int16_t wavetable[WAVETABLE_SIZE];
static Voice voices[MAX_VOICES];
static uint32_t rng_state = 0x12345678;
static TaskHandle_t synthTaskHandle = NULL;

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

// configure I2S (built-in DAC)
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

void synthNoteOn(float freqHz, float velocity) {
  // allocate voice (find inactive first)
  Voice *v = nullptr;
  for (int i=0;i<MAX_VOICES;i++) if (!voices[i].active) { v=&voices[i]; break; }
  if (!v) v = &voices[0]; // steal first if none free

  v->active = true;
  v->freqHz = freqHz;
  v->vel = velocity;
  v->stage = 1; // attack
  v->level = 0.0f;

  uint64_t denom = (uint64_t)SAMPLE_RATE * (uint64_t)SCALE_MICROHZ;
  uint64_t freq_micro = (uint64_t)(freqHz * (double)SCALE_MICROHZ + 0.5);
  v->step_numer = (uint64_t)WAVETABLE_SIZE * freq_micro;
  v->q = v->step_numer / denom;
  v->rem = v->step_numer % denom;
  v->phase_idx = 0;
  v->err = 0;
}

void synthNoteOff(float freqHz) {
  // stop any voice with similar freq (simple approach)
  for (int i=0;i<MAX_VOICES;i++) {
    if (voices[i].active && fabs(voices[i].freqHz - freqHz) < 0.5f) {
      if (voices[i].stage != 4) voices[i].stage = 4; // release
    }
  }
}

void synthSetADSR(float A,float D,float S,float R) {
  for (int i=0;i<MAX_VOICES;i++) {
    voices[i].adsr.a = A;
    voices[i].adsr.d = D;
    voices[i].adsr.s = S;
    voices[i].adsr.r = R;
  }
}

static inline void updateEnvelope(Voice &v) {
  float sr = (float)SAMPLE_RATE;
  switch(v.stage) {
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

void synthStreamTask(void *arg) {
  Serial.println("Synth stream task started");
  const size_t wordsPerFrame = 2;
  const size_t chunkWords = (size_t)CHUNK_FRAMES * wordsPerFrame;
  uint16_t *buf = (uint16_t*)malloc(chunkWords * sizeof(uint16_t));
  if (!buf) { Serial.println("alloc fail"); vTaskDelete(NULL); return; }

  const uint64_t denom = (uint64_t)SAMPLE_RATE * (uint64_t)SCALE_MICROHZ;
  const uint32_t dmask = (DITHER_BITS>0) ? ((1u<<DITHER_BITS)-1u) : 0;

  while(true) {
    // fill chunk
    for (int n=0;n<CHUNK_FRAMES;n++) {
      int32_t mix = 0;
      for (int i=0;i<MAX_VOICES;i++) {
        Voice &v = voices[i];
        if (!v.active) continue;

        // step
        v.phase_idx += (uint32_t)v.q;
        v.err += v.rem;

        if (DITHER_BITS>0) {
          uint32_t r1 = xorshift32();
          uint32_t r2 = xorshift32();
          int32_t tri = (int32_t)((r1 & dmask) - (r2 & dmask));
          int64_t cmp = (int64_t)v.err + (int64_t)tri;
          if (cmp >= (int64_t)denom) {
            v.phase_idx++;
            v.err = (uint64_t)(cmp - (int64_t)denom);
          } else if (cmp < 0) {
            v.err = 0;
          } else {
            v.err = (uint64_t)cmp;
          }
        } else {
          if (v.err >= denom) { v.phase_idx++; v.err -= denom; }
        }
        if (v.phase_idx >= (uint32_t)WAVETABLE_SIZE) v.phase_idx %= (uint32_t)WAVETABLE_SIZE;

        // envelope
        updateEnvelope(v);

        // sample and scale
        int32_t s = wavetable[v.phase_idx];
        // multiply by level and velocity (both <=1)
        float scaled = (float)s * v.level * v.vel;
        mix += (int32_t)scaled;
      }

      // clamp
      if (mix > 32767) mix = 32767;
      if (mix < -32767) mix = -32767;

      uint16_t w = waveToI2SWord_int16(mix);
      buf[2*n+0] = w;
      buf[2*n+1] = w;
    }

    size_t bytesToWrite = chunkWords * sizeof(uint16_t);
    size_t bytesWritten = 0;
    esp_err_t e = i2s_write(I2S_NUM_0, (const char*)buf, bytesToWrite, &bytesWritten, portMAX_DELAY);
    if (e != ESP_OK) {
      Serial.printf("i2s_write err %d\n",(int)e);
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
    while(1) delay(1000);
  }
  // create task
  xTaskCreatePinnedToCore(synthStreamTask, "synth", 8192, NULL, 5, &synthTaskHandle, 1);
}

void synthSetADSR(float A,float D,float S,float R) {
  for (int i=0;i<MAX_VOICES;i++) {
    voices[i].adsr.a = A;
    voices[i].adsr.d = D;
    voices[i].adsr.s = S;
    voices[i].adsr.r = R;
  }
}
// ---------------- End PolySynth ----------------


// ---------------- MIDI parsing & scheduling ----------------
#include <vector>
struct MidiEvent {
  uint64_t tick;    // MIDI tick position (absolute)
  uint64_t usec;    // computed absolute time in microseconds (filled after tempo map)
  uint8_t type;     // 0=NoteOn,1=NoteOff,2=Tempo
  uint8_t channel;  // MIDI channel
  uint8_t note;     // for note events
  uint8_t vel;      // velocity
  uint32_t tempo;   // for tempo events: microseconds per quarter
};

std::vector<MidiEvent> events; // merged events across tracks
uint16_t ticksPerQuarter = 480; // from header (divisions)
bool playbackPaused = false;
bool playbackStopped = true;
uint32_t startMicros32 = 0;
size_t eventCursor = 0;

uint32_t readByte(File &f) {
  int b = f.read();
  if (b < 0) return 0xFFFFFFFFu;
  return (uint32_t)b;
}

uint32_t readBE32(File &f) {
  uint32_t v = 0;
  for (int i=0;i<4;i++) {
    int b = f.read(); if (b<0) return 0;
    v = (v<<8) | (uint8_t)b;
  }
  return v;
}
uint16_t readBE16(File &f) {
  int hi = f.read(); int lo = f.read();
  if (hi<0 || lo<0) return 0;
  return (uint16_t)((hi<<8) | lo);
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

float midiNoteToFreq(uint8_t note) {
  return 440.0f * powf(2.0f, ((float)note - 69.0f) / 12.0f);
}

// parse a single MTrk and append events with absolute tick timestamps into local vector
bool parseTrackToEvents(File &f, uint32_t trackLen, std::vector<MidiEvent> &outEvents) {
  uint32_t trackEnd = f.position() + trackLen;
  uint64_t tickPos = 0;
  int runningStatus = -1;

  while ((uint32_t)f.position() < trackEnd) {
    uint32_t delta = readVLQ(f);
    tickPos += delta;

    int peek = f.read();
    if (peek < 0) return false;
    if (peek == 0xFF) {
      // Meta event
      int metaType = f.read();
      uint32_t len = readVLQ(f);
      if (metaType == 0x2F) {
        // end of track: consume and break
        for (uint32_t i=0;i<len;i++) f.read();
        break;
      } else if (metaType == 0x51 && len == 3) {
        // tempo
        int t0 = f.read(); int t1 = f.read(); int t2 = f.read();
        uint32_t usPerQuarter = ((uint32_t)t0 << 16) | ((uint32_t)t1 << 8) | (uint32_t)t2;
        MidiEvent me;
        me.tick = tickPos;
        me.type = 2; // tempo
        me.tempo = usPerQuarter;
        outEvents.push_back(me);
      } else {
        // skip meta data
        for (uint32_t i=0;i<len;i++) f.read();
      }
      runningStatus = -1;
    } else if ((peek & 0xF0) == 0xF0) {
      // SysEx
      if (peek == 0xF0 || peek == 0xF7) {
        uint32_t len = readVLQ(f);
        for (uint32_t i=0;i<len;i++) f.read();
      } else {
        // other system common - skip
      }
      runningStatus = -1;
    } else {
      int statusByte;
      int data1;
      if (peek & 0x80) {
        // status byte
        statusByte = peek;
        data1 = f.read(); if (data1 < 0) data1 = 0;
        runningStatus = statusByte;
      } else {
        // running status; peek is first data byte
        if (runningStatus < 0) {
          Serial.println("Running status used but not set");
          return false;
        }
        statusByte = runningStatus;
        data1 = peek;
      }

      uint8_t statusHigh = statusByte & 0xF0;
      uint8_t channel = statusByte & 0x0F;

      if (statusHigh == 0x80) {
        int data2 = f.read(); if (data2 < 0) data2 = 0;
        MidiEvent me;
        me.tick = tickPos;
        me.type = 1; // NoteOff
        me.channel = channel;
        me.note = (uint8_t)data1;
        me.vel = (uint8_t)data2;
        outEvents.push_back(me);
      } else if (statusHigh == 0x90) {
        int data2 = f.read(); if (data2 < 0) data2 = 0;
        if (data2 == 0) {
          MidiEvent me;
          me.tick = tickPos;
          me.type = 1; // NoteOff (via NoteOn vel 0)
          me.channel = channel;
          me.note = (uint8_t)data1;
          me.vel = 0;
          outEvents.push_back(me);
        } else {
          MidiEvent me;
          me.tick = tickPos;
          me.type = 0; // NoteOn
          me.channel = channel;
          me.note = (uint8_t)data1;
          me.vel = (uint8_t)data2;
          outEvents.push_back(me);
        }
      } else if (statusHigh == 0xA0 || statusHigh == 0xB0 || statusHigh == 0xE0) {
        // 2-data events we ignore but must consume second byte
        f.read();
      } else if (statusHigh == 0xC0 || statusHigh == 0xD0) {
        // 1-data events - already consumed data1
        // ignore
      } else {
        // unknown - best effort: try to skip one more byte
        f.read();
      }
    }
  }

  return true;
}

// merge multiple per-track event vectors into one sorted vector by tick
void mergeAndSortEvents(std::vector<std::vector<MidiEvent>> &perTrackEvents, std::vector<MidiEvent> &outMerged) {
  outMerged.clear();
  for (auto &v : perTrackEvents) {
    for (auto &e : v) outMerged.push_back(e);
  }
  // stable sort by tick
  std::stable_sort(outMerged.begin(), outMerged.end(), [](const MidiEvent &a, const MidiEvent &b){
    return a.tick < b.tick;
  });
}

// convert tick timestamps to absolute microseconds, honoring tempo events
void ticksToUsec(std::vector<MidiEvent> &merged, uint16_t tpq) {
  uint64_t curUsec = 0;
  uint64_t lastTick = 0;
  uint32_t usPerQuarter = 500000u; // default 120bpm

  for (size_t i=0;i<merged.size();i++) {
    MidiEvent &e = merged[i];
    uint64_t deltaTicks = e.tick - lastTick;
    if (deltaTicks) {
      // careful 64-bit math
      curUsec += (uint64_t)deltaTicks * (uint64_t)usPerQuarter / (uint64_t)tpq;
    }
    e.usec = curUsec;
    // If this event is a tempo event, update tempo for following events
    if (e.type == 2) {
      usPerQuarter = e.tempo;
    }
    lastTick = e.tick;
  }
}

// load MIDI from SD, parse tracks, build merged event list with usec timestamps
bool loadMidiFile(const char *path, std::vector<MidiEvent> &outEvents) {
  outEvents.clear();

  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.printf("Failed to open %s\n", path); return false; }

  char tag[5] = {0};
  if (f.readBytes(tag,4) != 4 || strncmp(tag,"MThd",4) != 0) {
    Serial.println("Not a MIDI file or missing MThd");
    f.close();
    return false;
  }
  uint32_t headerLen = readBE32(f);
  uint16_t format = readBE16(f);
  uint16_t ntrks = readBE16(f);
  ticksPerQuarter = readBE16(f);
  Serial.printf("MIDI header: format=%u tracks=%u tpq=%u\n", format, ntrks, ticksPerQuarter);

  std::vector<std::vector<MidiEvent>> perTrackEvents;
  perTrackEvents.resize(ntrks);

  for (int t=0;t<ntrks;t++) {
    char trackTag[5] = {0};
    if (f.readBytes(trackTag,4) != 4) { Serial.println("Track read failed"); f.close(); return false; }
    if (strncmp(trackTag,"MTrk",4) != 0) { Serial.printf("Expected MTrk, got '%s'\n", trackTag); f.close(); return false; }
    uint32_t trackLen = readBE32(f);
    uint32_t posBefore = f.position();
    if (!parseTrackToEvents(f, trackLen, perTrackEvents[t])) {
      Serial.printf("Failed to parse track %d\n", t);
      f.close(); return false;
    }
    // ensure position at track end (some parsers may not exactly consume)
    uint32_t posAfter = f.position();
    uint32_t consumed = posAfter - posBefore;
    if (consumed < trackLen) {
      // skip remaining
      f.seek(posBefore + trackLen);
    }
    Serial.printf("Parsed track %d: %u events\n", t, (unsigned)perTrackEvents[t].size());
  }

  f.close();

  // merge
  mergeAndSortEvents(perTrackEvents, outEvents);
  Serial.printf("Merged events total: %u\n", (unsigned)outEvents.size());

  // convert to us
  ticksToUsec(outEvents, ticksPerQuarter);
  if (outEvents.size()) {
    Serial.printf("First event at %llu us, last at %llu us\n", (unsigned long long)outEvents.front().usec, (unsigned long long)outEvents.back().usec);
  }
  return true;
}

// ---------------- Playback control ----------------
void startPlayback() {
  if (events.empty()) { Serial.println("No events loaded"); return; }
  playbackPaused = false;
  playbackStopped = false;
  eventCursor = 0;
  startMicros32 = micros();
  Serial.println("Playback started");
}

void stopPlayback() {
  playbackStopped = true;
  playbackPaused = false;
  // send noteOff for all voices
  for (int n=0;n<128;n++) {
    // naive: call noteOff for many possible freqs; but better: iterate voices
  }
  Serial.println("Playback stopped");
}

void pauseResume() {
  if (playbackStopped) return;
  playbackPaused = !playbackPaused;
  if (!playbackPaused) {
    // resume: shift startMicros so that upcoming events align
    uint32_t now = micros();
    uint64_t elapsed_us = (events[eventCursor].usec); // absolute time for next event
    // compute new start so that (micros()-start) >= events[eventCursor].usec
    startMicros32 = now - (uint32_t)(events[eventCursor].usec & 0xFFFFFFFFu);
    Serial.println("Resumed");
  } else {
    Serial.println("Paused");
  }
}

// process scheduled events in non-blocking manner; should be called frequently from loop()
void processScheduler() {
  if (playbackStopped || playbackPaused) return;
  // use 32-bit micros subtraction to handle wrap safely for practical song lengths
  uint32_t now32 = micros();
  uint32_t start32 = (uint32_t)startMicros32;
  uint32_t elapsed32 = now32 - start32;
  // advance cursor while event.usec <= elapsed32 (cast)
  while (eventCursor < events.size()) {
    uint64_t evtUsec = events[eventCursor].usec;
    if ((uint64_t)elapsed32 >= evtUsec) {
      MidiEvent &e = events[eventCursor];
      if (e.type == 0) {
        // Note On
        float freq = midiNoteToFreq(e.note);
        float velf = (float)e.vel / 127.0f;
        synthNoteOn(freq, velf);
      } else if (e.type == 1) {
        // Note Off
        float freq = midiNoteToFreq(e.note);
        synthNoteOff(freq);
      } else if (e.type == 2) {
        // tempo event: already applied at conversion step; no runtime action needed
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
  delay(50);
  Serial.println("\nMusic Box: Type 1 MIDI player (non-blocking)");

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

  startPlayback();
}

void loop() {
  // process scheduled MIDI events
  processScheduler();

  // serial console commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'p' || c == 'P') {
      pauseResume();
    } else if (c == 's' || c == 'S') {
      stopPlayback();
    } else if (c == 'r' || c == 'R') {
      // restart
      startPlayback();
    }
  }

  delay(1); // small yield, keep loop light
}
