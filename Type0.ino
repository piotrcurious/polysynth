/*
  MIDI Player "Music Box" for ESP32 (Type 0 MIDI files)
  - Reads /sdcard/song.mid from SD
  - Parses header, track, VLQ, Note On/Off, Tempo (0x51)
  - Uses PolySynthESP32 class: synth.begin(), synth.noteOn(freq, vel), synth.noteOff(freq)
  - Simple blocking playback (delay between events)
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "PolySynthESP32.h"   // from the synth refactor (place PolySynthESP32.h/.cpp in project)

#define SD_CS_PIN 5           // change for your board (common: 5, 13)
#define MIDI_FILENAME "/song.mid"

PolySynthESP32 synth;

// helper: read big-endian 32-bit
uint32_t readBE32(File &f) {
  uint32_t v = 0;
  for (int i = 0; i < 4; ++i) {
    int b = f.read();
    if (b < 0) return 0;
    v = (v << 8) | (uint32_t)(b & 0xFF);
  }
  return v;
}

// read big-endian 16-bit
uint16_t readBE16(File &f) {
  int hi = f.read();
  int lo = f.read();
  if (hi < 0 || lo < 0) return 0;
  return (uint16_t)((hi << 8) | lo);
}

// read single byte
int readByte(File &f) {
  return f.read();
}

// read variable-length quantity (VLQ), returns value and updates bytes read via reference if needed
uint32_t readVLQ(File &f) {
  uint32_t value = 0;
  while (true) {
    int b = f.read();
    if (b < 0) return value; // EOF
    value = (value << 7) | (uint32_t)(b & 0x7F);
    if ((b & 0x80) == 0) break;
  }
  return value;
}

// convert MIDI note number (0..127) to frequency (Hz)
float midiNoteToFreq(int noteNumber) {
  // formula: f = 440 * 2^((note-69)/12)
  return 440.0f * powf(2.0f, ((float)noteNumber - 69.0f) / 12.0f);
}

void playMidiType0(File &f) {
  // Read MTrk chunk header "MTrk"
  char tag[5] = {0};
  if (f.readBytes(tag, 4) != 4) {
    Serial.println("Track header read failed");
    return;
  }
  if (strncmp(tag, "MTrk", 4) != 0) {
    Serial.printf("Expected MTrk, got '%s'\n", tag);
    return;
  }

  uint32_t trackLen = readBE32(f);
  uint32_t trackStart = f.position();
  uint32_t trackEnd = trackStart + trackLen;

  // default tempo = 500000 microseconds per quarter note (120 BPM)
  uint32_t usPerQuarter = 500000u;

  // running status support
  int runningStatus = -1;

  while (f.position() < trackEnd) {
    uint32_t deltaTicks = readVLQ(f);

    // convert deltaTicks to milliseconds (may be fractional)
    // We'll compute ms as integer but keep remainder in microseconds to reduce drift
    // ms = (deltaTicks * usPerQuarter) / ticksPerQuarter / 1000
    // ticksPerQuarter is read from file earlier and stored globally for this play routine
    // For simplicity, global variable ticksPerQuarter is used (set in setup after header parse)
    extern uint16_t ticksPerQuarter; // forward-declared to be set in setup

    if (ticksPerQuarter == 0) ticksPerQuarter = 480; // fallback

    // compute total microseconds for this delta
    uint64_t totalMicro = (uint64_t)deltaTicks * (uint64_t)usPerQuarter;
    // divide by ticks per quarter
    uint64_t eventMicro = totalMicro / (uint64_t)ticksPerQuarter;

    // delay by that many milliseconds (blocking simple player)
    if (eventMicro > 0) {
      uint32_t eventMs = (uint32_t)(eventMicro / 1000u);
      uint32_t remUs = (uint32_t)(eventMicro % 1000u);
      if (eventMs) delay(eventMs);
      if (remUs) delayMicroseconds(remUs);
    }

    // read event byte (may be status or data if running status)
    int b = f.read();
    if (b < 0) break;

    if (b == 0xFF) {
      // Meta event
      int metaType = f.read();
      if (metaType < 0) break;
      uint32_t len = readVLQ(f);
      if (metaType == 0x2F) {
        // End of track
        // consume remaining bytes if any of this meta
        for (uint32_t i=0;i<len;i++) f.read();
        Serial.println("End of track meta");
        break;
      } else if (metaType == 0x51 && len == 3) {
        // Set tempo: 3 bytes, microseconds per quarter
        int t0 = f.read();
        int t1 = f.read();
        int t2 = f.read();
        if (t0 < 0 || t1 < 0 || t2 < 0) break;
        usPerQuarter = ((uint32_t)t0 << 16) | ((uint32_t)t1 << 8) | (uint32_t)t2;
        Serial.printf("Tempo change: %u us/qn -> %.2f BPM\n", usPerQuarter, 60000000.0f / (float)usPerQuarter);
      } else {
        // skip any other meta data
        for (uint32_t i=0;i<len;i++) f.read();
      }
      runningStatus = -1; // meta resets running status
    } else if ((b & 0xF0) == 0xF0) {
      // SysEx or other system common messages: skip
      if (b == 0xF0 || b == 0xF7) {
        uint32_t len = readVLQ(f);
        for (uint32_t i=0;i<len;i++) f.read();
      } else {
        // other system messages we don't handle: ignore single byte
      }
      runningStatus = -1;
    } else {
      // MIDI event or running status:
      int statusByte;
      int data1;

      if (b & 0x80) {
        // this is a status byte
        statusByte = b;
        data1 = f.read();
        if (data1 < 0) break;
        runningStatus = statusByte;
      } else {
        // this is data byte meaning running status applies; b is first data byte
        if (runningStatus < 0) {
          Serial.println("Running status used but no running status set; abort");
          return;
        }
        statusByte = runningStatus;
        data1 = b; // first data byte already read as b
      }

      uint8_t statusHigh = statusByte & 0xF0;
      uint8_t channel = statusByte & 0x0F;

      if (statusHigh == 0x80) {
        // Note Off: second data byte follows
        int data2 = f.read();
        if (data2 < 0) break;
        int note = data1;
        // stop note
        float freq = midiNoteToFreq(note);
        synth.noteOff(freq);
      } else if (statusHigh == 0x90) {
        // Note On: second data byte follows
        int data2 = f.read();
        if (data2 < 0) break;
        int note = data1;
        int vel = data2;
        float freq = midiNoteToFreq(note);
        if (vel == 0) {
          synth.noteOff(freq);
        } else {
          float velf = (float)vel / 127.0f;
          synth.noteOn(freq, velf);
        }
      } else if (statusHigh == 0xA0 || statusHigh == 0xB0 || statusHigh == 0xE0) {
        // Aftertouch, CC, Pitch Bend: 2 data bytes - ignore them
        f.read(); // second data byte
      } else if (statusHigh == 0xC0 || statusHigh == 0xD0) {
        // Program change or Channel pressure: single data byte (we already have one)
        // nothing to do here
      } else {
        // unknown; try to be safe: if event expects second data byte, consume it.
        // We don't attempt a full robust spec here.
      }
    }
  }

  Serial.println("Finished track playback");
}

// ticksPerQuarter needs to be accessible to play function above
uint16_t ticksPerQuarter = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Music box: initializing synth and SD card");

  // Start synth
  synth.begin();
  // gentle ADSR for music-box sound
  synth.setADSR(0.003f, 0.08f, 0.6f, 0.25f);

  // Init SD
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD.begin failed - check CS pin and wiring");
    while (1) delay(500);
  }

  File midiFile = SD.open(MIDI_FILENAME, FILE_READ);
  if (!midiFile) {
    Serial.printf("Failed to open %s\n", MIDI_FILENAME);
    while (1) delay(500);
  }

  // Parse MIDI header: expect "MThd"
  char hdr[5] = {0};
  if (midiFile.readBytes(hdr, 4) != 4 || strncmp(hdr, "MThd", 4) != 0) {
    Serial.printf("Not a MIDI file (MThd missing). Got '%s'\n", hdr);
    midiFile.close();
    while (1) delay(500);
  }

  uint32_t headerLen = readBE32(midiFile);
  uint16_t format = readBE16(midiFile);
  uint16_t ntrks = readBE16(midiFile);
  ticksPerQuarter = readBE16(midiFile);

  Serial.printf("MIDI Header: len=%u format=%u tracks=%u tpq=%u\n", headerLen, format, ntrks, ticksPerQuarter);

  // For simplicity: only support Type 0 (format==0). Type 1 often needs per-track merging.
  if (format != 0) {
    Serial.println("Only MIDI Type 0 is supported by this simple player.");
    Serial.println("You can convert multi-track MIDI to single-track (Type 0) using many tools.");
  }

  // Seek to first track and play (playMidiType0 handles a single MTrk)
  playMidiType0(midiFile);

  midiFile.close();
  Serial.println("Playback finished. Restart board to play again.");
}

void loop() {
  // nothing — everything runs in setup blocking playback once.
  delay(1000);
}
