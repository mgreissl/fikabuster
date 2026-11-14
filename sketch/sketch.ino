// FikaBuster - hardware node.
// Reads the Modulino Thermo and Knob, drives the LED matrix and the Modulino Buzzer.
// All I2C and matrix work happens in loop(); Bridge handlers only exchange cached values.

#include <Arduino_RouterBridge.h>
#include <Arduino_Modulino.h>
#include <Arduino_LED_Matrix.h>

#include "frames.h"

ModulinoThermo thermo;
ModulinoKnob knob;
ModulinoBuzzer buzzer;
Arduino_LED_Matrix matrix;

bool thermoOk = false;
bool knobOk = false;
bool buzzerOk = false;

// ---------- Sensor cache (written by loop, read by Bridge) ----------
volatile float lastTemp = -1000;
volatile float lastHum = -1000;
unsigned long lastThermoMs = 0;

// ---------- Knob gestures (monotonic counters, Python diffs them) ----------
const unsigned long DOUBLE_CLICK_MS = 400;
const unsigned long LONG_PRESS_MS = 1200;
volatile long knobFree = 0;    // detents turned while not pressed
volatile long knobHeld = 0;    // detents turned while pressed
volatile long clicks = 0;
volatile long doubleClicks = 0;
volatile long longPresses = 0;
int lastKnobPos = 0;
bool wasPressed = false;
bool rotatedWhilePressed = false;
bool longFired = false;
bool pendingClick = false;
unsigned long pressStartMs = 0;
unsigned long firstClickMs = 0;

// ---------- Display ----------
char displayIn[96] = "icon:cup";
volatile int displaySeq = 1;
int displaySeen = 0;
char dispMode[16] = "";
char dispArg[80] = "";
uint8_t frame[FRAME_SIZE];
unsigned long dispStartMs = 0;
unsigned long lastFrameMs = 0;

// ---------- Sounds: {freq, ms}; freq 0 = rest, ms 0 = end ----------
struct Note { uint16_t f; uint16_t ms; };
const Note S_ACK[] = {{1500, 80}, {0, 0}};
const Note S_HAPPY[] = {{1319, 120}, {0, 90}, {1760, 200}, {0, 0}};
const Note S_WARN[] = {{2200, 60}, {0, 60}, {2200, 60}, {0, 60}, {2200, 60}, {0, 60},
                       {2200, 60}, {0, 60}, {2200, 60}, {0, 60}, {2200, 60}, {0, 0}};
const Note S_WOMP[] = {{311, 350}, {0, 120}, {262, 750}, {0, 0}};
const Note S_PARTY[] = {{1568, 70}, {0, 40}, {2093, 70}, {0, 40}, {1568, 70}, {0, 40}, {2637, 180}, {0, 0}};
const Note S_FRANTIC[] = {{2800, 40}, {0, 50}, {2800, 40}, {0, 50}, {2800, 40}, {0, 50}, {2800, 40}, {0, 50},
                          {2800, 40}, {0, 50}, {2800, 40}, {0, 50}, {2800, 40}, {0, 50}, {2800, 40}, {0, 50},
                          {2800, 40}, {0, 50}, {2800, 40}, {0, 50}, {2800, 40}, {0, 50}, {2800, 40}, {0, 0}};
const Note S_TROMBONE[] = {{392, 320}, {0, 40}, {370, 320}, {0, 40}, {349, 320}, {0, 40}, {330, 1000}, {0, 0}};
const Note S_ROBOT[] = {{200, 80}, {800, 80}, {300, 80}, {1200, 80}, {150, 80}, {1000, 80}, {600, 300}, {0, 0}};
const Note S_SAVED[] = {{1047, 90}, {0, 30}, {1319, 90}, {0, 30}, {1568, 200}, {0, 0}};
const Note S_TICK[] = {{3000, 15}, {0, 0}};

struct Sound { const char *name; const Note *notes; };
const Sound SOUNDS[] = {
  {"ack", S_ACK}, {"happy", S_HAPPY}, {"warn", S_WARN}, {"womp", S_WOMP}, {"party", S_PARTY},
  {"frantic", S_FRANTIC}, {"trombone", S_TROMBONE}, {"robot", S_ROBOT}, {"saved", S_SAVED}, {"tick", S_TICK},
};
const int SOUND_COUNT = sizeof(SOUNDS) / sizeof(SOUNDS[0]);
volatile int pendingSound = -1;
const Note *playing = nullptr;
int noteIdx = 0;
unsigned long noteEndMs = 0;

// ================= Bridge handlers =================

String read_sensors() {
  return String(thermoOk ? 1 : 0) + "," + String((float)lastTemp, 2) + "," + String((float)lastHum, 1) + "," +
         String((long)knobFree) + "," + String((long)knobHeld) + "," + String((long)clicks) + "," +
         String((long)doubleClicks) + "," + String((long)longPresses);
}

void set_display(String spec) {
  strncpy(displayIn, spec.c_str(), sizeof(displayIn) - 1);
  displayIn[sizeof(displayIn) - 1] = 0;
  displaySeq++;
}

void play(String name) {
  for (int i = 0; i < SOUND_COUNT; i++) {
    if (name == SOUNDS[i].name) {
      pendingSound = i;
      return;
    }
  }
}

void beep(int freq, int ms) {
  if (buzzerOk) buzzer.tone(freq, ms);
}

// ================= Knob =================

void updateKnob(unsigned long now) {
  if (!knobOk) return;
  int pos = knob.get();
  bool pressed = knob.isPressed();
  int delta = pos - lastKnobPos;
  lastKnobPos = pos;

  if (pressed && !wasPressed) {
    pressStartMs = now;
    rotatedWhilePressed = false;
    longFired = false;
  }
  if (delta != 0) {
    if (pressed) {
      knobHeld += delta;
      rotatedWhilePressed = true;
    } else {
      knobFree += delta;
    }
  }
  if (pressed && !rotatedWhilePressed && !longFired && now - pressStartMs >= LONG_PRESS_MS) {
    longFired = true;
    longPresses++;
    pendingClick = false;
  }
  if (!pressed && wasPressed && !rotatedWhilePressed && !longFired) {
    if (pendingClick && now - firstClickMs < DOUBLE_CLICK_MS) {
      doubleClicks++;
      pendingClick = false;
    } else {
      pendingClick = true;
      firstClickMs = now;
    }
  }
  if (pendingClick && !pressed && now - firstClickMs >= DOUBLE_CLICK_MS) {
    clicks++;
    pendingClick = false;
  }
  wasPressed = pressed;
}

// ================= Buzzer =================

void updateSound(unsigned long now) {
  int req = pendingSound;
  if (req >= 0) {
    pendingSound = -1;
    playing = SOUNDS[req].notes;
    noteIdx = 0;
    noteEndMs = now;
  }
  if (playing == nullptr || now < noteEndMs) return;
  const Note &n = playing[noteIdx];
  if (n.ms == 0) {
    playing = nullptr;
    return;
  }
  if (n.f > 0 && buzzerOk) buzzer.tone(n.f, n.ms);
  noteEndMs = now + n.ms;
  noteIdx++;
}

// ================= Display =================

void clearFrame() { memset(frame, 0, sizeof(frame)); }

void setPx(int x, int y, uint8_t v) {
  if (x < 0 || x >= FRAME_COLS || y < 0 || y >= FRAME_ROWS) return;
  if (v > frame[y * FRAME_COLS + x]) frame[y * FRAME_COLS + x] = v;
}

void drawIcon(const char *const *icon) {
  for (int y = 0; y < FRAME_ROWS; y++)
    for (int x = 0; x < FRAME_COLS; x++) {
      char c = icon[y][x];
      setPx(x, y, c == '#' ? 7 : (c == '+' ? 2 : 0));
    }
}

int textWidth(const char *s) {
  int n = strlen(s);
  return n == 0 ? 0 : n * 4 - 1;
}

void drawText(const char *s, int x0, int y0) {
  for (int i = 0; s[i]; i++) {
    const uint8_t *g = glyph(s[i]);
    for (int r = 0; r < 5; r++)
      for (int c = 0; c < 3; c++)
        if (g[r] & (4 >> c)) setPx(x0 + i * 4 + c, y0 + r, 7);
  }
}

// Draws text centred, or scrolling when wider than the matrix
void drawTextAuto(const char *s, unsigned long t) {
  int w = textWidth(s);
  if (w <= FRAME_COLS) {
    drawText(s, (FRAME_COLS - w) / 2, 1);
  } else {
    int span = w + FRAME_COLS;
    int off = (t / 90) % span;
    drawText(s, FRAME_COLS - off, 1);
  }
}

void drawSpinner(unsigned long t) {
  // Comet running around a rectangle (x 4..8, y 1..6)
  static int px[18], py[18];
  static bool init = false;
  if (!init) {
    int i = 0;
    for (int x = 4; x <= 8; x++) { px[i] = x; py[i++] = 1; }
    for (int y = 2; y <= 6; y++) { px[i] = 8; py[i++] = y; }
    for (int x = 7; x >= 4; x--) { px[i] = x; py[i++] = 6; }
    for (int y = 5; y >= 2; y--) { px[i] = 4; py[i++] = y; }
    init = true;
  }
  int head = (t / 60) % 18;
  const uint8_t trail[] = {7, 5, 3, 2, 1};
  for (int k = 0; k < 5; k++) {
    int i = (head - k + 18) % 18;
    setPx(px[i], py[i], trail[k]);
  }
}

void drawClock(unsigned long t) {
  // Fast-ticking clock for Espresso Sprint Mode
  const int dx[] = {0, 1, 1, 1, 0, -1, -1, -1};
  const int dy[] = {-1, -1, 0, 1, 1, 1, 0, -1};
  int cx = 6, cy = 4;
  setPx(6, 0, 2); setPx(10, 4, 2); setPx(2, 4, 2); setPx(6, 7, 2);
  setPx(cx, cy, 7);
  int d = (t / 125) % 8;
  for (int l = 1; l <= 3; l++) setPx(cx + dx[d] * l, cy + dy[d] * l, 7);
}

const char *const *iconByName(const char *name) {
  for (int i = 0; i < ICON_COUNT; i++)
    if (strcmp(name, ICONS[i].name) == 0) return ICONS[i].rows;
  return ICON_CUP;
}

void updateDisplay(unsigned long now) {
  if (displaySeq != displaySeen) {
    displaySeen = displaySeq;
    char buf[96];
    strncpy(buf, displayIn, sizeof(buf));
    buf[sizeof(buf) - 1] = 0;
    char *sep = strchr(buf, ':');
    if (sep) {
      *sep = 0;
      strncpy(dispArg, sep + 1, sizeof(dispArg) - 1);
    } else {
      dispArg[0] = 0;
    }
    dispArg[sizeof(dispArg) - 1] = 0;
    strncpy(dispMode, buf, sizeof(dispMode) - 1);
    dispMode[sizeof(dispMode) - 1] = 0;
    for (char *p = dispArg; *p; p++) *p = toupper(*p);
    dispStartMs = now;
    lastFrameMs = 0;
  }
  if (now - lastFrameMs < 40) return;
  lastFrameMs = now;
  unsigned long t = now - dispStartMs;
  bool blinkOn = (t / 350) % 2 == 0;

  clearFrame();
  if (strcmp(dispMode, "icon") == 0) {
    drawIcon(iconByName(dispArg));
  } else if (strcmp(dispMode, "blink") == 0) {
    if (blinkOn) drawIcon(iconByName(dispArg));
  } else if (strcmp(dispMode, "text") == 0) {
    drawTextAuto(dispArg, t);
  } else if (strcmp(dispMode, "blinktext") == 0) {
    if (blinkOn || textWidth(dispArg) > FRAME_COLS) drawTextAuto(dispArg, t);
  } else if (strcmp(dispMode, "spin") == 0) {
    drawSpinner(t);
  } else if (strcmp(dispMode, "clock") == 0) {
    drawClock(t);
  }
  matrix.draw(frame);
}

// ================= Setup / loop =================

void setup() {
  Serial.begin(115200);
  Modulino.begin();
  thermoOk = thermo.begin();
  knobOk = knob.begin();
  buzzerOk = buzzer.begin();
  if (knobOk) {
    knob.set(0);
    lastKnobPos = 0;
  }
  matrix.begin();
  matrix.setGrayscaleBits(3);
  matrix.clear();
  Serial.println(String("thermo=") + thermoOk + " knob=" + knobOk + " buzzer=" + buzzerOk);

  Bridge.begin();
  Bridge.provide("read_sensors", read_sensors);
  Bridge.provide("set_display", set_display);
  Bridge.provide("play", play);
  Bridge.provide("beep", beep);
}

void loop() {
  unsigned long now = millis();
  if (!thermoOk && now - lastThermoMs >= 2000) {  // not found at boot (loose cable, slow power-up): keep retrying
    lastThermoMs = now;
    thermoOk = thermo.begin();
    if (thermoOk) Serial.println("thermo found");
  }
  if (thermoOk && now - lastThermoMs >= 250) {
    lastThermoMs = now;
    lastTemp = thermo.getTemperature();
    lastHum = thermo.getHumidity();
  }
  updateKnob(now);
  updateSound(now);
  updateDisplay(now);
  delay(10);
}
