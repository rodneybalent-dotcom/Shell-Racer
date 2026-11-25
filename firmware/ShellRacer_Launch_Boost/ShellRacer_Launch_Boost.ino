#include <Arduino.h>
#include <esp_arduino_version.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <esp_system.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <atomic>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// ---------- PINS: labels on the Nano header ----------
const uint8_t STEER_IN = D2;
const uint8_t THROTTLE_IN = D3;
const uint8_t CH3_IN = D4;
const uint8_t STEER_OUT = D8;
const uint8_t ESC_OUT = D9;
const uint8_t SPEED_LED_PIN = D5;
const uint8_t I2C_SDA_PIN = A4;
const uint8_t I2C_SCL_PIN = A5;
const uint8_t CHARGE_LED_1 = A1;
const uint8_t CHARGE_LED_2 = A2;
const uint8_t CHARGE_LED_3 = A3;
// Power-up animation: LED 1, then LED 2, then LED 3; all OFF after 3 seconds.
const uint32_t STARTUP_LED_STEP_MS = 1000;

// ---------- CAR NAME / NETWORK SETTINGS ----------
// Change CAR_NAME for each car in your fleet.
const char CAR_NAME[] = "Green Shell";
const char WIFI_SSID[] = "YOUR_WIFI_SSID";
const char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
const char NODE_RED_URL[] = "http://YOUR_NODE_RED_HOST:1880/shellracer/telemetry";
const bool WIFI_UPLOAD_ENABLED = true;
// HTTP transport runs on a background task and never blocks the RC loop.
const uint32_t WIFI_RETRY_MS = 5000;
const uint32_t HTTP_TIMEOUT_MS = 700;
const size_t TELEMETRY_JSON_MAX = 1792;
const uint8_t TELEMETRY_QUEUE_DEPTH = 5;

// ---------- CONFIGURATION ----------
const bool SERIAL_ENABLED = true;
const bool ENABLE_COLOUR_SENSOR = true;  // Set false to bypass sensor while troubleshooting.
const bool CH3_BOOST_ENABLED = false;
const uint16_t CH3_BOOST_THRESHOLD_US = 1600;

// Receiver pulse widths (measured in your serial tests).
const int RX_NEUTRAL_US = 1485;  // Match the controller with the trigger released.
const int RX_FORWARD_MAX_US = 2075;
const int RX_REVERSE_MIN_US = 1250;
const int RX_DEADZONE_US = 25;
const int ARM_DEADZONE_US = 25;
// ESC pulse widths: independent of the receiver's much wider input range.
const int ESC_NEUTRAL_US = 1485;  // Neutral pulse verified with the ESC connected.
const int ESC_FORWARD_MAX_US = 1650;
const int ESC_BOOST_MAX_US = 1720;
const int ESC_REVERSE_MIN_US = 1250;
const int NORMAL_FORWARD_PERCENT = 100;
const int NORMAL_REVERSE_PERCENT = 50;
const int THROTTLE_STABILITY_US = 10;

const int STEER_CENTER_US = 1485;
const int STEER_LOW_TRAVEL_US = 400;   // Travel for receiver pulses below centre. RIGHT TURN Higher Turns more
const int STEER_HIGH_TRAVEL_US = 330;  // Travel for receiver pulses above centre. LEFT TURN  Lower Turns more
static_assert(STEER_LOW_TRAVEL_US > 0 && STEER_HIGH_TRAVEL_US > 0,
              "Steering travel must be positive");
const int CORNER_SLOWDOWN_PERCENT = 40;
const int BOOST_CORNER_SLOWDOWN_PERCENT = 30;
const int CORNER_START_US = 150;
const int BOOST_CORNER_START_US = 80;
const int GREEN_SLOWDOWN_PERCENT = 20;
const uint32_t GREEN_SLOWDOWN_MS = 1500;
const uint32_t BOOST_MS = 1000;
const uint8_t YELLOW_HITS_REQUIRED = 2;

const uint8_t Y_R_MIN = 210, Y_R_MAX = 255;
const uint8_t Y_G_MIN = 200, Y_G_MAX = 255;
const uint8_t Y_B_MIN = 60, Y_B_MAX = 180;
const uint8_t G_R_MIN = 100, G_R_MAX = 200;
const uint8_t G_G_MIN = 150, G_G_MAX = 255;
const uint8_t G_B_MIN = 140, G_B_MAX = 200;
// Red thresholds from measured samples (R=255, G=56..102, B=51..102).
const uint8_t R_R_MIN = 245, R_R_MAX = 255;
const uint8_t R_G_MIN = 140,  R_G_MAX = 170;
const uint8_t R_B_MIN = 160,  R_B_MAX = 180;
const uint32_t RED_CHARGE_LOCKOUT_MS = 1000;
const uint32_t RED_BOOST_MS = 4000;  // Red power mode duration
const uint32_t RED_FLASH_INTERVAL_MS = 250; // 250ms ON, 250ms OFF
const uint8_t RED_POWER_EXTRA_PERCENT = 30; // +10% commanded forward output above neutral

const uint32_t RC_TIMEOUT_US = 150000; // Keep real failsafe at 150 ms.
const uint16_t RC_PULSE_MIN_US = 900;
const uint16_t RC_PULSE_MAX_US = 2100;
const uint32_t ARM_STABLE_MS = 500;
const uint32_t SERIAL_INTERVAL_MS = 500; // 2 JSON telemetry records/second; adjustable
const uint32_t COLOUR_SAMPLE_MS = 50; // Read the TCS34725 at 20 Hz.
const uint8_t TCS34725_I2C_ADDRESS = 0x29;
const uint32_t I2C_TIMEOUT_MS = 20;
const uint32_t REVERSE_BRAKE_MS = 400;
const uint32_t REVERSE_NEUTRAL_MS = 120;
const int REVERSE_BRAKE_PULSE_US = 1300; // Requires ESC brake/reverse support.
const uint32_t SPEED_LED_PWM_HZ = 200;
const uint8_t SPEED_LED_PWM_BITS = 8;
const uint8_t SPEED_LED_PWM_CHANNEL = 7;
const int SPEED_LED_DEADZONE_US = 20;

Servo steering, esc;
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);
bool colourOK = false;
bool ledReady = false;
uint8_t telemetryLedDuty = 0; // Actual requested PWM duty (0..255)

// Fleet settings are passed from the Wi-Fi worker to the control loop by value.
struct FleetSettings {
  int speed, yellowExtra, yellowMs, greenPercent, greenMs, turn, boostTurn;
  int rgb[9];
  int revision;
  unsigned long long flashId;
  uint32_t flashStartMs;
  int racePhase;
  unsigned long long raceId;
  int blueRgb[3];
  uint32_t raceRequestMs;
  bool raceValid;
  bool raceLightsWaiting;
  bool raceWinner;
  uint8_t stripsPerLap;
  uint16_t launchBoostRemainingMs;
  uint32_t launchBoostReceivedMs;
};
FleetSettings fleet = {100, ESC_BOOST_MAX_US - ESC_FORWARD_MAX_US, BOOST_MS,
  GREEN_SLOWDOWN_PERCENT, GREEN_SLOWDOWN_MS, CORNER_SLOWDOWN_PERCENT,
  BOOST_CORNER_SLOWDOWN_PERCENT, {-1,-1,-1,-1,-1,-1,-1,-1,-1}, 0};
// Race-enabled cars require a fresh fleet response before accepting throttle.
const bool RACE_MODE_ENABLED = true;
const uint32_t RACE_LINK_TIMEOUT_MS = 1800;
const uint32_t RACE_FAST_POLL_MS = 100;
const uint16_t RACE_FAST_HTTP_TIMEOUT_MS = 120;
enum RacePhase : int { RACE_OFF, RACE_STAGE, RACE_COUNTDOWN, RACE_RUNNING, RACE_FINISHED };
bool raceLocked = RACE_MODE_ENABLED;
bool raceStagingLights = false;
bool blue = false;
unsigned long long lapRaceId = 0;
uint32_t raceLaps = 0;
uint32_t raceBoot = 0;
uint32_t lastLapMs = 0;
uint32_t lastStripMs = 0;
uint8_t lapStrips = 0;
uint32_t latestLapDurationMs = 0, fastestLapDurationMs = 0;
unsigned long long lastWinnerStrobeId = 0;
uint32_t winnerStrobeStartedMs = 0;
bool winnerStrobeActive = false;
uint8_t blueClearSamples = 0;
QueueHandle_t fleetQueue = nullptr;

// iPad drive runs separately from fleet settings and telemetry.
const uint32_t DRIVE_WATCHDOG_MS = 220;
const int DRIVE_MAX_FORWARD_PERCENT = 30; // Percentage of normal ESC forward travel.
enum DriveMode : uint8_t { DRIVE_RC, DRIVE_IPAD, DRIVE_STOPPED };
struct RemoteDriveCommand {
  unsigned long long id;
  uint32_t requestedAt;
  int mode, steer, throttle;
};
QueueHandle_t remoteDriveQueue = nullptr;
std::atomic<unsigned int> remoteDriveStatus{0};
DriveMode remoteDriveMode = DRIVE_RC;
unsigned long long remoteDriveId = 0;
RemoteDriveCommand remoteDriveCommand = {};

void remoteDriveWorker(void *) {
  String url(NODE_RED_URL);
  int slash = url.lastIndexOf('/');
  if (slash < 0) { vTaskDelete(nullptr); return; }
  url = url.substring(0, slash) + "/drive";
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      WiFiClient client;
      HTTPClient http;
      http.setConnectTimeout(90);
      http.setTimeout(90);
      if (http.begin(client, url)) {
        unsigned int state = remoteDriveStatus.load();
        char body[128];
        snprintf(body, sizeof(body), "{\"action\":\"car\",\"car\":\"%s\",\"mode\":%u,\"ready\":%s}",
                 CAR_NAME, state & 3U, (state & 4U) ? "true" : "false");
        http.addHeader("Content-Type", "application/json");
        uint32_t requestedAt = millis();
        int status = http.POST((uint8_t *)body, strlen(body));
        if (status == 200 && http.getSize() > 0 && http.getSize() < 64) {
          String response = http.getString();
          RemoteDriveCommand command = {};
          int parsed = 0;
          command.requestedAt = requestedAt;
          if (sscanf(response.c_str(), "D1 %1d %16llu %4d %3d%n", &command.mode, &command.id,
                     &command.steer, &command.throttle, &parsed) == 4 &&
              parsed == (int)response.length() && command.mode >= 0 && command.mode <= 3 &&
              abs(command.steer) <= 100 && command.throttle >= 0 && command.throttle <= 100 &&
              millis() - requestedAt < DRIVE_WATCHDOG_MS) {
            xQueueOverwrite(remoteDriveQueue, &command);
          }
        }
        http.end();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(45));
  }
}

void startRemoteDrive() {
  if (!WIFI_UPLOAD_ENABLED) return;
  remoteDriveQueue = xQueueCreate(1, sizeof(RemoteDriveCommand));
  if (remoteDriveQueue) xTaskCreate(remoteDriveWorker, "ipad-drive", 6144, nullptr, 1, nullptr);
}

void pollFleet(const char *packet, int &lastRacePhase, bool fast) {
  WiFiClient client;
  HTTPClient http;
  String url(NODE_RED_URL);
  int slash = url.lastIndexOf('/');
  if (slash < 0 || !fleetQueue) return;
  url = url.substring(0, slash) + "/fleet-sync";
  const uint16_t timeoutMs = fast ? RACE_FAST_HTTP_TIMEOUT_MS : HTTP_TIMEOUT_MS;
  http.setTimeout(timeoutMs);
  http.setConnectTimeout(timeoutMs);
  if (!http.begin(client, url)) return;
  const char *fleetHeaders[] = {"X-ShellRacer-Flash", "X-ShellRacer-Race", "X-ShellRacer-Race-Lights", "X-ShellRacer-Winner", "X-ShellRacer-Strips", "X-ShellRacer-Launch-Boost"};
  http.collectHeaders(fleetHeaders, 6);
  http.addHeader("X-ShellRacer-Race", "1");
  http.addHeader("X-ShellRacer-Flash", "1");
  http.addHeader("X-ShellRacer-Race-Lights", "1");
  http.addHeader("X-ShellRacer-Strips", "1");
  http.addHeader("X-ShellRacer-Launch-Boost", "1");
  http.addHeader("Content-Type", "application/json");
  const uint32_t requestMs = millis();
  int status = http.POST((uint8_t*)packet, strlen(packet));
  const uint32_t responseMs = millis();
  if (status == 200 && http.getSize() > 0 && http.getSize() <= 160) {
    String body = http.getString();
    int v[17], end = 0;
    int count = sscanf(body.c_str(),
      "SR1 %9d %3d %3d %5d %3d %5d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d%n",
      &v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6],&v[7],
      &v[8],&v[9],&v[10],&v[11],&v[12],&v[13],&v[14],&v[15],&v[16],&end);
    const int upper[] = {999999999,100,ESC_BOOST_MAX_US-ESC_FORWARD_MAX_US,10000,100,10000,100,100};
    bool valid = count == 17 && end == (int)body.length();
    for (int i=0; valid && i<8; ++i) valid = v[i]>=0 && v[i]<=upper[i];
    for (int i=8; valid && i<17; i+=3) {
      bool disabled = v[i]==-1 && v[i+1]==-1 && v[i+2]==-1;
      bool rgb = true;
      for (int j=i;j<i+3;++j) rgb &= v[j]>=0 && v[j]<=255;
      valid = disabled || rgb;
    }
    if (valid) {
      FleetSettings next = {v[1],v[2],v[3],v[4],v[5],v[6],v[7],{},v[0]};
      memcpy(next.rgb, v+8, sizeof(next.rgb));
      const String command = http.header("X-ShellRacer-Flash");
      unsigned long long id = 0;
      unsigned int waitMs = 0;
      int parsed = 0;
      const uint32_t estimatedTransitMs = (responseMs - requestMs) / 2;
      if (sscanf(command.c_str(), "%16llu %4u%n", &id, &waitMs, &parsed) == 2 &&
          parsed == (int)command.length() && id > 0 &&
          waitMs <= 2500 && waitMs > estimatedTransitMs) {
        next.flashId = id;
        next.flashStartMs = responseMs + waitMs - estimatedTransitMs;
      }
      const String race = http.header("X-ShellRacer-Race");
      int consumed = 0;
      next.raceValid = sscanf(race.c_str(), "R1 %1d %16llu %3d %3d %3d%n",
        &next.racePhase, &next.raceId, &next.blueRgb[0], &next.blueRgb[1],
        &next.blueRgb[2], &consumed) == 5 && consumed == (int)race.length() &&
        next.racePhase >= RACE_OFF && next.racePhase <= RACE_FINISHED &&
        (next.racePhase == RACE_OFF || next.raceId > 0);
      const bool unsetBlue = next.blueRgb[0] == -1 && next.blueRgb[1] == -1 && next.blueRgb[2] == -1;
      const bool validBlue = next.blueRgb[0] >= 0 && next.blueRgb[0] <= 255 &&
        next.blueRgb[1] >= 0 && next.blueRgb[1] <= 255 &&
        next.blueRgb[2] >= 0 && next.blueRgb[2] <= 255;
      next.raceValid = next.raceValid && (unsetBlue || validBlue);
      next.raceRequestMs = requestMs;
      const String raceLights = http.header("X-ShellRacer-Race-Lights");
      const long strips = http.header("X-ShellRacer-Strips").toInt();
      next.stripsPerLap = strips >= 1 && strips <= 12 ? strips : 1;
      const long remaining = http.header("X-ShellRacer-Launch-Boost").toInt();
      if (remaining > 0 && remaining <= 10000 && (uint32_t)remaining > estimatedTransitMs) {
        next.launchBoostRemainingMs = remaining - estimatedTransitMs;
        next.launchBoostReceivedMs = responseMs;
      }
      next.raceWinner = http.header("X-ShellRacer-Winner") == "1";
      next.raceLightsWaiting = raceLights == "stage" || raceLights == "countdown";
      if (next.raceValid) {
        lastRacePhase = raceLights == "countdown" ? RACE_COUNTDOWN : next.racePhase;
      }
      xQueueOverwrite(fleetQueue, &next);
    }
  }
  http.end();
}

// ---------- ASYNCHRONOUS WI-FI TELEMETRY ----------
// Do NOT call HTTPClient in loop(); a slow request could hold the ESC update.
// JSONL goes to Serial and is copied into a bounded queue for the Wi-Fi task.
struct TelemetryPacket { char json[TELEMETRY_JSON_MAX]; };
QueueHandle_t telemetryQueue = nullptr;
volatile uint32_t telemetryDropped = 0;
volatile uint32_t telemetrySent = 0;
volatile uint32_t telemetryHttpErrors = 0;

void sendJsonLine(const char *json) {
  if (SERIAL_ENABLED) Serial.println(json);
  if (WIFI_UPLOAD_ENABLED && telemetryQueue != nullptr) {
    TelemetryPacket packet = {};
    size_t n = strlen(json);
    if (n >= sizeof(packet.json)) { ++telemetryDropped; return; }
    memcpy(packet.json, json, n + 1);
    if (xQueueSend(telemetryQueue, &packet, 0) != pdTRUE) ++telemetryDropped;
  }
}

void wifiTelemetryTask(void *) {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t lastFleetPoll = 0;
  uint32_t lastFastPoll = 0;
  int lastRacePhase = RACE_OFF;
  uint32_t lastAttempt = millis();
  TelemetryPacket packet;
  for (;;) {
    if (WiFi.status() != WL_CONNECTED &&
        (uint32_t)(millis() - lastAttempt) >= WIFI_RETRY_MS) {
      lastAttempt = millis();
      WiFi.reconnect();
    }
    if (lastRacePhase == RACE_COUNTDOWN && WiFi.status() == WL_CONNECTED &&
        (uint32_t)(millis() - lastFastPoll) >= RACE_FAST_POLL_MS) {
      lastFastPoll = millis();
      pollFleet("{\"type\":\"race-sync\"}", lastRacePhase, true);
    }
    const bool countdown = lastRacePhase == RACE_COUNTDOWN;
    if (xQueueReceive(telemetryQueue, &packet, pdMS_TO_TICKS(countdown ? 25 : 100)) != pdTRUE) continue;
    // Drop stale data instead of blocking control or building an endless backlog.
    if (WiFi.status() != WL_CONNECTED) { ++telemetryDropped; continue; }
    WiFiClient client;
    HTTPClient http;
    const uint16_t timeoutMs = countdown ? RACE_FAST_HTTP_TIMEOUT_MS : HTTP_TIMEOUT_MS;
    http.setTimeout(timeoutMs);
    http.setConnectTimeout(timeoutMs);
    if (!http.begin(client, NODE_RED_URL)) { ++telemetryHttpErrors; continue; }
    http.addHeader("Content-Type", "application/json");
    int result = http.POST((uint8_t *)packet.json, strlen(packet.json));
    http.end();
    if (result >= 200 && result < 300) ++telemetrySent;
    else ++telemetryHttpErrors;
    if (strstr(packet.json, "\"type\":\"telemetry\"") &&
        (uint32_t)(millis()-lastFleetPoll) >= 1000) {
      lastFleetPoll = millis();
      pollFleet(packet.json, lastRacePhase, countdown);
    }
  }
}

void startWifiTelemetry() {
  if (!WIFI_UPLOAD_ENABLED) return;
  telemetryQueue = xQueueCreate(TELEMETRY_QUEUE_DEPTH, sizeof(TelemetryPacket));
  if (telemetryQueue == nullptr) return;
  // Core 0 handles network calls; receiver ISR / drive loop remain independent.
  if (xTaskCreatePinnedToCore(wifiTelemetryTask, "shellracer_wifi", 8192,
                              nullptr, 1, nullptr, 0) != pdPASS) {
    vQueueDelete(telemetryQueue);
    telemetryQueue = nullptr;
  }
}

// ---------- INPUT CAPTURE ----------
// No ISR spinlock: each interrupt does minimal work, and the main loop copies
// the three captures with interrupts briefly disabled.
// Count valid and invalid widths to distinguish a missing edge from bad pulses.
struct PulseCapture {
  volatile uint32_t riseUs = 0;
  volatile uint32_t lastValidUs = 0;
  volatile uint16_t widthUs = 0;
  volatile uint32_t validCount = 0;
  volatile uint32_t invalidCount = 0;
};
PulseCapture steeringRx, throttleRx, ch3Rx;

void IRAM_ATTR capturePulse(PulseCapture &p, uint8_t pin) {
  uint32_t nowUs = micros();
  if (digitalRead(pin)) {
    p.riseUs = nowUs;
  } else {
    uint32_t rise = p.riseUs;
    p.riseUs = 0;
    if (!rise) return;
    uint32_t width = nowUs - rise;
    if (width >= RC_PULSE_MIN_US && width <= RC_PULSE_MAX_US) {
      p.widthUs = (uint16_t)width;
      p.lastValidUs = nowUs;
      ++p.validCount;
    } else {
      ++p.invalidCount;
    }
  }
}
void IRAM_ATTR steerISR() { capturePulse(steeringRx, STEER_IN); }
void IRAM_ATTR throttleISR() { capturePulse(throttleRx, THROTTLE_IN); }
void IRAM_ATTR ch3ISR() { capturePulse(ch3Rx, CH3_IN); }

struct RcChannel {
  uint16_t pulse = 0;
  uint32_t ageUs = 0;
  uint32_t validCount = 0;
  uint32_t invalidCount = 0;
  bool ok = false;
};
struct RcReading {
  RcChannel steer;
  RcChannel throttle;
  RcChannel ch3;
};
// Explicit prototypes stop Arduino's sketch preprocessor from forward-declaring
// functions before the structs above.
RcReading readReceiver();
void printDiagnostics(const RcReading &rc, const char *state, int escOutput);
void updateRaceNeutral(const RcReading &rc, uint32_t now, bool raceOnline);
RcChannel makeChannel(uint16_t width, uint32_t last, uint32_t valid, uint32_t invalid, uint32_t nowUs);

RcChannel makeChannel(uint16_t width, uint32_t last, uint32_t valid, uint32_t invalid, uint32_t nowUs) {
  RcChannel ch;
  ch.pulse = width;
  ch.ageUs = last ? (uint32_t)(nowUs - last) : 0xFFFFFFFFUL;
  ch.validCount = valid;
  ch.invalidCount = invalid;
  ch.ok = last != 0 && ch.ageUs < RC_TIMEOUT_US;
  return ch;
}
RcReading readReceiver() {
  uint16_t sWidth, tWidth, cWidth;
  uint32_t sLast, tLast, cLast, sValid, tValid, cValid, sBad, tBad, cBad;
  noInterrupts();
  sWidth = steeringRx.widthUs; sLast = steeringRx.lastValidUs;
  sValid = steeringRx.validCount; sBad = steeringRx.invalidCount;
  tWidth = throttleRx.widthUs; tLast = throttleRx.lastValidUs;
  tValid = throttleRx.validCount; tBad = throttleRx.invalidCount;
  cWidth = ch3Rx.widthUs; cLast = ch3Rx.lastValidUs;
  cValid = ch3Rx.validCount; cBad = ch3Rx.invalidCount;
  interrupts();
  uint32_t nowUs = micros();
  RcReading rc;
  rc.steer = makeChannel(sWidth, sLast, sValid, sBad, nowUs);
  rc.throttle = makeChannel(tWidth, tLast, tValid, tBad, nowUs);
  rc.ch3 = makeChannel(cWidth, cLast, cValid, cBad, nowUs);
  return rc;
}

// ---------- TIMERS ----------
struct OneShotTimer {
  uint32_t startMs = 0, durationMs = 0;
  bool running = false;
  void start(uint32_t now, uint32_t duration) { startMs = now; durationMs = duration; running = true; }
  void stop() { running = false; }
  bool active(uint32_t now) {
    if (running && (uint32_t)(now - startMs) >= durationMs) running = false;
    return running;
  }
};
OneShotTimer boostTimer, greenTimer;

// ---------- COLOUR ----------
// Sensor absence leaves steering/throttle working. Colour triggers remain inactive.
uint8_t r8 = 0, g8 = 0, b8 = 0;
bool yellow = false, green = false, red = false, yellowLatched = false;
uint8_t yellowHits = 0;
uint32_t lastColourMs = 0;
uint32_t lastColourHealthyMs = 0;
uint32_t colourReadCount = 0;
uint32_t colourReadSlowCount = 0;
bool inRange(uint8_t x, uint8_t lo, uint8_t hi) { return x >= lo && x <= hi; }
void readColour() {
  uint32_t startedMs = millis();
  uint16_t r, g, b, clear;
  tcs.getRawData(&r, &g, &b, &clear);
  uint16_t maximum = max(r, max(g, b));
  if (!clear || !maximum) {
    ++colourReadCount;
    if ((uint32_t)(millis() - startedMs) > 40) ++colourReadSlowCount;
    lastColourHealthyMs = millis();
    r8 = g8 = b8 = 0; yellow = green = red = blue = false;
    return;
  }
  r8 = (uint32_t)r * 255 / maximum;
  g8 = (uint32_t)g * 255 / maximum;
  b8 = (uint32_t)b * 255 / maximum;
  ++colourReadCount;
  if ((uint32_t)(millis() - startedMs) > 40) ++colourReadSlowCount;
  lastColourHealthyMs = millis();
  yellow = inRange(r8, Y_R_MIN, Y_R_MAX) && inRange(g8, Y_G_MIN, Y_G_MAX) && inRange(b8, Y_B_MIN, Y_B_MAX);
  green = inRange(r8, G_R_MIN, G_R_MAX) && inRange(g8, G_G_MIN, G_G_MAX) && inRange(b8, G_B_MIN, G_B_MAX);
  red = inRange(r8, R_R_MIN, R_R_MAX) && inRange(g8, R_G_MIN, R_G_MAX) && inRange(b8, R_B_MIN, R_B_MAX);
  bool *flags[] = {&yellow,&green,&red};
  for (int i=0;i<3;++i) {
    const int *ref = fleet.rgb + 3*i;
    if (ref[0]>=0) *flags[i] = abs((int)r8-ref[0])<=15 &&
      abs((int)g8-ref[1])<=15 && abs((int)b8-ref[2])<=15;
  }
  blue = fleet.raceValid && fleet.blueRgb[0] >= 0
    ? abs((int)r8-fleet.blueRgb[0])<=15 && abs((int)g8-fleet.blueRgb[1])<=15 &&
      abs((int)b8-fleet.blueRgb[2])<=15
    : b8 == 255 && r8 >= 45 && r8 <= 105 && g8 >= 105 && g8 <= 190;
  if (blue) yellow = green = red = false;
  else if (yellow) green = false;
}

// ---------- SPEED LED ----------
void setLed(uint8_t duty) {
  telemetryLedDuty = ledReady ? duty : 0;
  if (!ledReady) { digitalWrite(SPEED_LED_PIN, LOW); return; }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SPEED_LED_PIN, duty);
#else
  ledcWrite(SPEED_LED_PWM_CHANNEL, duty);
#endif
}
void setupLed() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledReady = ledcAttach(SPEED_LED_PIN, SPEED_LED_PWM_HZ, SPEED_LED_PWM_BITS);
#else
  double freq = ledcSetup(SPEED_LED_PWM_CHANNEL, SPEED_LED_PWM_HZ, SPEED_LED_PWM_BITS);
  if (freq > 0) { ledcAttachPin(SPEED_LED_PIN, SPEED_LED_PWM_CHANNEL); ledReady = true; }
#endif
  setLed(0);
}
void updateSpeedLed(int output, bool boost, bool armed) {
  int difference = abs(output - ESC_NEUTRAL_US);
  if (!armed || difference <= SPEED_LED_DEADZONE_US) { setLed(0); return; }
  if (boost && output > ESC_NEUTRAL_US) { setLed(255); return; }
  int maximum = output > ESC_NEUTRAL_US ? ESC_FORWARD_MAX_US - ESC_NEUTRAL_US : ESC_NEUTRAL_US - ESC_REVERSE_MIN_US;
  int span = max(1, maximum - SPEED_LED_DEADZONE_US);
  int scaled = constrain((long)(difference - SPEED_LED_DEADZONE_US) * 255 / span, 0L, 255L);
  setLed((uint8_t)(51 + (long)scaled * 153 / 255));
}

// ---------- RED CHARGE LIGHTS (non-blocking) ----------
// Each charge requires a fresh red encounter and respects RED_CHARGE_LOCKOUT_MS.
// The third hit holds power on red, then starts the normal timed boost.
void telemetryEvent(const char *name, const char *detail);
uint8_t redCharges = 0;
bool redSeenPrevious = false;
bool redBoostRunning = false;
bool redHasCharged = false;
uint32_t redBoostStartedMs = 0;  // Start of the LED flash pattern.
bool redBoostHolding = false;
uint32_t lastRedBoostSampleMs = 0;
uint32_t redBoostCountdownMs = 0;
uint32_t lastRedChargeMs = 0;
// Startup animation runs without delay() so receiver processing continues.
bool startupLightsActive = true;
uint32_t startupLightsStartMs = 0;

void updateStartupLights(uint32_t now) {
  if (!startupLightsActive) return;
  uint32_t elapsed = (uint32_t)(now - startupLightsStartMs);
  if (elapsed < STARTUP_LED_STEP_MS) {
    digitalWrite(CHARGE_LED_1, HIGH);
    digitalWrite(CHARGE_LED_2, LOW);
    digitalWrite(CHARGE_LED_3, LOW);
  } else if (elapsed < STARTUP_LED_STEP_MS * 2UL) {
    digitalWrite(CHARGE_LED_1, HIGH);
    digitalWrite(CHARGE_LED_2, HIGH);
    digitalWrite(CHARGE_LED_3, LOW);
  } else if (elapsed < STARTUP_LED_STEP_MS * 3UL) {
    digitalWrite(CHARGE_LED_1, HIGH);
    digitalWrite(CHARGE_LED_2, HIGH);
    digitalWrite(CHARGE_LED_3, HIGH);
  } else {
    digitalWrite(CHARGE_LED_1, LOW);
    digitalWrite(CHARGE_LED_2, LOW);
    digitalWrite(CHARGE_LED_3, LOW);
    startupLightsActive = false;
    telemetryEvent("startup_lights", "complete");
  }
}


const uint32_t FLEET_FLASH_STEP_MS = 250;
const uint32_t FLEET_FLASH_DURATION_MS = 6 * FLEET_FLASH_STEP_MS;
unsigned long long lastFleetFlashId = 0;
uint32_t fleetFlashStartMs = 0;
bool fleetFlashScheduled = false;

uint8_t visibleChargeLeds(uint32_t now) {
  if (!redBoostRunning) return redCharges;
  uint32_t elapsed = (uint32_t)(now - redBoostStartedMs);
  // All three LEDs flash in unison, starting ON at activation.
  return ((elapsed / RED_FLASH_INTERVAL_MS) % 2 == 0) ? 3 : 0;
}
uint32_t launchBoostRemaining(uint32_t now) {
  if (!fleet.raceValid || fleet.racePhase != RACE_RUNNING || raceLocked ||
      now - fleet.raceRequestMs >= RACE_LINK_TIMEOUT_MS) return 0;
  const uint32_t elapsed = now - fleet.launchBoostReceivedMs;
  return elapsed < fleet.launchBoostRemainingMs ? fleet.launchBoostRemainingMs - elapsed : 0;
}
void updateRaceLaps(uint32_t now, bool colourSampled) {
  if (lapRaceId != fleet.raceId) {
    lapRaceId = fleet.raceId;
    raceLaps = 0;
    lapStrips = 0;
    lastStripMs = 0;
    latestLapDurationMs = fastestLapDurationMs = 0;
    blueClearSamples = 0;
  }
  if (raceLocked || fleet.racePhase != RACE_RUNNING) {
    blueClearSamples = 0;
    return;
  }
  if (!colourSampled) return;
  if (!blue) {
    if (blueClearSamples < 2) ++blueClearSamples;
    return;
  }
  const uint8_t stripsPerLap = fleet.stripsPerLap ? fleet.stripsPerLap : 1;
  const uint32_t stripDebounceMs = stripsPerLap == 1 ? 1200 : 150;
  if (blueClearSamples >= 2 && ((!raceLaps && !lapStrips) || now-lastStripMs >= stripDebounceMs)) {
    lastStripMs = now;
    if (++lapStrips < stripsPerLap) {
      blueClearSamples = 0;
      return;
    }
    lapStrips = 0;
    if (raceLaps > 0) {
      latestLapDurationMs = now - lastLapMs;
      if (!fastestLapDurationMs || latestLapDurationMs < fastestLapDurationMs)
        fastestLapDurationMs = latestLapDurationMs;
    }
    ++raceLaps;
    lastLapMs = now;
  }
  blueClearSamples = 0;
}
void showChargeLeds(uint32_t now) {
  if (winnerStrobeActive) {
    // Cumulative on/off steps for LEDs 1, 2 and 3, repeated three times.
    const uint32_t step = (now - winnerStrobeStartedMs) / 120;
    if (step < 18) {
      const uint8_t masks[] = {1, 3, 7, 6, 4, 0};
      const uint8_t mask = masks[step % 6];
      digitalWrite(CHARGE_LED_1, (mask & 1) != 0);
      digitalWrite(CHARGE_LED_2, (mask & 2) != 0);
      digitalWrite(CHARGE_LED_3, (mask & 4) != 0);
      return;
    }
    winnerStrobeActive = false;
  }
  if (raceStagingLights) {
    const bool on = (now / FLEET_FLASH_STEP_MS) % 2 == 0;
    digitalWrite(CHARGE_LED_1, on);
    digitalWrite(CHARGE_LED_2, on);
    digitalWrite(CHARGE_LED_3, on);
    return;
  }
  if (startupLightsActive) return;
  uint8_t lit = visibleChargeLeds(now);
  if (fleetFlashScheduled) {
    const int32_t elapsed = (int32_t)(now - fleetFlashStartMs);
    if (elapsed >= (int32_t)FLEET_FLASH_DURATION_MS) {
      fleetFlashScheduled = false;
    } else if (elapsed >= 0) {
      lit = ((elapsed / FLEET_FLASH_STEP_MS) % 2 == 0) ? 3 : 0;
    }
  }
  digitalWrite(CHARGE_LED_1, lit >= 1 ? HIGH : LOW);
  digitalWrite(CHARGE_LED_2, lit >= 2 ? HIGH : LOW);
  digitalWrite(CHARGE_LED_3, lit >= 3 ? HIGH : LOW);
}
void resetRedCharge() {
  redCharges = 0;
  redSeenPrevious = false;
  redBoostRunning = false;
  redBoostHolding = false;
  redHasCharged = false;
  lastRedChargeMs = 0;
  showChargeLeds(millis());
}
uint32_t redBoostRemaining(uint32_t now) {
  if (!redBoostRunning) return 0;
  if (redBoostHolding) return RED_BOOST_MS;
  const uint32_t elapsed = now - redBoostCountdownMs;
  return elapsed < RED_BOOST_MS ? RED_BOOST_MS - elapsed : 0;
}

void updateRedBoost(uint32_t now, bool colourSampled) {
  if (redBoostHolding) {
    if (colourSampled && red) {
      lastRedBoostSampleMs = now;
    } else if (colourSampled || now - lastRedBoostSampleMs >= 3 * COLOUR_SAMPLE_MS) {
      // Start the countdown once, even if another red strip is crossed.
      redBoostHolding = false;
      redBoostCountdownMs = now;
    }
  }
  if (redBoostRunning && redBoostRemaining(now) == 0) {
    redBoostRunning = false;
    redCharges = 0;
    telemetryEvent("red_power_end", "charge_reset");
  }
  showChargeLeds(now);
}
void updateRedDetection(uint32_t now) {
  bool enteredRed = red && !redSeenPrevious;
  redSeenPrevious = red;
  if (!enteredRed || redBoostRunning) return;
  if (redHasCharged && (uint32_t)(now - lastRedChargeMs) < RED_CHARGE_LOCKOUT_MS) return;
  lastRedChargeMs = now;
  redHasCharged = true;
  if (redCharges < 3) ++redCharges;
  telemetryEvent("red_charge", redCharges == 1 ? "1" : (redCharges == 2 ? "2" : "3"));
  if (redCharges == 3) {
    redBoostRunning = true;
    redBoostStartedMs = now;
    lastRedBoostSampleMs = now;
    redBoostHolding = true;
    greenTimer.stop();
    telemetryEvent("red_power_start", "hold_until_red_clears");
  }
  showChargeLeds(now);
}

// ---------- DRIVE STATE ----------
enum ReversePhase { REV_IDLE, REV_BRAKE, REV_PAUSE };
ReversePhase reversePhase = REV_IDLE;
uint32_t reverseStartedMs = 0;
bool hasDrivenForward = false, reverseUnlocked = false, ch3Latched = false;
bool armed = false;
uint32_t neutralStartMs = 0;
unsigned long long preparedRaceId = 0;
uint32_t failsafeCount = 0;
int lastNormalEscOutput = ESC_NEUTRAL_US;

void resetDriveState() {
  reversePhase = REV_IDLE;
  hasDrivenForward = reverseUnlocked = ch3Latched = false;
  lastNormalEscOutput = ESC_NEUTRAL_US;
  boostTimer.stop(); greenTimer.stop();
  yellowHits = 0; yellowLatched = false;
  resetRedCharge();
}
void stopCar() {
  steering.writeMicroseconds(STEER_CENTER_US);
  esc.writeMicroseconds(ESC_NEUTRAL_US);
  updateSpeedLed(ESC_NEUTRAL_US, false, false);
}
int throttleOutput(int input, bool boost, uint32_t now) {
  if (abs(input - RX_NEUTRAL_US) <= RX_DEADZONE_US) {
    reversePhase = REV_IDLE;
    lastNormalEscOutput = ESC_NEUTRAL_US;
    return ESC_NEUTRAL_US;
  }
  if (input > RX_NEUTRAL_US) {
    hasDrivenForward = true;
    reverseUnlocked = false; reversePhase = REV_IDLE;
    input = constrain(input, RX_NEUTRAL_US, RX_FORWARD_MAX_US);
    int maximum = boost ? ESC_FORWARD_MAX_US + fleet.yellowExtra :
      ESC_NEUTRAL_US + (long)(ESC_FORWARD_MAX_US - ESC_NEUTRAL_US) * NORMAL_FORWARD_PERCENT / 100;
    int result = map(input, RX_NEUTRAL_US, RX_FORWARD_MAX_US, ESC_NEUTRAL_US, maximum);
    if (boost) return result;
    if (abs(result - lastNormalEscOutput) <= THROTTLE_STABILITY_US) return lastNormalEscOutput;
    lastNormalEscOutput = result;
    return result;
  }
  // Automatic reverse-wake sequence for ESCs requiring brake/neutral/reverse.
  if (hasDrivenForward && !reverseUnlocked) {
    if (reversePhase == REV_IDLE) { reversePhase = REV_BRAKE; reverseStartedMs = now; }
    if (reversePhase == REV_BRAKE) {
      if ((uint32_t)(now - reverseStartedMs) < REVERSE_BRAKE_MS) return REVERSE_BRAKE_PULSE_US;
      reversePhase = REV_PAUSE; reverseStartedMs = now;
    }
    if (reversePhase == REV_PAUSE) {
      if ((uint32_t)(now - reverseStartedMs) < REVERSE_NEUTRAL_MS) return ESC_NEUTRAL_US;
      reverseUnlocked = true; reversePhase = REV_IDLE;
    }
  }
  input = constrain(input, RX_REVERSE_MIN_US, RX_NEUTRAL_US);
  int minimum = ESC_NEUTRAL_US - (long)(ESC_NEUTRAL_US - ESC_REVERSE_MIN_US) * NORMAL_REVERSE_PERCENT / 100;
  return map(input, RX_NEUTRAL_US, RX_REVERSE_MIN_US, ESC_NEUTRAL_US, minimum);
}
int limitSteeringPulse(int pulse) {
  return constrain(pulse, STEER_CENTER_US - STEER_LOW_TRAVEL_US,
                   STEER_CENTER_US + STEER_HIGH_TRAVEL_US);
}

int steeringTravelForOffset(int offset) {
  return offset < 0 ? STEER_LOW_TRAVEL_US : STEER_HIGH_TRAVEL_US;
}

int cornerReductionPercent(int steeringOffset, bool boost) {
  int distance = abs(steeringOffset);
  int travel = steeringTravelForOffset(steeringOffset);
  int start = boost ? BOOST_CORNER_START_US : CORNER_START_US;
  if (distance <= start || travel <= start) return 0;
  int position = constrain(map(distance, start, travel, 0, 100), 0, 100);
  return (long)position * (boost ? fleet.boostTurn : fleet.turn) / 100;
}

int applyCornerLimit(int output, int steeringOffset, bool boost) {
  if (output <= ESC_NEUTRAL_US) return output;
  if (abs(steeringOffset) <= (boost ? BOOST_CORNER_START_US : CORNER_START_US)) return output;
  int reduction = cornerReductionPercent(steeringOffset, boost);
  int maximum = boost ? ESC_BOOST_MAX_US : ESC_FORWARD_MAX_US;
  int allowed = ESC_NEUTRAL_US + (long)(maximum - ESC_NEUTRAL_US) * (100 - reduction) / 100;
  return min(output, allowed);
}

// ---------- JSON LINES TELEMETRY ----------
// Every serial line is a standalone JSON object: future Node-RED friendly.
// Do not treat the commanded ESC pulse as measured speed or distance.
// No battery voltage, RPM, GPS position, Wi-Fi or server connection is present.
// The schema version stays constant when telemetry is moved to Wi-Fi later.
void telemetryEvent(const char *name, const char *detail) {
  char json[280];
  snprintf(json, sizeof(json),
    "{\"v\":1,\"car\":\"%s\",\"type\":\"event\",\"ms\":%lu,\"event\":\"%s\",\"detail\":\"%s\"}",
    CAR_NAME, (unsigned long)millis(), name, detail);
  sendJsonLine(json);
}

// Called at most once per configured reporting interval, including when disarmed.
void printDiagnostics(const RcReading &rc, const char *state, int escOutput) {
  static uint32_t lastPrinted = 0;
  static uint32_t prevSteer = 0, prevThrottle = 0, prevCh3 = 0;
  static uint32_t prevSteerBad = 0, prevThrottleBad = 0, prevCh3Bad = 0;
  uint32_t now = millis();
  if ((uint32_t)(now - lastPrinted) < SERIAL_INTERVAL_MS) return;
  const uint32_t interval = lastPrinted ? (uint32_t)(now - lastPrinted) : SERIAL_INTERVAL_MS;
  lastPrinted = now;

  // Pulse deltas per reporting period; approx 36 at a 72 Hz receiver and 500 ms.
  uint32_t steerCount = rc.steer.validCount - prevSteer;
  uint32_t throttleCount = rc.throttle.validCount - prevThrottle;
  uint32_t ch3Count = rc.ch3.validCount - prevCh3;
  uint32_t steerBad = rc.steer.invalidCount - prevSteerBad;
  uint32_t throttleBad = rc.throttle.invalidCount - prevThrottleBad;
  uint32_t ch3Bad = rc.ch3.invalidCount - prevCh3Bad;
  prevSteer = rc.steer.validCount; prevThrottle = rc.throttle.validCount; prevCh3 = rc.ch3.validCount;
  prevSteerBad = rc.steer.invalidCount; prevThrottleBad = rc.throttle.invalidCount; prevCh3Bad = rc.ch3.invalidCount;

  int steeringCommand = armed ? limitSteeringPulse(rc.steer.pulse) : STEER_CENTER_US;
  int steerOffset = steeringCommand - STEER_CENTER_US;
  int steerPct = (long)steerOffset * 100 / steeringTravelForOffset(steerOffset);
  int inputPct = 0;
  if (rc.throttle.ok && rc.throttle.pulse > RX_NEUTRAL_US + RX_DEADZONE_US)
    inputPct = constrain((long)((int)rc.throttle.pulse - RX_NEUTRAL_US) * 100 /
      max(1, RX_FORWARD_MAX_US - RX_NEUTRAL_US), 0L, 100L);
  else if (rc.throttle.ok && rc.throttle.pulse < RX_NEUTRAL_US - RX_DEADZONE_US)
    inputPct = constrain((long)((int)rc.throttle.pulse - RX_NEUTRAL_US) * 100 /
      max(1, RX_NEUTRAL_US - RX_REVERSE_MIN_US), -100L, 0L);

  const uint32_t launchRemaining = armed && inputPct > 0 && escOutput > ESC_NEUTRAL_US
    ? launchBoostRemaining(now) : 0;
  const bool boosting = boostTimer.active(now) || redBoostRunning || launchRemaining > 0;
  const bool slowingGreen = greenTimer.active(now) && !boosting;
  const uint32_t yellowRemaining = boostTimer.active(now) && (uint32_t)(now - boostTimer.startMs) < (uint32_t)fleet.yellowMs
    ? fleet.yellowMs - (uint32_t)(now - boostTimer.startMs) : 0;
  const uint32_t redRemaining = redBoostRemaining(now);
  int cornerSlowdown = escOutput > ESC_NEUTRAL_US ? cornerReductionPercent(steerOffset, boosting) : 0;

  // Ages are null before the first valid pulse, rather than 4294967 ms.
  char steerAge[16], throttleAge[16], ch3Age[16];
  if (rc.steer.ageUs == 0xFFFFFFFFUL) strcpy(steerAge, "null");
  else snprintf(steerAge, sizeof(steerAge), "%lu", (unsigned long)(rc.steer.ageUs / 1000UL));
  if (rc.throttle.ageUs == 0xFFFFFFFFUL) strcpy(throttleAge, "null");
  else snprintf(throttleAge, sizeof(throttleAge), "%lu", (unsigned long)(rc.throttle.ageUs / 1000UL));
  if (rc.ch3.ageUs == 0xFFFFFFFFUL) strcpy(ch3Age, "null");
  else snprintf(ch3Age, sizeof(ch3Age), "%lu", (unsigned long)(rc.ch3.ageUs / 1000UL));

  // One line; a downstream parser can JSON.parse(msg.payload).
  char json[TELEMETRY_JSON_MAX];
  int jsonLength = snprintf(json, sizeof(json),
    "{\"v\":1,\"car\":\"%s\",\"type\":\"telemetry\",\"ms\":%lu,\"period_ms\":%lu,"
    "\"fleet_revision\":%d,\"state\":\"%s\",\"armed\":%s,\"failsafe_count\":%lu,"
    "\"rx\":{"
      "\"steer\":{\"us\":%u,\"ok\":%s,\"age_ms\":%s,\"pulses\":%lu,\"invalid\":%lu},"
      "\"throttle\":{\"us\":%u,\"ok\":%s,\"age_ms\":%s,\"pulses\":%lu,\"invalid\":%lu},"
      "\"ch3\":{\"enabled\":%s,\"us\":%u,\"ok\":%s,\"age_ms\":%s,\"pulses\":%lu,\"invalid\":%lu}},"
    "\"control\":{\"steer_us\":%d,\"steer_pct\":%d,\"throttle_pct\":%d,\"esc_us\":%d,\"esc_neutral_us\":%d,\"reverse_phase\":%u},"
    "\"track\":{\"colour_ok\":%s,\"rgb\":[%u,%u,%u],\"yellow\":%s,\"green\":%s,\"red\":%s,\"blue\":%s,\"colour_reads\":%lu,\"slow_reads\":%lu},"
    "\"effects\":{\"boost\":%s,\"boost_remaining_ms\":%lu,\"green_slowdown\":%s,\"green_remaining_ms\":%lu,\"corner_slowdown_pct\":%d},"
    "\"led\":{\"pwm\":%u,\"pct\":%u},"
    "\"charge\":{\"level\":%u,\"lights_on\":%u,\"lockout_remaining_ms\":%lu,\"boost_active\":%s,\"boost_remaining_ms\":%lu,\"flashing\":%s,\"power_extra_pct\":%u},"
    "\"race\":{\"id\":%llu,\"phase\":%d,\"laps\":%lu,\"boot\":%lu,\"latest_lap_ms\":%lu,\"fastest_lap_ms\":%lu,\"strips\":%u},"
    "\"network\":{\"wifi_connected\":%s,\"sent\":%lu,\"dropped\":%lu,\"http_errors\":%lu}}",
    CAR_NAME, (unsigned long)now, (unsigned long)interval,
    fleet.revision, state, armed ? "true":"false", (unsigned long)failsafeCount,
    rc.steer.pulse, rc.steer.ok?"true":"false", steerAge, (unsigned long)steerCount, (unsigned long)steerBad,
    rc.throttle.pulse, rc.throttle.ok?"true":"false", throttleAge, (unsigned long)throttleCount, (unsigned long)throttleBad,
    CH3_BOOST_ENABLED?"true":"false", rc.ch3.pulse, rc.ch3.ok?"true":"false", ch3Age, (unsigned long)ch3Count, (unsigned long)ch3Bad,
    steeringCommand, steerPct, inputPct, escOutput, ESC_NEUTRAL_US, (unsigned int)reversePhase,
    colourOK?"true":"false", r8,g8,b8, yellow?"true":"false", green?"true":"false", red?"true":"false", blue?"true":"false",
    (unsigned long)colourReadCount, (unsigned long)colourReadSlowCount,
    boosting?"true":"false", (unsigned long)max(max(yellowRemaining, redRemaining), launchRemaining),
    slowingGreen?"true":"false", (unsigned long)(slowingGreen ? max(0L, (long)((long)fleet.greenMs - (long)(now - greenTimer.startMs))) : 0),
    cornerSlowdown, telemetryLedDuty, (unsigned int)((unsigned long)telemetryLedDuty * 100 / 255),
    (unsigned int)redCharges, (unsigned int)visibleChargeLeds(now),
    (unsigned long)(redHasCharged && (uint32_t)(now - lastRedChargeMs) < RED_CHARGE_LOCKOUT_MS
      ? RED_CHARGE_LOCKOUT_MS - (uint32_t)(now - lastRedChargeMs) : 0),
    redBoostRunning ? "true" : "false",
    (unsigned long)redRemaining,
    redBoostRunning ? "true" : "false",
    (unsigned int)(redBoostRunning ? RED_POWER_EXTRA_PERCENT : 0),
    lapRaceId, fleet.raceValid ? fleet.racePhase : RACE_OFF, (unsigned long)raceLaps, (unsigned long)raceBoot,
    (unsigned long)latestLapDurationMs, (unsigned long)fastestLapDurationMs, (unsigned int)lapStrips,
    WiFi.status() == WL_CONNECTED ? "true" : "false",
    (unsigned long)telemetrySent, (unsigned long)telemetryDropped,
    (unsigned long)telemetryHttpErrors
  );
  if (jsonLength > 0 && (size_t)jsonLength < sizeof(json)) sendJsonLine(json);
  else ++telemetryDropped;
}

// ---------- SETUP ----------
// Called only by loop(), before either input can update the ESC.
void updateRemoteDrive(RcReading &rc);
void updateRemoteDrive(RcReading &rc) {
  const bool receiverNeutral = rc.steer.ok && rc.throttle.ok &&
    abs((int)rc.throttle.pulse - RX_NEUTRAL_US) <= ARM_DEADZONE_US;
  RemoteDriveCommand command = {};
  if (remoteDriveQueue && xQueueReceive(remoteDriveQueue, &command, 0) == pdTRUE &&
      millis() - command.requestedAt < DRIVE_WATCHDOG_MS) {
    if (command.mode == 1 && command.id &&
        remoteDriveMode == DRIVE_RC && receiverNeutral && armed && command.id != remoteDriveId) {
      remoteDriveId = command.id;
      remoteDriveCommand = command;
      remoteDriveCommand.throttle = 0;
      remoteDriveMode = DRIVE_IPAD;
      resetDriveState();
    } else if (command.mode == 1 && remoteDriveMode == DRIVE_IPAD && command.id == remoteDriveId) {
      remoteDriveCommand = command;
    } else if (command.mode == 2 && remoteDriveMode == DRIVE_IPAD) {
      remoteDriveMode = DRIVE_STOPPED;
    } else if (command.mode == 3 && remoteDriveMode != DRIVE_RC && receiverNeutral) {
      remoteDriveMode = DRIVE_RC;
      armed = false;
      neutralStartMs = 0;
      resetDriveState();
    }
  }
  if (remoteDriveMode == DRIVE_IPAD &&
      (!receiverNeutral || millis() - remoteDriveCommand.requestedAt >= DRIVE_WATCHDOG_MS)) {
    remoteDriveMode = DRIVE_STOPPED;
    ++failsafeCount;
  }
  if (remoteDriveMode == DRIVE_STOPPED && armed) {
    armed = false;
    neutralStartMs = 0;
    resetDriveState();
  }
  remoteDriveStatus.store((unsigned int)remoteDriveMode |
    ((remoteDriveMode == DRIVE_RC && receiverNeutral && armed) ? 4U : 0U));
  if (remoteDriveMode != DRIVE_IPAD) return;
  const int steer = remoteDriveCommand.steer;
  const int travel = steer < 0 ? STEER_LOW_TRAVEL_US : STEER_HIGH_TRAVEL_US;
  rc.steer.pulse = STEER_CENTER_US + steer * travel / 100;
  const int throttle = remoteDriveCommand.throttle;
  rc.throttle.pulse = throttle == 0 ? RX_NEUTRAL_US :
    RX_NEUTRAL_US + RX_DEADZONE_US + 1 +
    (RX_FORWARD_MAX_US - RX_NEUTRAL_US - RX_DEADZONE_US - 1) * throttle / 100;
  rc.ch3.ok = false;
}

// Qualify neutral during staging, then carry that time into a confirmed GO for the same race.
void updateRaceNeutral(const RcReading &rc, uint32_t now, bool raceOnline) {
  const bool receiverNeutral = rc.steer.ok && rc.throttle.ok &&
    abs((int)rc.throttle.pulse - RX_NEUTRAL_US) <= ARM_DEADZONE_US;
  const bool neutralInRcMode = receiverNeutral && remoteDriveMode == DRIVE_RC;
  if (raceLocked) {
    if (raceOnline && neutralInRcMode &&
        (fleet.racePhase == RACE_STAGE || fleet.racePhase == RACE_COUNTDOWN)) {
      if (preparedRaceId != fleet.raceId) {
        preparedRaceId = fleet.raceId;
        neutralStartMs = now ? now : 1;
      }
    } else {
      preparedRaceId = 0;
      neutralStartMs = 0;
    }
  } else if (preparedRaceId != 0) {
    if (!raceOnline || !neutralInRcMode || fleet.racePhase != RACE_RUNNING ||
        fleet.raceId != preparedRaceId) neutralStartMs = 0;
    preparedRaceId = 0;
  }
}

void setup() {
  raceBoot = esp_random();
  fleetQueue = xQueueCreate(1, sizeof(FleetSettings));
  if (SERIAL_ENABLED) { Serial.begin(115200); delay(1000); }
  pinMode(STEER_IN, INPUT); pinMode(THROTTLE_IN, INPUT);
  if (CH3_BOOST_ENABLED) pinMode(CH3_IN, INPUT);
  pinMode(SPEED_LED_PIN, OUTPUT); digitalWrite(SPEED_LED_PIN, LOW);
  // MOSFET gates: LOW=off on the tested active-HIGH wiring.
  pinMode(CHARGE_LED_1, OUTPUT); pinMode(CHARGE_LED_2, OUTPUT); pinMode(CHARGE_LED_3, OUTPUT);
  digitalWrite(CHARGE_LED_1, LOW); digitalWrite(CHARGE_LED_2, LOW); digitalWrite(CHARGE_LED_3, LOW);
  startupLightsStartMs = millis();
  startupLightsActive = true;
  updateStartupLights(startupLightsStartMs);
  steering.setPeriodHertz(50); esc.setPeriodHertz(50);
  steering.attach(STEER_OUT, 500, 2500);
  esc.attach(ESC_OUT, 500, 2500);
  stopCar();
  setupLed();
  {
    char bootRecord[200];
    snprintf(bootRecord, sizeof(bootRecord),
      "{\"v\":1,\"car\":\"%s\",\"type\":\"event\",\"ms\":%lu,\"event\":\"boot\",\"reset_reason\":%d}",
      CAR_NAME, (unsigned long)millis(), (int)esp_reset_reason());
    sendJsonLine(bootRecord);
  }
  // Initialise I2C only after the servo and ESC have safe neutral outputs.
  // A quick address probe avoids calling the Adafruit driver on a missing sensor.
  if (ENABLE_COLOUR_SENSOR) {
    if (SERIAL_ENABLED) {
      telemetryEvent("colour_init", "probing_0x29");
    }
    if (!Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN)) {
      if (SERIAL_ENABLED) telemetryEvent("colour_error", "i2c_begin_failed");
    } else {
      Wire.setClock(100000);
      Wire.setTimeOut(I2C_TIMEOUT_MS);
      Wire.beginTransmission(TCS34725_I2C_ADDRESS);
      uint8_t status = Wire.endTransmission();
      if (status == 0) {
        if (SERIAL_ENABLED) telemetryEvent("colour_detected", "address_0x29");
        colourOK = tcs.begin();
      } else {
        if (SERIAL_ENABLED) telemetryEvent("colour_error", "no_ack_at_0x29");
      }
    }
  }
  if (SERIAL_ENABLED) {
    telemetryEvent("colour_status", !ENABLE_COLOUR_SENSOR ? "disabled" : (colourOK ? "ready" : "not_found"));
  }
  attachInterrupt(digitalPinToInterrupt(STEER_IN), steerISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_IN), throttleISR, CHANGE);
  if (CH3_BOOST_ENABLED) attachInterrupt(digitalPinToInterrupt(CH3_IN), ch3ISR, CHANGE);
  startWifiTelemetry();
  startRemoteDrive();
}

// ---------- LOOP ----------
void loop() {
  if (fleetQueue && xQueueReceive(fleetQueue,&fleet,0)==pdTRUE) {
    if (fleet.raceValid && fleet.racePhase == RACE_FINISHED &&
        fleet.raceWinner && fleet.raceId != lastWinnerStrobeId) {
      lastWinnerStrobeId = fleet.raceId;
      winnerStrobeStartedMs = millis();
      winnerStrobeActive = true;
    }
    boostTimer.durationMs = fleet.yellowMs;
    greenTimer.durationMs = fleet.greenMs;
    if (fleet.flashId && fleet.flashId != lastFleetFlashId) {
      lastFleetFlashId = fleet.flashId;
      fleetFlashStartMs = fleet.flashStartMs;
      fleetFlashScheduled = (int32_t)(millis() - fleetFlashStartMs) <= 0;
    }
  }
  uint32_t now = millis();
  updateStartupLights(now);

  // Always sample the colour sensor, even without a transmitter signal.
  // This keeps RGB and colour_reads updating in NO SIGNAL / WAIT NEUTRAL.
  // Driving effects are still processed only after the receiver is armed.
  bool colourSampled = false;
  if (colourOK && (uint32_t)(now - lastColourMs) >= COLOUR_SAMPLE_MS) {
    lastColourMs = now;
    readColour();
    colourSampled = true;
  }

  const bool raceOnline = fleet.raceValid && millis()-fleet.raceRequestMs < RACE_LINK_TIMEOUT_MS;
  // An offline boot allows physical RC. Keep an explicit lock until a fresh
  // server response clears it, even if the link drops or a reply is invalid.
  static bool serverRaceLock = false;
  if (raceOnline) {
    serverRaceLock = fleet.racePhase == RACE_STAGE || fleet.racePhase == RACE_COUNTDOWN ||
      fleet.racePhase == RACE_FINISHED;
  }
  raceLocked = RACE_MODE_ENABLED && serverRaceLock;
  raceStagingLights = raceOnline && (fleet.raceLightsWaiting ||
    fleet.racePhase == RACE_STAGE || fleet.racePhase == RACE_COUNTDOWN);
  updateRaceLaps(now, colourSampled);
  updateRedBoost(now, colourSampled);
  showChargeLeds(now);
  // Avoid "entering red" from an old reading when the car becomes armed.
  if (!armed) redSeenPrevious = red;
  RcReading rc = readReceiver();
  updateRaceNeutral(rc, now, raceOnline);
  if (raceLocked) {
    if (armed) resetDriveState();
    armed = false;
    if (remoteDriveMode == DRIVE_IPAD) remoteDriveMode = DRIVE_STOPPED;
    remoteDriveStatus.store((unsigned int)remoteDriveMode);
    stopCar();
    printDiagnostics(rc, raceOnline ? "RACE LOCKED" : "RACE LINK LOST", ESC_NEUTRAL_US);
    return;
  }
  updateRemoteDrive(rc);
  if (remoteDriveMode == DRIVE_STOPPED) {
    stopCar();
    printDiagnostics(rc, "IPAD STOPPED", ESC_NEUTRAL_US);
    return;
  }
  if (!rc.steer.ok || !rc.throttle.ok) {
    if (armed) {
      ++failsafeCount;
      telemetryEvent("failsafe", !rc.steer.ok && !rc.throttle.ok ? "both_rx_lost" :
                     (!rc.steer.ok ? "steering_rx_lost" : "throttle_rx_lost"));
    }
    armed = false; neutralStartMs = 0; resetDriveState(); stopCar();
    printDiagnostics(rc, "NO SIGNAL", ESC_NEUTRAL_US);
    return;
  }
  if (!armed) {
    if (abs((int)rc.throttle.pulse - RX_NEUTRAL_US) <= ARM_DEADZONE_US) {
      if (!neutralStartMs) neutralStartMs = now ? now : 1;
      if ((uint32_t)(now - neutralStartMs) >= ARM_STABLE_MS) {
        armed = true; resetDriveState();
        telemetryEvent("armed", "neutral_stable");
      }
    } else {
      neutralStartMs = 0;
    }
    if (!armed) {
      stopCar(); printDiagnostics(rc, "WAIT NEUTRAL", ESC_NEUTRAL_US); return;
    }
  }
  int steerCommand = limitSteeringPulse(rc.steer.pulse);
  steering.writeMicroseconds(steerCommand);
  int steeringOffset = steerCommand - STEER_CENTER_US;
  bool forward = (int)rc.throttle.pulse > RX_NEUTRAL_US + RX_DEADZONE_US;

  // Apply track effects only on a fresh sample, with valid/armed RC.
  // Don't count the same yellow sample repeatedly while the loop runs.
  if (colourSampled) {
    if (!startupLightsActive) updateRedDetection(now);
    if (yellow && forward) {
      greenTimer.stop(); // Yellow takes priority even before boost qualifies.
      if (yellowHits < YELLOW_HITS_REQUIRED) ++yellowHits;
      if (yellowHits >= YELLOW_HITS_REQUIRED && !yellowLatched) {
        yellowLatched = true; boostTimer.start(now, fleet.yellowMs); greenTimer.stop();
        telemetryEvent("boost", "yellow");
      }
    } else if (!yellow) { yellowHits = 0; yellowLatched = false; }
    else yellowHits = 0;
    if (green && forward && !boostTimer.active(now)) greenTimer.start(now, fleet.greenMs);
  }
  bool ch3Pressed = CH3_BOOST_ENABLED && rc.ch3.ok && rc.ch3.pulse >= CH3_BOOST_THRESHOLD_US;
  if (ch3Pressed && !ch3Latched && forward) {
    boostTimer.start(now, fleet.yellowMs); greenTimer.stop();
    telemetryEvent("boost", "ch3");
  }
  ch3Latched = ch3Pressed;
  if (!forward) boostTimer.stop();
  // Preserve the original yellow/CH3 boost mapping. Red power is a
  // separate +10% increase to forward ESC command; never force movement.
  bool boost = boostTimer.active(now) || (forward && launchBoostRemaining(now) > 0);
  if (boost || redBoostRunning) greenTimer.stop();
  int output = throttleOutput(rc.throttle.pulse, boost, now);
  if (!boost && !redBoostRunning && greenTimer.active(now) && output > ESC_NEUTRAL_US)
    output -= (long)(output - ESC_NEUTRAL_US) * min(fleet.greenPercent, 35) / 100;
  // Apply red power only during forward throttle, and respect corner limiting.
  // 10% is an ESC command increase, not guaranteed measured vehicle speed.
  if (redBoostRunning && forward && output > ESC_NEUTRAL_US) {
    int forwardCommand = output - ESC_NEUTRAL_US;
    output = ESC_NEUTRAL_US + (long)forwardCommand * (100 + RED_POWER_EXTRA_PERCENT) / 100;
  }
  output = applyCornerLimit(output, steeringOffset, boost || redBoostRunning);
  output = constrain(output, ESC_REVERSE_MIN_US, ESC_BOOST_MAX_US);
  if (output > ESC_NEUTRAL_US) output = ESC_NEUTRAL_US + (long)(output - ESC_NEUTRAL_US) * fleet.speed / 100;
  if (remoteDriveMode == DRIVE_IPAD) {
    const int ceiling = ESC_NEUTRAL_US +
      (ESC_FORWARD_MAX_US - ESC_NEUTRAL_US) * DRIVE_MAX_FORWARD_PERCENT / 100;
    output = remoteDriveCommand.throttle == 0 ? ESC_NEUTRAL_US :
      constrain(output, ESC_NEUTRAL_US, ceiling);
    // Recheck after calculating track effects, immediately before setting the ESC.
    if (millis() - remoteDriveCommand.requestedAt >= DRIVE_WATCHDOG_MS) output = ESC_NEUTRAL_US;
  }
  esc.writeMicroseconds(output);
  updateSpeedLed(output, boost || redBoostRunning, armed);
  printDiagnostics(rc, remoteDriveMode == DRIVE_IPAD ? "IPAD" : "RUN", output);
}