/*
  DAPNET POCSAG Pager — LilyGO T-Beam v1.2 (AXP2101 + SX1276 + SSD1306)

  Turns the T-Beam into a standalone POCSAG pager receiver. Listens on
  439.9875 MHz (IARU R1 DAPNET allocation), filters on configurable RIC
  addresses, displays messages on the 128×64 OLED, beeps the piezo, and
  persists the inbox to SPIFFS across reboots.

  Architecture:
    Core 0 — rxTask: POCSAG reception via RadioLib (may block in readData)
    Core 1 — loopTask: UI, button handling, battery, OLED (never blocks)
    Communication: FreeRTOS queue (rxQueue) between cores

  Build (Arduino IDE):
    Board: ESP32 Dev Module
    Upload Speed: 115200
    Partition Scheme: Default 4MB with 1.5MB SPIFFS
    Libraries: RadioLib, XPowersLib, Adafruit SSD1306, Adafruit GFX, ArduinoJson
    Two uploads needed: "ESP32 Sketch Data Upload" (SPIFFS) + normal sketch upload
*/

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <XPowersLib.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <esp_wifi.h>
#include <time.h>
#include <sys/time.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_SS     18
#define LORA_DIO0   26
#define LORA_RST    23
#define LORA_DIO1   33
#define LORA_DIO2   32
#define OLED_SDA    21
#define OLED_SCL    22
#define BUTTON_USR  38
#define BUZZER      25
// Note: the visual alert LED is the AXP2101 CHGLED (red LED below the OLED).
// It's not on a GPIO — it's controlled by the PMU over I²C via
// XPowersLib's setChargingLedMode(). T-Beam v1.2 has no free user LED on a GPIO.

// RIC 2504 = DAPNET "Skyper OTA Time" broadcast, sent every ~5 min.
// Used as the sole time source — no GPS, no NTP, no WiFi.
#define TIME_RIC    2504

// ─── Config (loaded from /config.json on SPIFFS at boot) ────
#define CFG_MAX_RICS    8
#define CFG_RIC_NAME_LEN 12   // alias length incl. null terminator; fits status bar header

// One RIC entry from config — number plus optional human-readable alias
// shown in alert/detail headers and the status bar instead of the raw RIC.
struct RicEntry {
    uint32_t ric;
    char     name[CFG_RIC_NAME_LEN];   // empty string = no alias, fall back to number
};

struct Config {
    RicEntry rics[CFG_MAX_RICS];
    int      ricCount;
    char     lang[4];            // UI language: "en" | "sk" | "fr" | "es" | "pt"
    bool     storeMessages;
    char     messageFolder[24];
    char     tz[48];             // POSIX TZ string, e.g. "CET-1CEST,M3.5.0/2,M10.5.0/3"
    float    freq;               // DAPNET carrier in MHz (IARU R1: 439.98750)
    float    offset;             // per-board crystal correction in MHz
    bool     buzzer;             // master on/off for piezo alerts
    int      buzzerVolume;       // 0..100, applied as PWM duty cycle (0 = silent)
    bool     ledBlink;           // master on/off for LED alert (CHGLED + ext)
    int      extLedPin;          // optional external LED GPIO; -1 = disabled
    bool     extLedActiveHigh;   // true = HIGH lights LED (typical wiring to GND)
};

// Fallback used only if SPIFFS mount fails or /config.json is missing/malformed.
// Intentionally empty RIC list — do NOT ship a real RIC here, otherwise a
// misconfigured unit would pick up another ham's traffic on first boot.
Config cfg = {
    .rics          = {},
    .ricCount      = 0,
    .lang          = "en",
    .storeMessages = true,
    .messageFolder = "/msgs",
    .tz            = "CET-1CEST,M3.5.0/2,M10.5.0/3",
    .freq          = 439.98750f,
    .offset        = 0.0044f,
    .buzzer           = true,
    .buzzerVolume     = 70,
    .ledBlink         = true,
    .extLedPin        = -1,
    .extLedActiveHigh = true
};

// ─── Messages ───────────────────────────────────────────
#define MAX_MSGS   20          // max messages kept in RAM + SPIFFS
#define MAX_LEN    200         // max characters per message

struct PagerMsg {
    uint32_t addr;             // sender RIC address
    char     text[MAX_LEN];    // message body (ASCII-filtered)
    uint32_t timestamp;        // epoch seconds (wall clock) or millis()/1000 (uptime)
    bool     tsIsEpoch;        // true = real wall clock, false = uptime since boot
    bool     unread;           // shown with dot indicator in inbox
};

PagerMsg inbox[MAX_MSGS];      // newest message at index 0 (LIFO)
int msgCount  = 0;             // number of messages currently in inbox
int totalMsgs = 0;             // total messages received since boot
int viewIdx   = 0;             // currently selected message index in inbox UI

// ─── Translations ───────────────────────────────────────
enum Lang  { LANG_EN, LANG_SK, LANG_FR, LANG_ES, LANG_PT, LANG_COUNT };
enum StrId {
    STR_NEW, STR_MESSAGES, STR_WAITING, STR_NO_MESSAGES,
    STR_COUNT
};

// ASCII-only (SSD1306 default font has no diacritics)
static const char* translations[LANG_COUNT][STR_COUNT] = {
    /* EN */ { "new",        "messages", "Waiting for messages...", "No messages"   },
    /* SK */ { "novych",     "sprav",    "Cakam na spravy...",      "Ziadne spravy" },
    /* FR */ { "nouveaux",   "messages", "En attente de messages...","Aucun message"},
    /* ES */ { "nuevos",     "mensajes", "Esperando mensajes...",   "Sin mensajes"  },
    /* PT */ { "novas",      "mensagens","A espera de mensagens...","Sem mensagens" }
};

int langIdx = LANG_EN;
const char* T(StrId k) { return translations[langIdx][k]; }

// ─── UI state ───────────────────────────────────────────
// Four-screen state machine driven by a single button (GPIO 38):
//   SCR_IDLE   → home screen (clock, RIC, message count, battery)
//   SCR_ALERT  → newly arrived message in large font (stays lit until user presses)
//   SCR_INBOX  → scrollable message list with selection highlight
//   SCR_DETAIL → full message body with sender header and age
enum Screen { SCR_IDLE, SCR_ALERT, SCR_INBOX, SCR_DETAIL };
Screen screen = SCR_IDLE;
unsigned long lastActivity  = 0;   // last user interaction (button press or new message)
unsigned long lastBatUpdate = 0;   // last battery voltage readout
unsigned long btnDownTime   = 0;   // millis() when button was pressed down
bool oledOn = true;                // false = display blanked (just framebuffer, no HW sleep)
bool btnWasDown = false;           // edge detection for button press/release
float batVoltage = 0;
int batPercent   = 0;
bool rtcValid = false;             // true after first successful Skyper OTA time sync

// Fragment reassembly: RadioLib returns ~50 chars per readData() call.
// If a second fragment from the same RIC arrives within this window,
// it's appended to the previous message instead of creating a new one.
unsigned long lastMsgTime = 0;
#define FRAG_WINDOW_MS 5000

// ─── LED alert (PMU-driven + optional external LED) ────
// Two-stage pager-style alert. Drives two outputs in parallel:
//   • AXP2101 CHGLED (red, below OLED) — hardware blink via I²C
//   • optional external LED on cfg.extLedPin — software blink in tickAlertLed
// State machine:
//   1. New message → BLINKING for LED_BLINK_MS (4 Hz on both LEDs).
//   2. Then → STEADY (both LEDs solid on) as persistent "unread" indicator.
//   3. Detail opened or button pressed → OFF (CHGLED reverts to charging state).
#define LED_BLINK_MS         3000
#define LED_BLINK_HALF_MS    125    // ~4 Hz blink (matches PMU's BLINK_4HZ)
enum LedAlertMode { LED_OFF, LED_BLINKING, LED_STEADY };
LedAlertMode  ledMode       = LED_OFF;
unsigned long ledStartedAt  = 0;
unsigned long ledLastToggle = 0;   // for software-blinking the external LED
bool          extLedState   = false;

// ─── Auto-scroll for long messages ─────────────────────
// Messages longer than ~3 lines (30 chars at text size 2) overflow the
// 128×64 display. The body auto-scrolls: pause at top → pixel scroll
// → pause at bottom → wrap. Header stays fixed above the scroll area.
#define BODY_TOP        10         // y-pixel where message body starts (below header)
#define BODY_H          (64 - BODY_TOP)  // available pixels for body
#define SCROLL_STEP_MS  80         // ms between 1-pixel scroll steps
#define SCROLL_PAUSE_MS 2000       // pause at top/bottom before scrolling
int  scrollY     = 0;              // current vertical scroll offset in pixels
int  scrollMax   = 0;              // max scroll offset (0 = no scroll needed)
int  scrollState = 0;              // 0=pause_top, 1=scrolling_down, 2=pause_bottom
unsigned long scrollTimer = 0;
bool scrollInProgress = false;     // true when called from scroll animation (skip scroll init)

// ─── Objects ────────────────────────────────────────────
SX1276 radio = new Module(LORA_SS, LORA_DIO0, LORA_RST, LORA_DIO1);
PagerClient pager(&radio);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
XPowersAXP2101 pmu;

// ─── Core 0 RX task + queue ────────────────────────────
// RadioLib's readData() polls DIO2 in a tight loop (PhysicalLayer::read)
// with NO timeout. When a POCSAG transmission ends mid-frame, it blocks
// indefinitely. Running this on a dedicated FreeRTOS task pinned to Core 0
// keeps the UI on Core 1 fully responsive even during a blocked read.
// The idle task watchdog for Core 0 is disabled so it won't trigger a reset.

struct RxMsg {
    uint32_t addr;             // decoded RIC address
    char     text[MAX_LEN];    // decoded message text
};
QueueHandle_t rxQueue = nullptr;  // 4-entry queue bridging Core 0 → Core 1

// Runs forever on Core 0. Only touches SPI (SX1276) — never I²C (OLED/PMU).
void rxTask(void* param) {
    for (;;) {
        if (pager.available() >= 2) {
            String str;
            uint32_t addr = 0;
            int state = pager.readData(str, 0, &addr);  // may block here!
            if (state == RADIOLIB_ERR_NONE) {
                RxMsg msg;
                msg.addr = addr;
                strlcpy(msg.text, str.c_str(), MAX_LEN);
                xQueueSend(rxQueue, &msg, portMAX_DELAY);
            }
        }
        vTaskDelay(1);  // yield to other Core 0 tasks
    }
}

// ═══════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════

// Kill WiFi + Bluetooth and downclock CPU to 80 MHz. This is a pager,
// never a network device. The extra draw is wasted and WiFi RF can
// desensitize the SX1276 RX frontend. Called once at the top of setup().
void disableRadios() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    esp_wifi_stop();
    esp_bt_controller_disable();
    setCpuFrequencyMhz(80);
}

// Blank the OLED framebuffer and push it. No hardware sleep command —
// the panel stays powered, just shows all-black.
void sleepOLED() {
    oled.clearDisplay();
    oled.display();
    oledOn = false;
}

// Mark the display as active. The next draw call will show content.
void wakeOLED() {
    oledOn = true;
}

// Read battery voltage + charge state from AXP2101 via I²C.
// Linear approximation: 3.2V = 0%, 4.15V = 100%.
void updateBattery() {
    batVoltage = pmu.getBattVoltage() / 1000.0;
    batPercent = constrain((int)((batVoltage - 3.2) / 0.95 * 100), 0, 100);
    lastBatUpdate = millis();

    bool isCharging = pmu.isCharging();
    bool usbIn = pmu.isVbusIn();
    Serial.printf("[BAT] %.2fV (%d%%) USB:%s Charging:%s\n",
                  batVoltage, batPercent,
                  usbIn ? "yes" : "no",
                  isCharging ? "yes" : "no");
}

int unreadCount() {
    int n = 0;
    for (int i = 0; i < msgCount; i++)
        if (inbox[i].unread) n++;
    return n;
}

// Format message age as "Xs ago" / "Xm ago" / "Xh ago".
// Uses wall clock if the message has a real timestamp, else uptime.
char* formatAge(uint32_t ts, bool tsIsEpoch, char* buf, int len) {
    uint32_t now = tsIsEpoch ? (uint32_t)time(nullptr) : (uint32_t)(millis() / 1000);
    unsigned long age = (now >= ts) ? (now - ts) : 0;
    if (age < 60)        snprintf(buf, len, "%lus ago", age);
    else if (age < 3600) snprintf(buf, len, "%lum ago", age / 60);
    else                 snprintf(buf, len, "%luh ago", age / 3600);
    return buf;
}

// HH:MM local time if rtcValid, else "--:--"
void formatClock(char* buf, int len) {
    if (rtcValid) {
        time_t now = time(nullptr);
        struct tm* lt = localtime(&now);
        snprintf(buf, len, "%02d:%02d", lt->tm_hour, lt->tm_min);
    } else {
        snprintf(buf, len, "--:--");
    }
}

// Set ESP32 system clock from a Skyper OTA Time broadcast.
// Input is UTC; after setting, switches to the user's local timezone
// (from cfg.tz) and persists the epoch to /rtc.txt on SPIFFS.
void setTimeFromSkyper(int hh, int mm, int ss, int dd, int mo, int yy) {
    setenv("TZ", "UTC0", 1);  // temporarily UTC for mktime()
    tzset();
    struct tm t = {};
    t.tm_year = 2000 + yy - 1900;
    t.tm_mon  = mo - 1;
    t.tm_mday = dd;
    t.tm_hour = hh;
    t.tm_min  = mm;
    t.tm_sec  = ss;
    time_t utc = mktime(&t);
    struct timeval tv = { .tv_sec = utc, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    // Switch to local timezone (loaded from config.json, resolved via tz.csv)
    setenv("TZ", cfg.tz, 1);
    tzset();
    rtcValid = true;
    // Persist to SPIFFS so time survives full power-off (with coin cell backup)
    File tf = SPIFFS.open("/rtc.txt", "w");
    if (tf) { tf.println(utc); tf.close(); }
    Serial.printf("[RTC] Sync: 20%02d-%02d-%02d %02d:%02d:%02d UTC → saved\n",
                  yy, mo, dd, hh, mm, ss);
}

// Parse "[Skyper OTA Time] HHMMSS DDMMYY" (or bare "HHMMSS DDMMYY").
// Returns true if the format matched. Only updates the clock when:
//   - rtcValid is false (first sync — always apply), or
//   - the drift between current clock and incoming time exceeds 60 seconds
// This avoids unnecessary settimeofday() calls on every 5-minute broadcast.
bool tryParseSkyperTime(const char* text) {
    const char* p = strstr(text, "] ");
    p = p ? p + 2 : text;  // skip "[Skyper OTA Time] " prefix if present
    int hh, mm, ss, dd, mo, yy;
    if (sscanf(p, "%2d%2d%2d %2d%2d%2d", &hh, &mm, &ss, &dd, &mo, &yy) == 6) {
        if (hh < 24 && mm < 60 && ss < 60 &&
            dd >= 1 && dd <= 31 && mo >= 1 && mo <= 12) {
            if (rtcValid) {
                // Clock already set — only correct if drift > 60 seconds
                setenv("TZ", "UTC0", 1); tzset();
                struct tm t = {};
                t.tm_year = 2000 + yy - 1900; t.tm_mon = mo - 1; t.tm_mday = dd;
                t.tm_hour = hh; t.tm_min = mm; t.tm_sec = ss;
                time_t incoming = mktime(&t);
                time_t current  = time(nullptr);
                setenv("TZ", cfg.tz, 1); tzset();
                long diff = (long)(incoming - current);
                if (diff < 0) diff = -diff;
                if (diff > 60) {
                    Serial.printf("[RTC] Drift %lds — correcting\n", diff);
                    setTimeFromSkyper(hh, mm, ss, dd, mo, yy);
                }
                return true;
            }
            setTimeFromSkyper(hh, mm, ss, dd, mo, yy);
            return true;
        }
    }
    return false;
}

// ═══════════════════════════════════════════════════════
//  CONFIG  (SPIFFS /config.json + /tz.csv)
// ═══════════════════════════════════════════════════════

// Resolve a human-readable timezone alias (e.g. "Europe/Bratislava") to
// its POSIX TZ string by looking it up in /tz.csv on SPIFFS.
// Format: one "alias=POSIX" pair per line. If no match, cfg.tz is left
// unchanged (so raw POSIX strings also work in config.json).
static void resolveTzAlias() {
    File f = SPIFFS.open("/tz.csv", "r");
    if (!f) return;
    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        int eq = line.indexOf('=');
        if (eq <= 0) continue;
        if (line.substring(0, eq) == cfg.tz) {
            strlcpy(cfg.tz, line.substring(eq + 1).c_str(), sizeof(cfg.tz));
            f.close();
            return;
        }
    }
    f.close();
}

static int langCodeToIdx(const char* c) {
    if (!strcmp(c, "en")) return LANG_EN;
    if (!strcmp(c, "sk")) return LANG_SK;
    if (!strcmp(c, "fr")) return LANG_FR;
    if (!strcmp(c, "es")) return LANG_ES;
    if (!strcmp(c, "pt")) return LANG_PT;
    return LANG_EN;
}

void loadConfig() {
    if (!SPIFFS.begin(true)) {
        Serial.println(F("[CFG] SPIFFS mount FAIL — using defaults"));
        langIdx = LANG_EN;
        return;
    }
    File f = SPIFFS.open("/config.json", "r");
    if (!f) {
        Serial.println(F("[CFG] /config.json missing — using defaults"));
        langIdx = LANG_EN;
        return;
    }
#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    StaticJsonDocument<512> doc;
#endif
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) {
        Serial.printf("[CFG] JSON parse error: %s — using defaults\n", err.c_str());
        langIdx = LANG_EN;
        return;
    }

    // Accept two forms per entry — backward compatible with the original
    // numeric-only schema, plus an object form that adds a friendly alias:
    //   "rics": [ 314046, { "ric": 1040, "name": "EMERGENCY" } ]
    JsonArray ra = doc["rics"].as<JsonArray>();
    if (ra) {
        cfg.ricCount = 0;
        for (JsonVariant v : ra) {
            if (cfg.ricCount >= CFG_MAX_RICS) break;
            RicEntry& e = cfg.rics[cfg.ricCount];
            e.name[0] = '\0';
            if (v.is<JsonObject>()) {
                e.ric = v["ric"].as<uint32_t>();
                if (v["name"].is<const char*>())
                    strlcpy(e.name, v["name"], sizeof(e.name));
            } else {
                e.ric = v.as<uint32_t>();
            }
            if (e.ric != 0) cfg.ricCount++;
        }
    }
    if (doc["lang"].is<const char*>())
        strlcpy(cfg.lang, doc["lang"], sizeof(cfg.lang));
    cfg.storeMessages = doc["storeMessages"] | cfg.storeMessages;
    if (doc["messageFolder"].is<const char*>())
        strlcpy(cfg.messageFolder, doc["messageFolder"], sizeof(cfg.messageFolder));
    if (doc["tz"].is<const char*>())
        strlcpy(cfg.tz, doc["tz"], sizeof(cfg.tz));

    // Radio + alert overrides — all optional, fall back to baked-in defaults.
    cfg.freq         = doc["freq"]         | cfg.freq;
    cfg.offset       = doc["offset"]       | cfg.offset;
    cfg.buzzer           = doc["buzzer"]           | cfg.buzzer;
    cfg.buzzerVolume     = doc["buzzerVolume"]     | cfg.buzzerVolume;
    cfg.ledBlink         = doc["ledBlink"]         | cfg.ledBlink;
    cfg.extLedPin        = doc["extLedPin"]        | cfg.extLedPin;
    cfg.extLedActiveHigh = doc["extLedActiveHigh"] | cfg.extLedActiveHigh;
    if (cfg.buzzerVolume < 0)   cfg.buzzerVolume = 0;
    if (cfg.buzzerVolume > 100) cfg.buzzerVolume = 100;

    langIdx = langCodeToIdx(cfg.lang);
    resolveTzAlias();

    Serial.printf("[CFG] lang=%s store=%s folder=%s tz=%s freq=%.4f off=%.4f buz=%s vol=%d led=%s extLed=%d/%s rics=",
                  cfg.lang, cfg.storeMessages ? "y" : "n", cfg.messageFolder, cfg.tz,
                  cfg.freq, cfg.offset,
                  cfg.buzzer ? "y" : "n", cfg.buzzerVolume,
                  cfg.ledBlink ? "y" : "n",
                  cfg.extLedPin, cfg.extLedActiveHigh ? "AH" : "AL");
    for (int i = 0; i < cfg.ricCount; i++) {
        if (cfg.rics[i].name[0])
            Serial.printf("%lu(%s)", (unsigned long)cfg.rics[i].ric, cfg.rics[i].name);
        else
            Serial.printf("%lu", (unsigned long)cfg.rics[i].ric);
        Serial.print(i + 1 < cfg.ricCount ? "," : "\n");
    }
}

bool addrIsUserRic(uint32_t addr) {
    for (int i = 0; i < cfg.ricCount; i++)
        if (cfg.rics[i].ric == addr) return true;
    return false;
}

// Returns the user-defined alias for a RIC, or nullptr if none/unknown.
// Caller decides the fallback (usually printing the raw number).
const char* ricNameFor(uint32_t addr) {
    for (int i = 0; i < cfg.ricCount; i++)
        if (cfg.rics[i].ric == addr && cfg.rics[i].name[0])
            return cfg.rics[i].name;
    return nullptr;
}

// ═══════════════════════════════════════════════════════
//  MESSAGE PERSISTENCE  (SPIFFS flat filesystem)
//
//  SPIFFS has no real directories — paths with '/' are just filenames.
//  Each message is one file:
//    Path (RTC synced):  <folder>/<ric>_<YYYYMMDDHHMMSS>.txt
//    Path (pre-sync):    <folder>/<ric>_u<uptimeSec>.txt
//    Body:               "<epoch>\t<ric>\t<text>\n"
//
//  On boot, loadSavedMessages() scans all files, filters by folder
//  prefix, sorts by the epoch stored inside (not by filename — RIC-first
//  names don't sort chronologically across multiple RICs), and keeps
//  the newest MAX_MSGS entries.
// ═══════════════════════════════════════════════════════

void saveMessageToFS(const PagerMsg& m) {
    if (!cfg.storeMessages) return;
    char path[48];
    if (m.tsIsEpoch) {
        time_t tt = (time_t)m.timestamp;
        struct tm* lt = localtime(&tt);
        snprintf(path, sizeof(path), "%s/%lu_%04d%02d%02d%02d%02d%02d.txt",
                 cfg.messageFolder, (unsigned long)m.addr,
                 1900 + lt->tm_year, lt->tm_mon + 1, lt->tm_mday,
                 lt->tm_hour, lt->tm_min, lt->tm_sec);
    } else {
        snprintf(path, sizeof(path), "%s/%lu_u%010lu.txt",
                 cfg.messageFolder, (unsigned long)m.addr,
                 (unsigned long)m.timestamp);
    }
    // Collision avoidance: two messages in the same second get suffix _a, _b, …
    // Cap is 26 collisions/second (a..z); after that we overwrite the last file —
    // in practice POCSAG can't deliver that many messages per second anyway.
    if (SPIFFS.exists(path)) {
        char suf[4] = "_a";
        size_t baseLen = strlen(path) - 4;  // pred ".txt"
        for (char c = 'a'; c <= 'z'; c++) {
            suf[1] = c;
            snprintf(path + baseLen, 8, "%s.txt", suf);
            if (!SPIFFS.exists(path)) break;
        }
    }
    File f = SPIFFS.open(path, "w");
    if (!f) { Serial.printf("[FS] save FAIL %s\n", path); return; }
    f.printf("%lu\t%lu\t%s\n", (unsigned long)m.timestamp, (unsigned long)m.addr, m.text);
    f.close();
    Serial.printf("[FS] saved %s\n", path);
}

void loadSavedMessages() {
    if (!cfg.storeMessages) return;

    // SPIFFS is flat — iterate root, filter by folder prefix, sort by
    // embedded timestamp (not filename, since RIC-first names don't sort chronologically).
    File root = SPIFFS.open("/");
    if (!root) return;

    struct LoadedMsg {
        uint32_t ts;
        uint32_t ric;
        char     text[MAX_LEN];
        bool     tsIsEpoch;
    };
    // Static: ~8.5 KB array would overflow the 8 KB loopTask stack if local.
    static LoadedMsg loaded[MAX_MSGS * 2];
    int loadedCount = 0;
    const int maxLoaded = sizeof(loaded) / sizeof(loaded[0]);
    size_t prefixLen = strlen(cfg.messageFolder);

    File entry = root.openNextFile();
    while (entry) {
        const char* raw = entry.name();
        char full[48];
        if (raw[0] == '/') strlcpy(full, raw, sizeof(full));
        else               snprintf(full, sizeof(full), "/%s", raw);

        bool match = (strncmp(full, cfg.messageFolder, prefixLen) == 0 &&
                      full[prefixLen] == '/');
        if (match && loadedCount < maxLoaded) {
            File ff = SPIFFS.open(full, "r");
            if (ff) {
                String line = ff.readStringUntil('\n');
                ff.close();
                int t1 = line.indexOf('\t');
                int t2 = line.indexOf('\t', t1 + 1);
                if (t1 > 0 && t2 > t1) {
                    LoadedMsg& lm = loaded[loadedCount];
                    lm.ts  = strtoul(line.substring(0, t1).c_str(), nullptr, 10);
                    lm.ric = strtoul(line.substring(t1 + 1, t2).c_str(), nullptr, 10);
                    strlcpy(lm.text, line.substring(t2 + 1).c_str(), MAX_LEN);
                    // Threshold 1e9 seconds ≈ year 2001 — anything larger is a real
                    // wall-clock epoch; anything smaller is seconds-since-boot (uptime)
                    // recorded before the first Skyper time sync.
                    lm.tsIsEpoch = (lm.ts > 1000000000UL);
                    loadedCount++;
                }
            }
        }
        entry = root.openNextFile();
    }

    // Sort ascending by timestamp (oldest first).
    for (int i = 0; i < loadedCount - 1; i++)
        for (int j = i + 1; j < loadedCount; j++)
            if (loaded[i].ts > loaded[j].ts) {
                LoadedMsg t = loaded[i]; loaded[i] = loaded[j]; loaded[j] = t;
            }

    // Keep the newest MAX_MSGS. Each is shift-inserted at inbox[0] (newest first).
    int start = loadedCount > MAX_MSGS ? loadedCount - MAX_MSGS : 0;
    for (int i = start; i < loadedCount; i++) {
        if (msgCount < MAX_MSGS) msgCount++;
        for (int k = msgCount - 1; k > 0; k--) inbox[k] = inbox[k - 1];
        inbox[0].addr      = loaded[i].ric;
        strlcpy(inbox[0].text, loaded[i].text, MAX_LEN);
        inbox[0].timestamp = loaded[i].ts;
        inbox[0].tsIsEpoch = loaded[i].tsIsEpoch;
        inbox[0].unread    = false;
        totalMsgs++;
    }
    Serial.printf("[FS] loaded %d messages (%d on disk)\n", msgCount, loadedCount);
}

// Store a new message at inbox[0], shifting existing messages down.
// Non-printable characters are filtered (POCSAG can contain control codes).
void storeMessage(uint32_t addr, const char* text) {
    if (msgCount < MAX_MSGS) msgCount++;
    for (int i = msgCount - 1; i > 0; i--) inbox[i] = inbox[i - 1];
    inbox[0].addr = addr;

    // ASCII filter: keep printable chars (32–126), replace newlines with spaces
    int j = 0;
    for (int i = 0; text[i] && j < MAX_LEN - 1; i++) {
        char c = text[i];
        if (c >= 32 && c < 127)    inbox[0].text[j++] = c;
        else if (c == '\n')        inbox[0].text[j++] = ' ';
    }
    inbox[0].text[j] = '\0';

    if (rtcValid) {
        inbox[0].timestamp = (uint32_t)time(nullptr);
        inbox[0].tsIsEpoch = true;
    } else {
        inbox[0].timestamp = millis() / 1000;
        inbox[0].tsIsEpoch = false;
    }
    inbox[0].unread = true;
    totalMsgs++;
    viewIdx = 0;

    saveMessageToFS(inbox[0]);
}

// ═══════════════════════════════════════════════════════
//  SCREENS
// ═══════════════════════════════════════════════════════

// ── Status bar (9px white band at top of screen) ──
// Shows: frequency + primary RIC (or "N new" when unread), battery % on right.
// Used by idle and inbox screens; alert/detail have their own headers.
void drawMiniBar() {
    oled.fillRect(0, 0, 128, 9, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setTextSize(1);
    oled.setCursor(2, 1);

    int ur = unreadCount();
    if (ur > 0) {
        char b[16];
        snprintf(b, sizeof(b), "%d %s", ur, T(STR_NEW));
        oled.print(b);
    } else {
        // "439.988 ALIAS[+]" or "439.988 1234567[+]" — frequency + primary RIC.
        // Alias preferred when set (saves space, more readable than raw number).
        char top[24];
        if (cfg.ricCount > 0) {
            const char* tail = cfg.ricCount > 1 ? "+" : "";
            if (cfg.rics[0].name[0]) {
                snprintf(top, sizeof(top), "%.3f %s%s",
                         cfg.freq, cfg.rics[0].name, tail);
            } else {
                snprintf(top, sizeof(top), "%.3f %lu%s",
                         cfg.freq, (unsigned long)cfg.rics[0].ric, tail);
            }
        } else {
            snprintf(top, sizeof(top), "%.3f", cfg.freq);
        }
        oled.print(top);
    }

    // Battery right side
    char bp[6];
    snprintf(bp, sizeof(bp), "%d%%", batPercent);
    int bw = strlen(bp) * 6;
    oled.setCursor(128 - bw - 2, 1);
    oled.print(bp);
}

// ── IDLE: clean status ──
void drawIdle() {
    oled.clearDisplay();
    drawMiniBar();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);

    // Clock big in center when we have real time, else small RIC list
    if (rtcValid) {
        char clk[6];
        formatClock(clk, sizeof(clk));
        oled.setTextSize(2);
        int w = strlen(clk) * 12;            // size-2 char = 12px
        oled.setCursor((128 - w) / 2, 14);
        oled.print(clk);
        oled.setTextSize(1);
    } else {
        oled.setCursor(4, 18);
        oled.print(F("Waiting for time sync..."));
    }

    // Message count summary (RIC moved to status bar)
    oled.setCursor(4, 38);
    if (msgCount > 0) {
        oled.print(totalMsgs);
        oled.print(' ');
        oled.print(T(STR_MESSAGES));
        oled.print(F(" ("));
        oled.print(unreadCount());
        oled.print(' ');
        oled.print(T(STR_NEW));
        oled.print(F(")"));
    } else {
        oled.print(T(STR_WAITING));
    }

    unsigned long s = millis() / 1000;
    char info[24];
    snprintf(info, sizeof(info), "%.2fV  up %luh%02lum",
             batVoltage, s / 3600, (s / 60) % 60);
    oled.setCursor(4, 56);
    oled.print(info);

    oled.display();
}

// ── ALERT: message text + thin sender+time header ──
void drawAlert(int idx) {
    if (idx >= msgCount) return;

    // Parse sender from message (DAPNET format: "CALL: text").
    // 9-char limit = sender[10] - 1 for null terminator; ham callsigns are ≤7 chars
    // in practice, so a colon farther in is treated as body punctuation, not a sender.
    char sender[10] = "";
    const char* body = inbox[idx].text;
    const char* colon = strchr(inbox[idx].text, ':');
    if (colon && (colon - inbox[idx].text) < 9) {
        int slen = colon - inbox[idx].text;
        strncpy(sender, inbox[idx].text, slen);
        sender[slen] = '\0';
        body = colon + 1;
        while (*body == ' ') body++;
    }

    // Init scroll (skip when called from scroll animation)
    if (!scrollInProgress) {
        int cpl = 128 / 12;   // size-2 char = 12px wide → 10 chars/line
        int lines = ((int)strlen(body) + cpl - 1) / cpl;
        int textH = lines * 16;  // size-2 char = 16px tall
        scrollMax = textH > BODY_H ? textH - BODY_H : 0;
        scrollY = 0;
        scrollState = 0;
        scrollTimer = millis();
    }

    oled.clearDisplay();

    // Body first (may bleed into header area during scroll — header covers it)
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setTextWrap(true);
    oled.setCursor(0, BODY_TOP - scrollY);
    oled.print(body);

    // Header on top (covers any body bleed)
    oled.fillRect(0, 0, 128, BODY_TOP, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setTextSize(1);
    oled.setCursor(2, 1);
    if (strlen(sender) > 0) {
        oled.print(sender);
    } else {
        const char* alias = ricNameFor(inbox[idx].addr);
        if (alias) {
            oled.print(alias);
        } else {
            char ric[12];
            snprintf(ric, sizeof(ric), "%lu", inbox[idx].addr);
            oled.print(ric);
        }
    }
    char ts[8];
    formatClock(ts, sizeof(ts));
    oled.setCursor(100, 1);
    oled.print(ts);

    oled.display();
}

// ── INBOX: list of messages ──
// Shows 4 messages at a time, each gets 13px row
#define INBOX_ROWS  4
#define ROW_H       13
#define LIST_Y      10

void drawInbox() {
    oled.clearDisplay();
    drawMiniBar();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);

    if (msgCount == 0) {
        oled.setCursor(16, 30);
        oled.print(T(STR_NO_MESSAGES));
        oled.display();
        return;
    }

    // Which page are we on?
    int page = viewIdx / INBOX_ROWS;
    int startIdx = page * INBOX_ROWS;

    for (int row = 0; row < INBOX_ROWS; row++) {
        int idx = startIdx + row;
        if (idx >= msgCount) break;

        int y = LIST_Y + row * ROW_H;
        bool sel = (idx == viewIdx);

        // Selection highlight
        if (sel) {
            oled.fillRect(0, y, 128, ROW_H, SSD1306_WHITE);
            oled.setTextColor(SSD1306_BLACK);
        } else {
            oled.setTextColor(SSD1306_WHITE);
        }

        // Unread dot
        if (inbox[idx].unread) {
            int dotColor = sel ? SSD1306_BLACK : SSD1306_WHITE;
            oled.fillCircle(3, y + ROW_H / 2, 2, dotColor);
        }

        // Message preview: first ~18 chars of text
        oled.setCursor(9, y + 3);
        char preview[19];
        strncpy(preview, inbox[idx].text, 18);
        preview[18] = '\0';
        oled.print(preview);

        // Separator line (not on selected or last)
        if (!sel && row < INBOX_ROWS - 1 && idx + 1 < msgCount) {
            oled.drawFastHLine(8, y + ROW_H - 1, 112,
                               SSD1306_WHITE);
        }
    }

    // Bottom: page indicator + hint
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(2, 57);
    char nav[24];
    int pages = (msgCount + INBOX_ROWS - 1) / INBOX_ROWS;
    snprintf(nav, sizeof(nav), "%d/%d  BTN:open  LONG:back",
             viewIdx + 1, msgCount);
    oled.print(nav);

    oled.display();
}

// ── DETAIL: full message text ──
void drawDetail(int idx) {
    if (idx >= msgCount) return;
    inbox[idx].unread = false;
    stopAlertLed();   // message is now considered read

    // Parse sender
    char sender[10] = "";
    const char* body = inbox[idx].text;
    const char* colon = strchr(inbox[idx].text, ':');
    if (colon && (colon - inbox[idx].text) < 9) {
        int slen = colon - inbox[idx].text;
        strncpy(sender, inbox[idx].text, slen);
        sender[slen] = '\0';
        body = colon + 1;
        while (*body == ' ') body++;
    }

    // Init scroll (skip when called from scroll animation)
    if (!scrollInProgress) {
        int cpl = 128 / 12;
        int lines = ((int)strlen(body) + cpl - 1) / cpl;
        int textH = lines * 16;
        scrollMax = textH > BODY_H ? textH - BODY_H : 0;
        scrollY = 0;
        scrollState = 0;
        scrollTimer = millis();
    }

    oled.clearDisplay();

    // Body first (header covers any bleed)
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setTextWrap(true);
    oled.setCursor(0, BODY_TOP - scrollY);
    oled.print(body);

    // Header on top
    oled.fillRect(0, 0, 128, BODY_TOP, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setTextSize(1);
    oled.setCursor(2, 1);
    if (strlen(sender) > 0) {
        oled.print(sender);
    } else {
        const char* alias = ricNameFor(inbox[idx].addr);
        if (alias) {
            oled.print(alias);
        } else {
            char ric[12];
            snprintf(ric, sizeof(ric), "%lu", inbox[idx].addr);
            oled.print(ric);
        }
    }
    char info[16];
    char age[8];
    formatAge(inbox[idx].timestamp, inbox[idx].tsIsEpoch, age, sizeof(age));
    snprintf(info, sizeof(info), "%d/%d %s", idx + 1, msgCount, age);
    int iw = strlen(info) * 6;
    oled.setCursor(128 - iw - 2, 1);
    oled.print(info);

    oled.display();
}

// ── DAPNET splash bitmap (128×64, 1bpp) ──
// Sourced from the original ON6RF firmware
// (https://github.com/ManoDaSilva/ESP32-Pocsag-Pager — main.cpp `displayLogo[]`).
// 1024 B in PROGMEM. Drawn once on boot to identify the network the pager rides on.
static const uint8_t dapnetLogo[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x03, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x07, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x07, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0xe1, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0xc0, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x80, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x80, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x80, 0xf8, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0xc0, 0xf8, 0x00, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0xe1, 0xfc, 0x01, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x07, 0xff, 0xff, 0x07, 0xff, 0xf8, 0x0f, 0xe0, 0x1c, 0x1f, 0xc3, 0x03, 0x3f, 0xef, 0xfc,
    0x00, 0x07, 0xff, 0xff, 0x8f, 0xff, 0xfc, 0x0f, 0xf8, 0x1c, 0x1f, 0xe3, 0x83, 0x3f, 0xef, 0xfc,
    0x00, 0x03, 0xff, 0xef, 0xff, 0xf3, 0xfe, 0x0c, 0x18, 0x36, 0x18, 0x73, 0xc3, 0x30, 0x00, 0xc0,
    0x00, 0x01, 0xff, 0xc7, 0xff, 0xc0, 0xfe, 0x0c, 0x0c, 0x36, 0x18, 0x33, 0xc3, 0x30, 0x00, 0xc0,
    0x00, 0x00, 0x7f, 0x01, 0xff, 0x00, 0x3e, 0x0c, 0x0c, 0x36, 0x18, 0x73, 0x63, 0x30, 0x00, 0xc0,
    0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x3f, 0x0c, 0x0c, 0x63, 0x1f, 0xe3, 0x33, 0x3f, 0xe0, 0xc0,
    0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x3f, 0x0c, 0x0c, 0x63, 0x1f, 0xc3, 0x33, 0x3f, 0xe0, 0xc0,
    0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x1f, 0x8c, 0x0c, 0x7f, 0x18, 0x03, 0x1b, 0x30, 0x00, 0xc0,
    0x00, 0x00, 0xe0, 0x00, 0x7c, 0x00, 0x0f, 0x8c, 0x0c, 0xff, 0x98, 0x03, 0x0f, 0x30, 0x00, 0xc0,
    0x00, 0x0f, 0xfe, 0x00, 0x7c, 0x00, 0x0f, 0x8c, 0x18, 0xc1, 0x98, 0x03, 0x0f, 0x30, 0x00, 0xc0,
    0x00, 0x3f, 0xfe, 0x00, 0x7c, 0x00, 0x0f, 0x8f, 0xf8, 0xc1, 0x98, 0x03, 0x07, 0x3f, 0xe0, 0xc0,
    0x00, 0x7f, 0xff, 0xc0, 0x7c, 0x00, 0x1f, 0x8f, 0xe1, 0x80, 0xd8, 0x03, 0x03, 0x3f, 0xe0, 0xc0,
    0x00, 0xff, 0xff, 0xe0, 0x3e, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x03, 0xff, 0x1f, 0xf1, 0xfe, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0xf8, 0x07, 0xff, 0xff, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0xf0, 0x01, 0xff, 0xff, 0x80, 0x7e, 0x3e, 0x1c, 0x1e, 0x7d, 0xf0, 0x00, 0xc6, 0x70, 0x18,
    0x0f, 0xe0, 0x00, 0xff, 0xff, 0xe1, 0xfe, 0x33, 0x1c, 0x33, 0x61, 0x98, 0x00, 0xc6, 0xd8, 0x38,
    0x0f, 0xc0, 0x00, 0xff, 0xdf, 0xff, 0xfe, 0x33, 0x36, 0x60, 0x61, 0x98, 0x00, 0x6c, 0xd8, 0x78,
    0x0f, 0x80, 0x00, 0x7e, 0x07, 0xff, 0xf8, 0x33, 0x36, 0x60, 0x7d, 0x98, 0x00, 0x6c, 0xd8, 0x58,
    0x0f, 0x80, 0x00, 0x3e, 0x03, 0xff, 0xf0, 0x3e, 0x36, 0x67, 0x61, 0xf0, 0x00, 0x6c, 0xd8, 0x18,
    0x1f, 0x00, 0x00, 0x3e, 0x00, 0xff, 0xc0, 0x30, 0x7f, 0x63, 0x61, 0xb0, 0x70, 0x6c, 0xd8, 0x18,
    0x0f, 0x00, 0x00, 0x3e, 0x00, 0x3f, 0x00, 0x30, 0x63, 0x33, 0x61, 0x98, 0x00, 0x38, 0xdb, 0x18,
    0x1f, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x30, 0x63, 0x1e, 0x7d, 0x8c, 0x00, 0x38, 0x73, 0x18,
    0x1f, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0f, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x1f, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0f, 0x80, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0f, 0x80, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0f, 0x80, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0f, 0xc0, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0xe0, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0xf0, 0x07, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x03, 0xfc, 0x1f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x7f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ── SPLASH: shown for ~1.5 s right after OLED init, before the radio comes up ──
void drawSplash() {
    oled.clearDisplay();
    oled.drawBitmap(0, 0, dapnetLogo, 128, 64, SSD1306_WHITE);
    oled.display();
}

// ─── Alert LED ──────────────────────────────────────────
// Writes the external LED with the configured polarity. No-op if no
// external LED is wired (extLedPin < 0). Updates extLedState for the
// software-blink toggle in tickAlertLed().
void extLedWrite(bool on) {
    if (cfg.extLedPin < 0) return;
    bool level = cfg.extLedActiveHigh ? on : !on;
    digitalWrite(cfg.extLedPin, level ? HIGH : LOW);
    extLedState = on;
}

void startAlertLed() {
    if (!cfg.ledBlink) return;
    ledMode       = LED_BLINKING;
    ledStartedAt  = millis();
    ledLastToggle = millis();
    pmu.setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
    extLedWrite(true);   // first half-cycle of the blink
}

// Called when the message is read (detail opened) or acknowledged (button press).
// We force the CHGLED OFF (not CTRL_CHG) so it doesn't keep glowing as a
// charging indicator while plugged in — that looks identical to a stuck
// alert from the user's POV. Charge status is visible on the OLED instead.
void stopAlertLed() {
    if (ledMode == LED_OFF) return;   // skip needless I²C writes
    ledMode = LED_OFF;
    pmu.setChargingLedMode(XPOWERS_CHG_LED_OFF);
    extLedWrite(false);
}

// Called from loop() every iteration. Cheap when LED_OFF.
// PMU does its own blinking; we only need to toggle the external LED here
// and detect the BLINKING → STEADY transition.
void tickAlertLed() {
    if (ledMode == LED_OFF) return;
    unsigned long now = millis();

    if (ledMode == LED_BLINKING) {
        if (now - ledStartedAt >= LED_BLINK_MS) {
            ledMode = LED_STEADY;
            pmu.setChargingLedMode(XPOWERS_CHG_LED_ON);
            extLedWrite(true);   // pin held HIGH for the steady "unread" signal
            return;
        }
        if (cfg.extLedPin >= 0 && now - ledLastToggle >= LED_BLINK_HALF_MS) {
            extLedWrite(!extLedState);
            ledLastToggle = now;
        }
    }
    // LED_STEADY: nothing to do — both LEDs already solid on.
}

// ─── Buzzer ─────────────────────────────────────────────
// Plays a tone for `ms` milliseconds, blocking. Volume comes from
// cfg.buzzerVolume (0..100), implemented as PWM duty cycle on the LEDC
// peripheral — ~50% duty (max for a square wave) maps to volume 100.
// Returns immediately if the buzzer is disabled in config (no audible
// output AND no time wasted on blocking delays).
void playTone(uint16_t freq, uint16_t ms) {
    if (!cfg.buzzer || cfg.buzzerVolume <= 0 || freq == 0 || ms == 0) return;
    uint8_t duty = (cfg.buzzerVolume * 128) / 100;   // cap at 128/256 = 50% (square-wave max)
    if (duty < 1) duty = 1;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(BUZZER, freq, 8);
    ledcWrite(BUZZER, duty);
    delay(ms);
    ledcWrite(BUZZER, 0);
    ledcDetach(BUZZER);
#else
    ledcSetup(0, freq, 8);
    ledcAttachPin(BUZZER, 0);
    ledcWrite(0, duty);
    delay(ms);
    ledcWrite(0, 0);
    ledcDetachPin(BUZZER);
#endif
}

void beepAlert() {
    if (!cfg.buzzer || cfg.buzzerVolume <= 0) return;
    for (int i = 0; i < 3; i++) {
        playTone(2730, 100);
        delay(30);
        playTone(3200, 100);
        delay(30);
    }
}

// ═══════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    Serial.println(F("\n[Pager] T-Beam v1.2 POCSAG v3"));

    disableRadios();
    pinMode(BUZZER, OUTPUT);
    pinMode(BUTTON_USR, INPUT);
    Wire.begin(OLED_SDA, OLED_SCL);
    Wire.setTimeOut(50);

    loadConfig();

    // RTC recovery (3 sources, checked in order):
    // 1. ESP32 internal RTC — survives soft resets, stays powered while LiPo connected
    // 2. SPIFFS /rtc.txt    — survives full power-off if coin cell keeps flash
    // 3. Skyper OTA          — definitive source, arrives within ~5 min of boot
    time_t bootTime = time(nullptr);
    if (bootTime > 1700000000) {   // already valid (survived soft reset)
        setenv("TZ", cfg.tz, 1);
        tzset();
        rtcValid = true;
        Serial.println(F("[RTC] Time survived reset"));
    } else {
        File tf = SPIFFS.open("/rtc.txt", "r");
        if (tf) {
            time_t saved = (time_t)tf.readStringUntil('\n').toInt();
            tf.close();
            if (saved > 1700000000) {
                struct timeval tv = { .tv_sec = saved, .tv_usec = 0 };
                settimeofday(&tv, NULL);
                setenv("TZ", cfg.tz, 1);
                tzset();
                rtcValid = true;
                Serial.printf("[RTC] Restored from SPIFFS: %lu (may be stale)\n",
                              (unsigned long)saved);
            }
        }
    }

    loadSavedMessages();

    // ── AXP2101 PMU init (order-sensitive — must come before OLED) ──
    // PMU and OLED share the same I²C bus (SDA 21, SCL 22).
    if (!pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, OLED_SDA, OLED_SCL)) {
        Serial.println(F("[PMU] FATAL!"));
        while (true) delay(1000);
    }
    pmu.setALDO2Voltage(3300);
    pmu.enableALDO2();
    pmu.setDC1Voltage(3300);
    pmu.enableDC1();
    pmu.disableALDO3();   // GPS off — keep NEO-6M unpowered to save ~30 mA
    pmu.disableBLDO1();
    pmu.disableBLDO2();
    pmu.disableDLDO1();
    pmu.disableDLDO2();

    // VBUS input limits — without this, defaults to ~100 mA and charger is starved
    pmu.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);
    pmu.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA);

    // Battery protection — shutdown below 2.6V to prevent deep discharge
    pmu.setSysPowerDownVoltage(2600);

    // T-Beam has no NTC on the TS pin — without this the AXP2101 reads
    // out-of-range temp and drops to trickle charge while still reporting isCharging()=true
    pmu.disableTSPinMeasure();

    // Enable ADC channels for battery monitoring
    pmu.enableBattDetection();
    pmu.enableBattVoltageMeasure();
    pmu.enableVbusVoltageMeasure();
    pmu.enableSystemVoltageMeasure();

    pmu.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
    pmu.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);
    pmu.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_200MA);
    pmu.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);

    // CHGLED reserved for alert use only — kill the default charging-state
    // indicator so a plugged-in device doesn't look like it has a stuck alert.
    pmu.setChargingLedMode(XPOWERS_CHG_LED_OFF);

    // Optional external alert LED — opt-in via cfg.extLedPin (-1 = disabled).
    if (cfg.extLedPin >= 0) {
        pinMode(cfg.extLedPin, OUTPUT);
        extLedWrite(false);
    }

    delay(100);

    // OLED
    oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    // Boot splash — stays up while radio init runs below; the brief delay
    // also gives the SSD1306 panel time to settle after first power-on.
    drawSplash();
    oledOn = true;
    lastActivity = millis();
    delay(1500);

    // Radio. POCSAG is 2-FSK modulation (not LoRa), so we take the SX1276 out
    // of its default LoRa mode and into FSK mode before handing it to PagerClient.
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    int state = radio.beginFSK();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[Radio] FAIL %d\n", state);
        oled.clearDisplay();
        oled.setTextSize(2);
        oled.setTextColor(SSD1306_WHITE);
        oled.setCursor(0, 20);
        oled.print(F("Radio\nFAIL!"));
        oled.display();
        while (true) delay(1000);
    }

    // 1200 baud = POCSAG-1200 (a.k.a. POCSAG-2). DAPNET transmits almost exclusively
    // at this rate; POCSAG-512 and POCSAG-2400 exist but aren't used on DAPNET.
    state = pager.begin(cfg.freq + cfg.offset, 1200);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[Pager] FAIL %d\n", state);
        while (true) delay(1000);
    }

    // Build RIC filter: user RICs from config + TIME_RIC for clock sync.
    // Arrays must be static — RadioLib stores pointers, not copies.
    // Mask 0xFFFFF = all 20 POCSAG address bits significant → exact-match filter.
    // A narrower mask would accept prefix/group matches we don't want.
    static uint32_t rxAddrs[CFG_MAX_RICS + 1];
    static uint32_t rxMasks[CFG_MAX_RICS + 1];
    int n = 0;
    for (int i = 0; i < cfg.ricCount && n < CFG_MAX_RICS; i++) {
        rxAddrs[n] = cfg.rics[i].ric;
        rxMasks[n] = 0xFFFFF;
        n++;
    }
    rxAddrs[n] = TIME_RIC;
    rxMasks[n] = 0xFFFFF;
    n++;
    state = pager.startReceive(LORA_DIO2, rxAddrs, rxMasks, n);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[RX] FAIL %d\n", state);
        while (true) delay(1000);
    }

    Serial.printf("[OK] %d RICs + time @ %.4f MHz\n", cfg.ricCount, cfg.freq + cfg.offset);

    playTone(2000, 100);
    delay(50);
    playTone(3000, 100);

    // Launch POCSAG reception on Core 0. readData() can block indefinitely
    // when a transmission ends mid-frame, but the UI on Core 1 stays responsive.
    // Queue depth 4 is plenty: POCSAG batches arrive ~1/sec max and Core 1
    // drains the queue on every loop() (~100 Hz).
    // Stack 4096 covers RadioLib's readData() + the Arduino String it returns;
    // priority 1 matches the default Arduino loopTask.
    rxQueue = xQueueCreate(4, sizeof(RxMsg));
    xTaskCreatePinnedToCore(rxTask, "rx", 4096, nullptr, 1, nullptr, 0);
    // Reconfigure WDT to only watch Core 1's idle task. Core 0's idle task
    // won't run while rxTask is blocked in readData() — that's expected.
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms    = 5000,
        .idle_core_mask = (1 << 1),
        .trigger_panic = true
    };
    esp_task_wdt_reconfigure(&wdt_cfg);

    Serial.println(F("[OK] rxTask on Core 0, UI on Core 1"));

    updateBattery();
    drawIdle();
    lastActivity = millis();
    enableLoopWDT();
}

// ═══════════════════════════════════════════════════════
//  LOOP  (runs on Core 1 — never blocks, always responsive)
//
//  Responsibilities: drain RX queue, handle button, auto-scroll,
//  battery refresh, OLED auto-blank. ~100 Hz polling rate (delay(10)).
// ═══════════════════════════════════════════════════════

void loop() {
    // ── Drain POCSAG messages from Core 0 queue (non-blocking) ──
    RxMsg rx;
    while (xQueueReceive(rxQueue, &rx, 0) == pdTRUE) {
        Serial.printf("\n>>> RIC:%lu | %s\n", rx.addr, rx.text);

        if (rx.addr == TIME_RIC) {
            tryParseSkyperTime(rx.text);
            if (rtcValid && screen == SCR_IDLE && oledOn) drawIdle();
        } else if (addrIsUserRic(rx.addr)) {
            // Fragment reassembly: RadioLib returns ~50 chars per batch.
            // If same RIC within 5s, append to previous message.
            bool isContinuation = (msgCount > 0 &&
                                   inbox[0].addr == rx.addr &&
                                   millis() - lastMsgTime < FRAG_WINDOW_MS);
            if (isContinuation) {
                int existing = strlen(inbox[0].text);
                if (existing < MAX_LEN - 1) {
                    strlcpy(inbox[0].text + existing, rx.text, MAX_LEN - existing);
                    saveMessageToFS(inbox[0]);
                    Serial.printf("[FRAG] appended → %d chars\n", (int)strlen(inbox[0].text));
                }
            } else {
                storeMessage(rx.addr, rx.text);
                beepAlert();      // only beep on first fragment
                startAlertLed();  // visual cue in case the buzzer is missed
            }
            lastMsgTime = millis();
            wakeOLED();
            drawAlert(0);
            lastActivity = millis();
            screen = SCR_ALERT;
        }
    }

    // ── Button handling ──
    bool btnDown = (digitalRead(BUTTON_USR) == LOW);

    if (btnDown && !btnWasDown) {
        // Button just pressed — acknowledges any pending alert blink
        btnDownTime = millis();
        btnWasDown = true;
        stopAlertLed();
    }

    if (!btnDown && btnWasDown) {
        // Button just released
        unsigned long held = millis() - btnDownTime;
        btnWasDown = false;
        wakeOLED();
        lastActivity = millis();

        if (held > 800) {
            // ── LONG PRESS: go to idle ──
            screen = SCR_IDLE;
            drawIdle();

        } else {
            // ── SHORT PRESS: depends on current screen ──
            switch (screen) {
                case SCR_IDLE:
                    // Always redraw idle first (wakes from blank)
                    drawIdle();
                    if (msgCount > 0) {
                        viewIdx = 0;
                        screen = SCR_INBOX;
                        drawInbox();
                    }
                    break;

                case SCR_ALERT:
                    viewIdx = 0;
                    screen = SCR_INBOX;
                    drawInbox();
                    break;

                case SCR_INBOX:
                    // Open selected message in detail
                    inbox[viewIdx].unread = false;
                    screen = SCR_DETAIL;
                    drawDetail(viewIdx);
                    break;

                case SCR_DETAIL:
                    // Next message, wrap around
                    viewIdx = (viewIdx + 1) % msgCount;
                    if (viewIdx == 0) {
                        // Wrapped around → back to inbox
                        screen = SCR_INBOX;
                        drawInbox();
                    } else {
                        drawDetail(viewIdx);
                    }
                    break;
            }
        }
    }

    // Navigation model: short press in inbox opens detail; in detail,
    // short press cycles to next message (wraps to inbox at end).
    // No dedicated scroll gesture — cursor moves through the cycle.

    // ── Auto-scroll for messages longer than the visible body area ──
    if (scrollMax > 0 && (screen == SCR_ALERT || screen == SCR_DETAIL)) {
        unsigned long now = millis();
        if (scrollState == 0 && now - scrollTimer >= SCROLL_PAUSE_MS) {
            scrollState = 1;
            scrollTimer = now;
        } else if (scrollState == 1 && now - scrollTimer >= SCROLL_STEP_MS) {
            scrollY++;
            scrollTimer = now;
            if (scrollY >= scrollMax) { scrollState = 2; scrollTimer = now; }
            scrollInProgress = true;
            if (screen == SCR_ALERT) drawAlert(0);
            else                     drawDetail(viewIdx);
            scrollInProgress = false;
        } else if (scrollState == 2 && now - scrollTimer >= SCROLL_PAUSE_MS) {
            scrollY = 0;
            scrollState = 0;
            scrollTimer = now;
            scrollInProgress = true;
            if (screen == SCR_ALERT) drawAlert(0);
            else                     drawDetail(viewIdx);
            scrollInProgress = false;
        }
    }

    // ── Battery refresh ──
    if (millis() - lastBatUpdate > 30000) {
        updateBattery();
        if (screen == SCR_IDLE && oledOn) drawIdle();
    }

    // ── Auto-blank OLED on idle screen after 10s ──
    // Only blanks on SCR_IDLE — inbox/detail/alert stay lit indefinitely
    // so the user can browse without the display timing out.
    // Alert screen stays lit until user acknowledges with a button press.
    if (oledOn && screen == SCR_IDLE &&
        millis() - lastActivity > 10000) {
        sleepOLED();
    }

    tickAlertLed();

    delay(10);  // ~100 Hz polling rate for button + queue
}
