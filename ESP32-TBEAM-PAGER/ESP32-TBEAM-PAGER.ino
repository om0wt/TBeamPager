/*
  POCSAG Pager — T-Beam v1.2 (AXP2101) — v3

  Board: ESP32 Dev Module, Upload Speed: 115200, Partition: Default 4MB w/ 1.5MB SPIFFS
  Libraries: RadioLib, XPowersLib, Adafruit SSD1306, Adafruit GFX, ArduinoJson
  Filesystem: upload /data via "ESP32 Sketch Data Upload" (SPIFFS)
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

float frequency = 439.98750;
float offset    = 0.0044;
#define TIME_RIC    2504       // DAPNET Skyper OTA Time broadcast

// ─── Config (loaded from /config.json on SPIFFS) ────────
#define CFG_MAX_RICS 8

struct Config {
    uint32_t rics[CFG_MAX_RICS];
    int      ricCount;
    char     lang[4];            // "en" | "sk" | "fr" | "es" | "pt"
    bool     storeMessages;
    char     messageFolder[24];
};

Config cfg = {
    .rics          = { 1234567 },
    .ricCount      = 1,
    .lang          = "en",
    .storeMessages = true,
    .messageFolder = "/msgs"
};

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

// ─── Messages ───────────────────────────────────────────
#define MAX_MSGS   20
#define MAX_LEN    200

struct PagerMsg {
    uint32_t addr;
    char     text[MAX_LEN];
    uint32_t timestamp;      // epoch seconds (wall clock) alebo millis()/1000 uptime
    bool     tsIsEpoch;      // true → timestamp je reálny čas, false → uptime
    bool     unread;
};

PagerMsg inbox[MAX_MSGS];
int msgCount  = 0;
int totalMsgs = 0;
int viewIdx   = 0;

// ─── UI ─────────────────────────────────────────────────
enum Screen { SCR_IDLE, SCR_ALERT, SCR_INBOX, SCR_DETAIL };
Screen screen = SCR_IDLE;
unsigned long lastActivity  = 0;
unsigned long lastBatUpdate = 0;
unsigned long btnDownTime   = 0;
bool oledOn = true;
bool btnWasDown = false;
float batVoltage = 0;
int batPercent   = 0;
bool rtcValid = false;       // true po prvom Skyper OTA Time sync

// ─── Objects ────────────────────────────────────────────
SX1276 radio = new Module(LORA_SS, LORA_DIO0, LORA_RST, LORA_DIO1);
PagerClient pager(&radio);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
XPowersAXP2101 pmu;

// ═══════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════

void disableRadios() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    esp_wifi_stop();
    esp_bt_controller_disable();
    setCpuFrequencyMhz(80);
}

void sleepOLED() {
    oled.clearDisplay();
    oled.display();        // just blank the screen
    oledOn = false;
}

void wakeOLED() {
    oledOn = true;         // that's it — display is always on, just blank
}

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

// Set ESP32 system time from Skyper OTA Time broadcast (UTC input).
void setTimeFromSkyper(int hh, int mm, int ss, int dd, int mo, int yy) {
    setenv("TZ", "UTC0", 1);
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
    // Slovensko: CET (+1) / CEST (+2), auto DST podla POSIX TZ stringu
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();
    rtcValid = true;
    Serial.printf("[RTC] Sync: 20%02d-%02d-%02d %02d:%02d:%02d UTC\n",
                  yy, mo, dd, hh, mm, ss);
}

// Parse "[Skyper OTA Time] HHMMSS DDMMYY" (or bare "HHMMSS DDMMYY").
// Returns true if matched and clock was set.
bool tryParseSkyperTime(const char* text) {
    const char* p = strstr(text, "] ");
    p = p ? p + 2 : text;
    int hh, mm, ss, dd, mo, yy;
    if (sscanf(p, "%2d%2d%2d %2d%2d%2d", &hh, &mm, &ss, &dd, &mo, &yy) == 6) {
        if (hh < 24 && mm < 60 && ss < 60 &&
            dd >= 1 && dd <= 31 && mo >= 1 && mo <= 12) {
            setTimeFromSkyper(hh, mm, ss, dd, mo, yy);
            return true;
        }
    }
    return false;
}

// ═══════════════════════════════════════════════════════
//  CONFIG  (SPIFFS /config.json)
// ═══════════════════════════════════════════════════════

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

    JsonArray ra = doc["rics"].as<JsonArray>();
    if (ra) {
        cfg.ricCount = 0;
        for (JsonVariant v : ra) {
            if (cfg.ricCount >= CFG_MAX_RICS) break;
            cfg.rics[cfg.ricCount++] = v.as<uint32_t>();
        }
    }
    if (doc["lang"].is<const char*>())
        strlcpy(cfg.lang, doc["lang"], sizeof(cfg.lang));
    cfg.storeMessages = doc["storeMessages"] | false;
    if (doc["messageFolder"].is<const char*>())
        strlcpy(cfg.messageFolder, doc["messageFolder"], sizeof(cfg.messageFolder));

    langIdx = langCodeToIdx(cfg.lang);

    Serial.printf("[CFG] lang=%s store=%s folder=%s rics=",
                  cfg.lang, cfg.storeMessages ? "y" : "n", cfg.messageFolder);
    for (int i = 0; i < cfg.ricCount; i++)
        Serial.printf("%lu%s", cfg.rics[i], i + 1 < cfg.ricCount ? "," : "\n");
}

bool addrIsUserRic(uint32_t addr) {
    for (int i = 0; i < cfg.ricCount; i++)
        if (cfg.rics[i] == addr) return true;
    return false;
}

// ═══════════════════════════════════════════════════════
//  MESSAGE PERSISTENCE  (SPIFFS)
//  Path:   <folder>/<ric>_<YYYYMMDDHHMMSS>.txt   (ak rtcValid)
//          <folder>/<ric>_u<uptimeSec>.txt       (fallback pred time sync)
//  Body:   "<epoch>\t<ric>\t<text>\n"
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
    // Proti kolizii (dve spravy v rovnakej sekunde) — pridame sufix.
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

    // SPIFFS je flat — iteraciou cez "/" prejdeme vsetky subory, filtrujeme prefix folderu,
    // a triedime podla timestamp-u z obsahu (nie filename-u, kedze ten zacina RIC).
    File root = SPIFFS.open("/");
    if (!root) return;

    struct LoadedMsg {
        uint32_t ts;
        uint32_t ric;
        char     text[MAX_LEN];
        bool     tsIsEpoch;
    };
    LoadedMsg loaded[MAX_MSGS * 2];
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
                    lm.tsIsEpoch = (lm.ts > 1000000000UL);
                    loadedCount++;
                }
            }
        }
        entry = root.openNextFile();
    }

    // Sort vzostupne podla ts (najstarsie prve).
    for (int i = 0; i < loadedCount - 1; i++)
        for (int j = i + 1; j < loadedCount; j++)
            if (loaded[i].ts > loaded[j].ts) {
                LoadedMsg t = loaded[i]; loaded[i] = loaded[j]; loaded[j] = t;
            }

    // Nacitaj poslednych MAX_MSGS (najnovsie ako posledne → po shift-e konci na inbox[0]).
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

// ─── Store message with ASCII filter ────────────────────
void storeMessage(uint32_t addr, const char* text) {
    if (msgCount < MAX_MSGS) msgCount++;
    for (int i = msgCount - 1; i > 0; i--) inbox[i] = inbox[i - 1];
    inbox[0].addr = addr;

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

// ── Tiny status bar (just 8px tall) ──
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
        char freq[10];
        snprintf(freq, sizeof(freq), "%.3f", frequency);  // e.g. "439.988"
        oled.print(freq);
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

    // RIC list (up to 3 shown)
    oled.setCursor(4, 34);
    oled.print(F("RIC "));
    for (int i = 0; i < cfg.ricCount && i < 3; i++) {
        if (i) oled.print(F(","));
        oled.print(cfg.rics[i]);
    }
    if (cfg.ricCount > 3) oled.print(F("+"));

    // Messages summary
    oled.setCursor(4, 44);
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

// ── ALERT: BIG message text, thin sender+time header ──
void drawAlert(int idx) {
    if (idx >= msgCount) return;
    oled.clearDisplay();

    // Parse sender from message (DAPNET format: "CALL: text")
    char sender[10] = "";
    const char* body = inbox[idx].text;
    const char* colon = strchr(inbox[idx].text, ':');
    if (colon && (colon - inbox[idx].text) < 9) {
        int slen = colon - inbox[idx].text;
        strncpy(sender, inbox[idx].text, slen);
        sender[slen] = '\0';
        body = colon + 1;
        while (*body == ' ') body++;  // skip space after colon
    }

    // Header: sender left, time right (9px)
    oled.fillRect(0, 0, 128, 9, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setTextSize(1);
    oled.setCursor(2, 1);
    if (strlen(sender) > 0) {
        oled.print(sender);
    } else {
        char ric[12];
        snprintf(ric, sizeof(ric), "%lu", inbox[idx].addr);
        oled.print(ric);
    }

    // Time on right (wall clock if rtcValid, else "--:--")
    char ts[8];
    formatClock(ts, sizeof(ts));
    oled.setCursor(100, 1);
    oled.print(ts);

    // Message body in BIG font — 55px = 3 lines of size 2
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setTextWrap(true);
    oled.setCursor(0, 14);
    oled.print(body);

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

// ── DETAIL: full message in BIG font ──
void drawDetail(int idx) {
    if (idx >= msgCount) return;
    inbox[idx].unread = false;
    oled.clearDisplay();

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

    // Header: sender + position + age (9px)
    oled.fillRect(0, 0, 128, 9, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setTextSize(1);
    oled.setCursor(2, 1);
    if (strlen(sender) > 0) {
        oled.print(sender);
    } else {
        char ric[12];
        snprintf(ric, sizeof(ric), "%lu", inbox[idx].addr);
        oled.print(ric);
    }

    // Position + age on right
    char info[16];
    char age[8];
    formatAge(inbox[idx].timestamp, inbox[idx].tsIsEpoch, age, sizeof(age));
    snprintf(info, sizeof(info), "%d/%d %s", idx + 1, msgCount, age);
    int iw = strlen(info) * 6;
    oled.setCursor(128 - iw - 2, 1);
    oled.print(info);

    // Message body BIG
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setTextWrap(true);
    oled.setCursor(0, 14);
    oled.print(body);

    oled.display();
}

// ─── Buzzer ─────────────────────────────────────────────
void beepAlert() {
    for (int i = 0; i < 3; i++) {
        tone(BUZZER, 2730, 100);
        delay(130);
        tone(BUZZER, 3200, 100);
        delay(130);
    }
    noTone(BUZZER);
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

    loadConfig();
    loadSavedMessages();

    // PMU
    if (!pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, OLED_SDA, OLED_SCL)) {
        Serial.println(F("[PMU] FATAL!"));
        while (true) delay(1000);
    }
    pmu.setALDO2Voltage(3300);
    pmu.enableALDO2();
    pmu.setDC1Voltage(3300);
    pmu.enableDC1();
    pmu.disableALDO3();   // GPS off
    pmu.disableBLDO1();
    pmu.disableBLDO2();
    pmu.disableDLDO1();
    pmu.disableDLDO2();

    // VBUS input limits — bez toho ostane input ~100 mA a charger ledva tlaci
    pmu.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);
    pmu.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA);

    // Ochrana batérie
    pmu.setSysPowerDownVoltage(2600);

    // T-Beam nema NTC na batke — bez tohto ide charger do trickle kvoli "over-temp"
    pmu.disableTSPinMeasure();

    // ADC kanaly
    pmu.enableBattDetection();
    pmu.enableBattVoltageMeasure();
    pmu.enableVbusVoltageMeasure();
    pmu.enableSystemVoltageMeasure();

    pmu.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
    pmu.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);
    pmu.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_200MA);
    pmu.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);

    delay(100);

    // OLED
    oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    // Radio
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

    state = pager.begin(frequency + offset, 1200);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[Pager] FAIL %d\n", state);
        while (true) delay(1000);
    }

    // Prijimame user RIC list z konfiguracie + TIME_RIC (Skyper OTA Time).
    // RadioLib PagerClient drzi pointery, tak pole musi byt static a zit po celu dobu.
    static uint32_t rxAddrs[CFG_MAX_RICS + 1];
    static uint32_t rxMasks[CFG_MAX_RICS + 1];
    int n = 0;
    for (int i = 0; i < cfg.ricCount && n < CFG_MAX_RICS; i++) {
        rxAddrs[n] = cfg.rics[i];
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

    Serial.printf("[OK] %d RICs + time @ %.4f MHz\n", cfg.ricCount, frequency + offset);

    tone(BUZZER, 2000, 100);
    delay(150);
    tone(BUZZER, 3000, 100);
    delay(100);
    noTone(BUZZER);

    updateBattery();
    drawIdle();
    lastActivity = millis();
}

// ═══════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════

void loop() {
    static unsigned long lastDbg = 0;
    // if (millis() - lastDbg > 500) {
    //     Serial.printf("BTN38=%d\n", digitalRead(BUTTON_USR));
    //     lastDbg = millis();
    // }

    // ── POCSAG receive ──
    if (pager.available() >= 2) {
        String str;
        uint32_t addr = 0;
        int state = pager.readData(str, 0, &addr);

        if (state == RADIOLIB_ERR_NONE) {
            Serial.printf("\n>>> RIC:%lu | %s\n", addr, str.c_str());

            if (addr == TIME_RIC) {
                // Skyper OTA time broadcast → set clock, no inbox/beep/screen change
                tryParseSkyperTime(str.c_str());
                if (rtcValid && screen == SCR_IDLE && oledOn) drawIdle();
            } else if (addrIsUserRic(addr)) {
                storeMessage(addr, str.c_str());
                wakeOLED();
                Wire.begin(OLED_SDA, OLED_SCL);
                delay(20);
                drawAlert(0);
                beepAlert();
                lastActivity = millis();
                screen = SCR_ALERT;
            }
            // iné RIC ignorujeme
        }
    }

    // ── Button handling ──
    bool btnDown = (digitalRead(BUTTON_USR) == LOW);

    if (btnDown && !btnWasDown) {
        // Button just pressed
        btnDownTime = millis();
        btnWasDown = true;
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

    // Scroll inbox with repeated presses when in inbox
    // (viewIdx changes happen in short press handler above,
    //  but we need a way to move cursor in inbox list)
    // We reuse: from INBOX, short press opens detail.
    // To move cursor in inbox, we use: from DETAIL, press cycles.
    // Then long press → back to idle.

    // ── Battery refresh ──
    if (millis() - lastBatUpdate > 30000) {
        updateBattery();
        if (screen == SCR_IDLE) drawIdle();
    }

    // ── Auto-sleep OLED after 60s ──
    if (oledOn && screen != SCR_ALERT &&
        millis() - lastActivity > 60000) {
        screen = SCR_IDLE;
        sleepOLED();
    }

    delay(10);
}
