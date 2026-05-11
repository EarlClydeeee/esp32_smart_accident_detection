/*
 * Helmivo — ESP32 smart helmet (WiFi AP+STA, web UI, MPU6050, u-blox GPS, Telegram, GPIO)
 *
 * Pins: Relay G14 (active-LOW = lights on), Buzzer G25, Button G13 (INPUT_PULLUP),
 *       MPU6050 SDA G21 / SCL G22, GPS UART RX G16 / TX G17 (9600)
 *
 * Bot token: optional file secrets.h (#define HELMIVO_BOT_TOKEN "..."). Listed in .gitignore.
 * Chat ID + optional token override: web UI (NVS). Rotate token with @BotFather if leaked.
 *
 * TESTER ADDITIONS (2025):
 *   GET  /tester             — tester panel UI
 *   POST /api/test/accident  — inject a fake accident event (ST_CANCEL_WINDOW)
 *   POST /api/test/flashlight — toggle relay for 5 s (lights test)
 *   POST /api/test/buzzer     — beep buzzer for 3 s
 *   GET  /api/test/posture    — current MPU reading (G, pitch, roll, tilt, gyro)
 *   GET  /api/test/gps        — current GPS lat/lon, fix, sats, age
 *
 *   POST /api/set-telegram    — save chat ID + optional bot token (application/x-www-form-urlencoded body: chat, bot)
 *
 * SERIAL MONITOR ADDITION:
 *   GET  /api/serial-log      — returns JSON array of buffered log lines
 *                               query param: ?since=N  (return only lines after index N)
 *   Ring buffer: SERIAL_LOG_LINES lines of SERIAL_LOG_LINE_LEN chars each.
 *   All Serial.print/println calls are intercepted via a custom Print subclass
 *   that writes to both the UART and the ring buffer simultaneously.
 */

 #include <Wire.h>
 #include <WiFi.h>
 #include <WebServer.h>
 #include <Preferences.h>
 #include <WiFiClientSecure.h>
 #include <HTTPClient.h>
 #include <time.h>
 #include <string.h>
 #include <ctype.h>

#include "secrets.h"

 // Arduino inserts generated prototypes after includes, before this struct's definition.
 struct RawData;

 // ── GPIO ───────────────────────────────────────────────────────────────────
 static const int PIN_RELAY = 14;
 static const int PIN_BUZZER = 25;
 static const int PIN_BUTTON = 13;

 static const int I2C_SDA = 21;
 static const int I2C_SCL = 22;

 // GPS: ESP32 RX <- module TX (G16), ESP32 TX -> module RX (G17)
 static const int GPS_RX_PIN = 16;
 static const int GPS_TX_PIN = 17;

 // ── WiFi AP (always for local UI) ──────────────────────────────────────────
 static const char *AP_SSID = "Helmivo";
 static const char *AP_PASS = "Helmivo";

 // ── Telegram (Preferences "helmivo": keys "chat", "bot") ───────────────────
 // Default token from secrets.h if defined there; else empty until set via web UI.
 #ifndef HELMIVO_BOT_TOKEN
 #define HELMIVO_BOT_TOKEN ""
 #endif

 // ── MPU6050 ───────────────────────────────────────────────────────────────
 #define MPU_ADDR 0x68
 #define REG_PWR_MGMT_1 0x6B
 #define REG_ACCEL_CONFIG 0x1C
 #define REG_GYRO_CONFIG 0x1B
 #define REG_ACCEL_XOUT_H 0x3B
 #define ACCEL_SCALE 4096.0f
 #define GYRO_SCALE 131.0f
 #define ALPHA 0.98f
 #define CALIB_SAMPLES 200

 // Subtle bump (log only, serial)
 #define SUBTLE_G_LO 1.35f
 #define SUBTLE_G_HI 2.49f
 #define SUBTLE_COOLDOWN_MS 400

 // Hard impact edges (same family as before)
 #define IMPACT_LIGHT 2.5f
 #define IMPACT_MODERATE 4.0f
 #define IMPACT_SEVERE 6.0f
 #define IMPACT_REARM 1.3f
 #define IMPACT_COOLDOWN_MS 250

 // Accident / driver state (dashboard + serial)
 #define HIGH_G_IMPACT 4.0f
 #define FREEFALL_G 0.3f
 #define FREEFALL_MIN_MS 100
 #define VIOLENT_SPIN_GYRO_DPS 300.0f
 #define POST_IMPACT_SETTLE_MS 2500
 #define TILT_ACCIDENT_DEG 60.0f
 #define DOWN_Z_MAX_G 0.35f
 #define DOWN_XY_MIN_G 0.65f
 #define INACTIVITY_REQUIRED_MS 10000
 #define MOTION_GYRO_ACTIVE_DPS 25.0f
 #define SOFT_FALL_STILL_MS 30000
 #define RECOVERY_TILT_DEG 35.0f
 #define RECOVERY_AZ_MIN_G 0.65f
 #define RECOVERY_G_BAND_LO 0.85f
 #define RECOVERY_G_BAND_HI 1.20f
 #define CANCEL_WINDOW_MS 10000
 #define ALERT_LOCKOUT_MS 30000

 // Telegram impact pipeline
 #define TELEGRAM_COUNTDOWN_TOTAL_MS 15000UL
 #define TELEGRAM_BUZZER_START_MS 11000UL
 #define EMERGENCY_BLINK_MS 500
 #define BUTTON_HOLD_MANUAL_MS 3000UL

 // ── Tester durations ──────────────────────────────────────────────────────
 #define TEST_FLASHLIGHT_MS 5000UL
 #define TEST_BUZZER_MS     3000UL

 // ── Serial log ring buffer ────────────────────────────────────────────────
 #define SERIAL_LOG_LINES    120      // number of lines kept in memory
 #define SERIAL_LOG_LINE_LEN  96      // max chars per line (truncated if longer)

 struct SerialLogEntry {
   char     text[SERIAL_LOG_LINE_LEN];
   uint32_t seq;          // monotonically-increasing sequence number
   uint32_t tsMs;         // millis() at time of write
 };

 static SerialLogEntry  s_logBuf[SERIAL_LOG_LINES];
 static int             s_logHead = 0;    // next write index (wraps)
 static uint32_t        s_logSeq  = 0;    // next sequence number
 static char            s_lineBuf[SERIAL_LOG_LINE_LEN * 2];  // assembly buffer
 static size_t          s_lineBufLen = 0;

 // Push a completed line into the ring buffer
 static void pushLogLine(const char *line) {
   SerialLogEntry &e = s_logBuf[s_logHead];
   e.seq  = s_logSeq++;
   e.tsMs = (uint32_t)millis();
   strncpy(e.text, line, SERIAL_LOG_LINE_LEN - 1);
   e.text[SERIAL_LOG_LINE_LEN - 1] = '\0';
   s_logHead = (s_logHead + 1) % SERIAL_LOG_LINES;
 }

 // Custom Print that tees to HardwareSerial AND ring buffer
 class TeeSerial : public Print {
 public:
   HardwareSerial &hw;
   TeeSerial(HardwareSerial &s) : hw(s) {}

   size_t write(uint8_t c) override {
     hw.write(c);
     if (c == '\n') {
       s_lineBuf[s_lineBufLen] = '\0';
       pushLogLine(s_lineBuf);
       s_lineBufLen = 0;
     } else if (c != '\r') {
       if (s_lineBufLen < sizeof(s_lineBuf) - 1)
         s_lineBuf[s_lineBufLen++] = (char)c;
     }
     return 1;
   }

   size_t write(const uint8_t *buf, size_t len) override {
     size_t n = 0;
     while (n < len) n += write(buf[n]);
     return n;
   }

   // Passthrough helpers so existing Serial.printf etc. compile
   void begin(unsigned long baud) { hw.begin(baud); }
   int  available()                 { return hw.available(); }
   int  read()                      { return hw.read(); }
   int  peek()                      { return hw.peek(); }
   void flush()                     { hw.flush(); }
 };

 // Replace Serial with tee — all existing Serial.print/println/printf calls
 // automatically log to ring buffer without any other code changes.
 static TeeSerial TSerial(Serial);
 #define Serial TSerial

 // ── Web & storage ─────────────────────────────────────────────────────────
 WebServer server(80);
 Preferences prefs;

 String storedSsid;
 String storedWifiPass;
 String storedChatId;
 String storedBotToken;

 // ── GPS / time ───────────────────────────────────────────────────────────
 struct GpsState {
   double lat = 0, lon = 0;
   bool fix = false;
   uint8_t satellites = 0;
   char lastRawTime[16] = {0};
   unsigned long lastFixMs = 0;
 } gps;

 HardwareSerial gpsSerial(2);
 unsigned long lastGpsDrainMs = 0;
 bool ntpConfigured = false;

 // ── MPU ───────────────────────────────────────────────────────────────────
 struct RawData {
   int16_t ax, ay, az;
   int16_t gx, gy, gz;
 };

 float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
 float pitch = 0, roll = 0;
 unsigned long lastTime = 0;
 unsigned long lastImpactMs = 0;
 unsigned long impactCount = 0;
 float peakGSession = 1.0f;
 bool impactArmed = true;
 unsigned long subtleLastMs = 0;
 unsigned long subtleCount = 0;
 float lastSubtleG = 0;

 enum AccidentState {
   ST_NORMAL = 0,
   ST_IMPACT_SETTLE,
   ST_INACTIVITY,
   ST_CANCEL_WINDOW,
   ST_LOCKOUT
 };
 AccidentState accidentState = ST_NORMAL;
 unsigned long stateEnteredMs = 0;
 unsigned long freefallStartMs = 0;
 bool wasInFreefallLatch = false;
 unsigned long softFallSegmentStartMs = 0;
 float eventPeakG = 0;
 float eventPeakTilt = 0;
 uint8_t eventSeverity = 0;
 bool flagImpactDetected = false;
 bool flagDriverDown = false;
 bool flagDriverInactive = false;
 bool flagSoftFallPath = false;
 bool eventHadFreefall = false;
 bool eventViolentSpin = false;
 int cancelLastPrintedSec = -1;

 // ── Telegram / alerts ───────────────────────────────────────────────────────
 enum TgCountdownKind { TG_NONE = 0, TG_IMPACT };
 static TgCountdownKind tgActiveKind = TG_NONE;
 static unsigned long tgCountdownStartMs = 0;
 static bool tgBuzzerPhase = false;
 static unsigned long lastTelegramAttemptMs = 0;
 static char lastTelegramStatus[48] = "idle";
 static char lastTelegramSummary[160] = {0};
 static unsigned long lastTelemetrySerialMs = 0;

 // Button manual
 static unsigned long btnDownSince = 0;
 static bool btnManualLatched = false;
 static bool helmivoTestCountdownPending = false;

 // Relay / buzzer scheduler
 static bool relayEmergencyMode = false;
 static bool relayBlinkState = false;
 static unsigned long relayLastToggleMs = 0;

 static unsigned long manualRelayUntilMs = 0;
 static float g_imuTotalG = 1.0f;
 static float g_imuPitch = 0.0f;
 static float g_imuRoll = 0.0f;

 // ── Tester state ──────────────────────────────────────────────────────────
 static unsigned long testFlashlightUntilMs = 0;
 static unsigned long testBuzzerUntilMs     = 0;
 static float  g_imuTiltDeg  = 0.0f;
 static float  g_imuGyroMag  = 0.0f;

 // ── Forward decls ─────────────────────────────────────────────────────────
 static void mpuWrite(uint8_t reg, uint8_t value);
 static void mpuRead(RawData *out);
 static void loadPrefs();
 static void saveWifi(const String &ssid, const String &pw);
 static void saveTelegram(const String &chat, const String &bot);
 static String telegramSingleLine(const String &s);
 static bool sendTelegramMessage(const String &text);
 static String urlEncode(const String &s);
 static String formatLocalDateTime();
 static void configNtpIfNeeded();
 static void drainGpsForPeriod(unsigned long windowMs);
 static bool parseGgaSentence(char *line);
 static double nmeaCoordToDeg(const char *field, char hemi);
 static void startImpactTelegramCountdown();
 static void cancelTelegramCountdown(const char *reason);
 static void updateTelegramCountdown(unsigned long nowMs);
 static void sendManualEmergencyAlert();
 static void updateRelayAndBuzzer(unsigned long nowMs);
 static void updateButton(unsigned long nowMs);
 static int localHour();
 static bool inNightLightWindow();
 static const char *accidentStateLabel();

 // ── HTML (PROGMEM) ────────────────────────────────────────────────────────
 static const char PAGE_SHELL_START[] PROGMEM = R"raw(
 <!DOCTYPE html><html lang="en"><head>
 <meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
 <title>)raw";

 static const char PAGE_SHELL_MID[] PROGMEM = R"raw(</title>
 <style>
 :root{--bg:#0c0f14;--card:#151b24;--line:#263244;--txt:#e8edf5;--muted:#8b9bb4;--accent:#5eead4;--accent2:#a78bfa;--danger:#fb7185;--ok:#34d399}
 *{box-sizing:border-box}
 body{margin:0;font-family:ui-sans-serif,system-ui,Segoe UI,Roboto,Helvetica,Arial;background:radial-gradient(1200px 800px at 20% -10%,#1a2240 0%,var(--bg) 55%);color:var(--txt);min-height:100vh}
 .wrap{max-width:920px;margin:0 auto;padding:28px 18px 48px}
 .top{display:flex;align-items:center;justify-content:space-between;gap:14px;margin-bottom:22px}
 .badge{font-size:12px;letter-spacing:.12em;text-transform:uppercase;color:var(--muted)}
 h1{font-size:26px;margin:0;letter-spacing:-.02em}
 p.lead{margin:8px 0 0;color:var(--muted);max-width:62ch;line-height:1.5}
 .grid{display:grid;gap:14px}
 @media(min-width:820px){.grid2{grid-template-columns:1fr 1fr}}
 .card{background:linear-gradient(180deg,rgba(255,255,255,.03),rgba(255,255,255,.01));border:1px solid var(--line);border-radius:16px;padding:16px 16px 14px;box-shadow:0 14px 40px rgba(0,0,0,.35)}
 .card h2{margin:0 0 10px;font-size:15px;color:var(--muted);font-weight:600;letter-spacing:.06em;text-transform:uppercase}
 a.btn,button.btn{display:inline-flex;align-items:center;justify-content:center;padding:10px 14px;border-radius:12px;border:1px solid var(--line);background:rgba(255,255,255,.04);color:var(--txt);text-decoration:none;cursor:pointer;font-weight:600}
 a.btn.primary,button.btn.primary{background:linear-gradient(135deg,var(--accent),#2dd4bf);color:#061016;border:0}
 a.btn.secondary,button.btn.secondary{border-color:rgba(94,234,212,.35);color:var(--accent)}
 .row{display:flex;flex-wrap:wrap;gap:10px;margin-top:12px}
 label{display:block;font-size:12px;color:var(--muted);margin:10px 0 6px}
 input{width:100%;padding:11px 12px;border-radius:12px;border:1px solid var(--line);background:#0b1018;color:var(--txt)}
 input:not([type=checkbox]):not([type=radio]){-webkit-user-select:text;user-select:text;touch-action:manipulation}
 textarea{-webkit-user-select:text;user-select:text;touch-action:manipulation}
 small.hint{display:block;margin-top:8px;color:var(--muted);line-height:1.45}
 .status-pill{display:inline-flex;align-items:center;gap:8px;padding:6px 10px;border-radius:999px;border:1px solid var(--line);font-size:12px;color:var(--muted)}
 .dot{width:8px;height:8px;border-radius:999px;background:var(--muted)}
 .dot.ok{background:var(--ok)} .dot.bad{background:var(--danger)}
 table{width:100%;border-collapse:collapse;font-size:14px}
 td{padding:10px 0;border-bottom:1px solid var(--line);vertical-align:top}
 td.k{color:var(--muted);width:42%}
 pre{margin:0;white-space:pre-wrap;font-size:12px;color:var(--muted)}
 </style></head><body><div class="wrap">
 )raw";

 static const char PAGE_SHELL_END[] PROGMEM = R"raw(
 </div></body></html>
 )raw";

 static void sendPageHeader(const char *title) {
   server.sendContent_P(PAGE_SHELL_START);
   server.sendContent(title);
   server.sendContent_P(PAGE_SHELL_MID);
 }

 static void sendPageFooter() { server.sendContent_P(PAGE_SHELL_END); }

 // ── Prefs ─────────────────────────────────────────────────────────────────
 static void loadPrefs() {
   prefs.begin("helmivo", true);
   storedSsid = prefs.getString("ssid", "");
   storedWifiPass = prefs.getString("wpass", "");
   storedChatId = telegramSingleLine(prefs.getString("chat", ""));
   storedBotToken = telegramSingleLine(prefs.getString("bot", ""));
   prefs.end();
   if (storedBotToken.length() == 0) storedBotToken = telegramSingleLine(String(HELMIVO_BOT_TOKEN));
 }

 static void saveWifi(const String &ssid, const String &pw) {
   prefs.begin("helmivo", false);
   prefs.putString("ssid", ssid);
   prefs.putString("wpass", pw);
   prefs.end();
   storedSsid = ssid;
   storedWifiPass = pw;
 }

 static String trimTelegramChat(const String &s) {
   int n = (int)s.length();
   int a = 0;
   while (a < n && isspace((unsigned char)s[a])) a++;
   int b = n - 1;
   while (b >= a && isspace((unsigned char)s[b])) b--;
   return s.substring(a, b + 1);
 }

 static String htmlAttrEscape(const String &s) {
   String o;
   o.reserve(s.length() + 8);
   for (size_t i = 0; i < s.length(); i++) {
     char c = s[i];
     if (c == '&')
       o += "&amp;";
     else if (c == '"')
       o += "&quot;";
     else if (c == '<')
       o += "&lt;";
     else
       o += c;
   }
   return o;
 }

 // Strip breaks/control chars so HTML value="..." cannot corrupt the page (fixes "cannot type" symptom).
 static String telegramSingleLine(const String &s) {
   String t = trimTelegramChat(s);
   String out;
   out.reserve(t.length());
   for (size_t i = 0; i < t.length(); i++) {
     unsigned char c = (unsigned char)t[i];
     if (c < 32) continue;
     out += (char)c;
   }
   return out;
 }

 static void saveTelegram(const String &chat, const String &bot) {
   String ch = telegramSingleLine(chat);
   prefs.begin("helmivo", false);
   
   // Explicitly save or clear the chat ID
   if (ch.length() > 0) {
     prefs.putString("chat", ch);
   } else {
     prefs.remove("chat"); 
   }
   storedChatId = ch;

   // Handle the bot token update
   if (bot.length() > 0) {
     String bt = telegramSingleLine(bot);
     prefs.putString("bot", bt);
     storedBotToken = bt;
   }
   
   prefs.end();
   
   Serial.printf("[HELMIVO] Telegram NVS saved: chat %s, bot %s\n",
                 ch.length() ? "set" : "cleared", bot.length() ? "updated" : "unchanged");
 }

 static const char *telegramPostRedirectTarget(const String &want) {
   if (want == "/") return "/";
   if (want == "/dashboard") return "/dashboard";
   if (want == "/telegram") return "/telegram";
   return "/dashboard";
 }

 static String urlEncode(const String &s) {
   String out;
   out.reserve(s.length() * 2);
   for (size_t i = 0; i < s.length(); i++) {
     char c = s[i];
     if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
         c == '-' || c == '_' || c == '.' || c == '~')
       out += c;
     else if (c == ' ')
       out += '+';
     else {
       char buf[5];
       snprintf(buf, sizeof(buf), "%%%02X", (unsigned char)c);
       out += buf;
     }
   }
   return out;
 }

 static String formatLocalDateTime() {
   struct tm t;
   time_t now = time(nullptr);
   if (now < 100000) return String("(time not synced)");
   localtime_r(&now, &t);
   char b[40];
   strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S %Z", &t);
   return String(b);
 }

 static void configNtpIfNeeded() {
   if (ntpConfigured) return;
   if (WiFi.status() != WL_CONNECTED) return;
   setenv("TZ", "Asia/Manila", 1);
   tzset();
   configTime(0, 0, "pool.ntp.org", "time.nist.gov");
   ntpConfigured = true;
 }

 static bool sendTelegramMessage(const String &text) {
   if (WiFi.status() != WL_CONNECTED) {
     strncpy(lastTelegramStatus, "no_wifi", sizeof(lastTelegramStatus) - 1);
     lastTelegramStatus[sizeof(lastTelegramStatus) - 1] = '\0';
     return false;
   }
   if (storedChatId.length() == 0) {
     strncpy(lastTelegramStatus, "no_chat_id", sizeof(lastTelegramStatus) - 1);
     lastTelegramStatus[sizeof(lastTelegramStatus) - 1] = '\0';
     return false;
   }
   if (storedBotToken.length() == 0) {
     strncpy(lastTelegramStatus, "no_bot_token", sizeof(lastTelegramStatus) - 1);
     lastTelegramStatus[sizeof(lastTelegramStatus) - 1] = '\0';
     return false;
   }

   WiFiClientSecure client;
   client.setInsecure();
   HTTPClient https;
   String url = "https://api.telegram.org/bot" + storedBotToken + "/sendMessage";
   if (!https.begin(client, url)) {
     strncpy(lastTelegramStatus, "begin_fail", sizeof(lastTelegramStatus) - 1);
     lastTelegramStatus[sizeof(lastTelegramStatus) - 1] = '\0';
     return false;
   }
   https.addHeader("Content-Type", "application/x-www-form-urlencoded");
   String body =
       String("chat_id=") + urlEncode(storedChatId) + "&text=" + urlEncode(text);
   int code = https.POST(body);
   snprintf(lastTelegramStatus, sizeof(lastTelegramStatus), "http_%d", code);
   https.end();
   lastTelegramAttemptMs = millis();
   if (code >= 200 && code < 300) {
     int n = (int)text.length();
     if (n >= (int)sizeof(lastTelegramSummary) - 1) n = (int)sizeof(lastTelegramSummary) - 1;
     memcpy(lastTelegramSummary, text.c_str(), (size_t)n);
     lastTelegramSummary[n] = '\0';
     return true;
   }
   return false;
 }

 static double nmeaCoordToDeg(const char *field, char hemi) {
   if (!field || field[0] == '\0') return NAN;
   float v = (float)atof(field);
   int deg = (int)(v / 100);
   float minutes = v - (float)deg * 100.0f;
   double d = (double)deg + (double)minutes / 60.0;
   if (hemi == 'S' || hemi == 'W') d = -d;
   return d;
 }

 static bool parseGgaSentence(char *line) {
   if (strncmp(line, "$GPGGA", 6) != 0 && strncmp(line, "$GNGGA", 6) != 0) return false;
   const int MAXF = 16;
   char *fields[MAXF];
   int flen = 0;
   fields[flen++] = line;
   for (char *p = line; *p && flen < MAXF; p++) {
     if (*p == ',') {
       *p = '\0';
       fields[flen++] = p + 1;
     }
   }
   if (flen < 10) return false;
   if (fields[1]) strncpy(gps.lastRawTime, fields[1], sizeof(gps.lastRawTime) - 1);
   int qual = atoi(fields[6] && fields[6][0] ? fields[6] : "0");
   gps.satellites = (uint8_t)atoi(fields[7] && fields[7][0] ? fields[7] : "0");
   if (qual == 0 || !fields[2] || fields[2][0] == '\0' || !fields[4] || fields[4][0] == '\0') {
     gps.fix = false;
     return true;
   }
   char ns = fields[3] && fields[3][0] ? fields[3][0] : 'N';
   char ew = fields[5] && fields[5][0] ? fields[5][0] : 'E';
   gps.lat = nmeaCoordToDeg(fields[2], ns);
   gps.lon = nmeaCoordToDeg(fields[4], ew);
   gps.fix = true;
   gps.lastFixMs = millis();
   return true;
 }

 static void drainGpsForPeriod(unsigned long windowMs) {
   unsigned long t0 = millis();
   static char lineBuf[120];
   static size_t li = 0;
   while ((long)(millis() - t0) < (long)windowMs) {
     while (gpsSerial.available()) {
       char c = (char)gpsSerial.read();
       if (c == '\n' || c == '\r') {
         if (li > 0) {
           lineBuf[li] = '\0';
           parseGgaSentence(lineBuf);
           li = 0;
         }
       } else if (li < sizeof(lineBuf) - 1) {
         lineBuf[li++] = (char)c;
       }
     }
     delay(1);
   }
 }

 static void startImpactTelegramCountdown() {
   if (tgActiveKind != TG_NONE) return;
   tgActiveKind = TG_IMPACT;
   tgCountdownStartMs = millis();
   tgBuzzerPhase = false;
   relayEmergencyMode = true;
   strncpy(lastTelegramStatus, "countdown_armed", sizeof(lastTelegramStatus) - 1);
   lastTelegramStatus[sizeof(lastTelegramStatus) - 1] = '\0';
   Serial.println(F("[HELMIVO] Impact -> Telegram countdown 15s (cancel: web or serial 'c')"));
 }

 static void cancelTelegramCountdown(const char *reason) {
   bool hadTg = (tgActiveKind != TG_NONE);
   tgActiveKind = TG_NONE;
   tgBuzzerPhase = false;
   digitalWrite(PIN_BUZZER, LOW);
   if (manualRelayUntilMs == 0) relayEmergencyMode = false;
   if (hadTg) Serial.printf("[HELMIVO] Countdown cancelled: %s\n", reason);
 }

 static void updateTelegramCountdown(unsigned long nowMs) {
   if (tgActiveKind != TG_IMPACT) return;
   unsigned long elapsed = nowMs - tgCountdownStartMs;
   static int lastPrintedSec = -1;
   int sec = (int)(elapsed / 1000UL);
   if (sec != lastPrintedSec) {
     lastPrintedSec = sec;
     int show = sec;
     if (show > 15) show = 15;
     Serial.printf("[HELMIVO] Telegram countdown: %d / 15 s\n", show);
   }
   tgBuzzerPhase = (elapsed >= TELEGRAM_BUZZER_START_MS);

   if (elapsed >= TELEGRAM_COUNTDOWN_TOTAL_MS) {
     lastPrintedSec = -1;
     String maps =
       String("https://maps.google.com/?q=") + String(gps.lat, 6) + "," + String(gps.lon, 6);
     String msg;
     msg.reserve(420);
     msg += F("Helmivo IMPACT ALERT\n\n");
     msg += F("Impact detected. This message is sent 15 seconds after detection.\n\n");
     msg += F("Local time: ");
     msg += formatLocalDateTime();
     msg += F("\nGPS fix: ");
     msg += gps.fix ? F("yes") : F("no");
     msg += F("\nLat: ");
     msg += String(gps.lat, 6);
     msg += F("\nLon: ");
     msg += String(gps.lon, 6);
     msg += F("\nMap: ");
     msg += maps;
     msg += F("\n\nPeak G (session): ");
     msg += String(peakGSession, 2);
     sendTelegramMessage(msg);
     tgActiveKind = TG_NONE;
     tgBuzzerPhase = false;
     digitalWrite(PIN_BUZZER, LOW);
     relayEmergencyMode = false;
     if (inNightLightWindow()) {
       digitalWrite(PIN_RELAY, LOW);
       Serial.println(F("[HELMIVO] Night window: relay kept ON after alert"));
     }
     strncpy(lastTelegramStatus, "impact_sent", sizeof(lastTelegramStatus) - 1);
     lastTelegramStatus[sizeof(lastTelegramStatus) - 1] = '\0';
     Serial.println(F("[HELMIVO] Impact Telegram sent"));
   }
 }

 static void sendManualEmergencyAlert() {
   if (WiFi.status() != WL_CONNECTED) {
     strncpy(lastTelegramStatus, "manual_no_wifi", sizeof(lastTelegramStatus) - 1);
     lastTelegramStatus[sizeof(lastTelegramStatus) - 1] = '\0';
     Serial.println(F("[HELMIVO] Manual alert: WiFi not connected"));
     return;
   }
   String maps =
     String("https://maps.google.com/?q=") + String(gps.lat, 6) + "," + String(gps.lon, 6);
   String msg;
   msg.reserve(440);
   msg += F("[MANUAL EMERGENCY] Helmivo rider request\n\n");
   msg += F("Emergency button held 3 seconds.\n\n");
   msg += F("Timestamp: ");
   msg += formatLocalDateTime();
   msg += F("\nGPS fix: ");
   msg += gps.fix ? F("yes") : F("no");
   msg += F("\nLat: ");
   msg += String(gps.lat, 6);
   msg += F("\nLon: ");
   msg += String(gps.lon, 6);
   msg += F("\nMap: ");
   msg += maps;
   msg += F("\n\nFormat: MANUAL / rider_initiated / Helmivo v1");
   sendTelegramMessage(msg);
   manualRelayUntilMs = millis() + 4000;
   relayEmergencyMode = true;
   strncpy(lastTelegramStatus, "manual_sent", sizeof(lastTelegramStatus) - 1);
   lastTelegramStatus[sizeof(lastTelegramStatus) - 1] = '\0';
   Serial.println(F("[HELMIVO] Manual emergency Telegram dispatched"));
 }

 static int localHour() {
   time_t tt = time(nullptr);
   if (tt < 100000) return 12;
   struct tm t;
   localtime_r(&tt, &t);
   return t.tm_hour;
 }

 static bool inNightLightWindow() {
   int h = localHour();
   return (h >= 18 || h < 5);
 }

 static void updateRelayAndBuzzer(unsigned long nowMs) {
   if (testBuzzerUntilMs != 0) {
     if (nowMs < testBuzzerUntilMs) {
       digitalWrite(PIN_BUZZER, HIGH);
     } else {
       testBuzzerUntilMs = 0;
       digitalWrite(PIN_BUZZER, LOW);
     }
   } else {
     static unsigned long buzzNext = 0;
     static bool buzzOn = false;
     if (tgBuzzerPhase) {
       if ((long)(nowMs - buzzNext) >= 0) {
         buzzOn = !buzzOn;
         digitalWrite(PIN_BUZZER, buzzOn ? HIGH : LOW);
         buzzNext = nowMs + 200;
       }
     } else {
       digitalWrite(PIN_BUZZER, LOW);
     }
   }

   if (testFlashlightUntilMs != 0) {
     if (nowMs < testFlashlightUntilMs) {
       digitalWrite(PIN_RELAY, LOW);
       return;
     } else {
       testFlashlightUntilMs = 0;
     }
   }

   bool manualBlink = (manualRelayUntilMs != 0 && nowMs < manualRelayUntilMs);
   if (manualRelayUntilMs != 0 && nowMs >= manualRelayUntilMs) {
     manualRelayUntilMs = 0;
     relayEmergencyMode = false;
   }

   bool emergency = relayEmergencyMode || manualBlink;
   if (emergency) {
     if (nowMs - relayLastToggleMs >= EMERGENCY_BLINK_MS) {
       relayBlinkState = !relayBlinkState;
       relayLastToggleMs = nowMs;
       digitalWrite(PIN_RELAY, relayBlinkState ? HIGH : LOW);
     }
   } else {
     relayLastToggleMs = nowMs;
     relayBlinkState = false;
     if (inNightLightWindow()) digitalWrite(PIN_RELAY, LOW);
     else digitalWrite(PIN_RELAY, HIGH);
   }
 }

 static void updateButton(unsigned long nowMs) {
   bool down = digitalRead(PIN_BUTTON) == LOW;
   if (down) {
     if (btnDownSince == 0) btnDownSince = nowMs;
     if (!btnManualLatched && (nowMs - btnDownSince >= BUTTON_HOLD_MANUAL_MS)) {
       btnManualLatched = true;
       cancelTelegramCountdown("manual_override");
       sendManualEmergencyAlert();
     }
   } else {
     btnDownSince = 0;
     btnManualLatched = false;
   }
 }

 static void mpuWrite(uint8_t reg, uint8_t value) {
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(reg);
   Wire.write(value);
   Wire.endTransmission();
 }

 static void mpuRead(RawData *out) {
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(REG_ACCEL_XOUT_H);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU_ADDR, 14, true);
   RawData &d = *out;
   d.ax = (Wire.read() << 8) | Wire.read();
   d.ay = (Wire.read() << 8) | Wire.read();
   d.az = (Wire.read() << 8) | Wire.read();
   Wire.read();
   Wire.read();
   d.gx = (Wire.read() << 8) | Wire.read();
   d.gy = (Wire.read() << 8) | Wire.read();
   d.gz = (Wire.read() << 8) | Wire.read();
 }

 static void calibrateGyro() {
   Serial.println(F("[MPU] Calibrating gyro — keep still."));
   long sumGX = 0, sumGY = 0, sumGZ = 0;
   for (int i = 0; i < CALIB_SAMPLES; i++) {
     RawData d;
     mpuRead(&d);
     sumGX += d.gx;
     sumGY += d.gy;
     sumGZ += d.gz;
     delay(5);
   }
   gyroBiasX = sumGX / (float)CALIB_SAMPLES;
   gyroBiasY = sumGY / (float)CALIB_SAMPLES;
   gyroBiasZ = sumGZ / (float)CALIB_SAMPLES;
 }

 static float tiltFromVertical(float ax, float ay, float az, float totalG) {
   if (totalG < 0.15f) return 0.0f;
   float c = az / totalG;
   if (c > 1.0f) c = 1.0f;
   if (c < -1.0f) c = -1.0f;
   return acosf(c) * RAD_TO_DEG;
 }

 static float gyroMagnitudeDegPerS(const RawData *d) {
   float gxS = (d->gx - gyroBiasX) / GYRO_SCALE;
   float gyS = (d->gy - gyroBiasY) / GYRO_SCALE;
   float gzS = (d->gz - gyroBiasZ) / GYRO_SCALE;
   return sqrtf(gxS * gxS + gyS * gyS + gzS * gzS);
 }

 static bool orientationDriverDown(float tiltDeg, float axg, float ayg, float azg) {
   float hxy = sqrtf(axg * axg + ayg * ayg);
   bool gh = (fabsf(azg) < DOWN_Z_MAX_G && hxy > DOWN_XY_MIN_G);
   return (tiltDeg > TILT_ACCIDENT_DEG) || gh;
 }

 static bool uprightRecovered(float tiltDeg, float totalG, float azg) {
   return (tiltDeg < RECOVERY_TILT_DEG) &&
          (totalG >= RECOVERY_G_BAND_LO && totalG <= RECOVERY_G_BAND_HI) &&
          (fabsf(azg) > RECOVERY_AZ_MIN_G);
 }

 static bool motionLooksActive(float gyroMagDps) {
   return gyroMagDps > MOTION_GYRO_ACTIVE_DPS;
 }

 static void resetAccidentFlags() {
   flagImpactDetected = false;
   flagDriverDown = false;
   flagDriverInactive = false;
   flagSoftFallPath = false;
   eventHadFreefall = false;
   eventViolentSpin = false;
 }

 static uint8_t computeAccidentSeverity(float peakGVal, float peakTiltDeg, bool freefallFlag,
                                        bool violentSpinFlag, bool softFallFlag) {
   float s = 30.0f * (peakGVal / IMPACT_SEVERE) + 30.0f * (peakTiltDeg / 90.0f) +
             (freefallFlag ? 20.0f : 0.0f) + (violentSpinFlag ? 20.0f : 0.0f) +
             (softFallFlag ? 10.0f : 0.0f);
   if (s < 0.0f) s = 0.0f;
   if (s > 100.0f) s = 100.0f;
   return (uint8_t)(s + 0.5f);
 }

 static const char *accidentStateLabel() {
   switch (accidentState) {
     case ST_IMPACT_SETTLE: return "IMPACT_SETTLE";
     case ST_INACTIVITY:    return "INACTIVITY";
     case ST_CANCEL_WINDOW: return "ACCIDENT_CANCEL";
     case ST_LOCKOUT:       return "ALERT_LOCKOUT";
     default:               return "NORMAL";
   }
 }

 static String jsonEscape(const String &s) {
   String o;
   o.reserve(s.length() + 8);
   for (size_t i = 0; i < s.length(); i++) {
     char c = s[i];
     if (c == '"' || c == '\\') o += '\\';
     o += c;
   }
   return o;
 }

 // ═══════════════════════════════════════════════════════════════════════════
 //  SERIAL LOG API   GET /api/serial-log[?since=N]
 //
 //  Returns JSON:
 //  {
 //    "next_seq": 42,          // pass as ?since= in next poll
 //    "lines": [
 //      {"seq":38,"ts":12345,"text":"[TELEM] G=1.00 ..."},
 //      ...
 //    ]
 //  }
 //
 //  The client always increments its cursor by reading next_seq.
 //  If the ring wraps, the client will get up to SERIAL_LOG_LINES lines.
 // ═══════════════════════════════════════════════════════════════════════════
 static void handleSerialLog() {
   uint32_t since = 0;
   if (server.hasArg("since")) since = (uint32_t)server.arg("since").toInt();

   // collect matching entries in chronological order
   // ring head points to next-write slot (oldest useful = head if full)
   String out;
   out.reserve(2048);
   out += F("{\"next_seq\":");
   out += String(s_logSeq);
   out += F(",\"lines\":[");

   bool first = true;
   // iterate from oldest to newest
   for (int i = 0; i < SERIAL_LOG_LINES; i++) {
     int idx = (s_logHead + i) % SERIAL_LOG_LINES;
     const SerialLogEntry &e = s_logBuf[idx];
     if (e.seq == 0 && e.text[0] == '\0') continue;  // never written
     if (e.seq < since) continue;
     if (!first) out += ',';
     first = false;
     out += F("{\"seq\":");
     out += String(e.seq);
     out += F(",\"ts\":");
     out += String(e.tsMs);
     out += F(",\"text\":\"");
     // JSON-escape text inline
     for (const char *p = e.text; *p; p++) {
       char c = *p;
       if (c == '"')       out += F("\\\"");
       else if (c == '\\') out += F("\\\\");
       else if (c == '\n') out += F("\\n");
       else if (c == '\r') {}
       else                out += c;
     }
     out += F("\"}");
   }
   out += F("]}");
   server.send(200, "application/json", out);
 }

 // ═══════════════════════════════════════════════════════════════════════════
 //  TESTER — API handlers (unchanged)
 // ═══════════════════════════════════════════════════════════════════════════

 static void handleTestAccident() {
   if (accidentState == ST_LOCKOUT) {
     server.send(409, "application/json",
                 "{\"ok\":false,\"reason\":\"lockout_active\"}");
     return;
   }
   accidentState        = ST_CANCEL_WINDOW;
   stateEnteredMs       = millis();
   cancelLastPrintedSec = -1;
   flagImpactDetected   = true;
   flagDriverDown       = true;
   flagDriverInactive   = true;
   flagSoftFallPath     = false;
   eventHadFreefall     = false;
   eventViolentSpin     = false;
   eventPeakG           = 5.5f;
   eventPeakTilt        = 75.0f;
   eventSeverity        = computeAccidentSeverity(eventPeakG, eventPeakTilt, false, false, false);
   startImpactTelegramCountdown();
   Serial.println(F("[TESTER] Implement Accident: full 15s Telegram countdown started"));
   server.send(200, "application/json",
               "{\"ok\":true,\"state\":\"ACCIDENT_CANCEL\","
               "\"countdown_ms\":15000,"
               "\"severity\":" + String(eventSeverity) + "}");
 }

 static void handleTestFlashlight() {
   testFlashlightUntilMs = millis() + TEST_FLASHLIGHT_MS;
   Serial.printf("[TESTER] Flashlight ON for %lu ms\n", TEST_FLASHLIGHT_MS);
   server.send(200, "application/json",
               "{\"ok\":true,\"duration_ms\":" + String(TEST_FLASHLIGHT_MS) + "}");
 }

 static void handleTestBuzzer() {
   if (tgBuzzerPhase) {
     server.send(409, "application/json",
                 "{\"ok\":false,\"reason\":\"emergency_buzz_active\"}");
     return;
   }
   testBuzzerUntilMs = millis() + TEST_BUZZER_MS;
   Serial.printf("[TESTER] Buzzer ON for %lu ms\n", TEST_BUZZER_MS);
   server.send(200, "application/json",
               "{\"ok\":true,\"duration_ms\":" + String(TEST_BUZZER_MS) + "}");
 }

 static void handleTestPosture() {
   String j = "{";
   j += "\"g\":"       + String(g_imuTotalG, 4);
   j += ",\"pitch\":"  + String(g_imuPitch,  2);
   j += ",\"roll\":"   + String(g_imuRoll,   2);
   j += ",\"tilt\":"   + String(g_imuTiltDeg, 2);
   j += ",\"gyro_dps\":" + String(g_imuGyroMag, 2);
   j += ",\"peak_g\":" + String(peakGSession, 4);
   j += ",\"acc_state\":\"" + jsonEscape(String(accidentStateLabel())) + "\"";
   j += "}";
   server.send(200, "application/json", j);
 }

 static void handleTestGps() {
   unsigned long age = gps.lastFixMs ? (millis() - gps.lastFixMs) : 999999UL;
   String maps = gps.fix
     ? ("https://maps.google.com/?q=" + String(gps.lat, 6) + "," + String(gps.lon, 6))
     : "";
   String j = "{";
   j += "\"fix\":"  + String(gps.fix ? "true" : "false");
   j += ",\"lat\":" + String(gps.lat, 6);
   j += ",\"lon\":" + String(gps.lon, 6);
   j += ",\"sats\":" + String((int)gps.satellites);
   j += ",\"age_ms\":" + String(age);
   j += ",\"raw_time\":\"" + jsonEscape(String(gps.lastRawTime)) + "\"";
   j += ",\"maps_url\":\"" + jsonEscape(maps) + "\"";
   j += "}";
   server.send(200, "application/json", j);
 }

 // ═══════════════════════════════════════════════════════════════════════════
 //  TESTER page (unchanged, omitted for brevity — keep your existing code)
 // ═══════════════════════════════════════════════════════════════════════════
 static void handleTesterPage() {
   server.setContentLength(CONTENT_LENGTH_UNKNOWN);
   server.send(200, "text/html", "");
   server.sendContent(F(R"html(
 <!DOCTYPE html><html lang="en"><head>
 <meta charset="utf-8"/>
 <meta name="viewport" content="width=device-width,initial-scale=1"/>
 <title>Helmivo Tester</title>
 <link rel="preconnect" href="https://fonts.googleapis.com"/>
 <link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Barlow:wght@400;600;700&display=swap" rel="stylesheet"/>
 <style>
 :root{--bg:#07090d;--surface:#0e1219;--card:#121820;--border:#1e2d3d;--txt:#dde6f0;--muted:#5a7390;--accent:#00e5ff;--warn:#ffb300;--danger:#ff4060;--ok:#00e676;--purple:#b388ff;--font-mono:'Share Tech Mono',monospace;--font-body:'Barlow',sans-serif;}
 *{box-sizing:border-box;margin:0;padding:0}
 body{background:var(--bg);color:var(--txt);font-family:var(--font-body);min-height:100vh;background-image:radial-gradient(ellipse 80% 50% at 10% 0%,#0a1a2e 0%,var(--bg) 60%)}
 .shell{max-width:980px;margin:0 auto;padding:24px 16px 60px}
 .topbar{display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid var(--border);padding-bottom:16px;margin-bottom:24px}
 .topbar-left{display:flex;align-items:center;gap:14px}
 .helmet-icon{width:38px;height:38px;background:linear-gradient(135deg,#00e5ff22,#00e5ff11);border:1px solid #00e5ff44;border-radius:10px;display:flex;align-items:center;justify-content:center;font-size:20px}
 .title-block .label{font-size:11px;letter-spacing:.15em;text-transform:uppercase;color:var(--muted);margin-bottom:2px}
 .title-block h1{font-size:22px;font-weight:700;letter-spacing:-.01em}
 .nav-links{display:flex;gap:8px}
 .nav-btn{padding:7px 14px;border-radius:8px;border:1px solid var(--border);background:transparent;color:var(--muted);font-family:var(--font-body);font-size:13px;cursor:pointer;text-decoration:none;transition:border-color .2s,color .2s}
 .nav-btn:hover{border-color:var(--accent);color:var(--accent)}
 .sec-head{display:flex;align-items:center;gap:8px;margin-bottom:14px}
 .sec-head .dot{width:6px;height:6px;border-radius:50%;background:var(--accent)}
 .sec-head h2{font-size:11px;letter-spacing:.14em;text-transform:uppercase;color:var(--muted);font-weight:600}
 .trigger-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px;margin-bottom:24px}
 .trig-card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:18px 16px 16px;cursor:pointer;transition:transform .15s,border-color .2s,box-shadow .2s;display:flex;flex-direction:column;gap:10px;position:relative;overflow:hidden;user-select:none}
 .trig-card:hover{transform:translateY(-2px);box-shadow:0 8px 32px rgba(0,0,0,.5)}
 .trig-card:active{transform:scale(.97)}
 .trig-card.danger{border-color:#ff406033}.trig-card.danger:hover{border-color:var(--danger);box-shadow:0 0 24px #ff406033}
 .trig-card.cyan{border-color:#00e5ff22}.trig-card.cyan:hover{border-color:var(--accent);box-shadow:0 0 24px #00e5ff22}
 .trig-card.amber{border-color:#ffb30022}.trig-card.amber:hover{border-color:var(--warn);box-shadow:0 0 24px #ffb30022}
 .trig-card.cancel-card{border-color:#ffffff11}.trig-card.cancel-card:hover{border-color:#ffffff44}
 .trig-card .tc-icon{font-size:28px;line-height:1}
 .trig-card .tc-label{font-size:15px;font-weight:700;line-height:1.2}
 .trig-card.danger .tc-label{color:var(--danger)}.trig-card.cyan .tc-label{color:var(--accent)}.trig-card.amber .tc-label{color:var(--warn)}.trig-card.cancel-card .tc-label{color:var(--muted)}
 .trig-card .tc-desc{font-size:12px;color:var(--muted);line-height:1.5}
 .trig-card .tc-shine{position:absolute;top:0;left:-60%;width:50%;height:100%;background:linear-gradient(90deg,transparent,rgba(255,255,255,.04),transparent);transform:skewX(-20deg);transition:left .4s}
 .trig-card:hover .tc-shine{left:130%}
 #cd-wrap{display:none;margin-bottom:24px;background:var(--card);border:1px solid var(--border);border-radius:14px;padding:20px;flex-direction:column;align-items:center;gap:12px}
 #cd-wrap.active{display:flex;border-color:#ff406066;box-shadow:0 0 40px #ff406022}
 .ring-row{display:flex;align-items:center;gap:20px}
 svg#ring{transform:rotate(-90deg)}
 #ring-track{fill:none;stroke:#1e2d3d;stroke-width:6}
 #ring-fill{fill:none;stroke:var(--danger);stroke-width:6;stroke-linecap:round;transition:stroke-dashoffset .9s linear;stroke-dasharray:220;stroke-dashoffset:0}
 #cd-sec{font-family:var(--font-mono);font-size:36px;color:var(--danger);line-height:1}
 #cd-label{font-size:12px;color:var(--muted);letter-spacing:.1em;text-transform:uppercase}
 #cd-status{font-size:13px;color:var(--muted)}
 #cd-status b{color:var(--warn)}
 #toast{position:fixed;bottom:24px;right:20px;max-width:340px;padding:12px 16px;border-radius:10px;border:1px solid var(--border);background:#0d1520;font-size:13px;color:var(--txt);opacity:0;transform:translateY(10px);transition:opacity .25s,transform .25s;pointer-events:none;z-index:999;font-family:var(--font-mono)}
 #toast.show{opacity:1;transform:translateY(0)}
 .data-grid{display:grid;gap:14px;margin-bottom:24px}
 @media(min-width:640px){.data-grid{grid-template-columns:1fr 1fr}}
 .data-card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:16px}
 .data-card .card-head{display:flex;align-items:center;gap:8px;margin-bottom:14px;padding-bottom:10px;border-bottom:1px solid var(--border)}
 .data-card .card-head .icon{font-size:16px}
 .data-card .card-head .name{font-size:12px;letter-spacing:.12em;text-transform:uppercase;color:var(--muted);font-weight:600}
 .data-card .card-head .live-dot{margin-left:auto;width:7px;height:7px;border-radius:50%;background:var(--ok);animation:pulse 2s infinite}
 @keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
 .kv{display:flex;justify-content:space-between;align-items:baseline;padding:7px 0;border-bottom:1px solid #ffffff08;gap:12px}
 .kv:last-child{border-bottom:none}
 .kv .k{font-size:12px;color:var(--muted);white-space:nowrap}
 .kv .v{font-family:var(--font-mono);font-size:13px;color:var(--txt);text-align:right}
 .v.ok{color:var(--ok)}.v.bad{color:var(--danger)}.v.warn{color:var(--warn)}
 .raw-card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:16px}
 .raw-card details summary{cursor:pointer;font-size:11px;letter-spacing:.14em;text-transform:uppercase;color:var(--muted);font-weight:600;padding:4px 0}
 pre#raw{margin-top:12px;font-family:var(--font-mono);font-size:11px;color:var(--muted);white-space:pre-wrap;max-height:260px;overflow-y:auto}
 </style>
 </head>
 <body>
 <div class="shell">
 <div class="topbar">
   <div class="topbar-left">
     <div class="helmet-icon">&#128081;</div>
     <div class="title-block">
       <div class="label">Helmivo &mdash; Dev / QA</div>
       <h1>Hardware Tester</h1>
     </div>
   </div>
   <div class="nav-links">
     <a class="nav-btn" href="/dashboard">Dashboard</a>
     <a class="nav-btn" href="/">Home</a>
   </div>
 </div>
 <div class="sec-head"><div class="dot"></div><h2>Test Triggers</h2></div>
 <div class="trigger-grid">
   <div class="trig-card danger" id="btn-accident"><div class="tc-shine"></div><div class="tc-icon">&#9888;&#65039;</div><div class="tc-label">Implement Accident</div><div class="tc-desc">Fires full 15-second Telegram countdown with buzzer &amp; relay blink.</div></div>
   <div class="trig-card cyan" id="btn-flashlight"><div class="tc-shine"></div><div class="tc-icon">&#128294;</div><div class="tc-label">Flashlight (5 s)</div><div class="tc-desc">Forces relay ON (G14 LOW) for 5 seconds.</div></div>
   <div class="trig-card amber" id="btn-buzzer"><div class="tc-shine"></div><div class="tc-icon">&#128276;</div><div class="tc-label">Buzzer (3 s)</div><div class="tc-desc">Sounds G25 buzzer for 3 seconds.</div></div>
   <div class="trig-card cancel-card" id="btn-cancel"><div class="tc-shine"></div><div class="tc-icon">&#10006;</div><div class="tc-label">Cancel Alert</div><div class="tc-desc">Cancels any active Telegram countdown.</div></div>
 </div>
 <div id="cd-wrap">
   <div class="ring-row">
     <svg id="ring" width="80" height="80" viewBox="0 0 80 80"><circle id="ring-track" cx="40" cy="40" r="35"/><circle id="ring-fill" cx="40" cy="40" r="35"/></svg>
     <div><div id="cd-sec">15</div><div id="cd-label">seconds remaining</div></div>
   </div>
   <div id="cd-status">Countdown active &mdash; Telegram sends at <b>T=0</b></div>
 </div>
 <div class="sec-head"><div class="dot" style="background:var(--ok)"></div><h2>Live Sensor Readings</h2></div>
 <div class="data-grid">
   <div class="data-card">
     <div class="card-head"><span class="icon">&#129668;</span><span class="name">MPU6050 &mdash; Posture</span><span class="live-dot"></span></div>
     <div class="kv"><span class="k">Total G</span><span class="v" id="p-g">—</span></div>
     <div class="kv"><span class="k">Pitch</span><span class="v" id="p-pitch">—</span></div>
     <div class="kv"><span class="k">Roll</span><span class="v" id="p-roll">—</span></div>
     <div class="kv"><span class="k">Tilt from vertical</span><span class="v" id="p-tilt">—</span></div>
     <div class="kv"><span class="k">Gyro magnitude</span><span class="v" id="p-gyro">—</span></div>
     <div class="kv"><span class="k">Peak G (session)</span><span class="v" id="p-peak">—</span></div>
     <div class="kv"><span class="k">Accident state</span><span class="v" id="p-acc">—</span></div>
   </div>
   <div class="data-card">
     <div class="card-head"><span class="icon">&#128225;</span><span class="name">u-blox &mdash; GPS</span><span class="live-dot"></span></div>
     <div class="kv"><span class="k">Fix</span><span class="v" id="g-fix">—</span></div>
     <div class="kv"><span class="k">Latitude</span><span class="v" id="g-lat">—</span></div>
     <div class="kv"><span class="k">Longitude</span><span class="v" id="g-lon">—</span></div>
     <div class="kv"><span class="k">Satellites</span><span class="v" id="g-sats">—</span></div>
     <div class="kv"><span class="k">Fix age</span><span class="v" id="g-age">—</span></div>
     <div class="kv"><span class="k">NMEA time</span><span class="v" id="g-time">—</span></div>
     <div class="kv"><span class="k">Google Maps</span><span class="v" id="g-map">—</span></div>
   </div>
 </div>
 <div class="raw-card"><details><summary>&#9654; Raw JSON dump</summary><pre id="raw"></pre></details></div>
 </div>
 <div id="toast"></div>
 <script>
 let toastTimer;
 function showToast(txt,ok){const el=document.getElementById('toast');el.textContent=txt;el.style.borderColor=ok?'var(--ok)':'var(--danger)';el.style.color=ok?'var(--ok)':'var(--danger)';el.classList.add('show');clearTimeout(toastTimer);toastTimer=setTimeout(()=>el.classList.remove('show'),3500);}
 async function post(url){try{const r=await fetch(url,{method:'POST'});const j=await r.json();showToast((j.ok?'✓ ':'✗ ')+url.split('/').pop()+' — '+JSON.stringify(j),j.ok);return j;}catch(e){showToast('✗ '+String(e),false);}}
 document.getElementById('btn-accident').onclick=()=>post('/api/test/accident');
 document.getElementById('btn-flashlight').onclick=()=>post('/api/test/flashlight');
 document.getElementById('btn-buzzer').onclick=()=>post('/api/test/buzzer');
 document.getElementById('btn-cancel').onclick=()=>post('/api/cancel-alert');
 const CIRC=2*Math.PI*35;
 document.getElementById('ring-fill').style.strokeDasharray=CIRC;
 function updateRing(secLeft,total){const frac=Math.max(0,secLeft/total);document.getElementById('ring-fill').style.strokeDashoffset=CIRC*(1-frac);document.getElementById('cd-sec').textContent=Math.ceil(secLeft);const buzz=secLeft<=4;document.getElementById('ring-fill').style.stroke=buzz?'var(--warn)':'var(--danger)';document.getElementById('cd-status').innerHTML=secLeft>4?'Countdown active &mdash; Telegram sends at <b>T=0</b>':'<b style="color:var(--warn)">&#128276; Buzzer active &mdash; sending imminent</b>';}
 function ageLabel(ms){if(ms>=999999)return 'never';if(ms<2000)return ms+' ms';return(ms/1000).toFixed(1)+' s ago';}
 async function poll(){try{const[rp,rg,rs]=await Promise.all([fetch('/api/test/posture').then(r=>r.json()),fetch('/api/test/gps').then(r=>r.json()),fetch('/api/status').then(r=>r.json())]);
 document.getElementById('p-g').textContent=rp.g+' g';document.getElementById('p-pitch').textContent=rp.pitch+'°';document.getElementById('p-roll').textContent=rp.roll+'°';document.getElementById('p-tilt').textContent=rp.tilt+'°';document.getElementById('p-gyro').textContent=rp.gyro_dps+' dps';document.getElementById('p-peak').textContent=rp.peak_g+' g';const accEl=document.getElementById('p-acc');accEl.textContent=rp.acc_state;accEl.className='v '+(rp.acc_state==='NORMAL'?'ok':'bad');
 const fixEl=document.getElementById('g-fix');fixEl.textContent=rg.fix?'YES':'NO (searching)';fixEl.className='v '+(rg.fix?'ok':'bad');document.getElementById('g-lat').textContent=rg.lat.toFixed(6);document.getElementById('g-lon').textContent=rg.lon.toFixed(6);document.getElementById('g-sats').textContent=rg.sats+' sats';document.getElementById('g-age').textContent=ageLabel(rg.age_ms);document.getElementById('g-time').textContent=rg.raw_time||'—';const mapEl=document.getElementById('g-map');if(rg.fix&&rg.maps_url){mapEl.innerHTML='<a href="'+rg.maps_url+'" target="_blank" style="color:var(--accent);text-decoration:none">&#127757; Open Maps</a>';}else{mapEl.innerHTML='<span style="color:var(--muted)">No fix yet</span>';}
 const cdSec=rs.alert?rs.alert.impact_countdown_sec:0;const cdActive=cdSec>0;document.getElementById('cd-wrap').className=cdActive?'active':'';if(cdActive)updateRing(cdSec,15);
 document.getElementById('raw').textContent=JSON.stringify({posture:rp,gps:rg,alert:rs.alert,lights:rs.lights},null,2);}catch(e){document.getElementById('raw').textContent='// error: '+e;}}
 setInterval(poll,800);poll();
 </script></body></html>
 )html"));
   server.sendContent("");
 }

 // ═══════════════════════════════════════════════════════════════════════════
 //  Existing page handlers
 // ═══════════════════════════════════════════════════════════════════════════

 static void handleIndex() {
   bool sta = (WiFi.status() == WL_CONNECTED);
   server.setContentLength(CONTENT_LENGTH_UNKNOWN);
   server.send(200, "text/html", "");
   sendPageHeader("Helmivo");
   server.sendContent(F("<div class=\"top\"><div><div class=\"badge\">Helmivo</div><h1>Welcome</h1>"));
   server.sendContent(F("<p class=\"lead\">Configure Wi-Fi and Telegram, then open the live dashboard.</p></div>"));
   server.sendContent(F("<div class=\"status-pill\"><span class=\"dot "));
   server.sendContent(sta ? F("ok") : F("bad"));
   server.sendContent(F("\"></span>STA: "));
   server.sendContent(sta ? WiFi.SSID() : String("disconnected"));
   server.sendContent(F("</div></div><div class=\"grid grid2\">"));
   server.sendContent(F("<div class=\"card\"><h2>Quick links</h2>"));
   if (sta && storedSsid.length() > 0) {
     server.sendContent(F("<p style=\"margin:0 0 10px;color:var(--muted)\">Connected — open the dashboard for live data.</p>"));
     server.sendContent(F("<div class=\"row\"><a class=\"btn primary\" href=\"/dashboard\">Open dashboard</a>"));
     server.sendContent(F("<a class=\"btn secondary\" href=\"/wifi\">Change Wi-Fi</a>"));
     server.sendContent(F("<a class=\"btn secondary\" href=\"/telegram\">Telegram ID</a>"));
     server.sendContent(F("<a class=\"btn\" href=\"/tester\">&#128295; Tester</a></div>"));
   } else {
     server.sendContent(F("<div class=\"row\"><a class=\"btn primary\" href=\"/wifi\">Set Wi-Fi</a>"));
     server.sendContent(F("<a class=\"btn secondary\" href=\"/telegram\">Telegram ID</a>"));
     server.sendContent(F("<a class=\"btn\" href=\"/dashboard\">Dashboard</a>"));
     server.sendContent(F("<a class=\"btn\" href=\"/tester\">&#128295; Tester</a></div>"));
   }
   server.sendContent(F("<small class=\"hint\">SoftAP: <b>Helmivo</b> / <b>Helmivo</b> · UI at "));
   server.sendContent(WiFi.softAPIP().toString().c_str());
   server.sendContent(F("</small></div>"));
   server.sendContent(F("<div class=\"card\"><h2>Status</h2><table>"));
   server.sendContent(F("<tr><td class=\"k\">SoftAP IP</td><td>"));
   server.sendContent(WiFi.softAPIP().toString().c_str());
   server.sendContent(F("</td></tr><tr><td class=\"k\">STA IP</td><td>"));
   server.sendContent(sta ? WiFi.localIP().toString() : String("-"));
   server.sendContent(F("</td></tr><tr><td class=\"k\">Saved SSID</td><td>"));
   server.sendContent(storedSsid.length() ? storedSsid : String("(none)"));
   server.sendContent(F("</td></tr><tr><td class=\"k\">Telegram chat ID (read-only)</td><td>"));
   server.sendContent(storedChatId.length() ? storedChatId : String("(not set)"));
   server.sendContent(F("</td></tr></table><p class=\"hint\" style=\"margin-top:12px\">You cannot type in the table above. Edit the chat ID in the <a href=\"#telegram-setup\"><b>Telegram alerts</b></a> box below.</p></div></div>"));
   server.sendContent(F("<div class=\"card\" id=\"telegram-setup\" style=\"margin-top:14px\"><h2>Telegram alerts</h2>"));
   server.sendContent(F("<p class=\"lead\" style=\"font-size:14px;margin:8px 0 12px\">Type your numeric chat ID here (saved on the device). Bot token is hidden unless you change it.</p>"));
   server.sendContent(F("<form method=\"POST\" action=\"/telegram\"><input type=\"hidden\" name=\"after\" value=\"/\"/>"));
   server.sendContent(F("<label for=\"tg-chat-home\">Chat ID</label><input id=\"tg-chat-home\" type=\"text\" name=\"chat\" autocomplete=\"off\" spellcheck=\"false\" placeholder=\"e.g. 123456789\" value=\""));
   server.sendContent(htmlAttrEscape(storedChatId).c_str());
   server.sendContent(F("\"/><label for=\"tg-bot-home\">Bot token</label><input id=\"tg-bot-home\" name=\"bot\" type=\"password\" autocomplete=\"off\" placeholder=\"from BotFather (leave blank to keep current)\"/><div class=\"row\">"));
   server.sendContent(F("<button class=\"btn primary\" type=\"submit\">Save Telegram settings</button></div>"));
   server.sendContent(F("<small class=\"hint\">After saving you stay on this page. Open the dashboard when ready.</small></form></div>"));
   sendPageFooter();
   server.sendContent("");
 }

 static void handleWifiGet() {
   server.setContentLength(CONTENT_LENGTH_UNKNOWN);
   server.send(200, "text/html", "");
   sendPageHeader("Helmivo · Wi-Fi");
   server.sendContent(F("<div class=\"top\"><div><div class=\"badge\">Network</div><h1>Wi-Fi setup</h1>"));
   server.sendContent(F("<p class=\"lead\">Stored in NVS — not hardcoded.</p></div>"));
   server.sendContent(F("<a class=\"btn\" href=\"/\">Back</a></div><div class=\"card\">"));
   server.sendContent(F("<form method=\"POST\" action=\"/wifi\"><label>SSID</label>"));
   server.sendContent(F("<input name=\"ssid\" required value=\""));
   server.sendContent(storedSsid.c_str());
   server.sendContent(F("\"/><label>Password</label><input name=\"pass\" type=\"password\" "));
   server.sendContent(F("autocomplete=\"current-password\" placeholder=\"Password\"/><div class=\"row\">"));
   server.sendContent(F("<button class=\"btn primary\" type=\"submit\">Save & reboot</button>"));
   server.sendContent(F("</div><small class=\"hint\">Reboot applies STA connection.</small></form></div>"));
   sendPageFooter();
   server.sendContent("");
 }

 static void handleWifiPost() {
   saveWifi(server.arg("ssid"), server.arg("pass"));
   server.send(200, "text/html", "<!doctype html><meta charset=\"utf-8\"/><p>Saved. Rebooting…</p>");
   delay(400);
   ESP.restart();
 }

 static void handleTelegramGet() {
   server.setContentLength(CONTENT_LENGTH_UNKNOWN);
   server.send(200, "text/html", "");
   sendPageHeader("Helmivo · Telegram");

   // Construct the HTML page in a single String buffer to avoid sendContent fragmentation
   String page;
   page.reserve(1024);

   page += F("<div class=\"top\"><div><div class=\"badge\">Alerting</div><h1>Telegram</h1>");
   page += F("<p class=\"lead\">Numeric chat ID (try @userinfobot).</p></div>");
   page += F("<a class=\"btn\" href=\"/\">Back</a></div><div class=\"card\">");
   
   // Added enctype for robust POST data parsing
   page += F("<form method=\"POST\" action=\"/telegram\" enctype=\"application/x-www-form-urlencoded\">");
   page += F("<input type=\"hidden\" name=\"after\" value=\"/telegram\"/>");

   page += F("<label for=\"tg-chat-page\">Chat ID</label>");
   page += F("<input id=\"tg-chat-page\" type=\"text\" name=\"chat\" autocomplete=\"off\" spellcheck=\"false\" placeholder=\"numeric ID\" value=\"");
   page += htmlAttrEscape(storedChatId);
   page += F("\"/>");

   page += F("<label for=\"tg-bot-page\">Bot token</label>");
   page += F("<input id=\"tg-bot-page\" name=\"bot\" type=\"password\" autocomplete=\"off\" placeholder=\"blank = keep current token\"/>");

   page += F("<div class=\"row\"><button class=\"btn primary\" type=\"submit\">Save</button></div>");
   page += F("<small class=\"hint\">Nothing is hardcoded: both values live in NVS. Rotate token with @BotFather if leaked.</small>");
   page += F("</form></div>");

   server.sendContent(page);
   sendPageFooter();
   server.sendContent("");
 }

 static void handleTelegramPost() {
   // Safely extract arguments, defaulting to empty strings if missing
   String chat = server.hasArg("chat") ? server.arg("chat") : "";
   String bot = server.hasArg("bot") ? server.arg("bot") : "";
   String after = server.hasArg("after") ? server.arg("after") : "/dashboard";

   saveTelegram(chat, bot);

   const char *loc = telegramPostRedirectTarget(after);
   server.sendHeader("Location", loc, true);
   server.send(303, "text/plain", "");
 }

 static void handleApiSetTelegram() {
   String bot = server.arg("bot");
   saveTelegram(server.arg("chat"), bot);
   String j = "{\"ok\":true,\"chat_set\":";
   j += storedChatId.length() ? "true" : "false";
   j += ",\"bot_set\":";
   j += storedBotToken.length() ? "true" : "false";
   j += "}";
   server.send(200, "application/json", j);
 }

 // ═══════════════════════════════════════════════════════════════════════════
 //  DASHBOARD — now includes Serial Monitor panel
 // ═══════════════════════════════════════════════════════════════════════════
 static void handleDashboard() {
   server.setContentLength(CONTENT_LENGTH_UNKNOWN);
   server.send(200, "text/html", "");
   sendPageHeader("Helmivo · Dashboard");

   server.sendContent(F("<div class=\"top\"><div><div class=\"badge\">Live</div><h1>Dashboard</h1>"));
   server.sendContent(F("<p class=\"lead\">Auto refresh ~1.5s. AP stays on for setup.</p></div>"));
   server.sendContent(F("<div class=\"row\" style=\"margin:0\">"));
   server.sendContent(F("<a class=\"btn\" href=\"/\">Home</a>"));
   server.sendContent(F("<a class=\"btn secondary\" href=\"/wifi\">Change Wi-Fi</a>"));
   server.sendContent(F("<a class=\"btn secondary\" href=\"/telegram\">Telegram ID</a>"));
   server.sendContent(F("<a class=\"btn\" href=\"/tester\">&#128295; Tester</a>"));
   server.sendContent(F("<button class=\"btn\" type=\"button\" id=\"cancel\">Cancel impact send</button></div>"));

   server.sendContent(F("<div class=\"card\" style=\"margin-top:14px\"><h2>Telegram</h2>"));
   server.sendContent(F("<p style=\"margin:0 0 10px;font-size:13px;color:var(--muted)\">Edit chat ID in the fields below (not in the Overview table).</p>"));
   server.sendContent(F("<form method=\"POST\" action=\"/telegram\"><input type=\"hidden\" name=\"after\" value=\"/dashboard\"/>"));
   server.sendContent(F("<label for=\"tg-chat-dash\">Chat ID</label><input id=\"tg-chat-dash\" type=\"text\" name=\"chat\" autocomplete=\"off\" spellcheck=\"false\" value=\""));
   server.sendContent(htmlAttrEscape(storedChatId).c_str());
   server.sendContent(F("\"/><label for=\"tg-bot-dash\">Bot token</label><input id=\"tg-bot-dash\" name=\"bot\" type=\"password\" autocomplete=\"off\" placeholder=\"blank = keep current\"/><div class=\"row\">"));
   server.sendContent(F("<button class=\"btn primary\" type=\"submit\">Save</button></div></form></div>"));

   // Overview + raw JSON (existing grid)
   server.sendContent(F("<div class=\"grid grid2\" style=\"margin-top:14px\">"));
   server.sendContent(F("<div class=\"card\"><h2>Overview</h2><table id=\"ov\"></table></div>"));
   server.sendContent(F("<div class=\"card\"><h2>Raw JSON</h2><pre id=\"tl\"></pre></div></div>"));

   // ── Serial Monitor panel ──────────────────────────────────────────────
   server.sendContent(F(R"html(
 <div class="card" style="margin-top:14px">
   <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:10px">
     <h2 style="margin:0">&#9654;&nbsp;Serial Monitor</h2>
     <div style="display:flex;gap:8px;align-items:center">
       <span id="sm-status" style="font-size:11px;color:var(--muted)">connecting…</span>
       <button id="sm-pause" class="btn" style="padding:5px 10px;font-size:12px">&#9646;&#9646; Pause</button>
       <button id="sm-clear" class="btn" style="padding:5px 10px;font-size:12px">&#128465; Clear</button>
       <label style="display:flex;align-items:center;gap:5px;font-size:12px;color:var(--muted);cursor:pointer;margin:0">
         <input type="checkbox" id="sm-autoscroll" checked style="width:auto;padding:0"/> Autoscroll
       </label>
     </div>
   </div>
   <div id="sm-wrap" style="
     background:#070b10;
     border:1px solid var(--line);
     border-radius:10px;
     height:320px;
     overflow-y:auto;
     padding:10px 12px;
     font-family:'JetBrains Mono','Fira Code',ui-monospace,monospace;
     font-size:12px;
     line-height:1.65;
     position:relative;
   ">
     <div id="sm-lines"></div>
   </div>
   <div style="display:flex;gap:16px;margin-top:8px;font-size:11px;color:var(--muted)">
     <span>&#9679; <span style="color:var(--ok)">[HELMIVO]</span> system</span>
     <span>&#9679; <span style="color:#fbbf24">[TELEM]</span> telemetry</span>
     <span>&#9679; <span style="color:var(--danger)">[ACCIDENT]</span> accident</span>
     <span>&#9679; <span style="color:#a78bfa">[MPU]</span> sensor</span>
     <span>&#9679; <span style="color:var(--muted)">other</span></span>
   </div>
 </div>
 )html"));

   // ── JavaScript (overview poll + serial monitor) ───────────────────────
   server.sendContent(F(R"html(
 <script>
 // ── overview poll ──────────────────────────────────────────────────────────
 async function tick(){
   try{
     const r=await fetch('/api/status');
     const j=await r.json();
     const on=j.wifi&&j.wifi.connected;
     document.getElementById('ov').innerHTML=
       `<tr><td class="k">Wi-Fi STA</td><td>${on?j.wifi.ssid:'offline'} (${on?j.wifi.rssi:'—'} dBm)</td></tr>`+
       `<tr><td class="k">SoftAP</td><td>${j.ap.ip}</td></tr>`+
       `<tr><td class="k">Telegram</td><td>${j.telegram.last_status} · ${j.telegram.chat_set?'chat OK':'NO CHAT'} · bot ${j.telegram.bot_set?'OK':'NOT SET'}</td></tr>`+
       `<tr><td class="k">GPS</td><td>fix ${j.gps.fix?'yes':'no'} · sats ${j.gps.sats} · ${j.gps.lat}, ${j.gps.lon}</td></tr>`+
       `<tr><td class="k">IMU</td><td>G ${j.imu.g} · pitch ${j.imu.pitch}° · roll ${j.imu.roll}° · impacts ${j.imu.impacts} · subtle ${j.imu.subtle_count}</td></tr>`+
       `<tr><td class="k">Driver</td><td>${j.driver.accident} · down=${j.driver.down} · inactive=${j.driver.inactive}</td></tr>`+
       `<tr><td class="k">Alert pipeline</td><td>${j.alert.telegram_state} · ${j.alert.impact_countdown_sec}s left · buzz=${j.alert.buzzer_warn}</td></tr>`+
       `<tr><td class="k">Lights</td><td>${j.lights.mode} · night=${j.lights.night_window}</td></tr>`;
     document.getElementById('tl').textContent=JSON.stringify(j,null,2);
   }catch(e){document.getElementById('tl').textContent=String(e)}
 }
 document.getElementById('cancel').onclick=async()=>{
   await fetch('/api/cancel-alert',{method:'POST'}); tick();
 };
 setInterval(tick,1500); tick();

 // ── serial monitor ─────────────────────────────────────────────────────────
 (function(){
   const wrap    = document.getElementById('sm-wrap');
   const lines   = document.getElementById('sm-lines');
   const status  = document.getElementById('sm-status');
   const btnPause= document.getElementById('sm-pause');
   const btnClear= document.getElementById('sm-clear');
   const cbScroll= document.getElementById('sm-autoscroll');

   let cursor  = 0;       // next seq we want
   let paused  = false;
   let pending = [];      // buffer while paused
   const MAX_LINES = 500; // DOM line cap

   // ── colour tags ──
   const TAG_COLORS = [
     ['[ACCIDENT]', '#fb7185'],
     ['[HELMIVO]',  '#34d399'],
     ['[TELEM]',    '#fbbf24'],
     ['[TESTER]',   '#00e5ff'],
     ['[MPU]',      '#a78bfa'],
     ['[SUBTLE]',   '#64748b'],
     ['[GPS]',      '#38bdf8'],
   ];

   function colorize(raw) {
     const esc = raw.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
     for (const [tag, col] of TAG_COLORS) {
       if (esc.includes(tag)) {
         return esc.replace(
           tag,
           `<span style="color:${col};font-weight:700">${tag}</span>`
         );
       }
     }
     return `<span style="color:#8b9bb4">${esc}</span>`;
   }

   function tsLabel(ms) {
     const s = (ms / 1000) | 0;
     const m = (s / 60) | 0;
     const sec = s % 60;
     return String(m).padStart(2,'0') + ':' + String(sec).padStart(2,'0') + '.' +
            String(ms % 1000).padStart(3,'0');
   }

   function appendLines(batch) {
     const frag = document.createDocumentFragment();
     for (const entry of batch) {
       const row = document.createElement('div');
       row.style.cssText =
         'display:flex;gap:10px;padding:1px 0;border-bottom:1px solid rgba(255,255,255,.03)';
       const ts = document.createElement('span');
       ts.style.cssText = 'color:#334155;white-space:nowrap;user-select:none;min-width:72px';
       ts.textContent = tsLabel(entry.ts);
       const txt = document.createElement('span');
       txt.style.cssText = 'flex:1;word-break:break-all;color:#cbd5e1';
       txt.innerHTML = colorize(entry.text);
       row.appendChild(ts);
       row.appendChild(txt);
       frag.appendChild(row);
     }
     lines.appendChild(frag);

     // trim DOM to MAX_LINES
     while (lines.children.length > MAX_LINES) {
       lines.removeChild(lines.firstChild);
     }

     if (cbScroll.checked) {
       wrap.scrollTop = wrap.scrollHeight;
     }
   }

   function flush() {
     if (pending.length === 0) return;
     appendLines(pending);
     pending = [];
   }

   btnPause.onclick = () => {
     paused = !paused;
     btnPause.textContent = paused ? '▶ Resume' : '⏸ Pause';
     btnPause.style.borderColor = paused ? 'var(--accent)' : '';
     btnPause.style.color       = paused ? 'var(--accent)' : '';
     if (!paused) flush();
   };

   btnClear.onclick = () => {
     lines.innerHTML = '';
     pending = [];
   };

   let errorCount = 0;
   async function poll() {
     try {
       const r = await fetch('/api/serial-log?since=' + cursor);
       const j = await r.json();
       errorCount = 0;
       status.textContent = 'live · seq ' + j.next_seq;
       status.style.color = 'var(--ok)';
       if (j.lines && j.lines.length > 0) {
         cursor = j.next_seq;
         if (paused) {
           pending.push(...j.lines);
         } else {
           appendLines(j.lines);
         }
       }
     } catch(e) {
       errorCount++;
       status.textContent = 'error ×' + errorCount;
       status.style.color = 'var(--danger)';
     }
   }

   setInterval(poll, 700);
   poll();
 })();
 </script>
 )html"));

   sendPageFooter();
   server.sendContent("");
 }

 static void handleApiStatus() {
   bool sta = (WiFi.status() == WL_CONNECTED);
   unsigned long cd = 0;
   if (tgActiveKind == TG_IMPACT) {
     unsigned long e = millis() - tgCountdownStartMs;
     if (e < TELEGRAM_COUNTDOWN_TOTAL_MS) cd = (TELEGRAM_COUNTDOWN_TOTAL_MS - e + 999UL) / 1000UL;
   }
   String j = "{";
   j += "\"wifi\":{";
   j += "\"connected\":" + String(sta ? "true" : "false");
   j += ",\"ssid\":\"" + jsonEscape(sta ? WiFi.SSID() : String("")) + "\"";
   j += ",\"rssi\":" + String(sta ? WiFi.RSSI() : 0);
   j += "},\"ap\":{\"ip\":\"" + jsonEscape(WiFi.softAPIP().toString()) + "\"}";
   j += ",\"gps\":{";
   j += "\"fix\":" + String(gps.fix ? "true" : "false");
   j += ",\"lat\":" + String(gps.lat, 6);
   j += ",\"lon\":" + String(gps.lon, 6);
   j += ",\"sats\":" + String((int)gps.satellites);
   j += ",\"age_ms\":" + String(gps.lastFixMs ? (millis() - gps.lastFixMs) : 999999);
   j += "},\"imu\":{";
   j += "\"g\":" + String(g_imuTotalG, 3);
   j += ",\"pitch\":" + String(g_imuPitch, 2);
   j += ",\"roll\":" + String(g_imuRoll, 2);
   j += ",\"peak_g\":" + String(peakGSession, 3);
   j += ",\"impacts\":" + String(impactCount);
   j += ",\"subtle_count\":" + String(subtleCount);
   j += ",\"subtle_last_g\":" + String(lastSubtleG, 3);
   j += "},\"driver\":{";
   j += "\"accident\":\"" + jsonEscape(String(accidentStateLabel())) + "\"";
   j += ",\"down\":" + String(flagDriverDown ? "true" : "false");
   j += ",\"inactive\":" + String(flagDriverInactive ? "true" : "false");
   j += "},\"alert\":{";
   j += "\"impact_countdown_sec\":" + String((unsigned long)cd);
   j += ",\"buzzer_warn\":" + String(tgBuzzerPhase ? "true" : "false");
   j += ",\"telegram_state\":\"" + String(tgActiveKind == TG_IMPACT ? "countdown" : "idle") + "\"";
   j += "},\"lights\":{";
   j += "\"night_window\":" + String(inNightLightWindow() ? "true" : "false");
   j += ",\"mode\":\"" +
        String(relayEmergencyMode ? "emergency_blink" : (inNightLightWindow() ? "night_on" : "day_off")) +
        "\"";
   j += "},\"telegram\":{";
   j += "\"last_status\":\"" + jsonEscape(String(lastTelegramStatus)) + "\"";
   j += ",\"chat_set\":" + String(storedChatId.length() > 0 ? "true" : "false");
   j += ",\"bot_set\":" + String(storedBotToken.length() > 0 ? "true" : "false");
   j += "}}";
   server.send(200, "application/json", j);
 }

 static void handleCancelAlert() {
   cancelTelegramCountdown("web");
   if (accidentState == ST_CANCEL_WINDOW) {
     accidentState = ST_NORMAL;
     cancelLastPrintedSec = -1;
     resetAccidentFlags();
     softFallSegmentStartMs = 0;
   }
   server.send(200, "application/json", "{\"ok\":true}");
 }

 static void sendAccidentSerialJson(float peakGVal, float peakTiltDeg, bool freefallFlag,
                                   bool violentSpinFlag, bool softFallFlag, uint8_t severity,
                                   unsigned long impacts) {
   Serial.print(F("{\"event\":\"ACCIDENT\",\"peakG\":"));
   Serial.print(peakGVal, 2);
   Serial.print(F(",\"tiltDeg\":"));
   Serial.print(peakTiltDeg, 1);
   Serial.print(F(",\"freefall\":"));
   Serial.print(freefallFlag ? F("true") : F("false"));
   Serial.print(F(",\"violentSpin\":"));
   Serial.print(violentSpinFlag ? F("true") : F("false"));
   Serial.print(F(",\"softFall\":"));
   Serial.print(softFallFlag ? F("true") : F("false"));
   Serial.print(F(",\"severity\":"));
   Serial.print(severity);
   Serial.print(F(",\"impacts\":"));
   Serial.print(impacts);
   Serial.println(F("}"));
 }

 // ═══════════════════════════════════════════════════════════════════════════
 //  setup()
 // ═══════════════════════════════════════════════════════════════════════════
 void setup() {
   Serial.begin(115200);
   delay(300);

   pinMode(PIN_RELAY, OUTPUT);
   pinMode(PIN_BUZZER, OUTPUT);
   pinMode(PIN_BUTTON, INPUT_PULLUP);
   digitalWrite(PIN_RELAY, HIGH);
   digitalWrite(PIN_BUZZER, LOW);

   loadPrefs();

   gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

   Wire.begin(I2C_SDA, I2C_SCL);
   Wire.setClock(400000);
   mpuWrite(REG_PWR_MGMT_1, 0x00);
   mpuWrite(REG_ACCEL_CONFIG, 0x10);
   mpuWrite(REG_GYRO_CONFIG, 0x00);
   delay(80);
   calibrateGyro();

   RawData d0;
   mpuRead(&d0);
   float ax0 = d0.ax / ACCEL_SCALE;
   float ay0 = d0.ay / ACCEL_SCALE;
   float az0 = d0.az / ACCEL_SCALE;
   pitch = atan2f(-ax0, sqrtf(ay0 * ay0 + az0 * az0)) * RAD_TO_DEG;
   roll = atan2f(ay0, az0) * RAD_TO_DEG;
   lastTime = micros();

   WiFi.mode(WIFI_AP_STA);
   WiFi.softAP(AP_SSID, AP_PASS);
   if (storedSsid.length() > 0) WiFi.begin(storedSsid.c_str(), storedWifiPass.c_str());

   // ── existing routes ───────────────────────────────────────────────────
   server.on("/",                 HTTP_GET,  handleIndex);
   server.on("/wifi",             HTTP_GET,  handleWifiGet);
   server.on("/wifi",             HTTP_POST, handleWifiPost);
   server.on("/telegram",         HTTP_GET,  handleTelegramGet);
   server.on("/telegram",         HTTP_POST, handleTelegramPost);
   server.on("/dashboard",        HTTP_GET,  handleDashboard);
   server.on("/api/status",       HTTP_GET,  handleApiStatus);
   server.on("/api/set-telegram", HTTP_POST, handleApiSetTelegram);
   server.on("/api/cancel-alert", HTTP_POST, handleCancelAlert);

   // ── tester routes ─────────────────────────────────────────────────────
   server.on("/tester",                HTTP_GET,  handleTesterPage);
   server.on("/api/test/accident",     HTTP_POST, handleTestAccident);
   server.on("/api/test/flashlight",   HTTP_POST, handleTestFlashlight);
   server.on("/api/test/buzzer",       HTTP_POST, handleTestBuzzer);
   server.on("/api/test/posture",      HTTP_GET,  handleTestPosture);
   server.on("/api/test/gps",          HTTP_GET,  handleTestGps);

   // ── serial log route ──────────────────────────────────────────────────
   server.on("/api/serial-log",   HTTP_GET,  handleSerialLog);

   server.begin();

   Serial.println(F("\n=== Helmivo ready ==="));
   Serial.print(F("AP: "));
   Serial.print(AP_SSID);
   Serial.print(F(" IP: "));
   Serial.println(WiFi.softAPIP());
   Serial.println(F("Serial: 'c' cancel countdown | 'r' reset stats | 't' test countdown"));
   Serial.println(F("Dashboard serial monitor: http://192.168.4.1/dashboard"));
 }

 // ═══════════════════════════════════════════════════════════════════════════
 //  loop()
 // ═══════════════════════════════════════════════════════════════════════════
 void loop() {
   unsigned long nowMs = millis();
   server.handleClient();
   configNtpIfNeeded();

   if (nowMs - lastGpsDrainMs >= 1000) {
     lastGpsDrainMs = nowMs;
     drainGpsForPeriod(100);
   }

   while (Serial.available()) {
     char c = (char)Serial.read();
     if (c == 'r' || c == 'R') {
       impactCount = 0;
       peakGSession = 1.0f;
       lastImpactMs = 0;
       impactArmed = true;
       accidentState = ST_NORMAL;
       resetAccidentFlags();
       softFallSegmentStartMs = 0;
       freefallStartMs = 0;
       wasInFreefallLatch = false;
       cancelLastPrintedSec = -1;
       Serial.println(F("[HELMIVO] Impact + accident reset"));
     } else if (c == 'c' || c == 'C') {
       cancelTelegramCountdown("serial");
       if (accidentState == ST_CANCEL_WINDOW) {
         accidentState = ST_NORMAL;
         cancelLastPrintedSec = -1;
         resetAccidentFlags();
         softFallSegmentStartMs = 0;
         Serial.println(F("[HELMIVO] Accident cancel window cleared"));
       }
     } else if (c == 't' || c == 'T') {
       helmivoTestCountdownPending = true;
       Serial.println(F("[HELMIVO] Test: arming 15s Telegram countdown"));
     }
   }

   updateTelegramCountdown(nowMs);
   updateButton(nowMs);

   unsigned long now = micros();
   float dt = (now - lastTime) / 1e6f;
   lastTime = now;

   RawData d;
   mpuRead(&d);
   float ax = d.ax / ACCEL_SCALE;
   float ay = d.ay / ACCEL_SCALE;
   float az = d.az / ACCEL_SCALE;
   float totalG = sqrtf(ax * ax + ay * ay + az * az);
   if (totalG > peakGSession) peakGSession = totalG;

   float tiltDeg = tiltFromVertical(ax, ay, az, totalG);
   float gyroMag = gyroMagnitudeDegPerS(&d);

   g_imuTiltDeg = tiltDeg;
   g_imuGyroMag = gyroMag;

   if (totalG >= SUBTLE_G_LO && totalG < IMPACT_LIGHT &&
       (nowMs - subtleLastMs) > SUBTLE_COOLDOWN_MS) {
     subtleLastMs = nowMs;
     subtleCount++;
     lastSubtleG = totalG;
     Serial.printf("[SUBTLE] bump g=%.2f count=%lu\n", totalG, subtleCount);
   }

   if (totalG < FREEFALL_G) {
     if (freefallStartMs == 0) freefallStartMs = nowMs;
     else if (nowMs - freefallStartMs >= (unsigned long)FREEFALL_MIN_MS) wasInFreefallLatch = true;
   } else {
     freefallStartMs = 0;
   }

   if (impactArmed && totalG >= IMPACT_LIGHT && (nowMs - lastImpactMs) > IMPACT_COOLDOWN_MS) {
     lastImpactMs = nowMs;
     impactArmed = false;
     impactCount++;
     const char *sev = (totalG >= IMPACT_SEVERE) ? "SEVERE" : (totalG >= IMPACT_MODERATE ? "MODERATE" : "LIGHT");
     Serial.printf("\n*** IMPACT [%s] %.2f g hit #%lu ***\n", sev, totalG, impactCount);
     startImpactTelegramCountdown();
   }
   if (!impactArmed && totalG < IMPACT_REARM) impactArmed = true;

   float gyroX = (d.gx - gyroBiasX) / GYRO_SCALE;
   float gyroY = (d.gy - gyroBiasY) / GYRO_SCALE;
   float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
   float accelRoll = atan2f(ay, az) * RAD_TO_DEG;
   pitch = ALPHA * (pitch + gyroX * dt) + (1.0f - ALPHA) * accelPitch;
   roll = ALPHA * (roll + gyroY * dt) + (1.0f - ALPHA) * accelRoll;

   if (helmivoTestCountdownPending) {
     helmivoTestCountdownPending = false;
     startImpactTelegramCountdown();
   }

   if (accidentState == ST_LOCKOUT) {
     if (nowMs - stateEnteredMs >= (unsigned long)ALERT_LOCKOUT_MS) {
       accidentState = ST_NORMAL;
       cancelLastPrintedSec = -1;
       resetAccidentFlags();
       softFallSegmentStartMs = 0;
     }
   } else if (accidentState == ST_CANCEL_WINDOW) {
     if (uprightRecovered(tiltDeg, totalG, az)) {
       accidentState = ST_NORMAL;
       cancelLastPrintedSec = -1;
       resetAccidentFlags();
       softFallSegmentStartMs = 0;
       Serial.println(F("[ACCIDENT] Cleared — upright during cancel window"));
     } else {
       unsigned long elapsedC = nowMs - stateEnteredMs;
       uint16_t remain = (elapsedC < (unsigned long)CANCEL_WINDOW_MS)
                             ? (uint16_t)((unsigned long)CANCEL_WINDOW_MS - elapsedC)
                             : 0;
       uint8_t secLeft = (uint8_t)((remain + 999UL) / 1000UL);
       if ((int)secLeft != cancelLastPrintedSec) {
         cancelLastPrintedSec = secLeft;
         Serial.printf("[ACCIDENT] cancel window — %u s left (serial 'c')\n", secLeft);
       }
       if (elapsedC >= (unsigned long)CANCEL_WINDOW_MS) {
         sendAccidentSerialJson(eventPeakG, eventPeakTilt, eventHadFreefall, eventViolentSpin,
                                flagSoftFallPath, eventSeverity, impactCount);
         accidentState = ST_LOCKOUT;
         stateEnteredMs = nowMs;
         cancelLastPrintedSec = -1;
         softFallSegmentStartMs = 0;
         Serial.println(F("[ACCIDENT] Serial JSON logged (internal pipeline)"));
       }
     }
   } else if (accidentState == ST_INACTIVITY) {
     if (totalG > eventPeakG) eventPeakG = totalG;
     if (tiltDeg > eventPeakTilt) eventPeakTilt = tiltDeg;
     if (gyroMag > VIOLENT_SPIN_GYRO_DPS) eventViolentSpin = true;
     if (uprightRecovered(tiltDeg, totalG, az)) {
       accidentState = ST_NORMAL; cancelLastPrintedSec = -1; resetAccidentFlags(); softFallSegmentStartMs = 0;
       Serial.println(F("[ACCIDENT] Cleared — upright"));
     } else if (!orientationDriverDown(tiltDeg, ax, ay, az)) {
       accidentState = ST_NORMAL; cancelLastPrintedSec = -1; resetAccidentFlags(); softFallSegmentStartMs = 0;
       Serial.println(F("[ACCIDENT] Cleared — posture OK"));
     } else if (motionLooksActive(gyroMag)) {
       stateEnteredMs = nowMs;
     } else if (nowMs - stateEnteredMs >= (unsigned long)INACTIVITY_REQUIRED_MS) {
       flagDriverInactive = true;
       eventSeverity = computeAccidentSeverity(eventPeakG, eventPeakTilt, eventHadFreefall, eventViolentSpin, false);
       accidentState = ST_CANCEL_WINDOW; stateEnteredMs = nowMs; cancelLastPrintedSec = -1;
       Serial.println(F("[ACCIDENT] Conditions met — cancel window"));
     }
   } else if (accidentState == ST_IMPACT_SETTLE) {
     if (totalG > eventPeakG) eventPeakG = totalG;
     if (tiltDeg > eventPeakTilt) eventPeakTilt = tiltDeg;
     if (gyroMag > VIOLENT_SPIN_GYRO_DPS) eventViolentSpin = true;
     if (uprightRecovered(tiltDeg, totalG, az)) {
       accidentState = ST_NORMAL; cancelLastPrintedSec = -1; resetAccidentFlags(); softFallSegmentStartMs = 0;
       Serial.println(F("[ACCIDENT] Impact cleared — upright"));
     } else if (nowMs - stateEnteredMs >= (unsigned long)POST_IMPACT_SETTLE_MS) {
       if (orientationDriverDown(tiltDeg, ax, ay, az)) {
         flagDriverDown = true; accidentState = ST_INACTIVITY; stateEnteredMs = nowMs;
         Serial.println(F("[ACCIDENT] DRIVER_DOWN — inactivity watch"));
       } else {
         accidentState = ST_NORMAL; cancelLastPrintedSec = -1; resetAccidentFlags(); softFallSegmentStartMs = 0;
         Serial.println(F("[ACCIDENT] Posture OK after settle"));
       }
     }
   } else {
     bool condA = wasInFreefallLatch;
     bool condB = (totalG >= HIGH_G_IMPACT);
     bool condC = (gyroMag >= VIOLENT_SPIN_GYRO_DPS);
     if (condB || condC || condA) {
       flagImpactDetected = true; eventHadFreefall = condA; eventViolentSpin = condC;
       eventPeakG = totalG; eventPeakTilt = tiltDeg;
       accidentState = ST_IMPACT_SETTLE; stateEnteredMs = nowMs;
       wasInFreefallLatch = false; freefallStartMs = 0; softFallSegmentStartMs = 0;
       Serial.println(F("[ACCIDENT] High-g / spin / freefall — settle"));
     } else {
       bool downNow = orientationDriverDown(tiltDeg, ax, ay, az);
       bool moving = motionLooksActive(gyroMag);
       if (downNow && !moving) {
         if (softFallSegmentStartMs == 0) softFallSegmentStartMs = nowMs;
         else if (nowMs - softFallSegmentStartMs >= (unsigned long)SOFT_FALL_STILL_MS) {
           flagSoftFallPath = true; flagDriverDown = true; flagDriverInactive = true;
           flagImpactDetected = false; eventHadFreefall = false; eventViolentSpin = false;
           eventPeakG = totalG; eventPeakTilt = tiltDeg;
           eventSeverity = computeAccidentSeverity(eventPeakG, eventPeakTilt, false, false, true);
           accidentState = ST_CANCEL_WINDOW; stateEnteredMs = nowMs; cancelLastPrintedSec = -1;
           softFallSegmentStartMs = 0;
           Serial.println(F("[ACCIDENT] Soft fall — cancel window"));
         }
       } else {
         softFallSegmentStartMs = 0;
       }
     }
   }

   g_imuTotalG = totalG;
   g_imuPitch = pitch;
   g_imuRoll = roll;

   updateRelayAndBuzzer(nowMs);

   if (nowMs - lastTelemetrySerialMs >= 1000) {
     lastTelemetrySerialMs = nowMs;
     Serial.printf(
         "[TELEM] G=%.2f P=%.1f R=%.1f | GPS %s %.6f,%.6f | tg=%s | acc=%s | CD=%s\n",
         totalG, pitch, roll, gps.fix ? "FIX" : "no", gps.lat, gps.lon, lastTelegramStatus,
         accidentStateLabel(),
         tgActiveKind == TG_IMPACT ? "15s" : "idle");
   }

   delay(10);
 }