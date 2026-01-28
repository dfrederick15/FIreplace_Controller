#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <FS.h>
#include <SPIFFS.h>
#include <time.h>

// =========================
// Firmware Version
// =========================
const char* FW_VERSION = "SolenoidCtl v1.3.10";  // bumped version

// =========================
// WiFi Credentials
// =========================
const char* WIFI_SSID = "Frederick_iot";
const char* WIFI_PASS = "fred2018";

// =========================
// Web UI Security
// =========================
const char* WEB_USER = "admin";    // Web UI username
const char* WEB_PASS = "solenoid"; // Web UI password

// =========================
// Pin configuration
// =========================
const int PIN_TRIGGER = 23;

// Relay outputs
const int RELAY1_PIN = 32;  // +5V Switch
const int RELAY2_PIN = 33;  // GND Switch
const int RELAY3_PIN = 25;  // Direction A
const int RELAY4_PIN = 26;  // Direction B

// Relay logic
const uint8_t RELAY_ACTIVE_LEVEL   = HIGH;
const uint8_t RELAY_INACTIVE_LEVEL = LOW;

// =========================
// Config defaults & runtime
// =========================
const uint32_t DEFAULT_SOLENOID_PULSE_MS = 3000;
const uint32_t DEFAULT_DEBOUNCE_MS       = 50;
const bool     DEFAULT_AUTO_TRIGGER      = true;
const uint32_t DEFAULT_RELAY_DELAY_MS    = 50;   // lead/lag between dir and power relays

uint32_t solenoidPulseMs = DEFAULT_SOLENOID_PULSE_MS;
uint32_t debounceMs      = DEFAULT_DEBOUNCE_MS;
bool     autoTrigger     = DEFAULT_AUTO_TRIGGER;
uint32_t relayDelayMs    = DEFAULT_RELAY_DELAY_MS;

// JSON config file path
const char* CONFIG_PATH      = "/config.json";
const char* LOG_CSV_PATH     = "/log.csv";
const char* STATE_LOG_PATH   = "/state_log.csv";

// =========================
// State Tracking
// =========================
enum LogicalState { STATE_OFF = 0, STATE_ON = 1 };

LogicalState currentLogicalState = STATE_OFF;
LogicalState lastHandledState    = STATE_OFF;

bool     actionInProgress = false;
uint32_t actionStartMs    = 0;

String   lastActionText   = "None";
uint32_t lastActionTimeMs = 0;

// Debounce tracking
int      lastRawReading = HIGH;
int      stableReading  = HIGH;
uint32_t lastChangeMs   = 0;

// Web server
WebServer server(80);

// WiFi auto-reconnect
uint32_t lastWifiCheckMs              = 0;
const uint32_t WIFI_CHECK_INTERVAL_MS = 10000;  // 10s

// SPIFFS mounted flag
bool spiffsMounted = false;

// File upload handle
File uploadFile;

// =========================
// Debug Log Buffer (500 lines)
// =========================
const int LOG_SIZE = 500;
String   logMessages[LOG_SIZE];
String   logTimeStr[LOG_SIZE];  // human-readable time
uint32_t logTimes[LOG_SIZE];    // millis uptime
int      logIndex   = 0;
bool     logWrapped = false;

// Log idle-save tracking
uint32_t lastLogActivityMs = 0;
uint32_t lastLogSaveMs     = 0;
bool     logDirty          = false;

// =========================
// State Log Buffer (100 lines)
// current state + current time
// =========================
const int STATE_LOG_SIZE = 100;
String       stateLogIso[STATE_LOG_SIZE];   // ISO time string
time_t       stateLogEpoch[STATE_LOG_SIZE]; // epoch seconds
LogicalState stateLogVal[STATE_LOG_SIZE];
int          stateLogIndex   = 0;
bool         stateLogWrapped = false;

// Startup commanded-state pulse (30s after reboot)
const uint32_t STARTUP_COMMAND_DELAY_MS = 30000UL;
bool     startupCommandDone = false;
uint32_t startupCommandAtMs = 0;

// =========================
// Logical state statistics
// =========================
// Counts and total time spent in ON / OFF logical states
uint32_t    onCount        = 0;
uint32_t    offCount       = 0;
uint64_t    totalOnTimeMs  = 0;
uint64_t    totalOffTimeMs = 0;
LogicalState statsLastState    = STATE_OFF;
uint32_t    statsLastChangeMs  = 0;
bool        statsInitialized   = false;

// Forward declarations
String getCurrentTimeString();
String formatDurationDDHHMMSS(uint64_t ms);

// ============================================================================
// Time / NTP
// ============================================================================
String getCurrentTimeString() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 0)) {
    return "1970-01-01 00:00:00";
  }
  char buf[20];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo); // UTC
  return String(buf);
}

// Format uptime/duration into DD:HH:MM:SS
String formatDurationDDHHMMSS(uint64_t ms) {
  uint64_t totalSeconds = ms / 1000ULL;

  uint32_t days    = totalSeconds / 86400ULL;   // 24*60*60
  totalSeconds    %= 86400ULL;
  uint8_t hours    = totalSeconds / 3600ULL;
  totalSeconds    %= 3600ULL;
  uint8_t minutes  = totalSeconds / 60ULL;
  uint8_t seconds  = totalSeconds % 60ULL;

  char buf[20];
  snprintf(buf, sizeof(buf), "%02u:%02u:%02u:%02u",
           days, hours, minutes, seconds);
  return String(buf);
}

// Parse DD:HH:MM:SS back to milliseconds.
// If no ':' present, treat as raw milliseconds (backwards-compatible).
uint32_t parseDurationDDHHMMSS(const String &sIn) {
  String s = sIn;
  s.trim();
  if (s.indexOf(':') < 0) {
    // Old format (pure ms)
    return (uint32_t)s.toInt();
  }

  int p1 = s.indexOf(':');
  int p2 = s.indexOf(':', p1 + 1);
  int p3 = s.indexOf(':', p2 + 1);
  if (p1 < 0 || p2 < 0 || p3 < 0) {
    return 0;
  }

  uint32_t d = s.substring(0, p1).toInt();
  uint32_t h = s.substring(p1 + 1, p2).toInt();
  uint32_t m = s.substring(p2 + 1, p3).toInt();
  uint32_t sec = s.substring(p3 + 1).toInt();

  uint64_t totalSeconds = (uint64_t)d * 86400ULL +
                          (uint64_t)h * 3600ULL +
                          (uint64_t)m * 60ULL +
                          (uint64_t)sec;

  uint64_t totalMs = totalSeconds * 1000ULL;
  if (totalMs > 0xFFFFFFFFULL) {
    totalMs = 0xFFFFFFFFULL; // clamp to 32-bit
  }
  return (uint32_t)totalMs;
}

// ============================================================================
// Logical state statistics helpers
// ============================================================================

void initStateStats(LogicalState initialState) {
  statsLastState   = initialState;
  statsLastChangeMs = millis();
  statsInitialized  = true;
}

// Called whenever the logical state actually changes (OFF->ON or ON->OFF)
void recordStateTransition(LogicalState newState) {
  uint32_t now = millis();

  if (!statsInitialized) {
    statsLastState   = newState;
    statsLastChangeMs = now;
    statsInitialized  = true;
    return;
  }

  uint32_t elapsed = now - statsLastChangeMs;
  if (statsLastState == STATE_ON) {
    totalOnTimeMs += elapsed;
  } else {
    totalOffTimeMs += elapsed;
  }

  if (newState == STATE_ON) {
    onCount++;
  } else {
    offCount++;
  }

  statsLastState   = newState;
  statsLastChangeMs = now;
}

// Get total ON/OFF times including the currently active period
void getEffectiveDurations(uint64_t &onMs, uint64_t &offMs) {
  uint32_t now = millis();
  onMs  = totalOnTimeMs;
  offMs = totalOffTimeMs;

  if (!statsInitialized) return;

  uint32_t elapsed = now - statsLastChangeMs;
  if (statsLastState == STATE_ON) {
    onMs += elapsed;
  } else {
    offMs += elapsed;
  }
}

// Log a state change with ON/OFF counts and durations
void logStateChangeWithStats(const char* source, LogicalState newState) {
  uint64_t onMs, offMs;
  getEffectiveDurations(onMs, offMs);

  String msg = String(source) +
               String(": state -> ") + (newState == STATE_ON ? "ON" : "OFF") +
               " | ON_count=" + String(onCount) +
               " OFF_count=" + String(offCount) +
               " ON_time=" + formatDurationDDHHMMSS(onMs) +
               " OFF_time=" + formatDurationDDHHMMSS(offMs);

  extern void addLog(const String &msg);
  addLog(msg);
}

// ============================================================================
// State Log: write & load
// ============================================================================
void writeStateLogCsv() {
  if (!spiffsMounted) return;

  File f = SPIFFS.open(STATE_LOG_PATH, "w");
  if (!f) {
    return;
  }

  f.println("idx,iso_time,epoch,logical");

  int count = stateLogWrapped ? STATE_LOG_SIZE : stateLogIndex;
  for (int i = 0; i < count; ++i) {
    int idx = stateLogWrapped ? (stateLogIndex + i) % STATE_LOG_SIZE : i;

    String iso = stateLogIso[idx];
    iso.replace("\"", "'");

    const char* logicalStr = (stateLogVal[idx] == STATE_ON ? "ON" : "OFF");

    f.print(i);
    f.print(",");
    f.print("\""); f.print(iso); f.print("\"");
    f.print(",");
    f.print((unsigned long)stateLogEpoch[idx]);
    f.print(",");
    f.println(logicalStr);
  }

  f.close();
}

void addStateLog(LogicalState st) {
  time_t nowEpoch = time(nullptr);
  String iso      = getCurrentTimeString();

  stateLogIso[stateLogIndex]   = iso;
  stateLogEpoch[stateLogIndex] = nowEpoch;
  stateLogVal[stateLogIndex]   = st;

  stateLogIndex = (stateLogIndex + 1) % STATE_LOG_SIZE;
  if (stateLogIndex == 0) stateLogWrapped = true;

  // Save immediately after every state change (per requirement)
  writeStateLogCsv();
}

// Get last state entry from state log; returns false if none
bool getLastStateEntry(LogicalState &outState, time_t &outEpoch) {
  int count = stateLogWrapped ? STATE_LOG_SIZE : stateLogIndex;
  if (count <= 0) return false;

  int lastIdx = stateLogWrapped
                ? (stateLogIndex + STATE_LOG_SIZE - 1) % STATE_LOG_SIZE
                : (stateLogIndex - 1);

  outState = stateLogVal[lastIdx];
  outEpoch = stateLogEpoch[lastIdx];
  return true;
}

void loadStateLog() {
  if (!spiffsMounted) return;
  if (!SPIFFS.exists(STATE_LOG_PATH)) return;

  File f = SPIFFS.open(STATE_LOG_PATH, "r");
  if (!f) return;

  stateLogIndex   = 0;
  stateLogWrapped = false;

  int count = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length() || line.startsWith("idx")) continue;

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    if (c1 < 0 || c2 < 0 || c3 < 0) continue;

    String iso = line.substring(c1 + 1, c2);
    iso.trim();
    if (iso.startsWith("\"") && iso.endsWith("\"")) {
      iso = iso.substring(1, iso.length() - 1);
    }

    String epochStr = line.substring(c2 + 1, c3);
    epochStr.trim();
    unsigned long epochVal = (unsigned long)epochStr.toInt();

    String logicalStr = line.substring(c3 + 1);
    logicalStr.trim();
    LogicalState s = STATE_OFF;
    if (logicalStr.equalsIgnoreCase("ON")) {
      s = STATE_ON;
    } else {
      s = STATE_OFF;
    }

    stateLogIso[stateLogIndex]   = iso;
    stateLogEpoch[stateLogIndex] = (time_t)epochVal;
    stateLogVal[stateLogIndex]   = s;

    stateLogIndex = (stateLogIndex + 1) % STATE_LOG_SIZE;
    if (stateLogIndex == 0) stateLogWrapped = true;

    count++;
    if (count >= STATE_LOG_SIZE) break;
  }

  f.close();
}

// ============================================================================
// Main Debug Log: load CSV into RAM buffer on boot
// ============================================================================
void loadCsvLog() {
  if (!spiffsMounted) return;
  if (!SPIFFS.exists(LOG_CSV_PATH)) return;

  File f = SPIFFS.open(LOG_CSV_PATH, "r");
  if (!f) return;

  logIndex = 0;
  logWrapped = false;

  int count = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length() || line.startsWith("idx")) continue;

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    if (c1 < 0 || c2 < 0 || c3 < 0) continue;

    String tstr = line.substring(c1 + 1, c2);
    if (tstr.startsWith("\"") && tstr.endsWith("\""))
      tstr = tstr.substring(1, tstr.length() - 1);

    String uptimeStr = line.substring(c2 + 1, c3);
    uptimeStr.trim();
    if (uptimeStr.startsWith("\"") && uptimeStr.endsWith("\"")) {
      uptimeStr = uptimeStr.substring(1, uptimeStr.length() - 1);
    }

    uint32_t ms = parseDurationDDHHMMSS(uptimeStr);

    String msg = line.substring(c3 + 1);
    if (msg.startsWith("\"") && msg.endsWith("\""))
      msg = msg.substring(1, msg.length() - 1);

    logTimeStr[logIndex]   = tstr;
    logTimes[logIndex]     = ms;
    logMessages[logIndex]  = msg;

    logIndex = (logIndex + 1) % LOG_SIZE;
    if (logIndex == 0) logWrapped = true;

    count++;
    if (count >= LOG_SIZE) break;
  }

  f.close();
}

// ============================================================================
// Write entire CSV main log containing up to last 500 entries
// ============================================================================
void writeCsvLog() {
  if (!spiffsMounted) return;

  File f = SPIFFS.open(LOG_CSV_PATH, "w");
  if (!f) {
    return;
  }

  // Header: uptime in DD:HH:MM:SS instead of raw ms
  f.println("idx,iso_time,uptime,message");

  int count = logWrapped ? LOG_SIZE : logIndex;
  for (int i = 0; i < count; ++i) {
    int idx = logWrapped ? (logIndex + i) % LOG_SIZE : i;

    String msg  = logMessages[idx];
    String tstr = logTimeStr[idx];
    msg.replace("\"", "'");
    tstr.replace("\"", "'");

    String uptimeStr = formatDurationDDHHMMSS((uint64_t)logTimes[idx]);

    f.print(i);
    f.print(",");
    f.print("\""); f.print(tstr); f.print("\"");
    f.print(",");
    f.print("\""); f.print(uptimeStr); f.print("\"");
    f.print(",");
    f.print("\""); f.print(msg); f.print("\"");
    f.println();
  }

  f.close();

  lastLogSaveMs = millis();
  logDirty      = false;
}

void addLog(const String &msg) {
  uint32_t now  = millis();
  String   tstr = getCurrentTimeString();

  logTimes[logIndex]    = now;
  logMessages[logIndex] = msg;
  logTimeStr[logIndex]  = tstr;

  logIndex = (logIndex + 1) % LOG_SIZE;
  if (logIndex == 0) logWrapped = true;

  lastLogActivityMs = now;
  logDirty          = true;

  Serial.printf("LOG [%s | %u ms] %s\n", tstr.c_str(), now, msg.c_str());
}

// Save main log to SPIFFS after 1 minute of log inactivity
void checkLogIdleSave() {
  if (!logDirty) return;
  uint32_t now = millis();
  if (now - lastLogActivityMs >= 60000UL) {  // 60s idle
    writeCsvLog();
  }
}

// ============================================================================
// Relay Core Logic
// ============================================================================
void allRelaysOffImmediate() {
  digitalWrite(RELAY1_PIN, RELAY_INACTIVE_LEVEL);
  digitalWrite(RELAY2_PIN, RELAY_INACTIVE_LEVEL);
  digitalWrite(RELAY3_PIN, RELAY_INACTIVE_LEVEL);
  digitalWrite(RELAY4_PIN, RELAY_INACTIVE_LEVEL);
}

void setDirectionForward() {
  // Direction relays OFF (NC path) = Forward
  digitalWrite(RELAY3_PIN, RELAY_INACTIVE_LEVEL);
  digitalWrite(RELAY4_PIN, RELAY_INACTIVE_LEVEL);
}

void setDirectionReverse() {
  // Direction relays ON (NO path) = Reverse
  digitalWrite(RELAY3_PIN, RELAY_ACTIVE_LEVEL);
  digitalWrite(RELAY4_PIN, RELAY_ACTIVE_LEVEL);
}

void enableSupply() {
  digitalWrite(RELAY1_PIN, RELAY_ACTIVE_LEVEL);   // +5_OUT
  digitalWrite(RELAY2_PIN, RELAY_ACTIVE_LEVEL);   // GND_OUT
}

void disableSupply() {
  digitalWrite(RELAY1_PIN, RELAY_INACTIVE_LEVEL);
  digitalWrite(RELAY2_PIN, RELAY_INACTIVE_LEVEL);
}

bool relayIsOn(int pin) {
  return digitalRead(pin) == RELAY_ACTIVE_LEVEL;
}

// ============================================================================
// Solenoid Operation Logic with Sequencing
// ============================================================================
void stopPulseFailsafe() {
  // First: power relays OFF to remove coil current
  disableSupply();
  delay(relayDelayMs);  // sequencing tail delay

  // Then: direction relays OFF so everything is idle
  digitalWrite(RELAY3_PIN, RELAY_INACTIVE_LEVEL);
  digitalWrite(RELAY4_PIN, RELAY_INACTIVE_LEVEL);

  actionInProgress = false;
  addLog("Failsafe: power off then direction off.");
}

void startForwardPulse(const char* reason) {
  if (actionInProgress) {
    addLog(String("Forward pulse ignored (busy). Reason: ") + reason);
    return;
  }

  // Sequence: set direction first, wait, then enable power
  setDirectionForward();
  delay(relayDelayMs);      // lead delay before applying power
  enableSupply();

  actionInProgress = true;
  actionStartMs    = millis();

  lastActionText   = String("Forward Pulse (") + reason + ")";
  lastActionTimeMs = actionStartMs;

  addLog(lastActionText + " with relayDelayMs=" + String(relayDelayMs));
}

void startReversePulse(const char* reason) {
  if (actionInProgress) {
    addLog(String("Reverse pulse ignored (busy). Reason: ") + reason);
    return;
  }

  // Sequence: set direction first, wait, then enable power
  setDirectionReverse();
  delay(relayDelayMs);      // lead delay before applying power
  enableSupply();

  actionInProgress = true;
  actionStartMs    = millis();

  lastActionText   = String("Reverse Pulse (") + reason + ")";
  lastActionTimeMs = actionStartMs;

  addLog(lastActionText + " with relayDelayMs=" + String(relayDelayMs));
}

void updateSolenoidPulse() {
  if (!actionInProgress) return;

  // Only the power-on duration is counted
  if (millis() - actionStartMs >= solenoidPulseMs) {
    stopPulseFailsafe();
  }
}

// ============================================================================
// Trigger Debounce Logic (GPIO16)
// Tracks commanded logical state from external relay
// ============================================================================
void updateTriggerState() {
  int raw = digitalRead(PIN_TRIGGER);

  if (raw != lastRawReading) {
    lastChangeMs   = millis();
    lastRawReading = raw;
  }

  if ((millis() - lastChangeMs) >= debounceMs && raw != stableReading) {
    stableReading = raw;

    // Closed relay => LOW => ON state
    LogicalState newState = (stableReading == LOW) ? STATE_ON : STATE_OFF;

    if (newState != currentLogicalState) {
      // Update ON/OFF statistics
      recordStateTransition(newState);

      currentLogicalState = newState;

      // Log with stats
      logStateChangeWithStats("Trigger debounced", currentLogicalState);

      // State log entry for each logical state change
      addStateLog(currentLogicalState);
    }
  }
}

// ============================================================================
// JSON Config Handling (SPIFFS) /config.json
// ============================================================================
void saveConfigToSPIFFS() {
  if (!spiffsMounted) {
    addLog("Config save failed: SPIFFS not mounted.");
    return;
  }

  File f = SPIFFS.open(CONFIG_PATH, "w");
  if (!f) {
    addLog("Config save failed: cannot open /config.json for write.");
    return;
  }

  f.print("{\n");
  f.print("  \"solenoidPulseMs\": "); f.print(solenoidPulseMs); f.print(",\n");
  f.print("  \"debounceMs\": ");      f.print(debounceMs);      f.print(",\n");
  f.print("  \"autoTrigger\": ");     f.print(autoTrigger ? "true" : "false"); f.print(",\n");
  f.print("  \"relayDelayMs\": ");    f.print(relayDelayMs);    f.print("\n");
  f.print("}\n");
  f.close();

  logDirty = true;  // mark log dirty as config changed (we log this)
  addLog("Saved /config.json");
}

void loadConfigFromSPIFFS() {
  if (!spiffsMounted) {
    addLog("SPIFFS not mounted; cannot load config, using defaults.");
    solenoidPulseMs = DEFAULT_SOLENOID_PULSE_MS;
    debounceMs      = DEFAULT_DEBOUNCE_MS;
    autoTrigger     = DEFAULT_AUTO_TRIGGER;
    relayDelayMs    = DEFAULT_RELAY_DELAY_MS;
    return;
  }

  if (!SPIFFS.exists(CONFIG_PATH)) {
    addLog("No /config.json found; writing defaults.");
    solenoidPulseMs = DEFAULT_SOLENOID_PULSE_MS;
    debounceMs      = DEFAULT_DEBOUNCE_MS;
    autoTrigger     = DEFAULT_AUTO_TRIGGER;
    relayDelayMs    = DEFAULT_RELAY_DELAY_MS;
    saveConfigToSPIFFS();
    return;
  }

  File f = SPIFFS.open(CONFIG_PATH, "r");
  if (!f) {
    addLog("Failed to open /config.json; using defaults.");
    solenoidPulseMs = DEFAULT_SOLENOID_PULSE_MS;
    debounceMs      = DEFAULT_DEBOUNCE_MS;
    autoTrigger     = DEFAULT_AUTO_TRIGGER;
    relayDelayMs    = DEFAULT_RELAY_DELAY_MS;
    return;
  }

  String json = f.readString();
  f.close();

  json.replace("\r", "");
  json.replace("\n", "");

  auto getValue = [&](const char* key) -> String {
    int idx = json.indexOf(key);
    if (idx < 0) return "";
    idx = json.indexOf(":", idx);
    if (idx < 0) return "";
    int start = idx + 1;
    int end = json.indexOf(",", start);
    if (end < 0) end = json.indexOf("}", start);
    if (end < 0) return "";
    String v = json.substring(start, end);
    v.trim();
    return v;
  };

  String sPulse = getValue("solenoidPulseMs");
  String sDeb   = getValue("debounceMs");
  String sAuto  = getValue("autoTrigger");
  String sDelay = getValue("relayDelayMs");

  uint32_t newPulse = sPulse.toInt();
  uint32_t newDeb   = sDeb.toInt();
  bool     newAuto  = (sAuto == "true");
  uint32_t newDelay = sDelay.length() ? (uint32_t)sDelay.toInt() : DEFAULT_RELAY_DELAY_MS;

  if (newPulse < 100 || newPulse > 20000) newPulse = DEFAULT_SOLENOID_PULSE_MS;
  if (newDeb   < 5   || newDeb   > 2000 ) newDeb   = DEFAULT_DEBOUNCE_MS;
  if (newDelay > 1000) newDelay = 1000; // clamp

  solenoidPulseMs = newPulse;
  debounceMs      = newDeb;
  autoTrigger     = newAuto;
  relayDelayMs    = newDelay;

  addLog("Loaded /config.json: pulse=" + String(solenoidPulseMs)
         + " debounce=" + String(debounceMs)
         + " auto=" + String(autoTrigger ? "true" : "false")
         + " relayDelayMs=" + String(relayDelayMs));
}

// ============================================================================
// Web Utilities / Auth
// ============================================================================
String stateToString(LogicalState s) {
  return (s == STATE_ON) ? "ON" : "OFF";
}

String msToSec(uint32_t ms) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f", ms / 1000.0);
  return String(buf);
}

bool ensureAuth() {
  if (!server.authenticate(WEB_USER, WEB_PASS)) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

// ============================================================================
// /api/status  (JSON for dynamic UI)
// ============================================================================
void handleStatusApi() {
  if (!ensureAuth()) return;

  uint32_t now = millis();

  bool r1 = relayIsOn(RELAY1_PIN);
  bool r2 = relayIsOn(RELAY2_PIN);
  bool r3 = relayIsOn(RELAY3_PIN);
  bool r4 = relayIsOn(RELAY4_PIN);

  size_t flashSize  = ESP.getFlashChipSize();
  size_t sketchSize = ESP.getSketchSize();
  size_t freeSketch = ESP.getFreeSketchSpace();

  size_t spiffsTotal = 0;
  size_t spiffsUsed  = 0;
  if (spiffsMounted) {
    spiffsTotal = SPIFFS.totalBytes();
    spiffsUsed  = SPIFFS.usedBytes();
  }

  uint32_t heapFree    = ESP.getFreeHeap();
  String   currentTime = getCurrentTimeString();

  String lastActionSafe = lastActionText;
  lastActionSafe.replace("\"", "'");

  // WiFi stats
  String wifiSsid   = WiFi.SSID();
  long   wifiRssi   = WiFi.RSSI();
  int    wifiChan   = WiFi.channel();
  String wifiBssid  = WiFi.BSSIDstr();
  wifiSsid.replace("\"", "'");
  wifiBssid.replace("\"", "'");

  // Effective ON/OFF times
  uint64_t effOnMs, effOffMs;
  getEffectiveDurations(effOnMs, effOffMs);

  String json = "{";

  json += "\"fw\":\"" + String(FW_VERSION) + "\",";
  json += "\"time\":\"" + currentTime + "\",";
  json += "\"uptime_ms\":" + String(now) + ",";
  json += "\"trigger_closed\":" + String(stableReading == LOW ? "true" : "false") + ",";
  json += "\"logical\":\"" + stateToString(currentLogicalState) + "\",";
  json += "\"pulse_active\":" + String(actionInProgress ? "true" : "false") + ",";
  json += "\"auto_trigger\":" + String(autoTrigger ? "true" : "false") + ",";
  json += "\"last_action\":\"" + lastActionSafe + "\",";
  json += "\"last_action_age_ms\":" + String(now - lastActionTimeMs) + ",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";

  json += "\"solenoidPulseMs\":" + String(solenoidPulseMs) + ",";
  json += "\"debounceMs\":" + String(debounceMs) + ",";
  json += "\"relayDelayMs\":" + String(relayDelayMs) + ",";

  json += "\"relays\":{";
  json += "\"r1\":" + String(r1 ? "true" : "false") + ",";
  json += "\"r2\":" + String(r2 ? "true" : "false") + ",";
  json += "\"r3\":" + String(r3 ? "true" : "false") + ",";
  json += "\"r4\":" + String(r4 ? "true" : "false");
  json += "},";

  // Recent log entries
  json += "\"logs\":[";
  {
    int count = logWrapped ? LOG_SIZE : logIndex;
    int lastN = count > 10 ? 10 : count;
    for (int i = 0; i < lastN; ++i) {
      int logicalIndex = count - lastN + i;  // index in chronological order
      int idx = logWrapped ? (logIndex + logicalIndex) % LOG_SIZE : logicalIndex;

      String msg = logMessages[idx];
      String t   = logTimeStr[idx];
      msg.replace("\"", "'");
      t.replace("\"", "'");

      if (i > 0) json += ",";
      json += "{";
      json += "\"t\":\"" + t + "\",";
      json += "\"ms\":" + String(logTimes[idx]) + ",";
      json += "\"msg\":\"" + msg + "\"";
      json += "}";
    }
  }
  json += "],";

  json += "\"flash\":{";
  json += "\"chip\":" + String(flashSize) + ",";
  json += "\"sketch_used\":" + String(sketchSize) + ",";
  json += "\"sketch_free\":" + String(freeSketch);
  json += "},";

  json += "\"spiffs\":{";
  json += "\"mounted\":" + String(spiffsMounted ? "true" : "false") + ",";
  json += "\"total\":" + String(spiffsTotal) + ",";
  json += "\"used\":" + String(spiffsUsed);
  json += "},";

  // WiFi stats block
  json += "\"wifi\":{";
  json += "\"ssid\":\"" + wifiSsid + "\",";
  json += "\"rssi\":" + String(wifiRssi) + ",";
  json += "\"channel\":" + String(wifiChan) + ",";
  json += "\"bssid\":\"" + wifiBssid + "\"";
  json += "},";

  json += "\"heap_free\":" + String(heapFree) + ",";

  // ON/OFF stats
  json += "\"on_count\":" + String(onCount) + ",";
  json += "\"off_count\":" + String(offCount) + ",";
  json += "\"on_time_ms\":" + String((unsigned long)effOnMs) + ",";
  json += "\"off_time_ms\":" + String((unsigned long)effOffMs) + ",";
  json += "\"on_time_str\":\"" + formatDurationDDHHMMSS(effOnMs) + "\",";
  json += "\"off_time_str\":\"" + formatDurationDDHHMMSS(effOffMs) + "\"";

  json += "}";

  server.send(200, "application/json", json);
}

// ============================================================================
// Main Web UI Page
// ============================================================================
void handleRoot() {
  if (!ensureAuth()) return;

  String html =
R"=====(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Solenoid Controller</title>
<style>
body{font-family:Arial;background:#111;color:#eee;padding:20px;}
.card{background:#222;padding:15px;border-radius:8px;margin-bottom:12px;}
.status{font-size:1em;margin-bottom:6px;}
.label{color:#aaa;}
table{width:100%;border-collapse:collapse;margin-top:10px;}
th,td{border:1px solid #333;padding:5px;}
th{background:#111;}
button{padding:8px 16px;margin:6px;font-size:1em;border-radius:6px;border:none;cursor:pointer;}
.btn-fwd{background:#2e8b57;color:#fff;}
.btn-rev{background:#b22222;color:#fff;}
.btn-save{background:#1e90ff;color:#fff;}
input,select{padding:6px;background:#000;color:#eee;border:1px solid #555;border-radius:5px;}
a{color:#4ea3ff;}
.mono{font-family:Consolas,monospace;}
</style>
</head><body>
<h2>ESP32 Latching Solenoid Controller</h2>
<div class="status"><span class="label">Firmware:</span> <span id="fw"></span></div>
<div class="status"><span class="label">Clock (UTC):</span> <span id="clock"></span></div>

<div class="card">
  <div class="status"><span class="label">Trigger:</span> <span id="triggerState"></span></div>
  <div class="status"><span class="label">Logical State:</span> <span id="logicalState"></span></div>
  <div class="status"><span class="label">Pulse Active:</span> <span id="pulseActive"></span></div>
  <div class="status"><span class="label">Auto Trigger:</span> <span id="autoTrigger"></span></div>
  <div class="status"><span class="label">Uptime:</span> <span id="uptime"></span> s</div>
</div>

<div class="card">
  <h3>State Statistics</h3>
  <div class="status"><span class="label">Times Commanded ON:</span> <span id="onCount"></span></div>
  <div class="status"><span class="label">Times Commanded OFF:</span> <span id="offCount"></span></div>
  <div class="status"><span class="label">Total Time ON:</span> <span id="onTimeStr"></span></div>
  <div class="status"><span class="label">Total Time OFF:</span> <span id="offTimeStr"></span></div>
</div>

<div class="card">
  <h3>Relay State</h3>
  <table>
    <tr><th>Relay</th><th>GPIO</th><th>Role</th><th>State</th></tr>
    <tr><td>R1</td><td>32</td><td>+5V Switch</td><td id="r1State"></td></tr>
    <tr><td>R2</td><td>33</td><td>GND Switch</td><td id="r2State"></td></tr>
    <tr><td>R3</td><td>25</td><td>Dir A</td><td id="r3State"></td></tr>
    <tr><td>R4</td><td>26</td><td>Dir B</td><td id="r4State"></td></tr>
  </table>
</div>

<div class="card">
  <h3>Manual Control</h3>
  <form action="/cmd" method="GET">
    <button class="btn-fwd" name="do" value="forward">Pulse OPEN (Forward)</button>
    <button class="btn-rev" name="do" value="reverse">Pulse CLOSE (Reverse)</button>
  </form>
  <div class="status"><span class="label">Note:</span> manual pulses are ignored while a pulse is active.</div>
</div>

<div class="card">
  <h3>Configuration</h3>
  <form action="/config" method="GET">
    Pulse Length (ms):<br>
    <input type="number" name="pulse_ms" id="cfgPulse" min="100" max="20000"><br><br>
    Debounce (ms):<br>
    <input type="number" name="debounce_ms" id="cfgDebounce" min="5" max="2000"><br><br>
    Relay sequence delay (ms):<br>
    <input type="number" name="relay_delay" id="cfgRelayDelay" min="0" max="1000"><br><br>
    Auto Trigger:<br>
    <select name="auto" id="cfgAuto">
      <option value="1">Enabled</option>
      <option value="0">Disabled</option>
    </select><br><br>
    <button class="btn-save" type="submit">Save</button>
  </form>
</div>

<div class="card">
  <h3>Last Action</h3>
  <div class="status mono" id="lastAction"></div>
  <div class="status"><span class="label">Seconds Since:</span> <span id="lastActionAge"></span></div>
</div>

<div class="card">
  <h3>Recent Log (last 10 entries)</h3>
  <table>
    <tr><th>#</th><th>Time</th><th>Millis</th><th>Message</th></tr>
    <tbody id="logBody"></tbody>
  </table>
</div>

<div class="card">
  <h3>Flash / Filesystem</h3>
  <div class="status mono">
    Flash chip: <span id="flashChip"></span> bytes<br>
    Sketch used: <span id="flashSketchUsed"></span> bytes<br>
    Sketch free (OTA space): <span id="flashSketchFree"></span> bytes<br>
    SPIFFS mounted: <span id="spiffsMounted"></span><br>
    SPIFFS used: <span id="spiffsUsed"></span> / <span id="spiffsTotal"></span> bytes<br>
    Free heap: <span id="heapFree"></span> bytes
  </div>
</div>

<div class="card">
  <h3>Network / Tools</h3>
  <div class="status"><span class="label">IP:</span> <span id="ipAddr"></span></div>
  <div class="status"><span class="label">WiFi SSID:</span> <span id="wifiSsid"></span></div>
  <div class="status"><span class="label">RSSI:</span> <span id="wifiRssi"></span> dBm</div>
  <div class="status"><span class="label">Channel:</span> <span id="wifiChannel"></span></div>
  <div class="status"><span class="label">BSSID:</span> <span id="wifiBssid"></span></div>
  <div class="status"><a href="/debug">Debug Log</a></div>
  <div class="status"><a href="/update">OTA Update (upload .bin)</a></div>
  <div class="status"><a href="/files">File Browser (SPIFFS)</a></div>
</div>

<script>
let configLoaded = false;  // only populate form fields once

async function fetchStatus() {
  try {
    const res = await fetch('/api/status', {cache: 'no-cache'});
    if (!res.ok) return;
    const d = await res.json();

    const ge = id => document.getElementById(id);

    if (ge('fw')) ge('fw').textContent = d.fw || '';
    if (ge('clock')) ge('clock').textContent = d.time || '';

    if (ge('triggerState')) ge('triggerState').textContent =
      d.trigger_closed ? 'CLOSED (ON)' : 'OPEN (OFF)';

    if (ge('logicalState')) ge('logicalState').textContent = d.logical || '';

    if (ge('pulseActive')) ge('pulseActive').textContent =
      d.pulse_active ? 'YES' : 'NO';

    if (ge('autoTrigger')) ge('autoTrigger').textContent =
      d.auto_trigger ? 'ENABLED' : 'DISABLED';

    if (ge('uptime')) ge('uptime').textContent = (d.uptime_ms / 1000).toFixed(1);

    if (ge('lastAction')) ge('lastAction').textContent = d.last_action || 'None';
    if (ge('lastActionAge')) ge('lastActionAge').textContent =
      (d.last_action_age_ms / 1000).toFixed(1);

    if (ge('ipAddr')) ge('ipAddr').textContent = d.ip || '';

    // State statistics
    if (ge('onCount') && d.on_count !== undefined) {
      ge('onCount').textContent = d.on_count;
    }
    if (ge('offCount') && d.off_count !== undefined) {
      ge('offCount').textContent = d.off_count;
    }
    if (ge('onTimeStr') && d.on_time_str !== undefined) {
      ge('onTimeStr').textContent = d.on_time_str;
    }
    if (ge('offTimeStr') && d.off_time_str !== undefined) {
      ge('offTimeStr').textContent = d.off_time_str;
    }

    if (d.wifi) {
      if (ge('wifiSsid')) ge('wifiSsid').textContent = d.wifi.ssid || '';
      if (ge('wifiRssi') && d.wifi.rssi !== undefined) ge('wifiRssi').textContent = d.wifi.rssi;
      if (ge('wifiChannel') && d.wifi.channel !== undefined) ge('wifiChannel').textContent = d.wifi.channel;
      if (ge('wifiBssid')) ge('wifiBssid').textContent = d.wifi.bssid || '';
    }

    if (d.relays) {
      if (ge('r1State')) ge('r1State').textContent = d.relays.r1 ? 'ON' : 'OFF';
      if (ge('r2State')) ge('r2State').textContent = d.relays.r2 ? 'ON' : 'OFF';
      if (ge('r3State')) ge('r3State').textContent = d.relays.r3 ? 'ON' : 'OFF';
      if (ge('r4State')) ge('r4State').textContent = d.relays.r4 ? 'ON' : 'OFF';
    }

    if (d.flash) {
      if (ge('flashChip')) ge('flashChip').textContent = d.flash.chip;
      if (ge('flashSketchUsed')) ge('flashSketchUsed').textContent = d.flash.sketch_used;
      if (ge('flashSketchFree')) ge('flashSketchFree').textContent = d.flash.sketch_free;
    }

    if (d.spiffs) {
      if (ge('spiffsMounted')) ge('spiffsMounted').textContent = d.spiffs.mounted ? 'YES' : 'NO';
      if (ge('spiffsTotal')) ge('spiffsTotal').textContent = d.spiffs.total;
      if (ge('spiffsUsed')) ge('spiffsUsed').textContent = d.spiffs.used;
    }

    if (ge('heapFree')) ge('heapFree').textContent = d.heap_free;

    // Populate form fields only once (initial load), so user edits aren't overwritten
    if (!configLoaded) {
      if (ge('cfgPulse') && d.solenoidPulseMs !== undefined) {
        ge('cfgPulse').value = d.solenoidPulseMs;
      }
      if (ge('cfgDebounce') && d.debounceMs !== undefined) {
        ge('cfgDebounce').value = d.debounceMs;
      }
      if (ge('cfgRelayDelay') && d.relayDelayMs !== undefined) {
        ge('cfgRelayDelay').value = d.relayDelayMs;
      }
      if (ge('cfgAuto') && d.auto_trigger !== undefined) {
        ge('cfgAuto').value = d.auto_trigger ? '1' : '0';
      }
      configLoaded = true;
    }

    // Recent logs table
    if (Array.isArray(d.logs)) {
      const tbody = ge('logBody');
      if (tbody) {
        tbody.innerHTML = '';
        d.logs.forEach((entry, idx) => {
          const tr = document.createElement('tr');

          const tdIdx = document.createElement('td');
          tdIdx.textContent = idx;

          const tdTime = document.createElement('td');
          tdTime.textContent = entry.t || '';

          const tdMs = document.createElement('td');
          tdMs.textContent = (entry.ms !== undefined) ? entry.ms : '';

          const tdMsg = document.createElement('td');
          tdMsg.textContent = entry.msg || '';

          tr.appendChild(tdIdx);
          tr.appendChild(tdTime);
          tr.appendChild(tdMs);
          tr.appendChild(tdMsg);
          tbody.appendChild(tr);
        });
      }
    }

  } catch (e) {
    console.log('Status fetch error', e);
  }
}

setInterval(fetchStatus, 1000);
fetchStatus();
</script>

</body></html>
)=====";

  server.send(200, "text/html", html);
}

// ============================================================================
// Debug Log Page
// ============================================================================
void handleDebug() {
  if (!ensureAuth()) return;

  String html =
R"=====(
<!DOCTYPE html>
<html><head>
<title>Debug Log</title>
<meta http-equiv="refresh" content="3">
<style>
body{background:#000;color:#0f0;font-family:Consolas;font-size:14px;padding:20px;}
table{width:100%;border-collapse:collapse;}
td,th{border:1px solid #0a0;padding:4px;}
th{background:#030;}
a{color:#4ea3ff;}
</style>
</head><body>
<h2>Debug Log</h2>
<a href="/">Back</a><br><br>
<table><tr><th>#</th><th>Time</th><th>Millis</th><th>Message</th></tr>
)=====";

  int count = logWrapped ? LOG_SIZE : logIndex;
  for (int i = 0; i < count; ++i) {
    int idx = logWrapped ? (logIndex + i) % LOG_SIZE : i;
    html += "<tr><td>" + String(i) + "</td><td>" +
            logTimeStr[idx] + "</td><td>" +
            String(logTimes[idx]) + "</td><td>" +
            logMessages[idx] + "</td></tr>";
  }

  html += "</table></body></html>";

  server.send(200, "text/html", html);
}

// ============================================================================
// Manual Control Endpoint
// ============================================================================
void handleCmd() {
  if (!ensureAuth()) return;

  if (!server.hasArg("do")) {
    server.send(400, "text/plain", "Missing ?do= parameter");
    return;
  }

  if (actionInProgress) {
    addLog("Manual command ignored: pulse already active.");
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
    return;
  }

  String action = server.arg("do");

  if (action == "forward") {
    startForwardPulse("manual");
  } else if (action == "reverse") {
    startReversePulse("manual");
  } else {
    server.send(400, "text/plain", "Invalid command");
    return;
  }

  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}

// ============================================================================
// Config Save Handler
// ============================================================================
void handleConfig() {
  if (!ensureAuth()) return;

  if (server.hasArg("pulse_ms")) {
    uint32_t v = (uint32_t) server.arg("pulse_ms").toInt();
    solenoidPulseMs = constrain(v, (uint32_t)100, (uint32_t)20000);
  }

  if (server.hasArg("debounce_ms")) {
    uint32_t v = (uint32_t) server.arg("debounce_ms").toInt();
    debounceMs = constrain(v, (uint32_t)5, (uint32_t)2000);
  }

  if (server.hasArg("relay_delay")) {
    uint32_t v = (uint32_t) server.arg("relay_delay").toInt();
    relayDelayMs = constrain(v, (uint32_t)0, (uint32_t)1000);
  }

  if (server.hasArg("auto")) {
    autoTrigger = (server.arg("auto") == "1");
  }

  addLog("Config updated via web; saving to SPIFFS.");
  saveConfigToSPIFFS();

  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}

// ============================================================================
// Web-based OTA Upload Page (/update)
// ============================================================================
void handleUpdatePage() {
  if (!ensureAuth()) return;

  String html =
R"=====(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OTA Update</title>
<style>
body{font-family:Arial;background:#111;color:#eee;padding:20px;}
.card{background:#222;padding:15px;border-radius:8px;margin-bottom:12px;}
input[type=file]{margin:10px 0;color:#eee;}
button{padding:8px 16px;border:none;border-radius:6px;background:#1e90ff;color:#fff;cursor:pointer;}
a{color:#4ea3ff;}
</style>
</head>
<body>
<h2>OTA Firmware Update</h2>
<div class="card">
<form method="POST" action="/update" enctype="multipart/form-data">
<p>Select compiled firmware (.bin) and upload:</p>
<input type="file" name="firmware">
<br>
<button type="submit">Upload & Flash</button>
</form>
<p><a href="/">Back to main page</a></p>
</div>
</body>
</html>
)=====";

  server.send(200, "text/html", html);
}

void handleUpdateUpload() {
  if (!ensureAuth()) return;

  HTTPUpload &upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    addLog(String("OTA HTTP upload start: ") + upload.filename);
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
      addLog("Update.begin() failed");
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
      addLog("Update.write() failed");
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      addLog(String("OTA HTTP update success, size: ") + upload.totalSize);
    } else {
      Update.printError(Serial);
      addLog("Update.end() failed");
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.end();
    addLog("OTA HTTP update aborted");
  }
}

void handleUpdateDone() {
  if (!ensureAuth()) return;

  bool ok = !Update.hasError();

  if (ok) {
    // Success page with auto-redirect to main page
    String html =
R"=====(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OTA Update</title>
<style>
body{font-family:Arial;background:#111;color:#eee;padding:20px;text-align:center;}
.card{background:#222;padding:20px;border-radius:8px;max-width:400px;margin:40px auto;}
a{color:#4ea3ff;}
</style>
<script>
setTimeout(function(){
  window.location.href = "/";
}, 8000);
</script>
</head>
<body>
<div class="card">
  <h2>Update successful</h2>
  <p>Rebooting controller now...</p>
  <p>You will be returned to the main page automatically.</p>
  <p><a href="/">Go to main page now</a></p>
</div>
</body>
</html>
)=====";
    server.send(200, "text/html", html);
    addLog("Rebooting after successful HTTP OTA...");
    delay(1500);
    ESP.restart();
  } else {
    // Failure page with link back
    String html =
R"=====(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OTA Update Failed</title>
<style>
body{font-family:Arial;background:#111;color:#eee;padding:20px;text-align:center;}
.card{background:#222;padding:20px;border-radius:8px;max-width:400px;margin:40px auto;}
a{color:#4ea3ff;}
</style>
</head>
<body>
<div class="card">
  <h2>Update failed</h2>
  <p>See serial log for details.</p>
  <p><a href="/">Back to main page</a></p>
</div>
</body>
</html>
)=====";
    server.send(200, "text/html", html);
    addLog("HTTP OTA failed, staying on current firmware.");
  }
}

// ============================================================================
// SPIFFS File Browser + Upload
// ============================================================================
void handleFiles() {
  if (!ensureAuth()) return;

  if (!spiffsMounted) {
    server.send(500, "text/plain", "SPIFFS not mounted");
    return;
  }

  File root = SPIFFS.open("/");
  if (!root || !root.isDirectory()) {
    server.send(500, "text/plain", "SPIFFS root invalid");
    return;
  }

  String html =
R"=====(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SPIFFS File Browser</title>
<style>
body{font-family:Arial;background:#111;color:#eee;padding:20px;}
table{width:100%;border-collapse:collapse;margin-top:10px;}
th,td{border:1px solid #333;padding:5px;}
th{background:#222;}
a{color:#4ea3ff;}
.card{background:#222;padding:15px;border-radius:8px;margin-bottom:12px;}
button{padding:8px 16px;border:none;border-radius:6px;background:#1e90ff;color:#fff;cursor:pointer;}
input[type=file]{margin:10px 0;color:#eee;}
</style>
</head>
<body>
<h2>SPIFFS File Browser</h2>
)=====";

  html += "<p>Firmware: ";
  html += FW_VERSION;
  html += "</p><p><a href=\"/\">Back to main page</a></p>";
  html += "<p>CSV log file: <code>/log.csv</code></p>";
  html += "<p>State log file: <code>/state_log.csv</code></p>";

  // Upload form
  html +=
R"=====(
<div class="card">
  <h3>Upload File to SPIFFS</h3>
  <form method="POST" action="/upload" enctype="multipart/form-data">
    <input type="file" name="file">
    <br>
    <button type="submit">Upload</button>
  </form>
</div>
)=====";

  html += "<table><tr><th>Name</th><th>Size (bytes)</th><th>Action</th></tr>";

  File file = root.openNextFile();
  while (file) {
    String name = String(file.name());
    size_t size = file.size();

    html += "<tr><td>" + name + "</td><td>" + String(size) + "</td><td>";
    html += "<a href=\"/file?name=" + name + "\">Download</a>";
    html += "</td></tr>";

    file = root.openNextFile();
  }

  html += "</table></body></html>";

  server.send(200, "text/html", html);
}

void handleFileDownload() {
  if (!ensureAuth()) return;

  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing ?name parameter");
    return;
  }

  String path = server.arg("name");
  if (!path.startsWith("/")) path = "/" + path;

  if (!spiffsMounted) {
    server.send(500, "text/plain", "SPIFFS not mounted");
    return;
  }

  File file = SPIFFS.open(path, "r");
  if (!file || file.isDirectory()) {
    server.send(404, "text/plain", "File not found");
    return;
  }

  String contentType = "application/octet-stream";
  server.streamFile(file, contentType);
  file.close();
}

// Handle raw upload data (multipart)
void handleFileUploadData() {
  if (!ensureAuth()) return;

  if (!spiffsMounted) {
    return;
  }

  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;

    if (SPIFFS.exists(filename)) {
      SPIFFS.remove(filename);
    }

    uploadFile = SPIFFS.open(filename, "w");
    addLog(String("SPIFFS upload start: ") + filename);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) {
      uploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
      addLog(String("SPIFFS upload done, size=") + upload.totalSize);
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (uploadFile) {
      uploadFile.close();
    }
    addLog("SPIFFS upload aborted.");
  }
}

void handleFileUploadDone() {
  if (!ensureAuth()) return;
  server.sendHeader("Location", "/files");
  server.send(303, "text/plain", "");
}

// ============================================================================
// WiFi Setup & Reconnect + NTP
// ============================================================================
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  addLog("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
  }

  addLog(String("WiFi connected, IP: ") + WiFi.localIP().toString());

  configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
  addLog("NTP configTime() called; waiting for sync...");

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5000)) {
    addLog(String("Time synced: ") + getCurrentTimeString());
  } else {
    addLog("NTP sync failed (timeout).");
  }
}

void checkWiFiReconnect() {
  if (millis() - lastWifiCheckMs < WIFI_CHECK_INTERVAL_MS) return;
  lastWifiCheckMs = millis();

  if (WiFi.status() != WL_CONNECTED) {
    addLog("WiFi lost, attempting reconnect...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

// ============================================================================
// Startup commanded state (30s after reboot)
//
// Compare last logged state with current commanded state from GPIO.
// Only if they are equal AND the last state log is less than 5 minutes old,
// then apply that state with a pulse.
// ============================================================================
void checkStartupCommand() {
  if (startupCommandDone) return;

  uint32_t nowMs = millis();
  if (nowMs < startupCommandAtMs) return;
  if (actionInProgress) return;  // wait until any pulse is done

  LogicalState prevLogical = currentLogicalState;

  // Current commanded state from GPIO
  int raw = digitalRead(PIN_TRIGGER);
  LogicalState cmdState = (raw == LOW ? STATE_ON : STATE_OFF);

  // Last logged state
  LogicalState lastState;
  time_t       lastEpoch;
  bool hasLast = getLastStateEntry(lastState, lastEpoch);

  time_t startupEpoch = time(nullptr);

  bool doCommand = false;

  if (hasLast && lastEpoch > 0 && startupEpoch > 0) {
    long diff = (long)startupEpoch - (long)lastEpoch;
    if (diff < 0) diff = -diff;  // just in case
    // less than 5 minutes (300 seconds) AND state matches
    if (diff <= 300 && lastState == cmdState) {
      doCommand = true;
    } else {
      addLog("Startup 30s: last state mismatch or too old; skipping command.");
    }
  } else {
    addLog("Startup 30s: no valid state log; skipping command.");
  }

  if (!doCommand) {
    startupCommandDone = true;
    return;
  }

  // Apply the commanded state
  if (cmdState == STATE_ON) {
    startForwardPulse("startup_30s");
  } else {
    startReversePulse("startup_30s");
  }

  currentLogicalState = cmdState;
  lastHandledState    = cmdState;  // so autoTrigger doesn't re-fire
  startupCommandDone  = true;

  // Update stats if this actually changed the logical state
  if (cmdState != prevLogical) {
    recordStateTransition(cmdState);
    logStateChangeWithStats("Startup 30s command applied from state log", cmdState);
  } else {
    addLog("Startup 30s command applied from state log: state=" + stateToString(cmdState));
  }

  addStateLog(cmdState);
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Initialize idle/log timers
  uint32_t now = millis();
  lastLogActivityMs = now;
  lastLogSaveMs     = now;
  logDirty          = false;

  addLog(String("Booting: ") + FW_VERSION);

  // Failsafe startup: everything off
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  allRelaysOffImmediate();

  pinMode(PIN_TRIGGER, INPUT_PULLUP);

  // Mount SPIFFS
  spiffsMounted = SPIFFS.begin(true);
  addLog(spiffsMounted ? "SPIFFS mounted." : "SPIFFS mount failed.");

  // Load previous logs into RAM buffer (if present)
  loadCsvLog();
  addLog("Loaded previous debug log from CSV.");

  loadStateLog();
  addLog("Loaded state log from CSV.");

  // Load persisted config
  loadConfigFromSPIFFS();

  // Initial trigger state (at boot)
  stableReading       = lastRawReading = digitalRead(PIN_TRIGGER);
  currentLogicalState = (stableReading == LOW ? STATE_ON : STATE_OFF);
  lastHandledState    = currentLogicalState;

  // Initialize stats with initial logical state
  initStateStats(currentLogicalState);

  addLog(String("Initial logical state: ") + stateToString(currentLogicalState));

  // Schedule the 30s commanded-state action
  startupCommandAtMs = millis() + STARTUP_COMMAND_DELAY_MS;
  startupCommandDone = false;

  setupWiFi();

  // Routes
  server.on("/",           handleRoot);
  server.on("/api/status", handleStatusApi);
  server.on("/debug",      handleDebug);
  server.on("/cmd",        handleCmd);
  server.on("/config",     handleConfig);

  // OTA upload
  server.on("/update", HTTP_GET,  handleUpdatePage);
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);

  // SPIFFS file browser & download
  server.on("/files", HTTP_GET, handleFiles);
  server.on("/file",  HTTP_GET, handleFileDownload);

  // SPIFFS upload
  server.on("/upload", HTTP_POST, handleFileUploadDone, handleFileUploadData);

  server.begin();
  addLog("HTTP server started");
}

// ============================================================================
// Loop
// ============================================================================
void loop() {
  server.handleClient();
  checkWiFiReconnect();

  updateTriggerState();
  updateSolenoidPulse();
  checkStartupCommand();  // 30s post-boot commanded-state pulse with log check
  checkLogIdleSave();     // save main log to SPIFFS after 1 minute idle

  // Auto trigger ON/OFF edges when enabled
  if (!actionInProgress && autoTrigger && currentLogicalState != lastHandledState) {
    if (currentLogicalState == STATE_ON) {
      startForwardPulse("auto");
    } else {
      startReversePulse("auto");
    }
    lastHandledState = currentLogicalState;
  }
}
