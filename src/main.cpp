/*
 * BTL IoT – Giám sát ánh sáng (LDR) + chuyển động (PIR) + điều khiển đèn (relay)
 * ESP32 → Firebase Realtime Database (REST API qua HTTPS)
 *
 * Board: LuatOS CORE-ESP32-C3 (chip ESP32-C3)
 * Đấu nối:
 *   LDR (LM393):  VCC→3.3V, GND→GND, AO→GPIO0  (DO không dùng)
 *   PIR HW-416A:  VCC→5V,   GND→GND, OUT→GPIO3
 *   Relay CW-025: DC+→5V,   DC-→GND, IN→GPIO2, jumper ở H (kích mức cao)
 *                 COM/NO nối mạch LED, nên thêm điện trở 220Ω nối tiếp LED
 *
 * Arduino IDE: Board "ESP32C3 Dev Module", Flash Mode "DIO",
 *              USB CDC On Boot "Enabled" (để xem Serial Monitor qua cổng USB-C).
 * Không cần thư viện ngoài.
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

// ======================= CẤU HÌNH – SỬA Ở ĐÂY =======================
const char* WIFI_SSID     = "Wiinvent tang 3";
const char* WIFI_PASS     = "Wiinvent@123";
// URL Realtime Database, KHÔNG có dấu "/" ở cuối
const char* FIREBASE_URL  = "https://iot-assignment-7e7b3-default-rtdb.asia-southeast1.firebasedatabase.app";
// Để trống nếu CSDL ở test mode; nếu dùng database secret thì dán vào đây
const char* FIREBASE_AUTH = "";
const char* DEVICE_ID     = "esp32_01";
// ===================================================================

// Chân kết nối
const int PIN_LDR_AO = 0;      // ADC1_CH0 của ESP32-C3
const int PIN_LDR_DO = -1;     // -1 = không nối chân DO
const int PIN_PIR    = 3;
const int PIN_RELAY  = 2;
const int PIN_STATUS = 12;     // LED D4 trên board CORE-ESP32-C3: sáng khi có WiFi

// Relay kích mức cao (jumper ở H). Nếu chuyển jumper sang L thì đổi thành true.
const bool RELAY_ACTIVE_LOW = false;
// Module LDR thường cho AO cao khi TỐI. Nếu thấy độ sáng bị ngược, đổi thành false.
const bool LDR_HIGH_MEANS_DARK = true;

// Thời gian
const unsigned long CYCLE_MS     = 2000;   // chu kỳ đọc + cập nhật /current
const unsigned long HISTORY_MS   = 10000;  // chu kỳ ghi /history
const unsigned long MOTION_HOLD  = 15000;  // giữ đèn sau lần cuối có chuyển động (auto)

// Trạng thái điều khiển (đọc từ /control)
String ctrlMode     = "auto";
bool   ctrlRelay    = false;
int    ctrlThreshold = 68;   // 68% ≈ ADC 1300, ngưỡng tối của code cũ

// Trạng thái đo
float  lightPct = 0;
int    lightRaw = 0;
int    lightDigital = 0;
int    motionNow = 0;
bool   relayOn = false;
unsigned long lastMotionMs = 0;
int    lastPir = 0;
int    eventsSinceHistory = 0;       // số lần PIR 0→1 kể từ lần ghi history trước
float  lightSum = 0; int lightCount = 0;   // để lấy trung bình cho history
int    motionAny = 0;                // có chuyển động trong khoảng history

unsigned long lastCycle = 0, lastHistory = 0;

WiFiClientSecure secureClient;

// ------------------------------------------------------------------
String dbUrl(const String& path) {
  String url = String(FIREBASE_URL) + "/devices/" + DEVICE_ID + path + ".json";
  if (strlen(FIREBASE_AUTH) > 0) url += "?auth=" + String(FIREBASE_AUTH);
  return url;
}

// method: "GET", "PUT", "POST", "PATCH". Trả về mã HTTP, body trả về qua tham số.
int firebaseRequest(const char* method, const String& path, const String& payload, String& response) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("[Firebase] Bo qua %s %s: chua ket noi WiFi\n", method, path.c_str());
    return -1;
  }
  HTTPClient http;
  http.setTimeout(4000);
  if (!http.begin(secureClient, dbUrl(path))) {
    Serial.printf("[Firebase] Khong khoi tao duoc request %s %s\n", method, path.c_str());
    return -2;
  }
  http.addHeader("Content-Type", "application/json");
  int code = http.sendRequest(method, payload);
  response = (code > 0) ? http.getString() : "";
  http.end();
  if (code < 0 || code >= 400) {
    Serial.printf("[Firebase] Loi %s %s: HTTP %d, response=%s\n",
                  method, path.c_str(), code, response.c_str());
  }
  return code;
}

// Lấy giá trị đơn giản từ chuỗi JSON phẳng, ví dụ {"mode":"auto","relay":true,"threshold":40}
String jsonValue(const String& json, const String& key) {
  int k = json.indexOf("\"" + key + "\"");
  if (k < 0) return "";
  int colon = json.indexOf(':', k);
  if (colon < 0) return "";
  int start = colon + 1;
  while (start < (int)json.length() && json[start] == ' ') start++;
  int end = start;
  if (json[start] == '"') {                 // chuỗi
    end = json.indexOf('"', start + 1);
    return json.substring(start + 1, end);
  }
  while (end < (int)json.length() && json[end] != ',' && json[end] != '}') end++;
  return json.substring(start, end);
}

// ------------------------------------------------------------------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(PIN_STATUS, HIGH);
    return;
  }
  Serial.printf("[WiFi] Dang ket noi toi \"%s\"", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Ket noi thanh cong, IP: %s\n",
                  WiFi.localIP().toString().c_str());
    digitalWrite(PIN_STATUS, HIGH);
  } else {
    Serial.printf("\n[WiFi] Ket noi that bai, status=%d; se thu lai.\n",
                  WiFi.status());
    digitalWrite(PIN_STATUS, LOW);
  }
}

void setRelay(bool on) {
  if (relayOn != on) {
    Serial.printf("[Relay] %s -> %s\n",
                  relayOn ? "ON" : "OFF", on ? "ON" : "OFF");
  }
  relayOn = on;
  bool level = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(PIN_RELAY, level ? HIGH : LOW);
}

// Lấy mẫu PIR liên tục để không bỏ sót cạnh lên
void samplePir() {
  int p = digitalRead(PIN_PIR);
  if (p == HIGH) { lastMotionMs = millis(); motionAny = 1; }
  if (p == HIGH && lastPir == LOW) {
    eventsSinceHistory++;
    Serial.printf("[PIR] Phat hien chuyen dong (su kien #%d)\n",
                  eventsSinceHistory);
  }
  lastPir = p;
  motionNow = p;
}

void readLight() {
  long sum = 0;
  for (int i = 0; i < 10; i++) { sum += analogRead(PIN_LDR_AO); delay(2); }
  lightRaw = sum / 10;
  float pct = lightRaw * 100.0 / 4095.0;
  lightPct = LDR_HIGH_MEANS_DARK ? (100.0 - pct) : pct;
  lightPct = round(lightPct * 10) / 10.0;
  lightDigital = (PIN_LDR_DO >= 0) ? digitalRead(PIN_LDR_DO) : (lightPct < ctrlThreshold ? 1 : 0);
  lightSum += lightPct; lightCount++;
  Serial.printf("[LDR] raw=%d, muc sang=%.1f%%, digital=%d, nguong=%d%%\n",
                lightRaw, lightPct, lightDigital, ctrlThreshold);
}

void readControl() {
  String resp;
  int code = firebaseRequest("GET", "/control", "", resp);
  if (code == 200 && resp != "null") {
    String m = jsonValue(resp, "mode");
    if (m == "auto" || m == "manual") ctrlMode = m;
    String r = jsonValue(resp, "relay");
    if (r.length()) ctrlRelay = (r == "true" || r == "1");
    String th = jsonValue(resp, "threshold");
    if (th.length()) ctrlThreshold = constrain(th.toInt(), 0, 100);
    Serial.printf("[Control] mode=%s, relay=%s, threshold=%d%%\n",
                  ctrlMode.c_str(), ctrlRelay ? "ON" : "OFF", ctrlThreshold);
  } else if (code == 200) {
    // Chưa có nút /control → tạo mặc định
    String tmp;
    int putCode = firebaseRequest("PUT", "/control",
                                  "{\"mode\":\"auto\",\"relay\":false,\"threshold\":68}", tmp);
    Serial.printf("[Control] Chua co /control, tao mac dinh -> HTTP %d\n", putCode);
  } else {
    Serial.printf("GET /control loi: %d\n", code);
  }
}

void decideRelay() {
  if (ctrlMode == "manual") {
    Serial.printf("[Decision] Manual: relay=%s\n", ctrlRelay ? "ON" : "OFF");
    setRelay(ctrlRelay);
  } else {
    bool recentMotion = (millis() - lastMotionMs) < MOTION_HOLD && lastMotionMs != 0;
    bool dark = lightPct < ctrlThreshold;
    Serial.printf("[Decision] Auto: motion_recent=%s, dark=%s -> relay=%s\n",
                  recentMotion ? "yes" : "no", dark ? "yes" : "no",
                  (recentMotion && dark) ? "ON" : "OFF");
    setRelay(recentMotion && dark);
  }
}

void sendCurrent() {
  String body = "{";
  body += "\"light\":" + String(lightPct, 1);
  body += ",\"lightRaw\":" + String(lightRaw);
  body += ",\"lightDigital\":" + String(lightDigital);
  body += ",\"motion\":" + String(motionNow);
  body += ",\"relay\":" + String(relayOn ? "true" : "false");
  body += ",\"mode\":\"" + ctrlMode + "\"";
  body += ",\"threshold\":" + String(ctrlThreshold);
  body += ",\"t\":{\".sv\":\"timestamp\"}}";
  String resp;
  int code = firebaseRequest("PUT", "/current", body, resp);
  Serial.printf("[current] light=%.1f%% raw=%d do=%d motion=%d relay=%s mode=%s -> HTTP %d\n",
                lightPct, lightRaw, lightDigital, motionNow, relayOn ? "ON" : "OFF", ctrlMode.c_str(), code);
}

void sendHistory() {
  float avg = lightCount ? lightSum / lightCount : lightPct;
  String body = "{";
  body += "\"light\":" + String(avg, 1);
  body += ",\"motion\":" + String(motionAny);
  body += ",\"events\":" + String(eventsSinceHistory);
  body += ",\"relay\":" + String(relayOn ? "true" : "false");
  body += ",\"t\":{\".sv\":\"timestamp\"}}";
  String resp;
  int code = firebaseRequest("POST", "/history", body, resp);
  Serial.printf("[history] avg=%.1f%% events=%d -> HTTP %d\n", avg, eventsSinceHistory, code);
  if (code == 200) {
    eventsSinceHistory = 0; motionAny = 0; lightSum = 0; lightCount = 0;
  }
}

// ------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  delay(500);
  Serial.println("\n[BOOT] Bat dau khoi dong ESP32-C3");
  pinMode(PIN_PIR, INPUT);
  if (PIN_LDR_DO >= 0) pinMode(PIN_LDR_DO, INPUT);
  pinMode(PIN_STATUS, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  setRelay(false);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_LDR_AO, ADC_11db);   // đo được dải 0–3.3V

  secureClient.setInsecure();   // bỏ kiểm tra chứng chỉ cho đơn giản (đủ cho bài tập)
  connectWiFi();
  Serial.println("[BOOT] Hoan tat cau hinh. PIR can khoang 30-60 giay de khoi dong.");
}

void loop() {
  samplePir();

  if (millis() - lastCycle >= CYCLE_MS) {
    lastCycle = millis();
    Serial.println("[CYCLE] Bat dau chu ky doc cam bien/cap nhat Firebase");
    connectWiFi();
    readLight();
    readControl();
    decideRelay();
    sendCurrent();
  }

  if (millis() - lastHistory >= HISTORY_MS) {
    lastHistory = millis();
    sendHistory();
  }

  delay(20);
}
