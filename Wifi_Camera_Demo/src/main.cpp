#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include "esp_camera.h"

const char* WIFI_SSID = "wifi_ssid";
const char* WIFI_PASSWORD = "wifi_password";

WebServer server(80);

constexpr int LED_PIN = 48;
constexpr int LED_COUNT = 1;
Adafruit_NeoPixel statusLed(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

bool cameraPsramDetected = false;

void setStatusLed(uint8_t red, uint8_t green, uint8_t blue) {
  statusLed.setPixelColor(0, statusLed.Color(red, green, blue));
  statusLed.show();
}

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15

#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5

#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11

#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

void connectToWifi() {
  Serial.println();
  Serial.println("[WiFi] Starting station mode...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("[WiFi] Connecting to SSID: %s\n", WIFI_SSID);

  uint32_t startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    if (millis() - startAttemptTime > 30000) {
      Serial.println();
      Serial.println("[WiFi] Connection timeout. Retrying...");
      WiFi.disconnect();
      delay(500);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      startAttemptTime = millis();
    }
  }

  Serial.println();
  Serial.println("[WiFi] Connected!");
  Serial.print("[WiFi] IP address: ");
  Serial.println(WiFi.localIP());
}

bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;

  const bool hasPsram = psramFound();
  cameraPsramDetected = hasPsram;
  Serial.printf("[Camera] PSRAM found: %s\n", hasPsram ? "yes" : "no");

  if (hasPsram) {
    config.frame_size = FRAMESIZE_XGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 16;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[Camera] Init failed (first attempt) with error 0x%x\n", err);
    Serial.println("[Camera] Retrying with reduced frame buffers...");

    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 18;
    config.fb_count = 1;
    config.fb_location = hasPsram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

    err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("[Camera] Init failed (fallback) with error 0x%x\n", err);
      return false;
    }
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    sensor->set_vflip(sensor, 1);
  }

  Serial.println("[Camera] Camera initialized");
  return true;
}

void handleRoot() {
  const char* html =
      "<!doctype html><html><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>ESP32 Camera Live Feed</title></head>"
      "<body style='font-family:Arial;text-align:center;margin:0;padding:20px;'>"
      "<h2>ESP32 Camera Live Feed</h2>"
      "<img src='/stream' style='width:100%;max-width:1024px;height:auto;border:1px solid #ccc;'>"
      "</body></html>";

  server.send(200, "text/html", html);
}

void handleJpg() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    server.send(503, "text/plain", "Camera capture failed");
    return;
  }

  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Length", String(fb->len));
  server.send(200);
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleStream() {
  WiFiClient client = server.client();

  String response =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Cache-Control: no-cache\r\n"
      "Connection: close\r\n\r\n";
  client.print(response);

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[Camera] Frame capture failed");
      break;
    }

    client.print("--frame\r\n");
    client.print("Content-Type: image/jpeg\r\n");
    client.print("Content-Length: " + String(fb->len) + "\r\n\r\n");
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    esp_camera_fb_return(fb);
    delay(30);
  }
}

void setup() {
  Serial.begin(115200);

  statusLed.begin();
  statusLed.setBrightness(50);
  setStatusLed(255, 0, 0);

  delay(5000);
  Serial.println();
  Serial.println("[Boot] ESP32 Camera starting...");

  if (!initCamera()) {
    Serial.println("[Boot] Camera init failed. Halting.");
    while (true) {
      delay(1000);
    }
  }

  connectToWifi();

  if (WiFi.status() == WL_CONNECTED && WiFi.localIP()) {
    if (cameraPsramDetected) {
      setStatusLed(0, 255, 0);
    } else {
      setStatusLed(0, 0, 255);
    }
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/jpg", HTTP_GET, handleJpg);
  server.on("/stream", HTTP_GET, handleStream);

  server.begin();
  Serial.println("[Web] Server started");
  Serial.print("[Web] Open in browser: http://");
  Serial.println(WiFi.localIP());
}

void loop() {
  server.handleClient();
}
