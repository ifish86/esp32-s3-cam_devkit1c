#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include "esp_camera.h"
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

const char* WIFI_SSID = "wifi_ssid";
const char* WIFI_PASSWORD = "wifi_password";

WebServer server(80);

constexpr int LED_PIN = 48;
constexpr int LED_COUNT = 1;
Adafruit_NeoPixel statusLed(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

bool cameraPsramDetected = false;
bool isCapturing = false;
bool isServingWeb = false;  // Flag to prevent camera access during web serving
bool tftEnabled = true;      // Flag to enable/disable TFT display (set to true if display is connected)

// Persistent image storage
uint8_t* storedImageBuffer = nullptr;
size_t storedImageSize = 0;
char captureTimestamp[32] = "No image captured";
uint32_t lastCaptureTime = 0;
const uint32_t CAPTURE_COOLDOWN_MS = 3000;  // 3 seconds

// Camera pins
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

// TFT Display (ILI9341) pins - safe from PSRAM conflict (GPIO 26-37)
#define TFT_MOSI_GPIO_NUM  38
#define TFT_SCLK_GPIO_NUM  39
#define TFT_CS_GPIO_NUM    40
#define TFT_DC_GPIO_NUM    41
#define TFT_RST_GPIO_NUM   42
#define TFT_BL_GPIO_NUM    45

// Shutter Button
#define SHUTTER_BUTTON_GPIO_NUM 47

// LovyanGFX configuration for ILI9341
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel_instance;
  lgfx::Bus_SPI _bus_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = false;
      cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = TFT_SCLK_GPIO_NUM;
      cfg.pin_mosi = TFT_MOSI_GPIO_NUM;
      cfg.pin_miso = -1;
      cfg.pin_dc = TFT_DC_GPIO_NUM;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = TFT_CS_GPIO_NUM;
      cfg.pin_rst = TFT_RST_GPIO_NUM;
      cfg.pin_busy = -1;
      cfg.panel_width = 240;
      cfg.panel_height = 320;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = true;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;
      _panel_instance.config(cfg);
    }

    setPanel(&_panel_instance);
  }
};

static LGFX lcd;

void setStatusLed(uint8_t red, uint8_t green, uint8_t blue) {
  statusLed.setPixelColor(0, statusLed.Color(red, green, blue));
  statusLed.show();
}

void initDisplay() {
  if (!tftEnabled) {
    Serial.println("[Display] TFT display disabled - set tftEnabled=true to enable");
    return;
  }
  
  Serial.println("[Display] Initializing TFT with LovyanGFX...");
  
  lcd.init();
  lcd.setRotation(1);  // Landscape mode (320x240)
  lcd.fillScreen(TFT_BLACK);
  
  // Turn on backlight
  pinMode(TFT_BL_GPIO_NUM, OUTPUT);
  digitalWrite(TFT_BL_GPIO_NUM, HIGH);
  
  // Display startup message
  lcd.setTextColor(TFT_WHITE);
  lcd.setTextSize(2);
  lcd.setCursor(10, 10);
  lcd.println("ESP32-S3 Camera");
  lcd.setCursor(10, 35);
  lcd.println("Initializing...");
  
  Serial.println("[Display] ILI9341 initialized with LovyanGFX");
}

void updateDisplay(const char* status, uint32_t color = TFT_WHITE) {
  if (!tftEnabled) return;
  
  lcd.fillRect(0, 60, 320, 30, TFT_BLACK);
  lcd.setTextColor(color);
  lcd.setTextSize(2);
  lcd.setCursor(10, 65);
  lcd.println(status);
}

void displayCameraInfo() {
  if (!tftEnabled) return;
  
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_CYAN);
  lcd.setTextSize(2);
  lcd.setCursor(10, 10);
  lcd.println("Camera Status");
  
  lcd.setTextColor(TFT_WHITE);
  lcd.setTextSize(1);
  
  lcd.setCursor(10, 40);
  lcd.print("PSRAM: ");
  lcd.setTextColor(cameraPsramDetected ? TFT_GREEN : TFT_RED);
  lcd.println(cameraPsramDetected ? "Available" : "Not Found");
  
  lcd.setTextColor(TFT_WHITE);
  lcd.setCursor(10, 60);
  lcd.print("WiFi: ");
  if (WiFi.status() == WL_CONNECTED) {
    lcd.setTextColor(TFT_GREEN);
    lcd.println("Connected");
    lcd.setTextColor(TFT_WHITE);
    lcd.setCursor(10, 75);
    lcd.print("IP: ");
    lcd.println(WiFi.localIP().toString());
  } else {
    lcd.setTextColor(TFT_YELLOW);
    lcd.println("Connecting...");
  }
  
  lcd.setTextColor(TFT_WHITE);
  lcd.setCursor(10, 100);
  lcd.println("Stream: /stream");
  lcd.setCursor(10, 115);
  lcd.println("Snapshot: /jpg");
}

void displayCameraFrame() {
  if (!tftEnabled) return;
  
  // Don't access camera if serving web content
  if (isServingWeb || isCapturing) {
    delay(10);
    return;
  }
  
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    return;
  }
  
  // Draw JPEG directly using LovyanGFX built-in support
  lcd.drawJpg(fb->buf, fb->len, 0, 0, 320, 240);
  
  esp_camera_fb_return(fb);
}

bool captureHighResStill() {
  Serial.println("[Camera] Capturing high-res still...");
  
  // Get current sensor
  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    Serial.println("[Camera] Failed to get sensor");
    return false;
  }
  
  // Return all frame buffers before changing resolution
  esp_camera_return_all();
  delay(100);
  
  // Change to high resolution (1600x1200 = UXGA)
  Serial.println("[Camera] Switching to UXGA (1600x1200)...");
  sensor->set_framesize(sensor, FRAMESIZE_UXGA);
  
  // Also adjust JPEG quality for high-res capture (lower number = higher quality)
  sensor->set_quality(sensor, 10);
  
  delay(500);  // Give more time for sensor to adjust
  
  // Grab and discard 3 frames to let sensor stabilize at new resolution
  Serial.println("[Camera] Warming up sensor...");
  for (int i = 0; i < 3; i++) {
    camera_fb_t* tempFb = esp_camera_fb_get();
    if (tempFb) {
      Serial.printf("[Camera] Warmup frame %d: %dx%d (%d bytes)\n", 
                    i+1, tempFb->width, tempFb->height, tempFb->len);
      esp_camera_fb_return(tempFb);
    } else {
      Serial.printf("[Camera] Warmup frame %d failed\n", i+1);
    }
    delay(100);
  }
  
  // Now capture the actual high-res frame
  Serial.println("[Camera] Capturing final frame...");
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[Camera] High-res capture failed");
    // Restore settings
    sensor->set_framesize(sensor, FRAMESIZE_QVGA);
    sensor->set_quality(sensor, 12);
    esp_camera_return_all();
    return false;
  }
  
  Serial.printf("[Camera] Captured %dx%d image (%d bytes)\n", 
                fb->width, fb->height, fb->len);
  
  // Free previous stored image if it exists
  if (storedImageBuffer != nullptr) {
    free(storedImageBuffer);
    storedImageBuffer = nullptr;
  }
  
  // Allocate memory in PSRAM for persistent storage
  storedImageSize = fb->len;
  if (psramFound()) {
    storedImageBuffer = (uint8_t*)ps_malloc(storedImageSize);
  } else {
    storedImageBuffer = (uint8_t*)malloc(storedImageSize);
  }
  
  if (storedImageBuffer != nullptr) {
    // Copy image data to persistent storage
    memcpy(storedImageBuffer, fb->buf, storedImageSize);
    
    // Store timestamp
    unsigned long uptimeSeconds = millis() / 1000;
    unsigned long hours = uptimeSeconds / 3600;
    unsigned long minutes = (uptimeSeconds % 3600) / 60;
    unsigned long seconds = uptimeSeconds % 60;
    snprintf(captureTimestamp, sizeof(captureTimestamp), 
             "Uptime: %02lu:%02lu:%02lu", hours, minutes, seconds);
    
    Serial.printf("[Camera] Image saved to %s - %s\n", 
                  psramFound() ? "PSRAM" : "DRAM", captureTimestamp);
  } else {
    Serial.println("[Camera] Failed to allocate memory for image storage!");
  }
  
  // Show captured image info on display
  if (tftEnabled) {
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextColor(TFT_GREEN);
    lcd.setTextSize(2);
    lcd.setCursor(10, 10);
    lcd.println("Image Captured!");
    lcd.setTextSize(1);
    lcd.setCursor(10, 40);
    lcd.printf("%dx%d - %d KB", fb->width, fb->height, fb->len / 1024);
    lcd.setCursor(10, 55);
    lcd.println(captureTimestamp);
  }
  
  // Return the frame buffer
  esp_camera_fb_return(fb);
  
  // Restore to normal resolution and quality for preview
  Serial.println("[Camera] Restoring to preview mode...");
  sensor->set_framesize(sensor, FRAMESIZE_QVGA);
  sensor->set_quality(sensor, 12);
  esp_camera_return_all();
  delay(200);
  
  Serial.println("[Camera] Restored to preview resolution");
  
  // Show confirmation for 2 seconds
  delay(2000);
  
  return true;
}

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
    // Initialize with UXGA to allocate large enough buffers
    // We'll switch to QVGA after init for preview
    config.frame_size = FRAMESIZE_UXGA;  // 1600x1200 - buffers sized for max resolution
    config.jpeg_quality = 10;
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
    
    // If we initialized with UXGA for large buffers, switch to QVGA for preview
    if (hasPsram && config.frame_size == FRAMESIZE_UXGA) {
      Serial.println("[Camera] Switching to QVGA for preview mode...");
      sensor->set_framesize(sensor, FRAMESIZE_QVGA);
      sensor->set_quality(sensor, 12);
      delay(200);
      // Clear any buffered frames
      esp_camera_return_all();
      Serial.println("[Camera] Preview mode ready (buffers sized for UXGA)");
    }
  }

  Serial.println("[Camera] Camera initialized");
  return true;
}

void handleRoot() {
  String html = "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32 Camera - Captured Image</title>"
    "<style>"
    "body{font-family:Arial;text-align:center;margin:0;padding:20px;background:#f0f0f0;}"
    "h2{color:#333;margin-bottom:10px;}"
    ".timestamp{color:#666;font-size:14px;margin-bottom:20px;}"
    ".container{max-width:1600px;margin:0 auto;background:white;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);}"
    "img{width:100%;height:auto;border:1px solid #ccc;border-radius:4px;}"
    ".links{margin-top:20px;}"
    ".links a{display:inline-block;margin:5px;padding:10px 20px;background:#007bff;color:white;text-decoration:none;border-radius:4px;}"
    ".links a:hover{background:#0056b3;}"
    "</style>"
    "</head><body>"
    "<div class='container'>"
    "<h2>ESP32 Camera - Captured Image</h2>"
    "<div class='timestamp'>" + String(captureTimestamp) + "</div>";
  
  if (storedImageBuffer != nullptr && storedImageSize > 0) {
    html += "<img src='/captured.jpg' id='capturedImg'>";
  } else {
    html += "<p style='color:#999;'>No image captured yet. Press the shutter button to capture an image.</p>";
  }
  
  html += "<div class='links'>"
    "<a href='/stream'>Live Stream</a>"
    "<a href='/captured.jpg' download='capture.jpg'>Download Image</a>"
    "<a href='/'>Refresh</a>"
    "</div>"
    "</div>"
    "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleJpg() {
  // Serve the stored captured image
  if (storedImageBuffer != nullptr && storedImageSize > 0) {
    isServingWeb = true;  // Prevent camera access during serving
    
    Serial.printf("[Web] Serving captured image (%d bytes)\n", storedImageSize);
    
    server.sendHeader("Content-Type", "image/jpeg");
    server.sendHeader("Content-Length", String(storedImageSize));
    server.sendHeader("Content-Disposition", "inline; filename=\"capture.jpg\"");
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    
    WiFiClient client = server.client();
    if (client.connected()) {
      client.write("HTTP/1.1 200 OK\r\n");
      client.write("Content-Type: image/jpeg\r\n");
      client.printf("Content-Length: %d\r\n", storedImageSize);
      client.write("Connection: close\r\n\r\n");
      
      // Send in chunks to avoid buffer issues
      const size_t chunkSize = 1024;
      for (size_t i = 0; i < storedImageSize; i += chunkSize) {
        size_t remaining = storedImageSize - i;
        size_t toSend = (remaining < chunkSize) ? remaining : chunkSize;
        client.write(&storedImageBuffer[i], toSend);
        delay(1);  // Small delay to prevent buffer overflow
      }
      client.flush();
      delay(10);
    }
    
    isServingWeb = false;
    Serial.println("[Web] Image sent successfully");
  } else {
    server.send(404, "text/plain", "No image captured yet");
  }
}

void handleStream() {
  isServingWeb = true;  // Prevent camera conflicts
  
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
  
  isServingWeb = false;
}

void setup() {
  Serial.begin(115200);

  statusLed.begin();
  statusLed.setBrightness(50);
  setStatusLed(255, 0, 0);

  delay(5000);
  Serial.println();
  Serial.println("[Boot] ESP32 Camera starting...");
  
  // Initialize TFT Display (optional - set tftEnabled=true if display is connected)
  initDisplay();

  updateDisplay("Initializing Camera...", TFT_YELLOW);
  if (!initCamera()) {
    Serial.println("[Boot] Camera init failed. Halting.");
    updateDisplay("Camera FAILED!", TFT_RED);
    while (true) {
      delay(1000);
    }
  }
  updateDisplay("Camera OK!", TFT_GREEN);
  delay(500);

  updateDisplay("Connecting WiFi...", TFT_YELLOW);
  connectToWifi();

  if (WiFi.status() == WL_CONNECTED && WiFi.localIP()) {
    if (cameraPsramDetected) {
      setStatusLed(0, 255, 0);
    } else {
      setStatusLed(0, 0, 255);
    }
  }
  
  delay(1000);
  // Display full camera info on TFT
  displayCameraInfo();
  delay(2000);
  
  // Setup shutter button with internal pullup
  pinMode(SHUTTER_BUTTON_GPIO_NUM, INPUT_PULLUP);
  Serial.println("[Button] Shutter button configured on GPIO 47");
  
  // Clear display for camera preview
  if (tftEnabled) {
    lcd.fillScreen(TFT_BLACK);
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/jpg", HTTP_GET, handleJpg);
  server.on("/captured.jpg", HTTP_GET, handleJpg);  // Alias for captured image
  server.on("/stream", HTTP_GET, handleStream);

  server.begin();
  Serial.println("[Web] Server started");
  Serial.print("[Web] Open in browser: http://");
  Serial.println(WiFi.localIP());
}

void loop() {
  server.handleClient();
  
  // Check for button press (active LOW with pullup)
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(SHUTTER_BUTTON_GPIO_NUM);
  
  // Check if enough time has passed since last capture (3-second cooldown)
  uint32_t currentTime = millis();
  bool cooldownExpired = (currentTime - lastCaptureTime) >= CAPTURE_COOLDOWN_MS;
  
  if (buttonState == LOW && lastButtonState == HIGH && !isCapturing && cooldownExpired) {
    // Button pressed and cooldown expired - capture high-res image
    isCapturing = true;
    lastCaptureTime = currentTime;
    
    // Turn LED white
    setStatusLed(255, 255, 255);
    
    // Capture high-res still and save to PSRAM
    captureHighResStill();
    
    // Restore LED to normal color
    if (cameraPsramDetected) {
      setStatusLed(0, 255, 0);  // Green for PSRAM
    } else {
      setStatusLed(0, 0, 255);  // Blue for no PSRAM
    }
    
    isCapturing = false;
    delay(100);  // Debounce
  }
  
  lastButtonState = buttonState;
  
  // Continuously display camera preview (only if TFT is enabled and not busy)
  if (tftEnabled && !isCapturing && !isServingWeb) {
    displayCameraFrame();
  } else {
    delay(10);  // Small delay when not displaying
  }
}
