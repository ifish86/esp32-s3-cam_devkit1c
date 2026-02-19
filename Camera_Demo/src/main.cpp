#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include "esp_camera.h"
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "credentials.h"  // WiFi credentials
#include "FS.h"
#include "SD_MMC.h"

WebServer server(80);

constexpr int LED_PIN = 48;  // Built-in LED, can't be moved
constexpr int LED_COUNT = 1;
Adafruit_NeoPixel statusLed(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

bool cameraPsramDetected = false;
bool sdCardDetected = false;
bool isCapturing = false;
bool isServingWeb = false;  // Flag to prevent camera access during web serving
bool tftEnabled = true;      // Flag to enable/disable TFT display (set to true if display is connected)
bool tftPreviewEnabled = true;  // Enable continuous camera preview on TFT
bool audioEnabled = false;  // Track if audio PWM is initialized

// SD card photo tracking
int photoCount = 0;
uint64_t sdCardTotal = 0;
uint64_t sdCardUsed = 0;

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

// TFT Display (ILI9341) pins - moved to avoid SD card (38-43)
#define TFT_MOSI_GPIO_NUM  19
#define TFT_SCLK_GPIO_NUM  20
#define TFT_CS_GPIO_NUM    21
#define TFT_DC_GPIO_NUM    14
#define TFT_RST_GPIO_NUM   45
#define TFT_BL_GPIO_NUM    3   // TFT backlight on GPIO 3

// SD Card uses SD_MMC interface (built-in slot on Freenove board)
// Pins: CLK=39, CMD=38, D0=40, D1=41, D2=42, D3=43 (fixed by board)

// Shutter Button
#define SHUTTER_BUTTON_GPIO_NUM 47

// Audio buzzer pin (PWM-based beep)
#define BUZZER_GPIO_NUM    35
#define BUZZER_CHANNEL     0
#define BUZZER_FREQ        1000  // 1kHz beep
#define BUZZER_RESOLUTION  8

// LovyanGFX configuration for ILI9341
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel_instance;
  lgfx::Bus_SPI _bus_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI3_HOST;  // Changed from SPI2_HOST to avoid conflicts
      cfg.spi_mode = 0;
      cfg.freq_write = 20000000;  // Reduced from 40MHz to 20MHz for stability
      cfg.freq_read = 10000000;   // Reduced from 16MHz to 10MHz
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
      cfg.bus_shared = true;  // Changed to true - bus shared with other devices
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

void initAudio() {
  Serial.println("[Audio] Initializing PWM buzzer...");
  
  // Configure LEDC for PWM audio output
  ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, BUZZER_RESOLUTION);
  ledcAttachPin(BUZZER_GPIO_NUM, BUZZER_CHANNEL);
  ledcWrite(BUZZER_CHANNEL, 0);  // Start silent
  
  audioEnabled = true;
  Serial.printf("[Audio] PWM buzzer initialized on GPIO %d\n", BUZZER_GPIO_NUM);
}

void playClickSound() {
  // Play a short beep (shutter click sound) - only if audio is enabled
  if (!audioEnabled) return;
  
  ledcWriteTone(BUZZER_CHANNEL, BUZZER_FREQ);
  ledcWrite(BUZZER_CHANNEL, 128);  // 50% duty cycle
  delay(50);  // 50ms beep
  ledcWrite(BUZZER_CHANNEL, 0);    // Silence
}

void initDisplay() {
  if (!tftEnabled) {
    Serial.println("[Display] TFT display disabled - set tftEnabled=true to enable");
    return;
  }
  
  Serial.println("[Display] Initializing TFT with LovyanGFX...");
  Serial.printf("[Display] Pins: MOSI=%d, SCLK=%d, CS=%d, DC=%d, RST=%d, BL=%d\n",
                TFT_MOSI_GPIO_NUM, TFT_SCLK_GPIO_NUM, TFT_CS_GPIO_NUM, 
                TFT_DC_GPIO_NUM, TFT_RST_GPIO_NUM, TFT_BL_GPIO_NUM);
  
  // Hardware reset before init
  pinMode(TFT_RST_GPIO_NUM, OUTPUT);
  digitalWrite(TFT_RST_GPIO_NUM, LOW);
  delay(100);
  digitalWrite(TFT_RST_GPIO_NUM, HIGH);
  delay(200);
  
  Serial.println("[Display] Starting LovyanGFX init...");
  lcd.init();
  Serial.println("[Display] Init complete, setting rotation...");
  lcd.setRotation(1);  // Landscape mode (320x240)
  
  Serial.println("[Display] Attempting to fill screen red...");
  lcd.fillScreen(TFT_RED);    // Try red to see if it's working
  delay(1000);
  
  Serial.println("[Display] Filling screen black...");
  lcd.fillScreen(TFT_BLACK);
  
  // Turn on backlight
  pinMode(TFT_BL_GPIO_NUM, OUTPUT);
  digitalWrite(TFT_BL_GPIO_NUM, HIGH);
  Serial.println("[Display] Backlight enabled");
  
  // Display startup message
  Serial.println("[Display] Drawing text...");
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
  lcd.print("SD Card: ");
  lcd.setTextColor(sdCardDetected ? TFT_GREEN : TFT_RED);
  lcd.println(sdCardDetected ? "Ready" : "Not Found");
  
  lcd.setTextColor(TFT_WHITE);
  lcd.setCursor(10, 75);
  lcd.print("WiFi: ");
  if (WiFi.status() == WL_CONNECTED) {
    lcd.setTextColor(TFT_GREEN);
    lcd.println("Connected");
    lcd.setTextColor(TFT_WHITE);
    lcd.setCursor(10, 90);
    lcd.print("IP: ");
    lcd.println(WiFi.localIP().toString());
  } else {
    lcd.setTextColor(TFT_YELLOW);
    lcd.println("Connecting...");
  }
  
  lcd.setTextColor(TFT_WHITE);
  lcd.setCursor(10, 115);
  lcd.println("Stream: /stream");
  lcd.setCursor(10, 130);
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

bool initSDCard() {
  Serial.println("[SD] Initializing SD_MMC card (built-in slot)...");
  
  // Configure SD_MMC pins explicitly for Freenove ESP32-S3
  SD_MMC.setPins(39, 38, 40, 41, 42, 43);  // CLK, CMD, D0, D1, D2, D3
  Serial.println("[SD] Pins configured: CLK=39, CMD=38, D0=40, D1=41, D2=42, D3=43");
  
  // Try 1-bit mode first (more compatible, uses only CLK, CMD, D0)
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[SD] 1-bit mode failed, trying 4-bit mode...");
    // Try 4-bit mode (uses all 6 pins)
    if (!SD_MMC.begin("/sdcard", false)) {
      Serial.println("[SD] Card Mount Failed");
      Serial.println("[SD] Check: 1) SD card inserted 2) Card formatted");
      return false;
    }
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("[SD] No SD card attached");
    return false;
  }
  
  Serial.print("[SD] Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  
  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("[SD] Card Size: %lluMB\n", cardSize);
  
  // Count existing photos
  photoCount = 0;
  File root = SD_MMC.open("/");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      String filename = String(file.name());
      // Remove leading slash if present
      if (filename.startsWith("/")) {
        filename = filename.substring(1);
      }
      if (filename.startsWith("photo") && filename.endsWith(".jpg")) {
        photoCount++;
        Serial.printf("[SD] Found: %s\n", filename.c_str());
      }
      file = root.openNextFile();
    }
    root.close();
  }
  
  Serial.printf("[SD] Found %d existing photos\n", photoCount);
  
  return true;
}

void updateSDCardStats() {
  if (!sdCardDetected) return;
  
  sdCardTotal = SD_MMC.totalBytes();
  sdCardUsed = SD_MMC.usedBytes();
}

int getNextPhotoNumber() {
  if (!sdCardDetected) return -1;
  
  // Find the next available photo number
  int photoNum = 1;
  char filename[32];
  
  while (photoNum < 10000) {  // Limit to photo9999.jpg
    snprintf(filename, sizeof(filename), "/photo%03d.jpg", photoNum);
    if (!SD_MMC.exists(filename)) {
      return photoNum;
    }
    photoNum++;
  }
  
  return -1;  // No available slots
}

void displayPhotoOverlay() {
  if (!tftEnabled || !sdCardDetected) return;
  
  updateSDCardStats();
  
  // Calculate storage percentage
  float storagePercent = 0;
  if (sdCardTotal > 0) {
    storagePercent = (sdCardUsed * 100.0) / sdCardTotal;
  }
  
  // Draw semi-transparent overlay at bottom of screen
  lcd.fillRect(0, 220, 320, 20, TFT_BLACK);
  
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(1);
  
  // Photo count on the left
  lcd.setCursor(5, 225);
  lcd.printf("Photos: %d", photoCount);
  
  // Storage on the right
  lcd.setCursor(180, 225);
  lcd.printf("SD: %.1f%%", storagePercent);
}

bool captureHighResStill() {
  Serial.println("[Camera] Capturing high-res still...");
  
  // Get current sensor
  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    Serial.println("[Camera] Failed to get sensor");
    return false;
  }
  
  // CRITICAL: Pause ALL camera access to avoid DMA conflicts
  bool wasPreviewing = tftPreviewEnabled;
  tftPreviewEnabled = false;
  isServingWeb = true;  // Block web server camera access too
  delay(500);  // Give extra time for any pending operations to complete
  
  // Return all frame buffers and wait
  esp_camera_return_all();
  delay(300);
  
  // Switch to high resolution (1600x1200 = UXGA)
  Serial.println("[Camera] Switching to UXGA (1600x1200)...");
  sensor->set_framesize(sensor, FRAMESIZE_UXGA);
  sensor->set_quality(sensor, 12);  // Slightly higher quality number = less compression = less memory stress
  delay(800);  // UXGA needs more time to stabilize
  
  // More warmup frames to ensure sensor is fully stable at UXGA
  Serial.println("[Camera] Warming up sensor (UXGA requires more warmup)...");
  for (int i = 0; i < 8; i++) {
    camera_fb_t* tempFb = esp_camera_fb_get();
    if (tempFb) {
      Serial.printf("[Camera] Warmup frame %d: %dx%d (%d bytes)\n", 
                    i+1, tempFb->width, tempFb->height, tempFb->len);
      esp_camera_fb_return(tempFb);
    }
    delay(200);  // Longer delay between warmup frames for UXGA
  }
  
  // Extra delay before final capture to ensure sensor is fully stable
  delay(400);
  
  // Now capture the actual high-res frame
  Serial.println("[Camera] Capturing final frame...");
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[Camera] High-res capture failed");
    // Restore settings
    sensor->set_framesize(sensor, FRAMESIZE_QVGA);
    sensor->set_quality(sensor, 12);
    esp_camera_return_all();
    tftPreviewEnabled = wasPreviewing;  // Restore TFT preview
    isServingWeb = false;  // Restore web server access
    return false;
  }
  
  Serial.printf("[Camera] Captured %dx%d image (%d bytes)\n", 
                fb->width, fb->height, fb->len);
  
  bool savedToSD = false;
  char filename[32] = "N/A";
  
  // Save to SD card if available
  if (sdCardDetected) {
    int photoNum = getNextPhotoNumber();
    if (photoNum > 0) {
      snprintf(filename, sizeof(filename), "/photo%03d.jpg", photoNum);
      
      File file = SD_MMC.open(filename, FILE_WRITE);
      if (file) {
        file.write(fb->buf, fb->len);
        file.close();
        photoCount++;
        savedToSD = true;
        Serial.printf("[SD] Saved to %s\n", filename);
      } else {
        Serial.printf("[SD] Failed to open %s for writing\n", filename);
      }
    } else {
      Serial.println("[SD] No available photo slots (reached photo9999.jpg)");
    }
  }
  
  // Also keep a copy in PSRAM for web serving (last captured image)
  if (storedImageBuffer != nullptr) {
    free(storedImageBuffer);
    storedImageBuffer = nullptr;
  }
  
  storedImageSize = fb->len;
  if (psramFound()) {
    storedImageBuffer = (uint8_t*)ps_malloc(storedImageSize);
  } else {
    storedImageBuffer = (uint8_t*)malloc(storedImageSize);
  }
  
  if (storedImageBuffer != nullptr) {
    memcpy(storedImageBuffer, fb->buf, storedImageSize);
    
    // Store timestamp
    unsigned long uptimeSeconds = millis() / 1000;
    unsigned long hours = uptimeSeconds / 3600;
    unsigned long minutes = (uptimeSeconds % 3600) / 60;
    unsigned long seconds = uptimeSeconds % 60;
    snprintf(captureTimestamp, sizeof(captureTimestamp), 
             "Uptime: %02lu:%02lu:%02lu", hours, minutes, seconds);
    
    Serial.printf("[Camera] Copy saved to %s for web - %s\n", 
                  psramFound() ? "PSRAM" : "DRAM", captureTimestamp);
  }
  
  // Return the frame buffer
  esp_camera_fb_return(fb);
  
  // Show captured image on display for 1 second (using the stored copy)
  if (tftEnabled && storedImageBuffer != nullptr) {
    lcd.fillScreen(TFT_BLACK);
    
    // Draw captured image - will scale automatically to fit display
    lcd.drawJpg(storedImageBuffer, storedImageSize, 0, 0, 320, 240);
    
    Serial.println("[Display] Showing captured image for 1 second");
    delay(1000);
  }
  
  // Restore to normal resolution and quality
  Serial.println("[Camera] Restoring to preview mode...");
  sensor->set_framesize(sensor, FRAMESIZE_QVGA);
  sensor->set_quality(sensor, 12);
  esp_camera_return_all();
  delay(200);
  Serial.println("[Camera] Restored to preview resolution");
  
  // Re-enable TFT preview and web server camera access
  tftPreviewEnabled = wasPreviewing;
  isServingWeb = false;
  
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

  // Initialize SD Card
  updateDisplay("Initializing SD...", TFT_YELLOW);
  sdCardDetected = initSDCard();
  if (sdCardDetected) {
    updateDisplay("SD Card OK!", TFT_GREEN);
  } else {
    updateDisplay("SD Card Failed!", TFT_RED);
  }
  delay(500);

  // Connect to WiFi
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
  
  // Initialize PWM audio for shutter click sound (disabled when WiFi active to avoid conflicts)
  if (WiFi.status() != WL_CONNECTED) {
    initAudio();
  } else {
    Serial.println("[Audio] Audio disabled - WiFi active");
  }
  
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
  
  // Display WiFi info and instructions on TFT
  if (tftEnabled) {
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_GREEN);
    lcd.setCursor(10, 40);
    lcd.println("Web Server Ready");
    lcd.setTextSize(1);
    lcd.setTextColor(TFT_WHITE);
    lcd.setCursor(10, 80);
    lcd.print("IP: ");
    lcd.println(WiFi.localIP());
    lcd.setCursor(10, 100);
    lcd.println("Browse to view/stream");
    lcd.setCursor(10, 130);
    lcd.setTextColor(TFT_YELLOW);
    lcd.println("Press button to capture");
    lcd.setCursor(10, 150);
    lcd.print("Photos: ");
    lcd.println(photoCount);
  }
  
  Serial.println("[Boot] Setup complete");
  delay(1000);  // Give everything time to stabilize before entering loop
  Serial.println("[Boot] Entering main loop");
}

void loop() {
  // Handle web server requests (only if not capturing)
  if (!isCapturing) {
    server.handleClient();
  }
  
  // Small yield to prevent watchdog issues
  yield();
  
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
    
    // Play shutter click sound
    playClickSound();
    
    // Turn LED white
    setStatusLed(255, 255, 255);
    
    // Capture high-res still and save to SD card
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
  
  // Continuously display camera preview (only if preview is enabled and not busy)
  // When WiFi is enabled, preview is disabled to avoid camera access conflicts
  if (tftPreviewEnabled && !isCapturing && !isServingWeb) {
    displayCameraFrame();
    displayPhotoOverlay();  // Show photo count and SD card usage
  } else {
    delay(10);  // Small delay when not displaying
  }
}