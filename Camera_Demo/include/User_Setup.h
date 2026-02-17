// User Setup for TFT_eSPI - ILI9341 Display Configuration
// Configured to avoid conflicts with ESP32-S3 Camera pins

#define USER_SETUP_INFO "User_Setup"

// Define the display driver
#define ILI9341_DRIVER

// Display resolution
#define TFT_WIDTH  240
#define TFT_HEIGHT 320

// Pin configuration - Using pins that don't conflict with camera or PSRAM (GPIO 26-37)
#define TFT_MOSI 38  // SPI MOSI
#define TFT_SCLK 39  // SPI Clock
#define TFT_CS   40  // Chip select control pin
#define TFT_DC   41  // Data Command control pin
#define TFT_RST  42  // Reset pin (could also connect to RST pin on ESP32)
#define TFT_BL   45  // LED backlight control pin

// Use HSPI (SPI3) to avoid conflicts with camera
#define USE_HSPI_PORT

// Font support
#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH
#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH
#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH
#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH
#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts

#define SMOOTH_FONT

// SPI speed
#define SPI_FREQUENCY  27000000  // 27MHz for ILI9341
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000

// Optional: Enable if using DMA
// #define DMA_CHANNEL 1
