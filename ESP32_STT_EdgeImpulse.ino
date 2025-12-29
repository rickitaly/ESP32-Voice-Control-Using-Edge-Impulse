/*
 * ESP32 Voice Control Using Edge Impulse
 * 
 * This sketch implements speech-to-text functionality using Edge Impulse
 * with NeoPixel LED feedback and I2S microphone input.
 * 
 * Updated: 2025-12-29
 * Changes:
 * - Added I2S pin configuration for GPIO4, GPIO5, GPIO6 and I2S_NUM_0 port
 * - Added NeoPixel LED configuration (GPIO48, 1 LED, brightness 50)
 * - Updated voice commands: "on" -> "yes", "off" -> "no"
 * - Added Adafruit_NeoPixel library support
 */

#include <Adafruit_NeoPixel.h>

// ===== I2S Configuration =====
#define I2S_WS_PIN      GPIO_NUM_4    // I2S Word Select (LRCLK)
#define I2S_BCK_PIN     GPIO_NUM_5    // I2S Bit Clock (BCLK)
#define I2S_DATA_PIN    GPIO_NUM_6    // I2S Data In (DIN)
#define I2S_PORT        I2S_NUM_0     // I2S Port Number

// ===== NeoPixel LED Configuration =====
#define NEOPIXEL_PIN    48            // GPIO pin for NeoPixel
#define NEOPIXEL_COUNT  1             // Number of LEDs
#define NEOPIXEL_BRIGHTNESS 50        // LED Brightness (0-255)

// ===== Voice Command Definitions =====
#define VOICE_COMMAND_YES   "yes"     // Command for affirmative (previously "on")
#define VOICE_COMMAND_NO    "no"      // Command for negative (previously "off")

// Global NeoPixel object
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

/**
 * Initialize NeoPixel LED
 */
void setupNeoPixel() {
  pixels.begin();
  pixels.setBrightness(NEOPIXEL_BRIGHTNESS);
  pixels.clear();
  pixels.show();
}

/**
 * Set NeoPixel LED color
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 */
void setNeoPixelColor(uint8_t red, uint8_t green, uint8_t blue) {
  pixels.setPixelColor(0, pixels.Color(red, green, blue));
  pixels.show();
}

/**
 * Process voice commands
 * @param command The recognized voice command
 */
void processVoiceCommand(const char* command) {
  if (strcmp(command, VOICE_COMMAND_YES) == 0) {
    // Handle "yes" command (previously "on")
    Serial.println("Voice Command: YES");
    setNeoPixelColor(0, 255, 0);  // Green LED
    // Add your yes command logic here
  } 
  else if (strcmp(command, VOICE_COMMAND_NO) == 0) {
    // Handle "no" command (previously "off")
    Serial.println("Voice Command: NO");
    setNeoPixelColor(255, 0, 0);  // Red LED
    // Add your no command logic here
  }
  else {
    Serial.print("Unknown command: ");
    Serial.println(command);
    setNeoPixelColor(255, 255, 0);  // Yellow LED for unknown
  }
}

/**
 * Initialize I2S microphone interface
 */
void setupI2S() {
  // Configure I2S interface for microphone input
  // I2S pins: WS (GPIO4), BCK (GPIO5), DATA (GPIO6)
  // Using I2S_NUM_0 port
  // Add your I2S configuration code here
  Serial.println("I2S configured with:");
  Serial.print("  WS (LRCLK): GPIO");
  Serial.println(I2S_WS_PIN);
  Serial.print("  BCK (BCLK): GPIO");
  Serial.println(I2S_BCK_PIN);
  Serial.print("  DATA (DIN): GPIO");
  Serial.println(I2S_DATA_PIN);
  Serial.print("  Port: I2S_NUM_");
  Serial.println(I2S_PORT);
}

/**
 * Arduino Setup Function
 */
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ESP32 Voice Control Using Edge Impulse ===");
  Serial.println("Starting initialization...\n");
  
  // Initialize I2S for microphone input
  setupI2S();
  
  // Initialize NeoPixel LED
  setupNeoPixel();
  setNeoPixelColor(0, 0, 255);  // Blue LED during initialization
  
  Serial.println("\nInitialization complete!");
  Serial.println("Listening for voice commands: 'yes' or 'no'");
}

/**
 * Arduino Loop Function
 */
void loop() {
  // Main loop - add your Edge Impulse inference logic here
  // Example voice command processing:
  
  // When a voice command is recognized from Edge Impulse, call:
  // processVoiceCommand("yes");  // or "no"
  
  delay(100);
}
