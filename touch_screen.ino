#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <AppleMIDI.h>

// WiFi settings - CHANGE THESE
const char *ssid = "A202";
const char *pass = "VUah5EA9";

// MIDI settings
#define MIDI_CC_X 22     // CC number for X position
#define MIDI_CC_Y 23     // CC number for Y position
#define MIDI_CC_TOUCH 24 // CC number for touch state
#define MIDI_CHANNEL 1   // MIDI channel to send on

// Keep screen black
#define TFT_CS 5
#define TFT_DC 27
#define TFT_RST 33
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_MISO 19

// Display commands
#define CMD_SLEEP_OUT 0x11
#define CMD_DISPLAY_ON 0x29
#define CMD_COLUMN_SET 0x2A
#define CMD_ROW_SET 0x2B
#define CMD_MEM_WRITE 0x2C
#define CMD_INVERSION 0x21
#define CMD_NORMAL 0x20

// Screen settings
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 480
#define BLACK 0x0000

// GT911 Touch Controller Settings
#define TOUCH_SDA 33
#define TOUCH_SCL 32
#define TOUCH_INT 21 // Interrupt pin
#define TOUCH_RST 25 // Reset pin
#define TOUCH_ADDR 0x5D
#define TOUCH_REG_TRACK 0x814E // First touch point register

// Other peripherals
#define LED_R 4      // RGB LED - Red
#define LED_G 16     // RGB LED - Green
#define LED_B 17     // RGB LED - Blue
#define BACKLIGHT 27 // Display backlight

// Tracking connection state
bool isConnected = false;

// Create an instance of the AppleMIDI interface
APPLEMIDI_CREATE_INSTANCE(WiFiUDP, AppleMIDI, "ESP32-Touch", 5004);

void setup()
{
  // Initialize serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nESP32-3248S035C Touch MIDI Controller");

  // Configure display pins for black screen
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);

  // RGB LED
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, HIGH); // LEDs are active LOW
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);

  // Backlight
  pinMode(BACKLIGHT, OUTPUT);
  digitalWrite(BACKLIGHT, HIGH);

  // Hardware reset
  digitalWrite(TFT_RST, HIGH);
  delay(50);
  digitalWrite(TFT_RST, LOW);
  delay(50);
  digitalWrite(TFT_RST, HIGH);
  delay(150);

  // Initialize SPI
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI);
  SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));

  // Basic initialization sequence
  sendCommand(CMD_SLEEP_OUT); // Exit sleep mode
  delay(120);
  sendCommand(CMD_DISPLAY_ON); // Turn display on
  delay(120);

  // Set column address (whole screen)
  sendCommand(CMD_COLUMN_SET);
  sendData(0x00);
  sendData(0x00); // Start column
  sendData(0x01);
  sendData(0x3F); // End column (319)

  // Set row address (whole screen)
  sendCommand(CMD_ROW_SET);
  sendData(0x00);
  sendData(0x00); // Start row
  sendData(0x00);
  sendData(0xEF); // End row (239)

  // Prepare to send pixel data
  sendCommand(CMD_MEM_WRITE);

  // Fill screen with black
  Serial.println("Filling screen with BLACK...");
  digitalWrite(TFT_DC, HIGH);
  digitalWrite(TFT_CS, LOW);

  for (uint32_t i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++)
  {
    SPI.transfer(0x00); // Black pixel
    SPI.transfer(0x00);
    if ((i % 1000) == 0)
      yield();
  }
  digitalWrite(TFT_CS, HIGH);

  // Set display to inverted mode (this shows as black on this display)
  sendCommand(CMD_INVERSION);

  Serial.println("Screen set to BLACK");

  // Initialize I2C for GT911 touch controller
  Wire.begin(TOUCH_SDA, TOUCH_SCL);

  // Scan I2C bus
  Serial.println("Scanning I2C bus...");
  byte error, address;
  int deviceCount = 0;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      deviceCount++;
    }
  }

  if (deviceCount == 0)
  {
    Serial.println("No I2C devices found");
  }

  // Connect to WiFi
  Serial.print("Connecting to WiFi ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  // Wait for connection (with timeout)
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20)
  {
    delay(500);
    Serial.print(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println();
    Serial.print("WiFi connected - IP address: ");
    Serial.println(WiFi.localIP());

    // Blink green LED to indicate WiFi connection
    digitalWrite(LED_G, LOW);
    delay(200);
    digitalWrite(LED_G, HIGH);
  }
  else
  {
    Serial.println();
    Serial.println("WiFi connection failed!");

    // Blink red LED to indicate WiFi failure
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(LED_R, LOW);
      delay(200);
      digitalWrite(LED_R, HIGH);
      delay(200);
    }
  }

  Serial.println("OK, now make sure you have an rtpMIDI session that is Enabled");
  Serial.print("Add device named ESP32-Touch with Host/Port ");
  Serial.print(WiFi.localIP());
  Serial.println(":5004");
  Serial.println("Then press the Connect button");
  Serial.println("Then open a MIDI listener and monitor incoming MIDI commands");

  Serial.println("\nREADY TO TEST TOUCHSCREEN MIDI");
  Serial.println("Touch the screen to send MIDI Control Changes");
}

void loop()
{
  // Process incoming MIDI messages
  AppleMIDI.read();

  static unsigned long lastTouchCheck = 0;
  static bool wasTouch = false;
  static uint8_t lastX = 0;
  static uint8_t lastY = 0;

  // Check for touch input every 20ms
  if (millis() - lastTouchCheck > 20)
  {
    lastTouchCheck = millis();

    // Read touch status
    Wire.beginTransmission(TOUCH_ADDR);
    Wire.write(0x81); // Status register
    Wire.write(0x4E);
    Wire.endTransmission(false);

    Wire.requestFrom(TOUCH_ADDR, 1);
    if (Wire.available())
    {
      uint8_t status = Wire.read();
      bool isTouched = status & 0x80;      // Check if touch flag is set
      uint8_t touchPoints = status & 0x0F; // Number of touch points

      if (isTouched && touchPoints > 0)
      {
        // Read first touch point
        Wire.beginTransmission(TOUCH_ADDR);
        Wire.write(0x81);
        Wire.write(0x50); // First touch point X low byte
        Wire.endTransmission(false);

        Wire.requestFrom(TOUCH_ADDR, 4); // Read X and Y coordinates (2 bytes each)
        if (Wire.available() >= 4)
        {
          uint8_t xLow = Wire.read();
          uint8_t xHigh = Wire.read();
          uint8_t yLow = Wire.read();
          uint8_t yHigh = Wire.read();

          uint16_t x = xLow | (xHigh << 8);
          uint16_t y = yLow | (yHigh << 8);

          // Map touch coordinates to MIDI CC range (0-127)
          uint8_t midiX = map(x, 0, SCREEN_WIDTH, 0, 127);
          uint8_t midiY = map(y, 0, SCREEN_HEIGHT, 0, 127);

          // Only send if value changed (to avoid flooding)
          if (midiX != lastX)
          {
            AppleMIDI.sendControlChange(MIDI_CC_X, midiX, MIDI_CHANNEL);
            lastX = midiX;
          }

          if (midiY != lastY)
          {
            AppleMIDI.sendControlChange(MIDI_CC_Y, midiY, MIDI_CHANNEL);
            lastY = midiY;
          }

          // If this is a new touch, send touch state CC
          if (!wasTouch)
          {
            AppleMIDI.sendControlChange(MIDI_CC_TOUCH, 127, MIDI_CHANNEL);
            Serial.println("Touch ON - MIDI sent");
          }

          Serial.printf("Touch: X=%d, Y=%d (MIDI: %d, %d)\n", x, y, midiX, midiY);

          // Light up RGB LED based on touch position
          if (x < SCREEN_WIDTH / 3)
          {
            digitalWrite(LED_R, LOW);
            digitalWrite(LED_G, HIGH);
            digitalWrite(LED_B, HIGH);
          }
          else if (x < 2 * SCREEN_WIDTH / 3)
          {
            digitalWrite(LED_R, HIGH);
            digitalWrite(LED_G, LOW);
            digitalWrite(LED_B, HIGH);
          }
          else
          {
            digitalWrite(LED_R, HIGH);
            digitalWrite(LED_G, HIGH);
            digitalWrite(LED_B, LOW);
          }

          wasTouch = true;
        }
      }
      else if (wasTouch)
      {
        // Touch released
        digitalWrite(LED_R, HIGH);
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_B, HIGH);

        // Send touch off message
        AppleMIDI.sendControlChange(MIDI_CC_TOUCH, 0, MIDI_CHANNEL);

        Serial.println("Touch OFF - MIDI sent");
        wasTouch = false;
      }

      // Clear status register
      Wire.beginTransmission(TOUCH_ADDR);
      Wire.write(0x81);
      Wire.write(0x4E);
      Wire.write(0x00);
      Wire.endTransmission();
    }
  }

  // Brief delay
  delay(10);
}

// Send command to display
void sendCommand(uint8_t cmd)
{
  digitalWrite(TFT_DC, LOW); // Command mode
  digitalWrite(TFT_CS, LOW); // Select display
  SPI.transfer(cmd);
  digitalWrite(TFT_CS, HIGH);
  delay(1);
}

// Send data to display
void sendData(uint8_t data)
{
  digitalWrite(TFT_DC, HIGH); // Data mode
  digitalWrite(TFT_CS, LOW);
  SPI.transfer(data);
  digitalWrite(TFT_CS, HIGH);
  delay(1);
}