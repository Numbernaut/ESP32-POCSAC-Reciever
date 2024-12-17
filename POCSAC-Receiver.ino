/*
  RadioLib Pager (POCSAG) Receive Example with OLED Display and Message Navigation

  This example shows how to receive FSK packets without using
  SX127x packet engine and display messages on an OLED,
  implementing message storage and navigation using button presses.

  The first single press activates the screen with a generic display.
  Every subsequent short press iterates through the messages starting with the first.
  A long press turns off the display and resets the message iteration.

  Other modules that can be used to receive POCSAG:
  - SX127x/RFM9x
  - RF69
  - SX1231
  - CC1101
  - Si443x/RFM2x

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

// Include necessary libraries
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED display configuration
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels
#define OLED_RESET    -1    // Reset pin # (or -1 if sharing ESP32 reset pin)

// Create an instance of the OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// SX1278 module pin configuration
#define NSS_PIN     5     // NSS (Chip Select)
#define RESET_PIN  14     // Reset
// Note: DIO0 and DIO1 are not connected (-1) as per your setup
SX1278 radio = new Module(NSS_PIN, 2, RESET_PIN, 4);

// Receiving packets does not require a DIO2 pin connection in this setup
  const int pin = 12;

//frequency offset for cal
float baseFreq = 439.9875;
float offset = 0.008;

// Create Pager client instance using the FSK module
PagerClient pager(&radio);

// Variables to store messages
#define MAX_MESSAGES 10
String messageBuffer[MAX_MESSAGES];
int head = 0;          // Points to the next insertion index
int count = 0;         // Number of stored messages
int currentIndex = 0;  // Index of the current message to display
bool displayOn = false; // Indicates whether the display is on or off

// Buttons config
#define BUTTON_UP_PIN 25 // GPIO pin for the button
#define BUTTON_OK_PIN 16 // GPIO pin for the button
#define BUTTON_DOWN_PIN 26  // GPIO pin for the button
#define BUTTON_ESC_PIN 27 // GPIO pin for the button


// Debounce config
const unsigned long bounceDelay = 150; // milliseconds

// longPress thresholds
const unsigned long longPressThreshold = 800; // 0.8 second

// Variables tracking button state
int buttonPins[4] = {BUTTON_UP_PIN, BUTTON_OK_PIN, BUTTON_DOWN_PIN, BUTTON_ESC_PIN};
unsigned long buttonPressTime[4] = {0, 0, 0, 0}; // Timestamp of last Button press
unsigned long lastPressTime[4] = {0, 0, 0, 0} ;
bool buttonPressed[4] = {false, false, false, false};
bool longPressFlag[4] = {false, false, false, false}; // For detecting long presses

// Variables for non-blocking visual indicators
unsigned long singlePressIndicatorTime = 0;
bool singlePressIndicatorActive = false;

unsigned long longPressIndicatorTime = 0;
bool longPressIndicatorActive = false;

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600); 

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  delay(1000); // Pause for OLED initialization

  // Clear the buffer
  display.clearDisplay();

  // Display initial message
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);// Draw white text
  display.setCursor(0, 10);           // Start at top-left corner
  display.println("POCSAG Receiver");
  display.display();                   // Show initial message

  // Initialize SX1278 with default FSK settings
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.beginFSK();

  // Check for initialization errors
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); } // Halt on failure
  }

  // Initialize Pager client
  Serial.print(F("[Pager] Initializing ... "));
  // Set frequency to 439.9875 MHz and speed to 1200 bps
  state = pager.begin(baseFreq + offset, 1200);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); } // Halt on failure
  }

  // Start receiving POCSAG messages
  Serial.print(F("[Pager] Starting to listen ... "));
  // Address of this "pager": 1234567
  state = pager.startReceive(pin, 0, 0);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); } // Halt on failure
  }

  // Initialize button with internal pull-up resistor
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_OK_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_ESC_PIN, INPUT_PULLUP);

  // Initialize messageBuffer for testing, should be omitted
    for (int i = 0; i < MAX_MESSAGES; i++) {
    messageBuffer[i] = "Test Message " + String(i + 1);
  }
  count = MAX_MESSAGES; // Set count to indicate the buffer is full
  head = 0;             // Reset head for circular buffer behavior
}

void loop() {
  // Check if a POCSAG message is available
  if (pager.available() >= 2) {
    Serial.print(F("[Pager] Received pager data, decoding ... "));

    // Read the received data into a string
    String str;
    int state = pager.readData(str);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
      Serial.print(F("[Pager] Data:\t"));
      Serial.println(str);

      // Store the received message
      storeMessage(str);

    } else {
      // Handle errors during data reading
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
  }

  handleButtonPress(0); // UP
  handleButtonPress(1); // OK
  handleButtonPress(2); // DOWN
  handleButtonPress(3); // ESC

  if (displayOn) {
    // Handle visual indicators
    handleVisualIndicators();
    
    // Update display with new drawing operations
    display.display();
  }
}

void storeMessage(String newMessage) {
  messageBuffer[head] = newMessage;
  head = (head + 1) % MAX_MESSAGES;
  if (count < MAX_MESSAGES) {
    count++;
  }
}

void displayMessage(int index) {
  if (count == 0) {
    // No messages to display
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("No Messages");
    display.display();
    return;
  }

  // Ensure index is within bounds
  if (index < 0 || index >= count) {
    index = 0;
  }

  // Calculate actual index in messageBuffer
  int actualIndex = (head - count + index + MAX_MESSAGES) % MAX_MESSAGES;

  String message = messageBuffer[actualIndex];

  // Display the message
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Message ");
  display.print(index + 1);
  display.print("/");
  display.println(count);
  display.setCursor(0, 10);
  display.println(message);
  display.display();
}

void handleButtonPress(int buttonIndex) {
  unsigned long currentTime = millis();

  // Check if button is pressed
  if (digitalRead(buttonPins[buttonIndex]) == LOW) {
    if (!buttonPressed[buttonIndex] && (currentTime - lastPressTime[buttonIndex] > bounceDelay)) {
      buttonPressed[buttonIndex] = true;  // Button press detected
      buttonPressTime[buttonIndex] = currentTime;
      longPressFlag[buttonIndex] = false; // Reset long press flag for a new press
    }

    // Check for long press
    if (buttonPressed[buttonIndex] && !longPressFlag[buttonIndex] && (currentTime - buttonPressTime[buttonIndex] > longPressThreshold)) {
      longPressFlag[buttonIndex] = true; // Long press detected
      handleLongPress(buttonIndex);
    }
  } else {
    // Button is released
    if (buttonPressed[buttonIndex] ) {
      if (!longPressFlag[buttonIndex]) {
        handleSinglePress(buttonIndex); // Single press detected
      }
      buttonPressed[buttonIndex] = false;      // Reset for next press
      lastPressTime[buttonIndex] = currentTime; // Update last press time
      longPressFlag[buttonIndex] = false;      // Reset long press flag after handling
    }

  }
}

void handleSinglePress(int buttonIndex) {
  Serial.println(F("Single Press Detected"));

  if (buttonIndex == 0) {
  
    if (!displayOn) {
      // First single press: activate the screen with a generic display
      displayOn = true;
      display.ssd1306_command(SSD1306_DISPLAYON);
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 10);
      display.println("POCSAG Receiver");
      display.display();
      // Reset currentIndex
      currentIndex = 0;
    } else {
      // Subsequent presses: iterate messages starting with the first
      if (count == 0) {
        // No messages to display
        display.clearDisplay();
        display.setCursor(0, 10);
        display.println("No Messages");
        display.display();
      } else {
        displayMessage(currentIndex);
        // Increment currentIndex
        currentIndex = (currentIndex + 1) % count;
  
      }
    }
  
    // Activate single press indicator
    display.drawCircle(SCREEN_WIDTH - 2, 5, 2, WHITE);
    singlePressIndicatorTime = millis();
    singlePressIndicatorActive = true;
  }
  if (buttonIndex == 1) {
    Serial.println(F("OK pressed short"));
      // Initialize Pager client
      Serial.print(F("[Pager] ReInitializing ... "));
    // Set frequency to 439.9875 MHz and speed to 1200 bps
    offset -= 0.0001; 
    int state = pager.begin(439.9875 + offset, 1200);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
      Serial.println(offset,6);
      Serial.println(baseFreq + offset,6);
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true) { delay(10); } // Halt on failure
    }
  }   
  if (buttonIndex == 2) {
    Serial.println(F("DOWN pressed short"));
  }
  if (buttonIndex == 3) {
    Serial.println(F("ESC pressed short"));
         // Initialize Pager client
      Serial.print(F("[Pager] ReInitializing ... "));
    // Set frequency to 439.9875 MHz and speed to 1200 bps
    offset += 0.0001;
    int state = pager.begin(baseFreq + offset, 1200);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
      Serial.println(offset,6);
      Serial.println(baseFreq + offset,6);
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true) { delay(10); } // Halt on failure
    }
  } 
}

void handleLongPress(int buttonIndex) {
  Serial.println(F("Long Press Detected"));
  if (buttonIndex == 0) {
    // Turn off the display
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    displayOn = false;
    currentIndex = 0;
  
    // Activate long press indicator
    display.drawLine(SCREEN_WIDTH - 3, 15, SCREEN_WIDTH - 1, 15, WHITE);
    display.drawLine(SCREEN_WIDTH - 2, 14, SCREEN_WIDTH - 2, 16, WHITE);
    longPressIndicatorTime = millis();
    longPressIndicatorActive = true;
  }

  if (buttonIndex == 1) {
    Serial.println(F("OK pressed long"));
  }   
  if (buttonIndex == 2) {
    Serial.println(F("DOWN pressed long"));
    int state = radio.transmitDirect(439700000/radio.getFreqStep());
    Serial.println(F("TRANSMITTING  !"));
    Serial.println(state);
    
  }
  if (buttonIndex == 3) {
    Serial.println(F("ESC pressed long"));
  }
  
}

void handleVisualIndicators() {
  // Handle visual indicator for singlePress
  if (singlePressIndicatorActive) {
    if (millis() - singlePressIndicatorTime > 200) { // 200 ms
      // Erase indicator
      display.drawCircle(SCREEN_WIDTH - 2, 5, 2, BLACK);
      singlePressIndicatorActive = false;
    }
  }

  // Handle visual indicator for longPress
  if (longPressIndicatorActive) {
    if (millis() - longPressIndicatorTime > 200) { // 200 ms duration
      // Erase indicator
      display.drawLine(SCREEN_WIDTH - 3, 15, SCREEN_WIDTH - 1, 15, BLACK);
      display.drawLine(SCREEN_WIDTH - 2, 14, SCREEN_WIDTH - 2, 16, BLACK);
      longPressIndicatorActive = false;
    }
  }
}
