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
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

//Settings
Preferences Settings; 

// OLED display configuration
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels
#define OLED_RESET    -1    // Reset pin # (or -1 if sharing ESP32 reset pin)

// Create an instance of the OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define PIN        13
#define NUMPIXELS 1
// 
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// SX1278 module pin configuration
#define NSS_PIN     5     // NSS (Chip Select)
#define RESET_PIN  14     // Reset
// Note: DIO0 and DIO1 are not connected (-1) as per your setup
SX1278 radio = new Module(NSS_PIN, 2, RESET_PIN, 4);

// Receiving packets does not require a DIO2 pin connection in this setup
const int pin = 12;



// Create Pager client instance using the FSK module
PagerClient pager(&radio);

// Variables to store messages
#define MAX_MESSAGES 20
String messageBuffer[MAX_MESSAGES];
int head = 0;          // Points to the next insertion index
int count = 0;         // Number of stored messages
int currentIndex = 0;  // Index of the current message to display
bool displayOn = false; // Indicates whether the display is on or off

bool calMode = false; // Calibration mode if true

bool monitorMode = false; //monitor mode if true

// Buttons config
#define BUTTON_UP_PIN 25 // GPIO pin for the button
#define BUTTON_OK_PIN 16 // GPIO pin for the button
#define BUTTON_DOWN_PIN 26  // GPIO pin for the button
#define BUTTON_ESC_PIN 27 // GPIO pin for the button

float baseFreq = 439.9875;
float offset = 0.005;

//RIC DEFINITIONS
int ricNum = 6;

int rics[10] = {-1,               // 1st private ric
		1111, 1142, 1110, // Signalspielplatz services
		8, 2504,          // Generic/Global services
		-1, -1, -1, -1};   // 2nd & 3rd private ric, 2 spares

int masks[10] = {-1,               // 1st private ric
		-1, -1, -1,  	// Signalspielplatz services
		-1, -1,          // Generic/Global services
		-1, -1, -1, -1};   // 2nd & 3rd private ric, 2 spares

int ricColor[10] = { 0x7F7F7F,               // 1st private ric
		0x7F0000, 0x007F00, 0x00007F,  	// Signalspielplatz services
		-1, -1,          // Generic/Global services
		-1, -1, -1, -1};   // 2nd & 3rd private ric, 2 spares

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

int lookUpRICIndex(int ric) {
  for (int i=0; i < ricNum; i++) {
      if (rics[i] == ric)
        return i;    
  }
}

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

  pixels.begin();
  pixels.setPixelColor(0, 0x000000);
  pixels.show();

// Check persis
Settings.begin("Settings", true);
  
bool isSet = Settings.isKey("RIC0");

Settings.end();


//frequency offset for cal

if (isSet == false) {

// Display initial message
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);// Draw white text
  display.setCursor(0, 10);           // Start at top-left corner
  display.println("Calibration\nmissing");
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
  calMode = true;
 
  //radio.setAFC(true);

  //radio.setAFCBandwidth(20.0);

  // Serial loading of parameters

} else {
  
  Settings.begin("Settings", true);
  baseFreq = Settings.getFloat("baseFreq");
  offset = Settings.getFloat("offset");
  rics[0] = Settings.getInt("RIC0");
  rics[6] = Settings.getInt("RIC6");
  rics[7] = Settings.getInt("RIC7");
  monitorMode = Settings.getBool("monitorMode");
  Settings.end();

  if (monitorMode) {
    for (int i=0;i<10;i++) {
      masks[i] = 0;
    }
  }

// Display initial message
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);// Draw white text
  display.setCursor(0, 10);           // Start at top-left corner
  display.println("ChaosPager");
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

  state = pager.startReceive(pin, (uint32_t*) rics, (uint32_t*) masks, ricNum);
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
}

	//int calState = 0;
String readBytes; 

void calModeLoop() {
  //if (Serial.available() > 0) {
  Serial.println(F("CAL MODE!"));
  while (true) {

do {
	Serial.setTimeout(1000);
	readBytes = Serial.readString();

  Serial.println(readBytes);
  Serial.println(readBytes[0]);

 } while (readBytes[0] != '?');
  

if (readBytes[0] == '?') {

  Serial.println(readBytes);
	      
	if (readBytes[1] == 'S' && readBytes[2] == 'O') {
		Settings.begin("Settings", false);
		sscanf(readBytes.c_str(), "?SO%f", &offset);
		Settings.putFloat("offset", offset);
		Serial.println(F("SET OFFSET"));
		Serial.println(offset);
		Settings.end();
	}
	if (readBytes[1] == 'S' && readBytes[2] == 'F') {
		Settings.begin("Settings", false);
		sscanf(readBytes.c_str(), "?SF%f", &baseFreq);
		Settings.putFloat("baseFreq", baseFreq);
		Serial.println(F("SET FREQ"));
		Serial.println(baseFreq);
		Settings.end();
	}

	if (readBytes[1] == 'S' && readBytes[2] == 'R') {
		Settings.begin("Settings", false);
		sscanf(readBytes.c_str(), "?SR%d:%d:%d", &rics[0], &rics[6], &rics[7]);
		
		Settings.putInt("RIC0", rics[0]);
		Settings.putInt("RIC6", rics[6]);
		Settings.putInt("RIC7", rics[7]);
		Serial.println(F("SET RICS"));
		Serial.println(rics[0]);
		Serial.println(rics[6]);
		Serial.println(rics[7]);
		Settings.end();
	}

  if (readBytes[1] == 'M' && readBytes[2] == 'M') {
		Settings.begin("Settings", false);
 		monitorMode = !monitorMode;
		Settings.putBool("monitorMode", monitorMode);
		Serial.println(F("ENABLE MONITOR MODE"));
		Serial.println(monitorMode);
		Settings.end();
	}


  }

  }

}

void loop() {

    if (calMode)
      calModeLoop();

  Serial.setTimeout(100);
	readBytes = Serial.readString();

  if (readBytes[0] == '+' && readBytes[1] == '+' && readBytes[2] == '+' && readBytes[3] == '\n'){
      calModeLoop();
  }	


  // Check if a POCSAG message is available
  if (pager.available() >= 2) {
    Serial.print(F("[Pager] Received pager data, decoding ... "));
    Serial.print(pager.available());

    //Serial.printf("%X", pager.read());



    // Read the received data into a string
    String str;
    int len = 0; //
    int addr = 0;
    int state = pager.readData(str, len, (uint32_t*) &addr);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
      Serial.print(F("[Pager] RIC:\t"));
      Serial.println(addr);
      pixels.setPixelColor(0, ricColor[lookUpRICIndex(addr)]);
      pixels.show();
      Serial.print(F("[Pager] Size:\t"));
      Serial.println(str.length());
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
      display.println("ChaosPager");
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

  }   
  if (buttonIndex == 2) {
    Serial.println(F("DOWN pressed short"));
  }
  if (buttonIndex == 3) {
    Serial.println(F("ESC pressed short"));
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
          Serial.println(offset,6);
          Serial.println(baseFreq,6);
          Serial.println(rics[0]);
          Serial.println(rics[6]);
          Serial.println(rics[7]);
          Serial.println(monitorMode);

    
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
