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

#define RADIOLIB_DEBUG_PROTOCOL 1
#define RADIOLIB_DEBUG_PORT Serial

// Include necessary libraries
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <esp_task_wdt.h>


//Settings
Preferences Settings; 
#define WDT_TIMEOUT 5

// OLED display configuration
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels
#define OLED_RESET    14    // Reset pin # (or -1 if sharing ESP32 reset pin)

#define BUZZER_PIN 17

// Create an instance of the OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define PIN        13
#define NUMPIXELS 1
// 
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// SX1278 module pin configuration
#define NSS_PIN     5     // NSS (Chip Select)
#define RESET_PIN  14     // Reset

SX1278 radio = new Module(NSS_PIN, 2, RESET_PIN, 4);

// Receiving packets does not require a DIO2 pin connection in this setup
const int pin = 12;

const esp_task_wdt_config_t wdt_config = { WDT_TIMEOUT, 0, false};

// Create Pager client instance using the FSK module
PagerClient pager(&radio);

// Variables to store messages
#define MAX_MESSAGES 20
#define MAX_MESSAGE_LENGTH 80
#define MAX_RIC 0x1FFFFF

typedef struct {
  int ric;
  short function; //not provided by RadioLib yet
  char message[MAX_MESSAGE_LENGTH];  
} fullMsg_t;

//String messageBuffer[MAX_MESSAGES];
//char messageBuffer[MAX_MESSAGE_LENGTH * MAX_MESSAGES];
//char* messagePtr[MAX_MESSAGES];
fullMsg_t Msgs[MAX_MESSAGES];

int head = 0;          // Points to the next insertion index
int count = 0;         // Number of stored messages
int currentIndex = 0;  // Index of the current message to display
bool displayOn = false; // Indicates whether the display is on or off

bool calMode = false; // Calibration mode if true
bool buzzerMuted = true; //buzzer muted if true
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

int rics[11] = {-1,               // 1st private ric
		1111, 1142, 1110, // Signalspielplatz services
		8, 2504,          // Generic/Global services
		-1, -1, -1, -1,   // 2nd & 3rd private ric, 2 spares
    -1};             //default last entry for any RIC
int masks[11] = {-1,               // 1st private ric
		-1, -1, -1,  	// Signalspielplatz services
		-1, -1,          // Generic/Global services
		-1, -1, -1, -1,   // 2nd & 3rd private ric, 2 spares
    monitorMode};             //default last entry for any RIC

int ricColor[11] = { 0x1F1F1F,               // 1st private ric
		0x1F0000, 0x001F00, 0x00001F,  	// Signalspielplatz services
		0, 0,          // Generic/Global services
		0, 0, 0, 0,   // 2nd & 3rd private ric, 2 spares
    0x1F001F};             //default last entry for any RIC


// Tracks whether each RIC slot is "enabled" or not.
bool ricActive[11] = {
  true,  // RIC0 (Device RIC) 
  true,  // RIC1
  true,  // RIC2
  true,  // RIC3
  true,  // RIC4
  true,  // RIC5
  false, // RIC6 (User RIC)
  false, // RIC7
  false, // RIC8
  false,  // RIC9
  false  //ANY RIC
};

//------------ Menu Settings ------------
bool inMenu = false; // Tracks whether we are currently in the menu

// Menu items
char* menuItems[] = {
  "Mute Buzzer",
  "Change RIC IDs",
  "Turn screen off"
};
const int NUM_MENU_ITEMS = 3;
int currentMenuIndex = 0;

// RIC Menu
int activeMask = 0x3f;
bool inRicMenu = false;
int currentRicIndex = 0;
int ricMenuOffset = 0;     // for scrolling (0-6 since 10 lines total, 4 visible)
bool inRicDigitEdit = false;
int editDigitPos = 6;

//--------------------------------------

// Debounce config
const unsigned long bounceDelay = 80; // milliseconds

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
  return 10;
}

void buzzer(int freq, int duration) {
  if (buzzerMuted)
    return;
  float interval = 1000.0/freq;
  
  Serial.println(interval,9);
  esp_task_wdt_reset();
  for (int i=0;i < duration/interval; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(interval);
    esp_task_wdt_reset();
    digitalWrite(BUZZER_PIN, LOW);
    delay(interval);    
  }
}

void saveRic(int idx) {
  char lbl[8];
  sprintf(lbl,"RIC%d", idx);
  Settings.begin("Settings", false);
  Settings.putInt(lbl,rics[idx]);
  Settings.end();
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

  // Check persistent if storage already exists
  Settings.begin("Settings", true);
  
  bool isSet = Settings.isKey("RIC0");

  Settings.end();


  //frequency offset for cal

  if (isSet == false ) {
    //CAL MODE PREPARE

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
    //NORMAL OPERATION PREPARE
  
    //Get persistent data
    Settings.begin("Settings", true);
    baseFreq = Settings.getFloat("baseFreq");
    Serial.print("baseFreq: ");
    Serial.println(baseFreq);

    offset = Settings.getFloat("offset");
    Serial.print("Offset: ");
    Serial.println(offset);

    rics[0] = Settings.getInt("RIC0");
    Serial.print("RIC0: ");
    Serial.println(rics[0]);

    rics[6] = Settings.getInt("RIC6");
    Serial.print("RIC6: ");
    Serial.println(rics[6]);

    rics[7] = Settings.getInt("RIC7");
    Serial.print("RIC7: ");
    Serial.println(rics[7]);

    monitorMode = Settings.getBool("monitorMode");
    ricActive[10] = monitorMode;
    Serial.print("MM: ");
    Serial.println(monitorMode);

    if (Settings.isKey("activeMask")) {
      rics[8] = Settings.getInt("RIC8");
      Serial.print("RIC8: ");
      Serial.println(rics[8]);

      rics[9] = Settings.getInt("RIC9");
      Serial.print("RIC9: ");
      Serial.println(rics[9]);

      activeMask = Settings.getInt("activeMask");
      Serial.print("AM: ");
      Serial.println(activeMask);

      Settings.end();
      for (int i=0;i<10;i++) {
        if ((rics[i] != -1) && (activeMask & (1 << i)) )
	  ricActive[i] = true;
      }

    } else {
      Settings.end();
      activeMask = 0x0;  
      for (int i=0;i<10;i++) {
        if (rics[i] != -1) {
	        ricActive[i] = true;
          activeMask |= 1 << i; 
        }
      }
      Settings.begin("Settings", false);

      Settings.putInt("RIC8", rics[8]);
      Settings.putInt("RIC9", rics[9]);
      Settings.putInt("activeMask", activeMask);

      Settings.end();
	        
    }
    

    //make masks permissive (probably redundant)
    if (monitorMode) {
      for (int i=0;i<10;i++) {
        masks[i] = 0;
      }
    }

     displayChaosPager();

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

    //Don't bother with filters in monitorMode
    if (monitorMode) {
      //state = pager.startReceive(pin, (uint32_t) rics[0], (uint32_t) -1);
      state = pager.startReceive(pin, (uint32_t) rics[0], (uint32_t) 0);
    } else {
      state = pager.startReceive(pin, (uint32_t*) rics, (uint32_t*) masks, ricNum);
    }

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

    pinMode(BUZZER_PIN, OUTPUT);

    // Initialize messageBuffer for testing, should be omitted
    /*for (int i = 0; i < MAX_MESSAGES; i++) {
      messageBuffer[i] = "Test Message " + String(i + 1);
    }*/
    
    //count = MAX_MESSAGES; // Set count to indicate the buffer is full
    count = 0; //empty in this variant
    head = 0;             // Reset head for circular buffer behavior

    esp_task_wdt_init(&wdt_config); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch

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

      //Serial.println(readBytes);
      //Serial.println(readBytes[0]);

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
      if (readBytes[1] == '!') {
	      Settings.begin("Settings", false);
	      Settings.putFloat("offset",  0.005);
        Settings.putFloat("baseFreq", 439.9875);
	      Settings.putInt("RIC0", 1);
	      Settings.putInt("RIC6", 2);
	      Settings.putInt("RIC7", 3);
	      Serial.println(F("Fast Cal Performed"));
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

    esp_task_wdt_reset();

    // Check if a POCSAG message is available
    /*if (pager.available() == 1) {
      Serial.printf("batches: %d\n", pager.available());
    }*/

    if (pager.available() >= 2) {
      Serial.print(F("[Pager] Received pager data, decoding ... \n"));
  
      Serial.printf("batches: %d\n", pager.available());

      //Serial.printf("%X", pager.read());
      //DEBUG BLOCK @ RECEPTION
      Serial.printf("ricNum: %d\n", ricNum);
      for (int i=0;i<10;i++) {
        Serial.printf("RIC%d: %d MASK %d ACT %d\n", i, rics[i], masks[i], ricActive[i]);
      }
    



      // Read the received data into a string
      /*String str;
      int len = 0; //
      int addr = 0;
      int state = pager.readData(str, len, (uint32_t*) &addr);
      */

      char str[320];
      size_t len = 0; //
      int addr = 0;
      int state = pager.readData((uint8_t*) str, &len, (uint32_t*) &addr);

      if (state == RADIOLIB_ERR_NONE) {
      
      //RECEPTION LOG
      int ricIndex = lookUpRICIndex(addr);
      Serial.println(F("success!"));
      Serial.print(F("[Pager] RIC:\t"));
      Serial.println(addr);
      Serial.print(F("[Pager] IDX:\t"));
      Serial.println(ricIndex);
      //pixels.setPixelColor(0, ricColor[ricIndex]);
      //pixels.show();
      Serial.print(F("[Pager] Size(return):\t"));
      Serial.println(len);
      Serial.print(F("[Pager] Size(string):\t"));
      //Serial.println(str.length());
      Serial.println(strlen(str));
      Serial.print(F("[Pager] Data:\t"));
      Serial.println(str);

      if (ricActive[ricIndex]) {
        // Display the message

        pixels.setPixelColor(0, ricColor[ricIndex]);
        pixels.show();

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print("RIC: ");
        display.print(addr);
        display.setCursor(0, 10);
        display.print("Message: ");
        display.println(str);
        display.display();

        // Store the received message
        Msgs[head].ric = addr;
        Msgs[head].function = 0;
        strncpy(Msgs[head].message, str, len);
        Serial.print(F("Message stored\n"));
        head = (head + 1) % MAX_MESSAGES;
        if(count < MAX_MESSAGES)
          count++;
      
        //storeMessage(str);
        buzzer(440,500);
        buzzer(880,500);
        buzzer(440,500);
        buzzer(880,500);
      }
      delay(100);

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

/*void storeMessage(String newMessage) {
  messageBuffer[head] = newMessage;
  head = (head + 1) % MAX_MESSAGES;
  if (count < MAX_MESSAGES) {
    count++;
  }
}*/

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

  //String message = messageBuffer[actualIndex];
  pixels.setPixelColor(0, ricColor[lookUpRICIndex(Msgs[index].ric)]);
  pixels.show();

  // Display the message
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Msg ");
  display.print(index + 1);
  display.print("/");
  display.print(count);
  display.print(" RIC: ");
  display.println(Msgs[index].ric);
  display.setCursor(0, 10);
  display.println(Msgs[index].message);
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

  // If we are in a menu, defer to the menu handler
  if (inMenu) {
    handleMenuButtonPress(buttonIndex);
    return; 
  }

  if (inRicMenu) {
    handleRicMenuButtonPress(buttonIndex);
    return;
  }

  if (buttonIndex == 0) { // UP Button
    if (!displayOn) {
      // First single press: activate the screen with a generic display
      displayOn = true;
      display.ssd1306_command(SSD1306_DISPLAYON);
      displayChaosPager(); // Display "ChaosPager"
      currentIndex = 0; // Reset currentIndex
    } else if (count > 0) {
      // Increment currentIndex with wrap-around
      currentIndex = (currentIndex + 1) % count;
      displayMessage(currentIndex); // Display the next message
    } else {
      // No messages to display
      display.clearDisplay();
      display.setCursor(0, 10);
      display.println("No Messages");
      display.display();
    }

    // Activate single press indicator
    display.drawCircle(SCREEN_WIDTH - 2, 5, 2, WHITE);
    singlePressIndicatorTime = millis();
    singlePressIndicatorActive = true;
  }

  else if (buttonIndex == 2) { // DOWN Button
    if (!displayOn) {
      // If display is off, activate it and show "ChaosPager"
      displayOn = true;
      display.ssd1306_command(SSD1306_DISPLAYON);
      displayChaosPager(); // Display "ChaosPager"
      currentIndex = 0; // Reset currentIndex
    } else if (count > 0) {
      // Decrement currentIndex with wrap-around
      currentIndex = (currentIndex - 1 + count) % count;
      displayMessage(currentIndex); // Display the previous message
    } else {
      // No messages to display
      display.clearDisplay();
      display.setCursor(0, 10);
      display.println("No Messages");
      display.display();
    }

    // Activate single press indicator
    display.drawCircle(SCREEN_WIDTH - 2, 5, 2, WHITE);
    singlePressIndicatorTime = millis();
    singlePressIndicatorActive = true;
  }

  else if (buttonIndex == 1) { // OK Button
    Serial.println(F("OK pressed short"));
    // Existing logic for OK button remains unchanged
  }

  else if (buttonIndex == 3) { // ESC Button
    Serial.println(F("ESC pressed short"));
    // Existing logic for ESC button remains unchanged
  }
}


void handleLongPress(int buttonIndex) {
  Serial.println(F("Long Press Detected"));

  // Handle long presses within RIC Menu
  if (inRicMenu) {
    // Only RICs 6-9 can be edited via long press
    if (currentRicIndex >= 6 && currentRicIndex <= 9 && ricActive[currentRicIndex]) {
      // Enter Digit Edit Mode
      inRicDigitEdit = true;
      editDigitPos = 6; // Start with most significant digit
      Serial.print("Entering Digit Edit for RIC ");
      Serial.println(currentRicIndex);
      drawRicEditScreen();
      return; // Exit to prevent further handling
    }
  }


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
    // Enter Menu
    Serial.println(F("OK pressed long -> Entering Menu"));
    inMenu = true;
    currentMenuIndex = 0;
    drawMenu(currentMenuIndex);

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

// Helper Function to Display "ChaosPager"
void displayChaosPager() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10); // Adjust as needed for proper centering
  display.println("ChaosPager");
  display.display();
}

// ---------------------------- Menu -----------------------------------
void drawMenu(int selectedIndex) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Print each menu item; highlight the selected one with ">"
  for (int i = 0; i < NUM_MENU_ITEMS; i++) {
    display.setCursor(0, i * 10); 
    if (i == selectedIndex) {
      display.print("> ");
    } else {
      display.print("  ");
    }
    display.println(menuItems[i]);
  }

  display.display();
}

// ------------------------------------------------------------------------
// Menu navigation and execution 
// ------------------------------------------------------------------------
void handleMenuButtonPress(int buttonIndex) {
  switch (buttonIndex) {
    case 0: // UP
      currentMenuIndex--;
      if (currentMenuIndex < 0) {
        currentMenuIndex = NUM_MENU_ITEMS - 1;
      }
      drawMenu(currentMenuIndex);
      break;

    case 1: // OK
      // Perform the action of the selected item
      if (currentMenuIndex == 0) {
        // Mute Buzzer
        buzzerMuted = !buzzerMuted; 
        Serial.print("Buzzer muted? ");
        Serial.println(buzzerMuted);
	if (buzzerMuted) {
		menuItems[0] = "Unmute Buzzer";
	} else {
		menuItems[0] = "Mute Buzzer";
	}	
		
        // ? "YES" : "NO");
      }
      else if (currentMenuIndex == 1) {
      // "Change RIC IDs"
        Serial.println("Entering RIC Menu");
        enterRicMenu();
      }
      else if (currentMenuIndex == 2) {
        // Exit Menu
        inMenu = false;
        Serial.println("Exiting Menu");
        display.clearDisplay();
        display.display();
      }

      // If still in the menu, redraw
      if (inMenu) {
        drawMenu(currentMenuIndex);
      }
      break;

    case 2: // DOWN
      currentMenuIndex++;
      if (currentMenuIndex >= NUM_MENU_ITEMS) {
        currentMenuIndex = 0;
      }
      drawMenu(currentMenuIndex);
      break;

    case 3: // ESC
      inMenu = false;
      Serial.println("Exiting Menu (ESC)");
      displayChaosPager();
      currentIndex = 0;
      break;
  }
}



// RIC MENU (called from handleMenuButtonPress "Change RIC IDs")
void enterRicMenu() {
  inMenu = false;      // Leave the main menu
  inRicMenu = true;    // Enter the RIC menu
  currentRicIndex = 0;
  ricMenuOffset = 0;
  drawRicMenu();       // Draw from offset=0
}

void drawRicMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Show up to 4 lines from ricMenuOffset to ricMenuOffset+3
  for (int line = 0; line < 4; line++) {
    int i = ricMenuOffset + line;  // actual RIC index for this line
    if (i > 10) break;              // in case offset is near the end

    display.setCursor(0, line * 8);  // each line 8 px high

    // Draw ">" if this line is selected
    if (i == currentRicIndex) {
      display.print("> ");
    } else {
      display.print("  ");
    }

    if (i <= 9){
    // Show label
    display.print("RIC");
    display.print(i);
    display.print(": ");
    
    // RIC0 => Device RIC
    if (i == 0) {
      display.print(rics[i]);
      display.print(" Device ");
    } else {
      // If toggled off => -1 for [6..9], or ricActive[i]==false for [1..5]
      if (!ricActive[i]) {
        if (i <= 5) {
          // For 1..5, we keep the number but are "OFF" in a sense
          display.print(rics[i]);
          display.print(" (OFF)");
        } else {
          // For 6-9 -1 is set  => effectively "OFF"
          display.print(" (OFF)");
        }
      } else {
        // RIC is active, print its value
        display.print(rics[i]);
      }
    }
    } else if(i == 10){
      // Display "Toggle Monitor Mode"
      display.print("Monitor Mode: ");
      display.print(monitorMode ? "ON" : "OFF");
    }
  }

  display.display();
}


// Scrolling helper for RIC Menu
void scrollIfNeeded() {
  // If currentRicIndex is beyond the bottom of visible window
  if (currentRicIndex > ricMenuOffset + 3) {
    ricMenuOffset = currentRicIndex - 3;
  }
  // If currentRicIndex is above the top of visible window
  if (currentRicIndex < ricMenuOffset) {
    ricMenuOffset = currentRicIndex;
  }
  // Constrain offset so we don't go negative or too high
  if (ricMenuOffset < 0) ricMenuOffset = 0;
  if (ricMenuOffset > 7) ricMenuOffset = 7;  // have 11 lines => last offset is 7
}


// HANDLE BUTTONS IN RIC MENU (Setting RIC)
void handleRicMenuButtonPress(int buttonIndex) {
  if (inRicDigitEdit) {
    handleRicDigitEditButtonPress(buttonIndex);
    return;
  }

  switch (buttonIndex) {
    case 0: // UP
      currentRicIndex--;
      if (currentRicIndex < 0) currentRicIndex = 10;
      scrollIfNeeded();
      drawRicMenu();
      break;

    case 2: // DOWN
      currentRicIndex++;
      if (currentRicIndex > 10) currentRicIndex = 0;
      scrollIfNeeded();
      drawRicMenu();
      break;

    case 1: // OK => Toggle
      // RIC0 => do nothing (Device RIC locked for toggling)
      if (currentRicIndex == 0 || currentRicIndex == 4 || currentRicIndex == 5 ) {
        Serial.println("RIC0, RIC4 & RIC5 is always on. (Device RIC)");
      }
      else if (currentRicIndex <= 9){
        // If it's RIC[1..5]
        if (currentRicIndex <= 3) {
          // Flip ricActive
          ricActive[currentRicIndex] = !ricActive[currentRicIndex];
          Serial.print("Toggled RIC");
          Serial.print(currentRicIndex);
          Serial.print(" => ");
          Serial.println(ricActive[currentRicIndex] ? "ON" : "OFF");
        }
        // If it's RIC[6..9]
        else {
          if (!ricActive[currentRicIndex]) {
            // Turn it on => pick default number if -1
            ricActive[currentRicIndex] = true;
            if (rics[currentRicIndex] == -1) {
              rics[currentRicIndex] = 0000; 
            }
            Serial.print("Enabled RIC");
            Serial.println(currentRicIndex);
          } 
          else {
            // Turn it off => set to -1
            ricActive[currentRicIndex] = false;
            //rics[currentRicIndex] = -1;
            Serial.print("Disabled RIC");
            Serial.println(currentRicIndex);
          }}
      }else if (currentRicIndex == 10) {
        // Handle "Toggle Monitor Mode"
        monitorMode = !monitorMode; // Flip the boolean
        ricActive[10] = monitorMode;
        Settings.begin("Settings", false);
        Settings.putBool("monitorMode", monitorMode);
        Settings.end();
        Serial.print("Monitor Mode set to ");
        Serial.println(monitorMode ? "ON" : "OFF");
      }
      drawRicMenu();
      break;

    case 3: // ESC => Turn display off
      inRicMenu = false;
      inMenu = true;        // Enter Main Menu
      reorderRics69();  // reorder indices 6-9
      updateRicNum();
      updateActiveMask();
      Serial.println("Exiting RIC Menu and returning to Main Menu");
      // Draw the main menu
      drawMenu(currentMenuIndex);
      break;
  }
}

//Reorder the User RIC's in position 6-9
void reorderRics69() {
  // We'll bubble sort among [6..9] so that -1s go to the end
  for (int i = 6; i < 9; i++) {
    for (int j = i+1; j < 10; j++) {
      if (rics[i] == -1 && rics[j] != -1) {
        // swap
        int tmpVal = rics[i];
        rics[i] = rics[j];
        rics[j] = tmpVal;

        bool tmpAct = ricActive[i];
        ricActive[i] = ricActive[j];
        ricActive[j] = tmpAct;
      }
    }
  }
}

// UPDATE ricNum = count active RICs (not -1)
void updateRicNum() {
  int cnt = 0;
  for (int i = 0; i < 10; i++) {
    if (i <= 5) {
      // [0..5] are always "numeric" => count if active
      if (ricActive[i]) cnt++;
    } else {
      // [6..9] => only count if active (rics[i] != -1)
      if (ricActive[i] && rics[i] != -1) cnt++;
    }
  }
  ricNum = cnt;
  Serial.print("ricNum updated to ");
  Serial.println(ricNum);
}

void updateActiveMask() {
      for (int i=0;i<10;i++) {
        if (ricActive[i]) {
          activeMask |= 1 << i; 
        }
      }
      Settings.begin("Settings", false);

      Settings.putInt("activeMask", activeMask);

      Settings.end();
}

// RIC DIGIT EDITING
void handleRicDigitEditButtonPress(int buttonIndex) {
  // RIC must be active
  if (!ricActive[currentRicIndex]) {
    // Just in case
    inRicDigitEdit = false;
    drawRicMenu();
    return;
  }

  int currentValue = (rics[currentRicIndex] == -1) ? 0 : rics[currentRicIndex];

  switch (buttonIndex) {
    case 0: // UP => increment digit
    {
      int d = getRicDigit(currentValue, editDigitPos);
      d = (d + 1) % 10;
      rics[currentRicIndex] = setRicDigit(currentValue, editDigitPos, d);
      drawRicEditScreen();
      break;
    }
    case 2: // DOWN => decrement digit
    {
      int d = getRicDigit(currentValue, editDigitPos);
      d = (d - 1 + 10) % 10;
      rics[currentRicIndex] = setRicDigit(currentValue, editDigitPos, d);
      drawRicEditScreen();
      break;
    }
    case 1: // OK => prev digit
      editDigitPos++;
      if (editDigitPos > 6) {
        inRicDigitEdit = false;
        Serial.print("Done editing RIC ");
        Serial.println(currentRicIndex);
        saveRic(currentRicIndex);
        drawRicMenu();
      } else {
        drawRicEditScreen();
      }
      break;
    case 3: // ESC => next digit
      editDigitPos--;
      if (editDigitPos < 0) {
        inRicDigitEdit = false;
      	Serial.println("Cancelled digit editing");
        saveRic(currentRicIndex);
        drawRicMenu();
      } else {
        drawRicEditScreen();
      }
      break;
  }
}

// Basic helper to get a digit (least significant digit = position 0)
int getRicDigit(int value, int pos) {
  while (pos--) {
    value /= 10;
  }
  return value % 10;
}

// Basic helper to set a digit (least significant digit = position 0)
int setRicDigit(int original, int pos, int newDigit) {
  if (original < 0) original = 0;
  int power = 1;
  for (int i = 0; i < pos; i++) power *= 10;
  int oldDigit = (original / power) % 10;
  int newRIC = original - oldDigit*power + newDigit*power;
  Serial.printf("Old: %d New %d %d", original, newRIC, MAX_RIC);
  return (newRIC > MAX_RIC) ? original : newRIC;
}

void drawRicEditScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  int val = (rics[currentRicIndex] < 0) ? 0 : rics[currentRicIndex];
  char buf[7];
  // 5 digits max => zero-padded
  sprintf(buf, "%07d", val);

  display.setCursor(0, 0);
  display.print("Editing RIC");
  display.println(currentRicIndex);

  display.setCursor(0, 10);
  display.println("UP + DN - OK < ESC >");

  // Show digits right to left so pos=0 is the LSD
  display.setCursor(0, 20);
  for (int i = 0; i < 7; i++) {
    int idx =  i; // LSD is buf[0]
    if (6-i == editDigitPos) {
      display.print("[");
      display.print(buf[idx]);
      display.print("]");
    } else {
      display.print(" ");
      display.print(buf[idx]);
      display.print(" ");
    }
  }

  display.display();
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

  // Handle visual indicator for longPressmonitorMode
  if (longPressIndicatorActive) {
    if (millis() - longPressIndicatorTime > 200) { // 200 ms duration
      // Erase indicator
      display.drawLine(SCREEN_WIDTH - 3, 15, SCREEN_WIDTH - 1, 15, BLACK);
      display.drawLine(SCREEN_WIDTH - 2, 14, SCREEN_WIDTH - 2, 16, BLACK);
      longPressIndicatorActive = false;
    }
  }
}
