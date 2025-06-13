#include <WiFiManager.h>
#include <TFT_eSPI.h>
#include "Fonts/SegoeUI_Bold_48.h"
#include "Fonts/NotoSansBold36.h"
#include <ESP_I2S.h>
// Graphics and font library for ST7735 driver chip
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <BlynkSimpleEsp32.h>
#include <ArduinoOTA.h>

#include <BlynkSimpleEsp32.h>
#include <SPI.h>
#include <SteinhartHart.h>
#include <ESP32I2SAudio.h>
#include <WiFi.h>

#include <BackgroundAudio.h>
#include <LittleFS.h>
//#include <JohnnyCash.h>
//#include <eatit.h>
#define STASSID "mikesnet"
#define STAPSK "springchicken"
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
//#include <BackgroundAudioSpeech.h>
//#include <libespeak-ng/voice/en.h>


#define I2C_ADDRESS 0x48
#define AA_FONT_SMALL NotoSansBold36
#define AA_FONT_LARGE SegoeUI_Bold_48
//String temp1string,temp2string,temp3string;
Adafruit_ADS1115 adc; 
bool connected = false;
bool saved = false;
const char* ssid = STASSID;
const char* pass = STAPSK;
unsigned long reconnectTime;
ESP32I2SAudio audio(2, 3, 0);  // BCLK, LRCLK, DOUT (,MCLK)
unsigned long tempAlarmStart = 0;
bool tempAlarmActive = false;
BackgroundAudioWAV BMP(audio);
//BackgroundAudioSpeech BMP(audio);

char auth[] = "DU_j5IxaBQ3Dp-joTLtsB0DM70UZaEDd";

#define rotLpin 21
#define rotRpin 20
#define rotbutton 5
#define onewirepin 7
#define is2connectedthreshold 23000
#define ARDUINO_TASK_STACK_SIZE 8192

OneWire oneWire(onewirepin);
DallasTemperature sensors(&oneWire);

float minsLeft;
int animStep = 1;
float onewiretempC, tempF, tempA0, tempA1, tempA0f, tempA1f, barx;
long adc0, adc1, adc2, adc3;
long therm1, therm2, therm3;  //had to change to long since uint16 wasn't long enough
float temp1, temp2, temp3;
float volts0, volts1, volts2, volts3;
int channel = 1;
bool onewirefound = false;
bool calibrationMode = false;
bool playSound = false;
bool isPlaying = false;
bool bmp1began = false;
bool bmpdone = true;
int animpos = 80;
static char settempstring[192];
static char etastring[192];
static char v2String[192];
static char battString[192];
static char dallasString[192];
static char temp1string[192], temp2string[192], temp3string[192];
static char coeffAstring[192], coeffBstring[192], coeffCstring[192];
static char sampleString[192];
SteinhartHart thermistor(15062.08, 36874.80, 82837.54, 348.15, 323.15, 303.15);  //these are the default values for a Weber probe


TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
TFT_eSprite img = TFT_eSprite(&tft);
static byte abOld;           // Initialize state
volatile int count = 580;    // current rotary count
volatile int downcount = 0;  // current rotary count
volatile int upcount = 0;    // current rotary count
int old_count;               // old rotary count
volatile int enc_count = 0;

#define TFT_GREY 0x5AEB  // New colour

File f;
const size_t bufsize = 256;
 uint8_t buf[bufsize];

void waitForButtonsReleased() {
  while (!digitalRead(rotbutton)) {
    delay(10);  // debounce delay
  }
}

File audioFile;
bool isAudioPlaying = false;

void playWavFromFS(const char* filename) {
  Serial.println("Playing WAV file from LittleFS...");
  if (isAudioPlaying) return; // Already playing

  audioFile = LittleFS.open(filename, "r");
  if (!audioFile) {
    Serial.println("Failed to open WAV file!");
    return;
  }
  Serial.println("Flushing...");
  BMP.flush();
  Serial.println("Flushed.");
  isAudioPlaying = true;
}

void continueAudioPlayback() {
  if (!isAudioPlaying) return;
  if (!audioFile) {
    isAudioPlaying = false;
    return;
  }
  if (audioFile.available()) {
    if (BMP.availableForWrite() >= bufsize) {
      int len = audioFile.read(buf, bufsize);
      if (len > 0) {
        BMP.write(buf, len);
      }
    }
  } else {
    audioFile.close();
    isAudioPlaying = false;
    Serial.println("Audio playback finished.");
  }
}


float estimateBatteryTime(float voltage) {
  const int numPoints = 22;
  // Move these to static to avoid stack allocation on each call
  static const float voltages[numPoints] = {
    4.0862, 4.0399, 3.9740, 3.9136, 3.8576, 3.8075, 3.7614, 3.7199,
    3.6831, 3.6469, 3.6142, 3.5795, 3.4959, 3.4269, 3.3703, 3.3360,
    3.3160, 3.2935, 3.2665, 3.2400, 3.2240, 3.2140
  };
  static const float times[numPoints] = {
    1380, 1320, 1260, 1200, 1140, 1080, 1020, 960,
    900, 840, 780, 660, 360, 240, 97, 34,
    24, 18, 10, 3, 1, 0
  };

  if (voltage >= voltages[0]) return times[0];
  if (voltage <= voltages[numPoints - 1]) return 0;

  for (int i = 0; i < numPoints - 1; i++) {
    if (voltage <= voltages[i] && voltage > voltages[i + 1]) {
      float v1 = voltages[i];
      float v2 = voltages[i + 1];
      float t1 = times[i];
      float t2 = times[i + 1];
      return t1 + (voltage - v1) * (t2 - t1) / (v2 - v1);
    }
  }

  return 0;  // fallback
}



uint16_t cmap[24];
const char* cmapNames[24];


void initializeCmap() {
  cmap[0] = TFT_BLACK;
  cmapNames[0] = "BLACK";
  cmap[1] = TFT_NAVY;
  cmapNames[1] = "NAVY";
  cmap[2] = TFT_DARKGREEN;
  cmapNames[2] = "DARKGREEN";
  cmap[3] = TFT_DARKCYAN;
  cmapNames[3] = "DARKCYAN";
  cmap[4] = TFT_MAROON;
  cmapNames[4] = "MAROON";
  cmap[5] = TFT_PURPLE;
  cmapNames[5] = "PURPLE";
  cmap[6] = TFT_OLIVE;
  cmapNames[6] = "OLIVE";
  cmap[7] = TFT_LIGHTGREY;
  cmapNames[7] = "LIGHTGREY";
  cmap[8] = TFT_DARKGREY;
  cmapNames[8] = "DARKGREY";
  cmap[9] = TFT_BLUE;
  cmapNames[9] = "BLUE";
  cmap[10] = TFT_GREEN;
  cmapNames[10] = "GREEN";
  cmap[11] = TFT_CYAN;
  cmapNames[11] = "CYAN";
  cmap[12] = TFT_RED;
  cmapNames[12] = "RED";
  cmap[13] = TFT_MAGENTA;
  cmapNames[13] = "MAGENTA";
  cmap[14] = TFT_YELLOW;
  cmapNames[14] = "YELLOW";
  cmap[15] = TFT_WHITE;
  cmapNames[15] = "WHITE";
  cmap[16] = TFT_ORANGE;
  cmapNames[16] = "ORANGE";
  cmap[17] = TFT_GREENYELLOW;
  cmapNames[17] = "GREENYELLOW";
  cmap[18] = TFT_PINK;
  cmapNames[18] = "PINK";
  cmap[19] = TFT_BROWN;
  cmapNames[19] = "BROWN";
  cmap[20] = TFT_GOLD;
  cmapNames[20] = "GOLD";
  cmap[21] = TFT_SILVER;
  cmapNames[21] = "SILVER";
  cmap[22] = TFT_SKYBLUE;
  cmapNames[22] = "SKYBLUE";
  cmap[23] = TFT_VIOLET;
  cmapNames[23] = "VIOLET";
}


double oldtemp, tempdiff, eta, eta2, oldtemp2, tempdiff2;
int etamins, etasecs;

int settemp = 145;
bool is1connected = false;
bool is2connected = false;
int setSelection = 0;
int setAlarm, setUnits;
int setBGC = 15;  //background color
int setFGC = 0;
int setVolume = 100;

int rssi;

bool b1pressed = false;
bool b2pressed = false;
bool settingspage = false;
bool kincreased = false;
int setIcons = 1;

Preferences preferences;

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

#define every(interval) \        
  static uint32_t __every__##interval = millis(); \
  if (millis() - __every__##interval >= interval && (__every__##interval = millis()))



void drawWiFiSignalStrength(int32_t x, int32_t y, int32_t radius) {  //chatGPT-generated function to draw a wifi icon with variable rings
  // Get the RSSI value


  // Define colors
  uint32_t color;
  int numArcs;

  // Determine the color and number of arcs to draw based on RSSI value
  if (rssi > -60) {
    color = cmap[setFGC];
    numArcs = 3;
  } else if (rssi > -75) {
    color = cmap[setFGC];
    numArcs = 3;
  } else if (rssi > -85) {
    color = cmap[setFGC];
    numArcs = 3;
  } else {
    color = cmap[setFGC];
    numArcs = 1;
  }

  // Draw the base circle/dot
  img.fillCircle(x, y + 1, 1, color);

  // Draw arcs based on the determined number of arcs and color
  if (numArcs >= 1) {
    img.drawArc(x, y, radius / 3, radius / 3 - 1, 135, 225, color, cmap[setBGC]);  // Arc 1
  }
  if (numArcs >= 2) {
    img.drawArc(x, y, 2 * radius / 3, 2 * radius / 3 - 1, 135, 225, color, cmap[setBGC]);  // Arc 2
  }
  if (numArcs >= 3) {
    img.drawArc(x, y, radius, radius - 1, 135, 225, color, cmap[setBGC]);  // Arc 3
  }
}

double ADSToOhms(int16_t ADSreading) { //convert raw ADS reading to a measured resistance in ohms, knowing R1 is 22000 ohms
      float voltsX = adc.computeVolts(ADSreading);
      return (voltsX * 22000) / (3.3 - voltsX);
}


void drawTemps() {  //main screen

  if (!digitalRead(rotbutton)) {  //if both are pressed at the same time
    settingspage = true;          //go to settings page
    enc_count = count;
    waitForButtonsReleased();
  }


  img.fillSprite(cmap[setBGC]);
  img.setCursor(0, 0);
  img.setTextSize(1);
  img.setTextColor(cmap[setFGC], cmap[setBGC]);
  img.setTextDatum(TC_DATUM);

  // Proportional scaling factors
  // 240x240 -> 128x160: x = x * 0.533, y = y * 0.666
  // Example: 240 -> 128, 240 -> 160
  img.setTextFont(8);  //smaller font for 128x160
  if (is2connected && is1connected) {

    float a0ex = 164.4;
    float a1ex = 288.8;
    img.drawFloat(tempA0f, 1, 120, 3);   // 60,5 -> 32,3
    img.drawFloat(tempA1f, 1, 120, 90);  // 180,5 -> 96,3
    //img.drawFastVLine(64, 0, 56, cmap[setFGC]); // 120,0,85 -> 64,0,56
  } else if (is1connected) {
    img.drawFloat(tempA0f, 1, 120, 42);  // 115,5 -> 56,3
  } else if (!is1connected && is2connected) {
    img.drawFloat(tempA1f, 1, 120, 42);  // 115,5 -> 56,3
  } else {
    img.loadFont(AA_FONT_LARGE);               //use a smaller font
    img.drawString("N/C", 120, 22, 1);  // 115,5 -> 56,3
    img.unloadFont();
  }
  img.drawFastHLine(0, 175, 240, cmap[setFGC]);  // 0,85,240 -> 0,56,128
  img.unloadFont();
  img.setTextFont(1);
  img.setTextDatum(TR_DATUM);

  // Degrees/unit indicator
  if (setUnits == 0) {
    img.drawCircle(229, 2, 1, cmap[setFGC]);
    img.drawString("C", 239, 1);
  }  //Celsius
  else if (setUnits == 1) {
    img.drawCircle(229, 2, 1, cmap[setFGC]);
    img.drawString("F", 239, 1);
  }                                                         //Farenheit
  else if (setUnits == 2) { img.drawString("K", 239, 1); }  //don't draw the degrees symbol if we're in Kelvin


  img.setTextDatum(TL_DATUM);
  //img.setFreeFont(AA_FONT_LARGE);
  img.loadFont(AA_FONT_LARGE);
  img.setCursor(3, 175);  // 5,100+24 -> 3,70

  snprintf(settempstring, sizeof(settempstring), ">%d<", count / 4);

  //img.print("Set:");
  img.setTextDatum(TC_DATUM);
  img.drawString(settempstring, 120, 190);

  img.setTextDatum(TC_DATUM);
  //img.unloadFont();
  //img.loadFont(AA_FONT_SMALL);
  img.setCursor(120, 195 + 24);
  //img.drawString("ETA:", 120, 219, 1);

  if ((etamins < 1000) && (etamins >= 0)) {
    snprintf(etastring, sizeof(etastring), "%dmins", etamins);
  } else {
    snprintf(etastring, sizeof(etastring), "---mins");
  }
  img.setTextDatum(BC_DATUM);
  img.drawString(etastring, 120, 300);
  img.unloadFont();
  img.setTextFont(1);

  img.drawFastHLine(0, 320 - 14, 240, cmap[setFGC]);  // 0,226,240 -> 0,150,128

  if (setIcons == 0) {
    img.setCursor(0, 320 - 9);  // 1,231
    img.print(WiFi.localIP());

    snprintf(v2String, sizeof(v2String), "%ddB/%.2fv", rssi, volts2);
    img.setTextDatum(BR_DATUM);
    img.drawString(v2String, 239, 319);  // 239,239 -> 127,159
  } else if (setIcons == 1) {
    img.setCursor(0, 320 - 9);  // 1,231
    img.print(WiFi.localIP());
    img.setTextDatum(BR_DATUM);
    img.drawRect(212, 309, 20, 9, cmap[setFGC]);
    img.fillRect(212, 309, barx, 9, cmap[setFGC]);
    img.drawFastVLine(232, 311, 4, cmap[setFGC]);
    if (WiFi.status() == WL_CONNECTED) { drawWiFiSignalStrength(198, 316, 6); }  // 200,237,9 -> 92,157,6
  } else if (setIcons == 2) {

    int hours = minsLeft / 60;
    int mins = (int)minsLeft % 60;
    img.setCursor(0, 312);
    snprintf(battString, sizeof(battString), "Batt: %dh:%dm", hours, mins);
    img.printf("%s", battString);
    img.setTextDatum(BR_DATUM);
    img.drawRect(212, 309, 20, 9, cmap[setFGC]);
    img.fillRect(212, 309, barx, 9, cmap[setFGC]);
    img.drawFastVLine(232, 311, 4, cmap[setFGC]);
    if (WiFi.status() == WL_CONNECTED) { drawWiFiSignalStrength(198, 316, 6); }
  }

  img.fillRect(animpos, 312, 4, 4, cmap[setFGC]);  //draw our little status indicator
  animpos += 2;                                    //make it fly
  if (animpos > 160) { animpos = 130; }            //make it wrap around
  img.pushSprite(0, 0);
}

volatile bool rotLeft = false;   // Flag for left rotation
volatile bool rotRight = false;  // Flag for right rotation


void pinChangeISR() {
  enum { upMask = 0x66,
         downMask = 0x99 };
  byte abNew = (digitalRead(rotRpin) << 1) | digitalRead(rotLpin);
  byte criterion = abNew ^ abOld;
  if (criterion == 1 || criterion == 2) {
    if (upMask & (1 << (2 * abOld + abNew / 2))) {
      count++;
    } else {
      count--;  // upMask = ~downMask
    }
  }

  abOld = abNew;  // Save new state
}

void doADC() {  //take measurements from the ADC without blocking
  if (adc.conversionComplete()) {
    if (channel == 0) {
      adc0 = adc.getLastConversionResults();
      if (setUnits == 0) {tempA0f = thermistor.resistanceToTemperature(ADSToOhms(adc0)) - 273.15;}
      else if (setUnits == 1) {
        tempA0 = thermistor.resistanceToTemperature(ADSToOhms(adc0)) - 273.15;
        tempA0f = (tempA0 * 1.8) + 32;
      }
      else if (setUnits == 2) {tempA0f = thermistor.resistanceToTemperature(ADSToOhms(adc0));}

      adc.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_1, false);
      channel = 1;
      return;
    }
    if (channel == 1) {
      adc1 = adc.getLastConversionResults();
      if (setUnits == 0) {tempA1f = thermistor.resistanceToTemperature(ADSToOhms(adc1)) - 273.15;}
      else if (setUnits == 1) {
        tempA1 = thermistor.resistanceToTemperature(ADSToOhms(adc1)) - 273.15;
        tempA1f = (tempA1 * 1.8) + 32;
      }
      else if (setUnits == 2) {tempA1f = thermistor.resistanceToTemperature(ADSToOhms(adc1));}
      adc.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_2, false);
      channel = 2;
      return;
    }
    if (channel == 2) {  //channel for measuring battery voltage
      adc2 = adc.getLastConversionResults();
      adc.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
      channel = 0;
      return;
    }
  }
  

  

}



double mapf(float x, float in_min, float in_max, float out_min, float out_max)  //like default map function but returns float instead of int
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void forceADC() {  //force an ADC measurement and block the CPU to do it
  adc0 = adc.readADC_SingleEnded(0);
  adc1 = adc.readADC_SingleEnded(1);
  adc2 = adc.readADC_SingleEnded(2);
  volts2 = adc.computeVolts(adc2) * 2.0;  //battery voltage measurement
  //volts2 = maxlipo.cellVoltage();
  //adc0 = adc.getLastConversionResults();
  if (setUnits == 0) {tempA0f = thermistor.resistanceToTemperature(ADSToOhms(adc0)) - 273.15; tempA1f = thermistor.resistanceToTemperature(ADSToOhms(adc1)) - 273.15;}
  else if (setUnits == 1) {
    tempA0 = thermistor.resistanceToTemperature(ADSToOhms(adc0)) - 273.15;
    tempA0f = (tempA0 * 1.8) + 32;
    tempA1 = thermistor.resistanceToTemperature(ADSToOhms(adc1)) - 273.15;
    tempA1f = (tempA1 * 1.8) + 32;
  }
  else if (setUnits == 2) {tempA0f = thermistor.resistanceToTemperature(ADSToOhms(adc0)); tempA1f = thermistor.resistanceToTemperature(ADSToOhms(adc1));}
  barx = mapf (volts2, 3.2, 4.0, 0, 20);
}

uint8_t findDevices(int pin) {
  OneWire ow(pin);

  uint8_t address[8];
  uint8_t count = 0;


  if (ow.search(address)) {
    Serial.print("\nuint8_t pin");
    Serial.print(pin, DEC);
    Serial.println("[][8] = {");
    do {
      count++;
      Serial.println("  {");
      for (uint8_t i = 0; i < 8; i++) {
        Serial.print("0x");
        if (address[i] < 0x10) Serial.print("0");
        Serial.print(address[i], HEX);
        if (i < 7) Serial.print(", ");
      }
      Serial.println("  },");
    } while (ow.search(address));

    Serial.println("};");
    Serial.print("// nr devices found: ");
    Serial.println(count);
  }

  return count;
}

bool editMode = false;
int editSelection = 0;  // which menu item is being edited

void drawSettings() {         //if we're in settings mode
  if (old_count > settemp) {  //if the encoder was turned left
    rotLeft = true;
  } else if (old_count < settemp) {  //if the encoder was turned right
    rotRight = true;
  }
  old_count = settemp;
  img.fillSprite(TFT_BLACK);
  img.setCursor(0, 0);
  img.setTextSize(2);
  img.setTextColor(TFT_WHITE);
  img.setTextDatum(TL_DATUM);

  img.setTextFont(1);


  if (!digitalRead(rotbutton)) {
    b1pressed = true;
    waitForButtonsReleased();  //wait for button to be released
  }

  if (!editMode) {
    if (rotLeft) {
      setSelection--;
      rotLeft = false;  //reset flag
    }
    if (rotRight) {
      setSelection++;
      rotRight = false;  //reset flag
    }
    if (setSelection < 0) setSelection = 7;  //wrap around on the 0th menu item
    if (setSelection > 7) setSelection = 0;  //wrap around on the 8th menu item
    if (b1pressed) {
      editMode = true;
      editSelection = setSelection;
      if (setSelection == 6) {
        playSound = true;
        b1pressed = false;
        editMode = false;
      }
      if (setSelection == 7) {
        savePrefs();
        settingspage = false;
        count = enc_count;
        b1pressed = false;
        editMode = false;
        waitForButtonsReleased();
      }

      b1pressed = false;
      // Optionally, store the base count for this value if you want to "snap" to current value
    }
  } else {
    // In edit mode, use rotLeft/rotRight to change the value
    switch (editSelection) {
      case 0:  // Alarm
        if (rotLeft) {
          setAlarm = (setAlarm + 4) % 5;
          rotLeft = false;
        }
        if (rotRight) {
          setAlarm = (setAlarm + 1) % 5;
          rotRight = false;
        }
        break;
      case 1:  // Units
        if (rotLeft) {
          setUnits = (setUnits + 2) % 3;
          rotLeft = false;
        }
        if (rotRight) {
          setUnits = (setUnits + 1) % 3;
          rotRight = false;
        }
        break;
      case 2:  // BG Colour
        if (rotLeft) {
          setBGC = (setBGC + 23) % 24;
          rotLeft = false;
        }
        if (rotRight) {
          setBGC = (setBGC + 1) % 24;
          rotRight = false;
        }
        break;
      case 3:  // FG Colour
        if (rotLeft) {
          setFGC = (setFGC + 23) % 24;
          rotLeft = false;
        }
        if (rotRight) {
          setFGC = (setFGC + 1) % 24;
          rotRight = false;
        }
        break;
      case 4:  // Volume
        if (rotLeft) {
          setVolume = (setVolume + 100) % 101;
          BMP.setGain(setVolume / 100.0);
          rotLeft = false;
        }
        if (rotRight) {
          setVolume = (setVolume + 1) % 101;
          BMP.setGain(setVolume / 100.0);
          rotRight = false;
        }
        break;
      case 5:  // Icons
        if (rotLeft) {
          setIcons = (setIcons + 2) % 3;
          rotLeft = false;
        }
        if (rotRight) {
          setIcons = (setIcons + 1) % 3;
          rotRight = false;
        }
        break;

        // Special actions for Test Spk and Save
    }
    if (b1pressed) {
      editMode = false;
      b1pressed = false;
    }
  }

  // Draw menu items
  for (int i = 0; i < 8; i++) {
    if (setSelection == i && !editMode) {
      img.setTextColor(TFT_BLACK, TFT_WHITE, true);
    } else {
      img.setTextColor(TFT_WHITE);
    }
    switch (i) {
      case 0: img.println("Alarm:"); break;
      case 1: img.println("Units:"); break;
      case 2: img.println("BG Colour:"); break;
      case 3: img.println("FG Colour:"); break;
      case 4: img.println("Volume:"); break;
      case 5: img.println("Icons:"); break;
      case 6: img.println(">Test Spk<"); break;
      case 7: img.println(">Save<"); break;
    }
  }

  // Draw values on right, highlight if in edit mode
  img.setTextDatum(TR_DATUM);
  for (int i = 0; i < 8; i++) {
    int y = i * 16;
    bool highlight = (editMode && editSelection == i);
    if (highlight) img.setTextColor(TFT_BLACK, TFT_WHITE, true);
    else img.setTextColor(TFT_WHITE);

    switch (i) {
      case 0: img.drawNumber(setAlarm, 128, y); break;
      case 1:
        {
          const char* setUnitString = (setUnits == 0) ? "C" : (setUnits == 1) ? "F"
                                                                              : "K";
          img.drawString(setUnitString, 128, y);
          break;
        }
      case 2: img.drawNumber(setBGC, 128, y); break;
      case 3: img.drawNumber(setFGC, 128, y); break;
      case 4: img.drawNumber(setVolume, 128, y); break;
      case 5:
        {
          const char* setIconString = (setIcons == 0) ? "T" : (setIcons == 1) ? "Y"
                                                                              : "B";
          img.drawString(setIconString, 128, y);
          break;
        }
      case 6: break;  // Test Spk, no value
      case 7: break;  // Save, no value
    }
  }

  // Special actions for Test Spk and Save
  if (setSelection == 6 && !editMode && b1pressed) {
    playSound = true;
    b1pressed = false;
    editMode = false;
  }
  if (setSelection == 7 && !editMode && b1pressed) {
    savePrefs();
    b1pressed = false;
    editMode = false;
    waitForButtonsReleased();
  }

  // Draw sample string
  img.setTextDatum(TC_DATUM);
  img.setTextColor(cmap[setFGC], cmap[setBGC], true);
  snprintf(sampleString, sizeof(sampleString), "%d%s", setFGC, cmapNames[setFGC]);
  img.drawString(sampleString, 120, 200);

  img.pushSprite(0, 0);
}

bool gottenHot = false;

// --- drawCalib ---
void drawCalib() {

  if (gottenHot) {
    img.fillSprite(TFT_MAROON);
  } else {
    img.fillSprite(TFT_YELLOW);
  }
  img.setTextSize(2);
  img.setTextColor(TFT_WHITE, TFT_BLACK, true);
  img.setTextFont(1);
  img.setTextDatum(TL_DATUM);
  img.setCursor(0, 0);
  img.println("Calibrating!");
  //img.setTextSize(2);
  img.setCursor(0, 150);
  img.println("Please wait for all 3 temperature points to be measured...");
  // img.setTextSize(2);
  if (onewiretempC > 77.0) {
    if (!gottenHot) {
      gottenHot = true;
    }
  }

  // Remove static declaration since we're using global
  snprintf(dallasString, sizeof(dallasString), "%.2f C, A0: %.2f", onewiretempC, ADSToOhms(volts0));
  img.drawString(dallasString, 0, 20);
  if (gottenHot) {
    if ((onewiretempC >= 75.0) && (onewiretempC <= 76.0)) {
      temp1 = onewiretempC;
      therm1 = ADSToOhms(volts0);
      Serial.println("Temp1 set to: " + String(temp1) + " C, resistance: " + String(therm1) + " Ohms");
    }
    if ((onewiretempC >= 50.0) && (onewiretempC <= 51.0)) {
      temp2 = onewiretempC;
      therm2 = ADSToOhms(volts0);
      Serial.println("Temp2 set to: " + String(temp2) + " C, resistance: " + String(therm2) + " Ohms");
    }
    if ((onewiretempC >= 30.0) && (onewiretempC <= 31.0)) {
      temp3 = onewiretempC;
      therm3 = ADSToOhms(volts0);
      Serial.println("Temp3 set to: " + String(temp3) + " C, resistance: " + String(therm3) + " Ohms");
    }
  }

  // Remove static declaration
  //temp1string = String(temp1) + " = " + String(therm1);
  //temp2string = String(temp2) + " = " + String(therm2);
  //temp3string = String(temp3) + " = " + String(therm3);
  snprintf(temp1string, sizeof(temp1string), "%.2f = %ld", temp1, therm1);
  snprintf(temp2string, sizeof(temp2string), "%.2f = %ld", temp2, therm2);
  snprintf(temp3string, sizeof(temp3string), "%.2f = %ld", temp3, therm3);
  img.drawString(temp1string, 0, 40);
  img.drawString(temp2string, 0, 60);
  img.drawString(temp3string, 0, 80);

  if ((temp3 > 0) && (temp2 > 0) && (temp1 > 0)) {
    if (!saved) {
      Serial.println("Calculating coefficients data...");
      thermistor.setTemperature1(temp1 + 273.15);
      thermistor.setTemperature2(temp2 + 273.15);
      thermistor.setTemperature3(temp3 + 273.15);
      thermistor.setResistance1(therm1);
      thermistor.setResistance2(therm2);
      thermistor.setResistance3(therm3);
      thermistor.calcCoefficients();


      snprintf(coeffAstring, sizeof(coeffAstring), "A: %.5f", thermistor.getCoeffA());
      snprintf(coeffBstring, sizeof(coeffBstring), "B: %.5f", thermistor.getCoeffB());
      snprintf(coeffCstring, sizeof(coeffCstring), "C: %.5f", thermistor.getCoeffC());
      Serial.println("Saving coefficients to flash:");
      preferences.begin("my-app", false);
      preferences.putInt("temp1", temp1);
      preferences.putInt("temp2", temp2);
      preferences.putInt("temp3", temp3);
      preferences.putInt("therm1", therm1);
      preferences.putInt("therm2", therm2);
      preferences.putInt("therm3", therm3);
      preferences.end();
      saved = true;
    }
    img.fillSprite(TFT_GREEN);
    img.setCursor(0, 0);
    img.println("Calibration saved, please reboot!");
    Serial.println("Calibration saved, please reboot!");
    img.setTextSize(2);

    img.drawString(temp1string, 0, 40);
    img.drawString(temp2string, 0, 60);
    img.drawString(temp3string, 0, 80);
    img.drawString(coeffAstring, 0, 100);
    img.drawString(coeffBstring, 0, 120);
    img.drawString(coeffCstring, 0, 140);
  }
  img.pushSprite(0, 0);
}

SET_LOOP_TASK_STACK_SIZE(16 * 1024);

void setup() {


  initializeCmap();
  LittleFS.begin();
  BMP.begin();


  thermistor.calcCoefficients();
  pinMode(rotLpin, INPUT_PULLUP);
  pinMode(rotRpin, INPUT_PULLUP);
  pinMode(rotbutton, INPUT_PULLUP);

  abOld = count = old_count = 0;
  Serial.begin(115200);
  Serial.println("Hello!");
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  Wire.begin();
  adc.begin();  //fire up the ADC
  adc.setGain(GAIN_ONE);  //1x gain setting is perfect
  adc.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
  Serial.println("Loading prefs");
  preferences.begin("my-app", false);  //read preferences from flash, with default values if none are found
  temp1 = preferences.getInt("temp1", 0);
  temp2 = preferences.getInt("temp2", 0);
  temp3 = preferences.getInt("temp3", 0);
  therm1 = preferences.getInt("therm1", 0);
  therm2 = preferences.getInt("therm2", 0);
  therm3 = preferences.getInt("therm3", 0);
  setAlarm = preferences.getInt("setAlarm", 0);
  setUnits = preferences.getInt("setUnits", 1);
  setFGC = preferences.getInt("setFGC", 12);
  setBGC = preferences.getInt("setBGC", 4);
  setVolume = preferences.getInt("setVolume", 100);
  //setLEDmode = preferences.getInt("setLEDmode", 2);
  setIcons = preferences.getInt("setIcons", 1);
  count = preferences.getInt("enc_count", 580);
  preferences.end();
  BMP.setGain(setVolume / 100.0);
  if (setFGC == setBGC) {
    setFGC = 15;
    setBGC = 0;
  }                                                 //do not allow foreground and background colour to be the same
  if ((temp3 > 0) && (temp2 > 0) && (temp1 > 0)) {  //if probe temp calibration has been saved to flash, use it
    thermistor.setTemperature1(temp1 + 273.15);
    thermistor.setTemperature2(temp2 + 273.15);
    thermistor.setTemperature3(temp3 + 273.15);
    thermistor.setResistance1(therm1);
    thermistor.setResistance2(therm2);
    thermistor.setResistance3(therm3);
    thermistor.calcCoefficients();
  }
  img.setColorDepth(8);  //WE DONT HAVE ENOUGH RAM LEFT FOR 16 BIT AND JOHNNY CASH, MUST USE 8 BIT COLOUR!
  img.createSprite(240, 320);
  img.fillSprite(TFT_BLUE);
  img.setTextWrap(true);  // Wrap on width
  img.setTextSize(2);
  oldtemp = tempA0f;  // Initialize oldtemp for future ETA calculations
  oldtemp2 = tempA1f;
  forceADC();
  drawTemps();
  Serial.println("Starting wifi");
  WiFi.mode(WIFI_STA);  //precharge the wifi
  WiFiManager wm;       //FIRE UP THE WIFI MANAGER SYSTEM FOR WIFI PROVISIONING

  if ((!digitalRead(rotbutton)) || !wm.getWiFiIsSaved()) {  //If either both buttons are pressed on bootup OR no wifi info is saved
    Serial.println("Resetting stuff");
    //nvs_flash_erase(); // erase the NVS partition to wipe all settings
    //nvs_flash_init(); // initialize the NVS partition.
    preferences.begin("my-app", false);  //read preferences from flash, with default values if none are found
    preferences.putInt("temp1", 0);
    preferences.putInt("temp2", 0);
    preferences.putInt("temp3", 0);
    preferences.putInt("therm1", 0);
    preferences.putInt("therm2", 0);
    preferences.putInt("therm3", 0);
    preferences.putInt("setAlarm", 0);
    preferences.putInt("setUnits", 1);
    preferences.putInt("setFGC", 12);
    preferences.putInt("setBGC", 4);
    preferences.putInt("setVolume", 100);
    //preferences.putInt("setLEDmode", 2);
    preferences.putInt("setIcons", 1);
    preferences.putInt("enc_count", 580);
    preferences.end();
    Serial.println("Prefs reset");
    tft.fillScreen(TFT_ORANGE);
    tft.setCursor(0, 0);
    tft.setTextFont(1);
    tft.setTextSize(2);
    tft.println("SETTINGS RESET.");
    tft.println("");
    tft.println("Please connect to");
    tft.println("'MR MEAT SETUP'");
    tft.println("WiFi, and browse to");
    tft.println("192.168.4.1.");
    wm.resetSettings();


    bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    res = wm.autoConnect("MrMeat3 Setup");
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    if (!res) {  //if the wifi manager failed to connect to wifi
      tft.fillScreen(TFT_RED);
      tft.setCursor(0, 0);
      tft.println("Failed to connect, restarting");
      delay(3000);
      ESP.restart();
    } else {
      //if you get here you have connected to the WiFi
      tft.fillScreen(TFT_GREEN);
      tft.setCursor(0, 0);
      tft.println("Connected!");
      tft.println(WiFi.localIP());
      delay(3000);
      ESP.restart();  //restart the device now because bugs, user will never notice it's so fuckin fast
    }
  } else {  //if wifi information was saved, use it
    Serial.println("Wifi information found, using...");
    WiFi.begin(wm.getWiFiSSID(), wm.getWiFiPass());
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
  }




  sensors.begin();
  sensors.setResolution(11);
  if (findDevices(onewirepin) > 0)  //check and see if the probe is connected, if it is,
  {
    onewirefound = true;
    temp1 = 0;  //reset all saved temperatures
    temp2 = 0;
    temp3 = 0;
    calibrationMode = true;  //we're in Calibration town baby
    //temperatureSensors.onTemperatureChange(handleTemperatureChange);
    sensors.requestTemperatures();
    onewiretempC = sensors.getTempCByIndex(0);
    sensors.requestTemperatures();
    onewiretempC = sensors.getTempCByIndex(0);
    tft.fillScreen(TFT_PURPLE);
    tft.setCursor(0, 0);
    tft.setTextFont(1);
    tft.println("Calibration Mode!");
    tft.println("To begin, connect 1 meat probe in the left hole, immerse it and the calibration probe in a small cup of hot freshly boiled water (>75C), then press any button.");
    while (digitalRead(rotbutton)) { delay(1); }  //wait until a button is pressed
  } else {
    Serial.println("No onewire probes found.");
    attachInterrupt(digitalPinToInterrupt(rotLpin), pinChangeISR, CHANGE);  // Set up pin-change interrupts
    attachInterrupt(digitalPinToInterrupt(rotRpin), pinChangeISR, CHANGE);
  }
    Serial.printf("Free heap: %u\n", ESP.getFreeHeap());
    Serial.printf("Free stack: %u\n", uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
    Serial.printf("Free largest block: %u\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}



void doSound() {    //play the sound
  if (playSound) {  //if sound is enabled
    Serial.println("Playing sound");
    playSound = false;
    switch (setAlarm) {
      case 0:
        Serial.println("Playing 0");
        //BMP.end();
        playWavFromFS("/ringoffire.wav");
        break;
      case 1:
        Serial.println("Playing 1");
        //BMP.end();
        playWavFromFS("/weirdal.wav");
        break;
      case 2:
        Serial.println("Playing 2");
        //BMP.end();
        playWavFromFS("/shave.wav");
        break;
      case 3:
        Serial.println("Playing 3");
        //BMP.end();
        playWavFromFS("/suppertime.wav");
        break;
      case 4:
        Serial.println("Playing 4");
        //BMP.end();
        playWavFromFS("/dingfries.wav");
        break;
    }
  }
}

void savePrefs() {  //save settings routine
  if (setFGC == setBGC) {
    setFGC = 15;
    setBGC = 0;
  }
  preferences.begin("my-app", false);
  preferences.putInt("setAlarm", setAlarm);
  preferences.putInt("setUnits", setUnits);
  preferences.putInt("setFGC", setFGC);
  preferences.putInt("setBGC", setBGC);
  preferences.putInt("setVolume", setVolume);
  preferences.putInt("setIcons", setIcons);
  // preferences.putInt("setLEDmode", setLEDmode);
  preferences.putInt("enc_count", enc_count);
  preferences.end();
}


void loop() {
  continueAudioPlayback();

  if ((WiFi.status() == WL_CONNECTED) && (!connected)) {
    connected = true;
    rssi = WiFi.RSSI();
    ArduinoOTA.setHostname("MrMeat3");
    ArduinoOTA.begin();


    Serial.println("Connecting blynk...");
    Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
    Blynk.connect();  //Init Blynk
    Serial.println("Blynk connected.");
  }
  else if (WiFi.status() != WL_CONNECTED) {  //if no wifi, try to reconnect
    if (millis() - reconnectTime > 30000) {
      WiFi.disconnect();
      WiFi.reconnect();
      reconnectTime = millis();
    }
  }

    //set the set temperature to the count divided by 4, so we can set it with the rotary encoder

  if ((connected)) {
    ArduinoOTA.handle();
    Blynk.run();
  }
  
  continueAudioPlayback();
  doSound();  //play the sound if needed
  continueAudioPlayback();
  every(10) {  //every 5 milliseconds, so we don't waste battery power
    settemp = count / 4;
    doADC();
    if (adc0 < is2connectedthreshold) {
      is1connected = true;
    } else {
      is1connected = false;
    }  //check for probe2 connection
    if (adc1 < is2connectedthreshold) {
      is2connected = true;
    } else {
      is2connected = false;
    }  //check for probe2 connection

    if (!calibrationMode) {                //if it's not calibration town
      if (!settingspage) { drawTemps(); }  //if we're not in settings mode, draw the main screen
      else {
        drawSettings();
      }  //else draw the settings screen
    } else {
      drawCalib();
    }  //else draw the calibration screen
  bool tempAbove = ((tempA0f >= settemp) || (tempA1f >= settemp)) && (!calibrationMode) && (is1connected || is2connected) && (setVolume > 0) && (millis() > 8000);

  if (tempAbove) {
    if (!tempAlarmActive) {
      tempAlarmActive = true;
      tempAlarmStart = millis();
    } else if (millis() - tempAlarmStart > 500) {
      playSound = true;
    }
  } else {
    tempAlarmActive = false;
    tempAlarmStart = 0;
  }
  }

  continueAudioPlayback();




  every(2000) {
    //every 2 seconds update the wifi signal strength variable
    if (calibrationMode) {
      sensors.requestTemperatures();
      onewiretempC = sensors.getTempCByIndex(0);
    }
    //Serial.printf("Free heap: %u\n", ESP.getFreeHeap());
    //Serial.printf("Free stack: %u\n", uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
    //Serial.printf("Free largest block: %u\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  }






  
  every(15000) {
    int ETA_INTERVAL = 15;
    //manually set this to ETA_INTERVAL*1000, can't hardcode due to macro
    if (!calibrationMode) {
      minsLeft = estimateBatteryTime(volts2);
      tempdiff = tempA0f - oldtemp;
      if (is2connected) {  //If 2nd probe is connected, calculate whichever ETA is sooner in seconds
        tempdiff2 = tempA1f - oldtemp2;
        eta = (((settemp - tempA0f) / tempdiff) * ETA_INTERVAL);
        eta2 = (((settemp - tempA1f) / tempdiff2) * ETA_INTERVAL);
        if ((eta2 > 0) && (eta2 < 1000) && (eta2 < eta)) { eta = eta2; }
        oldtemp2 = tempA1f;
      } else  //Else if only one probe is connected, calculate the ETA in seconds
      {
        eta = (((settemp - tempA0f) / tempdiff) * ETA_INTERVAL);
      }
      etamins = eta / 60;  //cast it to int and divide it by 60 to get minutes with no remainder, ignore seconds because of inaccuracy
      oldtemp = tempA0f;
    }
  }

  every(30000) {  //every 10 seconds
    volts2 = adc.computeVolts(adc2) * 2.0; //update the battery voltage
    //volts2 = maxlipo.cellVoltage();
    barx = mapf (volts2, 3.2, 4.0, 0, 20); //update the battery icon length
    if (is2connected) { Blynk.virtualWrite(V4, tempA1f); }
    if (is1connected) { Blynk.virtualWrite(V2, tempA0f); }
    if ((etamins < 1000) && (etamins >= 0)) {
      Blynk.virtualWrite(V6, etamins);
      
    }
    Blynk.virtualWrite(V7, temperatureRead());
    Blynk.virtualWrite(V5, volts2);
    //Blynk.virtualWrite(V8, heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
   // UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

    //Blynk.virtualWrite(V9, stackHighWaterMark * sizeof(StackType_t));
  }
}
