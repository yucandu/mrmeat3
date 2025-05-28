#include <ESP_I2S.h>
// Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include <LittleFS.h>
#include <BackgroundAudioSpeech.h>
#include <libespeak-ng/voice/en.h>
#include <SteinhartHart.h>
#include <ESP32I2SAudio.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <BackgroundAudio.h>
#include <TFT_eSPI.h> 
#include <JohnnyCash.h>
#define STASSID "mikesnet"
#define STAPSK "springchicken"
#include<ADS1115_WE.h> 
#include<Wire.h>
#define I2C_ADDRESS 0x48
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <BlynkSimpleEsp32.h>
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

#include <ESP32RotaryEncoder.h>
const char *ssid = STASSID;
const char *pass = STAPSK;

ESP32I2SAudio audio(2, 3, 0); // BCLK, LRCLK, DOUT (,MCLK)

ROMBackgroundAudioWAV BMP(audio);


#define rotLpin 21
#define rotRpin 20
#define rotbutton 5
#define onewirepin 7

RotaryEncoder rotaryEncoder( rotLpin, rotRpin, rotbutton);

OneWire oneWire(onewirepin);
DallasTemperature sensors(&oneWire);

float onewiretempC, tempF, tempA0, tempA1, tempA0f, tempA1f, barx;
long adc0, adc1, adc2, adc3, therm1, therm2, therm3; //had to change to long since uint16 wasn't long enough
float temp1, temp2, temp3;
float volts0, volts1, volts2, volts3;
int channel = 1;
bool onewirefound = false;
bool pressed = false;

SteinhartHart thermistor(15062.08,36874.80,82837.54, 348.15, 323.15, 303.15); //these are the default values for a Weber probe


TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
TFT_eSprite img = TFT_eSprite(&tft);
volatile long enc_count = 0;

#define TFT_GREY 0x5AEB // New colour


void turnedRight()
{
  enc_count++;
}

void turnedLeft()
{
  enc_count--;  
}

void knobCallback( long value )
{
	switch( value )
	{
		case 1:
        enc_count++;
		break;

		case -1:
        enc_count--; 
		break;
	}
	rotaryEncoder.setEncoderValue( 0 );
}

void buttonCallback( unsigned long duration )
{
  pressed = true;
}



float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while(adc.isBusy()){delay(0);}
  voltage = adc.getRawResult(); // alternative: getResult_mV for Millivolt
  return voltage;
}

float readChannelV(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while(adc.isBusy()){delay(0);}
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}

void doADC() {  //take measurements from the ADC without blocking
  if (!adc.isBusy()) {
    if (channel == 0) {
      adc0 = adc.getResult_V();
      tempA0 = thermistor.resistanceToTemperature(ADSToOhms(adc0)) - 273.15;
      tempA0f = (tempA0 * 1.8) + 32;
      adc.setCompareChannels(ADS1115_COMP_1_GND);
      adc.startSingleMeasurement();
      channel = 1;
      return;
    }
    if (channel == 1) {
      adc1 = adc.getResult_V();
      tempA1 = thermistor.resistanceToTemperature(ADSToOhms(adc1)) - 273.15;
      tempA1f = (tempA1 * 1.8) + 32;
      adc.setCompareChannels(ADS1115_COMP_2_GND);
      adc.startSingleMeasurement();
      channel = 2;
      return;
    }
    if (channel == 2) {  //channel for measuring battery voltage
      volts2 = adc.getResult_V();
      adc.setCompareChannels(ADS1115_COMP_0_GND);
      adc.startSingleMeasurement();
      channel = 0;
      return;
    }
  }
}

double ADSToOhms(int16_t ADSreading) { //convert raw ADS reading to a measured resistance in ohms, knowing R1 is 22000 ohms
      float voltsX = ADSreading;
      return (voltsX * 22000) / (3.3 - voltsX);
}

double mapf(float x, float in_min, float in_max, float out_min, float out_max)  //like default map function but returns float instead of int
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void forceADC() {  //force an ADC measurement and block the CPU to do it
  adc0 = readChannelV(ADS1115_COMP_0_GND);
  adc1 = readChannelV(ADS1115_COMP_1_GND);
  volts2 = readChannelV(ADS1115_COMP_2_GND) * 2.0;  //battery voltage measurement
  tempA0 = thermistor.resistanceToTemperature(ADSToOhms(adc0)) - 273.15;
  tempA0f = (tempA0 * 1.8) + 32;
  tempA1 = thermistor.resistanceToTemperature(ADSToOhms(adc1)) - 273.15;
  tempA1f = (tempA1 * 1.8) + 32;
  barx = mapf (volts2, 3.2, 4.0, 0, 20);
}

uint8_t findDevices(int pin) {
  OneWire ow(pin);

  uint8_t address[8];
  uint8_t count = 0;


  if (ow.search(address))
  {
    Serial.print("\nuint8_t pin");
    Serial.print(pin, DEC);
    Serial.println("[][8] = {");
    do {
      count++;
      Serial.println("  {");
      for (uint8_t i = 0; i < 8; i++)
      {
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


void setup(void) {
  rotaryEncoder.setEncoderType( EncoderType::FLOATING );
  rotaryEncoder.setBoundaries( -1, 1, false );

  rotaryEncoder.onTurned( &knobCallback );

  rotaryEncoder.onPressed( &buttonCallback );

  rotaryEncoder.begin();

  //pinMode(rotbutton, INPUT_PULLUP);
 // attachInterrupt(digitalPinToInterrupt(rotLpin), encoder_isr, CHANGE);  // Set up pin-change interrupts
 // attachInterrupt(digitalPinToInterrupt(rotRpin), encoder_isr, CHANGE);
  Serial.begin(115200);
  Serial.println("Hello!");
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  Wire.begin();
  if(!adc.init()){
    Serial.println("ADS1115 not connected!");
  }
  adc.setVoltageRange_mV(ADS1115_RANGE_4096); // Set voltage range to 0-4.096V
  /*
  // Set "cursor" at top left corner of display (0,0) and select font 2
  // (cursor will move to next line automatically during printing with 'tft.println'
  //  or stay on the line is there is room for the text with tft.print)
  tft.setCursor(0, 0, 2);
  // Set the font colour to be white with a black background, set text size multiplier to 1
  tft.setTextColor(TFT_WHITE,TFT_BLACK);  tft.setTextSize(1);
  // We can now plot text on screen using the "print" class
  tft.print("Connecting to wifi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  WiFi.setTxPower (WIFI_POWER_8_5dBm);
  while (WiFi.status() != WL_CONNECTED) {
    tft.print(".");
    delay(100);
  }
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0, 2);
  tft.print("http://");
  tft.println(WiFi.localIP());
  delay(1000);
  tft.fillScreen(TFT_BLACK);*/

  

  img.setColorDepth(8);  //WE DONT HAVE ENOUGH RAM LEFT FOR 16 BIT AND JOHNNY CASH, MUST USE 8 BIT COLOUR!
  img.createSprite(128, 160);
  img.fillSprite(TFT_BLUE);
  BMP.begin();
  BMP.setGain(0.1);
  BMP.flush(); // Stop any existing output, reset for new file
  forceADC(); // Force an initial ADC read to populate the values
  //BMP.write(RingOfFire, sizeof(RingOfFire));
  sensors.begin();
  if (findDevices(onewirepin) > 0) //check and see if the probe is connected, if it is,
  {
    onewirefound = true;
  }
}

void doDisplay() {
  img.fillSprite(TFT_BLUE);
  img.drawRect(0, 0, tft.width(), tft.height(), TFT_WHITE);
  img.setTextColor(TFT_WHITE, TFT_BLUE);
  img.setTextSize(1);
  img.setCursor(0, 0);
  img.print("Rotary count: ");
  img.println(enc_count);
  //img.print("Step count: ");
  //img.println(stepCount);
  if (pressed) {
    img.setTextColor(TFT_RED, TFT_BLUE);
    img.println("Button pressed!");
    if (!BMP.done()) {
      BMP.flush(); // Stop playback
    } else {
      BMP.flush(); // Stop any existing output, reset for new file
      BMP.write(RingOfFire, sizeof(RingOfFire));
    }
    pressed = false;
  } else {
    img.setTextColor(TFT_WHITE, TFT_BLUE);
    img.println("Button not pressed");
  }

  if (!BMP.done()) {
    img.setTextColor(TFT_GREEN, TFT_BLUE);
    img.println("Playing...");
  } else {
    img.setTextColor(TFT_YELLOW, TFT_BLUE);
    img.println("Not playing");
  }
  img.println("A0 Temp: " + String(tempA0f, 1) + " F");
  img.println("A1 Temp: " + String(tempA1f, 1) + " F");
  img.println("A2 Voltage: " + String(volts2, 3) + " V");
  if (onewirefound) {
    sensors.requestTemperatures(); 
    onewiretempC = sensors.getTempCByIndex(0);
    img.println("OneWire Temp: " + String(onewiretempC, 2) + " C");
  }

  
  // Draw the sprite to the display
  img.pushSprite(0, 0);
  

}

void loop() {
  doADC();
  doDisplay();
}