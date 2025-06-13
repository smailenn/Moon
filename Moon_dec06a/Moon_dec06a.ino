/*
  The following variables are automatically generated and updated when changes are made to the Thing

  CloudColor color;
  float timeleft;
  float voltage;
  int bright;
  bool NEOPIXELS;

  Variables which are marked as READ/WRITE in the Cloud Thing will also have functions
  which are called when their values are changed from the Dashboard.
  These functions are generated with the Thing and added at the end of this sketch.

  BOARD:  ARDUINO MKR WIFI 1010
  
  -Turn on/off Cloud
  -Change Color
  -Change Brightness
  -Turn off in 30 minutes
  -measure battery 
  -sleep mode to save power 
  - 2500 mAh battery.  12 leds on Neopixel ring.  0.5 run times.  ~20 mA per led = 240 mA.  120 mAh / run.  ~21 runs with 
  hopeful savings from low power mode 
*/

#include <ArduinoBLE.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoLowPower.h>


#define LED_PIN     6 //NeoPixel Pin
#define LED_COUNT  12  // How many NeoPixels are attached to the Arduino?
//#define BRIGHTNESS 125 // Set BRIGHTNESS to about 1/5 (max = 255)

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGBW + NEO_KHZ800);
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

// BLE Service and Characteristics UUIDs (randomly generated, you can generate your own)
#define SERVICE_UUID           "12345678-1234-5678-1234-56789abcdef0"
#define CHAR_POWER_UUID        "12345678-1234-5678-1234-56789abcdef1"
#define CHAR_COLOR_UUID        "12345678-1234-5678-1234-56789abcdef2"
#define CHAR_BRIGHTNESS_UUID   "12345678-1234-5678-1234-56789abcdef3"
#define CHAR_VOLTAGE_UUID      "12345678-1234-5678-1234-56789abcdef4"

// Define characteristics
BLEBoolCharacteristic powerChar(CHAR_POWER_UUID, BLERead | BLEWrite);
BLECharacteristic colorChar(CHAR_COLOR_UUID, BLERead | BLEWrite, 4);  // RGBW - 4 bytes
BLEUnsignedCharCharacteristic brightnessChar(CHAR_BRIGHTNESS_UUID, BLERead | BLEWrite);
BLEFloatCharacteristic voltageChar(CHAR_VOLTAGE_UUID, BLERead | BLENotify);

BLEService moonLampService(SERVICE_UUID);

bool powerState = false;
uint8_t colorData[4] = {0, 0, 0, 0};  // RGBW
uint8_t brightness = 50; // default brightness (0-255)

unsigned long LEDonMillis = 0;
unsigned long timer = 30UL * 60UL * 1000UL;  // 30 minutes


void setup() {
  Serial.begin(9600);
  while (!Serial);

  strip.begin();
  strip.show(); // initialize all pixels to 'off'

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setDeviceName("MoonLamp");
  BLE.setLocalName("MoonLamp");
  BLE.setAdvertisedService(moonLampService);

  moonLampService.addCharacteristic(powerChar);
  moonLampService.addCharacteristic(colorChar);
  moonLampService.addCharacteristic(brightnessChar);
  moonLampService.addCharacteristic(voltageChar);

  BLE.addService(moonLampService);

  // Set default values
  powerChar.writeValue(powerState);
  colorChar.writeValue(colorData, 4);
  brightnessChar.writeValue(brightness);
  voltageChar.writeValue(readBatteryVoltage());

  BLE.advertise();

  Serial.println("BLE Moon Light");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    float v = readBatteryVoltage();
    voltageChar.writeValue(v);
    int pct = batteryPercent(v);


    while (central.connected()) {
      // Check if any characteristic was written to
      if (powerChar.written()) {
        powerState = powerChar.value();
        Serial.print("Power changed: ");
        Serial.println(powerState);
        updateLEDs();
        if (powerState) LEDonMillis = millis();
      }

      if (colorChar.written()) {
        colorChar.readValue(colorData, 4);
        Serial.print("Color changed: ");
        Serial.print(colorData[0]); Serial.print(", ");
        Serial.print(colorData[1]); Serial.print(", ");
        Serial.print(colorData[2]); Serial.print(", ");
        Serial.println(colorData[3]);
        updateLEDs();
      }

      if (brightnessChar.written()) {
        brightness = brightnessChar.value();
        Serial.print("Brightness changed: ");
        Serial.println(brightness);
        updateLEDs();
      }

      // Auto turn off after timer (30 min)
      if (powerState && (millis() - LEDonMillis > timer)) {
        powerState = false;
        powerChar.writeValue(false);
        updateLEDs();
        Serial.println("Timer expired: turning off LEDs.");
      }
    delay(100);  // BLE Polling
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
  // No connection: sleep for 10 seconds
  //Serial.println("No BLE central — sleeping...");
  //BLE.end(); // Stop BLE before sleep
  //LowPower.sleep(10000); // 10 seconds sleep
  //BLE.begin(); // Restart BLE after wake
  //BLE.advertise();
}

void updateLEDs() {
  if (powerState) {
    // Apply brightness scaling to color
    uint8_t r = (uint16_t(colorData[0]) * brightness) / 255;
    uint8_t g = (uint16_t(colorData[1]) * brightness) / 255;
    uint8_t b = (uint16_t(colorData[2]) * brightness) / 255;
    uint8_t w = (uint16_t(colorData[3]) * brightness) / 255;

    uint32_t c = strip.Color(r, g, b, w);
    for (int i = 0; i < LED_COUNT; i++) {
      strip.setPixelColor(i, c);
    }
  } else {
    strip.clear();
  }
  strip.show();
}

float readBatteryVoltage() {
  int raw = analogRead(ADC_BATTERY); // A6 = internal battery ADC pin
  float voltage = (raw / 1023.0) * 3.3 * 2.0; // Multiply by 2 due to internal voltage divider
  return voltage;
}

int batteryPercent(float voltage) {
  float minV = 3.3;
  float maxV = 4.2;
  int pct = (voltage - minV) / (maxV - minV) * 100.0;
  return constrain(pct, 0, 100);
}

