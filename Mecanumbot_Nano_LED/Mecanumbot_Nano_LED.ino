#include "LEDutils.h"

#if FASTLED_VERSION < 3001000
#error "Requires FastLED 3.1 or later; check github for latest code."
#endif


const byte INPUT_START_BYTE = 0xAA;
const byte OUTPUT_START_BYTE = 0xAB;
const byte REQUEST_BYTE = 0xAC;

CRGB leds[NUM_LEDS];

int led_settings[8] = {MODE_WAVE_RIGHT, CYAN_GP, MODE_WAVE_LEFT, CYAN_GP, MODE_PULSE, PINK_GP, MODE_PULSE, PINK_GP};
int LED_SETTINGS_COUNT = 8;
int new_sets[8]; 
String output = "";

void setup() {
  Serial.begin(115200);
  delay(3000); // 3 second delay for recovery
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS)
    //.setCorrection(TypicalLEDStrip) // cpt-city palettes have different color balance
    .setDither(BRIGHTNESS < 255);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
  setLedBatch(leds, INDEX_FL, led_settings[0], led_settings[1]);
  setLedBatch(leds, INDEX_FR, led_settings[2], led_settings[3]);
  setLedBatch(leds, INDEX_BL, led_settings[4], led_settings[5]);
  setLedBatch(leds, INDEX_BR, led_settings[6], led_settings[7]);
  FastLED.show();
}

void loop()
{ 
  //IO 
  handleIncoming();
  updateLEDFlashes();
}

void handleIncoming() {
  static byte buf[12];
  static byte idx = 0;

  while (Serial.available()) {
    byte b = Serial.read();
    if (b == REQUEST_BYTE)
      {
        sendFeedback();
        continue;
      }
    if (idx == 0 && b != INPUT_START_BYTE) continue;

    buf[idx++] = b;

    if (idx == 12) {
      byte FL_mode = buf[1];
      byte FL_color = buf[2];
      byte FR_mode = buf[3];
      byte FR_color = buf[4];
      byte BL_mode = buf[5];
      byte BL_color = buf[6];
      byte BR_mode = buf[7];
      byte BR_color = buf[8];

      byte tL = buf[9];
      byte tH = buf[10];

      byte cs = buf[11];
      if ((FL_mode ^ FL_color ^ FR_mode ^ FR_color ^ BL_mode ^ BL_color ^ BR_mode ^ BR_color ^ tL ^ tH) == cs) {

        // Update led_settings from packet
        led_settings[0] = FL_mode;
        led_settings[1] = FL_color;

        led_settings[2] = FR_mode;
        led_settings[3] = FR_color;

        led_settings[4] = BL_mode;
        led_settings[5] = BL_color;

        led_settings[6] = BR_mode;
        led_settings[7] = BR_color;
      }
              idx = 0;
    }
  }
}

void sendFeedback() {
  byte FL_mode = led_settings[0];
  byte FL_color = led_settings[1];
  byte FR_mode = led_settings[2];
  byte FR_color = led_settings[3];
  byte BL_mode = led_settings[4];
  byte BL_color = led_settings[5];
  byte BR_mode = led_settings[6];
  byte BR_color = led_settings[7];
  byte checksum = FL_mode ^ FL_color ^ FR_mode ^ FR_color ^ BL_mode ^ BL_color ^ BR_mode ^ BR_color;

  Serial.write(OUTPUT_START_BYTE);
  Serial.write(FL_mode);
  Serial.write(FL_color);
  Serial.write(FR_mode);
  Serial.write(FR_color);
  Serial.write(BL_mode);
  Serial.write(BL_color);
  Serial.write(BR_mode);
  Serial.write(BR_color);
  Serial.write(checksum);
}

void updateLEDFlashes()
{

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 33) return; 
  
  lastUpdate = millis();
  // Set Led modes and colors
  setLedBatch(leds, INDEX_FL, led_settings[0], led_settings[1]);
  setLedBatch(leds, INDEX_FR, led_settings[2], led_settings[3]);
  setLedBatch(leds, INDEX_BL, led_settings[4], led_settings[5]);
  setLedBatch(leds, INDEX_BR, led_settings[6], led_settings[7]);
  FastLED.show();
}