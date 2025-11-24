#include "LEDutils.h"

#if FASTLED_VERSION < 3001000
#error "Requires FastLED 3.1 or later; check github for latest code."
#endif

CRGB leds[NUM_LEDS];

int led_settings[8] = {MODE_WAVE_RIGHT, CYAN_GP, MODE_WAVE_LEFT, CYAN_GP, MODE_PULSE, PINK_GP, MODE_PULSE, PINK_GP};
int LED_SETTINGS_COUNT = 8;
int new_sets[8]; 
String output = "";

void setup() {
  Serial.begin(9600);
  delay(3000); // 3 second delay for recovery
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS)
    //.setCorrection(TypicalLEDStrip) // cpt-city palettes have different color balance
    .setDither(BRIGHTNESS < 255);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
}

void loop()
{ 
  output = "";
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    
    if (input.startsWith("GET")) {
      String output = generateLedSettingsString();
      Serial.println(output);
    } else {
      parseInputToNewSettings(input);
    } 

    applyNewSettings();
  }
  
  // Set Led modes and colors
  setLedBatch(leds, INDEX_FL, led_settings[0], led_settings[1]);
  setLedBatch(leds, INDEX_FR, led_settings[2], led_settings[3]);
  setLedBatch(leds, INDEX_BR, led_settings[4], led_settings[5]);
  setLedBatch(leds, INDEX_BL, led_settings[6], led_settings[7]);

  FastLED.show();
  FastLED.delay(20);
}


// Functions

String generateLedSettingsString() {
  String output = "";
  for (uint8_t i = 0; i < LED_SETTINGS_COUNT; i++) {
    if (led_settings[i] < 10) output += "0";
    output += String(led_settings[i]);
  }
  return output;
}

void parseInputToNewSettings(const String& input) {
  for (uint8_t i = 0; i < LED_SETTINGS_COUNT; i++) {
    new_sets[i] = input.substring(i * 2, i * 2 + 2).toInt();
  }
}

void applyNewSettings() {
  for (uint8_t i = 0; i < LED_SETTINGS_COUNT; i++) {
    if (new_sets[i] < gGradientPaletteCount) {
      led_settings[i] = new_sets[i];
    }
  }
}
