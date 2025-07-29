#include "LEDutils.h"

#if FASTLED_VERSION < 3001000
#error "Requires FastLED 3.1 or later; check github for latest code."
#endif

CRGB leds[NUM_LEDS];

CRGBPalette16 gCurrentPaletteFR(CRGB::Black);
CRGBPalette16 gCurrentPaletteFL(CRGB::Black);
CRGBPalette16 gCurrentPaletteBR(CRGB::Black);
CRGBPalette16 gCurrentPaletteBL(CRGB::Black);

void setup() {
  Serial.begin(9600);
  delay(3000); // 3 second delay for recovery
  Serial.println("LED is ready!");
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS)
    //.setCorrection(TypicalLEDStrip) // cpt-city palettes have different color balance
    .setDither(BRIGHTNESS < 255);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
}

void loop()
{ 
  colorWaveFix(leds+INDEX_FL, BATCH_LEDS, gGradientPalettes[CYAN_GP],  RIGHT);
  colorWaveFix(leds+INDEX_FR, BATCH_LEDS, gGradientPalettes[CYAN_GP],  LEFT);
  colorPulse(leds+INDEX_BR, BATCH_LEDS, gGradientPalettes[PINK_GP]);
  colorPulse(leds+INDEX_BL, BATCH_LEDS, gGradientPalettes[PINK_GP]);

  FastLED.show();
  FastLED.delay(20);
}

