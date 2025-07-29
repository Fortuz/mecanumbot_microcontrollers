#ifndef LED_UTILS_H
#define LED_UTILS_H

#include "FastLED.h"

#define BRIGHTNESS  240
#define DATA_PIN    5
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS   32 // in total
#define BATCH_LEDS 8

// Indexes for LED groups
#define INDEX_BL  0
#define INDEX_BR  8
#define INDEX_FR  16
#define INDEX_FL  24

#define LEFT  0
#define RIGHT 1

// Palette group indices
#define YELLOW_GP 0
#define GREEN_GP  1
#define RED_GP    2
#define BLUE_GP   3
#define CYAN_GP   4
#define PINK_GP   5
#define WHITE_GP  6
#define BLACK_GP  7

void colorSolid(CRGB* ledarray, uint16_t numleds, const CRGBPalette16& palette);
void colorPulse(CRGB* ledarray, uint16_t numleds, const CRGBPalette16& palette);
void colorWaveCus(CRGB* ledarray, uint16_t numleds, const CRGBPalette16& palette, int direction);
void colorWaveFix(CRGB* ledarray, uint16_t numleds, const CRGBPalette16& palette, int direction);

extern const TProgmemRGBGradientPalettePtr gGradientPalettes[];

#endif