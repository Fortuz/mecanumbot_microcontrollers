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
#define INDEX_BR  0
#define INDEX_BL  8
#define INDEX_FR  16
#define INDEX_FL  24

#define LEFT  0
#define RIGHT 1

// Progress fill: how many of a panel's LEDs are drawn in the progress color
// instead of the panel's own. 0 is no fill, BATCH_LEDS the whole panel.
#define PROGRESS_MAX BATCH_LEDS

// Which end of each panel the fill grows from. The strips cut the four corners
// of the top plate diagonally, and a bar should grow towards the front of the
// robot on all of them. A right wave flows towards pixel 0, and "right" seen
// from behind is the front end of FL and BR but the rear end of FR and BL --
// so FL and BR fill from their last pixel, FR and BL from their first.
// Flip an entry if a bar grows backwards on the robot.
#define FILL_FROM_LAST_FL true
#define FILL_FROM_LAST_FR false
#define FILL_FROM_LAST_BL false
#define FILL_FROM_LAST_BR true

// Palette color group indices
#define BLACK_GP  0
#define WHITE_GP  1
#define GREEN_GP  2
#define RED_GP    3
#define BLUE_GP   4
#define CYAN_GP   5
#define PINK_GP   6
#define YELLOW_GP 7

// Mode constants
#define MODE_WAVE_RIGHT  1
#define MODE_WAVE_LEFT   2
#define MODE_PULSE       3
#define MODE_SOLID       4
#define MODE_FAST_BLINK  5
# define MODE_SLOW_BLINK 6

void colorSolid(CRGB* ledarray, uint16_t numleds, const CRGBPalette16& palette);
void colorPulse(CRGB* ledarray, uint16_t numleds, const CRGBPalette16& palette);
void colorWaveCus(CRGB* ledarray, uint16_t numleds, const CRGBPalette16& palette, int direction);
void colorWaveFix(CRGB* ledarray, uint16_t numleds, const CRGBPalette16& palette, int direction,
                  const CRGBPalette16& fillPalette, uint8_t fill, bool fillFromLast);
void colorBlink(CRGB* ledarray, uint16_t numleds, const CRGBPalette16& palette, int blinkHalfPeriodMs);

void setLedBatch(CRGB* ledarray, int index, int mode, int color);
void setLedBatch(CRGB* ledarray, int index, int mode, int color,
                 uint8_t fill, int fillColor, bool fillFromLast);

extern const TProgmemRGBGradientPaletteRef gGradientPalettes[];
extern const uint8_t gGradientPaletteCount;

#endif