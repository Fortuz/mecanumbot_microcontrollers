#include "mecanumbot.h"

void setup() {
    Serial.begin(57600);
    Serial.println("Hello, Mecanumbot A New Age!");
    MecanumbotCore::begin();
}
void loop() {
    MecanumbotCore::run(); 
}