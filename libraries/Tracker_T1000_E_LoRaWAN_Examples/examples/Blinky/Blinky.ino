#include <Tracker_T1000_E_LoRaWAN_Examples.h>

////////////////////////////////////////////////////////////////////////////////
// Setup and loop

void setup()
{
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
}

void loop()
{
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
    delay(500);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
    delay(500);
}

////////////////////////////////////////////////////////////////////////////////
