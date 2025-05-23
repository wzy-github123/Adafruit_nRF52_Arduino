#include <Tracker_T1000_E_LoRaWAN_Examples.h>

////////////////////////////////////////////////////////////////////////////////
// button IRQ handler

static void button_irq_handler()
{
    Serial.println("user button press");
}

////////////////////////////////////////////////////////////////////////////////
// Setup and loop

void setup()
{
    Serial.begin(115200);
    pinMode(PIN_BUTTON1, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON1), button_irq_handler, RISING);
}

void loop()
{
    delay(1000);
}

////////////////////////////////////////////////////////////////////////////////
