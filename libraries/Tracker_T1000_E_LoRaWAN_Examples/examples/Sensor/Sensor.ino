#include <Tracker_T1000_E_LoRaWAN_Examples.h>

////////////////////////////////////////////////////////////////////////////////
// Setup and loop

void setup()
{
    Serial.begin(115200);

    pinMode(PIN_SENSE_POWER_EN, OUTPUT);
    digitalWrite(PIN_SENSE_POWER_EN, LOW);

    // Set the analog reference to 3.0V
    analogReference(AR_INTERNAL_3_0);
}

void loop()
{
    int16_t ntc = 0, lux = 0, bat = 0;

    ntc = sensor_ntc_sample();
    lux = sensor_lux_sample();
    bat = sensor_bat_sample();
    Serial.printf("ntc: %d, lux: %d, bat: %d\n", ntc, lux, bat);

    delay(1000);
}

////////////////////////////////////////////////////////////////////////////////
