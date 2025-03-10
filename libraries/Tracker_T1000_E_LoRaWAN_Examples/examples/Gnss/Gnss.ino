#include <Tracker_T1000_E_LoRaWAN_Examples.h>

////////////////////////////////////////////////////////////////////////////////
// Variables

// Timestamp for GNSS check
uint32_t lastGnssCheck = 0;

////////////////////////////////////////////////////////////////////////////////
// Setup and loop
void setup()
{
    gnss_init();
    gnss_scan_start();
}

void loop()
{
    // Read serial data and parse it
    gnss_handle_serial();

    // Print every second
    if (millis() - lastGnssCheck >= 1000)
    {
        lastGnssCheck = millis();

        if (gnss_get_fix_status())
        {
            int32_t lat = 0, lon = 0;
            gnss_get_position(&lat, &lon);
            Serial.printf("lat %d, lon %d\n", lat, lon);
        }
        else
        {
            Serial.println("@");
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
