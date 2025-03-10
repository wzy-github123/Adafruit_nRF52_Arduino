#include <Tracker_T1000_E_LoRaWAN_Examples.h>
#include <Wire.h> // for I2C

////////////////////////////////////////////////////////////////////////////////
// Acclerometer IRQ handler

static void acc_irq_handler()
{
    // Read out motion status registers and print messages according to the events
    uint8_t status_0 = qma6100p_get_motion_status();
    uint8_t status_1 = qma6100p_get_motion_status_1();
    if ((status_0 & 0x01) == 0x01)
        Serial.println("any motion x");
    if ((status_0 & 0x02) == 0x02)
        Serial.println("any motion y");
    if ((status_0 & 0x04) == 0x04)
        Serial.println("any motion z");
    if ((status_0 & 0x80) == 0x80)
        Serial.println("any motion end");
    if ((status_1 & 0x80) == 0x80)
        Serial.println("any tap shock");
}

////////////////////////////////////////////////////////////////////////////////
// Setup and loop

void setup()
{
    // Enable power for the accelerometer
    pinMode(PIN_ACC_POWER_EN, OUTPUT);
    digitalWrite(PIN_ACC_POWER_EN, HIGH);
    // Start I2C
    Wire.begin();

    // Initialize accelerometer
    qma6100p_init();
    // Set thresholds for motion event and tap event
    qma6100p_motion_init(30, 300);

    // Set interrupt handler for accelerometer IRQ
    attachInterrupt(digitalPinToInterrupt(PIN_ACC_INTERRUPT), acc_irq_handler, RISING);
}

void loop()
{
    // Send out accelerometer values every second
    int16_t ax = 0, ay = 0, az = 0;
    qma6100p_read_raw_data(&ax, &ay, &az);
    Serial.printf("ax:%d, ay:%d, az:%d\n", ax, ay, az);
    delay(1000);
}

////////////////////////////////////////////////////////////////////////////////
