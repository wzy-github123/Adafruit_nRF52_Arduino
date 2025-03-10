#include <Tracker_T1000_E_LoRaWAN_Examples.h>

#include <LbmT1000E.hpp>
#include <Lbmx.hpp>

// !Most important thing!
// Please follow local regulations to set lorawan duty cycle limitations
// smtc_modem_set_region_duty_cycle()

////////////////////////////////////////////////////////////////////////////////
// Types

enum class StateType
{
    Startup,
    Joining,
    Joined,
    Failed,
};

////////////////////////////////////////////////////////////////////////////////
// Constants

static constexpr smtc_modem_region_t REGION = SMTC_MODEM_REGION_AS_923_GRP1;
static const uint8_t DEV_EUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t JOIN_EUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t APP_KEY[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static constexpr uint32_t FIRST_UPLINK_DELAY = 5; // [sec.]
static constexpr uint32_t APP_TX_DUTYCYCLE = 60;  // [sec.]
static constexpr uint8_t LORAWAN_APP_PORT = 2;
static constexpr bool LORAWAN_CONFIRMED_MSG_ON = false;
static constexpr uint32_t LORAWAN_APP_DATA_MAX_SIZE = 242; // [bytes]

static constexpr uint32_t EXECUTION_PERIOD = 50; // [msec.]

////////////////////////////////////////////////////////////////////////////////
// Variables

static LbmT1000E &lbmT1000E = LbmT1000E::getInstance();
static StateType state = StateType::Startup;
static uint8_t app_data_buffer[LORAWAN_APP_DATA_MAX_SIZE];

////////////////////////////////////////////////////////////////////////////////
// MyLbmxEventHandlers

class MyLbmxEventHandlers : public LbmxEventHandlers
{
protected:
    void reset(const LbmxEvent &event) override;
    void joined(const LbmxEvent &event) override;
    void joinFail(const LbmxEvent &event) override;
    void alarm(const LbmxEvent &event) override;
    void txDone(const LbmxEvent &event) override;
    void downData(const LbmxEvent &event) override;
};

void MyLbmxEventHandlers::reset(const LbmxEvent &event)
{
    Serial.println("Application parameters:");
    Serial.printf("  - LoRaWAN uplink Fport = %d\n", LORAWAN_APP_PORT);
    Serial.printf("  - DM report interval   = %d\n", APP_TX_DUTYCYCLE);
    Serial.printf("  - Confirmed uplink     = %s\n", (LORAWAN_CONFIRMED_MSG_ON == true) ? "Yes" : "No");

    if (LbmxEngine::setRegion(REGION) != SMTC_MODEM_RC_OK)
        abort();
    if (LbmxEngine::setOTAA(DEV_EUI, JOIN_EUI, APP_KEY) != SMTC_MODEM_RC_OK)
        abort();

    Serial.println("Join the LoRaWAN network.");
    if (LbmxEngine::joinNetwork() != SMTC_MODEM_RC_OK)
        abort();

    // if((REGION == SMTC_MODEM_REGION_EU_868) || (REGION == SMTC_MODEM_REGION_RU_864))
    // {
    //     smtc_modem_set_region_duty_cycle( false );
    // }

    state = StateType::Joining;
}

void MyLbmxEventHandlers::joined(const LbmxEvent &event)
{
    state = StateType::Joined;
    Serial.println("Start the alarm event.");
    if (LbmxEngine::startAlarm(FIRST_UPLINK_DELAY) != SMTC_MODEM_RC_OK)
        abort();
}

void MyLbmxEventHandlers::joinFail(const LbmxEvent &event)
{
    state = StateType::Failed;
}

void MyLbmxEventHandlers::alarm(const LbmxEvent &event)
{
    uint32_t charge = 0;
    uint8_t app_data_size = 0;
    smtc_modem_get_charge(&charge);
    app_data_buffer[app_data_size++] = (uint8_t)(charge);
    app_data_buffer[app_data_size++] = (uint8_t)(charge >> 8);
    app_data_buffer[app_data_size++] = (uint8_t)(charge >> 16);
    app_data_buffer[app_data_size++] = (uint8_t)(charge >> 24);

    if (LbmxEngine::requestUplink(LORAWAN_APP_PORT, LORAWAN_CONFIRMED_MSG_ON, app_data_buffer, app_data_size) != SMTC_MODEM_RC_OK)
        abort();

    /* Schedule next packet transmission */
    if (LbmxEngine::startAlarm(APP_TX_DUTYCYCLE) != SMTC_MODEM_RC_OK)
        abort();
}

void MyLbmxEventHandlers::txDone(const LbmxEvent &event)
{
    static uint32_t uplink_count = 0;
    Serial.printf("Uplink count: %d\n", ++uplink_count);
}

void MyLbmxEventHandlers::downData(const LbmxEvent &event)
{
    Serial.println("Downlink received:");
    Serial.printf("  - LoRaWAN Fport = %d\n", event.event_data.downdata.fport);
    Serial.printf("  - Payload size  = %d\n", event.event_data.downdata.length);
    Serial.printf("  - RSSI          = %d dBm\n", event.event_data.downdata.rssi - 64);
    Serial.printf("  - SNR           = %d dB\n", event.event_data.downdata.snr >> 2);

    switch (event.event_data.downdata.window)
    {
    case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX1:
    {
        Serial.printf("  - Rx window     = %s\n", xstr(SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX1));
        break;
    }
    case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX2:
    {
        Serial.printf("  - Rx window     = %s\n", xstr(SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX2));
        break;
    }
    case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC:
    {
        Serial.printf("  - Rx window     = %s\n", xstr(SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC));
        break;
    }
    default:
        break;
    }

    if (event.event_data.downdata.length != 0)
    {
        Serial.printf("Payload", event.event_data.downdata.data, event.event_data.downdata.length);
    }
}

////////////////////////////////////////////////////////////////////////////////
// ModemEventHandler

static void ModemEventHandler()
{
    static LbmxEvent event;
    static MyLbmxEventHandlers handlers;

    while (event.fetch())
    {
        Serial.printf("----- %s -----\n", event.getEventString().c_str());

        handlers.invoke(event);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Setup and loop

void setup()
{
    delay(1000);
    Serial.println("\n---------- STARTUP ----------");

    lbmT1000E.begin();
    LbmxEngine::begin(lbmT1000E.getRadio(), ModemEventHandler);

    LbmxEngine::printVersions(lbmT1000E.getRadio());
}

void loop()
{
    const uint32_t sleepTime = LbmxEngine::doWork();
    delay(min(sleepTime, EXECUTION_PERIOD));
}

////////////////////////////////////////////////////////////////////////////////
