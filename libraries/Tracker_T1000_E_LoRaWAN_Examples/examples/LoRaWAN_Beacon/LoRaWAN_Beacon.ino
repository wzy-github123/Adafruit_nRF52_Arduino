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

static const uint8_t adr_custom_list_eu868_default[16] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5}; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7
static const uint8_t adr_custom_list_us915_default[16] = {1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3}; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7
static const uint8_t adr_custom_list_au915_default[16] = {3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5}; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7
static const uint8_t adr_custom_list_as923_default[16] = {3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5}; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7
static const uint8_t adr_custom_list_kr920_default[16] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5}; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7
static const uint8_t adr_custom_list_in865_default[16] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5}; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7
static const uint8_t adr_custom_list_ru864_default[16] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5}; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7

static constexpr uint32_t BEACON_SCAN_PERIOD_DEFAULT = 60; // [sec.]
static constexpr uint32_t BEACON_SCAN_TIME = 5;            // [sec.]
static constexpr uint8_t LORAWAN_APP_PORT = 2;
static constexpr bool LORAWAN_CONFIRMED_MSG_ON = false;
static constexpr uint32_t LORAWAN_APP_DATA_MAX_SIZE = 242; // [bytes]

static constexpr uint32_t EXECUTION_PERIOD = 50; // [msec.]

////////////////////////////////////////////////////////////////////////////////
// Variables

static LbmT1000E &lbmT1000E = LbmT1000E::getInstance();
static StateType state = StateType::Startup;
static uint8_t app_data_buffer[LORAWAN_APP_DATA_MAX_SIZE];
static uint8_t app_data_len = 0;
static uint8_t beacon_scan_status = 0;

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
    Serial.printf("  - DM report interval   = %d\n", BEACON_SCAN_PERIOD_DEFAULT);
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

    // Enable ADR
    uint8_t adr_custom_list_region[16] = {0};
    smtc_modem_region_t region;
    smtc_modem_get_region(0, &region);
    switch (region)
    {
    case SMTC_MODEM_REGION_EU_868:
        memcpy(adr_custom_list_region, adr_custom_list_eu868_default, sizeof(adr_custom_list_region));
        break;

    case SMTC_MODEM_REGION_US_915:
        memcpy(adr_custom_list_region, adr_custom_list_us915_default, sizeof(adr_custom_list_region));
        break;

    case SMTC_MODEM_REGION_AU_915:
        memcpy(adr_custom_list_region, adr_custom_list_au915_default, sizeof(adr_custom_list_region));
        break;

    case SMTC_MODEM_REGION_AS_923_GRP1:
    case SMTC_MODEM_REGION_AS_923_GRP2:
    case SMTC_MODEM_REGION_AS_923_GRP3:
    case SMTC_MODEM_REGION_AS_923_GRP4:
        memcpy(adr_custom_list_region, adr_custom_list_as923_default, sizeof(adr_custom_list_region));
        break;

    case SMTC_MODEM_REGION_KR_920:
        memcpy(adr_custom_list_region, adr_custom_list_kr920_default, sizeof(adr_custom_list_region));
        break;

    case SMTC_MODEM_REGION_IN_865:
        memcpy(adr_custom_list_region, adr_custom_list_in865_default, sizeof(adr_custom_list_region));
        break;

    case SMTC_MODEM_REGION_RU_864:
        memcpy(adr_custom_list_region, adr_custom_list_ru864_default, sizeof(adr_custom_list_region));
        break;

    default:
        break;
    }
    if (smtc_modem_adr_set_profile(0, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_list_region) != SMTC_MODEM_RC_OK)
        abort();

    Serial.println("Start the alarm event.");
    if (LbmxEngine::startAlarm(5) != SMTC_MODEM_RC_OK)
        abort();
}

void MyLbmxEventHandlers::joinFail(const LbmxEvent &event)
{
    state = StateType::Failed;
}

void MyLbmxEventHandlers::alarm(const LbmxEvent &event)
{
    if (beacon_scan_status == 0) // beacon scan
    {
        if (LbmxEngine::startAlarm(BEACON_SCAN_TIME) != SMTC_MODEM_RC_OK)
            abort();
        Serial.printf("Restart alarm timer in %d s\n", BEACON_SCAN_TIME);

        beacon_scan_status = 1;
        app_data_len = 0;
        memset(app_data_buffer, 0, sizeof(app_data_buffer));
        ble_scan_start();
    }
    else if (beacon_scan_status == 1) // send data
    {
        int32_t next_delay = BEACON_SCAN_PERIOD_DEFAULT - BEACON_SCAN_TIME;
        if (LbmxEngine::startAlarm(next_delay > 0 ? next_delay : 1000) != SMTC_MODEM_RC_OK)
            abort();
        Serial.printf("Restart alarm timer in %d s\n", next_delay > 0 ? next_delay : 1000);

        beacon_scan_status = 0;
        ble_scan_stop();
        ble_get_results(app_data_buffer, &app_data_len);
        ble_display_results();

        Serial.println("Sending uplink containing received iBeacons");
        if (LbmxEngine::requestUplink(LORAWAN_APP_PORT, LORAWAN_CONFIRMED_MSG_ON, app_data_buffer, app_data_len) != SMTC_MODEM_RC_OK)
            abort();
    }
}

void MyLbmxEventHandlers::txDone(const LbmxEvent &event)
{
    static uint32_t uplink_count = 0;
    Serial.println("Sending uplink done.");
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
        Serial.printf("\n\n----- Modem event: %s -----\n", event.getEventString().c_str());

        handlers.invoke(event);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Setup and loop

void setup()
{
    delay(1000);
    Serial.println("\n###### ===== T1000-E Beacon scan example ==== ######\n");

    ble_scan_init();

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
