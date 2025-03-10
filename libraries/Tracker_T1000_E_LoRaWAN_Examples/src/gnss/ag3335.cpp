#include "ag3335.hpp"
#include "minmea/minmea.h"

#include <Arduino.h>

static char gps_nmea_line[MINMEA_MAX_SENTENCE_LENGTH] = {0};

static struct minmea_sentence_rmc frame_rmc;
static struct minmea_sentence_gga frame_gga;
static struct minmea_sentence_gst frame_gst;
static struct minmea_sentence_gsv frame_gsv;
static struct minmea_sentence_vtg frame_vtg;
static struct minmea_sentence_zda frame_zda;

#define NMEA_MSG_BUFFER_SIZE 256
// Volatile to make the functions work in ISRs
volatile char g_rx1_buffer[NMEA_MSG_BUFFER_SIZE]; // Buffer to store incoming serial data
volatile uint16_t g_rx1_len = 0;                  // Current buffer length
volatile uint8_t g_rx1_line = 0;                  // Line counter

static int32_t latitude_i32 = 0, longitude_i32 = 0, speed_i32 = 0;

static void clear_serial_buffer()
{
    for (int i = 0; i < NMEA_MSG_BUFFER_SIZE; i++)
        g_rx1_buffer[i] = 0;
}

void gnss_uart_init()
{
    Serial2.begin(115200);
}

void gnss_uart_deinit()
{
    Serial2.end();
}

static uint8_t app_nmea_check_sum(const char *buf)
{
    uint8_t i = 0;
    uint8_t chk = 0;
    uint8_t len = strlen(buf);

    for (chk = buf[1], i = 2; i < len; i++)
    {
        chk ^= buf[i];
    }

    return chk;
}

void gnss_nmea_parse_line(char *line)
{
    switch (minmea_sentence_id(line, false))
    {
    case MINMEA_SENTENCE_RMC:
        minmea_parse_rmc(&frame_rmc, line);
        break;

    case MINMEA_SENTENCE_GGA:
        minmea_parse_gga(&frame_gga, line);
        break;

    case MINMEA_SENTENCE_GST:
        minmea_parse_gst(&frame_gst, line);
        break;

    case MINMEA_SENTENCE_GSV:
        minmea_parse_gsv(&frame_gsv, line);
        break;

    case MINMEA_SENTENCE_VTG:
        minmea_parse_vtg(&frame_vtg, line);
        break;

    case MINMEA_SENTENCE_ZDA:
        minmea_parse_zda(&frame_zda, line);
        break;

    case MINMEA_INVALID:
    default:
        break;
    }
}

void gnss_nmea_parse(char *str)
{
    uint16_t len = strlen(str);
    uint16_t begin = 0, end = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        if (str[i] == '$')
            begin = i;
        if (str[i] == '\r')
            end = i;

        if (end && end > begin)
        {
            memset(gps_nmea_line, 0, sizeof(gps_nmea_line));
            memcpy(gps_nmea_line, str + begin, end - begin);
            if (strncmp(gps_nmea_line, "$PAIR", 5) == 0) // ag3335 cmd parse
            {
                // TODO get version
            }
            else
            {
                gnss_nmea_parse_line(gps_nmea_line);
            }
            begin = 0;
            end = 0;
        }
    }
}

void gnss_init(void)
{
    pinMode(PIN_GNSS_POWER_EN, OUTPUT);
    digitalWrite(PIN_GNSS_POWER_EN, HIGH);
    delay(10);
    pinMode(PIN_GNSS_VRTC_EN, OUTPUT);
    digitalWrite(PIN_GNSS_VRTC_EN, HIGH);
    delay(10);

    pinMode(PIN_GNSS_RESET, OUTPUT);
    digitalWrite(PIN_GNSS_RESET, HIGH);
    delay(10);
    digitalWrite(PIN_GNSS_RESET, LOW);

    pinMode(PIN_GNSS_SLEEP_INT, OUTPUT);
    digitalWrite(PIN_GNSS_SLEEP_INT, HIGH);
    pinMode(PIN_GNSS_RTC_INT, OUTPUT);
    digitalWrite(PIN_GNSS_RTC_INT, LOW);
    pinMode(PIN_GNSS_RESETB, INPUT_PULLUP);
}

void gnss_scan_lock_sleep()
{
    char command[32] = {0};
    uint8_t check_sum = app_nmea_check_sum("$PAIR382,1");
    sprintf(command, "$PAIR382,1*%02X\r\n", check_sum);
    for (uint8_t i = 0; i < 25; i++)
    {
        Serial2.write(command);
        delay(40);
    }
}

void gnss_scan_unlock_sleep(void)
{
    char command[32] = {0};
    uint8_t check_sum = app_nmea_check_sum("$PAIR382,0");
    sprintf(command, "$PAIR382,0*%02X\r\n", check_sum);
    for (uint8_t i = 0; i < 4; i++)
    {
        Serial2.write(command);
        delay(40);
    }
}

void gnss_scan_enter_rtc_mode(void)
{
    const char *command = "$PAIR650,0*25\r\n";
    for (uint8_t i = 0; i < 25; i++)
    {
        Serial2.write(command);
        delay(40);
    }
}

void gnss_scan_clean()
{
    memset(&frame_rmc, 0, sizeof(struct minmea_sentence_rmc));
    memset(&frame_gga, 0, sizeof(struct minmea_sentence_gga));
    memset(&frame_gst, 0, sizeof(struct minmea_sentence_gst));
    memset(&frame_gsv, 0, sizeof(struct minmea_sentence_gsv));
    memset(&frame_vtg, 0, sizeof(struct minmea_sentence_vtg));
    memset(&frame_zda, 0, sizeof(struct minmea_sentence_zda));
}

bool gnss_scan_start()
{
    gnss_scan_clean();

    gnss_uart_init();

    digitalWrite(PIN_GNSS_POWER_EN, HIGH);
    delay(50);

    // Power up GPS
    digitalWrite(PIN_GNSS_RTC_INT, HIGH);
    delay(3);
    digitalWrite(PIN_GNSS_RTC_INT, LOW);
    delay(50);

    gnss_scan_lock_sleep();
    return true;
}

void gnss_scan_stop(void)
{
    gnss_scan_unlock_sleep();
    gnss_scan_enter_rtc_mode();

    delay(50);
    digitalWrite(PIN_GNSS_POWER_EN, LOW);

    // Clear any incomplete messages in buffer
    clear_serial_buffer();
    g_rx1_line = 0;
    g_rx1_len = 0;

    gnss_uart_deinit();
}

bool gnss_get_fix_status()
{
    bool result = false;
    float latitude = 0, longitude = 0, speed = 0;

    if (frame_rmc.latitude.scale && frame_rmc.longitude.scale && frame_rmc.speed.scale)
    {
        latitude = minmea_tocoord(&frame_rmc.latitude);
        longitude = minmea_tocoord(&frame_rmc.longitude);
        speed = minmea_tofloat(&frame_rmc.speed);
        if (latitude <= 180 && longitude <= 360)
        {
            latitude *= 1000000;
            longitude *= 1000000;
            speed *= 1000000;

            latitude_i32 = latitude;
            longitude_i32 = longitude;
            speed_i32 = speed;

            result = true;
        }
    }

    return result;
}

void gnss_get_position(int32_t *lat, int32_t *lon)
{
    *lat = latitude_i32;
    *lon = longitude_i32;
}

void gnss_parse_handler(char *nmea)
{
    gnss_nmea_parse(nmea);
}

void gnss_handle_serial()
{
    char g_rx1_data;

    while (Serial2.available())
    {
        g_rx1_data = Serial2.read(); // Read one byte

        // Store data in buffer if within bounds
        if (g_rx1_len < NMEA_MSG_BUFFER_SIZE - 1)
        {
            g_rx1_buffer[g_rx1_len++] = g_rx1_data;
        }

        // If buffer limit is exceeded or first character isn't '$', reset buffer
        if ((g_rx1_len >= NMEA_MSG_BUFFER_SIZE) || (g_rx1_buffer[0] != '$'))
        {
            g_rx1_len = 0;
        }

        // Check for end of a line
        if (g_rx1_data == '\n' && g_rx1_len != 0)
        {
            g_rx1_line++;

            // Process data after 2 lines received
            if (g_rx1_line >= 2)
            {
                gnss_parse_handler((char *)g_rx1_buffer); // Process GNSS message
                clear_serial_buffer();
                g_rx1_line = 0;
                g_rx1_len = 0;
            }
        }
    }
}