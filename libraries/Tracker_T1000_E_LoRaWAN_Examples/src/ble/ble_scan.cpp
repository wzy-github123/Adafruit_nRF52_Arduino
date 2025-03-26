#include "ble_scan.hpp"

#include "util.hpp"

#include <bluefruit.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Beacon specification, we are looking for apple iBeacons
#define BEACON_DATA_LEN 0x15
#define BEACON_DATA_TYPE 0x02
#define COMPANY_IDENTIFIER 0x004C

// Maximum number of beacons that can be stored
#define BLE_BEACON_BUF_MAX 16
// Number of beacons which are returned by ble_get_results
#define BLE_BEACON_SEND_NUM 5

// Buffer to store found beacons
BleBeacons_t ble_beacon_buf[BLE_BEACON_BUF_MAX] = {0};
uint8_t ble_beacon_res_num = 0;

// UUID filter configuration. Set to true and add UUIDs to apply
bool uuid_filter_enabled = false;
uint8_t ble_uuid_filter_array[16] = {0};

// Helper array to store beacon indices sorted by RSSI
uint8_t ble_beacon_rssi_array[BLE_BEACON_BUF_MAX] = {0};

// Function returning true if the givven uuid is in the filter array
static bool beacon_uuid_filter(uint8_t *uuid)
{
    if (!uuid_filter_enabled)
        return true;
    return (memcmp(uuid, ble_uuid_filter_array, 16) == 0);
}

// Function comparing two buffers, returning true if they are equal
static bool buf_cmp_value(uint8_t *a, uint8_t *b, uint8_t len)
{
    return memcmp(a, b, len) == 0;
}

// Function to compare two beacons' UUID, major, minor, and MAC address
static bool is_beacon_duplicate(BleBeacons_t *new_beacon)
{
    for (uint8_t i = 0; i < BLE_BEACON_BUF_MAX; i++)
    {
        // Compare UUID
        if (buf_cmp_value(ble_beacon_buf[i].uuid, new_beacon->uuid, 16) &&
            // Compare Major
            buf_cmp_value((uint8_t *)&ble_beacon_buf[i].major, (uint8_t *)&new_beacon->major, 2) &&
            // Compare Minor
            buf_cmp_value((uint8_t *)&ble_beacon_buf[i].minor, (uint8_t *)&new_beacon->minor, 2) &&
            // Note: Don't take RSSI into account
            // Compare MAC address
            buf_cmp_value((uint8_t *)&ble_beacon_buf[i].mac, (uint8_t *)&new_beacon->mac, 6))
        {
            // Duplicate found
            return true;
        }
    }
    // No duplicate found
    return false;
}

// Callback to react on received beacons
static void scan_callback(ble_gap_evt_adv_report_t *report)
{
    if (!report)
        return;

    uint8_t len = 0;
    uint8_t buffer[32] = {0};
    len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buffer, sizeof(buffer));
    int8_t rssi = report->rssi;

    if (len && buffer[2] == BEACON_DATA_TYPE && buffer[3] == BEACON_DATA_LEN)
    {
        uint16_t company_id = (buffer[0] | (buffer[1] << 8));
        if (company_id == COMPANY_IDENTIFIER)
        {
            BleBeacons_t beacon;
            beacon.company_id = company_id;
            memcpy(beacon.uuid, &buffer[4], 16);
            memcpy(&beacon.major, &buffer[20], 2);
            memcpy(&beacon.minor, &buffer[22], 2);
            beacon.rssi = buffer[24];
            beacon.rssi_ = rssi;
            memcpy(beacon.mac, report->peer_addr.addr, 6);

            // Save beacon if 1. buffer is not full, 2. is not a duplicate, 3. uuid should not be filtered
            if (ble_beacon_res_num < BLE_BEACON_BUF_MAX && !is_beacon_duplicate(&beacon) && (beacon_uuid_filter((uint8_t *)&buffer[4]) != 0))
            {
                // Check if beacon already exists

                // Add the new beacon to the buffer
                ble_beacon_buf[ble_beacon_res_num++] = beacon;

                // Update the rssi_array with the new beacon index
                ble_beacon_rssi_array[ble_beacon_res_num - 1] = ble_beacon_res_num - 1;

                // Sort the array based on RSSI (descending order)
                for (uint8_t i = ble_beacon_res_num - 1; i > 0; i--)
                {
                    if (ble_beacon_buf[ble_beacon_rssi_array[i]].rssi_ > ble_beacon_buf[ble_beacon_rssi_array[i - 1]].rssi_)
                    {
                        uint8_t temp = ble_beacon_rssi_array[i];
                        ble_beacon_rssi_array[i] = ble_beacon_rssi_array[i - 1];
                        ble_beacon_rssi_array[i - 1] = temp;
                    }
                    else
                    {
                        // If RSSI is already in the correct order, no need to swap further
                        break;
                    }
                }
            }
        }
    }
    Bluefruit.Scanner.resume();
}

void ble_scan_init()
{
    Bluefruit.begin();
    Bluefruit.setName("BLE Scanner");
    Bluefruit.Scanner.setRxCallback(scan_callback);
}

bool ble_scan_start()
{
    memset(ble_beacon_buf, 0, sizeof(ble_beacon_buf));
    ble_beacon_res_num = 0;
    Bluefruit.Scanner.start(0);
    return true;
}

bool ble_get_results(uint8_t *result, uint8_t *size)
{
    if (!result || !size || ble_beacon_res_num == 0)
        return false;
    *size = 0;

    // The array is already sorted by RSSI in scan_callback, so no need to sort again
    uint8_t beacon_num = (ble_beacon_res_num > BLE_BEACON_SEND_NUM) ? BLE_BEACON_SEND_NUM : ble_beacon_res_num;

    for (uint8_t i = 0; i < beacon_num; i++)
    {
        memcpy(result + i * 7, ble_beacon_buf[ble_beacon_rssi_array[i]].mac, 6);
        result[i * 7 + 6] = ble_beacon_buf[ble_beacon_rssi_array[i]].rssi_;
        *size += 7;
    }
    return (beacon_num > 0);
}

void ble_scan_stop()
{
    Bluefruit.Scanner.stop();
}

// Displays the results in sorted order based on RSSI
void ble_display_results()
{
    Serial.printf("Number of received iBeacons: %d\r\n", ble_beacon_res_num);

    // Display the sorted beacons using the rssi_array
    for (uint8_t i = 0; i < ble_beacon_res_num; i++)
    {
        // Print company ID
        Serial.printf("%04x, ", ble_beacon_buf[ble_beacon_rssi_array[i]].company_id);

        // Print UUID
        for (uint8_t j = 0; j < 16; j++)
        {
            Serial.printf("%02x ", ble_beacon_buf[ble_beacon_rssi_array[i]].uuid[j]);
        }

        // Print Major and Minor
        uint16_t major = 0, minor = 0;
        memcpyr(&major, &ble_beacon_buf[ble_beacon_rssi_array[i]].major, 2);
        memcpyr(&minor, &ble_beacon_buf[ble_beacon_rssi_array[i]].minor, 2);
        Serial.printf(", %04x, %04x, ", major, minor);

        // Print RSSI values
        Serial.printf("%d/%d dBm, ", ble_beacon_buf[ble_beacon_rssi_array[i]].rssi, ble_beacon_buf[ble_beacon_rssi_array[i]].rssi_);

        // Print MAC address
        for (uint8_t j = 0; j < 5; j++)
        {
            Serial.printf("%02x:", ble_beacon_buf[ble_beacon_rssi_array[i]].mac[5 - j]);
        }
        Serial.printf("%02x\r\n", ble_beacon_buf[ble_beacon_rssi_array[i]].mac[0]);
    }
}
