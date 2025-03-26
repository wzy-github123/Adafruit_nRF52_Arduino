#include "qma6100p.hpp"

#include <Adafruit_TinyUSB.h> // for Serial
#include <Arduino.h>
#include <Wire.h>             // for I2C

void qma6100p_write_reg(uint8_t reg_addr, uint8_t data)
{
    Wire.beginTransmission(QMA6100P_I2C_ADDRESS);
    Wire.write(reg_addr);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t qma6100p_read_reg(uint8_t reg_addr)
{
    Wire.beginTransmission(QMA6100P_I2C_ADDRESS);
    Wire.write(reg_addr);
    Wire.endTransmission(false);

    Wire.requestFrom(QMA6100P_I2C_ADDRESS, 1);
    if (Wire.available())
    {
        return Wire.read();
    }
    return 0;
}

void qma6100p_init()
{
    qma6100p_write_reg(QMA6100P_REG_RESET, 0xb6);
    delay(5);
    qma6100p_write_reg(QMA6100P_REG_RESET, 0x00);
    delay(10);

    // Check that accelerometer is there
    uint8_t chip_id = qma6100p_read_reg(QMA6100P_REG_CHIP_ID);
    if (chip_id == QMA6100P_DEVICE_ID)
    {
        if (Serial)
            Serial.println("\nQMA6100P exists\n");
    }
    else
    {
        if (Serial)
            Serial.println("\nQMA6100P does not exist\n");
    }

    qma6100p_write_reg(QMA6100P_REG_POWER_MANAGE, 0x80);
    qma6100p_write_reg(QMA6100P_REG_POWER_MANAGE, 0x84);
    qma6100p_write_reg(0x4a, 0x20);
    qma6100p_write_reg(0x56, 0x01);
    qma6100p_write_reg(0x5f, 0x80);
    delay(2);
    qma6100p_write_reg(0x5f, 0x00);
    delay(10);

    qma6100p_write_reg(QMA6100P_REG_RANGE, QMA6100P_RANGE_8G);
    qma6100p_write_reg(QMA6100P_REG_BW_ODR, QMA6100P_BW_100);
}

void qma6100p_motion_init(uint16_t any_th, uint16_t tap_th)
{
    any_th = constrain(any_th, 10, 2000);
    tap_th = constrain(tap_th, 10, 2000);

    uint32_t any_th_temp = (((uint32_t)any_th * 10000 / 15625) + 5) / 10;
    uint32_t tap_th_temp = (((uint32_t)tap_th * 10000 / 31250) + 5) / 10;

    any_th_temp = constrain(any_th_temp, 1, 255);
    tap_th_temp = constrain(tap_th_temp, 1, 63);

    qma6100p_write_reg(QMA6100P_REG_NO_MOTION_TH, 10);           // no motion threshold
    qma6100p_write_reg(QMA6100P_REG_ANY_MOTION_TH, any_th_temp); // any motion threshold
    qma6100p_write_reg(QMA6100P_REG_NO_ANY_MOTION_DUR, 0x8b);    // any motion duration 4*sample, no motion duration 120s
    qma6100p_write_reg(QMA6100P_REG_INT1_MAP_1, 0x81);           // any motion int map, no motion int map
    qma6100p_write_reg(QMA6100P_REG_INT_EN_2, 0xe7);             // any motion enable, no motion enable
    qma6100p_write_reg(QMA6100P_REG_INT_EN_0, 0x80);             // single tap enable
    qma6100p_write_reg(QMA6100P_REG_INT1_MAP_0, 0x80);           // single tap interrupt map
    qma6100p_write_reg(QMA6100P_REG_TAP_QUIET_TH, 3);            // tap quiet th 31.25*3 mg
    qma6100p_write_reg(QMA6100P_REG_TAP_1, 0x07);                // tap quiet time 20ms, shock time 75ms, durarion time 700ms
    qma6100p_write_reg(QMA6100P_REG_TAP_2, 0xc0 | tap_th_temp);  // tap input use x/y/z, shock threshold 31.25 mg
}

void qma6100p_read_raw_data(int16_t *ax, int16_t *ay, int16_t *az)
{
    int16_t temp = 0;
    int32_t acc_x = 0, acc_y = 0, acc_z = 0;

    temp = qma6100p_read_reg(QMA6100P_REG_XOUTL) + (qma6100p_read_reg(QMA6100P_REG_XOUTH) << 8);
    acc_x = temp >> 2;

    temp = qma6100p_read_reg(QMA6100P_REG_YOUTL) + (qma6100p_read_reg(QMA6100P_REG_YOUTH) << 8);
    acc_y = temp >> 2;

    temp = qma6100p_read_reg(QMA6100P_REG_ZOUTL) + (qma6100p_read_reg(QMA6100P_REG_ZOUTH) << 8);
    acc_z = temp >> 2;

    *ax = acc_x * QMA6100P_SENSITITY_8G / 1000.0;
    *ay = acc_y * QMA6100P_SENSITITY_8G / 1000.0;
    *az = acc_z * QMA6100P_SENSITITY_8G / 1000.0;
}

uint8_t qma6100p_get_motion_status()
{
    return qma6100p_read_reg(QMA6100P_REG_INT_STATUS_0);
}

uint8_t qma6100p_get_motion_status_1()
{
    return qma6100p_read_reg(QMA6100P_REG_INT_STATUS_1);
}
