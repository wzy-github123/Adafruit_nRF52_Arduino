/*
 * T1000EHardware.cpp
 * Copyright (C) 2023 Seeed K.K.
 * MIT License
 */

////////////////////////////////////////////////////////////////////////////////
// Includes

#include "T1000EHardware.hpp"
#include "internal/nrf_hal/system.hpp"
#include "lbm/smtc_modem_core/smtc_ralf/src/ralf_lr11xx.h"

////////////////////////////////////////////////////////////////////////////////
// Variables

T1000EHardware* T1000EHardware::instance_ = nullptr;

////////////////////////////////////////////////////////////////////////////////
// T1000EHardware

T1000EHardware& T1000EHardware::getInstance()
{
    if (instance_ == nullptr)
    {
        instance_ = new T1000EHardware();
    }

    return *instance_;
}

T1000EHardware::T1000EHardware()
{
    radio = RALF_LR11XX_INSTANTIATE(this);
}

void T1000EHardware::attachGnssPrescan(const std::function<void(void*)>& func, void* context)
{
    gnssPrescan_ = { func, context };
}

void T1000EHardware::invokeGnssPrescan()
{
    const std::function<void(void*)>& func = std::get<0>(gnssPrescan_);
    void* context = std::get<1>(gnssPrescan_);

    if (func) func(context);
}

void T1000EHardware::attachGnssPostscan(const std::function<void(void*)>& func, void* context)
{
    gnssPostscan_ = { func, context };
}

void T1000EHardware::invokeGnssPostscan()
{
    const std::function<void(void*)>& func = std::get<0>(gnssPostscan_);
    void* context = std::get<1>(gnssPostscan_);

    if (func) func(context);
}

void T1000EHardware::begin()
{
    nrf_hal::System::startHfclk();

    reset_.begin();
    busy_.begin();
    irq_.begin();
    ss_.begin();
    spim_.begin();

    rtcTimer.begin();
    rng.begin();
    timerOneshot.begin();

    radioMode_ = RadioMode::AWAKE;
}

void T1000EHardware::reset()
{
    ss_.write(1);

    // Putting the NRESET signal to low for at least 100 μs restarts the LR1110. / LR1110 Transceiver User Manual
    reset_.write(0);
    nrf_hal::System::SpinDelayUs(200);
    reset_.write(1);
    nrf_hal::System::SpinDelayUs(200);

    // At the end of the startup sequence, the device is set in Standby RC mode, the BUSY signal goes low and the device accepts commands. / LR1110 Transceiver User Manual
    while (busy_.read()) nrf_hal::System::delayMs(1);

    radioMode_ = RadioMode::AWAKE;
}

void T1000EHardware::beginTransfer()
{
    ss_.write(0);
}

void T1000EHardware::endTransfer()
{
    ss_.write(1);
}

void T1000EHardware::transfer(const uint8_t* txData, uint8_t* rxData, size_t length)
{
    spim_.transfer(txData, rxData, length);
}

void T1000EHardware::changedToSleep()
{
    radioMode_ = RadioMode::SLEEP;
}

void T1000EHardware::wakeupAndWaitForReady()
{
    if (radioMode_ == RadioMode::SLEEP)
    {
        beginTransfer();
        endTransfer();

        radioMode_ = RadioMode::AWAKE;
    }

    while (busy_.read()){}  // Seeed: Spin
}
void T1000EHardware::enterBootloaderMode()
{
    busy_.end();
    outbusy_.begin();
    outbusy_.write(0);
    reset();
    nrf_hal::System::delayMs(500);
    outbusy_.end();
    busy_.begin();
    nrf_hal::System::delayMs( 100 );

}
////////////////////////////////////////////////////////////////////////////////
