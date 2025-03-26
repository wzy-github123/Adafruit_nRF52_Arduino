/*
 * LbmT1000E.cpp
 * Copyright (C) 2023 Seeed K.K.
 * MIT License
 */

////////////////////////////////////////////////////////////////////////////////
// Includes

#include "LbmT1000E.hpp"
#include "T1000EHardware.hpp"

////////////////////////////////////////////////////////////////////////////////
// Variables

LbmT1000E* LbmT1000E::instance_ = nullptr;

////////////////////////////////////////////////////////////////////////////////
// LbmT1000E

LbmT1000E& LbmT1000E::getInstance()
{
    if (instance_ == nullptr)
    {
        instance_ = new LbmT1000E();
    }

    return *instance_;
}

LbmT1000E::LbmT1000E()
{
}

void LbmT1000E::attachGnssPrescan(const std::function<void(void*)>& func, void* context)
{
    T1000EHardware::getInstance().attachGnssPrescan(func, context);
}

void LbmT1000E::attachGnssPostscan(const std::function<void(void*)>& func, void* context)
{
    T1000EHardware::getInstance().attachGnssPostscan(func, context);
}

void LbmT1000E::begin()
{
    T1000EHardware::getInstance().begin();
}

ralf_t* LbmT1000E::getRadio()
{
    return &T1000EHardware::getInstance().radio;
}

void LbmT1000E::startWatchdog()
{
    T1000EHardware::getInstance().wdt.begin();
}

void LbmT1000E::reloadWatchdog()
{
    T1000EHardware::getInstance().wdt.reload();
}

////////////////////////////////////////////////////////////////////////////////
