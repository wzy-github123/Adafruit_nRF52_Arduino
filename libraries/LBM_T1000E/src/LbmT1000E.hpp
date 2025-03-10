/*
 * LbmT1000E.hpp
 * Copyright (C) 2023 Seeed K.K.
 * MIT License
 */

#pragma once

////////////////////////////////////////////////////////////////////////////////
// Includes

#include <functional>
#include <InternalFileSystem.h> // Load dependency library

////////////////////////////////////////////////////////////////////////////////
// Types

struct ralf_s;
typedef struct ralf_s ralf_t;

////////////////////////////////////////////////////////////////////////////////
// LbmT1000E

class LbmT1000E
{
private:
    static LbmT1000E* instance_;

public:
    static LbmT1000E& getInstance();

private:
    LbmT1000E();
    LbmT1000E(const LbmT1000E&) = delete;

public:
    static void attachGnssPrescan(const std::function<void(void*)>& func, void* context = nullptr);
    static void attachGnssPostscan(const std::function<void(void*)>& func, void* context = nullptr);

    static void begin();
    static ralf_t* getRadio();
    static void startWatchdog();
    static void reloadWatchdog();

};

////////////////////////////////////////////////////////////////////////////////
