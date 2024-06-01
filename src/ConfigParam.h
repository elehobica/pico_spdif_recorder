/*-----------------------------------------------------------/
/ ConfigParam.h
/------------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/-----------------------------------------------------------*/

#pragma once

#include "FlashParam.h"

//=================================
// Interface of ConfigParam class
//=================================
struct ConfigParam : FlashParamNs::FlashParam {
    static ConfigParam& instance()  // Singleton
    {
        static ConfigParam instance;
        return instance;
    }
    static constexpr uint32_t ID_BASE = FlashParamNs::CFG_ID_BASE;
    // Parameter<T>                      instance        id           name             default size
    FlashParamNs::Parameter<std::string> P_CFG_WIFI_SSID{ID_BASE + 0, "CFG_WIFI_SSID", "",     16};
    FlashParamNs::Parameter<std::string> P_CFG_WIFI_PASS{ID_BASE + 1, "CFG_WIFI_PASS", "",     16};
};
