/*-----------------------------------------------------------/
/ ConfigParam.h
/------------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/-----------------------------------------------------------*/

#pragma once

#include "FlashParam.h"

typedef enum {
    CFG_ID_WIFI_SSID = FlashParamNs::CFG_ID_BASE,
    CFG_ID_WIFI_PASS,
} ParamId_t;

//=================================
// Interface of ConfigParam class
//=================================
struct ConfigParam : FlashParamNs::FlashParam {
    static ConfigParam& instance() {  // Singleton
        static ConfigParam instance;
        return instance;
    }
    // Parameter<T>                      instance        id           name             default size
    FlashParamNs::Parameter<std::string> P_CFG_WIFI_SSID{CFG_ID_WIFI_SSID, "CFG_WIFI_SSID", "",     16};
    FlashParamNs::Parameter<std::string> P_CFG_WIFI_PASS{CFG_ID_WIFI_PASS, "CFG_WIFI_PASS", "",     16};
};
