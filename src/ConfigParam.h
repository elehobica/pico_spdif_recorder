/*-----------------------------------------------------------/
/ ConfigParam.h
/------------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/-----------------------------------------------------------*/

#pragma once

#include <stdint.h>
#include <stdlib.h>

#define configParam             (ConfigParam::ConfigParamUser::instance())

#define GETBOOL(identifier)     (configParam.getBool(identifier))
#define GETU8(identifier)       (configParam.getU8(identifier))
#define GETU16(identifier)      (configParam.getU16(identifier))
#define GETU32(identifier)      (configParam.getU32(identifier))
#define GETU64(identifier)      (configParam.getU64(identifier))
#define GETI8(identifier)       (configParam.getI8(identifier))
#define GETI16(identifier)      (configParam.getI16(identifier))
#define GETI32(identifier)      (configParam.getI32(identifier))
#define GETI64(identifier)      (configParam.getI64(identifier))
#define GETSTR(identifier)      (configParam.getStr(identifier))

#define GET_CFG_BOOT_COUNT GETU32(ConfigParam::CFG_BOOT_COUNT)
#define GET_CFG_WIFI_SSID  GETSTR(ConfigParam::CFG_WIFI_SSID)
#define GET_CFG_WIFI_PASS  GETSTR(ConfigParam::CFG_WIFI_PASS)

namespace ConfigParam
{
    enum class ParamType {
        CFG_NONE_T,
        CFG_BOOL_T,
        CFG_STRING_T,
        CFG_UINT8_T,
        CFG_UINT16_T,
        CFG_UINT32_T,
        CFG_UINT64_T,
        CFG_INT8_T,
        CFG_INT16_T,
        CFG_INT32_T,
        CFG_INT64_T
    };

    enum class LoadBehavior {
        LOAD_DEFAULT_IF_FLASH_IS_BLANK = 0,
        FORCE_LOAD_DEFAULT,
        ALWAYS_LOAD_FROM_FLASH
    };

    typedef enum {
        CFG_BOOT_COUNT = 0,
        CFG_WIFI_SSID,
        CFG_WIFI_PASS,
        __NUM_CFG_PARAMS__
    } ParamID_t;

    typedef struct _ParamItem_t {
        //const ParamID_t id;
        const uint32_t id;
        const char *name;
        const ParamType paramType;
        const size_t size;
        const char *defaultValue;
        const uint32_t flashAddr;
        void *ptr;
    } ParamItem_t;

    //====================================
    // Interface of ConfigParamUser class
    //====================================
    class ConfigParamUser
    {
    public:
        static ConfigParamUser& instance(); // Singleton
        void printInfo();
        void initialize(LoadBehavior loadDefaultBehavior = LoadBehavior::LOAD_DEFAULT_IF_FLASH_IS_BLANK);
        void incBootCount();
        void finalize();
        void read(uint32_t id, void *ptr);
        void write(uint32_t id, const void *ptr);
        bool getBool(uint32_t id);
        uint8_t getU8(uint32_t id);
        uint16_t getU16(uint32_t id);
        uint32_t getU32(uint32_t id);
        uint64_t getU64(uint32_t id);
        int8_t getI8(uint32_t id);
        int16_t getI16(uint32_t id);
        int32_t getI32(uint32_t id);
        int64_t getI64(uint32_t id);
        char *getStr(uint32_t id);
        void setU8(uint32_t id, const uint8_t val);
        void setU16(uint32_t id, const uint16_t val);
        void setU32(uint32_t id, const uint32_t val);
        void setU64(uint32_t id, const uint64_t val);
        void setI8(uint32_t id, const int8_t val);
        void setI16(uint32_t id, const int16_t val);
        void setI32(uint32_t id, const int32_t val);
        void setI64(uint32_t id, const int64_t val);
        void setStr(uint32_t id, const char *str);

    protected:
        ParamItem_t configParamItems[__NUM_CFG_PARAMS__] = {
        //  id               name                type                     size default ofs     ptr
            {CFG_BOOT_COUNT, "CFG_BOOT_COUNT",   ParamType::CFG_UINT32_T,   4, "0",    0x000,  nullptr},
            {CFG_WIFI_SSID,  "CFG_WIFI_SSID",    ParamType::CFG_STRING_T,  16, "",     0x004,  nullptr},
            {CFG_WIFI_PASS,  "CFG_WIFI_PASS",    ParamType::CFG_STRING_T,  16, "",     0x014,  nullptr}
        };
        int _numParams;

        ConfigParamUser(int numParams);
        virtual ~ConfigParamUser();
        ConfigParamUser(const ConfigParamUser&) = delete;
        ConfigParamUser& operator=(const ConfigParamUser&) = delete;
        uint32_t getBootCountFromFlash();
        void loadDefault();
    };

}  // namespace ConfigParam

