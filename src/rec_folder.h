/*------------------------------------------------------/
/ Copyright (c) 2026, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include "ff.h"

// rec_folder is helper class (prohibited to instantiate) to decide the folder to record at boot
// (called only from core0 before core1 starts)
class rec_folder final
{
public:
    // === Public class functions ===
    static bool setup(int& folder_num);

protected:
    // === Private class constants ===
    static constexpr int MAX_NUM_DIGITS = 9;  // to avoid int overflow
    static constexpr int FOLDER_NAME_SIZE = 12;  // enough for any int
    static constexpr int PATH_SIZE = FOLDER_NAME_SIZE + 1 + FF_LFN_BUF + 1;

    // === Private class functions ===
    static bool _parse_num(const char* name, int& num);
    static bool _is_wav_name(const char* name);
    static bool _is_empty(const char* folder);
    static bool _remove(const char* folder);
    static bool _make(const int num, int& folder_num);

    // === Private class variables ===
    static DIR _dir;  // static not to consume stack
    static FILINFO _fno;
    static char _path[PATH_SIZE];

    // === Constructor and Destructor (Prohibit) ===
    rec_folder() = delete;
    ~rec_folder() = delete;
};
