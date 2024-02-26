/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include <string>

bool config_wifi();
bool connect_wifi(const std::string& ssid, const std::string& password, const int retry = 3);
