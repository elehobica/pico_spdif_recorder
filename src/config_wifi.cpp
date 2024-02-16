/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include "config_wifi.h"

#include <cstdio>
#include <string>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "ntp_client.h"

constexpr size_t STR_SIZE = 256;


static bool _get_item(const char* id, char* str, bool num_only = false)
{
    int chr;
    int pos = 0;
    printf("%s: ", id);
    while (true) {
        if ((chr = getchar_timeout_us(1)) != PICO_ERROR_TIMEOUT) {
            char c = static_cast<char>(chr);
            if ((chr >= 0x20 && chr < 0x7f && !num_only) || (((chr >= '0' && chr <= '9') || chr == '+' || chr == '-') && num_only)) {
                if (pos < STR_SIZE - 1) {
                    putchar(c);
                    str[pos] = c;
                    pos++;
                }
            } else if (chr == 0x08 || chr == 0x7f) {  // BS
                if (pos > 0) {
                    putchar('\x08');
                    putchar(' ');
                    putchar('\x08');
                    pos--;
                }
            } else if (chr == 0x0d) {  // Enter
                str[pos] = '\0';
                for (int i = 0; i < pos; i++) putchar('\x08');
                printf("%s", str);
                return true;
            } else if (chr == 0x1b) {  // Esc
                for (int i = 0; i < pos; i++) putchar('\x08');
                for (int i = 0; i < pos; i++) putchar(' ');
                return false;
            }
        }
    }
}

bool config_wifi()
{
    bool flag;
    char str[STR_SIZE] = {};
    std::string ssid, password;
    int tz = 0;

    printf("[Wi-Fi / TZ configuration]\r\n");
    flag = _get_item("SSID", str);
    printf("\r\n");
    if (flag) {
        ssid = std::string(str);
        flag = _get_item("Password", str);
        printf("\r\n");
    }
    if (flag) {
        password = std::string(str);
        flag = _get_item("TZ", str, true);
        printf("\r\n");
    }
    if (!flag) {
        printf("Wi-Fi / TZ configuration canceled\r\n");
        return false;
    }
    tz = atoi(str);
    printf("Wi-Fi / TZ configured\r\n");
    printf("... connecting Wi-Fi\r\n");

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(ssid.c_str(), password.c_str(), CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("failed to connect\r\n");
        return false;
    }
    printf("connected\r\n");
    run_ntp(tz);

    return true;
}