/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include "config_wifi.h"

#include <cstdio>

#include "pico/stdlib.h"

static constexpr size_t STR_SIZE = 256;
static std::string ssid, password;

static bool _get_item(const char* id, char* str, bool num_only = false)
{
    int chr;
    int pos = 0;
    int esc_seq = 0;
    char esc_seq_str[2];
    printf("%s: ", id);
    while (true) {
        if ((chr = getchar_timeout_us(1)) != PICO_ERROR_TIMEOUT) {
            char c = static_cast<char>(chr);
            if (esc_seq) {
                esc_seq_str[(esc_seq - 1) % 2] = c;
                esc_seq++;
                if (esc_seq == 3 && esc_seq_str[0] == '[' && esc_seq_str[1] == 'D') {  // <-
                    if (pos > 0) {
                        putchar('\x08');
                        putchar(' ');
                        putchar('\x08');
                        pos--;
                    }
                }
            } else if (chr == 0x1b) {  // Esc
                esc_seq++;
            } else if ((chr >= 0x20 && chr < 0x7f && !num_only) || (((chr >= '0' && chr <= '9') || (pos == 0 && chr == '+') || (pos == 0 && chr == '-')) && num_only)) {
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
                if (pos > 0) {
                    str[pos] = '\0';
                    for (int i = 0; i < pos; i++) putchar('\x08');
                    for (int i = 0; i < pos; i++) putchar(str[i]);
                    return true;
                }
            }
        } else {
            if (esc_seq == 1) {  // single Esc
                for (int i = 0; i < pos; i++) putchar('\x08');
                for (int i = 0; i < pos; i++) putchar(' ');
                return false;
            }
            esc_seq = 0;
        }
    }

    return false;
}

bool config_wifi(std::string& ssid, std::string& password)
{
    bool flag;
    char str[STR_SIZE] = {};

    printf("[Wi-Fi configuration]\r\n");
    flag = _get_item("SSID", str);
    printf("\r\n");
    if (flag) {
        ssid = std::string(str);
        flag = _get_item("Password", str);
        printf("\r\n");
    }
    if (!flag) {
        printf("Wi-Fi configuration canceled\r\n");
        return false;
    }
    password = std::string(str);
    printf("Wi-Fi configured\r\n");

    return true;
}