/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <time.h>

#include "pico/stdlib.h"
#include "pico/aon_timer.h"

#include "tf_card_override.h"

#if !FF_FS_READONLY && !FF_FS_NORTC
/* get the current time */
DWORD get_fattime (void)
{
    struct tm t = {};
    aon_timer_get_time_calendar(&t);

    return ((DWORD) (t.tm_year - 80) << 25) |
           ((DWORD) (t.tm_mon + 1)   << 21) |
           ((DWORD) t.tm_mday        << 16) |
           ((DWORD) t.tm_hour        << 11) |
           ((DWORD) t.tm_min         <<  5) |
           ((DWORD) t.tm_sec         >>  1); // sec /2
}

BYTE get_tz (void)
{
    return (1<<7);  // TZ: on
}
#endif
