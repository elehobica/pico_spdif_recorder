/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include "hardware/rtc.h"
#include "pico/stdlib.h"

#include "tf_card_override.h"

#if !FF_FS_READONLY && !FF_FS_NORTC
/* get the current time */
DWORD get_fattime (void)
{
    datetime_t t;
    rtc_get_datetime(&t);

    return ((DWORD) (t.year - 1980) << 25) |
           ((DWORD) t.month         << 21) |
           ((DWORD) t.day           << 16) |
           ((DWORD) t.hour          << 11) |
           ((DWORD) t.min           <<  5) |
           ((DWORD) t.sec           >>  1); // sec /2
}

BYTE get_tz (void)
{
    return (1<<7);  // TZ: on
}
#endif
