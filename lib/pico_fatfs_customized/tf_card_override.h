/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include "ff.h"

#if !FF_FS_READONLY && !FF_FS_NORTC
BYTE get_tz (void);			/* Get current timezone */
#endif
