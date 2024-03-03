#include "pico_fatfs/fatfs/ff.h"

#if !FF_FS_READONLY && !FF_FS_NORTC
BYTE get_tz (void);			/* Get current timezone */
#endif
