/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "pico/util/datetime.h"

bool run_ntp(const char* tz, struct tm& t_rtc);
