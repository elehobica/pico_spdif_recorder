/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include "pico/util/queue.h"

#include "wav_file.h"

class wav_file_status
{
public:
    // === Public definitions of class ===
    enum class status_t {
        RESET = 0,
        REQ_PREPARE,
        PREPARED,
        REQ_FINALIZE,
        FINALIZED
    };

    // === Public class constants ===

    // === Public class functions ===
    // functions called from core0
    static void set_wait_grant_func(void (*func)());
    static void _blocking_wait_core0_grant();
    static void _drain_core0_grant();
    static void _send_core0_grant();
    // functions called from core1
    static void initialize();
    // functions called from core0 and core1

    // === Constructor and Destructor ===
    // called from core0
    wav_file_status(status_t status = status_t::RESET);
    virtual ~wav_file_status();

    // === Public member functions ===
    // functions called from core0
    void reset();
    void prepare(const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample);
    void finalize(const bool report_flag, const float truncate_sec = 0.0f);
    void set_status(status_t status);
    bool is_status(status_t status);
    void wait_status(status_t status);
    // functions called from core1
    void req_prepare(const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample);
    void req_finalize(const bool report_flag, const float truncate_sec = 0.0f);
    uint32_t write(const uint32_t* buff, const uint32_t sub_frame_count);
    void record_queue_ratio(float queue_ratio);
    void report_start();
    void report_final();
    bool is_data_written();

    // === Public member variables ===

protected:
    // === Private definitions of class ===

    // === Private class constants ===
    static constexpr int CORE0_GRANT_QUEUE_LENGTH = 1;

    // === Private class functions ===
    // functions called from core1

    // === Private class variables ===
    static void (*_wait_grant_func)();
    static queue_t _core0_grant_queue;

    // === Private member functions ===
    // functions called from core0

    // === Private member variables ===
    wav_file* _wav_file;
    status_t _status;
};
