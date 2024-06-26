/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include "pico/util/queue.h"

#include "spdif_rx.h"

extern "C" {
void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err);
}

// === Global definitions ===
enum class bits_per_sample_t {
    _16BITS = 16,
    _24BITS = 24,
};

// spdif_rec_wav is helper class (prohibited to instantiate)
class spdif_rec_wav final
{
public:
    // === Public definitions of class ===
    enum class error_type_t {
        SPDIF_QUEUE_FULL = 0,
        ILLEGAL_SUB_FRAME_COUNT,
        WAV_FILE_CMD_QUEUE_FULL,
        RECORD_CMD_QUEUE_FULL,
        WAV_OPEN_FAIL,
        WAV_DATA_WRITE_FAIL,
        WAV_DATA_SYNC_FAIL,
        WAV_CLOSE_FAIL,
        SUFFIX_FILE_FAIL
    };

    // === Public class constants ===
    static constexpr int NUM_CHANNELS = 2;
    static constexpr int NUM_SUB_FRAME_BUF = 80; // maximize buffers to the limit for the margin of writing latency as much as possible

    // === Public class functions ===
    // functions called from core0
    static void set_wait_grant_func(void (*func)());
    static bool is_standby();
    static bool is_recording();
    static void set_blank_split(const bool flag);
    static bool get_blank_split();
    static void set_verbose(const bool flag);
    static bool get_verbose();
    static int get_suffix();
    static void clear_suffix();
    static void process_wav_file_cmd();
    // functions called from core1
    static void record_process_loop(const char* log_prefix = "log_", const char* suffix_info_filename = "last_suffix.txt");
    // functions called from core0 and core1
    static void start_recording(const bits_per_sample_t bits_per_sample, const bool standby = false);
    static void end_recording(const bool immediate_split = false);
    static void split_recording(const bits_per_sample_t bits_per_sample);
    static void report_error(const error_type_t type, const uint32_t param = 0L);
    static void log_printf(const char* fmt, ...);

    // === Public member functions ===
    // functions called from core0

    // === Public member variables ===

protected:
    // === Private definitions of class ===
    enum class blank_status_t {
        NOT_BLANK = 0,
        BLANK_DETECTED,
        BLANK_SKIP,
        BLANK_END_DETECTED
    };
    enum class record_cmd_type_t {
        STANDBY_START_CMD = 0,
        START_CMD,
        END_CMD,
        END_FOR_SPLIT_CMD
    };
    typedef struct _record_cmd_data_t {
        record_cmd_type_t cmd;
        uint32_t   param[2];
    } record_cmd_data_t;
    typedef struct _error_packet_t {
        error_type_t type;
        uint32_t     param;
    } error_packet_t;
    typedef struct _sub_frame_buf_info_t {
        int      buf_id;
        uint32_t sub_frame_count;
    } sub_frame_buf_info_t;

    // === Private class constants ===
    static constexpr int SPDIF_QUEUE_LENGTH = NUM_SUB_FRAME_BUF - 1;
    static constexpr int RECORD_CMD_QUEUE_LENGTH = 2;
    static constexpr int ERROR_QUEUE_LENGTH = 10;
    static constexpr int BLANK_LEVEL = 16;  // level to detect blank supposing 16bit data
    static constexpr int SEVERE_BLANK_LEVEL = 4;  // severe level to detect blank SKIP supposing 16bit data
    static constexpr float PRE_START_SEC = 0.02;  // the seconds to load in prior to end of blank when start (must be  < NUM_SUB_FRAME_BUF * SPDIF_BLOCK_SIZE / NUM_CHANNELS / max_sample_freq / 2)
    static constexpr float BLANK_SEC = 0.5;  // the seconds to detect the blank
    static constexpr float BLANK_REPEAT_PROHIBIT_SEC = 10.0;  // the seconds within which detecting blank is prohibited
    static constexpr float BLANK_SKIP_SEC = 10.0;  // skip recording if blank time is longer than this seconds

    // === Private class functions ===
    // functions called from core1
    static void _handle_errors();
    static void _push_sub_frame_buf(const uint32_t* buff, const uint32_t sub_frame_count);
    static int _get_last_suffix();
    static void _set_last_suffix(int suffix);
    static blank_status_t _scan_blank(const uint32_t* buff, const uint32_t sub_frame_count, const uint32_t sample_freq);

    // === Private class variables ===
    static const char* _suffix_info_filename;
    static int _suffix;
    static char _log_filename[16];
    static bool _clear_log;
    static uint32_t _sub_frame_buf[SPDIF_BLOCK_SIZE * NUM_SUB_FRAME_BUF];
    static int _sub_frame_buf_id;
    static int _adaptive_blank_level;
    static int _adaptive_severe_blank_level;
    static float _blank_sec;
    static float _severe_blank_sec;
    static float _blank_scan_sec;
    static bool _standby_flag;
    static bool _recording_flag;
    static bool _blank_split;
    static bool _verbose;
    static queue_t _spdif_queue;
    static queue_t _record_cmd_queue;
    static queue_t _error_queue;

    // === Constructor and Destructor (Prohibit) ===
    spdif_rec_wav() = delete;
    ~spdif_rec_wav() = delete;

    // === Private member functions ===
    // functions called from core0

    // === Private member variables ===

    friend void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err);
};
