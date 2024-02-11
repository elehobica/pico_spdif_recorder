/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include <string>

#include "pico/util/queue.h"
#include "write_wav.h"

extern "C" {
void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err);
}

class write_wav;

class spdif_rec_wav
{
public:
    // === Public definitions of class ===
    enum class error_type_t {
        SPDIF_QUEUE_FULL = 0,
        ILLEGAL_SUB_FRAME_COUNT,
        FILE_CMD_QUEUE_FULL,
        FILE_CMD_REPLY_QUEUE_FULL,
        RECORD_CMD_QUEUE_FULL,
        WAV_OPEN_FAIL,
        WAV_DATA_WRITE_FAIL,
        WAV_DATA_SYNC_FAIL,
        WAV_CLOSE_FAIL,
        SUFFIX_FILE_FAIL
    };

    // === Public class constants ===

    // === Public class functions ===
    // functions called from core0
    static void set_wait_grant_func(void (*func)());
    static void process_file_cmd();
    static bool is_standby();
    static bool is_recording();
    static void set_blank_split(const bool flag);
    static bool get_blank_split();
    static void set_verbose(const bool flag);
    static bool get_verbose();
    static int get_suffix();
    static void clear_suffix();
    static void _blocking_wait_core0_grant();
    static void _drain_core0_grant();
    // functions called from core1
    static void record_process_loop(const char* log_prefix = "log_", const char* suffix_info_filename = "last_suffix.txt");
    // functions called from core0 and core1
    static void start_recording(const bits_per_sample_t bits_per_sample, const bool standby = false);
    static void end_recording(const bool immediate_split = false, const float truncate_sec = 0.0f);
    static void split_recording(const bits_per_sample_t bits_per_sample);
    static void _report_error(const error_type_t type, const uint32_t param = 0L);
    static void _log_printf(const char* fmt, ...);

protected:
    // === Private definitions of class ===
    enum class wav_status_t {
        RESET = 0,
        REQ_PREPARE,
        PREPARED,
        REQ_FINALIZE,
        FINALIZED
    };
    enum class file_cmd_type_t {
        PREPARE = 0,
        FINALIZE
    };
    typedef struct _file_cmd_data_t {
        file_cmd_type_t cmd;
        uint32_t   param[4];
    } file_cmd_data_t;
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
    static constexpr int NUM_CHANNELS = write_wav::NUM_CHANNELS;
    static constexpr int NUM_SUB_FRAME_BUF = write_wav::NUM_SUB_FRAME_BUF;
    static constexpr int SPDIF_QUEUE_LENGTH = NUM_SUB_FRAME_BUF - 1;
    static constexpr int FILE_CMD_QUEUE_LENGTH = 2;
    static constexpr int RECORD_CMD_QUEUE_LENGTH = 2;
    static constexpr int ERROR_QUEUE_LENGTH = 10;
    static constexpr int CORE0_GRANT_QUEUE_LENGTH = 1;
    static constexpr int BLANK_LEVEL = 16;  // level to detect blank supposing 16bit data
    static constexpr float BLANK_SEC = 0.5;  // the seconds to detect the blank
    static constexpr float BLANK_REPEAT_PROHIBIT_SEC = 10.0;  // the seconds within which detecting blank is prohibited
    static constexpr float BLANK_SKIP_SEC = 10.0;  // skip recording if blank time is longer than this seconds
    static constexpr const char* WAV_PREFIX = "record_";

    // === Private class functions ===
    // functions called from core1
    static void _process_file_reply_cmd();
    static void _process_error();
    static void _req_prepare_file(spdif_rec_wav& inst_w_sts, const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample);
    static void _req_finalize_file(spdif_rec_wav& inst_w_sts, const bool report_final, const float truncate_sec = 0.0f);
    static void _send_core0_grant();
    static void _push_sub_frame_buf(const uint32_t* buff, const uint32_t sub_frame_count);
    static int _get_last_suffix();
    static void _set_last_suffix(int suffix);
    static blank_status_t _scan_blank(const uint32_t* buff, const uint32_t sub_frame_count, const uint32_t sample_freq);

    // === Private class variables ===
    static void (*_wait_grant_func)();
    static const char* _suffix_info_filename;
    static int _suffix;
    static char _log_filename[16];
    static bool _clear_log;
    static uint32_t _sub_frame_buf[SPDIF_BLOCK_SIZE * NUM_SUB_FRAME_BUF];
    static int _sub_frame_buf_id;
    static float _blank_sec;
    static float _blank_scan_sec;
    static bool _standby_flag;
    static bool _recording_flag;
    static bool _blank_split;
    static bool _verbose;
    static queue_t _spdif_queue;
    static queue_t _file_cmd_queue;
    static queue_t _file_cmd_reply_queue;
    static queue_t _record_cmd_queue;
    static queue_t _error_queue;
    static queue_t _core0_grant_queue;

    // === Constructor and Destructor (Private use) ===
    // called from core0
    spdif_rec_wav(wav_status_t status = wav_status_t::RESET);
    virtual ~spdif_rec_wav();

    // === Private member functions ===
    // functions called from core0
    void reset();
    /*
    void prepare(const std::string filename, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample);
    void finalize(bool report_flag, float truncate_sec = 0.0f);
    */
    /*
    void prepare(file_cmd_data_t& cmd_data);
    void finalize(file_cmd_data_t& cmd_data);
    */
    void _process_file_cmd(file_cmd_data_t& cmd_data);

    // === Private member variables ===
    write_wav* inst;
    volatile wav_status_t status;

    friend void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err);
};

