/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include <string>

#include "pico/util/queue.h"
#include "spdif_rx.h"
#include "fatfs/ff.h"

extern "C" {
void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err);
}

class spdif_rec_wav
{
public:
    enum class bits_per_sample_t {
        _16BITS = 16,
        _24BITS = 24,
    };

    // process on core0
    static void file_cmd_process();
    // process on core1
    static void file_reply_cmd_process();
    static void record_process_loop(const char* log_prefix = "log_", const char* suffix_info_filename = "last_suffix.txt");
    static void start_recording(const bits_per_sample_t bits_per_sample, const bool standby = false);
    static void end_recording(const bool split = false);
    static void split_recording(const bits_per_sample_t bits_per_sample);
    static bool is_standby();
    static bool is_recording();
    static void set_blank_split(const bool flag);
    static bool get_blank_split();
    static void set_verbose(const bool flag);
    static bool get_verbose();
    static int get_suffix();
    static void clear_suffix();

protected:
    enum class call_status_t {
        RESET = 0,
        CALLED,
        DONE
    };
    enum class file_cmd_type_t {
        PREPARE = 0,
        FINALIZE
    };
    typedef struct _file_cmd_data_t {
        file_cmd_type_t cmd;
        uint32_t   param[3];
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
    typedef struct _sub_frame_buf_info_t {
        int      buf_id;
        uint32_t sub_frame_count;
    } sub_frame_buf_info_t;

    static constexpr int NUM_CHANNELS = 2;
    static constexpr int NUM_SUB_FRAME_BUF = 96; // maximize buffers to the limit for the margin of writing latency as much as possible
    static constexpr int SPDIF_QUEUE_LENGTH = NUM_SUB_FRAME_BUF - 1;
    static constexpr int FILE_CMD_QUEUE_LENGTH = 2;
    static constexpr int RECORD_CMD_QUEUE_LENGTH = 2;
    static constexpr int WAV_HEADER_SIZE = 44;
    static constexpr int BLANK_LEVEL = 16;  // level to detect blank supposing 16bit data
    static constexpr float BLANK_SEC = 0.5;  // the seconds to detect the blank
    static constexpr float BLANK_REPEAT_PROHIBIT_SEC = 10.0;  // the seconds within which detecting blank is prohibited
    static constexpr float BLANK_SKIP_SEC = 10.0;  // skip recording if blank time is longer than this seconds
    static constexpr const char* WAV_PREFIX = "record_";
    static const char* _suffix_info_filename;
    static int _suffix;
    static char _log_filename[16];
    static bool _clear_log;
    static uint32_t _sub_frame_buf[SPDIF_BLOCK_SIZE * NUM_SUB_FRAME_BUF];
    static int _sub_frame_buf_id;
    static float _blank_sec;
    static float _blank_scan_sec;
    static uint32_t _wav_buf[SPDIF_BLOCK_SIZE*3/4 * NUM_SUB_FRAME_BUF / 2];
    static bool _standby_flag;
    static bool _recording_flag;
    static bool _blank_split;
    static bool _verbose;
    static queue_t _spdif_queue;
    static queue_t _file_cmd_queue;
    static queue_t _file_cmd_reply_queue;
    static queue_t _record_cmd_queue;
    static spdif_rec_wav* _inst_prev;
    static spdif_rec_wav* _inst;
    static spdif_rec_wav* _inst_next;
    static call_status_t _status_prev;
    static call_status_t _status_next;

    spdif_rec_wav(const std::string filename, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample);
    virtual ~spdif_rec_wav();

    FIL                     _fil;
    const std::string       _filename;
    const uint32_t          _sample_freq;
    const bits_per_sample_t _bits_per_sample;
    uint32_t                _total_sample_count;
    uint32_t                _total_bytes;
    uint32_t                _total_time_us;
    float                   _best_bandwidth;
    float                   _worst_bandwidth;
    uint                    _queue_worst;

    static void _call_prepare_file(const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample);
    static void _call_finalize_file(const spdif_rec_wav* inst, const bool is_prev = true);
    static void _push_sub_frame_buf(const uint32_t* buff, const uint32_t sub_frame_count);
    static void _log_printf(const char* fmt, ...);
    static int _get_last_suffix();
    static void _set_last_suffix(int suffix);
    static blank_status_t _scan_blank(const uint32_t* buff, const uint32_t sub_frame_count, const uint32_t sample_freq);
    void _reset_blank_scan_sec();
    uint32_t _write_core(const uint32_t* buff, const uint32_t sub_frame_count);
    uint32_t _write(const uint32_t* buff, const uint32_t sub_frame_count);
    void _record_queue_level(uint queue_level);
    void _report_start();
    void _report_final();

    friend void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err);
};

