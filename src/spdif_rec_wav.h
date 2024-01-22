/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

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

    static void process_loop(const char* wav_prefix = "record_", const char* log_prefix = "log_", const char* suffix_info_filename = "last_suffix.txt");
    static void start_recording(const bits_per_sample_t bits_per_sample, const bool standby = false);
    static void end_recording(const bool split = false);
    static void split_recording(const bits_per_sample_t bits_per_sample);
    static bool is_recording();
    static void set_blank_split(const bool flag);
    static bool get_blank_split();
    static void set_verbose(const bool flag);
    static bool get_verbose();
    static int get_suffix();
    static void clear_suffix();

    spdif_rec_wav(const char* filename, const uint32_t sample_rate, const bits_per_sample_t bits_per_sample);
    virtual ~spdif_rec_wav();

private:
    enum class blank_status_t {
        NOT_BLANK = 0,
        BLANK_DETECTED,
        BLANK_SKIP,
        BLANK_END_DETECTED
    };
    enum class cmd_type_t {
        STANDBY_START_CMD = 0,
        START_CMD,
        END_CMD,
        END_FOR_SPLIT_CMD
    };
    typedef struct _cmd_data_t {
        cmd_type_t cmd;
        uint32_t   param1;
        uint32_t   param2;
    } cmd_data_t;
    typedef struct _sub_frame_buf_info_t {
        int      buf_id;
        uint32_t sub_frame_count;
    } sub_frame_buf_info_t;

    static constexpr int NUM_CHANNELS = 2;
    static constexpr int NUM_SUB_FRAME_BUF = 96; // maximize buffers to the limit for the margin of writing latency as much as possible
    static constexpr int SPDIF_QUEUE_LENGTH = NUM_SUB_FRAME_BUF - 1;
    static constexpr int CMD_QUEUE_LENGTH = 2;
    static constexpr int WAV_HEADER_SIZE = 44;
    static constexpr int BLANK_LEVEL = 16;  // level to detect blank supposing 16bit data
    static constexpr float BLANK_SEC = 0.5;  // the seconds to detect the blank
    static constexpr float BLANK_REPEAT_PROHIBIT_SEC = 10.0;  // the seconds within which detecting blank is prohibited
    static constexpr float BLANK_SKIP_SEC = 10.0;  // skip recording if blank time is longer than this seconds
    static const char* _suffix_info_filename;
    static int _suffix;
    static char _log_filename[16];
    static bool _clear_log;
    static uint32_t _sub_frame_buf[SPDIF_BLOCK_SIZE * NUM_SUB_FRAME_BUF];
    static int _sub_frame_buf_id;
    static uint32_t _wav_buf[SPDIF_BLOCK_SIZE*3/4 * NUM_SUB_FRAME_BUF / 2];
    static bool _standby_flag;
    static bool _recording_flag;
    static bool _blank_split;
    static bool _verbose;
    static queue_t _spdif_queue;
    static queue_t _cmd_queue;

    FIL _fil;
    const uint32_t          _sample_freq;
    const bits_per_sample_t _bits_per_sample;
    float                   _blank_sec;
    float                   _total_sec;

    static void log_printf(const char* fmt, ...);
    static int get_last_suffix();
    static void set_last_suffix(int suffix);
    static void push_sub_frame_buf(const uint32_t* buff, const uint32_t sub_frame_count);
    blank_status_t get_blank_status(const uint32_t* buff, const uint32_t sub_frame_count);
    uint32_t write(const uint32_t* buff, const uint32_t sub_frame_count);
    FRESULT finalize(const uint32_t num_samp);

    friend void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err);
};

