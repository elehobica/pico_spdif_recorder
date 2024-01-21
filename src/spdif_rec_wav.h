/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include "pico/util/queue.h"
#include "spdif_rx.h"
#include "fatfs/ff.h"

typedef enum _record_wav_cmd_t {
    START_CMD = 0,
    END_CMD
} record_wav_cmd_t;
typedef struct _record_wav_cmd_data_t {
    record_wav_cmd_t cmd;
    uint32_t       param1;
    uint32_t       param2;
} record_wav_cmd_data_t;

typedef enum _bits_per_sample_t {
    WAV_16BITS = 16,
    WAV_24BITS = 24
} bits_per_sample_t;

typedef struct _sub_frame_buf_info_t {
    int      buf_id;
    uint32_t sub_frame_count;
} sub_frame_buf_info_t;

extern "C" {
void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err);
}

class spdif_rec_wav
{
public:
    static void process_loop(const char* prefix = "record_");
    static void start_recording(const uint16_t bits_per_sample);
    static void end_recording();
    static bool is_recording();
    static void set_blank_split(const bool flag);
    static bool get_blank_split();
    static void set_verbose(const bool flag);
    static bool get_verbose();

    spdif_rec_wav(const char* filename, const uint32_t sample_rate, const uint16_t bits_per_sample);
    virtual ~spdif_rec_wav();

private:
    enum class blank_status_t {
        NOT_BLANK = 0,
        BLANK_DETECTED,
        BLANK_SKIP,
        BLANK_END_DETECTED
    };

    static constexpr int NUM_CHANNELS = 2;
    static constexpr int NUM_SUB_FRAME_BUF = 96; // maximize buffers to the limit for the margin of writing latency as much as possible
    static constexpr int SPDIF_QUEUE_LENGTH = NUM_SUB_FRAME_BUF - 1;
    static constexpr int RECORD_WAV_CMD_QUEUE_LENGTH = 2;
    static constexpr int WAV_HEADER_SIZE = 44;
    static constexpr int BLANK_LEVEL = 16;  // level to detect blank supposing 16bit data
    static constexpr float BLANK_SEC = 0.5;  // the seconds to detect the blank
    static constexpr float BLANK_REPEAT_PROHIBIT_SEC = 10.0;  // the seconds within which detecting blank is prohibited
    static constexpr float BLANK_SKIP_SEC = 10.0;  // skip recording if blank time is longer than this seconds
    static uint32_t _sub_frame_buf[SPDIF_BLOCK_SIZE * NUM_SUB_FRAME_BUF];
    static int _sub_frame_buf_id;
    static uint32_t _wav_buf[SPDIF_BLOCK_SIZE*3/4 * NUM_SUB_FRAME_BUF / 2];
    static bool _recording_flag;
    static bool _blank_split;
    static bool _verbose;
    static queue_t _spdif_queue;
    static queue_t _record_wav_cmd_queue;

    FIL _fil;
    const uint32_t _sample_freq;
    const uint16_t _bits_per_sample;
    float          _blank_sec;
    float          _total_sec;

    static void push_sub_frame_buf(const uint32_t* buff, const uint32_t sub_frame_count);
    blank_status_t get_blank_status(const uint32_t* buff, const uint32_t sub_frame_count);
    uint32_t write(const uint32_t* buff, const uint32_t sub_frame_count);
    FRESULT finalize(const int num_samp);

    friend void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err);
};

