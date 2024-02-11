/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include <string>

#include "spdif_rx.h"
#include "fatfs/ff.h"

// === Global definitions ===
enum class bits_per_sample_t {
    _16BITS = 16,
    _24BITS = 24,
};

class write_wav
{
public:
    // === Public class constants ===
    static constexpr int NUM_CHANNELS = 2;
    static constexpr int NUM_SUB_FRAME_BUF = 96; // maximize buffers to the limit for the margin of writing latency as much as possible
    static constexpr const char* WAV_PREFIX = "record_";

    // === Public class functions ===
    // functions called from core0

    // === Constructor and Destructor ===
    // called from core0
    write_wav(const std::string filename, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample);
    virtual ~write_wav();

    // === Public member functions ===
    // functions called from core1
    uint32_t _write(const uint32_t* buff, const uint32_t sub_frame_count);
    void _set_truncate(const float sec);
    void _record_queue_ratio(float queue_ratio);
    void _report_start();
    void _report_final();
    bool _is_data_written();

protected:
    // === Private class constants ===
    static constexpr int WAV_HEADER_SIZE = 44;
    static constexpr uint32_t MAX_TOTAL_BYTES = 0xfff00000;  // max total bytes of wav data to avoid 32bit overflow
    static constexpr uint32_t SEEK_STEP_BYTES = 10 * 1024 * 1024;  // 10MB

    // === Private class functions ===

    // === Private class variables ===
    static uint32_t _wav_buf[SPDIF_BLOCK_SIZE*3/4 * NUM_SUB_FRAME_BUF / 2];

    // === Private member functions ===
    // functions called from core0
    FRESULT _stepwise_seek(DWORD target_pos);
    // functions called from core1
    uint32_t _write_core(const uint32_t* buff, const uint32_t sub_frame_count);

    // === Private member variables ===
    FIL                     _fil;
    const std::string       _filename;
    const uint32_t          _sample_freq;
    const bits_per_sample_t _bits_per_sample;
    uint32_t                _total_bytes;
    uint32_t                _total_time_us;
    float                   _best_bandwidth;
    float                   _worst_bandwidth;
    float                   _worst_queue_ratio;
    bool                    _data_written;
    float                   _truncate_sec;
};

