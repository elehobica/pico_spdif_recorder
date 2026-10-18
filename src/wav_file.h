/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include <string>

#include "ff.h"
#include "spdif_rec_wav.h"
#include "wav_header.h"

class wav_file
{
public:
    // === Public class constants ===

    // === Public class functions ===
    // functions called from core0
#ifdef TEST_BUILD
    static void test_paint_stacks();
#endif

    // === Constructor and Destructor ===
    // called from core0
    wav_file(const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample);
    virtual ~wav_file();

    // === Public member functions ===
    // functions called from core0
    void sync();
    // functions called from core1
    uint32_t write(const uint32_t* buff, const uint32_t sub_frame_count);
    bool is_sync_required();
    void set_truncate(const float sec);
    void record_queue_ratio(float queue_ratio);
    void report_start() const;
    void report_final() const;
    bool is_data_written() const;

protected:
    // === Private class constants ===
    static constexpr int NUM_CHANNELS = spdif_rec_wav::NUM_CHANNELS;
    static constexpr int NUM_SUB_FRAME_BUF = spdif_rec_wav::NUM_SUB_FRAME_BUF;
    static constexpr const char* WAV_PREFIX = "record_";
    static constexpr int WAV_HEADER_SIZE = wav_header::WAV_HEADER_SIZE;
    static constexpr uint32_t MAX_TOTAL_BYTES = 0xfff00000;  // max total bytes of wav data to avoid 32bit overflow
    static constexpr int64_t SEEK_STEP_BYTES = 10 * 1024 * 1024;  // 10MB
#ifndef PERIODIC_SYNC_MBYTES
#define PERIODIC_SYNC_MBYTES 64
#endif
    // periodic f_sync interval to write FAT chain of exFAT gradually instead of at once in finalize (0: disabled)
    static_assert(PERIODIC_SYNC_MBYTES < 4096, "PERIODIC_SYNC_MBYTES must be less than 4096");
    static constexpr uint32_t PERIODIC_SYNC_BYTES = PERIODIC_SYNC_MBYTES * 1024u * 1024u;
#ifdef TEST_BUILD
#ifndef TEST_PRESET_BYTES
#define TEST_PRESET_BYTES 0
#endif
    // test only: preset wav data size by allocating clusters without writing actual data (0: disabled)
    static constexpr uint64_t TEST_PRESET_TOTAL_BYTES = TEST_PRESET_BYTES;
    static constexpr const char* TEST_BLOCKER_FILENAME = "test_blocker.bin";
#endif

    // === Private class functions ===

    // === Private class variables ===
    static uint32_t _wav_buf[SPDIF_BLOCK_SIZE*3/4 * NUM_SUB_FRAME_BUF / 2];

    // === Private member functions ===
    // functions called from core0
    FRESULT _stepwise_seek(const FSIZE_t pos);
    // functions called from core0 and core1
    uint64_t _get_truncate_bytes() const;
#ifdef TEST_BUILD
    FRESULT _test_preset(const uint64_t preset_bytes);
#endif
    // functions called from core1
    uint32_t _write_core(const uint32_t* buff, const uint32_t sub_frame_count);

    // === Private member variables ===
    FIL                     _fil;
    std::string             _filename;
    const uint32_t          _sample_freq;
    const bits_per_sample_t _bits_per_sample;
    bool                    _rf64_requested;  // CFG_SUPPORT_RF64_WAV at file creation
    bool                    _rf64_reserved;   // header reserved for RF64 (only when requested and exFAT)
    int                     _header_size;
    uint64_t                _total_bytes;
    uint64_t                _total_time_us;
    float                   _best_bandwidth;
    float                   _worst_bandwidth;
    float                   _worst_queue_ratio;
    bool                    _data_written;
    float                   _truncate_sec;
    uint64_t                _next_sync_bytes;
    volatile bool           _sync_pending;
#ifdef TEST_BUILD
    static constexpr int    TEST_NUM_SYNC_HIST = 6;
    uint32_t                _test_sync_count;
    uint32_t                _test_sync_max_us;
    uint32_t                _test_sync_hist[TEST_NUM_SYNC_HIST];  // < 5, 5-10, 10-20, 20-50, 50-100, >= 100 ms
    static constexpr uint32_t TEST_QUEUE_START_US = 1000000;  // period regarded as start of recording
    static constexpr uint32_t TEST_QUEUE_SYNC_US = 200000;  // period regarded as affected by sync after its completion
    bool                    _test_queue_started;
    uint32_t                _test_queue_start_us;
    float                   _test_queue_ratio_start;
    float                   _test_queue_ratio_after;
    float                   _test_queue_ratio_sync;
    volatile bool           _test_sync_done;
    volatile uint32_t       _test_sync_done_us;
#endif
};

