/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include "wav_file.h"

#include <string>
#include <cstring>
#include <cmath>

#include "spdif_rec_wav.h"
#include "wav_file_status.h"

/*-----------------/
/  Local function
/-----------------*/
static inline uint64_t _micros()
{
    return to_us_since_boot(get_absolute_time());
}

static inline void _blocking_wait_core0_grant()
{
    wav_file_status::_blocking_wait_core0_grant();
}

static inline void _drain_core0_grant()
{
    wav_file_status::_drain_core0_grant();
}

/*-----------------/
/  Class variables
/-----------------*/
uint32_t wav_file::_wav_buf[SPDIF_BLOCK_SIZE*3/4 * NUM_SUB_FRAME_BUF / 2];

/*------------------------/
/  Public class functions
/------------------------*/

/*-----------------/
/  Constructor
/-----------------*/
wav_file::wav_file(const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample) :
    _fil(),
    _filename(),
    _sample_freq(sample_freq),
    _bits_per_sample(bits_per_sample),
    _total_bytes(0),
    _total_time_us(0),
    _best_bandwidth(-INFINITY),
    _worst_bandwidth(INFINITY),
    _worst_queue_ratio(0.0f),
    _data_written(false),
    _truncate_sec(0.0f)
{
    char wav_filename[16];
    sprintf(wav_filename, "%s%03d.wav", WAV_PREFIX, suffix);
    _filename = std::string(wav_filename);

    for ( ; ; ) {
        FRESULT fr;     /* FatFs return code */
        UINT bw;
        uint8_t buf[WAV_HEADER_SIZE];
        uint16_t u16;
        uint32_t u32;

        _blocking_wait_core0_grant();
        fr = f_open(&_fil, _filename.c_str(), FA_WRITE | FA_CREATE_ALWAYS);

        _blocking_wait_core0_grant();
        // ChunkID
        memcpy(&buf[0], "RIFF", 4);
        // ChunkSize (temporary 0)
        memset(&buf[4], 0, 4);
        // Format
        memcpy(&buf[8], "WAVE", 4);

        // Subchunk1ID
        memcpy(&buf[12], "fmt ", 4);
        // Subchunk1Size
        u32 = 16;  // 16 for PCM
        memcpy(&buf[16], (const void *) &u32, 4);
        // AudioFormat
        u16 = 1;  // PCM = 1
        memcpy(&buf[20], (const void *) &u16, 2);
        // NumChannels
        u16 = 2;  // e.g. Stereo = 2
        memcpy(&buf[22], (const void *) &u16, 2);
        // SampleRate
        u32 = _sample_freq;  // e.g. 44100
        memcpy(&buf[24], (const void *) &u32, 4);
        // ByteRate
        u32 = _sample_freq * NUM_CHANNELS * (static_cast<uint16_t>(_bits_per_sample)/8);
        memcpy(&buf[28], (const void *) &u32, 4);
        // BlockAlign
        u16 = NUM_CHANNELS * (static_cast<uint16_t>(_bits_per_sample)/8);
        memcpy(&buf[32], (const void *) &u16, 2);
        // BitsPerSample
        u16 = static_cast<uint16_t>(_bits_per_sample);
        memcpy(&buf[34], (const void *) &u16, 2);

        // Subchunk2ID
        memcpy(&buf[36], "data", 4);
        // Subchunk2Size (temporary 0)
        memset(&buf[40], 0, 4);

        _blocking_wait_core0_grant();
        fr = f_write(&_fil, buf, sizeof(buf), &bw);
        if (fr != FR_OK || bw != sizeof(buf)) break;
        _blocking_wait_core0_grant();
        fr = f_sync(&_fil);
        if (fr != FR_OK) break;

        _drain_core0_grant();
        return;
    }

    spdif_rec_wav::_report_error(spdif_rec_wav::error_type_t::WAV_OPEN_FAIL);
}

/*-----------------/
/  Destructor
/-----------------*/
wav_file::~wav_file()
{
    _drain_core0_grant();
    for ( ; ; ) {
        FRESULT fr;     /* FatFs return code */

        if (_total_bytes == 0) {
            // remove file if no samples
            _blocking_wait_core0_grant();
            fr = f_close(&_fil);
            if (fr != FR_OK) break;

            _blocking_wait_core0_grant();
            fr = f_unlink(_filename.c_str());
            if (fr != FR_OK) break;
        } else {
            UINT bw;
            uint32_t u32;
            _blocking_wait_core0_grant();
            DWORD cur_pos = f_tell(&_fil);

            if (_truncate_sec > 0.0f) {
                uint32_t truncate_bytes = static_cast<uint32_t>(_truncate_sec * (static_cast<uint32_t>(_bits_per_sample)/8) * NUM_CHANNELS * _sample_freq);
                cur_pos -= truncate_bytes;
                _total_bytes -= truncate_bytes;
                _blocking_wait_core0_grant();
                fr = _stepwise_seek(cur_pos);
                if (fr != FR_OK) break;
                _blocking_wait_core0_grant();
                fr = f_truncate(&_fil);
                if (fr != FR_OK) break;
            }

            _blocking_wait_core0_grant();
            fr = f_sync(&_fil);
            if (fr != FR_OK) break;

            // ChunkSize
            _blocking_wait_core0_grant();
            fr = _stepwise_seek(4);
            if (fr != FR_OK) break;
            u32 = _total_bytes + (WAV_HEADER_SIZE - 8);
            _blocking_wait_core0_grant();
            fr = f_write(&_fil, static_cast<const void *>(&u32), sizeof(uint32_t), &bw);
            if (fr != FR_OK || bw != sizeof(uint32_t)) break;
            _blocking_wait_core0_grant();
            fr = f_sync(&_fil);
            if (fr != FR_OK) break;

            // Subchunk2Size
            _blocking_wait_core0_grant();
            fr = _stepwise_seek(40);
            if (fr != FR_OK) break;
            u32 = _total_bytes;
            _blocking_wait_core0_grant();
            fr = f_write(&_fil, static_cast<const void *>(&u32), sizeof(uint32_t), &bw);
            if (fr != FR_OK || bw != sizeof(uint32_t)) break;
            _blocking_wait_core0_grant();
            fr = f_sync(&_fil);
            if (fr != FR_OK) break;

            _blocking_wait_core0_grant();
            fr = _stepwise_seek(cur_pos);
            if (fr != FR_OK) break;
            _blocking_wait_core0_grant();
            fr = f_close(&_fil);
            if (fr != FR_OK) break;
        }

        _drain_core0_grant();
        return;
    }

    spdif_rec_wav::_report_error(spdif_rec_wav::error_type_t::WAV_OPEN_FAIL);
}

/*--------------------------/
/  Public Member functions
/--------------------------*/
uint32_t wav_file::write(const uint32_t* buff, const uint32_t sub_frame_count)
{
    uint64_t start_time = _micros();
    uint32_t bytes = _write_core(buff, sub_frame_count);
    uint32_t t_us = static_cast<uint32_t>(_micros() - start_time);
    float bandwidth = static_cast<float>(bytes) / t_us * 1e3;
    if (bandwidth > _best_bandwidth) _best_bandwidth = bandwidth;
    if (bandwidth < _worst_bandwidth) {
        _worst_bandwidth = bandwidth;
        if (spdif_rec_wav::get_verbose()) {
            printf("worst bandwidth updated: %7.2f KB/s\r\n", _worst_bandwidth);
        }
    }
    _total_bytes += bytes;
    _total_time_us += t_us;
    _data_written = true;

    // force immidiate split to avoid 32bit file size overflow
    if (_total_bytes > MAX_TOTAL_BYTES) {
        spdif_rec_wav::split_recording(_bits_per_sample);
        spdif_rec_wav::_log_printf("force immediate wav split due to file size\r\n");
    }

    return bytes;
}

void wav_file::set_truncate(const float sec)
{
    _truncate_sec = sec;
}

void wav_file::record_queue_ratio(float queue_ratio)
{
    if (queue_ratio > _worst_queue_ratio) _worst_queue_ratio = queue_ratio;
}

void wav_file::report_start()
{
    spdif_rec_wav::_log_printf("recording start \"%s\" @ %d bits %5.1f KHz (bitrate: %6.1f Kbps)\r\n", _filename.c_str(), _bits_per_sample, static_cast<float>(_sample_freq)*1e-3, static_cast<float>(_bits_per_sample)*_sample_freq*2*1e-3);
}

void wav_file::report_final()
{
    float total_sec_f = static_cast<float>(_total_bytes) / (static_cast<uint32_t>(_bits_per_sample)/8) / NUM_CHANNELS / _sample_freq - _truncate_sec;
    uint32_t total_sec = static_cast<uint32_t>(total_sec_f);
    uint32_t total_sec_dp = static_cast<uint32_t>((total_sec_f - total_sec) * 1e3);
    /*
    uint32_t total_sec = _total_bytes / (static_cast<uint32_t>(_bits_per_sample)/8) / NUM_CHANNELS / _sample_freq;
    uint32_t total_sec_dp = static_cast<uint64_t>(_total_bytes) / (static_cast<uint32_t>(_bits_per_sample)/8) / NUM_CHANNELS * 1000 / _sample_freq - total_sec*1000;
    */
    spdif_rec_wav::_log_printf("recording done \"%s\" %lu bytes (time:  %d:%02d.%03d)\r\n", _filename.c_str(), _total_bytes + WAV_HEADER_SIZE, total_sec/60, total_sec%60, total_sec_dp);
    if (spdif_rec_wav::get_verbose()) {
        float avg_bw = static_cast<float>(_total_bytes) / _total_time_us * 1e3;
        printf("SD Card writing bandwidth\r\n");
        printf(" avg:   %7.2f KB/s\r\n", avg_bw);
        printf(" best:  %7.2f KB/s\r\n", _best_bandwidth);
        printf(" worst: %7.2f KB/s\r\n", _worst_bandwidth);
        printf("WAV file required bandwidth\r\n");
        printf(" wav:   %7.2f KB/s\r\n", static_cast<float>((static_cast<uint32_t>(_bits_per_sample)*_sample_freq*2/8)) / 1e3);
        printf("spdif queue usage: %7.2f %%\r\n", _worst_queue_ratio * 1e2);
    }
}

bool wav_file::is_data_written()
{
    return _data_written;
}

/*--------------------------/
/  Protected class functions
/--------------------------*/

/*-----------------------------/
/  Protected Member functions
/-----------------------------*/
FRESULT wav_file::_stepwise_seek(DWORD target_pos)
{
    FRESULT fr;     /* FatFs return code */

    DWORD cur_pos = f_tell(&_fil);
    int64_t diff = static_cast<int64_t>(target_pos) - cur_pos;

    while (cur_pos != target_pos) {
        if (diff >= 0) {
            if (cur_pos + SEEK_STEP_BYTES < target_pos) {
                cur_pos += SEEK_STEP_BYTES;
            } else {
                cur_pos = target_pos;
            }
        } else {
            if (cur_pos > target_pos + SEEK_STEP_BYTES) {
                cur_pos -= SEEK_STEP_BYTES;
            } else {
                cur_pos = target_pos;
            }
        }
        _blocking_wait_core0_grant();
        fr = f_lseek(&_fil, cur_pos);
        if (fr != FR_OK) break;
    }
    return fr;
}

uint32_t wav_file::_write_core(const uint32_t* buff, const uint32_t sub_frame_count)
{
    FRESULT fr;     /* FatFs return code */
    UINT bw;

    if (_bits_per_sample == bits_per_sample_t::_16BITS) {
        for (int i = 0; i < sub_frame_count; i++) {
            _wav_buf[i/2] >>= 16;
            _wav_buf[i/2] |= ((buff[i] >> 12) & 0xffff) << 16;
        }
        fr = f_write(&_fil, static_cast<const void *>(_wav_buf), sub_frame_count*2, &bw);
        if (fr != FR_OK || bw != sub_frame_count*2) {
            spdif_rec_wav::_report_error(spdif_rec_wav::error_type_t::WAV_DATA_WRITE_FAIL);
        }
    } else if (_bits_per_sample == bits_per_sample_t::_24BITS) {
        for (int i = 0, j = 0; i < sub_frame_count; i += 4, j += 3) {
            _wav_buf[j+0] = (((buff[i+1] >> 4) & 0x0000ff) << 24) | (((buff[i+0] >> 4) & 0xffffff) >>  0);
            _wav_buf[j+1] = (((buff[i+2] >> 4) & 0x00ffff) << 16) | (((buff[i+1] >> 4) & 0xffff00) >>  8);
            _wav_buf[j+2] = (((buff[i+3] >> 4) & 0xffffff) <<  8) | (((buff[i+2] >> 4) & 0xff0000) >> 16);
        }
        fr = f_write(&_fil, static_cast<const void *>(_wav_buf), sub_frame_count*3, &bw);
        if (fr != FR_OK || bw != sub_frame_count*3) {
            spdif_rec_wav::_report_error(spdif_rec_wav::error_type_t::WAV_DATA_WRITE_FAIL);
        }
    }

    // comment below to let FATFS to do f_sync() timing for better performance
    /*
    fr = f_sync(&_fil);
    if (fr != FR_OK) {
        spdif_rec_wav::_report_error(spdif_rec_wav::error_type_t::WAV_DATA_SYNC_FAIL);
    }
    */

    return static_cast<uint32_t>(bw);
}
