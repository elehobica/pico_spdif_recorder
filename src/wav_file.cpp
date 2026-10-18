/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include "wav_file.h"

#include <string>
#include <cstring>
#include <cmath>

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
    wav_file_status::blocking_wait_core0_grant();
}

static inline void _drain_core0_grant()
{
    wav_file_status::drain_core0_grant();
}

/*----------------------------------------/
/  FATFS wrappers for waiting core0 grant
/----------------------------------------*/
static inline FRESULT _wrap_f_open(FIL* fp, const TCHAR* path, BYTE mode)
{
    _blocking_wait_core0_grant();
    return f_open(fp, path, mode);
}

static inline FRESULT _wrap_f_close(FIL* fp)
{
    _blocking_wait_core0_grant();
    return f_close(fp);
}

static inline FRESULT _wrap_f_read(FIL* fp, void* buff, UINT btr, UINT* br)
{
    _blocking_wait_core0_grant();
    return f_read(fp, buff, btr, br);
}

static inline FRESULT _wrap_f_write(FIL* fp, const void* buff, UINT btw, UINT* bw)
{
    _blocking_wait_core0_grant();
    return f_write(fp, buff, btw, bw);
}

static inline FRESULT _wrap_f_write_priority(FIL* fp, const void* buff, UINT btw, UINT* bw)
{
    // no blocking wait for core0 grant
    return f_write(fp, buff, btw, bw);
}

static inline FRESULT _wrap_f_lseek(FIL* fp, FSIZE_t ofs)
{
    _blocking_wait_core0_grant();
    return f_lseek(fp, ofs);
}

static inline FRESULT _wrap_f_truncate(FIL* fp)
{
    _blocking_wait_core0_grant();
    return f_truncate(fp);
}

static inline FRESULT _wrap_f_sync(FIL* fp)
{
    _blocking_wait_core0_grant();
    return f_sync(fp);
}

static inline FRESULT _wrap_f_sync_priority(FIL* fp)
{
    // no blocking wait for core0 grant
    return f_sync(fp);
}

static inline FRESULT _wrap_f_unlink(const TCHAR* path)
{
    _blocking_wait_core0_grant();
    return f_unlink(path);
}

static inline FSIZE_t _wrap_f_tell(FIL* fp)
{
    _blocking_wait_core0_grant();
    return f_tell(fp);
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
    _rf64_requested(spdif_rec_wav::get_support_rf64_wav()),
    _rf64_reserved(false),
    _header_size(WAV_HEADER_SIZE),
    _total_bytes(0),
    _total_time_us(0),
    _best_bandwidth(-INFINITY),
    _worst_bandwidth(INFINITY),
    _worst_queue_ratio(0.0f),
    _data_written(false),
    _truncate_sec(0.0f),
    _next_sync_bytes(PERIODIC_SYNC_BYTES),
    _sync_pending(false)
{
    char wav_filename[16];
    sprintf(wav_filename, "%s%03d.wav", WAV_PREFIX, suffix);
    _filename = std::string(wav_filename);

    _drain_core0_grant();
    for ( ; ; ) {
        FRESULT fr;     /* FatFs return code */
        UINT bw;
        uint8_t buf[wav_header::MAX_HEADER_SIZE];

        fr = _wrap_f_open(&_fil, _filename.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
        if (fr != FR_OK) break;

        // RF64 is available only on exFAT because FAT32 limits file size to 4 GiB - 1
        _rf64_reserved = _rf64_requested && _fil.obj.fs->fs_type == FS_EXFAT;
        _header_size = wav_header::generate(buf, _sample_freq, static_cast<uint16_t>(_bits_per_sample), NUM_CHANNELS, _rf64_reserved, 0);

        fr = _wrap_f_write(&_fil, buf, _header_size, &bw);
        if (fr != FR_OK || bw != _header_size) break;
        fr = _wrap_f_sync(&_fil);
        if (fr != FR_OK) break;

        return;
    }

    spdif_rec_wav::report_error(spdif_rec_wav::error_type_t::WAV_OPEN_FAIL);
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
            fr = _wrap_f_close(&_fil);
            if (fr != FR_OK) break;

            fr = _wrap_f_unlink(_filename.c_str());
            if (fr != FR_OK) break;
        } else if (_rf64_reserved) {
            UINT bw;
            uint8_t buf[wav_header::MAX_HEADER_SIZE];

            // note: truncate (trailing blank at blank skip or stop) takes time proportional to the file size
            // because FATFS follows the whole cluster chain from the head twice, by backward seek and by f_truncate
            // to check contiguity on exFAT (measured: about 0.84 sec in total for 20 GB file).
            // recording doesn't break meanwhile because core1 waits for this finalize without recording,
            // but restart of recording when sound is detected is delayed until it completes (see spdif_rec_wav)
            if (_truncate_sec > 0.0f) {
                uint64_t truncate_bytes = _get_truncate_bytes();
                FSIZE_t cur_pos = _wrap_f_tell(&_fil) - truncate_bytes;
                _total_bytes -= truncate_bytes;
                fr = _stepwise_seek(cur_pos);
                if (fr != FR_OK) break;
                fr = _wrap_f_truncate(&_fil);
                if (fr != FR_OK) break;
            }

            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK) break;

            // rewrite whole header at once as RIFF or RF64 depending on the size
            // no need to seek back to the end because FAT chain is already filled by f_sync above (and f_lseek)
            // and f_close doesn't depend on the file position
            fr = _stepwise_seek(0);
            if (fr != FR_OK) break;
            int header_size = wav_header::generate(buf, _sample_freq, static_cast<uint16_t>(_bits_per_sample), NUM_CHANNELS, true, _total_bytes);
            fr = _wrap_f_write(&_fil, buf, header_size, &bw);
            if (fr != FR_OK || bw != header_size) break;

            fr = _wrap_f_close(&_fil);
            if (fr != FR_OK) break;
        } else {
            UINT bw;
            uint32_t u32;
            FSIZE_t cur_pos = _wrap_f_tell(&_fil);

            if (_truncate_sec > 0.0f) {
                uint64_t truncate_bytes = _get_truncate_bytes();
                cur_pos -= truncate_bytes;
                _total_bytes -= truncate_bytes;
                fr = _stepwise_seek(cur_pos);
                if (fr != FR_OK) break;
                fr = _wrap_f_truncate(&_fil);
                if (fr != FR_OK) break;
            }

            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK) break;

            // ChunkSize
            fr = _stepwise_seek(4);
            if (fr != FR_OK) break;
            u32 = static_cast<uint32_t>(_total_bytes + (WAV_HEADER_SIZE - 8));
            fr = _wrap_f_write(&_fil, static_cast<const void *>(&u32), sizeof(uint32_t), &bw);
            if (fr != FR_OK || bw != sizeof(uint32_t)) break;
            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK) break;

            // Subchunk2Size
            fr = _stepwise_seek(40);
            if (fr != FR_OK) break;
            u32 = static_cast<uint32_t>(_total_bytes);
            fr = _wrap_f_write(&_fil, static_cast<const void *>(&u32), sizeof(uint32_t), &bw);
            if (fr != FR_OK || bw != sizeof(uint32_t)) break;
            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK) break;

            fr = _stepwise_seek(cur_pos);
            if (fr != FR_OK) break;
            fr = _wrap_f_close(&_fil);
            if (fr != FR_OK) break;
        }

        return;
    }

    spdif_rec_wav::report_error(spdif_rec_wav::error_type_t::WAV_CLOSE_FAIL);
}

/*--------------------------/
/  Public Member functions
/--------------------------*/
void wav_file::sync()
{
    FRESULT fr = _wrap_f_sync(&_fil);
    if (fr != FR_OK) {
        spdif_rec_wav::report_error(spdif_rec_wav::error_type_t::WAV_DATA_SYNC_FAIL);
    }
    _sync_pending = false;
}

bool wav_file::is_sync_required()
{
    if (PERIODIC_SYNC_BYTES == 0 || _sync_pending || _total_bytes < _next_sync_bytes) return false;
    _next_sync_bytes = _total_bytes + PERIODIC_SYNC_BYTES;
    _sync_pending = true;
    return true;
}

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

    // force immediate split to avoid 32bit file size overflow unless RF64
    if (!_rf64_reserved && _total_bytes > MAX_TOTAL_BYTES) {
        spdif_rec_wav::split_recording(_bits_per_sample);
        spdif_rec_wav::log_printf("force immediate wav split due to file size\r\n");
        if (_rf64_requested) {
            spdif_rec_wav::log_printf("WARNING: forced split at 4 GB despite CFG_SUPPORT_RF64_WAV because of FAT32\r\n");
        }
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

void wav_file::report_start() const
{
    printf("start \"%s\" @ %d bits %5.1f KHz (bitrate: %6.1f Kbps)\r\n", _filename.c_str(), _bits_per_sample, static_cast<float>(_sample_freq)*1e-3, static_cast<float>(_bits_per_sample)*_sample_freq*2*1e-3);
}

void wav_file::report_final() const
{
    double total_sec_f = static_cast<double>(_total_bytes) / (static_cast<uint32_t>(_bits_per_sample)/8) / NUM_CHANNELS / _sample_freq - _truncate_sec;
    uint32_t total_sec = static_cast<uint32_t>(total_sec_f);
    uint32_t total_sec_dp = static_cast<uint32_t>((total_sec_f - total_sec) * 1e3);
    const char* rf64_note = wav_header::is_rf64(_rf64_reserved, _total_bytes - _get_truncate_bytes()) ? " (RF64)" : "";
    spdif_rec_wav::log_printf("recorded \"%s\"%s @ %d bits %5.1f KHz (%6.1f Kbps) total %llu bytes (time:  %d:%02d.%03d)\r\n", _filename.c_str(), rf64_note, _bits_per_sample, static_cast<float>(_sample_freq)*1e-3, static_cast<float>(_bits_per_sample)*_sample_freq*2*1e-3, _total_bytes + _header_size, total_sec/60, total_sec%60, total_sec_dp);
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

bool wav_file::is_data_written() const
{
    return _data_written;
}

/*--------------------------/
/  Protected class functions
/--------------------------*/

/*-----------------------------/
/  Protected Member functions
/-----------------------------*/
uint64_t wav_file::_get_truncate_bytes() const
{
    if (_truncate_sec <= 0.0f) return 0;
    const uint32_t block_align = (static_cast<uint32_t>(_bits_per_sample)/8) * NUM_CHANNELS;
    uint64_t truncate_bytes = static_cast<uint64_t>(_truncate_sec * block_align * _sample_freq);
    truncate_bytes -= truncate_bytes % block_align;  // align to sample frame not to leave partial frame at the end of data
    return (truncate_bytes < _total_bytes) ? truncate_bytes : _total_bytes;  // _total_bytes is always aligned
}

FRESULT wav_file::_stepwise_seek(const FSIZE_t pos)
{
    FRESULT fr = FR_OK;     /* FatFs return code */

    int64_t target_pos = static_cast<int64_t>(pos);
    int64_t cur_pos    = static_cast<int64_t>(_wrap_f_tell(&_fil));
    int64_t diff       = target_pos - cur_pos;

    // backward seek: FATFS always follows the cluster chain from the head of the file,
    // therefore stepwise seek doesn't reduce the time for each step, but multiplies the total time
    if (diff < 0) {
        return _wrap_f_lseek(&_fil, static_cast<FSIZE_t>(target_pos));
    }

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
        fr = _wrap_f_lseek(&_fil, static_cast<FSIZE_t>(cur_pos));
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
        fr = _wrap_f_write_priority(&_fil, static_cast<const void *>(_wav_buf), sub_frame_count*2, &bw);
        if (fr != FR_OK || bw != sub_frame_count*2) {
            spdif_rec_wav::report_error(spdif_rec_wav::error_type_t::WAV_DATA_WRITE_FAIL);
        }
    } else if (_bits_per_sample == bits_per_sample_t::_24BITS) {
        for (int i = 0, j = 0; i < sub_frame_count; i += 4, j += 3) {
            _wav_buf[j+0] = (((buff[i+1] >> 4) & 0x0000ff) << 24) | (((buff[i+0] >> 4) & 0xffffff) >>  0);
            _wav_buf[j+1] = (((buff[i+2] >> 4) & 0x00ffff) << 16) | (((buff[i+1] >> 4) & 0xffff00) >>  8);
            _wav_buf[j+2] = (((buff[i+3] >> 4) & 0xffffff) <<  8) | (((buff[i+2] >> 4) & 0xff0000) >> 16);
        }
        fr = _wrap_f_write_priority(&_fil, static_cast<const void *>(_wav_buf), sub_frame_count*3, &bw);
        if (fr != FR_OK || bw != sub_frame_count*3) {
            spdif_rec_wav::report_error(spdif_rec_wav::error_type_t::WAV_DATA_WRITE_FAIL);
        }
    }

    // comment below to let FATFS to do f_sync() timing for better performance
    /*
    fr = _wrap_f_sync_priority(&_fil);
    if (fr != FR_OK) {
        spdif_rec_wav::report_error(spdif_rec_wav::error_type_t::WAV_DATA_SYNC_FAIL);
    }
    */

    return static_cast<uint32_t>(bw);
}
