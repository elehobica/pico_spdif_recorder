/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include "spdif_rec_wav.h"
#include "fatfs/ff.h"

static inline uint64_t _micros(void)
{
    return to_us_since_boot(get_absolute_time());
}

/*---------------------------------------/
/  Global callback function by spdif_rx
/---------------------------------------*/
void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err)
{
    spdif_rec_wav::push_sub_frame_buf(buff, sub_frame_count);
}

/*-----------------/
/  Class variables
/-----------------*/
const char* spdif_rec_wav::_suffix_info_filename;
int         spdif_rec_wav::_suffix;
uint32_t    spdif_rec_wav::_sub_frame_buf[SPDIF_BLOCK_SIZE * NUM_SUB_FRAME_BUF];
int         spdif_rec_wav::_sub_frame_buf_id = 0;
uint32_t    spdif_rec_wav::_wav_buf[SPDIF_BLOCK_SIZE*3/4 * NUM_SUB_FRAME_BUF / 2];
bool        spdif_rec_wav::_recording_flag = false;
bool        spdif_rec_wav:: _blank_split = true;
bool        spdif_rec_wav:: _verbose = false;
queue_t     spdif_rec_wav::_spdif_queue;
queue_t     spdif_rec_wav::_cmd_queue;

/*-----------------/
/  Class functions
/-----------------*/
void spdif_rec_wav::process_loop(const char* wav_prefix, const char* suffix_info_filename)
{
    // Initialize class variables
    _suffix_info_filename = suffix_info_filename;
    _recording_flag = false;

    // Local variables for this loop
    FATFS fs;
    char wav_filename[16];
    uint32_t samp_freq;
    bits_per_sample_t bits_per_sample = spdif_rec_wav::bits_per_sample_t::_16BITS;
    spdif_rec_wav *inst = nullptr;
    uint32_t* next_buf = &_sub_frame_buf[SPDIF_BLOCK_SIZE*0];
    int buf_accum = 1;

    // Mount FATFS
    FRESULT fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        printf("FATFS mount error %d\r\n", fr);
        return;
    }
    printf("FATFS mount ok\r\n");

    _suffix = get_last_suffix() + 1;

    // Initialize queues
    queue_init(&_spdif_queue, sizeof(sub_frame_buf_info_t), SPDIF_QUEUE_LENGTH);
    queue_init(&_cmd_queue, sizeof(cmd_data_t), CMD_QUEUE_LENGTH);

    printf("spdif_rec_wav process started\r\n");

    // Loop
    while (true) {
        if (queue_get_level(&_cmd_queue) > 0) {
            cmd_data_t cmd_data;
            queue_remove_blocking(&_cmd_queue, &cmd_data);
            if (cmd_data.cmd != cmd_type_t::START_CMD) continue;

            samp_freq = cmd_data.param1;
            bits_per_sample = static_cast<bits_per_sample_t>(cmd_data.param2);
            int total_count = 0;
            uint32_t total_bytes = 0;
            uint32_t total_time_us = 0;
            float best_bw = -INFINITY;
            float worst_bw = INFINITY;
            uint queue_worst = 0;
            sprintf(wav_filename, "%s%03d.wav", wav_prefix, _suffix);
            inst = new spdif_rec_wav(wav_filename, samp_freq, bits_per_sample);
            if (inst == nullptr) {
                printf("error1 %d\r\n", fr);
                return;
            }
            _recording_flag = true;
            printf("start recording \"%s\" @ %d bits %5.1f KHz (bitrate: %6.1f Kbps)\r\n", wav_filename, bits_per_sample, (float) samp_freq*1e-3, (float) bits_per_sample*samp_freq*2*1e-3);
            if (_verbose) {
                printf("wav bw required:  %7.2f KB/s\r\n", (float) (static_cast<uint32_t>(bits_per_sample)*samp_freq*2/8) / 1e3);
            }
            set_last_suffix(_suffix);

            while (true) {
                uint queue_level = queue_get_level(&_spdif_queue);
                if (queue_level > queue_worst) queue_worst = queue_level;
                if (queue_level >= NUM_SUB_FRAME_BUF/2) {
                    while (queue_level > 0) {
                        sub_frame_buf_info_t buf_info;
                        // check blank status
                        if (_blank_split) {
                            queue_peek_blocking(&_spdif_queue, &buf_info);
                            blank_status_t blank_status = inst->get_blank_status(&_sub_frame_buf[SPDIF_BLOCK_SIZE * buf_info.buf_id], buf_info.sub_frame_count);
                            if (blank_status == blank_status_t::BLANK_END_DETECTED) {
                                end_recording();
                                start_recording(bits_per_sample);
                                break;
                            } else if (blank_status == blank_status_t::BLANK_SKIP) {
                                end_recording();
                                break;
                            }
                        }
                        // get and accumulate buffer
                        queue_remove_blocking(&_spdif_queue, &buf_info);
                        buf_accum++;
                        // write file depending on the conditions
                        if (queue_level == 1 || buf_info.buf_id >= NUM_SUB_FRAME_BUF - 1 || buf_accum >= NUM_SUB_FRAME_BUF/2) {
                            uint32_t sub_frame_count = buf_info.sub_frame_count * buf_accum;
                            uint64_t start_time = _micros();
                            uint32_t bytes = inst->write(next_buf, sub_frame_count);
                            uint32_t t_us = static_cast<uint32_t>(_micros() - start_time);
                            float bw = static_cast<float>(bytes) / t_us * 1e3;
                            if (bw > best_bw) best_bw = bw;
                            if (bw < worst_bw) {
                                worst_bw = bw;
                                if (_verbose) {
                                    printf("worst bw updated: %7.2f KB/s\r\n", worst_bw);
                                }
                            }
                            total_bytes += bytes;
                            total_time_us += t_us;
                            total_count += sub_frame_count;
                            // update head of next buffer point to write
                            next_buf = &_sub_frame_buf[SPDIF_BLOCK_SIZE * ((buf_info.buf_id + 1) % NUM_SUB_FRAME_BUF)];
                            buf_accum = 0;
                            break;
                        }
                        queue_level = queue_get_level(&_spdif_queue);
                        if (queue_level > queue_worst) queue_worst = queue_level;
                    }
                }

                if (queue_get_level(&_cmd_queue) > 0) {
                    queue_remove_blocking(&_cmd_queue, &cmd_data);
                    if (cmd_data.cmd == cmd_type_t::END_CMD) break;
                }
            }

            _recording_flag = false;
            fr = inst->finalize(total_count);
            if (fr != FR_OK) {
                printf("error3 %d\r\n", fr);
                return;
            }
            delete inst;
            inst = nullptr;
            uint32_t total_sec = total_bytes / (static_cast<uint32_t>(bits_per_sample)/8) / NUM_CHANNELS / samp_freq;
            uint32_t total_sec_dp = static_cast<uint64_t>(total_bytes) / (static_cast<uint32_t>(bits_per_sample)/8) / NUM_CHANNELS * 1000 / samp_freq - total_sec*1000;
            printf("recording done \"%s\" %d bytes (time:  %d:%02d.%03d)\r\n", wav_filename, total_bytes + WAV_HEADER_SIZE, total_sec/60, total_sec%60, total_sec_dp);
            if (_verbose) {
                float avg_bw = (float) total_bytes / total_time_us * 1e3;
                printf("SD Card writing bandwidth\r\n");
                printf(" avg:   %7.2f KB/s\r\n", avg_bw);
                printf(" best:  %7.2f KB/s\r\n", best_bw);
                printf(" worst: %7.2f KB/s\r\n", worst_bw);
                printf("WAV file required bandwidth\r\n");
                printf(" wav:   %7.2f KB/s\r\n", (float) (static_cast<uint32_t>(bits_per_sample)*samp_freq*2/8) / 1e3);
                printf("spdif queue usage: %7.2f %%\r\n", (float) queue_worst / SPDIF_QUEUE_LENGTH * 100);
            }
            _suffix++;
        }
        sleep_ms(10);
    }
}

int spdif_rec_wav::get_last_suffix()
{
    FIL fil;
    FRESULT fr;
    char buff[16];
    UINT br;
    int suffix;

    for ( ; ; ) {
        fr = f_open(&fil, _suffix_info_filename, FA_READ | FA_OPEN_EXISTING);
        if (fr != FR_OK) break;
        fr = f_read(&fil, buff, sizeof(buff), &br);
        if (fr != FR_OK && br > 0) break;
        suffix = atoi(buff);
        f_close(&fil);
        return suffix;
    }
    // no suffix info file or error
    return -1;
}

void spdif_rec_wav::set_last_suffix(int suffix)
{
    FIL fil;
    FRESULT fr;
    char buff[16];
    UINT bw;

    for ( ; ; ) {
        fr = f_open(&fil, _suffix_info_filename, FA_WRITE | FA_CREATE_ALWAYS);
        if (fr != FR_OK) break;
        sprintf(buff, "%d", suffix);
        fr = f_write(&fil, buff, sizeof(buff), &bw);
        if (fr != FR_OK || bw != sizeof(buff)) break;
        f_close(&fil);
        return;
    }
    // error
}

void spdif_rec_wav::push_sub_frame_buf(const uint32_t* buff, const uint32_t sub_frame_count)
{
    if (!_recording_flag) return;

    if (sub_frame_count != SPDIF_BLOCK_SIZE) {
        printf("ERROR: illegal sub_frame_count\r\n");
        return;
    }

    memcpy(&_sub_frame_buf[SPDIF_BLOCK_SIZE*_sub_frame_buf_id], buff, sub_frame_count * 4);
    sub_frame_buf_info_t buf_info = {_sub_frame_buf_id, sub_frame_count};
    if (!queue_try_add(&_spdif_queue, &buf_info)) {
        printf("ERROR: _spdif_queue is full\r\n");
    }
    _sub_frame_buf_id = (_sub_frame_buf_id + 1) % NUM_SUB_FRAME_BUF;
}

void spdif_rec_wav::start_recording(const bits_per_sample_t bits_per_sample)
{
    cmd_data_t cmd_data;
    cmd_data.cmd = cmd_type_t::START_CMD;
    cmd_data.param1 = static_cast<uint32_t>(spdif_rx_get_samp_freq());
    cmd_data.param2 = static_cast<uint32_t>(bits_per_sample);
    queue_try_add(&_cmd_queue, &cmd_data);
}

void spdif_rec_wav::end_recording()
{
    cmd_data_t cmd_data;
    cmd_data.cmd = cmd_type_t::END_CMD;
    cmd_data.param1 = 0L;
    cmd_data.param2 = 0L;
    queue_try_add(&_cmd_queue, &cmd_data);
}

bool spdif_rec_wav::is_recording()
{
    return _recording_flag;
}

void spdif_rec_wav::set_blank_split(const bool flag)
{
    _blank_split = flag;
}

bool spdif_rec_wav::get_blank_split()
{
    return _blank_split;
}

void spdif_rec_wav::set_verbose(const bool flag)
{
    _verbose = flag;
}

bool spdif_rec_wav::get_verbose()
{
    return _verbose;
}

int spdif_rec_wav::get_suffix()
{
    return _suffix;
}

void spdif_rec_wav::clear_suffix()
{
    _suffix = 0;
}

/*-----------------/
/  Constructor
/-----------------*/
spdif_rec_wav::spdif_rec_wav(const char* filename, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample)
 : _sample_freq(sample_freq), _bits_per_sample(bits_per_sample), _blank_sec(0.0f), _total_sec(0.0f)
{
    FRESULT fr;     /* FatFs return code */
    UINT br;
    UINT bw;
    uint8_t buf[WAV_HEADER_SIZE];
    uint16_t u16;
    uint32_t u32;

    for ( ; ; ) {
        fr = f_open(&_fil, filename, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
        if (fr != FR_OK) break;
        fr = f_lseek(&_fil, 0);
        if (fr != FR_OK) break;
        fr = f_truncate(&_fil);
        if (fr != FR_OK) break;

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
        u32 = _sample_freq * NUM_CHANNELS * static_cast<uint16_t>(_bits_per_sample) / 8;
        memcpy(&buf[28], (const void *) &u32, 4);
        // BlockAlign
        u16 = NUM_CHANNELS * static_cast<uint16_t>(_bits_per_sample) / 8;
        memcpy(&buf[32], (const void *) &u16, 2);
        // BitsPerSample
        u16 = static_cast<uint16_t>(_bits_per_sample);
        memcpy(&buf[34], (const void *) &u16, 2);

        // Subchunk2ID
        memcpy(&buf[36], "data", 4);
        // Subchunk2Size (temporary 0)
        memset(&buf[40], 0, 4);

        fr = f_write(&_fil, buf, sizeof(buf), &bw);
        if (fr != FR_OK || bw != sizeof(buf)) break;

        return;
    }
    // error
}

/*-----------------/
/  Destructor
/-----------------*/
spdif_rec_wav::~spdif_rec_wav()
{
}

/*--------------------/
/  Member functions
/--------------------*/
spdif_rec_wav::blank_status_t spdif_rec_wav::get_blank_status(const uint32_t* buff, const uint32_t sub_frame_count)
{
    uint32_t data_accum = 0;
    blank_status_t status = blank_status_t::NOT_BLANK;

    for (int i = 0; i < sub_frame_count; i++) {
        data_accum += std::abs(static_cast<int16_t>(((buff[i] >> 12) & 0xffff)));
    }

    float time_sec = (float) sub_frame_count / NUM_CHANNELS / _sample_freq;

    uint32_t ave_level = data_accum / sub_frame_count;
    if (ave_level < BLANK_LEVEL) {
        status = (_blank_sec > BLANK_SKIP_SEC) ? blank_status_t::BLANK_SKIP : blank_status_t::BLANK_DETECTED;
        _blank_sec += time_sec;
    } else {
        if (_total_sec >= BLANK_REPEAT_PROHIBIT_SEC && _blank_sec > BLANK_SEC) {
            if (_verbose) printf("detected blank end\r\n");
            status = blank_status_t::BLANK_END_DETECTED;
        }
        _blank_sec = 0.0f;
    }
    _total_sec += time_sec;

    return status;
}

uint32_t spdif_rec_wav::write(const uint32_t* buff, const uint32_t sub_frame_count)
{
    FRESULT fr;     /* FatFs return code */
    UINT br;
    UINT bw;

    if (_bits_per_sample == spdif_rec_wav::bits_per_sample_t::_16BITS) {
        for (int i = 0; i < sub_frame_count; i++) {
            _wav_buf[i/2] >>= 16;
            _wav_buf[i/2] |= ((buff[i] >> 12) & 0xffff) << 16;
        }
        fr = f_write(&_fil, static_cast<const void *>(_wav_buf), sub_frame_count*2, &bw);
        if (fr != FR_OK || bw != sub_frame_count*2) printf("error 4\r\n");
    } else if (_bits_per_sample == spdif_rec_wav::bits_per_sample_t::_24BITS) {
        for (int i = 0, j = 0; i < sub_frame_count; i += 4, j += 3) {
            _wav_buf[j+0] = (((buff[i+1] >> 4) & 0x0000ff) << 24) | (((buff[i+0] >> 4) & 0xffffff) >>  0);
            _wav_buf[j+1] = (((buff[i+2] >> 4) & 0x00ffff) << 16) | (((buff[i+1] >> 4) & 0xffff00) >>  8);
            _wav_buf[j+2] = (((buff[i+3] >> 4) & 0xffffff) <<  8) | (((buff[i+2] >> 4) & 0xff0000) >> 16);
        }
        fr = f_write(&_fil, static_cast<const void *>(_wav_buf), sub_frame_count*3, &bw);
        if (fr != FR_OK || bw != sub_frame_count*3) printf("error 4\r\n");
    }

    return static_cast<uint32_t>(bw);
}

FRESULT spdif_rec_wav::finalize(const int num_samp)
{
    FRESULT fr;     /* FatFs return code */
    UINT br;
    UINT bw;
    uint32_t u32;

    // ChunkSize (temporary 0)
    fr = f_lseek(&_fil, 4);
    if (fr != FR_OK) return fr;
    u32 = 36 + num_samp * NUM_CHANNELS * static_cast<uint16_t>(_bits_per_sample) / 8;
    fr = f_write(&_fil, static_cast<const void *>(&u32), sizeof(uint32_t), &bw);
    if (fr != FR_OK || bw != sizeof(uint32_t)) return fr;

    // Subchunk2Size (temporary 0)
    fr = f_lseek(&_fil, 40);
    if (fr != FR_OK) return fr;
    u32 = num_samp * NUM_CHANNELS * static_cast<uint16_t>(_bits_per_sample) / 8;
    fr = f_write(&_fil, static_cast<const void *>(&u32), sizeof(uint32_t), &bw);
    if (fr != FR_OK || bw != sizeof(uint32_t)) return fr;

    f_close(&_fil);

    return FR_OK;
}
