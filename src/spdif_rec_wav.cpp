/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <cstdio>
#include <cstring>
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
uint32_t spdif_rec_wav::_sub_frame_buf[SPDIF_BLOCK_SIZE * NUM_SUB_FRAME_BUF];
int      spdif_rec_wav::_sub_frame_buf_id = 0;
uint32_t spdif_rec_wav::_wav_buf[SPDIF_BLOCK_SIZE*3/4 * NUM_SUB_FRAME_BUF / 2];
bool     spdif_rec_wav::_recording_flag;
queue_t  spdif_rec_wav::_spdif_queue;
queue_t  spdif_rec_wav::_record_wav_cmd_queue;

/*-----------------/
/  Class functions
/-----------------*/
void spdif_rec_wav::process_loop(const char* prefix)
{
    spdif_rec_wav *inst = nullptr;
    FATFS fs;
    FRESULT fr;     /* FatFs return code */
    int suffix = 0;
    record_wav_cmd_data_t cmd_data;
    char filename[16];
    uint32_t samp_freq;
    uint16_t bits_per_sample = WAV_16BITS;
    bool verbose = false;

    // Mount FATFS
    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        printf("mount error %d\r\n", fr);
        return;
    }
    printf("mount ok\r\n");

    // Initialize queues
    queue_init(&_spdif_queue, sizeof(sub_frame_buf_info_t), SPDIF_QUEUE_LENGTH);
    queue_init(&_record_wav_cmd_queue, sizeof(record_wav_cmd_data_t), RECORD_WAV_CMD_QUEUE_LENGTH);

    printf("spdif_rec_wav process started\r\n");

    // Initialize class variables
    _recording_flag = false;

    // Loop
    while (true) {
        if (queue_get_level(&_record_wav_cmd_queue) > 0) {
            queue_remove_blocking(&_record_wav_cmd_queue, &cmd_data);
            if (cmd_data.cmd == VERBOSE_CMD) {
                verbose = (bool) cmd_data.param1;
                printf("verbose: %s\r\n", verbose ? "on" : "off");
            }
            if (cmd_data.cmd != START_CMD) continue;

            samp_freq = cmd_data.param1;
            bits_per_sample = cmd_data.param2;
            int total_count = 0;
            uint32_t total_time_us = 0;
            uint32_t total_bytes = 0;
            float best_bw = -INFINITY;
            float worst_bw = INFINITY;
            uint queue_worst = 0;
            sprintf(filename, "%s%03d.wav", prefix, suffix);
            inst = new spdif_rec_wav(filename, samp_freq, bits_per_sample);
            if (inst == nullptr) {
                printf("error1 %d\r\n", fr);
                return;
            }
            _recording_flag = true;
            printf("start recording \"%s\" @ %d bits %d Hz (bitrate: %d bps)\r\n", filename, bits_per_sample, samp_freq, bits_per_sample*samp_freq*2);
            if (verbose) {
                printf("wav bw required:  %7.2f KB/s\r\n", (float) (bits_per_sample*samp_freq*2/8) / 1e3);
            }

            uint32_t *next_buf = &_sub_frame_buf[SPDIF_BLOCK_SIZE*0];
            while (true) {
                uint queue_level = queue_get_level(&_spdif_queue);
                if (queue_level > queue_worst) queue_worst = queue_level;
                if (queue_level >= NUM_SUB_FRAME_BUF/2) {
                    int buf_accum = 1;
                    while (queue_level > 0) {
                        sub_frame_buf_info_t buf_info;
                        queue_remove_blocking(&_spdif_queue, &buf_info);
                        if (queue_level == 1 || buf_info.buf_id >= NUM_SUB_FRAME_BUF - 1 || buf_accum >= NUM_SUB_FRAME_BUF/2) {
                            uint64_t start_time = _micros();
                            fr = inst->write(next_buf, buf_info.sub_frame_count * buf_accum);
                            uint32_t t_us = (uint32_t) (_micros() - start_time);
                            uint32_t bytes = buf_info.sub_frame_count * buf_accum * 4;
                            float bw = (float) bytes / t_us * 1e3;
                            if (bw > best_bw) best_bw = bw;
                            if (bw < worst_bw) {
                                worst_bw = bw;
                                if (verbose) {
                                    printf("worst bw updated: %7.2f KB/s\r\n", worst_bw);
                                }
                            }
                            total_time_us += t_us;
                            total_bytes += bytes;
                            if (fr != FR_OK) printf("error 4\r\n");
                            total_count += buf_info.sub_frame_count * buf_accum;
                            next_buf = &_sub_frame_buf[SPDIF_BLOCK_SIZE * ((buf_info.buf_id + 1) % NUM_SUB_FRAME_BUF)];
                            break;
                        } else {
                            buf_accum++;
                        }
                        queue_level = queue_get_level(&_spdif_queue);
                        if (queue_level > queue_worst) queue_worst = queue_level;
                    }
                }

                if (queue_get_level(&_record_wav_cmd_queue) > 0) {
                    queue_remove_blocking(&_record_wav_cmd_queue, &cmd_data);
                    if (cmd_data.cmd == VERBOSE_CMD) {
                        verbose = (bool) cmd_data.param1;
                        printf("verbose: %s\r\n", verbose ? "on" : "off");
                    }
                    if (cmd_data.cmd == END_CMD) break;
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
            printf("recording done \"%s\" %d bytes\r\n", filename, total_bytes);
            if (verbose) {
                float avg_bw = (float) total_bytes / total_time_us * 1e3;
                printf("SD Card writing bandwidth\r\n");
                printf(" avg:   %7.2f KB/s\r\n", avg_bw);
                printf(" best:  %7.2f KB/s\r\n", best_bw);
                printf(" worst: %7.2f KB/s\r\n", worst_bw);
                printf("WAV file required bandwidth\r\n");
                printf(" wav:   %7.2f KB/s\r\n", (float) (bits_per_sample*samp_freq*2/8) / 1e3);
                printf("spdif queue usage: %7.2f %%\r\n", (float) queue_worst / SPDIF_QUEUE_LENGTH * 100);
            }
            suffix++;
        }
        sleep_ms(10);
    }
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

void spdif_rec_wav::start_recording(uint16_t bits_per_sample)
{
    record_wav_cmd_data_t cmd_data;
    cmd_data.cmd = START_CMD;
    cmd_data.param1 = (uint32_t) spdif_rx_get_samp_freq();
    cmd_data.param2 = (uint32_t) bits_per_sample;
    queue_try_add(&_record_wav_cmd_queue, &cmd_data);
}

void spdif_rec_wav::end_recording()
{
    record_wav_cmd_data_t cmd_data;
    cmd_data.cmd = END_CMD;
    cmd_data.param1 = 0L;
    cmd_data.param2 = 0L;
    queue_try_add(&_record_wav_cmd_queue, &cmd_data);
}

void spdif_rec_wav::set_verbose(const bool flag)
{
    record_wav_cmd_data_t cmd_data;
    cmd_data.cmd = VERBOSE_CMD;
    cmd_data.param1 = (uint32_t) flag;
    cmd_data.param2 = 0L;
    queue_try_add(&_record_wav_cmd_queue, &cmd_data);
}

bool spdif_rec_wav::is_recording()
{
    return _recording_flag;
}

/*-----------------/
/  Constructor
/-----------------*/
spdif_rec_wav::spdif_rec_wav(const char* filename, const uint32_t sample_freq, const uint16_t bits_per_sample)
 : _sample_freq(sample_freq), _bits_per_sample(bits_per_sample)
{
    FRESULT fr;     /* FatFs return code */
    UINT br;
    UINT bw;
    constexpr int BUF_SIZE = 44;
    uint8_t buf[BUF_SIZE];
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
        u32 = _sample_freq * NUM_CHANNELS * _bits_per_sample / 8;
        memcpy(&buf[28], (const void *) &u32, 4);
        // BlockAlign
        u16 = NUM_CHANNELS * _bits_per_sample / 8;
        memcpy(&buf[32], (const void *) &u16, 2);
        // BitsPerSample
        u16 = _bits_per_sample;
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
FRESULT spdif_rec_wav::write(const uint32_t* buff, const uint32_t sub_frame_count)
{
    FRESULT fr;     /* FatFs return code */
    UINT br;
    UINT bw;

    if (_bits_per_sample == WAV_16BITS) {
        for (int i = 0; i < sub_frame_count; i++) {
            _wav_buf[i/2] >>= 16;
            _wav_buf[i/2] |= ((buff[i] >> 12) & 0xffff) << 16;
        }
        fr = f_write(&_fil, (const void *) _wav_buf, sub_frame_count*2, &bw);
        if (fr != FR_OK || bw != sub_frame_count*2) return fr;
    } else if (_bits_per_sample == WAV_24BITS) {
        for (int i = 0, j = 0; i < sub_frame_count; i += 4, j += 3) {
            _wav_buf[j+0] = (((buff[i+1] >> 4) & 0x0000ff) << 24) | (((buff[i+0] >> 4) & 0xffffff) >>  0);
            _wav_buf[j+1] = (((buff[i+2] >> 4) & 0x00ffff) << 16) | (((buff[i+1] >> 4) & 0xffff00) >>  8);
            _wav_buf[j+2] = (((buff[i+3] >> 4) & 0xffffff) <<  8) | (((buff[i+2] >> 4) & 0xff0000) >> 16);
        }
        fr = f_write(&_fil, (const void *) _wav_buf, sub_frame_count*3, &bw);
        if (fr != FR_OK || bw != sub_frame_count*3) return fr;
    }

    return FR_OK;
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
    u32 = 36 + num_samp * NUM_CHANNELS * _bits_per_sample / 8;
    fr = f_write(&_fil, (const void *) &u32, sizeof(uint32_t), &bw);
    if (fr != FR_OK || bw != sizeof(uint32_t)) return fr;

    // Subchunk2Size (temporary 0)
    fr = f_lseek(&_fil, 40);
    if (fr != FR_OK) return fr;
    u32 = num_samp * NUM_CHANNELS * _bits_per_sample / 8;
    fr = f_write(&_fil, (const void *) &u32, sizeof(uint32_t), &bw);
    if (fr != FR_OK || bw != sizeof(uint32_t)) return fr;

    f_close(&_fil);

    return FR_OK;
}