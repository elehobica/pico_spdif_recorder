/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <cstdio>
#include <cstring>
#include "record_wav.h"
#include "fatfs/ff.h"

/*---------------------------------------/
/  Global callback function by spdif_rx
/---------------------------------------*/
void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err)
{
    record_wav::push_sub_frame_buf(buff, sub_frame_count);
}

/*-----------------/
/  Class variables
/-----------------*/
uint32_t record_wav::_sub_frame_buf[SPDIF_BLOCK_SIZE * NUM_SUB_FRAME_BUF];
int      record_wav::_sub_frame_buf_id = 0;
uint32_t record_wav::_wav_buf[SPDIF_BLOCK_SIZE*3/4 * NUM_SUB_FRAME_BUF / 2];
bool     record_wav::_recording_flag;
queue_t  record_wav::_spdif_queue;
queue_t  record_wav::_record_wav_cmd_queue;

/*-----------------/
/  Class functions
/-----------------*/
void record_wav::process_loop()
{
    record_wav *inst = nullptr;
    FATFS fs;
    FRESULT fr;     /* FatFs return code */
    int suffix = 0;
    record_wav_cmd_data_t cmd_data;
    char filename[16];
    uint32_t samp_freq;
    uint16_t bits_per_sample = WAV_16BITS;

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

    printf("record_wav process started\r\n");

    // Initialize class variables
    _recording_flag = false;

    // Loop
    while (true) {
        if (queue_get_level(&_record_wav_cmd_queue) > 0) {
            queue_remove_blocking(&_record_wav_cmd_queue, &cmd_data);
            if (cmd_data.cmd != START_CMD) continue;

            samp_freq = cmd_data.param1;
            bits_per_sample = cmd_data.param2;
            int total_count = 0;
            sprintf(filename, "test%d.wav", suffix);
            inst = new record_wav(filename, samp_freq, bits_per_sample);
            if (inst == nullptr) {
                printf("error1 %d\r\n", fr);
                return;
            }
            _recording_flag = true;
            printf("start recording %s @ %d bits %d Hz\r\n", filename, bits_per_sample, samp_freq);

            uint32_t *next_buf = &_sub_frame_buf[SPDIF_BLOCK_SIZE*0];
            while (true) {
                uint queue_level = queue_get_level(&_spdif_queue);
                if (queue_level >= NUM_SUB_FRAME_BUF/2) {
                    int buf_accum = 1;
                    while (queue_level > 0) {
                        sub_frame_buf_info_t buf_info;
                        queue_remove_blocking(&_spdif_queue, &buf_info);
                        if (queue_level == 1 || buf_info.buf_id >= NUM_SUB_FRAME_BUF - 1) {
                            fr = inst->write(next_buf, buf_info.sub_frame_count * buf_accum);
                            total_count += buf_info.sub_frame_count * buf_accum;
                            if (fr != FR_OK) printf("error 4\r\n");
                            next_buf = &_sub_frame_buf[SPDIF_BLOCK_SIZE * ((buf_info.buf_id + 1) % NUM_SUB_FRAME_BUF)];
                            break;
                        } else {
                            buf_accum++;
                        }
                        queue_level = queue_get_level(&_spdif_queue);
                    }
                }

                if (queue_get_level(&_record_wav_cmd_queue) > 0) {
                    queue_remove_blocking(&_record_wav_cmd_queue, &cmd_data);
                    if (cmd_data.cmd == END_CMD) break;
                }
            }

            _recording_flag = false;
            fr = inst->finalize(total_count);
            if (fr != FR_OK) {
                printf("error3 %d\r\n", fr);
                return;
            }
            inst = nullptr;
            printf("recording done %s\r\n", filename);
            suffix++;
        }
        sleep_ms(10);
    }
}

void record_wav::push_sub_frame_buf(const uint32_t* buff, const uint32_t sub_frame_count)
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

void record_wav::start_recording(uint16_t bits_per_sample)
{
    record_wav_cmd_data_t cmd_data;
    cmd_data.cmd = START_CMD;
    cmd_data.param1 = (uint32_t) spdif_rx_get_samp_freq();
    cmd_data.param2 = (uint32_t) bits_per_sample;
    queue_try_add(&_record_wav_cmd_queue, &cmd_data);
}

void record_wav::end_recording()
{
    record_wav_cmd_data_t cmd_data;
    cmd_data.cmd = END_CMD;
    cmd_data.param1 = 0L;
    cmd_data.param2 = 0L;
    queue_try_add(&_record_wav_cmd_queue, &cmd_data);
}

bool record_wav::is_recording()
{
    return _recording_flag;
}

/*-----------------/
/  Constructor
/-----------------*/
record_wav::record_wav(const char *filename, const uint32_t sample_freq, const uint16_t bits_per_sample)
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
record_wav::~record_wav()
{
}

/*--------------------/
/  Member functions
/--------------------*/
FRESULT record_wav::write(const uint32_t* buff, const uint32_t sub_frame_count)
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

FRESULT record_wav::finalize(const int num_samp)
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
