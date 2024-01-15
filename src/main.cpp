/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <cstdio>
#include <cstring>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "pico/util/queue.h"

#include "spdif_rx.h"
#include "tf_card.h"
#include "fatfs/ff.h"

static constexpr uint PIN_LED = PICO_DEFAULT_LED_PIN;

static constexpr int NUM_SUB_FRAME_BUF = 64;
static uint32_t _sub_frame_buf[SPDIF_BLOCK_SIZE * NUM_SUB_FRAME_BUF];
static int _sub_frame_buf_id = 0;

typedef struct _sub_frame_buf_info_t {
    int      buf_id;
    uint32_t sub_frame_count;
} sub_frame_buf_info_t;
static queue_t _spdif_queue;
static constexpr int SPDIF_QUEUE_LENGTH = NUM_SUB_FRAME_BUF - 1;

typedef enum _wav_dump_cmd_t {
    START = 0,
    STOP
} wav_dump_cmd_t;
typedef struct _wav_dump_cmd_data_t {
    wav_dump_cmd_t cmd;
    uint32_t       param;
} wav_dump_cmd_data_t;
static queue_t _wav_dump_cmd_queue;
static constexpr int WAV_DUMP_CMD_QUEUE_LENGTH = 1;

static constexpr uint8_t PIN_DCDC_PSM_CTRL = 23;
static constexpr uint8_t PIN_PICO_SPDIF_RX_DATA = 15;
volatile static bool stable_flg = false;
volatile static bool lost_stable_flg = false;

FIL *_g_fil = nullptr;

static inline uint32_t _millis()
{
    return to_ms_since_boot(get_absolute_time());
}

void on_stable_func(spdif_rx_samp_freq_t samp_freq)
{
    // callback function should be returned as quick as possible
    stable_flg = true;
}

void on_lost_stable_func()
{
    // callback function should be returned as quick as possible
    lost_stable_flg = true;
}

FRESULT prepare_wav(const char *filename, uint32_t sample_rate, uint16_t bits_per_sample, FIL *fil)
{
    FRESULT fr;     /* FatFs return code */
    UINT br;
    UINT bw;
    const int NUM_CHANNELS = 2;
    const int BUF_SIZE = 44;
    uint8_t buf[BUF_SIZE];
    uint16_t u16;
    uint32_t u32;

    fr = f_open(fil, filename, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) return fr;
    fr = f_lseek(fil, 0);
    if (fr != FR_OK) return fr;
    fr = f_truncate(fil);
    if (fr != FR_OK) return fr;

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
    u32 = sample_rate;  // e.g. 44100
    memcpy(&buf[24], (const void *) &u32, 4);
    // ByteRate
    u32 = sample_rate * NUM_CHANNELS * bits_per_sample / 8;
    memcpy(&buf[28], (const void *) &u32, 4);
    // BlockAlign
    u16 = NUM_CHANNELS * bits_per_sample / 8;
    memcpy(&buf[32], (const void *) &u16, 2);
    // BitsPerSample
    u16 = bits_per_sample;
    memcpy(&buf[34], (const void *) &u16, 2);

    // Subchunk2ID
    memcpy(&buf[36], "data", 4);
    // Subchunk2Size (temporary 0)
    memset(&buf[40], 0, 4);

    fr = f_write(fil, buf, sizeof(buf), &bw);
    if (fr != FR_OK || bw != sizeof(buf)) return fr;

    return FR_OK;
}

FRESULT write_wav(uint32_t* buff, uint32_t sub_frame_count)
{
    FRESULT fr;     /* FatFs return code */
    UINT br;
    UINT bw;
    uint32_t wav_buf[sub_frame_count/2];

    for (int i = 0; i < sub_frame_count; i++) {
        wav_buf[i/2] >>= 16;
        wav_buf[i/2] |= ((buff[i] >> 12) & 0xffff) << 16;
    }

    fr = f_write(_g_fil, (const void *) wav_buf, sub_frame_count*2, &bw);
    if (fr != FR_OK || bw != sub_frame_count*2) return fr;

    return FR_OK;
}

FRESULT finalize_wav(FIL *fil, uint16_t bits_per_sample, int num_samp)
{
    FRESULT fr;     /* FatFs return code */
    UINT br;
    UINT bw;
    const int NUM_CHANNELS = 2;
    uint32_t u32;

    // ChunkSize (temporary 0)
    fr = f_lseek(fil, 4);
    if (fr != FR_OK) return fr;
    u32 = 36 + num_samp * NUM_CHANNELS * bits_per_sample / 8;
    fr = f_write(fil, (const void *) &u32, sizeof(uint32_t), &bw);
    if (fr != FR_OK || bw != sizeof(uint32_t)) return fr;

    // Subchunk2Size (temporary 0)
    fr = f_lseek(fil, 40);
    if (fr != FR_OK) return fr;
    u32 = num_samp * NUM_CHANNELS * bits_per_sample / 8;
    fr = f_write(fil, (const void *) &u32, sizeof(uint32_t), &bw);
    if (fr != FR_OK || bw != sizeof(uint32_t)) return fr;

    f_close(fil);

    return FR_OK;
}

void spdif_rx_init()
{
    spdif_rx_config_t spdif_rx_config = {
        .data_pin = PIN_PICO_SPDIF_RX_DATA,
        .pio_sm = 0,
        .dma_channel0 = 0,
        .dma_channel1 = 1,
        .alarm = 0,
        .flags = SPDIF_RX_FLAGS_ALL
    };

    spdif_rx_start(&spdif_rx_config);
    spdif_rx_set_callback_on_stable(on_stable_func);
    spdif_rx_set_callback_on_lost_stable(on_lost_stable_func);
}

extern "C" {
void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err)
{
    if (_g_fil == nullptr) return;

    memcpy(&_sub_frame_buf[SPDIF_BLOCK_SIZE*_sub_frame_buf_id], buff, sub_frame_count * 4);
    sub_frame_buf_info_t buf_info = {_sub_frame_buf_id, sub_frame_count};
    if (!queue_try_add(&_spdif_queue, &buf_info)) {
        printf("ERROR: _spdif_queue is full\r\n");
    }
    _sub_frame_buf_id = (_sub_frame_buf_id + 1) % NUM_SUB_FRAME_BUF;
}
}

void dump_wav()
{
    FIL fil;
    FRESULT fr;     /* FatFs return code */
    int suffix = 0;
    wav_dump_cmd_data_t cmd_data;
    char filename[16];
    uint32_t samp_freq;
    const uint16_t bits_per_sample = 16;

    printf("dump_wav() loop\n");

    while (true) {
        if (queue_get_level(&_wav_dump_cmd_queue) > 0) {
            queue_remove_blocking(&_wav_dump_cmd_queue, &cmd_data);
            if (cmd_data.cmd != START) continue;

            samp_freq = cmd_data.param;
            int total_count = 0;
            sprintf(filename, "test%d.wav", suffix);
            fr = prepare_wav(filename, samp_freq, bits_per_sample, &fil);
            if (fr != FR_OK) {
                printf("error1 %d\n", fr);
                return;
            }
            printf("start dumping %s @ %d\n", filename, samp_freq);
            _g_fil = &fil;

            uint32_t *next_buf = &_sub_frame_buf[SPDIF_BLOCK_SIZE*0];
            while (true) {
                uint queue_level = queue_get_level(&_spdif_queue);
                if (queue_level >= NUM_SUB_FRAME_BUF/2) {
                    int buf_accum = 1;
                    while (queue_level > 0) {
                        sub_frame_buf_info_t buf_info;
                        queue_remove_blocking(&_spdif_queue, &buf_info);
                        if (queue_level == 1 || buf_info.buf_id >= NUM_SUB_FRAME_BUF - 1) {
                            fr = write_wav(next_buf, buf_info.sub_frame_count * buf_accum);
                            total_count += buf_info.sub_frame_count * buf_accum;
                            if (fr != FR_OK) printf("error 4\n");
                            next_buf = &_sub_frame_buf[SPDIF_BLOCK_SIZE * ((buf_info.buf_id + 1) % NUM_SUB_FRAME_BUF)];
                            break;
                        } else {
                            buf_accum++;
                        }
                        queue_level = queue_get_level(&_spdif_queue);
                    }
                }

                if (queue_get_level(&_wav_dump_cmd_queue) > 0) {
                    queue_remove_blocking(&_wav_dump_cmd_queue, &cmd_data);
                    if (cmd_data.cmd == STOP) break;
                }
            }

            fr = finalize_wav(_g_fil, bits_per_sample, total_count);
            if (fr != FR_OK) {
                printf("error3 %d\n", fr);
                return;
            }
            _g_fil = nullptr;
            printf("dump done %s\n", filename);
            suffix++;
        }
        sleep_ms(10);
    }
}

int main()
{
    stdio_init_all();

    // LED
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, false);

    // DCDC PSM control
    // 0: PFM mode (best efficiency)
    // 1: PWM mode (improved ripple)
    gpio_init(PIN_DCDC_PSM_CTRL);
    gpio_set_dir(PIN_DCDC_PSM_CTRL, GPIO_OUT);
    gpio_put(PIN_DCDC_PSM_CTRL, 1); // PWM mode for less Audio noise

    // Queues
    queue_init(&_spdif_queue, sizeof(sub_frame_buf_info_t), SPDIF_QUEUE_LENGTH);
    queue_init(&_wav_dump_cmd_queue, sizeof(wav_dump_cmd_data_t), WAV_DUMP_CMD_QUEUE_LENGTH);

    // FATFS initialize
    FATFS fs;
    FRESULT fr;     /* FatFs return code */
    pico_fatfs_spi_config_t fatfs_spi_config = {
        spi0,
        CLK_SLOW_DEFAULT,
        CLK_FAST_DEFAULT,
        PIN_SPI0_MISO_DEFAULT,
        PIN_SPI0_CS_DEFAULT,
        PIN_SPI0_SCK_DEFAULT,
        PIN_SPI0_MOSI_DEFAULT,
        true
    };
    pico_fatfs_set_config(&fatfs_spi_config);

    // spdif_rx initialize
    spdif_rx_init();

    // mount fatfs
    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        printf("mount error %d\n", fr);
        return 1;
    }
    printf("mount ok\n");

    // Discard any input.
    while (uart_is_readable(uart0)) {
        uart_getc(uart0);
    }
    printf("\n");
    printf("Type any character to start\n");
    while (!uart_is_readable_within_us(uart0, 1000));

    // dump_wav runs on Core1
    multicore_reset_core1();
    multicore_launch_core1(dump_wav);

    int count = 0;
    bool is_dumping = false;
    while (true) {
        if (stable_flg) {
            stable_flg = false;
            printf("detected stable sync\n");
        }
        if (lost_stable_flg) {
            lost_stable_flg = false;
            printf("lost stable sync. waiting for signal\n");
            if (is_dumping) {
                wav_dump_cmd_data_t cmd_data;
                cmd_data.cmd = STOP;
                cmd_data.param = 0L;
                printf("Type any character to start\n");
                queue_try_add(&_wav_dump_cmd_queue, &cmd_data);
                is_dumping = !is_dumping;
            }
        }
        if (uart_is_readable_within_us(uart0, 0)) {
            wav_dump_cmd_data_t cmd_data;
            if (is_dumping) {
                cmd_data.cmd = STOP;
                cmd_data.param = 0L;
                printf("Type any character to start\n");
            } else {
                cmd_data.cmd = START;
                cmd_data.param = (uint32_t) spdif_rx_get_samp_freq();
                printf("Type any character to stop\n");
            }
            queue_try_add(&_wav_dump_cmd_queue, &cmd_data);
            is_dumping = !is_dumping;
            // Discard any input.
            while (uart_is_readable(uart0)) {
                uart_getc(uart0);
            }
        }
        if (is_dumping) {
            gpio_put(PIN_LED, (_millis() / 500) % 2 == 0);
        } else {
            gpio_put(PIN_LED, false);
        }

        tight_loop_contents();
        sleep_ms(10);
        count++;
    }

    return 0;
}
