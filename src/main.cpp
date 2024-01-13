/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <cstdio>
#include <cstring>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "spdif_rx.h"
#include "tf_card.h"
#include "fatfs/ff.h"

static constexpr uint8_t PIN_DCDC_PSM_CTRL = 23;
static constexpr uint8_t PIN_PICO_SPDIF_RX_DATA = 15;
volatile static bool stable_flg = false;
volatile static bool lost_stable_flg = false;

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

FRESULT write_wav(FIL *fil, int num_samp)
{
    FRESULT fr;     /* FatFs return code */
    UINT br;
    UINT bw;
    uint32_t u32 = 0;

    for (int i = 0; i < num_samp; i++) {
        fr = f_write(fil, (const void *) &u32, sizeof(uint32_t), &bw);
        if (fr != FR_OK || bw != sizeof(uint32_t)) return fr;
    }

    return FR_OK;
}

FRESULT finalize_wav(FIL *fil, uint32_t sample_rate, uint16_t bits_per_sample, int num_samp)
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

int main()
{
    stdio_init_all();

    // DCDC PSM control
    // 0: PFM mode (best efficiency)
    // 1: PWM mode (improved ripple)
    gpio_init(PIN_DCDC_PSM_CTRL);
    gpio_set_dir(PIN_DCDC_PSM_CTRL, GPIO_OUT);
    gpio_put(PIN_DCDC_PSM_CTRL, 1); // PWM mode for less Audio noise

    // FATFS initialize
    FATFS fs;
    FIL fil;
    FRESULT fr;     /* FatFs return code */
    //UINT br;
    //UINT bw;
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

    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        printf("mount error %d\n", fr);
        return 1;
    }
    printf("mount ok\n");

    fr = prepare_wav("test.wav", 44100, 16, &fil);
    if (fr != FR_OK) {
        printf("error1 %d\n", fr);
        return 1;
    }
    fr = write_wav(&fil, 1000);
    if (fr != FR_OK) {
        printf("error2 %d\n", fr);
        return 1;
    }
    fr = finalize_wav(&fil, 44100, 16, 1000);
    if (fr != FR_OK) {
        printf("error3 %d\n", fr);
        return 1;
    }
    printf("done\n");

    // spdif_rx initialize
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

    int count = 0;
    while (true) {
        if (stable_flg) {
            stable_flg = false;
            printf("detected stable sync\n");
        }
        if (lost_stable_flg) {
            lost_stable_flg = false;
            printf("lost stable sync. waiting for signal\n");
        }
        if (count % 100 == 0 && spdif_rx_get_state() == SPDIF_RX_STATE_STABLE) {
            spdif_rx_samp_freq_t samp_freq = spdif_rx_get_samp_freq();
            float samp_freq_actual = spdif_rx_get_samp_freq_actual();
            uint32_t c_bits;
            spdif_rx_get_c_bits(&c_bits, sizeof(c_bits), 0);
            printf("Samp Freq = %d Hz (%7.4f KHz)\n", (int) samp_freq, samp_freq_actual / 1e3);
            printf("c_bits = 0x%08x\n", c_bits);
            printf("parity errors = %d\n", spdif_rx_get_parity_err_count());
        }
        tight_loop_contents();
        sleep_ms(10);
        count++;
    }

    return 0;
}
