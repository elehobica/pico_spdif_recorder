/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <stdio.h>
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
    UINT br;
    UINT bw;
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
