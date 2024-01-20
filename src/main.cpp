/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <cstdio>
#include <cstring>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "spdif_rx.h"
#include "spdif_rec_wav.h"
#include "tf_card.h"

static constexpr uint PIN_LED = PICO_DEFAULT_LED_PIN;

static constexpr uint8_t PIN_DCDC_PSM_CTRL = 23;
static constexpr uint8_t PIN_PICO_SPDIF_RX_DATA = 15;
volatile static bool stable_flg = false;
volatile static bool lost_stable_flg = false;

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

void fatfs_config()
{
    // FATFS configuration
    pico_fatfs_spi_config_t fatfs_spi_config = {
        spi0,
        CLK_SLOW_DEFAULT,
        CLK_FAST_DEFAULT,
        PIN_SPI0_MISO_DEFAULT,
        PIN_SPI0_CS_DEFAULT,
        PIN_SPI0_SCK_DEFAULT,
        PIN_SPI0_MOSI_DEFAULT,
        true  // use internal pullup
    };
    pico_fatfs_set_config(&fatfs_spi_config);
}

void record_wav_process_loop()
{
    spdif_rec_wav::process_loop();
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

    // spdif_rx initialize
    spdif_rx_init();

    // FATFS config
    fatfs_config();

    // spdif_rec_wav process runs on Core1
    multicore_reset_core1();
    multicore_launch_core1(record_wav_process_loop);

    int count = 0;
    bool standby = false;
    bool standby_repeat = true;
    uint16_t bits_per_sample = WAV_16BITS;
    bool verbose = false;

    // Discard any input.
    while (uart_is_readable(uart0)) {
        uart_getc(uart0);
    }
    printf("\r\n");
    printf("bit resolution: %d bits\r\n", bits_per_sample);
    while (!uart_is_readable(uart0));

    while (true) {
        if (stable_flg) {
            stable_flg = false;
            printf("detected stable sync @ %d Hz\r\n", spdif_rx_get_samp_freq());
            if (standby) {
                spdif_rec_wav::start_recording(bits_per_sample);
                standby = false;
            }
        }
        if (lost_stable_flg) {
            lost_stable_flg = false;
            printf("lost stable sync. waiting for signal\r\n");
            if (spdif_rec_wav::is_recording()) {
                spdif_rec_wav::end_recording();
                standby = standby_repeat;
            }
        }
        if (uart_is_readable(uart0)) {
            char c = uart_getc(uart0);
            if (c == ' ') {
                if (spdif_rec_wav::is_recording()) {
                    spdif_rec_wav::end_recording();
                    standby_repeat = false;
                } else if (standby) {
                    printf("standby cancelled\r\n");
                    standby = false;
                    standby_repeat = false;
                } else if (spdif_rx_get_state() == SPDIF_RX_STATE_STABLE) {
                    spdif_rec_wav::start_recording(bits_per_sample);
                    standby = false;
                    standby_repeat = true;
                } else {
                    printf("standby start when stable sync detected\r\n");
                    standby = true;
                    standby_repeat = true;
                }
            } else if (c == 'r') {
                if (!spdif_rec_wav::is_recording()) {
                    if (bits_per_sample == WAV_16BITS) {
                        bits_per_sample = WAV_24BITS;
                    } else {
                        bits_per_sample = WAV_16BITS;
                    }
                    printf("bit resolution: %d bits\r\n", bits_per_sample);
                }
            } else if (c == 'v') {
                verbose = !verbose;
                spdif_rec_wav::set_verbose(verbose);
            }
            // Discard any input of the rest.
            while (uart_is_readable(uart0)) {
                uart_getc(uart0);
            }
        }
        if (spdif_rec_wav::is_recording()) {
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
