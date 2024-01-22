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
static bool core1_running = false;
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

void spdif_rec_wav_process_loop()
{
    core1_running = true;
    spdif_rec_wav::process_loop();
    core1_running = false;
}

void show_help(spdif_rec_wav::bits_per_sample_t bits_per_sample)
{
    printf("---------------------------\r\n");
    printf(" bit resolution: %d bits\r\n", static_cast<int>(bits_per_sample));
    printf(" blank split:    %s\r\n", spdif_rec_wav::get_blank_split() ? "on" : "off");
    printf(" verbose:        %s\r\n", spdif_rec_wav::get_verbose() ? "on" : "off");
    printf(" suffix to rec:  %03d\r\n", spdif_rec_wav::get_suffix());
    printf("---------------------------\r\n");
    printf("[serial interface help]\r\n");
    printf(" ' ' to start/stop recording\r\n");
    printf(" 's' to immediate split of wav file (*1)\r\n");
    printf(" 'r' to switch 16/24 bits (*2)\r\n");
    printf(" 'b' to toggle blank split\r\n");
    printf(" 'v' to toggle verbose\r\n");
    printf(" 'c' to clear suffix (*2)\r\n");
    printf(" 'h' to show this help\r\n");
    printf("  (*1) not effective unless recording\r\n");
    printf("  (*2) not effective while recording\r\n");
    printf("---------------------------\r\n");
}

int main()
{
    int count = 0;
    bool wait_sync = false;
    bool standby_repeat = true;
    spdif_rec_wav::bits_per_sample_t bits_per_sample = spdif_rec_wav::bits_per_sample_t::_16BITS;
    char c;

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

    sleep_ms(500);  // wait for USB serial terminal to be responded
    printf("\r\n");

    // spdif_rec_wav process runs on Core1
    multicore_reset_core1();
    multicore_launch_core1(spdif_rec_wav_process_loop);

    sleep_ms(500);  // wait for FATFS to be mounted

    // Discard any input.
    while (getchar_timeout_us(1) >= 0) {};

    printf("---------------------------\r\n");
    printf("--- pico_spdif_recorder ---\r\n");
    show_help(bits_per_sample);

    while (true) {
        if (!core1_running) {
            printf("ERROR: spdif_rec_wav process_loop exit\r\n");
            break;
        }
        if (stable_flg) {
            stable_flg = false;
            printf("detected stable sync @ %d Hz\r\n", spdif_rx_get_samp_freq());
            if (wait_sync) {
                printf("start when sound detected\r\n");
                spdif_rec_wav::start_recording(bits_per_sample, true);  // standby start
                wait_sync = false;
            }
        }
        if (lost_stable_flg) {
            lost_stable_flg = false;
            printf("lost stable sync. waiting for signal\r\n");
            if (spdif_rec_wav::is_recording()) {
                spdif_rec_wav::end_recording();
                wait_sync = standby_repeat;
            }
        }
        if ((c = getchar_timeout_us(1)) > 0) {
            if (c == ' ') {
                if (spdif_rec_wav::is_recording()) {
                    spdif_rec_wav::end_recording();
                    standby_repeat = false;
                } else if (wait_sync) {
                    printf("wait_sync cancelled\r\n");
                    wait_sync = false;
                    standby_repeat = false;
                } else if (spdif_rx_get_state() == SPDIF_RX_STATE_STABLE) {
                    printf("start when sound detected\r\n");
                    spdif_rec_wav::start_recording(bits_per_sample, true);  // standby start
                    wait_sync = false;
                    standby_repeat = true;
                } else {
                    printf("start when stable sync detected\r\n");
                    wait_sync = true;
                    standby_repeat = true;
                }
            } else if (c == 's') {
                if (spdif_rec_wav::is_recording()) {
                    spdif_rec_wav::end_recording();
                    spdif_rec_wav::start_recording(bits_per_sample);
                }
            } else if (c == 'r') {
                if (!spdif_rec_wav::is_recording()) {
                    if (bits_per_sample == spdif_rec_wav::bits_per_sample_t::_16BITS) {
                        bits_per_sample = spdif_rec_wav::bits_per_sample_t::_24BITS;
                    } else {
                        bits_per_sample = spdif_rec_wav::bits_per_sample_t::_16BITS;
                    }
                    printf("bit resolution: %d bits\r\n", bits_per_sample);
                }
            } else if (c == 'b') {
                bool blank_split = !spdif_rec_wav::get_blank_split();
                spdif_rec_wav::set_blank_split(blank_split);
                printf("blank split: %s\r\n", blank_split ? "on" : "off");
            } else if (c == 'v') {
                bool verbose = !spdif_rec_wav::get_verbose();
                spdif_rec_wav::set_verbose(verbose);
                printf("verbose: %s\r\n", verbose ? "on" : "off");
            } else if (c == 'c') {
                if (!spdif_rec_wav::is_recording()) {
                    spdif_rec_wav::clear_suffix();
                    printf("suffix to rec: %03d\r\n", spdif_rec_wav::get_suffix());
                }
            } else if (c == 'h') {
                show_help(bits_per_sample);
            }
            // Discard any input of the rest.
            while (getchar_timeout_us(1) >= 0) {};
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

    sleep_ms(1000);  // time to output something to serial
    return 0;
}
