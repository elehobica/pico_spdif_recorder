/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <cstdio>

#include "hardware/adc.h"
#include "hardware/rtc.h"
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "pico/util/queue.h"

#include "tf_card.h"
#include "ff.h"
#include "spdif_rx.h"
#include "spdif_rec_wav.h"
#include "config_wifi.h"
#include "ntp_client.h"
#include "ConfigParam.h"

bool picoW = false;
static constexpr uint PIN_LED = 25;  // PICO_DEFAULT_LED_PIN of Pico

static constexpr uint8_t PIN_DCDC_PSM_CTRL = 23;
static constexpr uint8_t PIN_PICO_SPDIF_RX_DATA = 15;

static constexpr uint PIN_SWITCH_24BIT      = 6;
static constexpr uint PIN_BUTTON_START_STOP = 7;

enum class main_error_t {
    CYW43_INIT_ERROR = 0,
    WIFI_CONNECTION_ERROR,
    TIMER_ERROR,
    NTP_ERROR,
    FATFS_ERROR,
    FLASH_PROGRAM_ERROR,
    MAIN_LOOP_ABORTED_ERROR
};

static repeating_timer_t timer;
static constexpr int INTERVAL_BUTTONS_CHECK_MS = 50;
static constexpr int NUM_SIGNAL_FILTER_STEPS = 2;  // 1 ~ 31
static constexpr uint32_t SIGNAL_FILTER_MASK = ((1UL << NUM_SIGNAL_FILTER_STEPS) - 1);
static constexpr int NUM_SIGNALS = 2;
static uint32_t signal_history[NUM_SIGNALS];
static uint32_t signal_history_filtered_pos[NUM_SIGNALS] = {0UL, 0UL};
static uint32_t signal_history_filtered_neg[NUM_SIGNALS] = {~0UL, ~0UL};

static bool core1_running = false;
volatile static bool stable_flg = false;
volatile static bool lost_stable_flg = false;
enum class signal_event_t {
    NONE = 0,
    BUTTON_PUSHED,
    SWITCH_ON,
    SWITCH_OFF
};

static inline uint32_t _millis()
{
    return to_ms_since_boot(get_absolute_time());
}

static bool _check_pico_w()
{
    adc_init();
    auto dir = gpio_get_dir(29);
    auto fnc = gpio_get_function(29);
    adc_gpio_init(29);
    adc_select_input(3);
    auto adc29 = adc_read();
    gpio_set_function(29, fnc);
    gpio_set_dir(29, dir);

    dir = gpio_get_dir(25);
    fnc = gpio_get_function(25);
    gpio_init(25);
    gpio_set_dir(25, GPIO_IN);
    auto gp25 = gpio_get(25);
    gpio_set_function(25, fnc);
    gpio_set_dir(25, dir);

    if (gp25) {
        return true; // Can't tell, so assume yes
    } else if (adc29 < 200) {
        return true; // PicoW
    } else {
        return false;
    }
}

static void _set_led(bool flag)
{
    if (picoW) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, flag);
    } else {
        gpio_put(PIN_LED, flag);
    }
}

static void _on_stable_func(spdif_rx_samp_freq_t samp_freq)
{
    // callback function should be returned as quick as possible
    stable_flg = true;
}

static void _on_lost_stable_func()
{
    // callback function should be returned as quick as possible
    lost_stable_flg = true;
}

static void _spdif_rx_init()
{
    spdif_rx_config_t spdif_rx_config = {
        .data_pin = PIN_PICO_SPDIF_RX_DATA,
        .pio_sm = 0,
        .dma_channel0 = 2,
        .dma_channel1 = 3,
        .alarm_pool = alarm_pool_get_default(),
        .flags = SPDIF_RX_FLAGS_ALL
    };

    spdif_rx_start(&spdif_rx_config);
    spdif_rx_set_callback_on_stable(_on_stable_func);
    spdif_rx_set_callback_on_lost_stable(_on_lost_stable_func);
}

static bool _fatfs_init()
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

    // Mount FATFS
    FATFS fs;
    FRESULT fr;
    for (int i = 0; i < 5; i++) {
        fr = f_mount(&fs, "", 1);
        if (fr == FR_OK) { break; }
        pico_fatfs_reboot_spi();
    }
    if (fr != FR_OK) {
        printf("FATFS mount error %d\r\n", fr);
        return false;
    }
    printf("FATFS mount ok\r\n");

    // Print card info
    const char* format = (fs.fs_type == FS_FAT12) ? "FAT12" :
                         (fs.fs_type == FS_FAT16) ? "FAT16" :
                         (fs.fs_type == FS_FAT32) ? "FAT32" :
                         (fs.fs_type == FS_EXFAT) ? "exFAT" : "unknown format";
    printf("Card info: %s %7.2f GB (GB = 1E9 Bytes)\n\n", format, fs.csize * fs.n_fatent * 512E-9);

    return true;
}

static void _core1_process()
{
    core1_running = true;
    flash_safe_execute_core_init();  // no access to flash on core1

    spdif_rec_wav::record_process_loop();
    core1_running = false;
}

static void _show_help(const bits_per_sample_t bits_per_sample)
{
    printf("---------------------------\r\n");
    printf(" bit resolution: %d bits\r\n", static_cast<int>(bits_per_sample));
    printf(" blank split:    %s\r\n", spdif_rec_wav::get_blank_split() ? "on" : "off");
    printf(" verbose:        %s\r\n", spdif_rec_wav::get_verbose() ? "on" : "off");
    printf(" suffix to rec:  %03d\r\n", spdif_rec_wav::get_suffix());
    printf("---------------------------\r\n");
    printf("[serial interface help]\r\n");
    printf(" ' ' to start/stop recording\r\n");
    printf(" 'r' to switch 16/24 bits\r\n");
    printf(" 's' to manual split (*1)\r\n");
    printf(" 'b' to toggle auto blank split\r\n");
    printf(" 'v' to toggle verbose\r\n");
    printf(" 'c' to clear suffix (*2)\r\n");
    if (picoW) {
        printf(" 'w' to configure wifi (*2)\r\n");
    }
    printf(" 'h' to show this help\r\n");
    printf("  (*1) only while recording\r\n");
    printf("  (*2) only while not recording\r\n");
    printf("---------------------------\r\n");
}

static void _led_blink_during_wait()
{
    _set_led((_millis() / 100) % 2 == 0);
    if (getchar_timeout_us(1) != PICO_ERROR_TIMEOUT) {
        printf("can't accept any commands during background file process\r\n");
        while (getchar_timeout_us(1) != PICO_ERROR_TIMEOUT) {};  // Discard any input.
    }
    sleep_ms(10);
}

static signal_event_t _scan_buttons()
{
    signal_event_t event;
    uint32_t signal[NUM_SIGNALS] = {
        gpio_get(PIN_SWITCH_24BIT),
        gpio_get(PIN_BUTTON_START_STOP)
    };
    for (int i = 0; i < NUM_SIGNALS; i++) {
        signal_history[i] = (signal_history[i] << 1) | (signal[i] & 0b1);
        if ((signal_history[i] & SIGNAL_FILTER_MASK) == SIGNAL_FILTER_MASK) {
            signal_history_filtered_pos[i] = (signal_history_filtered_pos[i] << 1) | 0b1;
            signal_history_filtered_neg[i] = (signal_history_filtered_neg[i] << 1) | 0b1;
        } else if ((signal_history[i] & SIGNAL_FILTER_MASK) == 0x0) {
            signal_history_filtered_pos[i] = (signal_history_filtered_pos[i] << 1) | 0b0;
            signal_history_filtered_neg[i] = (signal_history_filtered_neg[i] << 1) | 0b0;
        } else {
            signal_history_filtered_pos[i] = (signal_history_filtered_pos[i] << 1) | (signal_history_filtered_pos[i] & 0b1);
            signal_history_filtered_neg[i] = (signal_history_filtered_neg[i] << 1) | (signal_history_filtered_neg[i] & 0b1);
        }
    }
    if ((signal_history_filtered_pos[0] & 0b11) == 0b10) {
        event = signal_event_t::SWITCH_ON;
    } else if ((signal_history_filtered_neg[0] & 0b11) == 0b01) {
        event = signal_event_t::SWITCH_OFF;
    } else if ((signal_history_filtered_pos[1] & 0b11) == 0b10) {
        event = signal_event_t::BUTTON_PUSHED;
    } else {
        event = signal_event_t::NONE;
    }
    return event;
}

static void _toggle_start_stop(const bits_per_sample_t bits_per_sample, bool& wait_sync, bool& user_standy, bool& standby_repeat)
{
    if (spdif_rec_wav::is_standby()) {
        if (user_standy) {
            printf("cancelled\r\n");
            spdif_rec_wav::end_recording();
            user_standy = false;
            standby_repeat = false;
        } else {
            // no command needed because already in standby
            printf("start when sound detected\r\n");
            user_standy = true;
            standby_repeat = true;
        }
    } else if (spdif_rec_wav::is_recording()) {
        spdif_rec_wav::end_recording();
        user_standy = false;
        standby_repeat = false;
    } else if (wait_sync) {
        printf("wait_sync cancelled\r\n");
        wait_sync = false;
        user_standy = false;
        standby_repeat = false;
    } else if (spdif_rx_get_state() == SPDIF_RX_STATE_STABLE) {
        printf("start when sound detected\r\n");
        spdif_rec_wav::start_recording(bits_per_sample, true);  // standby start
        wait_sync = false;
        user_standy = true;
        standby_repeat = true;
    } else {
        printf("start when stable sync detected\r\n");
        wait_sync = true;
        user_standy = true;
        standby_repeat = true;
    }
}

static void _toggle_bit_resolution(bits_per_sample_t& bits_per_sample)
{
    if (bits_per_sample == bits_per_sample_t::_16BITS) {
        bits_per_sample = bits_per_sample_t::_24BITS;
    } else {
        bits_per_sample = bits_per_sample_t::_16BITS;
    }
}

static bool _connect_wifi(const std::string& ssid, const std::string& password, const int retry = 3)
{
    printf("... connecting Wi-Fi\r\n");

    cyw43_arch_enable_sta_mode();
    for (int i = 0; i < retry; i++) {
        if (cyw43_arch_wifi_connect_timeout_ms(ssid.c_str(), password.c_str(), CYW43_AUTH_WPA2_AES_PSK, 10000)) {
            printf("failed to connect: %d\r\n", i);
            if (i < retry - 1) {
                continue;
            } else {
                return false;
            }
        }
    }
    printf("connected\r\n");

    return true;
}

static void _led_disp_error(const main_error_t error, const bool forever = false)
{
    int n = static_cast<int>(error) + 5;  // start from blink 5 times
    while (true) {
        for (int i = 0; i < n*2; i++) {
            _set_led(false);
            sleep_ms(200);
            if (i < n) _set_led(true);
            sleep_ms(200);
        }
        if (!forever) break;
    }
}

int main()
{
    int count = 0;
    bool wait_sync = false;
    bool user_standy = false;
    bool standby_repeat = true;
    bits_per_sample_t bits_per_sample = bits_per_sample_t::_16BITS;
    int chr;
    // default RTC time
    datetime_t t_rtc = {
        .year  = static_cast<int16_t>(2024),
        .month = static_cast<int8_t>(1),
        .day   = static_cast<int8_t>(1),
        .dotw  = static_cast<int8_t>(1),  // 0 is Sunday, so 5 is Friday
        .hour  = static_cast<int8_t>(0),
        .min   = static_cast<int8_t>(0),
        .sec   = static_cast<int8_t>(0)
    };

    stdio_init_all();
    picoW = _check_pico_w();

    // serial connection waiting (max 1 sec)
    while (!stdio_usb_connected() && _millis() < 1000) {
        sleep_ms(100);
    }
    printf("\r\n");

    // print configuration parameters in flash
    //configParam.printInfo();

    // DCDC PSM control
    // 0: PFM mode (best efficiency)
    // 1: PWM mode (improved ripple)
    gpio_init(PIN_DCDC_PSM_CTRL);
    gpio_set_dir(PIN_DCDC_PSM_CTRL, GPIO_OUT);
    gpio_put(PIN_DCDC_PSM_CTRL, 1); // PWM mode for less Audio noise

    // Button/Switch
    gpio_init(PIN_BUTTON_START_STOP);
    gpio_set_dir(PIN_BUTTON_START_STOP, GPIO_IN);
    gpio_pull_up(PIN_BUTTON_START_STOP);
    gpio_init(PIN_SWITCH_24BIT);
    gpio_set_dir(PIN_SWITCH_24BIT, GPIO_IN);
    gpio_pull_up(PIN_SWITCH_24BIT);
    bits_per_sample = gpio_get(PIN_SWITCH_24BIT) ? bits_per_sample_t::_16BITS : bits_per_sample_t::_24BITS;

    // spdif_rx initialize
    _spdif_rx_init();

    // Pico / Pico W dependencies
    // cyw43_arch_init() needs to be done after _spdif_rx_init() (by unknown reason)
    if (picoW) {
        if (cyw43_arch_init()) {  // this is needed for driving LED
            printf("cyw43 init failed\r\n");
            _led_disp_error(main_error_t::CYW43_INIT_ERROR, true);
            return 1;
        }
        printf("Pico W\r\n");
        // connect Wi-Fi to get time by NTP
        if (GET_CFG_WIFI_SSID[0] >= 0x20 && GET_CFG_WIFI_SSID[0] <= 0x7e && GET_CFG_WIFI_PASS[0] >= 0x20 && GET_CFG_WIFI_PASS[0] <= 0x7e) {
            _set_led(true);
            if (_connect_wifi(std::string(GET_CFG_WIFI_SSID), std::string(GET_CFG_WIFI_PASS))) {
                if (!run_ntp("UTC+0", t_rtc)) {
                    printf("ERROR: failed NTP sync\r\n");
                    _led_disp_error(main_error_t::NTP_ERROR);
                }
            } else {
                printf("ERROR: failed Wi-Fi connection: %s\r\n", GET_CFG_WIFI_SSID);
                _led_disp_error(main_error_t::WIFI_CONNECTION_ERROR);
            }
        } else {
            printf("No Wi-Fi SSID information. Skip Wi-Fi connection for NTP\r\n");
        }
    } else {
        printf("Pico\r\n");
        // LED
        gpio_init(PIN_LED);
        gpio_set_dir(PIN_LED, GPIO_OUT);
    }
    _set_led(false);

    // RTC
    rtc_init();
    rtc_set_datetime(&t_rtc);

    // FATFS initialize
    if (!_fatfs_init()) {
        _led_disp_error(main_error_t::FATFS_ERROR, true);
        return 1;
    }

    spdif_rec_wav::set_wait_grant_func(_led_blink_during_wait);
    // spdif_rec_wav process runs on Core1
    multicore_reset_core1();
    multicore_launch_core1(_core1_process);

    sleep_ms(500);  // wait for FATFS to be mounted

    // Discard any input.
    while (getchar_timeout_us(1) != PICO_ERROR_TIMEOUT) {};

    printf("---------------------------\r\n");
    printf("--- pico_spdif_recorder ---\r\n");
    _show_help(bits_per_sample);

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
                user_standy = true;
            }
        }
        if (lost_stable_flg) {
            lost_stable_flg = false;
            printf("lost stable sync. waiting for signal\r\n");
            if (spdif_rec_wav::is_standby() || spdif_rec_wav::is_recording()) {
                spdif_rec_wav::end_recording();
                wait_sync = standby_repeat;
                user_standy = false;
            }
        }
        signal_event_t event = _scan_buttons();
        if (event == signal_event_t::BUTTON_PUSHED) {
            _toggle_start_stop(bits_per_sample, wait_sync, user_standy, standby_repeat);
        } else if (event == signal_event_t::SWITCH_ON) {
            if (bits_per_sample != bits_per_sample_t::_24BITS) {
                if (spdif_rec_wav::is_standby() || spdif_rec_wav::is_recording()) {
                    _toggle_start_stop(bits_per_sample, wait_sync, user_standy, standby_repeat);
                    bits_per_sample = bits_per_sample_t::_24BITS;
                    sleep_ms(100);
                    _toggle_start_stop(bits_per_sample, wait_sync, user_standy, standby_repeat);
                } else {
                    bits_per_sample = bits_per_sample_t::_24BITS;
                }
            }
            printf("bit resolution: %d bits\r\n", bits_per_sample);
        } else if (event == signal_event_t::SWITCH_OFF) {
            if (bits_per_sample != bits_per_sample_t::_16BITS) {
                if (spdif_rec_wav::is_standby() || spdif_rec_wav::is_recording()) {
                    _toggle_start_stop(bits_per_sample, wait_sync, user_standy, standby_repeat);
                    bits_per_sample = bits_per_sample_t::_16BITS;
                    sleep_ms(100);
                    _toggle_start_stop(bits_per_sample, wait_sync, user_standy, standby_repeat);
                } else {
                    bits_per_sample = bits_per_sample_t::_16BITS;
                }
            }
            printf("bit resolution: %d bits\r\n", bits_per_sample);
        } else if ((chr = getchar_timeout_us(1)) != PICO_ERROR_TIMEOUT) {
            char c = static_cast<char>(chr);
            if (c == ' ') {
                _toggle_start_stop(bits_per_sample, wait_sync, user_standy, standby_repeat);
            } else if (c == 'r') {
                if (spdif_rec_wav::is_standby() || spdif_rec_wav::is_recording()) {
                    _toggle_start_stop(bits_per_sample, wait_sync, user_standy, standby_repeat);
                    _toggle_bit_resolution(bits_per_sample);
                    sleep_ms(100);
                    _toggle_start_stop(bits_per_sample, wait_sync, user_standy, standby_repeat);
                } else {
                    _toggle_bit_resolution(bits_per_sample);
                }
                printf("bit resolution: %d bits\r\n", bits_per_sample);
            } else if (c == 's') {
                if (spdif_rec_wav::is_recording()) {
                    spdif_rec_wav::split_recording(bits_per_sample);
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
                if (!spdif_rec_wav::is_standby() && !spdif_rec_wav::is_recording()) {
                    spdif_rec_wav::clear_suffix();
                    printf("suffix to rec: %03d\r\n", spdif_rec_wav::get_suffix());
                }
            } else if (c == 'w' && picoW) {
                std::string ssid, password;
                if (config_wifi(ssid, password)) {
                    // store to flash
                    configParam.setStr(ConfigParam::ParamID_t::CFG_WIFI_SSID, ssid.c_str());
                    configParam.setStr(ConfigParam::ParamID_t::CFG_WIFI_PASS, password.c_str());
                    if (configParam.finalize()) {
                        // trial to connect Wi-Fi
                        if (_connect_wifi(ssid, password)) {
                            if (run_ntp("UTC+0", t_rtc)) {
                                rtc_set_datetime(&t_rtc);
                            }
                        }
                    } else {
                        printf("ERROR: failed to program flash\r\n");
                        _led_disp_error(main_error_t::FLASH_PROGRAM_ERROR);
                    }
                }
            } else if (c == 'h') {
                _show_help(bits_per_sample);
            }
            // Discard any input of the rest.
            while (getchar_timeout_us(1) != PICO_ERROR_TIMEOUT) {};
        }
        if (spdif_rec_wav::is_recording()) {
            _set_led((_millis() / 500) % 2 == 0);
        } else {
            _set_led(false);
        }

        // background file process on core0
        spdif_rec_wav::process_wav_file_cmd();

        tight_loop_contents();
        sleep_ms(10);
        count++;
    }

    sleep_ms(1000);  // time to output something to serial

    _led_disp_error(main_error_t::MAIN_LOOP_ABORTED_ERROR, true);

    return 0;
}
