/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include "spdif_rec_wav.h"

#include <cstring>
#include <cstdarg>

#include "fatfs/ff.h"
#include "wav_file_status.h"
#include "wav_file_cmd.h"

/*---------------------------------------/
/  Global callback function by spdif_rx
/---------------------------------------*/
void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err)
{
    spdif_rec_wav::_push_sub_frame_buf(buff, sub_frame_count);
}

/*-----------------/
/  Local function
/-----------------*/
static inline uint64_t _micros()
{
    return to_us_since_boot(get_absolute_time());
}

/*-----------------/
/  Class variables
/-----------------*/
const char*    spdif_rec_wav::_suffix_info_filename;
int            spdif_rec_wav::_suffix;
char           spdif_rec_wav::_log_filename[16];
bool           spdif_rec_wav::_clear_log;
uint32_t       spdif_rec_wav::_sub_frame_buf[SPDIF_BLOCK_SIZE * NUM_SUB_FRAME_BUF];
int            spdif_rec_wav::_sub_frame_buf_id = 0;
float          spdif_rec_wav::_blank_sec = 0.0f;
float          spdif_rec_wav::_severe_blank_sec = 0.0f;
float          spdif_rec_wav::_blank_scan_sec = 0.0f;
bool           spdif_rec_wav::_standby_flag = false;
bool           spdif_rec_wav::_recording_flag = false;
bool           spdif_rec_wav:: _blank_split = true;
bool           spdif_rec_wav:: _verbose = false;
queue_t        spdif_rec_wav::_spdif_queue;
queue_t        spdif_rec_wav::_record_cmd_queue;
queue_t        spdif_rec_wav::_error_queue;

/*------------------------/
/  Public class functions
/------------------------*/
void spdif_rec_wav::set_wait_grant_func(void (*func)())
{
    wav_file_status::set_wait_grant_func(func);
}

bool spdif_rec_wav::is_standby()
{
    return _standby_flag;
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
    _suffix = 1;
    _clear_log = true;
}

void spdif_rec_wav::process_wav_file_cmd()
{
    wav_file_cmd::process_wav_file_cmd();
}

void spdif_rec_wav::record_process_loop(const char* log_prefix, const char* suffix_info_filename)
{
    // Initialize class variables
    _suffix_info_filename = suffix_info_filename;
    _standby_flag = false;
    _recording_flag = false;
    _clear_log = true;

    // Local variables for this loop
    FATFS fs;
    uint32_t sample_freq;
    bits_per_sample_t bits_per_sample = bits_per_sample_t::_16BITS;
    wav_file_status prev = wav_file_status(wav_file_status::status_t::FINALIZED);
    wav_file_status cur  = wav_file_status();
    wav_file_status next = wav_file_status();
    uint32_t* buf_ptr = &_sub_frame_buf[SPDIF_BLOCK_SIZE*0];
    int buf_accum = 0;

    // Mount FATFS
    FRESULT fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        printf("FATFS mount error %d\r\n", fr);
        return;
    }
    printf("FATFS mount ok\r\n");

    _suffix = _get_last_suffix() + 1;  // initial suffix to start from 1
    sprintf(_log_filename, "%s%03d.txt", log_prefix, _suffix);

    // Initialize queues
    queue_init(&_spdif_queue, sizeof(sub_frame_buf_info_t), SPDIF_QUEUE_LENGTH);
    queue_init(&_record_cmd_queue, sizeof(record_cmd_data_t), RECORD_CMD_QUEUE_LENGTH);
    queue_init(&_error_queue, sizeof(_error_packet_t), ERROR_QUEUE_LENGTH);
    wav_file_status::initialize();

    printf("spdif_rec_wav process started\r\n");

    while (queue_is_empty(&_record_cmd_queue)) {}

    // Loop
    while (true) {
        if (!queue_is_empty(&_record_cmd_queue)) {
            record_cmd_data_t record_cmd_data;
            queue_remove_blocking(&_record_cmd_queue, &record_cmd_data);
            if (record_cmd_data.cmd != record_cmd_type_t::STANDBY_START_CMD && record_cmd_data.cmd != record_cmd_type_t::START_CMD) continue;

            // initialize variales for a single wav file
            sample_freq = record_cmd_data.param[0];
            bits_per_sample = static_cast<bits_per_sample_t>(record_cmd_data.param[1]);

            if (record_cmd_data.cmd == record_cmd_type_t::STANDBY_START_CMD) {
                _standby_flag = true;
                while (true) {
                    if (!queue_is_empty(&_spdif_queue)) {
                        sub_frame_buf_info_t buf_info;
                        // check blank status
                        queue_peek_blocking(&_spdif_queue, &buf_info);
                        blank_status_t blank_status = _scan_blank(&_sub_frame_buf[SPDIF_BLOCK_SIZE * buf_info.buf_id], buf_info.sub_frame_count, sample_freq);
                        if (blank_status == blank_status_t::NOT_BLANK || blank_status == blank_status_t::BLANK_END_DETECTED) {
                            buf_ptr = &_sub_frame_buf[SPDIF_BLOCK_SIZE * buf_info.buf_id];
                            buf_accum = 0;
                            break;
                        }
                        queue_remove_blocking(&_spdif_queue, &buf_info);
                    }
                    // check cancel of standby
                    if (!queue_is_empty(&_record_cmd_queue)) {
                        queue_remove_blocking(&_record_cmd_queue, &record_cmd_data);
                        if (record_cmd_data.cmd == record_cmd_type_t::START_CMD || record_cmd_data.cmd == record_cmd_type_t::END_CMD || record_cmd_data.cmd == record_cmd_type_t::END_FOR_SPLIT_CMD) {
                            break;
                        }
                    }
                }
                // check cancel of standby
                if (record_cmd_data.cmd == record_cmd_type_t::END_CMD || record_cmd_data.cmd == record_cmd_type_t::END_FOR_SPLIT_CMD) {
                    _standby_flag = false;
                    continue;
                }
            }

            if (next.is_status(wav_file_status::status_t::RESET)) {
                next.req_prepare(_suffix, sample_freq, bits_per_sample);
            }
            next.wait_status(wav_file_status::status_t::PREPARED);
            cur = next;
            next.reset();

            _recording_flag = true;
            _standby_flag = false;
            if (_clear_log) sprintf(_log_filename, "%s%03d.txt", log_prefix, _suffix);
            cur.report_start();
            _clear_log = false;

            if (_verbose) {
                printf("wav bw required:  %7.2f KB/s\r\n", static_cast<float>((static_cast<uint32_t>(bits_per_sample))*sample_freq*2/8) / 1e3);
            }
            _set_last_suffix(_suffix);

            while (true) {
                uint queue_level = queue_get_level(&_spdif_queue);
                cur.record_queue_ratio(static_cast<float>(queue_level) / SPDIF_QUEUE_LENGTH);
                if (queue_level >= NUM_SUB_FRAME_BUF/2) {
                    while (queue_level > 0) {
                        sub_frame_buf_info_t buf_info;
                        // check blank status
                        if (_blank_split) {
                            queue_peek_blocking(&_spdif_queue, &buf_info);
                            blank_status_t blank_status = _scan_blank(&_sub_frame_buf[SPDIF_BLOCK_SIZE * buf_info.buf_id], buf_info.sub_frame_count, sample_freq);
                            if (cur.is_data_written()) {
                                if (blank_status == blank_status_t::BLANK_END_DETECTED) {
                                    split_recording(bits_per_sample);
                                    break;
                                } else if (blank_status == blank_status_t::BLANK_SKIP) {
                                    end_recording(false, BLANK_SKIP_SEC);
                                    start_recording(bits_per_sample, true);  // standby start
                                    break;
                                }
                            }
                        }
                        // get and accumulate buffer
                        queue_remove_blocking(&_spdif_queue, &buf_info);
                        buf_accum++;
                        // write file depending on the conditions
                        if (queue_level == 1 || buf_info.buf_id >= NUM_SUB_FRAME_BUF - 1 || buf_accum >= NUM_SUB_FRAME_BUF/2) {
                            uint32_t sub_frame_count = buf_info.sub_frame_count * buf_accum;
                            cur.write(buf_ptr, sub_frame_count);
                            // update head of buffer to write
                            buf_ptr = &_sub_frame_buf[SPDIF_BLOCK_SIZE * ((buf_info.buf_id + 1) % NUM_SUB_FRAME_BUF)];
                            buf_accum = 0;
                            break;
                        }
                        queue_level = queue_get_level(&_spdif_queue);
                        cur.record_queue_ratio(static_cast<float>(queue_level) / SPDIF_QUEUE_LENGTH);
                    }
                } else if (queue_level < NUM_SUB_FRAME_BUF/4) {
                    if (cur.is_data_written() && next.is_status(wav_file_status::status_t::RESET)) {
                        // prepare next file
                        next.req_prepare(_suffix + 1, sample_freq, bits_per_sample);
                    }
                    wav_file_status::send_core0_grant();
                }

                _process_error();

                if (!queue_is_empty(&_record_cmd_queue)) {
                    queue_remove_blocking(&_record_cmd_queue, &record_cmd_data);
                    if (record_cmd_data.cmd == record_cmd_type_t::END_CMD || record_cmd_data.cmd == record_cmd_type_t::END_FOR_SPLIT_CMD) {
                        break;
                    }
                }
            }

            prev.wait_status(wav_file_status::status_t::FINALIZED);
            prev = cur;
            float truncate_sec = *(reinterpret_cast<float*>(&record_cmd_data.param[0]));
            prev.req_finalize(true, truncate_sec);

            if (record_cmd_data.cmd == record_cmd_type_t::END_CMD) {
                _recording_flag = false;
                prev.wait_status(wav_file_status::status_t::FINALIZED);
                next.req_finalize(false);
                next.wait_status(wav_file_status::status_t::FINALIZED);
                next.reset();

                // drain remained spdif queue to delete samples which should not be included in next wav
                while (!queue_is_empty(&_spdif_queue)) {
                    sub_frame_buf_info_t buf_info;
                    queue_remove_blocking(&_spdif_queue, &buf_info);
                }
                buf_ptr = &_sub_frame_buf[SPDIF_BLOCK_SIZE*0];
                buf_accum = 0;
            }
            _suffix++;
        }
    }
    _process_error();
}

void spdif_rec_wav::start_recording(const bits_per_sample_t bits_per_sample, const bool standby)
{
    record_cmd_data_t cmd_data;
    cmd_data.cmd = standby ? record_cmd_type_t::STANDBY_START_CMD : record_cmd_type_t::START_CMD;
    cmd_data.param[0] = static_cast<uint32_t>(spdif_rx_get_samp_freq());
    cmd_data.param[1] = static_cast<uint32_t>(bits_per_sample);
    if (!queue_try_add(&_record_cmd_queue, &cmd_data)) {
        report_error(error_type_t::RECORD_CMD_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
    }
}

void spdif_rec_wav::end_recording(const bool immediate_split, const float truncate_sec)
{
    record_cmd_data_t cmd_data;
    cmd_data.cmd = immediate_split ? record_cmd_type_t::END_FOR_SPLIT_CMD : record_cmd_type_t::END_CMD;
    cmd_data.param[0] = *(reinterpret_cast<const uint32_t*>(&truncate_sec));
    cmd_data.param[1] = 0L;
    if (!queue_try_add(&_record_cmd_queue, &cmd_data)) {
        report_error(error_type_t::RECORD_CMD_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
    }
}

void spdif_rec_wav::split_recording(const bits_per_sample_t bits_per_sample)
{
    end_recording(true);
    start_recording(bits_per_sample);
}

/*--------------------------/
/  Protected class functions
/--------------------------*/
void spdif_rec_wav::_process_error()
{
    while (!queue_is_empty(&_error_queue)) {
        error_packet_t packet;
        queue_remove_blocking(&_error_queue, &packet);
        switch (packet.type) {
        case error_type_t::ILLEGAL_SUB_FRAME_COUNT:
            log_printf("ERROR: illegal sub_frame_count\r\n");
            break;
        case error_type_t::SPDIF_QUEUE_FULL: {
            int error_count = static_cast<int>(packet.param);
            log_printf("ERROR: _spdif_queue is full x%d\r\n", error_count);
            break;
        }
        case error_type_t::WAV_FILE_CMD_QUEUE_FULL:
            log_printf("ERROR: _wav_file_cmd_queue is full (cmd=%d)\r\n", static_cast<int>(packet.param));
            break;
        case error_type_t::RECORD_CMD_QUEUE_FULL:
            log_printf("ERROR: _record_cmd_queue is full (cmd=%d)\r\n", static_cast<int>(packet.param));
            break;
        case error_type_t::WAV_OPEN_FAIL:
            log_printf("ERROR: wav file open failed\r\n");
            break;
        case error_type_t::WAV_DATA_WRITE_FAIL:
            log_printf("ERROR: wav data write failed\r\n");
            break;
        case error_type_t::WAV_DATA_SYNC_FAIL:
            log_printf("ERROR: wav data sync failed\r\n");
            break;
        case error_type_t::WAV_CLOSE_FAIL:
            log_printf("ERROR: wav file close failed\r\n");
            break;
        case error_type_t::SUFFIX_FILE_FAIL:
            log_printf("ERROR: suffix file failed\r\n");
            break;
        default:
            log_printf("ERROR: unknown error\r\n");
            break;
        }
    }
}

void spdif_rec_wav::report_error(const error_type_t type, const uint32_t param)
{
    error_packet_t packet;
    packet.type = type;
    packet.param = param;
    if (!queue_try_add(&_error_queue, &packet)) {
        printf("ERROR: _error_queue is full\r\n");
    }
}

void spdif_rec_wav::log_printf(const char* fmt, ...)
{
    char buff[512];
    va_list va;

    // variable arg processing
    va_start(va, fmt);
    vsprintf(buff, fmt, va);
    va_end(va);

    // print on serial
    printf(buff);

    // print on log file
    for ( ; ; ) {
        FIL fil;
        FRESULT fr;
        UINT bw;

        // File create mode:
        //  FA_CREATE_ALWAYS: Creates a new file. If the file is existing, it will be truncated and overwritten.
        //  FA_OPEN_ALWAYS: Opens the file if it is existing. If not, a new file will be created.
        //  FA_OPEN_APPEND: Same as FA_OPEN_ALWAYS except the read/write pointer is set end of the file.
        if (_clear_log) {
            fr = f_open(&fil, _log_filename, FA_WRITE | FA_CREATE_ALWAYS);
            if (fr != FR_OK) break;
        } else {
            fr = f_open(&fil, _log_filename, FA_WRITE | FA_OPEN_APPEND);
            if (fr != FR_OK) break;
        }
        fr = f_write(&fil, buff, strnlen(buff, sizeof(buff)), &bw);
        if (fr != FR_OK || bw != strnlen(buff, sizeof(buff))) break;
        fr = f_close(&fil);
        if (fr != FR_OK) break;

        return;
    }

    printf("ERROR: printing on log file failed\r\n");
}

void spdif_rec_wav::_push_sub_frame_buf(const uint32_t* buff, const uint32_t sub_frame_count)
{
    // note that reporting errors through _error_queue is mandatory
    //   because log_printf includes file access and mutex from IRQ routine like this function locks whole system.

    static int error_count = 0;

    if (!_standby_flag && !_recording_flag) return;

    if (sub_frame_count != SPDIF_BLOCK_SIZE) {
        report_error(error_type_t::ILLEGAL_SUB_FRAME_COUNT);
        return;
    }

    memcpy(&_sub_frame_buf[SPDIF_BLOCK_SIZE*_sub_frame_buf_id], buff, sub_frame_count * 4);
    sub_frame_buf_info_t buf_info = {_sub_frame_buf_id, sub_frame_count};
    if (queue_try_add(&_spdif_queue, &buf_info)) {
        _sub_frame_buf_id = (_sub_frame_buf_id + 1) % NUM_SUB_FRAME_BUF;
        if (error_count > 0) {
            report_error(error_type_t::SPDIF_QUEUE_FULL, error_count);
        }
        error_count = 0;
    } else {
        error_count++;
        if (error_count >= 1000) {
            report_error(error_type_t::SPDIF_QUEUE_FULL, error_count);
            error_count -= 1000;
        }
    }
}

int spdif_rec_wav::_get_last_suffix()
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
        fr = f_close(&fil);
        if (fr != FR_OK) break;
        return suffix;
    }
    // no suffix info file or error
    return 0;
}

void spdif_rec_wav::_set_last_suffix(int suffix)
{
    FIL fil;
    FRESULT fr;
    char buff[16];
    UINT bw;

    for ( ; ; ) {
        fr = f_open(&fil, _suffix_info_filename, FA_WRITE | FA_CREATE_ALWAYS);
        if (fr != FR_OK) break;
        sprintf(buff, "%d", suffix);
        fr = f_write(&fil, buff, strnlen(buff, sizeof(buff)), &bw);
        if (fr != FR_OK || bw != strnlen(buff, sizeof(buff))) break;
        fr = f_close(&fil);
        if (fr != FR_OK) break;
        return;
    }

    report_error(error_type_t::SUFFIX_FILE_FAIL);
}

spdif_rec_wav::blank_status_t spdif_rec_wav::_scan_blank(const uint32_t* buff, const uint32_t sub_frame_count, const uint32_t sample_freq)
{
    uint32_t data_accum = 0;
    blank_status_t status = blank_status_t::NOT_BLANK;

    for (int i = 0; i < sub_frame_count; i++) {
        data_accum += std::abs(static_cast<int16_t>(((buff[i] >> 12) & 0xffff)));
    }

    float time_sec = static_cast<float>(sub_frame_count) / NUM_CHANNELS / sample_freq;

    uint32_t ave_level = data_accum / sub_frame_count;
    if (ave_level < SEVERE_BLANK_LEVEL) {
        status = (_severe_blank_sec > BLANK_SKIP_SEC) ? blank_status_t::BLANK_SKIP : blank_status_t::BLANK_DETECTED;
        _blank_sec += time_sec;
        _severe_blank_sec += time_sec;
    } else if (ave_level < BLANK_LEVEL) {
        status = blank_status_t::BLANK_DETECTED;
        _blank_sec += time_sec;
        _severe_blank_sec = 0.0f;
    } else {
        if (_blank_scan_sec >= BLANK_REPEAT_PROHIBIT_SEC && _blank_sec > BLANK_SEC) {
            if (_verbose) printf("detected blank end\r\n");
            status = blank_status_t::BLANK_END_DETECTED;
            _blank_scan_sec = 0.0f;
        }
        _blank_sec = 0.0f;
        _severe_blank_sec = 0.0f;
    }
    _blank_scan_sec += time_sec;

    return status;
}

/*--------------------------/
/  Public Member functions
/--------------------------*/

/*-----------------------------/
/  Protected Member functions
/-----------------------------*/
