/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdarg>
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
    spdif_rec_wav::_push_sub_frame_buf(buff, sub_frame_count);
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
float          spdif_rec_wav::_blank_scan_sec = 0.0f;
uint32_t       spdif_rec_wav::_wav_buf[SPDIF_BLOCK_SIZE*3/4 * NUM_SUB_FRAME_BUF / 2];
bool           spdif_rec_wav::_standby_flag = false;
bool           spdif_rec_wav::_recording_flag = false;
bool           spdif_rec_wav:: _blank_split = true;
bool           spdif_rec_wav:: _verbose = false;
queue_t        spdif_rec_wav::_spdif_queue;
queue_t        spdif_rec_wav::_file_cmd_queue;
queue_t        spdif_rec_wav::_file_cmd_reply_queue;
queue_t        spdif_rec_wav::_record_cmd_queue;
queue_t        spdif_rec_wav::_error_queue;
queue_t        spdif_rec_wav::_core0_grant_queue;

/*------------------------/
/  Public class functions
/------------------------*/
void spdif_rec_wav::process_file_cmd()
{
    while (!queue_is_empty(&_file_cmd_queue)) {
        file_cmd_data_t cmd_data;
        queue_remove_blocking(&_file_cmd_queue, &cmd_data);
        inst_with_status_t* inst_w_sts = reinterpret_cast<inst_with_status_t*>(cmd_data.param[0]);
        if (cmd_data.cmd == file_cmd_type_t::PREPARE) {
            uint32_t suffix = cmd_data.param[1];
            uint32_t sample_freq = cmd_data.param[2];
            bits_per_sample_t bits_per_sample = static_cast<bits_per_sample_t>(cmd_data.param[3]);
            char wav_filename[16];
            sprintf(wav_filename, "%s%03d.wav", WAV_PREFIX, suffix);
            inst_w_sts->inst = new spdif_rec_wav(std::string(wav_filename), sample_freq, bits_per_sample);
            //cmd_data.param[0] = 0L;  // reuse inst_w_sts
            cmd_data.param[1] = 0L;
            cmd_data.param[2] = 0L;
            cmd_data.param[3] = 0L;
            if (!queue_try_add(&_file_cmd_reply_queue, &cmd_data)) {
                _report_error(error_type_t::FILE_CMD_REPLY_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
            }
        } else if (cmd_data.cmd == file_cmd_type_t::FINALIZE) {
            bool report_flag = static_cast<bool>(cmd_data.param[1]);
            if (report_flag) inst_w_sts->inst->_report_final();
            delete inst_w_sts->inst;
            //cmd_data.param[0] = 0L;  // reuse inst_w_sts
            cmd_data.param[1] = 0L;
            cmd_data.param[2] = 0L;
            cmd_data.param[3] = 0L;
            if (!queue_try_add(&_file_cmd_reply_queue, &cmd_data)) {
                _report_error(error_type_t::FILE_CMD_REPLY_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
            }
        }
    }
}

void spdif_rec_wav::blocking_wait_core0_grant()
{
    while (queue_is_empty(&_core0_grant_queue)) {}
    drain_core0_grant();
}

void spdif_rec_wav::drain_core0_grant()
{
    while (!queue_is_empty(&_core0_grant_queue)) {
        bool flag;
        queue_remove_blocking(&_core0_grant_queue, &flag);
    }
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
    inst_with_status_t prev = inst_with_status_t(inst_status_t::FINALIZED);
    inst_with_status_t cur  = inst_with_status_t();
    inst_with_status_t next = inst_with_status_t();
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
    queue_init(&_file_cmd_queue, sizeof(file_cmd_data_t), FILE_CMD_QUEUE_LENGTH);
    queue_init(&_file_cmd_reply_queue, sizeof(file_cmd_data_t), FILE_CMD_QUEUE_LENGTH);
    queue_init(&_record_cmd_queue, sizeof(record_cmd_data_t), RECORD_CMD_QUEUE_LENGTH);
    queue_init(&_error_queue, sizeof(_error_packet_t), ERROR_QUEUE_LENGTH);
    queue_init(&_core0_grant_queue, sizeof(bool), CORE0_GRANT_QUEUE_LENGTH);

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
                            _blank_scan_sec = 0.0f;
                            buf_ptr = &_sub_frame_buf[SPDIF_BLOCK_SIZE * buf_info.buf_id];
                            buf_accum = 0;
                            break;
                        }
                        queue_remove_blocking(&_spdif_queue, &buf_info);
                    }
                    // check cancel of standby
                    if (!queue_is_empty(&_record_cmd_queue)) {
                        queue_remove_blocking(&_record_cmd_queue, &record_cmd_data);
                        if (record_cmd_data.cmd == record_cmd_type_t::START_CMD || record_cmd_data.cmd == record_cmd_type_t::END_CMD || record_cmd_data.cmd == record_cmd_type_t::END_FOR_SPLIT_CMD) break;
                    }
                }
                // check cancel of standby
                if (record_cmd_data.cmd == record_cmd_type_t::END_CMD || record_cmd_data.cmd == record_cmd_type_t::END_FOR_SPLIT_CMD) {
                    _standby_flag = false;
                    continue;
                }
            }

            if (next.status == inst_status_t::RESET) {
                _req_prepare_file(next, _suffix, sample_freq, bits_per_sample);
            }
            while (next.status != inst_status_t::PREPARED) {
                _send_core0_grant();
                _process_file_reply_cmd();
            }
            cur = next;
            next.reset();
            // prepare next file
            _req_prepare_file(next, _suffix + 1, sample_freq, bits_per_sample);

            _recording_flag = true;
            _standby_flag = false;
            if (_clear_log) sprintf(_log_filename, "%s%03d.txt", log_prefix, _suffix);
            cur.inst->_report_start();
            _clear_log = false;

            if (_verbose) {
                printf("wav bw required:  %7.2f KB/s\r\n", (float) (static_cast<uint32_t>(bits_per_sample)*sample_freq*2/8) / 1e3);
            }
            _set_last_suffix(_suffix);

            while (true) {
                uint queue_level = queue_get_level(&_spdif_queue);
                cur.inst->_record_queue_level(queue_level);
                if (queue_level >= NUM_SUB_FRAME_BUF/2) {
                    while (queue_level > 0) {
                        sub_frame_buf_info_t buf_info;
                        // check blank status
                        if (_blank_split) {
                            queue_peek_blocking(&_spdif_queue, &buf_info);
                            blank_status_t blank_status = _scan_blank(&_sub_frame_buf[SPDIF_BLOCK_SIZE * buf_info.buf_id], buf_info.sub_frame_count, sample_freq);
                            if (blank_status == blank_status_t::BLANK_END_DETECTED) {
                                split_recording(bits_per_sample);
                                break;
                            } else if (blank_status == blank_status_t::BLANK_SKIP) {
                                end_recording();
                                start_recording(bits_per_sample, true);  // standby start
                                break;
                            }
                        }
                        // get and accumulate buffer
                        queue_remove_blocking(&_spdif_queue, &buf_info);
                        buf_accum++;
                        // write file depending on the conditions
                        if (queue_level == 1 || buf_info.buf_id >= NUM_SUB_FRAME_BUF - 1 || buf_accum >= NUM_SUB_FRAME_BUF/2) {
                            uint32_t sub_frame_count = buf_info.sub_frame_count * buf_accum;
                            cur.inst->_write(buf_ptr, sub_frame_count);
                            // update head of buffer to write
                            buf_ptr = &_sub_frame_buf[SPDIF_BLOCK_SIZE * ((buf_info.buf_id + 1) % NUM_SUB_FRAME_BUF)];
                            buf_accum = 0;
                            break;
                        }
                        queue_level = queue_get_level(&_spdif_queue);
                        cur.inst->_record_queue_level(queue_level);
                    }
                } else {
                    _send_core0_grant();
                }

                _process_file_reply_cmd();
                _process_error();

                if (!queue_is_empty(&_record_cmd_queue)) {
                    queue_remove_blocking(&_record_cmd_queue, &record_cmd_data);
                    if (record_cmd_data.cmd == record_cmd_type_t::END_CMD || record_cmd_data.cmd == record_cmd_type_t::END_FOR_SPLIT_CMD) break;
                }
            }

            while (prev.status != inst_status_t::FINALIZED) {
                _send_core0_grant();
                _process_file_reply_cmd();
            }
            prev = cur;
            _req_finalize_file(prev);

            if (record_cmd_data.cmd == record_cmd_type_t::END_CMD) {
                _recording_flag = false;
                while (prev.status != inst_status_t::FINALIZED) {
                    _send_core0_grant();
                    _process_file_reply_cmd();
                }
                _req_finalize_file(next, false);
                while (next.status != inst_status_t::FINALIZED) {
                    _send_core0_grant();
                    _process_file_reply_cmd();
                }
                next.reset();

                sleep_ms(1);  // wait for buffer push to stop
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
        _report_error(error_type_t::RECORD_CMD_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
    }
}

void spdif_rec_wav::end_recording(const bool split)
{
    record_cmd_data_t cmd_data;
    cmd_data.cmd = split ? record_cmd_type_t::END_FOR_SPLIT_CMD : record_cmd_type_t::END_CMD;
    cmd_data.param[0] = 0L;
    cmd_data.param[1] = 0L;
    if (!queue_try_add(&_record_cmd_queue, &cmd_data)) {
        _report_error(error_type_t::RECORD_CMD_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
    }
}

void spdif_rec_wav::split_recording(const bits_per_sample_t bits_per_sample)
{
    end_recording(true);
    start_recording(bits_per_sample);
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

/*--------------------------/
/  Protected class functions
/--------------------------*/
void spdif_rec_wav::_process_file_reply_cmd()
{
    while (!queue_is_empty(&_file_cmd_reply_queue)) {
        file_cmd_data_t cmd_data;
        queue_remove_blocking(&_file_cmd_reply_queue, &cmd_data);
        inst_with_status_t* inst_w_sts = reinterpret_cast<inst_with_status_t*>(cmd_data.param[0]);
        if (cmd_data.cmd == file_cmd_type_t::PREPARE) {
            inst_w_sts->status = inst_status_t::PREPARED;
        } else if (cmd_data.cmd == file_cmd_type_t::FINALIZE) {
            inst_w_sts->status = inst_status_t::FINALIZED;
        }
    }
}

void spdif_rec_wav::_process_error()
{
    while (!queue_is_empty(&_error_queue)) {
        error_packet_t packet;
        queue_remove_blocking(&_error_queue, &packet);
        switch (packet.type) {
        case error_type_t::ILLEGAL_SUB_FRAME_COUNT:
            _log_printf("ERROR: illegal sub_frame_count\r\n");
            break;
        case error_type_t::SPDIF_QUEUE_FULL: {
            int error_count = static_cast<int>(packet.param);
            _log_printf("ERROR: _spdif_queue is full x%d\r\n", error_count);
            break;
        }
        case error_type_t::FILE_CMD_QUEUE_FULL:
            _log_printf("ERROR: _file_cmd_queue is full (cmd=%d)\r\n", (int) packet.param);
            break;
        case error_type_t::FILE_CMD_REPLY_QUEUE_FULL:
            _log_printf("ERROR: _file_cmd_reply_queue is full (cmd=%d)\r\n", (int) packet.param);
            break;
        case error_type_t::RECORD_CMD_QUEUE_FULL:
            _log_printf("ERROR: _record_cmd_queue is full (cmd=%d)\r\n", (int) packet.param);
            break;
        default:
            _log_printf("ERROR: unknown error\r\n");
            break;
        }
    }
}

void spdif_rec_wav::_req_prepare_file(inst_with_status_t& inst_w_sts, const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample)
{
    file_cmd_data_t cmd_data;
    cmd_data.cmd = file_cmd_type_t::PREPARE;
    cmd_data.param[0] = reinterpret_cast<uint32_t>(&inst_w_sts);
    cmd_data.param[1] = suffix;
    cmd_data.param[2] = sample_freq;
    cmd_data.param[3] = static_cast<uint32_t>(bits_per_sample);
    if (queue_try_add(&_file_cmd_queue, &cmd_data)) {
        inst_w_sts.status = inst_status_t::REQ_PREPARE;
    } else {
        _report_error(error_type_t::FILE_CMD_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
    }
}

void spdif_rec_wav::_req_finalize_file(inst_with_status_t& inst_w_sts, const bool report_flag)
{
    file_cmd_data_t cmd_data;
    cmd_data.cmd = file_cmd_type_t::FINALIZE;
    cmd_data.param[0] = reinterpret_cast<uint32_t>(&inst_w_sts);
    cmd_data.param[1] = static_cast<uint32_t>(report_flag);
    cmd_data.param[2] = 0L;
    cmd_data.param[3] = 0L;
    if (queue_try_add(&_file_cmd_queue, &cmd_data)) {
        inst_w_sts.status = inst_status_t::REQ_FINALIZE;
    } else {
        _report_error(error_type_t::FILE_CMD_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
    }
}

void spdif_rec_wav::_report_error(const error_type_t type, const uint32_t param)
{
    error_packet_t packet;
    packet.type = type;
    packet.param = param;
    if (!queue_try_add(&_error_queue, &packet)) {
        printf("ERROR: _error_queue is full\r\n");
    }
}

void spdif_rec_wav::_send_core0_grant()
{
    bool flag = true;
    queue_try_add(&_core0_grant_queue, &flag);
}

void spdif_rec_wav::_push_sub_frame_buf(const uint32_t* buff, const uint32_t sub_frame_count)
{
    // note that reporting errors through _error_queue is mandatory
    //   because _log_printf includes file access and mutex from IRQ routine like this function locks whole system.

    static int error_count = 0;

    if (!_standby_flag && !_recording_flag) return;

    if (sub_frame_count != SPDIF_BLOCK_SIZE) {
        _report_error(error_type_t::ILLEGAL_SUB_FRAME_COUNT);
        return;
    }

    memcpy(&_sub_frame_buf[SPDIF_BLOCK_SIZE*_sub_frame_buf_id], buff, sub_frame_count * 4);
    sub_frame_buf_info_t buf_info = {_sub_frame_buf_id, sub_frame_count};
    if (queue_try_add(&_spdif_queue, &buf_info)) {
        _sub_frame_buf_id = (_sub_frame_buf_id + 1) % NUM_SUB_FRAME_BUF;
        if (error_count > 0) {
            _report_error(error_type_t::SPDIF_QUEUE_FULL, error_count);
        }
        error_count = 0;
    } else {
        error_count++;
        if (error_count >= 1000) {
            _report_error(error_type_t::SPDIF_QUEUE_FULL, error_count);
            error_count -= 1000;
        }
    }
}

void spdif_rec_wav::_log_printf(const char* fmt, ...)
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
    // error
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
    // error
}

spdif_rec_wav::blank_status_t spdif_rec_wav::_scan_blank(const uint32_t* buff, const uint32_t sub_frame_count, const uint32_t sample_freq)
{
    uint32_t data_accum = 0;
    blank_status_t status = blank_status_t::NOT_BLANK;

    for (int i = 0; i < sub_frame_count; i++) {
        data_accum += std::abs(static_cast<int16_t>(((buff[i] >> 12) & 0xffff)));
    }

    float time_sec = (float) sub_frame_count / NUM_CHANNELS / sample_freq;

    uint32_t ave_level = data_accum / sub_frame_count;
    if (ave_level < BLANK_LEVEL) {
        status = (_blank_sec > BLANK_SKIP_SEC) ? blank_status_t::BLANK_SKIP : blank_status_t::BLANK_DETECTED;
        _blank_sec += time_sec;
    } else {
        if (_blank_scan_sec >= BLANK_REPEAT_PROHIBIT_SEC && _blank_sec > BLANK_SEC) {
            if (_verbose) printf("detected blank end\r\n");
            status = blank_status_t::BLANK_END_DETECTED;
        }
        _blank_sec = 0.0f;
    }
    _blank_scan_sec += time_sec;

    return status;
}

/*-----------------/
/  Constructor
/-----------------*/
spdif_rec_wav::spdif_rec_wav(const std::string filename, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample) :
    _fil(),
    _filename(filename),
    _sample_freq(sample_freq),
    _bits_per_sample(bits_per_sample),
    _total_sample_count(0),
    _total_bytes(0),
    _total_time_us(0),
    _best_bandwidth(-INFINITY),
    _worst_bandwidth(INFINITY),
    _queue_worst(0)
{
    drain_core0_grant();
    for ( ; ; ) {
        FRESULT fr;     /* FatFs return code */
        UINT bw;
        uint8_t buf[WAV_HEADER_SIZE];
        uint16_t u16;
        uint32_t u32;

        blocking_wait_core0_grant();
        fr = f_open(&_fil, _filename.c_str(), FA_WRITE | FA_CREATE_ALWAYS);

        blocking_wait_core0_grant();
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
        u32 = _sample_freq * NUM_CHANNELS * (static_cast<uint16_t>(_bits_per_sample)/8);
        memcpy(&buf[28], (const void *) &u32, 4);
        // BlockAlign
        u16 = NUM_CHANNELS * (static_cast<uint16_t>(_bits_per_sample)/8);
        memcpy(&buf[32], (const void *) &u16, 2);
        // BitsPerSample
        u16 = static_cast<uint16_t>(_bits_per_sample);
        memcpy(&buf[34], (const void *) &u16, 2);

        // Subchunk2ID
        memcpy(&buf[36], "data", 4);
        // Subchunk2Size (temporary 0)
        memset(&buf[40], 0, 4);

        blocking_wait_core0_grant();
        fr = f_write(&_fil, buf, sizeof(buf), &bw);
        if (fr != FR_OK || bw != sizeof(buf)) break;

        drain_core0_grant();
        return;
    }
    // error
}

/*-----------------/
/  Destructor
/-----------------*/
spdif_rec_wav::~spdif_rec_wav()
{
    drain_core0_grant();
    for ( ; ; ) {
        FRESULT fr;     /* FatFs return code */

        if (_total_bytes == 0) {
            // remove file if no samples
            blocking_wait_core0_grant();
            fr = f_close(&_fil);
            if (fr != FR_OK) break;
            blocking_wait_core0_grant();
            fr = f_unlink(_filename.c_str());
            if (fr != FR_OK) break;
        } else {
            UINT bw;
            uint32_t u32;
            blocking_wait_core0_grant();
            DWORD cur_pos = f_tell(&_fil);

            // ChunkSize
            blocking_wait_core0_grant();
            fr = _gradual_seek(4);
            if (fr != FR_OK) break;
            u32 = _total_bytes + (WAV_HEADER_SIZE - 8);
            blocking_wait_core0_grant();
            fr = f_write(&_fil, static_cast<const void *>(&u32), sizeof(uint32_t), &bw);
            if (fr != FR_OK || bw != sizeof(uint32_t)) break;

            // Subchunk2Size
            blocking_wait_core0_grant();
            fr = _gradual_seek(40);
            if (fr != FR_OK) break;
            u32 = _total_bytes;
            blocking_wait_core0_grant();
            fr = f_write(&_fil, static_cast<const void *>(&u32), sizeof(uint32_t), &bw);
            if (fr != FR_OK || bw != sizeof(uint32_t)) break;

            blocking_wait_core0_grant();
            fr = _gradual_seek(cur_pos);
            if (fr != FR_OK) break;
            blocking_wait_core0_grant();
            fr = f_close(&_fil);
            if (fr != FR_OK) break;
        }

        drain_core0_grant();
        return;
    }
    // error
}

/*--------------------------/
/  Public Member functions
/--------------------------*/

/*-----------------------------/
/  Protected Member functions
/-----------------------------*/
FRESULT spdif_rec_wav::_gradual_seek(DWORD target_pos)
{
    FRESULT fr;     /* FatFs return code */

    DWORD cur_pos = f_tell(&_fil);
    int64_t diff = (int64_t) target_pos - cur_pos;

    while (cur_pos != target_pos) {
        if (diff >= 0) {
            if (cur_pos + SEEK_STEP_BYTES < target_pos) {
                cur_pos += SEEK_STEP_BYTES;
            } else {
                cur_pos = target_pos;
            }
        } else {
            if (cur_pos > target_pos + SEEK_STEP_BYTES) {
                cur_pos -= SEEK_STEP_BYTES;
            } else {
                cur_pos = target_pos;
            }
        }
        blocking_wait_core0_grant();
        fr = f_lseek(&_fil, cur_pos);
        if (fr != FR_OK) break;
    }
    return fr;
}

uint32_t spdif_rec_wav::_write_core(const uint32_t* buff, const uint32_t sub_frame_count)
{
    FRESULT fr;     /* FatFs return code */
    UINT bw;

    if (_bits_per_sample == bits_per_sample_t::_16BITS) {
        for (int i = 0; i < sub_frame_count; i++) {
            _wav_buf[i/2] >>= 16;
            _wav_buf[i/2] |= ((buff[i] >> 12) & 0xffff) << 16;
        }
        fr = f_write(&_fil, static_cast<const void *>(_wav_buf), sub_frame_count*2, &bw);
        if (fr != FR_OK || bw != sub_frame_count*2) printf("error 4\r\n");
    } else if (_bits_per_sample == bits_per_sample_t::_24BITS) {
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

uint32_t spdif_rec_wav::_write(const uint32_t* buff, const uint32_t sub_frame_count)
{
    uint64_t start_time = _micros();
    uint32_t bytes = _write_core(buff, sub_frame_count);
    uint32_t t_us = static_cast<uint32_t>(_micros() - start_time);
    float bandwidth = static_cast<float>(bytes) / t_us * 1e3;
    if (bandwidth > _best_bandwidth) _best_bandwidth = bandwidth;
    if (bandwidth < _worst_bandwidth) {
        _worst_bandwidth = bandwidth;
        if (_verbose) {
            printf("worst bandwidth updated: %7.2f KB/s\r\n", _worst_bandwidth);
        }
    }
    _total_sample_count += sub_frame_count;
    _total_bytes += bytes;
    _total_time_us += t_us;

    return bytes;
}

void spdif_rec_wav::_record_queue_level(uint queue_level)
{
    if (queue_level > _queue_worst) _queue_worst = queue_level;
}

void spdif_rec_wav::_report_start()
{
    _log_printf("recording start \"%s\" @ %d bits %5.1f KHz (bitrate: %6.1f Kbps)\r\n", _filename.c_str(), _bits_per_sample, (float) _sample_freq*1e-3, (float) _bits_per_sample*_sample_freq*2*1e-3);
}

void spdif_rec_wav::_report_final()
{
    uint32_t total_sec = _total_bytes / (static_cast<uint32_t>(_bits_per_sample)/8) / NUM_CHANNELS / _sample_freq;
    uint32_t total_sec_dp = static_cast<uint64_t>(_total_bytes) / (static_cast<uint32_t>(_bits_per_sample)/8) / NUM_CHANNELS * 1000 / _sample_freq - total_sec*1000;
    _log_printf("recording done \"%s\" %lu bytes (time:  %d:%02d.%03d)\r\n", _filename.c_str(), _total_bytes + WAV_HEADER_SIZE, total_sec/60, total_sec%60, total_sec_dp);
    if (_verbose) {
        float avg_bw = (float) _total_bytes / _total_time_us * 1e3;
        printf("SD Card writing bandwidth\r\n");
        printf(" avg:   %7.2f KB/s\r\n", avg_bw);
        printf(" best:  %7.2f KB/s\r\n", _best_bandwidth);
        printf(" worst: %7.2f KB/s\r\n", _worst_bandwidth);
        printf("WAV file required bandwidth\r\n");
        printf(" wav:   %7.2f KB/s\r\n", (float) (static_cast<uint32_t>(_bits_per_sample)*_sample_freq*2/8) / 1e3);
        printf("spdif queue usage: %7.2f %%\r\n", (float) _queue_worst / SPDIF_QUEUE_LENGTH * 100);
    }
}
