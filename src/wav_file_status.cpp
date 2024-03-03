/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include "wav_file_status.h"

#include "wav_file.h"
#include "wav_file_cmd.h"

void (*wav_file_status::_wait_grant_func)() = nullptr;
queue_t wav_file_status::_core0_grant_queue;

void wav_file_status::set_wait_grant_func(void (*func)())
{
    _wait_grant_func = func;
}

void wav_file_status::blocking_wait_core0_grant()
{
    drain_core0_grant();
    while (queue_is_empty(&_core0_grant_queue)) {
        if (_wait_grant_func != nullptr) {
            (*_wait_grant_func)();
        }
    }
}

void wav_file_status::drain_core0_grant()
{
    // don't use while (!queue_is_empty) loop because it could drain grants which come after
    uint queue_level = queue_get_level(&_core0_grant_queue);
    for (uint i = 0; i < queue_level; i++) {
        bool flag;
        queue_remove_blocking(&_core0_grant_queue, &flag);
    }
}

void wav_file_status::send_core0_grant()
{
    bool flag = true;
    queue_try_add(&_core0_grant_queue, &flag);
}

void wav_file_status::initialize()
{
    queue_init(&_core0_grant_queue, sizeof(bool), CORE0_GRANT_QUEUE_LENGTH);
    wav_file_cmd::initialize();
}

wav_file_status::wav_file_status(status_t status) :
    _wav_file(nullptr), _status(status)
{
}

wav_file_status::~wav_file_status()
{
}

void wav_file_status::prepare(const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample)
{
    _wav_file = new wav_file(suffix, sample_freq, bits_per_sample);
}

void wav_file_status::finalize(const bool report_flag, const float truncate_sec)
{
    if (truncate_sec > 0.0f) _wav_file->set_truncate(truncate_sec);
    if (report_flag) _wav_file->report_final();
    delete _wav_file;
    _wav_file = nullptr;
}

void wav_file_status::set_status(status_t status)
{
    _status = status;
}

bool wav_file_status::is_equal_status(status_t status) const
{
    return _status == status;
}

void wav_file_status::wait_status(status_t status)
{
    while (_status != status) {
        send_core0_grant();
    }
}

void wav_file_status::reset()
{
    _wav_file = nullptr;
    _status = status_t::RESET;
}

void wav_file_status::req_prepare(const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample)
{
    wav_file_cmd::req_prepare(*this, suffix, sample_freq, bits_per_sample);
}

void wav_file_status::req_finalize(const bool report_flag, const float truncate_sec)
{
    wav_file_cmd::req_finalize(*this, report_flag, truncate_sec);
}

uint32_t wav_file_status::write(const uint32_t* buff, const uint32_t sub_frame_count)
{
    return _wav_file->write(buff, sub_frame_count);
}

void wav_file_status::record_queue_ratio(float queue_ratio)
{
    _wav_file->record_queue_ratio(queue_ratio);
}

void wav_file_status::report_start()
{
    _wav_file->report_start();
}

void wav_file_status::report_final()
{
    _wav_file->report_final();
}

bool wav_file_status::is_data_written()
{
    return _wav_file->is_data_written();
}
