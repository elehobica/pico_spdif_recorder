/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include "wav_file_cmd.h"

#include <string>
#include <cstring>

#include "spdif_rec_wav.h"
#include "wav_file_status.h"
#include "write_wav.h"

queue_t wav_file_cmd::_wav_file_cmd_queue;
queue_t wav_file_cmd::_wav_file_cmd_reply_queue;

void wav_file_cmd::initialize()
{
    queue_init(&_wav_file_cmd_queue, sizeof(wav_file_cmd_data_t), WAV_FILE_CMD_QUEUE_LENGTH);
    queue_init(&_wav_file_cmd_reply_queue, sizeof(wav_file_cmd_data_t), WAV_FILE_CMD_QUEUE_LENGTH);
}

void wav_file_cmd::req_prepare(wav_file_status& inst_w_sts, const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample)
{
    wav_file_cmd_data_t cmd_data;
    cmd_data.cmd = wav_file_cmd_type_t::PREPARE;
    cmd_data.param[0] = reinterpret_cast<uint32_t>(&inst_w_sts);
    cmd_data.param[1] = suffix;
    cmd_data.param[2] = sample_freq;
    cmd_data.param[3] = static_cast<uint32_t>(bits_per_sample);
    if (queue_try_add(&_wav_file_cmd_queue, &cmd_data)) {
        inst_w_sts.set_status(wav_file_status::wav_file_status_t::REQ_PREPARE);
    } else {
        spdif_rec_wav::_report_error(spdif_rec_wav::error_type_t::WAV_FILE_CMD_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
    }
}

void wav_file_cmd::req_finalize(wav_file_status& inst_w_sts, const bool report_flag, const float truncate_sec)
{
    wav_file_cmd_data_t cmd_data;
    cmd_data.cmd = wav_file_cmd_type_t::FINALIZE;
    cmd_data.param[0] = reinterpret_cast<uint32_t>(&inst_w_sts);
    cmd_data.param[1] = static_cast<uint32_t>(report_flag);
    cmd_data.param[2] = *(reinterpret_cast<const uint32_t*>(&truncate_sec));
    cmd_data.param[3] = 0L;
    if (queue_try_add(&_wav_file_cmd_queue, &cmd_data)) {
        inst_w_sts.set_status(wav_file_status::wav_file_status_t::REQ_FINALIZE);
    } else {
        spdif_rec_wav::_report_error(spdif_rec_wav::error_type_t::WAV_FILE_CMD_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
    }
}

wav_file_cmd* wav_file_cmd::parse(wav_file_cmd_data_t& cmd_data)
{
    wav_file_cmd* inst;
    if (cmd_data.cmd == wav_file_cmd_type_t::PREPARE) {
        inst = new prepare(cmd_data);
    } else if (cmd_data.cmd == wav_file_cmd_type_t::FINALIZE) {
        inst = new finalize(cmd_data);
    } else {
        inst = new nop(cmd_data);
    }
    return inst;
}

void wav_file_cmd::process_wav_file_cmd()
{
    while (!queue_is_empty(&_wav_file_cmd_queue)) {
        wav_file_cmd_data_t cmd_data;
        queue_remove_blocking(&_wav_file_cmd_queue, &cmd_data);
        wav_file_cmd* cmd = parse(cmd_data);
        cmd->execute();
        delete cmd;
    }
}

void wav_file_cmd::process_file_reply_cmd()
{
    while (!queue_is_empty(&_wav_file_cmd_reply_queue)) {
        wav_file_cmd_data_t cmd_data;
        queue_remove_blocking(&_wav_file_cmd_reply_queue, &cmd_data);
        wav_file_status* inst_w_sts = reinterpret_cast<wav_file_status*>(cmd_data.param[0]);
        if (cmd_data.cmd == wav_file_cmd_type_t::PREPARE) {
            inst_w_sts->set_status(wav_file_status::wav_file_status_t::PREPARED);
        } else if (cmd_data.cmd == wav_file_cmd_type_t::FINALIZE) {
            inst_w_sts->set_status(wav_file_status::wav_file_status_t::FINALIZED);
        }
    }
}

bool wav_file_cmd::reply_wav_file_cmd(wav_file_cmd_data_t& cmd_data)
{
    if (!queue_try_add(&_wav_file_cmd_reply_queue, &cmd_data)) {
        spdif_rec_wav::_report_error(spdif_rec_wav::error_type_t::WAV_FILE_CMD_REPLY_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
        return false;
    }
    return true;
}

wav_file_cmd::wav_file_cmd(wav_file_cmd_data_t& cmd_data) :
    _cmd_data(cmd_data)
{
}

wav_file_cmd::~wav_file_cmd()
{
}

nop::nop(wav_file_cmd_data_t& cmd_data) :
    wav_file_cmd(cmd_data)
{
}

bool nop::execute()
{
    return true;
}

prepare::prepare(wav_file_cmd_data_t& cmd_data) :
    wav_file_cmd(cmd_data)
{
}

bool prepare::execute()
{
    wav_file_status* inst_w_sts = reinterpret_cast<wav_file_status*>(_cmd_data.param[0]);
    uint32_t suffix = _cmd_data.param[1];
    uint32_t sample_freq = _cmd_data.param[2];
    bits_per_sample_t bits_per_sample = static_cast<bits_per_sample_t>(_cmd_data.param[3]);
    char wav_filename[16];
    sprintf(wav_filename, "%s%03d.wav", write_wav::WAV_PREFIX, suffix);
    inst_w_sts->prepare(std::string(wav_filename), sample_freq, bits_per_sample);
    //_cmd_data.param[0] = 0L;  // reuse inst_w_sts
    _cmd_data.param[1] = 0L;
    _cmd_data.param[2] = 0L;
    _cmd_data.param[3] = 0L;
    return reply_wav_file_cmd(_cmd_data);
}

finalize::finalize(wav_file_cmd_data_t& cmd_data) :
    wav_file_cmd(cmd_data)
{
}

bool finalize::execute()
{
    wav_file_status* inst_w_sts = reinterpret_cast<wav_file_status*>(_cmd_data.param[0]);
    bool report_flag = static_cast<bool>(_cmd_data.param[1]);
    float truncate_sec = *(reinterpret_cast<float*>(&_cmd_data.param[2]));
    inst_w_sts->finalize(report_flag, truncate_sec);
    //_cmd_data.param[0] = 0L;  // reuse inst_w_sts
    _cmd_data.param[1] = 0L;
    _cmd_data.param[2] = 0L;
    _cmd_data.param[3] = 0L;
    return reply_wav_file_cmd(_cmd_data);
}