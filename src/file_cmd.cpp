/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include "file_cmd.h"

#include <string>
#include <cstring>

#include "spdif_rec_wav.h"
#include "write_wav.h"

queue_t file_cmd::_file_cmd_queue;
queue_t file_cmd::_file_cmd_reply_queue;

void file_cmd::initialize()
{
    queue_init(&_file_cmd_queue, sizeof(file_cmd_data_t), FILE_CMD_QUEUE_LENGTH);
    queue_init(&_file_cmd_reply_queue, sizeof(file_cmd_data_t), FILE_CMD_QUEUE_LENGTH);
}

void file_cmd::req_prepare_file(spdif_rec_wav& inst_w_sts, const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample)
{
    file_cmd_data_t cmd_data;
    cmd_data.cmd = file_cmd_type_t::PREPARE;
    cmd_data.param[0] = reinterpret_cast<uint32_t>(&inst_w_sts);
    cmd_data.param[1] = suffix;
    cmd_data.param[2] = sample_freq;
    cmd_data.param[3] = static_cast<uint32_t>(bits_per_sample);
    if (queue_try_add(&_file_cmd_queue, &cmd_data)) {
        inst_w_sts.set_status(spdif_rec_wav::wav_status_t::REQ_PREPARE);
    } else {
        spdif_rec_wav::_report_error(spdif_rec_wav::error_type_t::FILE_CMD_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
    }
}

void file_cmd::req_finalize_file(spdif_rec_wav& inst_w_sts, const bool report_flag, const float truncate_sec)
{
    file_cmd_data_t cmd_data;
    cmd_data.cmd = file_cmd_type_t::FINALIZE;
    cmd_data.param[0] = reinterpret_cast<uint32_t>(&inst_w_sts);
    cmd_data.param[1] = static_cast<uint32_t>(report_flag);
    cmd_data.param[2] = *(reinterpret_cast<const uint32_t*>(&truncate_sec));
    cmd_data.param[3] = 0L;
    if (queue_try_add(&_file_cmd_queue, &cmd_data)) {
        inst_w_sts.set_status(spdif_rec_wav::wav_status_t::REQ_FINALIZE);
    } else {
        spdif_rec_wav::_report_error(spdif_rec_wav::error_type_t::FILE_CMD_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
    }
}

file_cmd* file_cmd::parse(file_cmd_data_t& cmd_data)
{
    file_cmd* inst;
    if (cmd_data.cmd == file_cmd_type_t::PREPARE) {
        inst = new prepare(cmd_data);
    } else if (cmd_data.cmd == file_cmd_type_t::FINALIZE) {
        inst = new finalize(cmd_data);
    } else {
        inst = new nop(cmd_data);
    }
    return inst;
}

void file_cmd::process_file_cmd()
{
    while (!queue_is_empty(&_file_cmd_queue)) {
        file_cmd_data_t cmd_data;
        queue_remove_blocking(&_file_cmd_queue, &cmd_data);
        file_cmd* cmd = parse(cmd_data);
        cmd->execute();
        delete cmd;
    }
}

void file_cmd::process_file_reply_cmd()
{
    while (!queue_is_empty(&_file_cmd_reply_queue)) {
        file_cmd_data_t cmd_data;
        queue_remove_blocking(&_file_cmd_reply_queue, &cmd_data);
        spdif_rec_wav* inst_w_sts = reinterpret_cast<spdif_rec_wav*>(cmd_data.param[0]);
        if (cmd_data.cmd == file_cmd_type_t::PREPARE) {
            inst_w_sts->set_status(spdif_rec_wav::wav_status_t::PREPARED);
        } else if (cmd_data.cmd == file_cmd_type_t::FINALIZE) {
            inst_w_sts->set_status(spdif_rec_wav::wav_status_t::FINALIZED);
        }
    }
}

bool file_cmd::reply_file_cmd(file_cmd_data_t& cmd_data)
{
    if (!queue_try_add(&_file_cmd_reply_queue, &cmd_data)) {
        spdif_rec_wav::_report_error(spdif_rec_wav::error_type_t::FILE_CMD_REPLY_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
        return false;
    }
    return true;
}

file_cmd::file_cmd(file_cmd_data_t& cmd_data) :
    _cmd_data(cmd_data)
{
}

file_cmd::~file_cmd()
{
}

nop::nop(file_cmd_data_t& cmd_data) :
    file_cmd(cmd_data)
{
}

bool nop::execute()
{
    return true;
}

prepare::prepare(file_cmd_data_t& cmd_data) :
    file_cmd(cmd_data)
{
}

bool prepare::execute()
{
    spdif_rec_wav* inst_w_sts = reinterpret_cast<spdif_rec_wav*>(_cmd_data.param[0]);
    uint32_t suffix = _cmd_data.param[1];
    uint32_t sample_freq = _cmd_data.param[2];
    bits_per_sample_t bits_per_sample = static_cast<bits_per_sample_t>(_cmd_data.param[3]);
    char wav_filename[16];
    sprintf(wav_filename, "%s%03d.wav", write_wav::WAV_PREFIX, suffix);
    inst_w_sts->inst = new write_wav(std::string(wav_filename), sample_freq, bits_per_sample);
    //_cmd_data.param[0] = 0L;  // reuse inst_w_sts
    _cmd_data.param[1] = 0L;
    _cmd_data.param[2] = 0L;
    _cmd_data.param[3] = 0L;
    return reply_file_cmd(_cmd_data);
}

finalize::finalize(file_cmd_data_t& cmd_data) :
    file_cmd(cmd_data)
{
}

bool finalize::execute()
{
    spdif_rec_wav* inst_w_sts = reinterpret_cast<spdif_rec_wav*>(_cmd_data.param[0]);
    bool report_flag = static_cast<bool>(_cmd_data.param[1]);
    float truncate_sec = *(reinterpret_cast<float*>(&_cmd_data.param[2]));
    if (truncate_sec > 0.0f) inst_w_sts->inst->_set_truncate(truncate_sec);
    if (report_flag) inst_w_sts->inst->_report_final();
    delete inst_w_sts->inst;
    //_cmd_data.param[0] = 0L;  // reuse inst_w_sts
    _cmd_data.param[1] = 0L;
    _cmd_data.param[2] = 0L;
    _cmd_data.param[3] = 0L;
    return reply_file_cmd(_cmd_data);
}