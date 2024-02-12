/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include "wav_file_cmd.h"

queue_t wav_file_cmd::_wav_file_cmd_queue;
// relation from wav_file_cmd_type_t tp status_t
const std::map<const wav_file_cmd::wav_file_cmd_type_t, const wav_file_status::status_t> wav_file_cmd::_cmd_to_req_status = {
    {wav_file_cmd::wav_file_cmd_type_t::PREPARE,  wav_file_status::status_t::REQ_PREPARE},
    {wav_file_cmd::wav_file_cmd_type_t::FINALIZE, wav_file_status::status_t::REQ_FINALIZE}
};
const std::map<const wav_file_cmd::wav_file_cmd_type_t, const wav_file_status::status_t> wav_file_cmd::_cmd_to_done_status = {
    {wav_file_cmd::wav_file_cmd_type_t::PREPARE,  wav_file_status::status_t::PREPARED},
    {wav_file_cmd::wav_file_cmd_type_t::FINALIZE, wav_file_status::status_t::FINALIZED}
};

/*--------------------------/
/  WAV file command class
/--------------------------*/
void wav_file_cmd::initialize()
{
    queue_init(&_wav_file_cmd_queue, sizeof(wav_file_cmd_data_t), WAV_FILE_CMD_QUEUE_LENGTH);
}

bool wav_file_cmd::req_prepare(wav_file_status& wfs, const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample)
{
    wav_file_cmd_data_t cmd_data;
    cmd_data.cmd = wav_file_cmd_type_t::PREPARE;
    cmd_data.wfs = &wfs;
    cmd_data.param[0] = suffix;
    cmd_data.param[1] = sample_freq;
    cmd_data.param[2] = static_cast<uint32_t>(bits_per_sample);
    return _req_wav_file_cmd(cmd_data);
}

bool wav_file_cmd::req_finalize(wav_file_status& wfs, const bool report_flag, const float truncate_sec)
{
    wav_file_cmd_data_t cmd_data;
    cmd_data.cmd = wav_file_cmd_type_t::FINALIZE;
    cmd_data.wfs = &wfs;
    cmd_data.param[0] = static_cast<uint32_t>(report_flag);
    cmd_data.param[1] = *(reinterpret_cast<const uint32_t*>(&truncate_sec));
    cmd_data.param[2] = 0L;
    return _req_wav_file_cmd(cmd_data);
}

void wav_file_cmd::process_wav_file_cmd()
{
    while (!queue_is_empty(&_wav_file_cmd_queue)) {
        wav_file_cmd_data_t cmd_data;
        queue_remove_blocking(&_wav_file_cmd_queue, &cmd_data);
        {  // execute command
            wav_file_cmd* command = _parse(cmd_data);
            command->execute();
            delete command;
        }
        {  // set status as done
            wav_file_status* wfs = cmd_data.wfs;
            const wav_file_status::status_t status = _cmd_to_done_status.at(cmd_data.cmd);
            wfs->set_status(status);
        }
    }
}

wav_file_cmd* wav_file_cmd::_parse(wav_file_cmd_data_t& cmd_data)
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

bool wav_file_cmd::_req_wav_file_cmd(wav_file_cmd_data_t& cmd_data)
{
    if (queue_try_add(&_wav_file_cmd_queue, &cmd_data)) {
        // set status as requested
        wav_file_status* wfs = cmd_data.wfs;
        const wav_file_status::status_t status = _cmd_to_req_status.at(cmd_data.cmd);
        wfs->set_status(status);
        return true;
    } else {
        spdif_rec_wav::report_error(spdif_rec_wav::error_type_t::WAV_FILE_CMD_QUEUE_FULL, static_cast<uint32_t>(cmd_data.cmd));
        return false;
    }
}

wav_file_cmd::wav_file_cmd(wav_file_cmd_data_t& cmd_data) :
    _cmd_data(cmd_data)
{
}

wav_file_cmd::~wav_file_cmd()
{
}

/*--------------------------/
/  nop command
/--------------------------*/
nop::nop(wav_file_cmd_data_t& cmd_data) :
    wav_file_cmd(cmd_data)
{
}

bool nop::execute()
{
    return true;
}

/*--------------------------/
/  prepare command
/--------------------------*/
prepare::prepare(wav_file_cmd_data_t& cmd_data) :
    wav_file_cmd(cmd_data)
{
}

bool prepare::execute()
{
    wav_file_status* wfs = _cmd_data.wfs;
    uint32_t suffix = _cmd_data.param[0];
    uint32_t sample_freq = _cmd_data.param[1];
    bits_per_sample_t bits_per_sample = static_cast<bits_per_sample_t>(_cmd_data.param[2]);
    wfs->prepare(suffix, sample_freq, bits_per_sample);
    return true;
}

/*--------------------------/
/  finalize command
/--------------------------*/
finalize::finalize(wav_file_cmd_data_t& cmd_data) :
    wav_file_cmd(cmd_data)
{
}

bool finalize::execute()
{
    wav_file_status* wfs = _cmd_data.wfs;
    bool report_flag = static_cast<bool>(_cmd_data.param[0]);
    float truncate_sec = *(reinterpret_cast<float*>(&_cmd_data.param[1]));
    wfs->finalize(report_flag, truncate_sec);
    return true;
}
