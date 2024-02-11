/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include "pico/util/queue.h"

#include "spdif_rec_wav.h"

class wav_file_status;

/*--------------------------/
/  WAV file command class
/--------------------------*/
class wav_file_cmd
{
protected:
    enum class wav_file_cmd_type_t {
        PREPARE = 0,
        FINALIZE
    };
    typedef struct _wav_file_cmd_data_t {
        wav_file_cmd_type_t cmd;
        uint32_t   param[4];
    } wav_file_cmd_data_t;

    static constexpr int WAV_FILE_CMD_QUEUE_LENGTH = 2;

    static queue_t _wav_file_cmd_queue;
    static queue_t _wav_file_cmd_reply_queue;

public:
    static void initialize();
    static void req_prepare(wav_file_status& wfs, const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample);
    static void req_finalize(wav_file_status& wfs, const bool report_final, const float truncate_sec = 0.0f);
    static wav_file_cmd* parse(wav_file_cmd_data_t& cmd_data);
    static void process_wav_file_cmd();
    static void process_file_reply_cmd();

protected:
    static bool _reply_wav_file_cmd(wav_file_cmd_data_t& cmd_data);

public:
    wav_file_cmd(wav_file_cmd_data_t& cmd_data);
    virtual ~wav_file_cmd();
    virtual bool execute() = 0;

    wav_file_cmd_data_t _cmd_data;
};

/*--------------------------/
/  nop command
/--------------------------*/
class nop : public wav_file_cmd
{
public:
    nop(wav_file_cmd_data_t& cmd_data);
    bool execute();
};

/*--------------------------/
/  prepare command
/--------------------------*/
class prepare : public wav_file_cmd
{
public:
    prepare(wav_file_cmd_data_t& cmd_data);
    bool execute();
};

/*--------------------------/
/  finalize command
/--------------------------*/
class finalize : public wav_file_cmd
{
public:
    finalize(wav_file_cmd_data_t& cmd_data);
    bool execute();
};