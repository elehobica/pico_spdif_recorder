/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include "pico/util/queue.h"
#include "write_wav.h"

class spdif_rec_wav;

class file_cmd
{
protected:
    enum class file_cmd_type_t {
        PREPARE = 0,
        FINALIZE
    };
    typedef struct _file_cmd_data_t {
        file_cmd_type_t cmd;
        uint32_t   param[4];
    } file_cmd_data_t;

    static constexpr int FILE_CMD_QUEUE_LENGTH = 2;

    static queue_t _file_cmd_queue;
    static queue_t _file_cmd_reply_queue;

public:
    static void initialize();
    static void req_prepare_file(spdif_rec_wav& inst_w_sts, const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample);
    static void req_finalize_file(spdif_rec_wav& inst_w_sts, const bool report_final, const float truncate_sec = 0.0f);
    static file_cmd* parse(file_cmd_data_t& cmd_data);
    static void process_file_cmd();
    static void process_file_reply_cmd();
    static bool reply_file_cmd(file_cmd_data_t& cmd_data);

    file_cmd(file_cmd_data_t& cmd_data);
    virtual ~file_cmd();
    virtual bool execute() = 0;

protected:
    file_cmd_data_t _cmd_data;
};

class nop : public file_cmd
{
public:
    nop(file_cmd_data_t& cmd_data);
    bool execute();
};

class prepare : public file_cmd
{
public:
    prepare(file_cmd_data_t& cmd_data);
    bool execute();
};

class finalize : public file_cmd
{
public:
    finalize(file_cmd_data_t& cmd_data);
    bool execute();
};