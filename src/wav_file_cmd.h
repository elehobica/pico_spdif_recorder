/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include <map>

#include "pico/util/queue.h"

#include "spdif_rec_wav.h"
#include "wav_file_status.h"

/*--------------------------/
/  WAV file command class
/--------------------------*/
class wav_file_cmd
{
protected:
    // === Private definitions of class ===
    enum class wav_file_cmd_type_t {
        PREPARE = 0,
        FINALIZE
    };
    typedef struct _wav_file_cmd_data_t {
        wav_file_cmd_type_t cmd;
        uint32_t   param[4];
    } wav_file_cmd_data_t;

    // === Private class constants ===
    static constexpr int WAV_FILE_CMD_QUEUE_LENGTH = 2;
    static const std::map<const wav_file_cmd_type_t, const wav_file_status::status_t> _cmd_to_status;

    // === Private class variables ===
    static queue_t _wav_file_cmd_queue;
    static queue_t _wav_file_cmd_reply_queue;

public:
    // === Public class functions ===
    // functions called from core1
    static void initialize();
    static bool req_prepare(wav_file_status& wfs, const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample);
    static bool req_finalize(wav_file_status& wfs, const bool report_final, const float truncate_sec = 0.0f);
    static void process_file_reply_cmd();
    // functions called from core0
    static void process_wav_file_cmd();

    // === Public member functions ===
    // functions called from core0
    virtual bool execute() = 0;

protected:
    // === Private class functions ===
    // functions called from core0
    static wav_file_cmd* _parse(wav_file_cmd_data_t& cmd_data);
    static bool _reply_wav_file_cmd(wav_file_cmd_data_t& cmd_data);
    // functions called from core1
    static bool _req_wav_file_cmd(wav_file_cmd_data_t& cmd_data);

    // === Constructor and Destructor (Private) ===
    wav_file_cmd(wav_file_cmd_data_t& cmd_data);
    virtual ~wav_file_cmd();

    // === Private member variables ===
    wav_file_cmd_data_t _cmd_data;
};

/*--------------------------/
/  nop command
/--------------------------*/
class nop : public wav_file_cmd
{
public:
    nop(wav_file_cmd_data_t& cmd_data);
    virtual bool execute();
};

/*--------------------------/
/  prepare command
/--------------------------*/
class prepare : public wav_file_cmd
{
public:
    prepare(wav_file_cmd_data_t& cmd_data);
    virtual bool execute();
};

/*--------------------------/
/  finalize command
/--------------------------*/
class finalize : public wav_file_cmd
{
public:
    finalize(wav_file_cmd_data_t& cmd_data);
    virtual bool execute();
};