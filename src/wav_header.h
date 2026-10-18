/*------------------------------------------------------/
/ Copyright (c) 2026, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include <cstdint>
#include <cstring>

// wav_header is helper class (prohibited to instantiate)
// no dependency on pico-sdk or FATFS to be testable on host PC
class wav_header final
{
public:
    // === Public class constants ===
    static constexpr int WAV_HEADER_SIZE = 44;  // standard RIFF WAV
    static constexpr int RF64_RESERVED_HEADER_SIZE = 80;  // RIFF WAV with JUNK chunk reserved for ds64 chunk (EBU Tech 3306)
    static constexpr int MAX_HEADER_SIZE = RF64_RESERVED_HEADER_SIZE;

    // === Public class functions ===
    // generate header of given data size into buf and return header size
    //  rf64_reserved: false: 44 bytes standard RIFF WAV
    //                 true:  80 bytes with JUNK chunk, which is replaced with ds64 chunk as RF64 if RIFF size exceeds 32bit
    //  data_bytes:    0 means temporary header at file creation (sizes are filled with 0)
    static int generate(uint8_t* buf, const uint32_t sample_freq, const uint16_t bits_per_sample, const uint16_t num_channels, const bool rf64_reserved, const uint64_t data_bytes)
    {
        const int header_size = rf64_reserved ? RF64_RESERVED_HEADER_SIZE : WAV_HEADER_SIZE;
        const bool rf64 = is_rf64(rf64_reserved, data_bytes);
        const uint64_t riff_bytes = (data_bytes == 0) ? 0 : data_bytes + (header_size - 8);
        const uint16_t block_align = num_channels * (bits_per_sample / 8);
        int pos = 0;

        memset(buf, 0, header_size);

        // RIFF chunk
        _put_id(buf, pos, rf64 ? "RF64" : "RIFF");
        _put_u32(buf, pos, rf64 ? 0xffffffff : static_cast<uint32_t>(riff_bytes));
        _put_id(buf, pos, "WAVE");

        // JUNK chunk reserved for ds64 chunk
        if (rf64_reserved) {
            _put_id(buf, pos, rf64 ? "ds64" : "JUNK");
            _put_u32(buf, pos, DS64_CHUNK_SIZE);
            if (rf64) {
                _put_u64(buf, pos, riff_bytes);  // riffSize
                _put_u64(buf, pos, data_bytes);  // dataSize
                _put_u64(buf, pos, data_bytes / block_align);  // sampleCount
                _put_u32(buf, pos, 0);  // tableLength
            } else {
                pos += DS64_CHUNK_SIZE;  // filled with 0
            }
        }

        // fmt chunk
        _put_id(buf, pos, "fmt ");
        _put_u32(buf, pos, 16);  // 16 for PCM
        _put_u16(buf, pos, 1);  // AudioFormat: PCM = 1
        _put_u16(buf, pos, num_channels);
        _put_u32(buf, pos, sample_freq);
        _put_u32(buf, pos, sample_freq * block_align);  // ByteRate
        _put_u16(buf, pos, block_align);
        _put_u16(buf, pos, bits_per_sample);

        // data chunk
        _put_id(buf, pos, "data");
        _put_u32(buf, pos, rf64 ? 0xffffffff : static_cast<uint32_t>(data_bytes));

        return pos;
    }

    // whether the header is RF64 (only when reserved and RIFF size exceeds 32bit)
    static bool is_rf64(const bool rf64_reserved, const uint64_t data_bytes)
    {
        return rf64_reserved && data_bytes + (RF64_RESERVED_HEADER_SIZE - 8) > 0xffffffffULL;
    }

protected:
    // === Private class constants ===
    static constexpr uint32_t DS64_CHUNK_SIZE = 28;  // riffSize(8) + dataSize(8) + sampleCount(8) + tableLength(4)

    // === Private class functions ===
    static void _put_id(uint8_t* buf, int& pos, const char* id)
    {
        memcpy(&buf[pos], id, 4);
        pos += 4;
    }
    static void _put_u16(uint8_t* buf, int& pos, const uint16_t val)
    {
        for (int i = 0; i < 2; i++) buf[pos++] = static_cast<uint8_t>(val >> (i * 8));
    }
    static void _put_u32(uint8_t* buf, int& pos, const uint32_t val)
    {
        for (int i = 0; i < 4; i++) buf[pos++] = static_cast<uint8_t>(val >> (i * 8));
    }
    static void _put_u64(uint8_t* buf, int& pos, const uint64_t val)
    {
        for (int i = 0; i < 8; i++) buf[pos++] = static_cast<uint8_t>(val >> (i * 8));
    }

    // === Constructor and Destructor (Prohibit) ===
    wav_header() = delete;
    ~wav_header() = delete;
};
