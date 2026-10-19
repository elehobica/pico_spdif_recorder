#!/usr/bin/env python3
"""
WAV file I/O supporting RIFF (with JUNK or other chunks) and RF64

pico_spdif_recorder writes the following WAV formats:
  - standard RIFF WAV with 44 bytes header
  - RIFF WAV with 80 bytes header including JUNK chunk reserved for ds64 chunk
    (when RF64 WAV support is on, WAV file of 4 GB or less)
  - RF64 WAV with ds64 chunk (EBU Tech 3306, when RF64 WAV support is on and WAV file exceeds 4 GB)

Python standard 'wave' module can read the first two but not RF64.
This module reads all of them and writes RIFF or RF64 depending on the data size.
Audio data is copied in blocks without loading whole file on memory to handle huge files.
"""

import os
import re
import shutil
import struct
from dataclasses import dataclass
from typing import BinaryIO, Iterable, Tuple


# Constants
WAVE_FORMAT_PCM = 0x0001
WAVE_FORMAT_EXTENSIBLE = 0xFFFE
RIFF_MAX_SIZE = 0xFFFFFFFF  # max value of 32-bit size field
COPY_BLOCK_BYTES = 4 * 1024 * 1024  # block size to copy audio data


def natural_key(name: str) -> list:
    """Sort key comparing digit parts as numbers (e.g. 'record_999.wav' < 'record_1000.wav')"""
    return [int(t) if t.isdigit() else t for t in re.split(r'(\d+)', name)]


@dataclass
class WavParams:
    """Parameters of WAV file (compatible with the attributes of wave.getparams())"""
    nchannels: int
    sampwidth: int
    framerate: int
    nframes: int
    data_offset: int  # file offset of audio data
    data_size: int    # bytes of audio data
    rf64: bool        # True if RF64 format

    @property
    def frame_size(self) -> int:
        return self.nchannels * self.sampwidth

    @property
    def duration(self) -> float:
        return self.nframes / float(self.framerate)


def read_params(filepath: str) -> WavParams:
    """Parse WAV header (RIFF or RF64) and return its parameters"""
    file_size = os.path.getsize(filepath)
    with open(filepath, 'rb') as f:
        riff_id, _, wave_id = struct.unpack('<4sI4s', _read_exact(f, 12))
        if riff_id not in (b'RIFF', b'RF64') or wave_id != b'WAVE':
            raise ValueError(f"Not a WAV file: {filepath}")
        rf64 = riff_id == b'RF64'

        ds64_data_size = None
        fmt = None
        while True:
            header = f.read(8)
            if len(header) < 8:
                raise ValueError(f"data chunk not found: {filepath}")
            chunk_id, chunk_size = struct.unpack('<4sI', header)
            chunk_pos = f.tell()
            if chunk_id == b'ds64':
                # riffSize (8), dataSize (8), sampleCount (8), tableLength (4)
                _, ds64_data_size, _ = struct.unpack('<QQQ', _read_exact(f, 24))
            elif chunk_id == b'fmt ':
                fmt = _parse_fmt(_read_exact(f, chunk_size), filepath)
            elif chunk_id == b'data':
                if fmt is None:
                    raise ValueError(f"fmt chunk not found before data chunk: {filepath}")
                if rf64 and chunk_size == RIFF_MAX_SIZE:
                    if ds64_data_size is None:
                        raise ValueError(f"RF64 without ds64 chunk: {filepath}")
                    data_size = ds64_data_size
                else:
                    data_size = chunk_size
                nchannels, sampwidth, framerate = fmt
                frame_size = nchannels * sampwidth
                available = file_size - chunk_pos
                if data_size > available:
                    print(f"  ! Warning: data size ({data_size} bytes) exceeds file size in {os.path.basename(filepath)}, "
                          f"using {available} bytes")
                    data_size = available
                data_size -= data_size % frame_size  # ignore partial frame at the end
                return WavParams(nchannels, sampwidth, framerate, data_size // frame_size,
                                 chunk_pos, data_size, rf64)
            # skip chunk with padding byte (JUNK, LIST and so on)
            f.seek(chunk_pos + chunk_size + (chunk_size & 1))


def write_header(f: BinaryIO, nchannels: int, sampwidth: int, framerate: int, data_size: int) -> None:
    """Write WAV header for given data size: RIFF (44 bytes) or RF64 (80 bytes) if exceeding 32-bit"""
    block_align = nchannels * sampwidth
    fmt_chunk = struct.pack('<4sIHHIIHH', b'fmt ', 16, WAVE_FORMAT_PCM, nchannels, framerate,
                            framerate * block_align, block_align, sampwidth * 8)
    riff_size = 4 + len(fmt_chunk) + 8 + data_size + (data_size & 1)
    if riff_size <= RIFF_MAX_SIZE:
        f.write(struct.pack('<4sI4s', b'RIFF', riff_size, b'WAVE'))
        f.write(fmt_chunk)
        f.write(struct.pack('<4sI', b'data', data_size))
    else:
        ds64_chunk = struct.pack('<4sIQQQI', b'ds64', 28, riff_size + 36, data_size,
                                 data_size // block_align, 0)
        f.write(struct.pack('<4sI4s', b'RF64', RIFF_MAX_SIZE, b'WAVE'))
        f.write(ds64_chunk)
        f.write(fmt_chunk)
        f.write(struct.pack('<4sI', b'data', RIFF_MAX_SIZE))


def write_wav(output_path: str, params: WavParams, sources: Iterable[Tuple[str, int, int]]) -> None:
    """
    Write WAV file by concatenating audio data of sources
    sources: list of (filepath, data offset in bytes, data size in bytes)
    """
    sources = list(sources)
    data_size = sum(size for _, _, size in sources)
    with open(output_path, 'wb') as out:
        write_header(out, params.nchannels, params.sampwidth, params.framerate, data_size)
        for filepath, offset, size in sources:
            with open(filepath, 'rb') as src:
                src.seek(offset)
                _copy_bytes(src, out, size)
        if data_size & 1:
            out.write(b'\x00')  # padding byte


def copy_file(src_path: str, dst_path: str) -> None:
    """Copy file in blocks (as is, including header)"""
    shutil.copyfile(src_path, dst_path)


def _parse_fmt(data: bytes, filepath: str) -> Tuple[int, int, int]:
    """Parse fmt chunk and return (nchannels, sampwidth, framerate) of PCM"""
    audio_format, nchannels, framerate, _, _, bits = struct.unpack('<HHIIHH', data[:16])
    if audio_format == WAVE_FORMAT_EXTENSIBLE and len(data) >= 26:
        audio_format = struct.unpack('<H', data[24:26])[0]  # first 2 bytes of SubFormat GUID
    if audio_format != WAVE_FORMAT_PCM:
        raise ValueError(f"Unsupported audio format ({audio_format:#06x}): {filepath}")
    return nchannels, (bits + 7) // 8, framerate


def _read_exact(f: BinaryIO, size: int) -> bytes:
    data = f.read(size)
    if len(data) != size:
        raise ValueError("Unexpected end of file in WAV header")
    return data


def _copy_bytes(src: BinaryIO, dst: BinaryIO, size: int) -> None:
    while size > 0:
        block = src.read(min(size, COPY_BLOCK_BYTES))
        if not block:
            raise ValueError("Unexpected end of file in WAV data")
        dst.write(block)
        size -= len(block)
