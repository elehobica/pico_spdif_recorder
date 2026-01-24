# WAV Tagger with ID3v2.3 Support

A Python tool for processing WAV files recorded by [pico_spdif_recorder](../README.md). This tool automatically merges or splits WAV files based on album metadata and adds ID3v2.3 tags with UTF-8 encoding for multi-language support.

## Features

- **Automatic Processing**: Compares WAV file durations with expected track times and automatically processes files within tolerance
- **Interactive Mode**: Prompts user when WAV duration exceeds tolerance for manual decision
- **Smart Merge/Split**: Merges short files or splits long files to match expected track durations
- **ID3v2.3 Tags**: Adds proper ID3v2.3 metadata tags (not RIFF tags) with UTF-8 encoding
- **Multi-language Support**: Full UTF-8 support for Japanese, Korean, and other languages in metadata

## Installation

### Requirements

- Python 3.x (Python 3.7 or later recommended)

### Setup Virtual Environment

It's recommended to use a Python virtual environment to avoid conflicts with system packages.

#### Windows

```bash
# Navigate to wav_tagger directory
cd wav_tagger

# Create virtual environment
python -m venv venv

# Activate virtual environment
source venv\Scripts\activate  #bash

# Install dependencies
pip install -r requirements.txt
```

#### Linux / macOS

```bash
# Navigate to wav_tagger directory
cd wav_tagger

# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### Deactivate Virtual Environment

When you're done, deactivate the virtual environment:

```bash
deactivate
```

## Usage

**Note**: Make sure the virtual environment is activated before running the script.

### Basic Syntax

```bash
python wav_tagger.py <wav_directory> <metadata.yaml> <output_directory> [tolerance_seconds]
```

### Arguments

| Argument | Required | Description |
|----------|----------|-------------|
| `wav_directory` | Yes | Directory containing input WAV files |
| `metadata.yaml` | Yes | YAML file with album metadata |
| `output_directory` | Yes | Directory for output files with ID3 tags |
| `tolerance_seconds` | No | Tolerance in seconds (default: 3) |

### Examples

Using default tolerance (3 seconds):
```bash
python wav_tagger.py ./input ./album.yaml ./output
```

Using custom tolerance (5 seconds):
```bash
python wav_tagger.py ./input ./album.yaml ./output 5
```

## YAML Metadata Format

### Basic Structure

```yaml
album: "Album Name"
artist: "Artist Name"
year: 2024
genre: "Classical"
tracks:
  - title: "Track Title 1"
    time: "3:45"
    track-number: 1
  - title: "Track Title 2"
    time: "4:12"
    track-number: 2
```

### Field Specifications

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `album` | string | Yes | Album name (UTF-8 supported) |
| `artist` | string | Yes | Artist name (UTF-8 supported) |
| `year` | integer | Yes | Release year |
| `genre` | string | Yes | Music genre (UTF-8 supported) |
| `tracks` | list | Yes | List of track metadata |

### Track Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `title` | string | Yes | Track title (UTF-8 supported) |
| `time` | string | Yes | Expected duration in `mm:ss` format |
| `track-number` | integer | Yes | Track number (1-based) |

### Important Notes

- **Use kebab-case** for YAML parameter names (`track-number`, not `trackNumber`)
- **Time format** is `mm:ss` without decimal seconds (e.g., `3:45`, `12:03`)
- **UTF-8 encoding** is supported for all text fields (album, artist, title, genre)
- **Tolerance** is NOT specified in YAML - it's a command line argument

### Example with Japanese Text

```yaml
album: "ピアノ協奏曲集"
artist: "東京交響楽団"
year: 2024
genre: "Classical"
tracks:
  - title: "月の光"
    time: "5:23"
    track-number: 1
  - title: "亜麻色の髪の乙女"
    time: "2:41"
    track-number: 2
```

## How It Works

### Processing Flow

1. **Read WAV Files**: Scans the input directory for WAV files in alphabetical order
2. **Compare Durations**: For each track, compares actual WAV duration with expected time from YAML
3. **Automatic Processing**: If within tolerance (±3 seconds by default), processes automatically
4. **Interactive Mode**: If outside tolerance, prompts user for action
5. **Add ID3 Tags**: Creates ID3v2.3 tags with UTF-8 encoded metadata
6. **Output**: Saves tagged files with format `{track-number:02d}. {title}.wav`

### Tolerance Behavior

When WAV duration is **within tolerance**:
- File is automatically used as-is
- ID3 tag is added
- Proceeds to next track

When WAV duration is **outside tolerance**:
- User is prompted with options based on the difference

### Interactive Options

#### If WAV is LONGER than expected:

1. **Use as is**: Keep full duration, no splitting
2. **Split at expected time**: First part becomes current track, remainder is used for next processing
3. **Skip this track**: Skip and move to next track

#### If WAV is SHORTER than expected:

1. **Use as is**: Accept shorter duration
2. **Merge with next WAV file**: Combine with next file to reach expected duration
3. **Skip this track**: Skip and move to next track

## Output Format

### File Naming

Output files are named according to the pattern:
```
{track-number:02d}. {title}.wav
```

Examples:
- `01. Adagio in G Minor for Strings and Organ.wav`
- `02. Canon in D Major.wav`
- `03. 月の光.wav`

### File Structure

Each output WAV file contains:
```
[ID3v2.3 Tag Header]
[ID3v2.3 Tag Frames (TIT2, TPE1, TALB, TYER, TCON, TRCK)]
[Original WAV Audio Data (RIFF format)]
```

### ID3v2.3 Tag Details

- **Format**: ID3v2.3 (prepended to WAV file, not RIFF tags)
- **Encoding**: UTF-8 (encoding byte = 3)
- **Frames**:
  - `TIT2`: Title
  - `TPE1`: Artist
  - `TALB`: Album
  - `TYER`: Year
  - `TCON`: Genre
  - `TRCK`: Track number

## Technical Notes

### WAV File Compatibility

- **Merging**: Requires matching parameters (sample rate, bit depth, channels)
- **Splitting**: Done at frame boundaries to maintain audio integrity
- **ID3 Tags**: Prepended to WAV file (standard practice for ID3 in WAV)
- **Synchsafe Integer**: Used for tag size encoding in ID3v2.3 header

### Error Handling

The tool will report errors if:
- Input WAV files have incompatible parameters during merge
- YAML file is malformed or missing required fields
- Input directory is empty or files are missing
- Output directory cannot be created

## Typical Workflow

### 1. Record with pico_spdif_recorder

Record your album using the pico_spdif_recorder device. Use blank split feature to automatically separate tracks.

### 2. Create Metadata YAML

Create a YAML file with your album information:

```yaml
album: "My Album"
artist: "My Artist"
year: 2024
genre: "Classical"
tracks:
  - title: "First Movement"
    time: "8:23"
    track-number: 1
  - title: "Second Movement"
    time: "6:15"
    track-number: 2
```

### 3. Run wav_tagger

Activate the virtual environment and process your recorded files:

**Windows:**
```bash
cd wav_tagger
venv\Scripts\activate
python wav_tagger.py ./recorded_files ./album.yaml ./output
```

**Linux / macOS:**
```bash
cd wav_tagger
source venv/bin/activate
python wav_tagger.py ./recorded_files ./album.yaml ./output
```

### 4. Review and Adjust

- Check the output files
- If needed, adjust tolerance or re-run with different options
- Verify ID3 tags in your media player

## Troubleshooting

### "No WAV files found"
- Check that WAV files exist in the input directory
- Verify files have `.wav` extension (lowercase)

### "WAV parameters mismatch"
- Ensure all WAV files have the same sample rate, bit depth, and channel count
- Pico_spdif_recorder ensures consistency when recording from the same source

### "Invalid time format"
- Use `mm:ss` format only (e.g., `3:45`, not `3:45.5`)
- Don't include decimal seconds

### Tolerance too strict/loose
- Adjust the tolerance parameter (default is 3 seconds)
- Use a larger tolerance for more automatic processing
- Use a smaller tolerance for more precision

## License

This tool is part of the pico_spdif_recorder project.
Released under the BSD-2-Clause License.

## Related Projects

- [pico_spdif_recorder](../README.md): Main firmware for recording S/PDIF audio
