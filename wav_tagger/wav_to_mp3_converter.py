#!/usr/bin/env python3
"""
WAV to MP3 Converter with WAV preprocessing (merge/split) and verification

Unified pipeline:
  1. Load YAML metadata (from Spotify metadata extractor)
  2. Compare WAV durations against expected track durations
  3. If mismatch: interactive merge/split using WavProcessor
  4. Convert to MP3 via ffmpeg (320kbps, ID3v2.3)
  5. Embed cover art if JPG found in output folder
  6. Post-conversion WAV⇔MP3 duration verification

Usage:
  python wav_to_mp3_converter.py <wav_directory> <metadata.yaml> <output_directory> [options]

Options:
  --tolerance N     Duration tolerance in seconds (default: 3)
  --bitrate N       MP3 bitrate e.g. 320k (default: 320k)
  --cover PATH      Cover art image path (auto-detected if not specified)
  --no-cover        Skip cover art embedding
  --skip-preprocess Skip WAV preprocessing (assume WAVs already match tracks)
  --auto            Non-interactive mode: use WAV as-is when mismatch
"""

import os
import sys
import struct
import wave
import yaml
import subprocess
import shutil
import argparse
from pathlib import Path
from typing import List, Dict, Tuple, Optional


# ============================================================
# Constants
# ============================================================
MAX_FILENAME_LENGTH = 128
MAX_ID3_TITLE_LENGTH = 256


# ============================================================
# WAV Processing (from wav_tagger.py)
# ============================================================
class WavProcessor:
    """Process WAV files: merge, split, and duration check"""

    def __init__(self, tolerance_seconds: int = 3):
        self.tolerance_seconds = tolerance_seconds

    def parse_time(self, time_str: str) -> int:
        """Parse time string (m:ss or mm:ss) to seconds"""
        parts = time_str.split(':')
        if len(parts) != 2:
            raise ValueError(f"Invalid time format: {time_str}. Expected m:ss or mm:ss")
        minutes, seconds = map(int, parts)
        return minutes * 60 + seconds

    def get_wav_duration(self, filepath: str) -> float:
        """Get duration of WAV file in seconds"""
        with wave.open(filepath, 'rb') as wav:
            frames = wav.getnframes()
            rate = wav.getframerate()
            return frames / float(rate)

    def read_wav_data(self, filepath: str) -> Tuple[bytes, tuple]:
        """Read WAV file and return audio data and parameters"""
        with wave.open(filepath, 'rb') as wav:
            params = wav.getparams()
            frames = wav.readframes(wav.getnframes())
            return frames, params

    def merge_wav_files(self, filepaths: List[str], output_path: str) -> None:
        """Merge multiple WAV files into one"""
        if not filepaths:
            raise ValueError("No files to merge")

        first_frames, params = self.read_wav_data(filepaths[0])
        all_frames = [first_frames]

        for filepath in filepaths[1:]:
            frames, file_params = self.read_wav_data(filepath)
            if (file_params.nchannels != params.nchannels or
                file_params.sampwidth != params.sampwidth or
                file_params.framerate != params.framerate):
                raise ValueError(f"WAV parameters mismatch in {filepath}")
            all_frames.append(frames)

        with wave.open(output_path, 'wb') as output:
            output.setparams(params)
            output.writeframes(b''.join(all_frames))

    def split_wav_file(self, filepath: str, split_time: float,
                       output1: str, output2: str) -> None:
        """Split WAV file at specified time (in seconds)"""
        with wave.open(filepath, 'rb') as wav:
            params = wav.getparams()
            frames = wav.readframes(wav.getnframes())

            frame_size = params.sampwidth * params.nchannels
            split_frame = int(split_time * params.framerate)
            split_byte = split_frame * frame_size

            with wave.open(output1, 'wb') as out1:
                out1.setparams(params)
                out1.writeframes(frames[:split_byte])

            with wave.open(output2, 'wb') as out2:
                out2.setparams(params)
                out2.writeframes(frames[split_byte:])


# ============================================================
# Helper functions
# ============================================================
def format_time(seconds: float) -> str:
    """Format seconds as m:ss"""
    minutes = int(seconds // 60)
    secs = int(seconds % 60)
    return f"{minutes}:{secs:02d}"


def sanitize_filename(filename: str) -> str:
    """Replace invalid filename characters with '-'"""
    invalid_chars = '<>:"/\\|?*'
    sanitized = filename
    for char in invalid_chars:
        sanitized = sanitized.replace(char, '-')
    return sanitized.strip('. ')


def load_metadata(yaml_path: str) -> Dict:
    """Load metadata from YAML file"""
    with open(yaml_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def find_cover_art(directory: str) -> Optional[str]:
    """Auto-detect cover art image in directory"""
    for ext in ['jpg', 'jpeg', 'png']:
        # Check for common names first
        for name in ['cover', 'cover_art', 'folder', 'front']:
            path = os.path.join(directory, f"{name}.{ext}")
            if os.path.exists(path):
                return path
        # Then check any image file
        for f in os.listdir(directory):
            if f.lower().endswith(f'.{ext}') and not f.startswith('.'):
                return os.path.join(directory, f)
    return None


def get_mp3_duration(filepath: str) -> Optional[float]:
    """Get MP3 duration using ffprobe"""
    cmd = [
        'ffprobe', '-v', 'quiet',
        '-show_entries', 'format=duration',
        '-of', 'csv=p=0',
        filepath
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode == 0 and result.stdout.strip():
        try:
            return float(result.stdout.strip())
        except ValueError:
            return None
    return None


def get_user_choice(track_title: str, expected_time: str,
                    actual_time: str, diff: float, is_longer: bool,
                    multi_track_count: int = 0, multi_track_time: str = "") -> int:
    """Ask user what to do when WAV duration doesn't match expected time"""
    print(f"\n{'='*60}")
    print(f"Track: \"{track_title}\" (Expected: {expected_time})")
    print(f"Current WAV duration: {actual_time}")
    print(f"Difference: {'+' if is_longer else '-'}{abs(diff):.2f}s "
          f"(tolerance exceeded)")
    print(f"{'='*60}")

    max_choice = 3
    if is_longer:
        print("\nOptions:")
        print("1. Use as is (keep full duration)")
        print("2. Split at expected time (first part -> current track, "
              "rest -> next)")
        print("3. Skip this track")
        if multi_track_count > 1:
            print(f"4. Combine next {multi_track_count} tracks into one WAV "
                  f"(total expected: {multi_track_time})")
            max_choice = 4
    else:
        print("\nOptions:")
        print("1. Use as is (shorter duration)")
        print("2. Merge with next WAV file")
        print("3. Skip this track")

    while True:
        try:
            prompt = f"\nEnter your choice (1-{max_choice}, default=1): "
            user_input = input(prompt).strip()
            if user_input == "":
                return 1
            choice = int(user_input)
            if 1 <= choice <= max_choice:
                return choice
            print(f"Invalid choice. Please enter 1-{max_choice}.")
        except ValueError:
            print("Invalid input. Please enter a number.")


# ============================================================
# Phase 1: WAV Preprocessing (merge/split)
# ============================================================
def preprocess_wavs(wav_dir: str, metadata: Dict, temp_dir: str,
                    tolerance: int = 3, auto_mode: bool = False) -> List[Tuple[str, Dict]]:
    """
    Compare WAV files against metadata and preprocess if needed.
    Returns list of (wav_path, track_metadata) tuples ready for conversion.
    """
    processor = WavProcessor(tolerance_seconds=tolerance)
    wav_files = sorted([f for f in os.listdir(wav_dir) if f.endswith('.wav')])

    if not wav_files:
        print("No WAV files found in the directory.")
        return []

    tracks = metadata['tracks']
    print(f"\nFound {len(wav_files)} WAV files, {len(tracks)} tracks in metadata")

    os.makedirs(temp_dir, exist_ok=True)
    results = []

    current_wav_idx = 0
    track_idx = 0

    while track_idx < len(tracks):
        track = tracks[track_idx]

        if current_wav_idx >= len(wav_files):
            print(f"\n  Warning: No more WAV files for track {track['track-number']}")
            break

        track_title = track['title']
        expected_duration = processor.parse_time(track['time'])
        track_number = track['track-number']

        # Current WAV file - handle both absolute and relative paths
        current_wav_name = wav_files[current_wav_idx]
        if os.path.isabs(current_wav_name):
            current_wav_path = current_wav_name
        else:
            current_wav_path = os.path.join(wav_dir, current_wav_name)

        actual_duration = processor.get_wav_duration(current_wav_path)
        diff = actual_duration - expected_duration

        print(f"\n  Track {track_number}: {track_title}")
        print(f"    Expected: {track['time']} ({expected_duration}s)")
        print(f"    WAV: {os.path.basename(current_wav_path)} "
              f"({format_time(actual_duration)}, {actual_duration:.1f}s)")

        # Build track metadata dict
        track_meta = {
            'title': track_title,
            'artist': track.get('artist', metadata['artist']),
            'album': metadata['album'],
            'year': str(metadata['year']),
            'genre': metadata['genre'],
            'track_number': track_number,
            'total_tracks': len(tracks),
        }

        if abs(diff) <= tolerance:
            # Within tolerance - use as-is
            print(f"    OK (diff: {diff:+.1f}s)")
            results.append((current_wav_path, track_meta))
            current_wav_idx += 1
        elif auto_mode:
            # Auto mode: use as-is regardless
            print(f"    AUTO: using as-is (diff: {diff:+.1f}s)")
            results.append((current_wav_path, track_meta))
            current_wav_idx += 1
        else:
            # Interactive mode
            multi_track_count = 0
            multi_track_time = ""

            if diff > 0:  # WAV is longer
                cumulative_duration = 0
                for i in range(track_idx, len(tracks)):
                    cumulative_duration += processor.parse_time(tracks[i]['time'])
                    if abs(actual_duration - cumulative_duration) <= tolerance:
                        multi_track_count = i - track_idx + 1
                        multi_track_time = format_time(cumulative_duration)
                        break
                    elif cumulative_duration > actual_duration + tolerance:
                        break

            choice = get_user_choice(
                track_title, track['time'],
                format_time(actual_duration), diff,
                is_longer=(diff > 0),
                multi_track_count=multi_track_count,
                multi_track_time=multi_track_time
            )

            if choice == 1:
                # Use as is
                print("    → Using as is")
                results.append((current_wav_path, track_meta))
                current_wav_idx += 1

            elif choice == 2:
                if diff > 0:
                    # Split
                    print(f"    → Splitting at {track['time']}")
                    part1 = os.path.join(temp_dir, f"split_{track_number:02d}_a.wav")
                    part2 = os.path.join(temp_dir, f"split_{track_number:02d}_b.wav")
                    processor.split_wav_file(current_wav_path, expected_duration, part1, part2)
                    results.append((part1, track_meta))
                    # Insert remainder for next iteration
                    wav_files.insert(current_wav_idx + 1, part2)
                    current_wav_idx += 1
                else:
                    # Merge with next
                    if current_wav_idx + 1 >= len(wav_files):
                        print("    ! No next file to merge. Using as is.")
                        results.append((current_wav_path, track_meta))
                        current_wav_idx += 1
                    else:
                        next_name = wav_files[current_wav_idx + 1]
                        if os.path.isabs(next_name):
                            next_wav_path = next_name
                        else:
                            next_wav_path = os.path.join(wav_dir, next_name)
                        merged = os.path.join(temp_dir, f"merged_{track_number:02d}.wav")
                        print(f"    → Merging with {os.path.basename(next_wav_path)}")
                        processor.merge_wav_files([current_wav_path, next_wav_path], merged)
                        results.append((merged, track_meta))
                        current_wav_idx += 2

            elif choice == 4:
                # Combine multiple tracks
                print(f"    → Combining {multi_track_count} tracks")
                combined_titles = [tracks[track_idx + i]['title']
                                   for i in range(multi_track_count)]
                track_meta['title'] = " / ".join(combined_titles)
                results.append((current_wav_path, track_meta))
                track_idx += multi_track_count - 1
                current_wav_idx += 1

            else:
                # Skip
                print("    → Skipping")
                current_wav_idx += 1
                track_idx += 1
                continue

        track_idx += 1

    return results


# ============================================================
# Phase 2: Convert WAV to MP3
# ============================================================
def convert_to_mp3(wav_path: str, mp3_path: str, track_meta: Dict,
                   bitrate: str = '320k', cover_path: Optional[str] = None) -> bool:
    """Convert a single WAV to MP3 with ID3 tags and optional cover art"""
    cmd = [
        'ffmpeg', '-y',
        '-i', wav_path,
    ]

    # Add cover art input if available
    if cover_path:
        cmd += ['-i', cover_path]

    # Audio mapping
    cmd += ['-map', '0:a']

    # Cover art mapping
    if cover_path:
        cmd += [
            '-map', '1:0',
            '-codec:v:0', 'mjpeg',
            '-metadata:s:v', 'title=Album cover',
            '-metadata:s:v', 'comment=Cover (front)',
            '-disposition:v:0', 'attached_pic',
        ]

    # Audio codec settings
    cmd += [
        '-codec:a', 'libmp3lame',
        '-b:a', bitrate,
        '-id3v2_version', '3',
    ]

    # Metadata
    title = track_meta['title']
    if len(title) > MAX_ID3_TITLE_LENGTH:
        title = title[:MAX_ID3_TITLE_LENGTH].rstrip()

    cmd += [
        '-metadata', f"title={title}",
        '-metadata', f"artist={track_meta['artist']}",
        '-metadata', f"album={track_meta['album']}",
        '-metadata', f"date={track_meta['year']}",
        '-metadata', f"genre={track_meta['genre']}",
        '-metadata', f"track={track_meta['track_number']}/{track_meta['total_tracks']}",
    ]

    cmd.append(mp3_path)

    result = subprocess.run(cmd, capture_output=True, text=True)
    return result.returncode == 0


# ============================================================
# Phase 3: Post-conversion verification
# ============================================================
def verify_durations(wav_mp3_pairs: List[Tuple[str, str, Dict]],
                     tolerance: float = 0.5) -> bool:
    """Verify MP3 durations match original WAV durations"""
    print(f"\n{'='*60}")
    print("Post-conversion verification (WAV vs MP3)")
    print(f"{'='*60}")

    all_ok = True
    for wav_path, mp3_path, track_meta in wav_mp3_pairs:
        processor = WavProcessor()
        wav_dur = processor.get_wav_duration(wav_path)
        mp3_dur = get_mp3_duration(mp3_path)

        if mp3_dur is None:
            print(f"  ✗ Track {track_meta['track_number']}: "
                  f"Could not read MP3 duration")
            all_ok = False
            continue

        diff = abs(wav_dur - mp3_dur)
        status = "OK" if diff <= tolerance else "MISMATCH"

        if diff > tolerance:
            all_ok = False
            print(f"  ✗ Track {track_meta['track_number']:2d} {track_meta['title']}: "
                  f"WAV={format_time(wav_dur)} MP3={format_time(mp3_dur)} "
                  f"diff={diff:.2f}s [{status}]")
        else:
            print(f"  ✓ Track {track_meta['track_number']:2d}: "
                  f"WAV={format_time(wav_dur)} MP3={format_time(mp3_dur)} "
                  f"diff={diff:.3f}s")

    return all_ok


# ============================================================
# Main pipeline
# ============================================================
def main():
    parser = argparse.ArgumentParser(
        description='WAV to MP3 Converter with preprocessing and verification',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic conversion (interactive mode for mismatches)
  python wav_to_mp3_converter.py ./wavs album.yaml ./mp3s

  # Auto mode (no prompts, use WAVs as-is)
  python wav_to_mp3_converter.py ./wavs album.yaml ./mp3s --auto

  # Skip preprocessing (WAVs already match tracks 1:1)
  python wav_to_mp3_converter.py ./wavs album.yaml ./mp3s --skip-preprocess

  # Custom bitrate and cover art
  python wav_to_mp3_converter.py ./wavs album.yaml ./mp3s --bitrate 256k --cover art.jpg
        """
    )
    parser.add_argument('wav_dir', help='Directory containing input WAV files')
    parser.add_argument('yaml_path', help='YAML metadata file')
    parser.add_argument('output_dir', help='Output directory for MP3 files')
    parser.add_argument('--tolerance', type=int, default=3,
                        help='Duration tolerance in seconds (default: 3)')
    parser.add_argument('--bitrate', default='320k',
                        help='MP3 bitrate (default: 320k)')
    parser.add_argument('--cover', default=None,
                        help='Cover art image path (auto-detected if not specified)')
    parser.add_argument('--no-cover', action='store_true',
                        help='Skip cover art embedding')
    parser.add_argument('--skip-preprocess', action='store_true',
                        help='Skip WAV preprocessing (assume 1:1 WAV-to-track mapping)')
    parser.add_argument('--auto', action='store_true',
                        help='Non-interactive mode: use WAV as-is when mismatch')
    parser.add_argument('--verify-tolerance', type=float, default=0.5,
                        help='Post-conversion verification tolerance in seconds (default: 0.5)')

    args = parser.parse_args()

    # Validate inputs
    if not os.path.isdir(args.wav_dir):
        print(f"Error: WAV directory not found: {args.wav_dir}")
        sys.exit(1)
    if not os.path.isfile(args.yaml_path):
        print(f"Error: YAML file not found: {args.yaml_path}")
        sys.exit(1)

    # Load metadata
    metadata = load_metadata(args.yaml_path)
    print(f"\n{'='*60}")
    print(f"Album: {metadata['album']}")
    print(f"Artist: {metadata['artist']}")
    print(f"Year: {metadata['year']}")
    print(f"Tracks: {len(metadata['tracks'])}")
    print(f"Bitrate: {args.bitrate}")
    print(f"{'='*60}")

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)

    # Temp directory for preprocessing
    temp_dir = os.path.join(args.output_dir, '.temp_wav')

    # --------------------------------------------------------
    # Phase 1: WAV Preprocessing
    # --------------------------------------------------------
    if args.skip_preprocess:
        print("\n[Phase 1] Skipping WAV preprocessing")
        processor = WavProcessor(tolerance_seconds=args.tolerance)
        wav_files = sorted([f for f in os.listdir(args.wav_dir) if f.endswith('.wav')])
        tracks = metadata['tracks']

        if len(wav_files) != len(tracks):
            print(f"  WARNING: WAV count ({len(wav_files)}) != track count ({len(tracks)})")
            print(f"  Using min({len(wav_files)}, {len(tracks)}) tracks")

        track_pairs = []
        for i, track in enumerate(tracks):
            if i >= len(wav_files):
                break
            wav_path = os.path.join(args.wav_dir, wav_files[i])
            track_meta = {
                'title': track['title'],
                'artist': track.get('artist', metadata['artist']),
                'album': metadata['album'],
                'year': str(metadata['year']),
                'genre': metadata['genre'],
                'track_number': track['track-number'],
                'total_tracks': len(tracks),
            }
            track_pairs.append((wav_path, track_meta))
    else:
        print(f"\n[Phase 1] WAV Preprocessing (tolerance: ±{args.tolerance}s)")
        track_pairs = preprocess_wavs(
            args.wav_dir, metadata, temp_dir,
            tolerance=args.tolerance,
            auto_mode=args.auto
        )

    if not track_pairs:
        print("No tracks to convert. Exiting.")
        sys.exit(1)

    print(f"\n  → {len(track_pairs)} tracks ready for conversion")

    # --------------------------------------------------------
    # Phase 2: Convert to MP3
    # --------------------------------------------------------
    print(f"\n[Phase 2] Converting WAV to MP3 ({args.bitrate})")

    # Find cover art
    cover_path = None
    if not args.no_cover:
        if args.cover:
            cover_path = args.cover
        else:
            # Auto-detect: check output dir first, then wav dir
            cover_path = find_cover_art(args.output_dir)
            if not cover_path:
                cover_path = find_cover_art(args.wav_dir)

        if cover_path:
            cover_size = os.path.getsize(cover_path) / 1024
            print(f"  Cover art: {os.path.basename(cover_path)} ({cover_size:.0f} KB)")
        else:
            print("  Cover art: not found (skipping)")

    verification_pairs = []
    success_count = 0
    fail_count = 0

    for wav_path, track_meta in track_pairs:
        track_number = track_meta['track_number']
        title = track_meta['title']
        safe_title = sanitize_filename(title)

        # Truncate filename if too long
        max_title_length = MAX_FILENAME_LENGTH - 9  # "XX - " + ".mp3"
        if len(safe_title) > max_title_length:
            safe_title = safe_title[:max_title_length].rstrip()

        mp3_filename = f"{track_number:02d} - {safe_title}.mp3"
        mp3_path = os.path.join(args.output_dir, mp3_filename)

        # Skip if already exists and reasonable size
        if os.path.exists(mp3_path) and os.path.getsize(mp3_path) > 100000:
            size_mb = os.path.getsize(mp3_path) / (1024 * 1024)
            print(f"  - {track_number:2d}/{track_meta['total_tracks']} "
                  f"{title} (already exists, {size_mb:.1f}MB)")
            verification_pairs.append((wav_path, mp3_path, track_meta))
            success_count += 1
            continue

        ok = convert_to_mp3(wav_path, mp3_path, track_meta,
                            bitrate=args.bitrate, cover_path=cover_path)

        if ok:
            size_mb = os.path.getsize(mp3_path) / (1024 * 1024)
            print(f"  ✓ {track_number:2d}/{track_meta['total_tracks']} "
                  f"{title} ({size_mb:.1f}MB)")
            verification_pairs.append((wav_path, mp3_path, track_meta))
            success_count += 1
        else:
            print(f"  ✗ {track_number:2d}/{track_meta['total_tracks']} "
                  f"{title} FAILED")
            fail_count += 1

    print(f"\n  → Converted: {success_count}, Failed: {fail_count}")

    # --------------------------------------------------------
    # Phase 3: Verification
    # --------------------------------------------------------
    if verification_pairs:
        all_ok = verify_durations(verification_pairs,
                                  tolerance=args.verify_tolerance)
        if all_ok:
            print("\n  ✓ All tracks verified successfully!")
        else:
            print("\n  ⚠ Some tracks have duration mismatches!")

    # --------------------------------------------------------
    # Cleanup
    # --------------------------------------------------------
    if os.path.exists(temp_dir):
        shutil.rmtree(temp_dir)

    # Summary
    total_size = sum(
        os.path.getsize(os.path.join(args.output_dir, f))
        for f in os.listdir(args.output_dir)
        if f.endswith('.mp3')
    )

    print(f"\n{'='*60}")
    print(f"Complete!")
    print(f"  Output: {args.output_dir}")
    print(f"  Files: {success_count} MP3s, {total_size / (1024*1024):.1f} MB total")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
