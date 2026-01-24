#!/usr/bin/env python3
"""
WAV File Tagger with ID3v2.3 Support

This script processes WAV files based on album metadata (YAML format),
automatically merges/splits them according to track durations,
and adds ID3v2.3 tags with UTF-8 encoding.
"""

import os
import struct
import wave
import yaml
from pathlib import Path
from typing import List, Dict, Tuple, Optional


# Constants
MAX_FILENAME_LENGTH = 128  # Maximum output filename length in characters
MAX_ID3_TITLE_LENGTH = 256  # Maximum ID3 title field length in characters


class WavProcessor:
    """Process WAV files: merge, split, and add ID3v2.3 tags"""
    
    def __init__(self, tolerance_seconds: int = 3):
        self.tolerance_seconds = tolerance_seconds
    
    def parse_time(self, time_str: str) -> int:
        """Parse time string (mm:ss) to seconds"""
        parts = time_str.split(':')
        if len(parts) != 2:
            raise ValueError(f"Invalid time format: {time_str}. Expected mm:ss")
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
        
        # Read first file to get parameters
        first_frames, params = self.read_wav_data(filepaths[0])
        all_frames = [first_frames]
        
        # Read remaining files and verify compatibility
        for filepath in filepaths[1:]:
            frames, file_params = self.read_wav_data(filepath)
            if (file_params.nchannels != params.nchannels or
                file_params.sampwidth != params.sampwidth or
                file_params.framerate != params.framerate):
                raise ValueError(f"WAV parameters mismatch in {filepath}")
            all_frames.append(frames)
        
        # Write merged file
        with wave.open(output_path, 'wb') as output:
            output.setparams(params)
            output.writeframes(b''.join(all_frames))
    
    def split_wav_file(self, filepath: str, split_time: float, 
                      output1: str, output2: str) -> None:
        """Split WAV file at specified time (in seconds)"""
        with wave.open(filepath, 'rb') as wav:
            params = wav.getparams()
            frames = wav.readframes(wav.getnframes())
            
            # Calculate split point in bytes
            frame_size = params.sampwidth * params.nchannels
            split_frame = int(split_time * params.framerate)
            split_byte = split_frame * frame_size
            
            # Write first part
            with wave.open(output1, 'wb') as out1:
                out1.setparams(params)
                out1.writeframes(frames[:split_byte])
            
            # Write second part
            with wave.open(output2, 'wb') as out2:
                out2.setparams(params)
                out2.writeframes(frames[split_byte:])
    
    def create_id3v23_tag(self, metadata: Dict) -> bytes:
        """Create ID3v2.3 tag with UTF-8 encoding"""
        frames = []
        
        # Text frames with UTF-8 encoding (encoding byte = 3)
        text_frames = {
            'TIT2': metadata.get('title', ''),      # Title
            'TPE1': metadata.get('artist', ''),     # Artist
            'TALB': metadata.get('album', ''),      # Album
            'TYER': str(metadata.get('year', '')),  # Year
            'TCON': metadata.get('genre', ''),      # Genre
            'TRCK': str(metadata.get('track-number', ''))  # Track number
        }
        
        for frame_id, text in text_frames.items():
            if text:
                # Encoding byte (3 = UTF-8) + text
                frame_data = b'\x03' + text.encode('utf-8')
                frame_size = len(frame_data)
                
                # Frame header: ID (4 bytes) + Size (4 bytes) + Flags (2 bytes)
                frame_header = (
                    frame_id.encode('ascii') +
                    struct.pack('>I', frame_size) +
                    b'\x00\x00'
                )
                frames.append(frame_header + frame_data)
        
        # Calculate total tag size
        frames_data = b''.join(frames)
        tag_size = len(frames_data)
        
        # Convert to synchsafe integer
        synchsafe_size = bytes([
            (tag_size >> 21) & 0x7F,
            (tag_size >> 14) & 0x7F,
            (tag_size >> 7) & 0x7F,
            tag_size & 0x7F
        ])
        
        # ID3v2.3 header: ID3 + version (2.3.0) + flags + size
        header = b'ID3\x03\x00\x00' + synchsafe_size
        
        return header + frames_data
    
    def add_id3_tag_to_wav(self, wav_path: str, output_path: str, 
                          metadata: Dict) -> None:
        """Add ID3v2.3 tag to WAV file"""
        # Create ID3 tag
        id3_tag = self.create_id3v23_tag(metadata)
        
        # Read WAV file
        with open(wav_path, 'rb') as f:
            wav_data = f.read()
        
        # Write ID3 tag + WAV data
        with open(output_path, 'wb') as f:
            f.write(id3_tag)
            f.write(wav_data)


def load_metadata(yaml_path: str) -> Dict:
    """Load metadata from YAML file"""
    with open(yaml_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def format_time(seconds: float) -> str:
    """Format seconds as mm:ss"""
    minutes = int(seconds // 60)
    secs = int(seconds % 60)
    return f"{minutes}:{secs:02d}"


def sanitize_filename(filename: str) -> str:
    """Replace invalid filename characters with '-'"""
    # Windows/Unix invalid filename characters
    invalid_chars = '<>:"/\\|?*'
    sanitized = filename
    for char in invalid_chars:
        sanitized = sanitized.replace(char, '-')
    # Also remove leading/trailing spaces and dots (Windows restriction)
    sanitized = sanitized.strip('. ')
    return sanitized


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
                return 1  # Default: Use as is
            choice = int(user_input)
            if 1 <= choice <= max_choice:
                return choice
            print(f"Invalid choice. Please enter 1-{max_choice}.")
        except ValueError:
            print("Invalid input. Please enter a number.")


def process_album(wav_dir: str, yaml_path: str, output_dir: str, tolerance: int = 3) -> None:
    """Main processing function"""
    # Load metadata
    metadata = load_metadata(yaml_path)
    
    processor = WavProcessor(tolerance_seconds=tolerance)
    
    # Get list of WAV files
    wav_files = sorted([f for f in os.listdir(wav_dir) if f.endswith('.wav')])
    if not wav_files:
        print("No WAV files found in the directory.")
        return
    
    print(f"Found {len(wav_files)} WAV files")
    print(f"Album: {metadata['album']}")
    print(f"Artist: {metadata['artist']}")
    print(f"Tolerance: ±{tolerance} seconds\n")
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Processing state
    current_wav_idx = 0
    temp_dir = Path(output_dir) / 'temp'
    os.makedirs(temp_dir, exist_ok=True)
    
    track_idx = 0
    while track_idx < len(metadata['tracks']):
        track = metadata['tracks'][track_idx]

        if current_wav_idx >= len(wav_files):
            print(f"\nWarning: No more WAV files for track {track['track-number']}")
            break

        track_title = track['title']
        expected_duration = processor.parse_time(track['time'])
        track_number = track['track-number']
        
        print(f"\nProcessing Track {track_number}: {track_title}")
        
        # Current WAV file
        current_wav_path = os.path.join(wav_dir, wav_files[current_wav_idx])
        actual_duration = processor.get_wav_duration(current_wav_path)
        diff = actual_duration - expected_duration
        
        print(f"  Expected: {track['time']} ({expected_duration}s)")
        print(f"  WAV file: {wav_files[current_wav_idx]} ({format_time(actual_duration)})")
        
        # Check if within tolerance
        if abs(diff) <= tolerance:
            # Within tolerance - use as is
            print(f"  ✓ Within tolerance (±{tolerance}s)")
            temp_wav = temp_dir / f"track_{track_number:02d}.wav"
            
            # Copy file
            with open(current_wav_path, 'rb') as src, open(temp_wav, 'wb') as dst:
                dst.write(src.read())
            
            current_wav_idx += 1
        else:
            # Outside tolerance - check if multiple tracks fit in this WAV
            multi_track_count = 0
            multi_track_time = ""

            if diff > 0:  # WAV is longer than expected
                # Try to find how many tracks fit in this WAV
                cumulative_duration = 0
                for i in range(track_idx, len(metadata['tracks'])):
                    cumulative_duration += processor.parse_time(metadata['tracks'][i]['time'])
                    if abs(actual_duration - cumulative_duration) <= tolerance:
                        multi_track_count = i - track_idx + 1
                        multi_track_time = format_time(cumulative_duration)
                        break
                    elif cumulative_duration > actual_duration + tolerance:
                        break

            # Ask user
            choice = get_user_choice(
                track_title,
                track['time'],
                format_time(actual_duration),
                diff,
                is_longer=(diff > 0),
                multi_track_count=multi_track_count,
                multi_track_time=multi_track_time
            )
            
            if choice == 1:
                # Use as is
                print("  → Using current WAV as is")
                temp_wav = temp_dir / f"track_{track_number:02d}.wav"
                with open(current_wav_path, 'rb') as src, open(temp_wav, 'wb') as dst:
                    dst.write(src.read())
                current_wav_idx += 1
                
            elif choice == 2:
                if diff > 0:
                    # Split file
                    print(f"  → Splitting at {track['time']}")
                    temp_wav = temp_dir / f"track_{track_number:02d}.wav"
                    remainder = temp_dir / f"remainder_{current_wav_idx}.wav"
                    
                    processor.split_wav_file(
                        current_wav_path,
                        expected_duration,
                        str(temp_wav),
                        str(remainder)
                    )
                    
                    # Insert remainder for next iteration (use absolute path)
                    wav_files.insert(current_wav_idx + 1, str(remainder.resolve()))
                    current_wav_idx += 1
                else:
                    # Merge with next file
                    if current_wav_idx + 1 >= len(wav_files):
                        print("  ! No next file to merge. Skipping.")
                        current_wav_idx += 1
                        continue
                    
                    print(f"  → Merging with next WAV file")
                    next_wav_path = os.path.join(wav_dir, wav_files[current_wav_idx + 1])
                    temp_wav = temp_dir / f"track_{track_number:02d}.wav"
                    
                    processor.merge_wav_files(
                        [current_wav_path, next_wav_path],
                        str(temp_wav)
                    )
                    current_wav_idx += 2
                    
            elif choice == 4:
                # Combine multiple tracks into one WAV
                print(f"  → Combining {multi_track_count} tracks into one WAV")
                temp_wav = temp_dir / f"track_{track_number:02d}.wav"

                # Copy file
                with open(current_wav_path, 'rb') as src, open(temp_wav, 'wb') as dst:
                    dst.write(src.read())

                # Create combined metadata (use combined track titles)
                combined_titles = [metadata['tracks'][track_idx + i]['title']
                                 for i in range(multi_track_count)]
                track_title = " / ".join(combined_titles)

                # Advance track_idx by the number of combined tracks - 1
                # (the loop increment will add 1 more at the end)
                track_idx += multi_track_count - 1
                current_wav_idx += 1
            else:
                # Skip
                print("  → Skipping track")
                current_wav_idx += 1
                track_idx += 1
                continue
        
        # Add ID3 tag
        safe_title = sanitize_filename(track_title)

        # Truncate filename if too long
        # Format: "{track_number:02d} - {safe_title}.wav"
        # Fixed parts: "XX - " (5 chars) + ".wav" (4 chars) = 9 chars
        max_title_length = MAX_FILENAME_LENGTH - 9
        if len(safe_title) > max_title_length:
            safe_title = safe_title[:max_title_length].rstrip()

        output_filename = f"{track_number:02d} - {safe_title}.wav"
        output_path = os.path.join(output_dir, output_filename)

        # Truncate ID3 title if too long
        id3_title = track_title
        if len(id3_title) > MAX_ID3_TITLE_LENGTH:
            id3_title = id3_title[:MAX_ID3_TITLE_LENGTH].rstrip()

        track_metadata = {
            'title': id3_title,
            'artist': track.get('artist', metadata['artist']),
            'album': metadata['album'],
            'year': metadata['year'],
            'genre': metadata['genre'],
            'track-number': track_number
        }
        
        processor.add_id3_tag_to_wav(str(temp_wav), output_path, track_metadata)
        print(f"  ✓ Saved: {output_filename}")

        track_idx += 1
    
    # Cleanup temp directory
    import shutil
    shutil.rmtree(temp_dir)
    
    print(f"\n{'='*60}")
    print("Processing complete!")
    print(f"Output directory: {output_dir}")
    print(f"{'='*60}")


if __name__ == '__main__':
    import sys
    
    if len(sys.argv) < 4 or len(sys.argv) > 5:
        print("Usage: python wav_tagger.py <wav_directory> <metadata.yaml> <output_directory> [tolerance_seconds]")
        print("\nArguments:")
        print("  wav_directory      : Directory containing input WAV files")
        print("  metadata.yaml      : YAML file with album metadata")
        print("  output_directory   : Directory for output files")
        print("  tolerance_seconds  : Optional. Tolerance in seconds (default: 3)")
        print("\nExample:")
        print("  python wav_tagger.py ./input ./album.yaml ./output")
        print("  python wav_tagger.py ./input ./album.yaml ./output 5")
        sys.exit(1)
    
    wav_directory = sys.argv[1]
    yaml_file = sys.argv[2]
    output_directory = sys.argv[3]
    tolerance_seconds = int(sys.argv[4]) if len(sys.argv) == 5 else 3
    
    if not os.path.isdir(wav_directory):
        print(f"Error: WAV directory not found: {wav_directory}")
        sys.exit(1)
    
    if not os.path.isfile(yaml_file):
        print(f"Error: YAML file not found: {yaml_file}")
        sys.exit(1)
    
    process_album(wav_directory, yaml_file, output_directory, tolerance_seconds)
