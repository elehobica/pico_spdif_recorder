/*------------------------------------------------------/
/ Copyright (c) 2026, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include "rec_folder.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <strings.h>

#include "wav_header.h"

/*-----------------/
/  Class variables
/-----------------*/
DIR     rec_folder::_dir;
FILINFO rec_folder::_fno;
char    rec_folder::_path[rec_folder::PATH_SIZE];

/*------------------------/
/  Public class functions
/------------------------*/
bool rec_folder::setup(int& folder_num)
{
    // record in a new folder for each boot to keep the directory search by f_open short
    // (FATFS searches directory entries including deleted ones until it reaches never used entry,
    //  therefore deleting files in the root directory doesn't shorten the search without format)
    std::map<int, bool> found;  // number -> true: folder, false: file
    FRESULT fr;

    // read root directory only once
    fr = f_opendir(&_dir, "/");
    if (fr != FR_OK) {
        printf("ERROR: failed to open root directory (%d)\r\n", fr);
        return false;
    }
    while (true) {
        fr = f_readdir(&_dir, &_fno);
        if (fr != FR_OK || _fno.fname[0] == '\0') break;
        int num;
        if (_parse_num(_fno.fname, num)) {
            found[num] = (_fno.fattrib & AM_DIR) != 0;
        }
    }
    f_closedir(&_dir);
    if (fr != FR_OK) {
        printf("ERROR: failed to read root directory (%d)\r\n", fr);
        return false;
    }

    // find the first unused number and the last folder before it
    int n = 1;
    int last_folder = 0;
    for (auto it = found.find(n); it != found.end(); it = found.find(++n)) {
        if (it->second) last_folder = n;
    }

    // recreate the last folder if empty to clear deleted entries in it, otherwise make a new folder
    char name[FOLDER_NAME_SIZE];
    bool made = false;
    if (last_folder > 0) {
        snprintf(name, sizeof(name), "%03d", last_folder);
        if (_is_empty(name)) {
            if (_remove(name) && _make(last_folder, folder_num)) {
                printf("recreated empty folder \"%s\" to record\r\n", name);
                made = true;
            } else {
                printf("WARNING: failed to recreate empty folder \"%s\"\r\n", name);
            }
        }
    }
    if (!made) {
        if (!_make(n, folder_num)) {
            printf("ERROR: failed to create folder \"%03d\"\r\n", n);
            return false;
        }
        printf("created folder \"%03d\" to record\r\n", folder_num);
    }

    // change current directory to the folder to open files by relative path without searching root directory
    // (current directory is common to both cores and must not be changed after core1 starts)
    snprintf(name, sizeof(name), "%03d", folder_num);
    fr = f_chdir(name);
    if (fr != FR_OK) {
        printf("ERROR: failed to change directory to \"%s\" (%d)\r\n", name, fr);
        return false;
    }
    return true;
}

/*--------------------------/
/  Protected class functions
/--------------------------*/
bool rec_folder::_parse_num(const char* name, int& num)
{
    // accept only the name exactly same as "%03d" (e.g. "001", "999", "1000", but not "0004" or "4")
    const size_t len = strlen(name);
    if (len < 3 || len > MAX_NUM_DIGITS) return false;
    for (size_t i = 0; i < len; i++) {
        if (name[i] < '0' || name[i] > '9') return false;
    }
    num = atoi(name);
    char buf[FOLDER_NAME_SIZE];
    snprintf(buf, sizeof(buf), "%03d", num);
    return num > 0 && strcmp(buf, name) == 0;
}

bool rec_folder::_is_wav_name(const char* name)
{
    const size_t len = strlen(name);
    return len >= 4 && strcasecmp(&name[len - 4], ".wav") == 0;
}

bool rec_folder::_is_empty(const char* folder)
{
    // empty: only WAV files of header size or less (e.g. left by power off during standby)
    FRESULT fr = f_opendir(&_dir, folder);
    if (fr != FR_OK) return false;
    bool empty = true;
    while (true) {
        fr = f_readdir(&_dir, &_fno);
        if (fr != FR_OK) {
            empty = false;
            break;
        }
        if (_fno.fname[0] == '\0') break;
        if ((_fno.fattrib & AM_DIR) || !_is_wav_name(_fno.fname) || _fno.fsize > wav_header::RF64_RESERVED_HEADER_SIZE) {
            empty = false;
            break;
        }
    }
    f_closedir(&_dir);
    return empty;
}

bool rec_folder::_remove(const char* folder)
{
    // remove files in the folder (no sub folder because it's empty) and the folder itself
    FRESULT fr = f_opendir(&_dir, folder);
    if (fr != FR_OK) return false;
    while (true) {
        fr = f_readdir(&_dir, &_fno);
        if (fr != FR_OK || _fno.fname[0] == '\0') break;
        snprintf(_path, sizeof(_path), "%s/%s", folder, _fno.fname);
        fr = f_unlink(_path);
        if (fr != FR_OK) break;
    }
    f_closedir(&_dir);
    if (fr != FR_OK) return false;
    return f_unlink(folder) == FR_OK;
}

bool rec_folder::_make(const int num, int& folder_num)
{
    char name[FOLDER_NAME_SIZE];
    snprintf(name, sizeof(name), "%03d", num);
    if (f_mkdir(name) != FR_OK) return false;
    folder_num = num;
    return true;
}
