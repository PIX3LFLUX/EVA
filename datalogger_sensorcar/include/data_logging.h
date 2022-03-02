#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include "globals.h"

#define SD_CS                   13   /* ESP32 pin for CS pin of SD card */
#define LOG_FILE_NAME           "/data.txt"
#define CLOSE_FILE_SAMPLE_NO    100 /* close and reopen file every CLOSE_FILE_SAMPLE_NO samples to save them to the SD card. This is slow. */

IRAM_ATTR void appendFile(fs::FS &fs, const char * path, const char * message);
IRAM_ATTR void writeFile(fs::FS &fs, const char * path, const char * message);
void init_SD();
IRAM_ATTR void init_log_file(const char* log_file_header);
IRAM_ATTR void close_log_file();