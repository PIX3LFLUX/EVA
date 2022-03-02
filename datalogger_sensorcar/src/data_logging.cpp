#include "data_logging.h"
DRAM_ATTR File log_file;

RTC_DATA_ATTR uint32_t readingID = 0; /* counter. Increments every time a line is appended to the log. Is used to close and reopen the file to save progress. The file can't be opened and closed every line, it takes too much time. */

/* Append data to the SD card */
IRAM_ATTR void appendFile(fs::FS &fs, const char * path, const char * message)
{
  if (xSemaphoreTake(sd_card_access_semaphore,0) == pdTRUE)
  {
    if (!log_file)
    {
      log_file = fs.open(path, FILE_APPEND);
    }

    #if DEBUG
      bool print_success = log_file.print(message);
      log_file.close();
      Serial.printf("Appending to file: %s\n", path);
      if(print_success)
      {
        Serial.println("Message appended");
      } 
      else
      {
        Serial.println("Append failed");
      }
    #else
      log_file.print(message);
      #if (MEASURE_SYSTEM == MEASURE_MODE_MANUAL) /* free running sampling, there is no event to stop capture, so it is stopped periodically */
        readingID += 1;
        if ((readingID % CLOSE_FILE_SAMPLE_NO) == 0)
        {
          readingID = 0;
          log_file.close();
        }
      #endif
    #endif
    xSemaphoreGive(sd_card_access_semaphore);
  }
}

/* Write to the SD card */
IRAM_ATTR void writeFile(fs::FS &fs, const char * path, const char * message)
{
  if (xSemaphoreTake(sd_card_access_semaphore,0) == pdTRUE)
  {
    if (!log_file)
    {
      log_file = fs.open(path, FILE_WRITE);
    }
    log_file.print(message);
    log_file.close();

    #if DEBUG
      Serial.printf("Writing file: %s\n", path);
    #endif
    xSemaphoreGive(sd_card_access_semaphore);
  }
}

void init_SD()
{
  SD.begin(SD_CS);
  if(!SD.begin(SD_CS))
  {
    Serial.println("Card Mount Failed");
    ESP.restart();
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    ESP.restart();
  }
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS))
  {
    Serial.println("ERROR - SD card initialization failed!");
    ESP.restart();
  }
  xSemaphoreGive(sd_card_access_semaphore);
}

IRAM_ATTR void init_log_file(const char* log_file_header)
{
  if (xSemaphoreTake(sd_card_access_semaphore,portMAX_DELAY) == pdTRUE)
  {
    log_file = SD.open(LOG_FILE_NAME);
    if (!log_file)
    {
      log_file.close();
      xSemaphoreGive(sd_card_access_semaphore);
      Serial.println("File doesn't exist. Creating it.");
      writeFile(SD, LOG_FILE_NAME, log_file_header);
    }
    else
    {
      log_file.close();
      xSemaphoreGive(sd_card_access_semaphore);
      Serial.println("File already exists.");
      appendFile(SD, LOG_FILE_NAME, log_file_header);
    }
    Serial.println("Writing header.");
    log_file.close();
    xSemaphoreGive(sd_card_access_semaphore);
  }
}

IRAM_ATTR void close_log_file()
{
  if (xSemaphoreTake(sd_card_access_semaphore,portMAX_DELAY) == pdTRUE)
  {
    log_file.close();
    xSemaphoreGive(sd_card_access_semaphore);
  }
}