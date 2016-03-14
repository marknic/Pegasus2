#include <iostream>
#include <cstdlib>
#include <string.h>
#include <cstdio>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

# include <stdlib.h>
#include "UartStream.h"

using namespace std;

#ifndef TRUE
#define TRUE                             (1==1)
#endif

#ifndef FALSE
#define FALSE                            (1==2)
#endif

#define ROW_COUNT																6
#define ROW_LENGTH                                                             16
#define MAX_WORD_COUNT                                                         20
#define DATA_ARRAY_STR_LEN                                       (ROW_LENGTH + 2)
#define USER_MSG_MAX_LEN                                                       80
#define USER_MSG_ADJUSTED_MAX_LEN                         (USER_MSG_MAX_LEN + 20)
#define MESSAGE_FILE                                "/home/pi/epaper/message.txt"

#define TELEMETRY_LOG_FILE		       "/home/pi/PegasusMission/telemetryLog.txt"


void process_message_data(char* dataIn);
void format_text_for_display(char* stringToDisplay);
void write_to_log(const char* source, char* data);

int _output_row_count = 0;

char words[MAX_WORD_COUNT][DATA_ARRAY_STR_LEN];
char rows[ROW_COUNT][DATA_ARRAY_STR_LEN];

//UartStream _serialStream_Main('0', process_message_data, FALSE, "MsgDisplay 1");
UartStream* _serialStream_Main;

time_t _current_time;


void process_message_data(char* dataIn)
{
    printf("Calling external Python script...\n");
    printf("Received user message: '%s'\n", dataIn);
    
    if (dataIn == NULL) return;

    int len = strlen(dataIn);
    
    while ((len > 0) && ((dataIn[len - 1] == ' ') || (dataIn[len - 1] == '\r') || (dataIn[len - 1] == '\n') || (dataIn[len - 1] == '\t')))
    {
        dataIn[len - 1] = '\0';
        
        len = strlen(dataIn);
    }
    
    char msgOut[1024];
    
    sprintf(msgOut, "Received user message: '%s'\n", dataIn);
    write_to_log("DATA", msgOut);
    
    
    
    format_text_for_display(dataIn);	
    
    std::string filename = "/home/pi/epaper/pegasus/epaper_displayfile.py";
    std::string command = "python ";
    command += filename;
    system(command.c_str());
    
}

char* format_timestamp_with_milliseconds(char* timestampDest)
{
    if (timestampDest == NULL) return NULL;
    
    struct timeval timeNow;

    int mtime;    

    gettimeofday(&timeNow, NULL);

    mtime = timeNow.tv_usec / 1000.0 + 0.5;
    
    _current_time = time(NULL);
    
    struct tm tm = *gmtime(&_current_time);

    sprintf(timestampDest, 
        "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ", 
        tm.tm_year + 1900,
        tm.tm_mon + 1,
        tm.tm_mday, 
        tm.tm_hour,
        tm.tm_min,
        tm.tm_sec,
        mtime);

    return timestampDest;
}

void write_to_log(const char* source, char* data) {

    char timestamp[36];
    char telemetry_string[256];

    FILE* fp;
    
    if ((source == NULL) || (data == NULL)) return;
    
    /* open the file for append */
    fp = fopen(TELEMETRY_LOG_FILE, "a");
    
    if (fp == NULL) {
        printf("I couldn't open log for appending.\n");
        
        return;
    }

    format_timestamp_with_milliseconds(timestamp);

    fprintf(fp, "%s,%s,%s", source, timestamp, data);

    fflush(fp);
    
    /* close the file */
    fclose(fp);
}


int build_rows(int wordCount) {

    int rowIndex = 0;
    int rowLetterCount = 0;
    int wordLen;
    int i;
       
    // Clear the rows
    for (i = 0; i < ROW_COUNT; i++) {
        rows[i][0] = '\0';
        memset((void*)&rows[i][0], 0, DATA_ARRAY_STR_LEN);
    }

    for (i = 0; i < wordCount; i++) {
        wordLen = strlen(words[i]);

        if (wordLen < ROW_LENGTH) {
            if ((wordLen + rowLetterCount) > ROW_LENGTH) {
                rowLetterCount = 0;
                rowIndex++;
            }

            strcpy(&rows[rowIndex][rowLetterCount], words[i]);

            rowLetterCount += wordLen;
            rows[rowIndex][rowLetterCount] = ' ';
            rowLetterCount++;
        }
    }

    return rowIndex + 1;
}


void draw_rows() {

    int i;
    
    FILE* fd = fopen(MESSAGE_FILE, "w");
    
    if (fd == NULL) {
        printf("Couldn't open file for writing.\n");
        
        return;
    }

    if (_output_row_count)
    {
        for (i = 0; i < _output_row_count; i++)
        {
            int rowLen = strlen(rows[i]);
            
            for (int j = rowLen; j < DATA_ARRAY_STR_LEN - 1; j++)
            {
                rows[i][j] = ' ';
            }
            
            rows[i][DATA_ARRAY_STR_LEN - 1] = '\0';
            
            printf("%s\n", rows[i]);
        
            fprintf(fd, "%s\n", rows[i]);
        }
    }
    else
    {
        fprintf(fd, "\n");
    }
        
    fflush(fd);
    
    /* close the file */
    fclose(fd);		
}



int strsplit(const char* str, const char* delim, char dataArray[][DATA_ARRAY_STR_LEN]) {
    // copy the original string so that we don't overwrite parts of it
    // (don't do this if you don't need to keep the old line,
    // as this is less efficient)
    char *s = strdup(str);

        // these three variables are part of a very common idiom to
        // implement a dynamically-growing array
    u_int32_t tokens_alloc = 1;
    u_int32_t tokens_used = 0;
    
    char **tokens = (char**) calloc(tokens_alloc, sizeof(char*));

    char *token, *strtok_ctx;
    for (token = strtok_r(s, delim, &strtok_ctx); token != NULL; token = strtok_r(NULL, delim, &strtok_ctx)) 
    {
        // check if we need to allocate more space for tokens
        if (tokens_used == tokens_alloc) {
            tokens_alloc *= 2;
            tokens = (char**) realloc(tokens, tokens_alloc * sizeof(char*));
        }
        
        tokens[tokens_used++] = strdup(token);
    }

        // cleanup
    if (tokens_used == 0) {
        free(tokens);
        tokens = NULL;
    
    }
    else {
        tokens = (char**) realloc(tokens, tokens_used * sizeof(char*));
    }
    
    free(s);
    
    for (u_int32_t i = 0; i < tokens_used; i++) 
    {
        strcpy(dataArray[i], tokens[i]);
        
        printf("    token: \"%s\"\n", tokens[i]);
        free(tokens[i]);
    }
    
    if (tokens != NULL)
    {
        free(tokens);
    }

    return tokens_used;
}


void format_text_for_display(char* stringToDisplay) {

    int count = 0;
    int tmpLength = strlen(stringToDisplay);
    _output_row_count = 0;
    
    if (tmpLength > 0)
    {
        if (stringToDisplay)
        {
            count = strsplit(stringToDisplay, " ", words);
                    
            _output_row_count = build_rows(count);
        }
    }
    
    draw_rows();
}


int main(int argc, char *argv[])
{
    int received0;
    
    _serialStream_Main = new UartStream('0', process_message_data, FALSE, "MsgDisplay 1");
    
    printf("Pegasus2EpaperDisplay Running...\n");
    
    write_to_log("APP", "Starting Application...\n");

    while (TRUE) {
        received0 = _serialStream_Main->uart_receive();

        usleep(1000);
    }
    
    return 0;
}

