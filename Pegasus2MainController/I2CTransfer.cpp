#include "I2CTransfer.h"
#include <stdint.h>


I2CTransfer::I2CTransfer(uint8_t i2c_address) {
    
    if (i2c_address == 0) {
        printf("Error: I2C Address cannot be 0");
    }

    _i2c_address = i2c_address;
}


uint8_t* I2CTransfer::get_i2c_data_packet(uint8_t* buffer, int length) {

    if (length > MAX_I2C_DATA_LENGTH) return NULL;

    if (buffer == NULL) return NULL;

    if ((_file = open(DEVICE_NAME, O_RDWR)) < 0) {
        fprintf(stderr, "I2C: Failed to access %d\n", DEVICE_NAME);
        return(NULL);
    }

    //printf("I2C: acquiring bus to 0x%x\n", _i2c_address);

    if (ioctl(_file, I2C_SLAVE, _i2c_address) < 0) {
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", _i2c_address);

        close(_file);
        return(NULL);
    }

    read(_file, buffer, length);

    usleep(10000);

    close(_file);

    return (buffer);
}




char* I2CTransfer::get_i2c_ascii_string(char* buffer, int length, int8_t* success ) {

    if ((_file = open(DEVICE_NAME, O_RDWR)) < 0) {
        fprintf(stderr, "I2C: Failed to access %d\n", DEVICE_NAME);
        return (NULL);
    }

        //printf("I2C: acquiring bus to 0x%x\n", _i2c_address);

    if (ioctl(_file, I2C_SLAVE, _i2c_address) < 0) {
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", _i2c_address);

        close(_file);
        return (NULL);
    }

    int readCount = length;
    int bytesRead = 0;
    int bufferPosition = 0;
    int byteCountRead = 0;
    int totalByteCountRead = 0;

    *success = I2C_TRANSFER_SUCCESS;
    
    if (length > MAX_I2C_DATA_LENGTH) {

        readCount = SENSOR_DATA_LENGTH;

        while (bufferPosition < length) {

            byteCountRead = read(_file, &buffer[bufferPosition], readCount);
    
            totalByteCountRead += byteCountRead;
            
            bytesRead += readCount;

            bufferPosition += SENSOR_DATA_LENGTH;

            if (bytesRead + SENSOR_DATA_LENGTH > length) {
                readCount = length - bufferPosition;
            }

            usleep(10000);
        }
    }
    else {
        byteCountRead = read(_file, buffer, readCount);
        
        totalByteCountRead += byteCountRead;
    }

    buffer[length] = '\0';

    // data returned the right size?
    if (totalByteCountRead != length)
    {
        *success = I2C_TRANSFER_LENGTH_MISMATCH;
    }
    
    // check for non-printable characters
    if (*success == I2C_TRANSFER_SUCCESS)
    {
        for (int i = 0; i < length; i++)
        {
            if ((buffer[i] < 32) || (buffer[i] > 127))
            {
                *success = I2C_TRANSFER_NON_PRINTABLE;
                break;
            }
        }
    }

    usleep(10000);

    close(_file);

    return (buffer);
}



char* I2CTransfer::get_i2c_string(char* buffer, int length) {

    if ((_file = open(DEVICE_NAME, O_RDWR)) < 0) {
        fprintf(stderr, "I2C: Failed to access %d\n", DEVICE_NAME);
        return(NULL);
    }

    //printf("I2C: acquiring bus to 0x%x\n", _i2c_address);

    if (ioctl(_file, I2C_SLAVE, _i2c_address) < 0) {
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", _i2c_address);

        close(_file);
        return(NULL);
    }

    int readCount = length;
    int bytesRead = 0;
    int bufferPosition = 0;
    int byteCountRead = 0;
    int totalByteCountRead = 0;

    if (length > MAX_I2C_DATA_LENGTH) {

        readCount = SENSOR_DATA_LENGTH;

        while (bufferPosition < length) {

            byteCountRead = read(_file, &buffer[bufferPosition], readCount);
    
            totalByteCountRead += byteCountRead;
            
            bytesRead += readCount;

            bufferPosition += SENSOR_DATA_LENGTH;

            if (bytesRead + SENSOR_DATA_LENGTH > length) {
                readCount = length - bufferPosition;
            }

            usleep(10000);
        }
    }
    else {
        byteCountRead = read(_file, buffer, readCount);
        
        totalByteCountRead += byteCountRead;
    }
    
    buffer[length] = '\0';

    usleep(10000);

    close(_file);

    return(buffer);
}


void I2CTransfer::send_user_text(char* userText) {

    int totalLength;

    if (userText == NULL) {
        return;
    }

    int len = strlen(userText);

    if ((len == 0) || (len > USER_MSG_MAX_LEN)) {
        return;
    }

    if ((_file = open(DEVICE_NAME, O_RDWR)) < 0) {
        fprintf(stderr, "I2C: Failed to access %s\n", DEVICE_NAME);
        return;
    }

    if (ioctl(_file, I2C_SLAVE, _i2c_address) < 0) {
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", _i2c_address);
        close(_file);
    }

    int charPosition = 0;
    int dataLength;
    char userTextLen = (char) len;

    buf[0] = userTextLen;
    strcpy(&buf[1], userText);

    totalLength = userTextLen + 1;

    while (totalLength > MAX_I2C_DATA_LENGTH) {

        dataLength = MAX_I2C_DATA_LENGTH;

        if (totalLength == (MAX_I2C_DATA_LENGTH + 1)) {
            dataLength--;
        }

        write(_file, &buf[charPosition], dataLength);
        charPosition += dataLength;
        totalLength -= dataLength;

        usleep(10000);
    }

    if (totalLength) {
        write(_file, &buf[charPosition], totalLength);
        usleep(10000);
    }


    close(_file);

}



void I2CTransfer::send_command(uint8_t command) {

    //printf("I2C: Connecting\n");
    char cmdBuf[2];

    if ((_file = open(DEVICE_NAME, O_RDWR)) < 0) {
        fprintf(stderr, "I2C: Failed to access %s\n", DEVICE_NAME);
        return;
    }

    //printf("I2C: acquiring bus to 0x%x\n", _i2c_address);

    if (ioctl(_file, I2C_SLAVE, _i2c_address) < 0) {
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", _i2c_address);
        close(_file);
    }

    cmdBuf[0] = command;
    cmdBuf[1] = '\0';

    write(_file, cmdBuf, 1);
    usleep(10000);

    close(_file);
}