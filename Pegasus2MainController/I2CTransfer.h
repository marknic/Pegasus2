#ifndef I2C_TRANSFER_H

#include <stdint.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>  // open
#include <unistd.h>  // read / usleep / write / close
#include <stdio.h>  // printf / fprintf
#include <string.h>  // strlen / strcpy

#define I2C_TRANSFER_H


// The I2C bus: This is for V2 pi's. For V1 Model B you need i2c-0
#define DEVICE_NAME                   "/dev/i2c-1"
#define SENSOR_DATA_LENGTH                      32
#define SENSOR_DATA_TOTAL_LENGTH                64
#define MAX_I2C_DATA_LENGTH                     32
#define USER_MSG_MAX_LEN                        60

class I2CTransfer
{
private:
    uint8_t _i2c_address;
    char buf[USER_MSG_MAX_LEN];
    int _file;
public:
    I2CTransfer(uint8_t _i2c_address);
    //~I2CTransfer();

    char* get_i2c_string(char* buffer, int length);
    uint8_t* get_i2c_data_packet(uint8_t* buffer, int length);
    void send_command(uint8_t command);
    void send_user_text(char* userText);

};

#endif