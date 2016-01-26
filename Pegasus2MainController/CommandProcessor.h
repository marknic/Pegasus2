#define GPSECHO                                         false
#define CLOCK_ADDRESS                                    0x68

#define TEMP_SENSOR_PIN                                     0
#define RADIO_SIGNAL_STRENGTH_PIN                           6

#define WRITE_DATA_TO_STORAGE                           false

#define SERIAL_BAUD_RATE                               115200
#define GPS_BAUD_RATE                                    9600

#define COMMAND_MSG_MAX_SIZE                               36
#define RADIO_MSG_MAX_SIZE                                256
#define RADIO_MSG_MAX_OFFSET         (RADIO_MSG_MAX_SIZE - 1)
#define RADIO_MSG_OFFSET_INIT                              -1

#define TELEMETRY_INDICATOR_CHAR                          '$'
#define TELEMETRY_INDICATOR_CHAR_STATION                  '#'
#define COMMAND_INDICATOR_CHAR                            '{'
#define MSG_FROM_CRAFT_INDICATOR_CHAR                     '}'

#define MESSAGE_END_DELIMITER                            '\n'
#define MESSAGE_END_DELIMITER_BACKSLASH                  '\\'
#define MESSAGE_END_DELIMITER_VERTBAR                     '|'

#define POS_LAT                                            23
#define POS_LON                                            24
#define POS_ALT                                            25

#define COMMAND_PREFIX_SIZE                                 3
#define COMMAND_ID_POSITION                                 1

#define GPS_EEPROM_ADDRESS                                  0

#define COMMAND_SUCCESS                                     0
#define COMMAND_ERROR                                      -1

#define MINUTES_ALOFT_MAX                                 120

#define USER_MESSAGE_SIZE_MAX                              40