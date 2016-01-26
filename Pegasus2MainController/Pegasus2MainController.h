
#ifndef TRUE
#define TRUE                             (1==1)
#endif

#ifndef FALSE
#define FALSE                            (1==2)
#endif

#define TEST_TELEMETRY                    TRUE

#define DEBUG_PRINT                        TRUE

#define ADDRESS_SUBPROC_3                  0x06
#define ADDRESS_SUBPROC_1                  0x04
#define PACKET_LENGTH                        32
#define SIZES_PACKET_LENGTH                   3
                                          
#define CRAFT_NOTE_COUNT                     17
#define CRAFT_NOTE_TEXT_LENGTH               60
#define LIFTOFF_POS                           0
#define SAFE_MODE_POS                         1
#define ABOVE_TRAFFIC_POS                     2
#define CURVATURE_POS                         3
#define GOAL_ALTITUDE_POS                     4
#define BALLOON_RELEASED_POS                  5
#define PARACHUTE_DEPLOYED_POS                6
#define SEE_MY_HOUSE_POS                      7
#define STRATOSPHERE_POS                      8
#define DIVING_POS                            9
#define PLUMMETING_POS                       10
#define SCREAMING_POS                        11
#define MINUTES_TO_AUTORELEASE_POS           12
#define AUTODEPLOY_HEIGHT_POS                13
#define LIGHTS_ON_POS                        14
#define LIGHTS_OFF_POS                       15
#define CRAFT_ARMED_POS                      16
                                          
#define CRAFT_MESSAGE_NOT_SENT                0
#define CRAFT_MESSAGE_SENT                    1
#define CRAFT_MESSAGE_MULTISEND            0xff
                                          
#define MESSAGE_NO_VALUE                    -99


#define PROC2_DATA_POS_AIR_PRESSURE           0
#define PROC2_DATA_POS_PRESSURE_TEMP          1
#define PROC2_DATA_POS_HUMIDITY               2
#define PROC2_DATA_POS_THERMOCOUPLE_TEMP      3
#define PROC2_DATA_POS_ACCEL_X                4
#define PROC2_DATA_POS_ACCEL_Y                5
#define PROC2_DATA_POS_ACCEL_Z                6
#define PROC2_DATA_POS_GYRO_X                 7
#define PROC2_DATA_POS_GYRO_Y                 8
#define PROC2_DATA_POS_GYRO_Z                 9
#define PROC2_DATA_POS_MAG_X                 10
#define PROC2_DATA_POS_MAG_Y                 11
#define PROC2_DATA_POS_MAG_Z                 12
#define PROC2_DATA_TMP36                     13


#define PROC1_DATA_POS_TEMP_IN                0
#define PROC1_DATA_POS_TEMP_OUT               1
#define PROC1_DATA_POS_BALLOON_RELAY          2
#define PROC1_DATA_POS_AUX_RELAY              3
#define PROC1_DATA_POS_BATTERY_MAIN           4
#define PROC1_DATA_POS_BATTERY_AUX            5
#define PROC1_DATA_POS_REFERENCE_LEVEL        6
#define PROC1_DATA_POS_UV_LEVEL               7
#define PROC1_DATA_POS_CAMERA_UP              8
#define PROC1_DATA_POS_GEIGER_CPS             9
#define PROC1_DATA_POS_GEIGER_CPM            10
#define PROC1_DATA_POS_GEIGER_CPH            11

#define PROC1_COMMAND_GET_TELEMETRY           0
#define PROC1_COMMAND_RELEASE_BALLOON         1
#define PROC1_COMMAND_DEPLOY_PARACHUTE        2
#define PROC1_COMMAND_POSITION_CAMERA_OUT     3
#define PROC1_COMMAND_POSITION_CAMERA_UP      4
#define PROC1_COMMAND_RESET_DATA              5

#define PROC3_COMMAND_LEDS_ON                 1
#define PROC3_COMMAND_LEDS_OFF                2
#define PROC3_SEND_THANK_MSR                  3
#define PROC3_SEND_PEOPLE_OF_EARTH            4
#define PROC3_SEND_FLIGHT_TEAM                5


// wiringPi Pins
#define GPIO_PIN_SAFETY                       3  // BCM Pin 22
#define GPIO_PIN_BALLOON                     26  // BCM Pin 12
#define GPIO_PIN_PARACHUTE                   22  // BCM Pin 6

#define PIN_REMOVED_INDICATED                 1
#define PIN_INSERTED_INDICATED                0


#define DATA_ARRAY_COUNT                     39
#define DATA_ARRAY_STR_LEN                   16
#define PROC2_DATA_COUNT                     14
#define PROC1_DATA_COUNT                     12

#define ALTITUDE_STRATOSPHERE             13106   // 43,000 ft.
#define ALTITUDE_ABOVE_TRAFFIC            12192   // 40,000 ft.
#define ALTITUDE_I_CAN_SEE                15240   // 50,000 ft.
#define ALTITUDE_CURVATURE                16764   // 65,000 ft.
#define ALTITUDE_GOAL                     30480   // 100,000 ft.


#define DEFAULT_SECONDS_TILL_RELEASE (150 * 60)   // 2.5 hours
#define DEFAULT_ALTITUDE_FOR_DEPLOY        2000   // meters above launch height
#define DEFAULT_ALTITUDE_FOR_ARMING        3000   // meters above launch height

#define FALL_LIMIT_SPEED                (-12.5)   // Speed the balloon must achieve to indicate a fall
#define DEFAULT_PARACHUTE_DECENT_RATE       5.0
#define DEFAULT_ASCENT_RATE					5.0

#define MPH_PER_KNOT          1.150779448023543
#define KPH_PER_KNOT                      1.852

#define DO_SPEED_IN_MPH                   FALSE          
#define DO_SPEED_IN_KPH                    TRUE        

#define TELEMETRY_DATA_LEN                  256
#define GENERAL_BUFFER_LEN                  128
#define GPS_DATA_BUFFER_LEN                  64


#define TELEMETRY_LOG_FILE					"/home/pi/PegasusMission/telemetryLog.txt"
#define IS_GPS								  1
#define IS_PROC2							  2