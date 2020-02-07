#define FRSKY_POLL_HEADER                     0x7E

/**
 * Request poll IDs
 */

#define FRSKY_POLL_ID_VARIO                   0x00 // [1] Vari-H (altimeter high precision)
#define FRSKY_POLL_ID_FLVSS                   0xA1 // [2] FLVSS / MLVSS (LiPo)
#define FRSKY_POLL_ID_FAS                     0x22 // [3] FAS (current)
#define FRSKY_POLL_ID_GPS                     0x83 // [4] GPS / Vari-N (altimeter normal precision)
#define FRSKY_POLL_ID_RPM                     0xE4 // [5] RPM
#define FRSKY_POLL_ID_6                       0x45 // [6] SP2UH
#define FRSKY_POLL_ID_SP2R                    0xC6 // [7] SP2UR
#define FRSKY_POLL_ID_8                       0x67 // [8] -
#define FRSKY_POLL_ID_9                       0x48 // [9] -
#define FRSKY_POLL_ID_ASS                     0xE9 // [10] ASS (air speed)
#define FRSKY_POLL_ID_11                      0x6A // [11] -
#define FRSKY_POLL_ID_12                      0xCB // [12] -
#define FRSKY_POLL_ID_13                      0xAC // [13] -
#define FRSKY_POLL_ID_14                      0x0D // [14] -
#define FRSKY_POLL_ID_15                      0x8E // [15] -
#define FRSKY_POLL_ID_16                      0x2F // [16] -
#define FRSKY_POLL_ID_17                      0xD0 // [17] -
#define FRSKY_POLL_ID_18                      0x71 // [18] -
#define FRSKY_POLL_ID_19                      0xF2 // [19] -
#define FRSKY_POLL_ID_20                      0x53 // [20] -
#define FRSKY_POLL_ID_21                      0x34 // [21] -
#define FRSKY_POLL_ID_22                      0x95 // [22] -
#define FRSKY_POLL_ID_23                      0x16 // [23] -
#define FRSKY_POLL_ID_IMU                     0xB7 // [24] IMU ACC (x,y,z)
#define FRSKY_POLL_ID_25                      0x98 // [25] RX / TX internal telemetry
#define FRSKY_POLL_ID_26                      0x39 // [26] PowerBox (aka Redudancy Bus)
#define FRSKY_POLL_ID_TEMP                    0xBA // [27] Temp
#define FRSKY_POLL_ID_FUEL                    0x1B // [28] Fuel (ArduPilot/Betaflight)

/**
 * OpenTX sensor IDs
 */

#define FRSKY_SENSOR_ID_ALT_FIRST             0x0100 // Vari-H altimeter high precision (int) float * 100 [m]
#define FRSKY_SENSOR_ID_ALT_LAST              0x010F // Vari-N altimeter normal precision (int) float * 100 [m]
#define FRSKY_SENSOR_ID_VARIO_FIRST           0x0110 // Vari-H altimeter high precision (int) float * 100 [meters per second]
#define FRSKY_SENSOR_ID_VARIO_LAST            0X011F // Vari-N altimeter normal precision (int) float * 100 [meters per second]
#define FRSKY_SENSOR_ID_CURR_FIRST            0x0200 // FAS current sensor (int) float * 10 [A]
#define FRSKY_SENSOR_ID_CURR_LAST             0x020F // FAS current sensor (int) float * 10 [A]
#define FRSKY_SENSOR_ID_VFAS_FIRST            0x0210 // FAS voltage sensor (int) float * 100 [V]
#define FRSKY_SENSOR_ID_VFAS_LAST             0x021F // FAS voltage sensor (int) float * 100 [V]
#define FRSKY_SENSOR_ID_CELLS_FIRST           0x0300 // FLVSS/MLVSS
#define FRSKY_SENSOR_ID_CELLS_LAST            0x030F // FLVSS/MLVSS
#define FRSKY_SENSOR_ID_T1_FIRST              0x0400 // Temperature int [°C]
#define FRSKY_SENSOR_ID_T1_LAST               0x040F // Temperature int [°C]
#define FRSKY_SENSOR_ID_T2_FIRST              0x0410 // Temperature int [°C]
#define FRSKY_SENSOR_ID_T2_LAST               0x041F // Temperature int [°C]
#define FRSKY_SENSOR_ID_RPM_FIRST             0x0500 // RPM int [rpm]
#define FRSKY_SENSOR_ID_RPM_LAST              0x050F // RPM int [rpm]
#define FRSKY_SENSOR_ID_FUEL_FIRST            0x0600 // Fuel int 0~100 [%]
#define FRSKY_SENSOR_ID_FUEL_LAST             0x060F // Fuel int 0~100 [%]
#define FRSKY_SENSOR_ID_ACCX_FIRST            0x0700 // Accelerometer (X) (int) float * 100 [g]
#define FRSKY_SENSOR_ID_ACCX_LAST             0x070F // Accelerometer (X) (int) float * 100 [g]
#define FRSKY_SENSOR_ID_ACCY_FIRST            0x0710 // Accelerometer (Y) (int) float * 100 [g]
#define FRSKY_SENSOR_ID_ACCY_LAST             0x071F // Accelerometer (Y) (int) float * 100 [g]
#define FRSKY_SENSOR_ID_ACCZ_FIRST            0x0720 // Accelerometer (Z) (int) float * 100 [g]
#define FRSKY_SENSOR_ID_ACCZ_LAST             0x072F // Accelerometer (Z) (int) float * 100 [g]
#define FRSKY_SENSOR_ID_GPS_LONG_LATI_FIRST   0x0800 // GPS
#define FRSKY_SENSOR_ID_GPS_LONG_LATI_LAST    0x080F // GPS
#define FRSKY_SENSOR_ID_GPS_ALT_FIRST         0x0820 // GPS Altitude (int) float * 100 [m]
#define FRSKY_SENSOR_ID_GPS_ALT_LAST          0x082F // GPS Altitude (int) float * 100 [m]
#define FRSKY_SENSOR_ID_GPS_SPEED_FIRST       0x0830 // GPS Speed (int) float * 1000 [kts]
#define FRSKY_SENSOR_ID_GPS_SPEED_LAST        0x083F // GPS Speed (int) float * 1000 [kts]
#define FRSKY_SENSOR_ID_GPS_COURSE_FIRST      0x0840 // GPS Heading (int) float * 100 [°]
#define FRSKY_SENSOR_ID_GPS_COURSE_LAST       0x084F // GPS Heading (int) float * 100 [°]
#define FRSKY_SENSOR_ID_GPS_TIME_DATE_FIRST   0x0850 // GPS Time & date
#define FRSKY_SENSOR_ID_GPS_TIME_DATE_LAST    0x085F // GPS Time & date
#define FRSKY_SENSOR_ID_A3_FIRST              0x0900 // Analog voltage (int) float * 100 [V]
#define FRSKY_SENSOR_ID_A3_LAST               0x090F // Analog voltage (int) float * 100 [V]
#define FRSKY_SENSOR_ID_A4_FIRST              0x0910 // Analog voltage (int) float * 100 [V]
#define FRSKY_SENSOR_ID_A4_LAST               0x091F // Analog voltage (int) float * 100 [V]
#define FRSKY_SENSOR_ID_AIR_SPEED_FIRST       0x0A00 // ASS (int) float * 10 [kts]
#define FRSKY_SENSOR_ID_AIR_SPEED_LAST        0x0A0F // ASS (int) float * 10 [kts]
#define FRKSY_SENSOR_ID_POWERBOX_BATT1_FIRST	0x0B00 // Power box voltage (int) float * 1000 [V]
#define FRKSY_SENSOR_ID_POWERBOX_BATT1_LAST	  0x0B0F // Power box voltage (int) float * 1000 [V]
#define FRSKY_SENSOR_ID_POWERBOX_BATT2_FIRST	0x0B10 // Power box voltage (int) float * 1000 [V]
#define FRSKY_SENSOR_ID_POWERBOX_BATT2_LAST	  0x0B1F // Power box voltage (int) float * 1000 [V]
#define FRSKY_SENSOR_ID_POWERBOX_STATE_FIRST	0x0B20 // Power box state (raw)
#define FRSKY_SENSOR_ID_POWERBOX_STATE_LAST	  0x0B2F // Power box state (raw)
#define FRSKY_SENSOR_ID_POWERBOX_CNSP_FIRST	  0x0B30 // Power box consumption (int) int [mAh]
#define FRSKY_SENSOR_ID_POWERBOX_CNSP_LAST	  0x0B3F // Power box consumption (int) int [mAh]

#define FRSKY_SENSOR_ID_RSSI	                0xF101 // RSSI dB - N/A - already handled by receiver

#define FRSKY_SENSOR_ID_ADC1	                0xF102 // RX voltage - N/A - already handled by receiver
#define FRSKY_SENSOR_ID_A1		                FRSKY_SENSOR_ID_ADC1
#define FRSKY_SENSOR_ID_BTRX	                FRSKY_SENSOR_ID_ADC1

#define FRSKY_SENSOR_ID_ADC2	                0xF103 // Analog voltage (volts)
#define FRSKY_SENSOR_ID_A2		                FRSKY_SENSOR_ID_ADC2

#define FRSKY_SENSOR_ID_SP2UART_A		          0xFD00
#define FRSKY_SENSOR_ID_SP2UART_B		          0xFD01

#define FRSKY_SENSOR_ID_BATT	                0xF104

#define FRSKY_SENSOR_ID_SWR	                  0xF105

#define FRSKY_SENSOR_ID_XJT_VERSION	          0xF106

#define FRSKY_SENSOR_ID_FUEL_QTY_FIRST	    	0x0A10 // Fuel consumption (int) float * 100 [ml]
#define FRSKY_SENSOR_ID_FUEL_QTY_LAST	      	0x0A1F // Fuel consumption (int) float * 100 [ml]


/**
 * OpenTX Passthrough IDs
 */

#define FRSKY_PT_SENSOR_ID_DIY                 0X5000
#define FRSKY_PT_SENSOR_ID_STATUS_TEXT         0X5000
#define FRSKY_PT_SENSOR_ID_AP_STATUS           0X5001
#define FRSKY_PT_SENSOR_ID_GPS_STATUS          0X5002
#define FRSKY_PT_SENSOR_SUBID_GPS_LAT          0
#define FRSKY_PT_SENSOR_SUBID_GPS_LON          1
#define FRSKY_PT_SENSOR_ID_BATTERY_1           0X5003
#define FRSKY_PT_SENSOR_ID_HOME                0X5004
#define FRSKY_PT_SENSOR_ID_VEL_YAW             0X5005
#define FRSKY_PT_SENSOR_ID_ATT_RNG             0X5006
#define FRSKY_PT_SENSOR_ID_PARAMETERS          0X5007
#define FRSKY_PT_SENSOR_ID_BATTERY_2           0X5008
#define FRSKY_PT_SENSOR_ID_WAYPOINTS_MISSIONS  0x5009
#define FRSKY_PT_SENSOR_ID_SERVO_RAW           0x50F1
#define FRSKY_PT_SENSOR_ID_VFR_HUD             0x50F2
#define FRSKY_PT_SENSOR_ID_WIND                0x50F3
