/* location of I2C addresses. For more info, see datasheet, chapter 8: Register mapping */
#define ADDRESS_IMU_FRONT   0x6A
#define ADDRESS_IMU_BACK    0x6B

#define LINEAR_ACCELERATION_CONTROL_REGISTER 0x10
/*
ODR_XL3 ODR_XL2 ODR_XL1 ODR_XL0 FS_XL1 FS_XL0 BW_XL1 BW_XL0

ODR_XL [3:0] Output data rate and power mode selection. Default value: 0000. For all values, see Table 47 in datasheet.
    0 0 0 0 Power-down
    0 0 1 1 52 Hz
    0 1 0 0 104 Hz
    0 1 0 1 208 Hz
    0 1 1 1 833 Hz
    1 0 0 0 1.66 kHz
    1 0 0 1 3.33 kHz
    1 0 1 0 6.66 kHz
FS_XL [1:0] Accelerometer full-scale selection. Default value: 00
    (00: ±2 g; 01: ±16 g; 10: ±4 g; 11: ±8 g)
BW_XL [1:0] Anti-aliasing filter bandwidth selection. Default value: 00. Only used when XL_BW_SCAL_ODR = 1, which is MSB of 0x13
    (00: 400 Hz; 01: 200 Hz; 10: 100 Hz; 11: 50 Hz)
*/
    #define ODR_ACCEL_0Hz     (0x00 << 4)   /* ODR: output data rate */
    #define ODR_ACCEL_104Hz   (0x04 << 4)
    #define ODR_ACCEL_833Hz   (0x07 << 4)
    #define ODR_ACCEL_1660Hz  (0x08 << 4)
    #define ODR_ACCEL_6660Hz  (0x0A << 4)
    #define SCALE_2G          (0x00 << 2)
    #define SCALE_4G          (0x02 << 2)
    #define SCALE_8G          (0x03 << 2)
    #define SCALE_16G         (0x01 << 2)

#define ANGULAR_RATE_CONTROL_REGISTER 0x11
/*
ODR_G3 ODR_G2 ODR_G1 ODR_G0 FS_G1 FS_G0 FS_125 0

ODR_G [3:0] Gyroscope output data rate selection. Default value: 0000
    0 0 0 0 Power-down
    0 0 1 1 52 Hz
    0 1 0 0 104 Hz
    0 1 0 1 208 Hz
    0 1 1 1 833 Hz
    1 0 0 0 1.66 kHz
FS_G [1:0] Gyroscope full-scale selection. Default value: 00
    (00: 250 dps; 01: 500 dps; 10: 1000 dps; 11: 2000 dps)
FS_125 Gyroscope full-scale at 125 dps. Default value: 0
    (0: disabled; 1: enabled)
*/
    #define ODR_GYRO_0Hz     (0x00 << 4)
    #define ODR_GYRO_104Hz   (0x04 << 4)
    #define ODR_GYRO_833Hz   (0x07 << 4)
    #define ODR_GYRO_1660Hz  (0x08 << 4)
    #define SCALE_125DPS     (0x01 << 1)
    #define SCALE_250DPS     (0x00 << 2)
    #define SCALE_500DPS     (0x01 << 2)
    #define SCALE_1000DPS    (0x02 << 2)
    #define SCALE_2000DPS    (0x03 << 2)

/* data registers */
#define ROTATION_X_LOW      0x22
#define ROTATION_X_HIGH     0x23
#define ROTATION_Y_LOW      0x24
#define ROTATION_Y_HIGH     0x25
#define ROTATION_Z_LOW      0x26
#define ROTATION_Z_HIGH     0x27

#define ACCELERATION_X_LOW  0x28
#define ACCELERATION_X_HIGH 0x29
#define ACCELERATION_Y_LOW  0x2A
#define ACCELERATION_Y_HIGH 0x2B
#define ACCELERATION_Z_LOW  0x2C
#define ACCELERATION_Z_HIGH 0x2D

/* transmission frequencies in Hz */
#define STANDARD_MODE       100000 
#define FAST_MODE           400000
#define FAST_MODE_PLUS      1000000 /* fastest supported mode */
#define HIGH_SPEED_MODE     3400000

#define GRAVITY_FACTOR      9.80665

#include <Wire.h>       /* arduino i2c library */
#include <Arduino.h>
#include "globals.h"

extern DRAM_ATTR unsigned long imu_timestamp;
extern DRAM_ATTR int16_t front_imu_raw_data_array[6];
extern DRAM_ATTR int16_t back_imu_raw_data_array[6];

#if CALIBRATE_ACCELERATION
    extern DRAM_ATTR double_t front_imu_calibrated_acceleration_array[3];
    extern DRAM_ATTR double_t back_imu_calibrated_acceleration_array[3];
    /* calibration values
    row vector, cal values: xx xy xz xb, yx yy yz yb, zx zy zz zb
    x_car = xx*measured_x + xy*measured_y + xz*measured_z + xb and so on
    this formats them as well so that 1.0 is equal to 1g.
    */

    /* calibration values for +-8g */   
    const double_t calibration_values_front [] = { 5.49391498367321e-07, 0.000242665106478849, 1.48803148659940e-05, 0.00782013884499783, 0.000244729606560598, -1.30868908647501e-07, -4.73889257422259e-06, 0.00967661395696673, -4.27858538728495e-06, 1.52530721834046e-05, -0.000241466842188727, 0.00728956178570483 };
    const double_t calibration_values_back [] = { -3.19415627158884e-06, 0.000242641634827048, 6.15240127187166e-06, -0.00811203599963953, 0.000241871778667421, 2.86384418200320e-06, 7.98659477621670e-07, 0.00469760197311742, 1.41746531657993e-06, 5.91491608451484e-06, -0.000241812270066516, 0.0210918849264041 };

    /* calibration values for +-16g */
    /*
    const double_t calibration_values_front [] = { 1.02116949902745e-06, 0.000485238530816457, 2.56733216508941e-05, 0.000147457224302067, 0.000489123667111545, -8.09922091756310e-07, -1.02146124957591e-05, 0.000774821647134688, -9.76766565330662e-06, 2.64022873581938e-05, -0.000482909689989141, 0.00453797699394315 };
    const double_t calibration_values_back [] = { -5.68000538606822e-06, 0.000484780366495398, 8.90901572173184e-06, -0.0107063435566905, 0.000482752933776492, 5.33039522900044e-06, 5.24187631733225e-07, 0.00416002876460825, 4.69406030618897e-07, 8.30570600832508e-06, -0.000482848442737067, 0.0183260247200123 };
    */
#endif

void init_imu();
inline void write_to_i2c_register(uint8_t slave_address, uint8_t register_address, uint8_t value_to_write);
IRAM_ATTR void imu_read();