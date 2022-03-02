#include "imu_lsm6ds3.h"

DRAM_ATTR unsigned long imu_timestamp           = 0;
DRAM_ATTR int16_t front_imu_raw_data_array[6]   = { 0 }; /* 012: xyz rotation, 345: xyz acceleration */
DRAM_ATTR int16_t back_imu_raw_data_array[6]    = { 0 }; /* 012: xyz rotation, 345: xyz acceleration */

#if CALIBRATE_ACCELERATION
    DRAM_ATTR double_t front_imu_calibrated_acceleration_array[3] = { 0 }; /* 012: xyz acceleration */
    DRAM_ATTR double_t back_imu_calibrated_acceleration_array[3] = { 0 }; /* 012: xyz acceleration */
#endif

void init_imu()
{
    Wire.begin();
    Wire.setClock(FAST_MODE_PLUS);

    /* Set up acceleration mode */
    uint8_t value_to_write = SCALE_8G | ODR_ACCEL_104Hz; /* for choosing values, see definition of the registers in the header file. If calibration is used, use the right sets of calibration parameters in the header file. */
    write_to_i2c_register(ADDRESS_IMU_FRONT,LINEAR_ACCELERATION_CONTROL_REGISTER,value_to_write);
    write_to_i2c_register(ADDRESS_IMU_BACK,LINEAR_ACCELERATION_CONTROL_REGISTER,value_to_write);

    /* Set up rotation mode */
    value_to_write = SCALE_2000DPS | ODR_ACCEL_104Hz;
    write_to_i2c_register(ADDRESS_IMU_FRONT,ANGULAR_RATE_CONTROL_REGISTER,value_to_write);
    write_to_i2c_register(ADDRESS_IMU_BACK,ANGULAR_RATE_CONTROL_REGISTER,value_to_write);
}

inline void write_to_i2c_register(uint8_t slave_address, uint8_t register_address, uint8_t value_to_write)
{
    Wire.beginTransmission(slave_address);
    Wire.write(register_address);
    Wire.write(value_to_write);
    Wire.endTransmission();
    #if DEBUG
        Serial.printf("value_to_write: %d\n", value_to_write);
    #endif
}

IRAM_ATTR void imu_read()
{
    imu_timestamp = micros();

    /* front IMU */
    /* Start at the lowest register address, sensor will increment address when reading. Rotation address registers continue right after the acceleration registers, so reading can continue. If some registers were skipped, that has to be accounted for. That's also likely slower since it requires a serial write of the new read address beforehand. */
    Wire.beginTransmission(ADDRESS_IMU_FRONT);
    Wire.write(ROTATION_X_LOW); 
    Wire.endTransmission(); /* transmits the bytes written by Wire.write */
    Wire.requestFrom(ADDRESS_IMU_FRONT,12,1);
    /* rotation values */
    front_imu_raw_data_array[0]  = Wire.read();
    front_imu_raw_data_array[0] += Wire.read() << 8;
    front_imu_raw_data_array[1]  = Wire.read();
    front_imu_raw_data_array[1] += Wire.read() << 8;
    front_imu_raw_data_array[2]  = Wire.read();
    front_imu_raw_data_array[2] += Wire.read() << 8;
    /* acceleration values */
    front_imu_raw_data_array[3]  = Wire.read();
    front_imu_raw_data_array[3] += Wire.read() << 8;
    front_imu_raw_data_array[4]  = Wire.read();
    front_imu_raw_data_array[4] += Wire.read() << 8;
    front_imu_raw_data_array[5]  = Wire.read();
    front_imu_raw_data_array[5] += Wire.read() << 8;  

    /* back IMU */
    /* rotation values */
    Wire.beginTransmission(ADDRESS_IMU_BACK);
    Wire.write(ROTATION_X_LOW);
    Wire.endTransmission();
    Wire.requestFrom(ADDRESS_IMU_BACK,12,1);
    back_imu_raw_data_array[0]  = Wire.read();
    back_imu_raw_data_array[0] += Wire.read() << 8;
    back_imu_raw_data_array[1]  = Wire.read();
    back_imu_raw_data_array[1] += Wire.read() << 8;
    back_imu_raw_data_array[2]  = Wire.read();
    back_imu_raw_data_array[2] += Wire.read() << 8;    
    /* acceleration values */
    back_imu_raw_data_array[3]  = Wire.read();
    back_imu_raw_data_array[3] += Wire.read() << 8;
    back_imu_raw_data_array[4]  = Wire.read();
    back_imu_raw_data_array[4] += Wire.read() << 8;
    back_imu_raw_data_array[5]  = Wire.read();
    back_imu_raw_data_array[5] += Wire.read() << 8;

    #if CALIBRATE_ACCELERATION
        /* done without vector or matrix libraries to keep it lightweight */
        /* front sensor */
        /* x axis vehicle */
        front_imu_calibrated_acceleration_array[0] = calibration_values_front[0] * front_imu_raw_data_array[3] + calibration_values_front[1] * front_imu_raw_data_array[4] + calibration_values_front[2] * front_imu_raw_data_array[5] + calibration_values_front[3];
        /* y axis vehicle */
        front_imu_calibrated_acceleration_array[1] = calibration_values_front[4] * front_imu_raw_data_array[3] + calibration_values_front[5] * front_imu_raw_data_array[4] + calibration_values_front[6] * front_imu_raw_data_array[5] + calibration_values_front[7];
        /* z axis vehicle */
        front_imu_calibrated_acceleration_array[2] = calibration_values_front[8] * front_imu_raw_data_array[3] + calibration_values_front[9] * front_imu_raw_data_array[4] + calibration_values_front[10] * front_imu_raw_data_array[5] + calibration_values_front[11];

        /* back sensor */
        /* x axis vehicle */
        back_imu_calibrated_acceleration_array[0] = calibration_values_back[0] * back_imu_raw_data_array[3] + calibration_values_back[1] * back_imu_raw_data_array[4] + calibration_values_back[2] * back_imu_raw_data_array[5] + calibration_values_back[3];
        /* y axis vehicle */
        back_imu_calibrated_acceleration_array[1] = calibration_values_back[4] * back_imu_raw_data_array[3] + calibration_values_back[5] * back_imu_raw_data_array[4] + calibration_values_back[6] * back_imu_raw_data_array[5] + calibration_values_back[7];
        /* z axis vehicle */
        back_imu_calibrated_acceleration_array[2] = calibration_values_back[8] * back_imu_raw_data_array[3] + calibration_values_back[9] * back_imu_raw_data_array[4] + calibration_values_back[10] * back_imu_raw_data_array[5] + calibration_values_back[11];
    #endif
}
