#ifndef IMU_BNO055_DRIVER_H
#define IMU_BNO055_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"
#include "bno055.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    struct bno055_t sensor;
    bool initialised;
} bno055_imu_t;

typedef struct {
    double orientation_w;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
    double linear_acceleration_x;
    double linear_acceleration_y;
    double linear_acceleration_z;
} bno055_imu_sample_t;

typedef struct {
    bool has_accel;
    bool has_gyro;
    bool has_mag;
    bool has_sic;
    struct bno055_accel_offset_t accel;
    struct bno055_gyro_offset_t gyro;
    struct bno055_mag_offset_t mag;
    struct bno055_sic_matrix_t sic;
} bno055_imu_offsets_t;

bool bno055_imu_init(bno055_imu_t *imu,
                     i2c_inst_t *i2c_instance,
                     uint8_t address);

bool bno055_imu_read(const bno055_imu_t *imu,
                     bno055_imu_sample_t *sample);

bool bno055_imu_get_calibration(const bno055_imu_t *imu,
                                uint8_t *system_calib,
                                uint8_t *gyro_calib,
                                uint8_t *accel_calib,
                                uint8_t *mag_calib);

bool bno055_imu_read_offsets(const bno055_imu_t *imu,
                             bno055_imu_offsets_t *offsets);

bool bno055_imu_write_offsets(bno055_imu_t *imu,
                              const bno055_imu_offsets_t *offsets);

#ifdef __cplusplus
}
#endif

#endif
