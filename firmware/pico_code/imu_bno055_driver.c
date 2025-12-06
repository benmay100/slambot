#include "imu_bno055_driver.h"

#include <math.h>
#include <string.h>

#include "pico/stdlib.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define I2C_INIT_DELAY_MS 10
#define MODE_SWITCH_DELAY_MS 20
#define NDOF_STARTUP_DELAY_MS 100

bool bno055_imu_init(bno055_imu_t *imu,
                     i2c_inst_t *i2c_instance,
                     uint8_t address) {
    if (imu == NULL || i2c_instance == NULL) {
        return false;
    }

    memset(imu, 0, sizeof(*imu));

    if (bno055_pico_init(&imu->sensor, i2c_instance, address) != BNO055_SUCCESS) {
        return false;
    }

    if (bno055_set_power_mode(BNO055_POWER_MODE_NORMAL) != BNO055_SUCCESS) {
        return false;
    }
    sleep_ms(I2C_INIT_DELAY_MS);

    if (bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG) != BNO055_SUCCESS) {
        return false;
    }
    sleep_ms(MODE_SWITCH_DELAY_MS);

    if (bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ) != BNO055_SUCCESS) {
        return false;
    }
    if (bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS) != BNO055_SUCCESS) {
        return false;
    }
    if (bno055_set_euler_unit(BNO055_EULER_UNIT_RAD) != BNO055_SUCCESS) {
        return false;
    }

    if (bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF) != BNO055_SUCCESS) {
        return false;
    }
    sleep_ms(NDOF_STARTUP_DELAY_MS);

    imu->initialised = true;
    return true;
}

bool bno055_imu_read(const bno055_imu_t *imu,
                     bno055_imu_sample_t *sample) {
    if (imu == NULL || sample == NULL || !imu->initialised) {
        return false;
    }

    struct bno055_quaternion_t quaternion = {0};
    struct bno055_gyro_t gyro = {0};
    struct bno055_accel_t accel = {0};

    if (bno055_read_quaternion_wxyz(&quaternion) != BNO055_SUCCESS) {
        return false;
    }
    if (bno055_read_gyro_xyz(&gyro) != BNO055_SUCCESS) {
        return false;
    }
    if (bno055_read_accel_xyz(&accel) != BNO055_SUCCESS) {
        return false;
    }

    sample->orientation_w = (double)quaternion.w / 16384.0;
    sample->orientation_x = (double)quaternion.x / 16384.0;
    sample->orientation_y = (double)quaternion.y / 16384.0;
    sample->orientation_z = (double)quaternion.z / 16384.0;

    sample->angular_velocity_x = (double)gyro.x / BNO055_GYRO_DIV_RPS;
    sample->angular_velocity_y = (double)gyro.y / BNO055_GYRO_DIV_RPS;
    sample->angular_velocity_z = (double)gyro.z / BNO055_GYRO_DIV_RPS;

    /* Acceleration vector is reported in m/s^2 and still includes gravity, as expected by ROS REP 103. */
    sample->linear_acceleration_x = (double)accel.x / BNO055_ACCEL_DIV_MSQ;
    sample->linear_acceleration_y = (double)accel.y / BNO055_ACCEL_DIV_MSQ;
    sample->linear_acceleration_z = (double)accel.z / BNO055_ACCEL_DIV_MSQ;

    return true;
}

bool bno055_imu_get_calibration(const bno055_imu_t *imu,
                                uint8_t *system_calib,
                                uint8_t *gyro_calib,
                                uint8_t *accel_calib,
                                uint8_t *mag_calib) {
    if (imu == NULL || !imu->initialised) {
        return false;
    }

    uint8_t sys = 0;
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;

    if (bno055_get_sys_calib_stat(&sys) != BNO055_SUCCESS) {
        return false;
    }
    if (bno055_get_gyro_calib_stat(&gyro) != BNO055_SUCCESS) {
        return false;
    }
    if (bno055_get_accel_calib_stat(&accel) != BNO055_SUCCESS) {
        return false;
    }
    if (bno055_get_mag_calib_stat(&mag) != BNO055_SUCCESS) {
        return false;
    }

    if (system_calib != NULL) {
        *system_calib = sys;
    }
    if (gyro_calib != NULL) {
        *gyro_calib = gyro;
    }
    if (accel_calib != NULL) {
        *accel_calib = accel;
    }
    if (mag_calib != NULL) {
        *mag_calib = mag;
    }

    return true;
}

bool bno055_imu_read_offsets(const bno055_imu_t *imu,
                             bno055_imu_offsets_t *offsets) {
    if (imu == NULL || offsets == NULL || !imu->initialised) {
        return false;
    }

    memset(offsets, 0, sizeof(*offsets));

    uint8_t original_mode = 0;
    if (bno055_get_operation_mode(&original_mode) != BNO055_SUCCESS) {
        return false;
    }

    bool restore_mode = (original_mode != BNO055_OPERATION_MODE_CONFIG);
    if (restore_mode) {
        if (bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG) != BNO055_SUCCESS) {
            return false;
        }
        sleep_ms(MODE_SWITCH_DELAY_MS);
    }

    bool success = true;

    struct bno055_accel_offset_t accel = {0};
    if (bno055_read_accel_offset(&accel) == BNO055_SUCCESS) {
        offsets->accel = accel;
        offsets->has_accel = true;
    } else {
        success = false;
    }

    struct bno055_gyro_offset_t gyro = {0};
    if (bno055_read_gyro_offset(&gyro) == BNO055_SUCCESS) {
        offsets->gyro = gyro;
        offsets->has_gyro = true;
    } else {
        success = false;
    }

    struct bno055_mag_offset_t mag = {0};
    if (bno055_read_mag_offset(&mag) == BNO055_SUCCESS) {
        offsets->mag = mag;
        offsets->has_mag = true;
    } else {
        success = false;
    }

    struct bno055_sic_matrix_t sic = {0};
    if (bno055_read_sic_matrix(&sic) == BNO055_SUCCESS) {
        offsets->sic = sic;
        offsets->has_sic = true;
    }

    if (restore_mode) {
        if (bno055_set_operation_mode(original_mode) != BNO055_SUCCESS) {
            success = false;
        } else {
            if (original_mode == BNO055_OPERATION_MODE_NDOF) {
                sleep_ms(NDOF_STARTUP_DELAY_MS);
            } else {
                sleep_ms(MODE_SWITCH_DELAY_MS);
            }
        }
    }

    return success;
}

bool bno055_imu_write_offsets(bno055_imu_t *imu,
                              const bno055_imu_offsets_t *offsets) {
    if (imu == NULL || offsets == NULL || !imu->initialised) {
        return false;
    }

    if (!offsets->has_accel && !offsets->has_gyro && !offsets->has_mag && !offsets->has_sic) {
        return true;
    }

    uint8_t original_mode = 0;
    if (bno055_get_operation_mode(&original_mode) != BNO055_SUCCESS) {
        return false;
    }

    bool restore_mode = (original_mode != BNO055_OPERATION_MODE_CONFIG);
    if (restore_mode) {
        if (bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG) != BNO055_SUCCESS) {
            return false;
        }
        sleep_ms(MODE_SWITCH_DELAY_MS);
    }

    bool success = true;

    if (offsets->has_accel) {
        struct bno055_accel_offset_t accel = offsets->accel;
        if (bno055_write_accel_offset(&accel) != BNO055_SUCCESS) {
            success = false;
        }
    }

    if (offsets->has_gyro) {
        struct bno055_gyro_offset_t gyro = offsets->gyro;
        if (bno055_write_gyro_offset(&gyro) != BNO055_SUCCESS) {
            success = false;
        }
    }

    if (offsets->has_mag) {
        struct bno055_mag_offset_t mag = offsets->mag;
        if (bno055_write_mag_offset(&mag) != BNO055_SUCCESS) {
            success = false;
        }
    }

    if (offsets->has_sic) {
        struct bno055_sic_matrix_t sic = offsets->sic;
        if (bno055_write_sic_matrix(&sic) != BNO055_SUCCESS) {
            success = false;
        }
    }

    if (restore_mode) {
        if (bno055_set_operation_mode(original_mode) != BNO055_SUCCESS) {
            success = false;
        } else {
            if (original_mode == BNO055_OPERATION_MODE_NDOF) {
                sleep_ms(NDOF_STARTUP_DELAY_MS);
            } else {
                sleep_ms(MODE_SWITCH_DELAY_MS);
            }
        }
    }

    return success;
}
