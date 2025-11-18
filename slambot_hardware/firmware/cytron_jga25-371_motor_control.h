#ifndef CYTRON_JGA25_371_MOTOR_CONTROL_H
#define CYTRON_JGA25_371_MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MOTOR_TICKS_PER_REV 2490.0f
#define MOTOR_TICKS_PER_RAD (MOTOR_TICKS_PER_REV / (2.0f * (float)M_PI))
#define MAX_COMMANDED_RADS_PER_SEC 5.89f

#define MOTOR_COUNT 4

typedef enum {
    MOTOR_FRONT_LEFT = 0,
    MOTOR_FRONT_RIGHT = 1,
    MOTOR_REAR_LEFT = 2,
    MOTOR_REAR_RIGHT = 3
} motor_index_t;

typedef struct {
    uint8_t pwm_pin;
    uint8_t dir_pin;
} motor_channels_t;

extern const motor_channels_t motor_channel_map[MOTOR_COUNT];

#define ENCODER_A_FL_PIN 17
#define ENCODER_B_FL_PIN 16
#define ENCODER_A_FR_PIN 18
#define ENCODER_B_FR_PIN 19
#define ENCODER_A_RL_PIN 20
#define ENCODER_B_RL_PIN 21
#define ENCODER_A_RR_PIN 15
#define ENCODER_B_RR_PIN 14

typedef struct {
    motor_channels_t channels;
    volatile int32_t* ticks;
    float target_tps;
    float active_target_tps;
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float i_max;
    float max_output_step;
    float max_target_slew;
    float filtered_velocity;
    float last_measured_tps;
    uint32_t last_update_ms;
    int32_t last_position;
    int32_t pwm_output;
    bool command_active;
    int8_t kick_direction;
    uint32_t kick_ms_remaining;
    int32_t kick_pwm;
} motor_t;

extern volatile int32_t g_encoder_counts[MOTOR_COUNT];

void motor_driver_init_pins(void);
void encoder_init(void);

void motor_init(motor_t* motor,
                motor_index_t index,
                float kp, float ki, float kd,
                float i_max,
                float max_output_step,
                float max_target_slew,
                int32_t kick_pwm);

void motor_set_target_tps(motor_t* motor, float target_tps);
void motor_stop(motor_t* motor);
void motor_update(motor_t* motor, uint32_t now_ms);
int32_t motor_get_ticks(motor_index_t index);
float ticks_per_second_from_radians(float rad_per_sec);

#endif
