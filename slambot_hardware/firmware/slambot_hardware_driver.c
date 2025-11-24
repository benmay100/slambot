/*
 * SLAMBOT_HARDWARE_DRIVER.C
 
 For use with four JGA25-371 (100rpm) motors, Two Cytron MDD10A Dual-channel drivers, and the Raspberry Pi Pico using micro-ROS.

 *
 * This module wires PWM outputs to a Cytron MDD10A dual-channel driver, reads
 * quadrature encoders, and exposes a simple ROS 2 interface:
 *   - Subscribes to /wheel_commands (Float32MultiArray of wheel rad/s targets)
 *   - Publishes /pico_motor_driver with per-wheel telemetry
 *   - Publishes /tick_counter for raw encoder counts
 *
 * The control loop wraps PID regulation, stiction compensation, slew limiting,
 * and encoder debouncing. Encoders are decoded via a 4-state quadrature table
 * with a minimum edge interval guard to reject contact bounce.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <sensor_msgs/msg/imu.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"

#include "cytron_jga25-371_motor_control.h"
#include "imu_bno055_driver.h"

#define LED_PIN 25

/* Control loop cadence and guardrails */
#define CONTROL_INTERVAL_MS 20U
#define MOTOR_STATE_PUBLISH_MS 100U
#define WHEEL_STATE_PUBLISH_MS 100U
#define IMU_PUBLISH_MS 50U
#define TICK_PUBLISH_MS 250U
#define AGENT_CONNECT_TIMEOUT_MS 300000U /* 5 minutes to allow for time-syncing before latching */
#define COMMAND_TIMEOUT_MS 1000U

/* Velocity estimation and PID behaviour tuning */
#define STOP_TPS_THRESHOLD 5.0f
#define VELOCITY_FILTER_ALPHA 0.25f
#define PWM_LIMIT 60000.0f
#define MIN_EFFECTIVE_PWM 1500.0f
#define MAX_OUTPUT_STEP_PER_SEC 30000.0f
#define MAX_TARGET_SLEW_TPS_PER_SEC 2500.0f
#define KICK_PWM_LEVEL 9000
#define KICK_DURATION_MS 150U

/* Debounce threshold between valid encoder edges (us) */
#define ENCODER_MIN_INTERVAL_US 80U

/* IMU wiring */
#define IMU_I2C_PORT i2c1
#define IMU_I2C_SDA_PIN 26 // Pico GP26 / ADC0
#define IMU_I2C_SCL_PIN 27 // Pico GP27 / ADC1
#define BNO055_I2C_ADDRESS 0x28

/* Default PID gains applied to every wheel */
static const float PID_KP = 1.2f;
static const float PID_KI = 0.02f;
static const float PID_KD = 0.06f;

const motor_channels_t motor_channel_map[MOTOR_COUNT] = {
    [MOTOR_FRONT_LEFT]  = {.pwm_pin = 12, .dir_pin = 13},
    [MOTOR_FRONT_RIGHT] = {.pwm_pin = 10, .dir_pin = 11},
    [MOTOR_REAR_LEFT]   = {.pwm_pin = 8,  .dir_pin = 9 },
    [MOTOR_REAR_RIGHT]  = {.pwm_pin = 6,  .dir_pin = 7 }
};

volatile int32_t g_encoder_counts[MOTOR_COUNT] = {0};

static motor_t motors[MOTOR_COUNT];
static volatile uint32_t last_command_ms = 0;

static std_msgs__msg__String motor_state_msg;
static std_msgs__msg__String tick_msg;
static std_msgs__msg__Float32MultiArray wheel_state_msg;
static std_msgs__msg__Float32MultiArray wheel_command_msg;
static sensor_msgs__msg__Imu imu_msg;
static std_msgs__msg__Float32MultiArray imu_calib_msg;
static std_msgs__msg__String imu_offsets_msg;
static std_msgs__msg__Int16MultiArray imu_offsets_array_msg;

static rcl_publisher_t motor_state_pub;
static rcl_publisher_t wheel_state_pub;
static rcl_publisher_t tick_pub;
static rcl_publisher_t imu_pub;
static rcl_publisher_t imu_calib_pub;
static rcl_publisher_t imu_offsets_pub;
static rcl_publisher_t imu_offsets_array_pub;
static rcl_subscription_t wheel_cmd_sub;

static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;

static char motor_state_buffer[256];
static char tick_buffer[128];
static float wheel_state_buffer[MOTOR_COUNT * 2];
static float wheel_command_buffer[MOTOR_COUNT];
static float imu_calib_buffer[4];
static char imu_offsets_buffer[1024];
static int16_t imu_offsets_array_buffer[24];
static char imu_frame_id[16] = "imu_link";

/* GPIO assignment mirrors the wiring harness so the ISR can map back to motors */
static const uint8_t encoder_a_pins[MOTOR_COUNT] = {
    ENCODER_A_FL_PIN,
    ENCODER_A_FR_PIN,
    ENCODER_A_RL_PIN,
    ENCODER_A_RR_PIN
};

static const uint8_t encoder_b_pins[MOTOR_COUNT] = {
    ENCODER_B_FL_PIN,
    ENCODER_B_FR_PIN,
    ENCODER_B_RL_PIN,
    ENCODER_B_RR_PIN
};

/* Direction signs align encoder phase with the robot frame */
static const int8_t encoder_direction_sign[MOTOR_COUNT] = {
    -1, // FRONT_LEFT
    1,  // FRONT_RIGHT
    -1, // REAR_LEFT
    1   // REAR_RIGHT
};

static volatile uint32_t encoder_last_time_us[MOTOR_COUNT] = {0};
static volatile uint8_t encoder_last_state[MOTOR_COUNT] = {0};
static volatile uint32_t encoder_filtered_events[MOTOR_COUNT] = {0};
static volatile uint32_t encoder_invalid_transitions[MOTOR_COUNT] = {0};

static bno055_imu_t imu_sensor = {0};
static bool imu_offsets_published = false;
static bool imu_offsets_applied = false;
static bool imu_offsets_boot_report_pending = false;
static uint8_t imu_offsets_attempts = 0;
static uint32_t imu_offsets_last_attempt_ms = 0;

/* Automatically-applied BNO055 offsets captured on 2025-11-16 (evening run). */
static const bno055_imu_offsets_t k_static_imu_offsets = {
    .has_accel = true,
    .has_gyro = true,
    .has_mag = true,
    .has_sic = true,
    .accel = { .x = -13, .y = 29, .z = -28, .r = 1000 },
    .gyro = { .x = -2, .y = -2, .z = 0 },
    .mag = { .x = 75, .y = 626, .z = -625, .r = 888 },
    .sic = {
        .sic_0 = 16384,
        .sic_1 = 0,
        .sic_2 = 0,
        .sic_3 = 0,
        .sic_4 = 16384,
        .sic_5 = 0,
        .sic_6 = 0,
        .sic_7 = 0,
        .sic_8 = 16384
    }
};

/* Clamp helper avoids sprinkling fmin/fmax calls through the PID code */
static inline float clampf(float value, float min_val, float max_val) {
    if (value < min_val) {
        return min_val;
    }
    if (value > max_val) {
        return max_val;
    }
    return value;
}

/* Apply the signed PWM output to the Cytron driver */
static inline void motor_apply_output(motor_t* motor, int32_t speed) {
    uint16_t duty = (uint16_t)(speed >= 0 ? speed : -speed);
    if (duty > 65535U) {
        duty = 65535U;
    }

    if (speed >= 0) {
        gpio_put(motor->channels.dir_pin, 0);
    } else {
        gpio_put(motor->channels.dir_pin, 1);
    }

    pwm_set_gpio_level(motor->channels.pwm_pin, duty);
}

/* Translate the interrupt source back to a motor index */
static inline motor_index_t encoder_index_from_gpio(uint gpio) {
    switch (gpio) {
        case ENCODER_A_FL_PIN:
        case ENCODER_B_FL_PIN:
            return MOTOR_FRONT_LEFT;
        case ENCODER_A_FR_PIN:
        case ENCODER_B_FR_PIN:
            return MOTOR_FRONT_RIGHT;
        case ENCODER_A_RL_PIN:
        case ENCODER_B_RL_PIN:
            return MOTOR_REAR_LEFT;
        case ENCODER_A_RR_PIN:
        case ENCODER_B_RR_PIN:
            return MOTOR_REAR_RIGHT;
        default:
            return (motor_index_t)-1;
    }
}

static void encoder_irq_handler(uint gpio, uint32_t events) {
    (void)events;

    motor_index_t index = encoder_index_from_gpio(gpio);
    if (index < 0 || index >= MOTOR_COUNT) {
        return;
    }

    uint32_t now_us = time_us_32();
    if ((now_us - encoder_last_time_us[index]) < ENCODER_MIN_INTERVAL_US) {
        encoder_filtered_events[index]++;
        return;
    }

    bool a_level = gpio_get(encoder_a_pins[index]);
    bool b_level = gpio_get(encoder_b_pins[index]);
    uint8_t new_state = (uint8_t)((a_level ? 2U : 0U) | (b_level ? 1U : 0U));
    uint8_t prev_state = encoder_last_state[index];

    static const int8_t transition_table[4][4] = {
        { 0,  1, -1,  0 },
        { -1, 0,  0,  1 },
        { 1,  0,  0, -1 },
        { 0, -1,  1,  0 }
    };

    int8_t delta = transition_table[prev_state][new_state];
    if (delta != 0) {
        g_encoder_counts[index] += (int32_t)(delta * encoder_direction_sign[index]);
        encoder_last_state[index] = new_state;
        encoder_last_time_us[index] = now_us;
    } else if (new_state != prev_state) {
        encoder_invalid_transitions[index]++;
        encoder_last_state[index] = new_state;
        encoder_last_time_us[index] = now_us;
    }
}

/* Configure PWM slices and direction GPIOs */
void motor_driver_init_pins(void) {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        gpio_init(motor_channel_map[i].dir_pin);
        gpio_set_dir(motor_channel_map[i].dir_pin, GPIO_OUT);
        gpio_put(motor_channel_map[i].dir_pin, 0);

        gpio_set_function(motor_channel_map[i].pwm_pin, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(motor_channel_map[i].pwm_pin);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_wrap(&cfg, 65535U);
        pwm_config_set_clkdiv(&cfg, 1.0f);
        pwm_init(slice, &cfg, true);
        pwm_set_gpio_level(motor_channel_map[i].pwm_pin, 0);
    }
}

/* Set encoder GPIOs to inputs with hysteresis, seed state machine, and enable IRQs */
void encoder_init(void) {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        gpio_init(encoder_a_pins[i]);
        gpio_set_dir(encoder_a_pins[i], GPIO_IN);
        gpio_pull_up(encoder_a_pins[i]);
        gpio_set_input_hysteresis_enabled(encoder_a_pins[i], true);
        gpio_set_slew_rate(encoder_a_pins[i], GPIO_SLEW_RATE_SLOW);

        gpio_init(encoder_b_pins[i]);
        gpio_set_dir(encoder_b_pins[i], GPIO_IN);
        gpio_pull_up(encoder_b_pins[i]);
        gpio_set_input_hysteresis_enabled(encoder_b_pins[i], true);
        gpio_set_slew_rate(encoder_b_pins[i], GPIO_SLEW_RATE_SLOW);
        bool a_level = gpio_get(encoder_a_pins[i]);
        bool b_level = gpio_get(encoder_b_pins[i]);
        encoder_last_state[i] = (uint8_t)((a_level ? 2U : 0U) | (b_level ? 1U : 0U));
        encoder_last_time_us[i] = time_us_32();
    }

    gpio_set_irq_enabled_with_callback(ENCODER_A_FL_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &encoder_irq_handler);
    gpio_set_irq_enabled(ENCODER_B_FL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_A_FR_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_B_FR_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_A_RL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_B_RL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_A_RR_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_B_RR_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

/* Populate motor structures with PID gains, slew limits, and housekeeping state */
void motor_init(motor_t* motor,
                motor_index_t index,
                float kp, float ki, float kd,
                float i_max,
                float max_output_step,
                float max_target_slew,
                int32_t kick_pwm) {
    motor->channels = motor_channel_map[index];
    motor->ticks = &g_encoder_counts[index];
    motor->target_tps = 0.0f;
    motor->active_target_tps = 0.0f;
    motor->kp = kp;
    motor->ki = ki;
    motor->kd = kd;
    motor->integral = 0.0f;
    motor->last_error = 0.0f;
    motor->i_max = i_max;
    motor->max_output_step = max_output_step;
    motor->max_target_slew = max_target_slew;
    motor->filtered_velocity = 0.0f;
    motor->last_measured_tps = 0.0f;
    motor->last_update_ms = to_ms_since_boot(get_absolute_time());
    motor->last_position = *(motor->ticks);
    motor->pwm_output = 0;
    motor->command_active = false;
    motor->kick_direction = 0;
    motor->kick_ms_remaining = 0;
    motor->kick_pwm = kick_pwm;
    motor->filtered_velocity = 0.0f;
    motor->last_measured_tps = 0.0f;
    motor_apply_output(motor, 0);
}

/* Reset the PID state and drive outputs to zero */
void motor_stop(motor_t* motor) {
    motor->target_tps = 0.0f;
    motor->active_target_tps = 0.0f;
    motor->command_active = false;
    motor->integral = 0.0f;
    motor->last_error = 0.0f;
    motor->kick_ms_remaining = 0;
    motor->kick_direction = 0;
    motor->pwm_output = 0;
    motor->filtered_velocity = 0.0f;
    motor->last_measured_tps = 0.0f;
    motor->last_position = *(motor->ticks);
    motor->last_update_ms = to_ms_since_boot(get_absolute_time());
    motor_apply_output(motor, 0);
}

/* Update the commanded ticks per second, handling kick-starts and direction changes */
void motor_set_target_tps(motor_t* motor, float target_tps) {
    float abs_target = fabsf(target_tps);
    bool was_active = motor->command_active;
    int8_t previous_direction = motor->kick_direction;

    if (abs_target < STOP_TPS_THRESHOLD) {
        motor_stop(motor);
        return;
    }

    motor->target_tps = target_tps;
    motor->command_active = true;

    int8_t new_direction = (target_tps >= 0.0f) ? 1 : -1;
    if (!was_active || new_direction != previous_direction) {
        motor->active_target_tps = 0.0f;
        motor->kick_ms_remaining = KICK_DURATION_MS;
        motor->kick_direction = new_direction;
        motor->integral = 0.0f;
        motor->last_error = 0.0f;
        motor->filtered_velocity = 0.0f;
        motor->last_measured_tps = 0.0f;
        motor->last_position = *(motor->ticks);
    }
}

/* PID update: estimate velocity, slew the target, and integrate the controller */
void motor_update(motor_t* motor, uint32_t now_ms) {
    int32_t current_ticks = *(motor->ticks);

    if (motor->last_update_ms == 0U) {
        motor->last_update_ms = now_ms;
        motor->last_position = current_ticks;
        return;
    }

    uint32_t delta_ms = now_ms - motor->last_update_ms;
    if (delta_ms == 0U) {
        return;
    }

    float dt_s = (float)delta_ms / 1000.0f;
    int32_t delta_ticks = current_ticks - motor->last_position;
    float raw_tps = (float)delta_ticks * (1000.0f / (float)delta_ms);

    motor->filtered_velocity = (VELOCITY_FILTER_ALPHA * raw_tps) +
                               ((1.0f - VELOCITY_FILTER_ALPHA) * motor->filtered_velocity);
    motor->last_measured_tps = motor->filtered_velocity;

    float target_delta = motor->target_tps - motor->active_target_tps;
    float max_target_step = motor->max_target_slew * dt_s;
    if (target_delta > max_target_step) {
        target_delta = max_target_step;
    } else if (target_delta < -max_target_step) {
        target_delta = -max_target_step;
    }
    motor->active_target_tps += target_delta;

    if (!motor->command_active && fabsf(motor->active_target_tps) < STOP_TPS_THRESHOLD) {
        motor->active_target_tps = 0.0f;
    }

    float error = motor->active_target_tps - motor->filtered_velocity;

    if (!motor->command_active) {
        motor->integral = 0.0f;
        motor->last_error = error;
        motor->pwm_output = 0;
        motor_apply_output(motor, 0);
        motor->last_position = current_ticks;
        motor->last_update_ms = now_ms;
        return;
    }

    motor->integral += error * dt_s;
    motor->integral = clampf(motor->integral, -motor->i_max, motor->i_max);

    float derivative = dt_s > 0.0f ? (error - motor->last_error) / dt_s : 0.0f;
    float pid_increment = (motor->kp * error) + (motor->ki * motor->integral) + (motor->kd * derivative);

    float desired_pwm = (float)motor->pwm_output + pid_increment;
    float max_pwm_step = motor->max_output_step * dt_s;
    float pwm_delta = desired_pwm - (float)motor->pwm_output;

    if (pwm_delta > max_pwm_step) {
        pwm_delta = max_pwm_step;
    } else if (pwm_delta < -max_pwm_step) {
        pwm_delta = -max_pwm_step;
    }

    float new_pwm = (float)motor->pwm_output + pwm_delta;

    if (motor->kick_ms_remaining > 0U) {
        motor->kick_ms_remaining = (motor->kick_ms_remaining > delta_ms) ?
                                   (motor->kick_ms_remaining - delta_ms) : 0U;
        int32_t min_pwm = motor->kick_pwm;
        if (motor->kick_direction >= 0) {
            if (new_pwm < (float)min_pwm) {
                new_pwm = (float)min_pwm;
            }
        } else {
            if (new_pwm > -(float)min_pwm) {
                new_pwm = -(float)min_pwm;
            }
        }
    } else if (fabsf(new_pwm) > 0.0f && fabsf(new_pwm) < MIN_EFFECTIVE_PWM) {
        new_pwm = (new_pwm >= 0.0f) ? MIN_EFFECTIVE_PWM : -MIN_EFFECTIVE_PWM;
    }

    new_pwm = clampf(new_pwm, -PWM_LIMIT, PWM_LIMIT);

    motor->pwm_output = (int32_t)new_pwm;
    motor_apply_output(motor, motor->pwm_output);

    motor->last_error = error;
    motor->last_position = current_ticks;
    motor->last_update_ms = now_ms;
}

int32_t motor_get_ticks(motor_index_t index) {
    return g_encoder_counts[index];
}

/* Helper for radians-to-ticks conversion used by the ROS subscription callback */
float ticks_per_second_from_radians(float rad_per_sec) {
    return rad_per_sec * MOTOR_TICKS_PER_RAD;
}

static inline float radians_from_ticks_per_second(float ticks_per_sec) {
    return ticks_per_sec / MOTOR_TICKS_PER_RAD;
}

static inline float radians_from_ticks(int32_t ticks) {
    return (float)ticks / MOTOR_TICKS_PER_RAD;
}

static void wheel_command_callback(const void* msgin) {
    const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*)msgin;
    size_t count = msg->data.size < MOTOR_COUNT ? msg->data.size : MOTOR_COUNT;

    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        float rad_per_sec = (i < count) ? msg->data.data[i] : 0.0f;
        rad_per_sec = clampf(rad_per_sec, -MAX_COMMANDED_RADS_PER_SEC, MAX_COMMANDED_RADS_PER_SEC);
        float target_tps = ticks_per_second_from_radians(rad_per_sec);
        motor_set_target_tps(&motors[i], target_tps);
    }

    last_command_ms = to_ms_since_boot(get_absolute_time());
}

static void init_messages(void) {
    motor_state_msg.data.data = motor_state_buffer;
    motor_state_msg.data.size = 0;
    motor_state_msg.data.capacity = sizeof(motor_state_buffer);

    tick_msg.data.data = tick_buffer;
    tick_msg.data.size = 0;
    tick_msg.data.capacity = sizeof(tick_buffer);

    wheel_state_msg.data.data = wheel_state_buffer;
    wheel_state_msg.data.size = MOTOR_COUNT * 2;
    wheel_state_msg.data.capacity = MOTOR_COUNT * 2;
    wheel_state_msg.layout.dim.size = 0;
    wheel_state_msg.layout.dim.capacity = 0;
    wheel_state_msg.layout.data_offset = 0;

    wheel_command_msg.data.data = wheel_command_buffer;
    wheel_command_msg.data.size = 0;
    wheel_command_msg.data.capacity = MOTOR_COUNT;
    wheel_command_msg.layout.dim.size = 0;
    wheel_command_msg.layout.dim.capacity = 0;
    wheel_command_msg.layout.data_offset = 0;

    memset(&imu_msg, 0, sizeof(imu_msg));
    imu_msg.header.frame_id.data = imu_frame_id;
    imu_msg.header.frame_id.size = strlen(imu_frame_id);
    imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);

    imu_calib_msg.data.data = imu_calib_buffer;
    imu_calib_msg.data.size = 4;
    imu_calib_msg.data.capacity = 4;
    imu_calib_msg.layout.dim.size = 0;
    imu_calib_msg.layout.dim.capacity = 0;
    imu_calib_msg.layout.data_offset = 0;

    imu_offsets_msg.data.data = imu_offsets_buffer;
    imu_offsets_msg.data.size = 0;
    imu_offsets_msg.data.capacity = sizeof(imu_offsets_buffer);

    imu_offsets_array_msg.data.data = imu_offsets_array_buffer;
    imu_offsets_array_msg.data.size = 24;
    imu_offsets_array_msg.data.capacity = 24;
    imu_offsets_array_msg.layout.dim.size = 0;
    imu_offsets_array_msg.layout.dim.capacity = 0;
    imu_offsets_array_msg.layout.data_offset = 0;
}

/* Bring up PWM, encoders, and initialise every motor instance */
static void setup_motors(void) {
    motor_driver_init_pins();
    encoder_init();

    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        motor_init(&motors[i], (motor_index_t)i,
                   PID_KP, PID_KI, PID_KD,
                   50.0f,
                   MAX_OUTPUT_STEP_PER_SEC,
                   MAX_TARGET_SLEW_TPS_PER_SEC,
                   KICK_PWM_LEVEL);
    }
}

/* Configure I2C pins and bring the BNO055 online in NDOF mode */
static void apply_static_imu_offsets(void) {
    if (!k_static_imu_offsets.has_accel &&
        !k_static_imu_offsets.has_gyro &&
        !k_static_imu_offsets.has_mag &&
        !k_static_imu_offsets.has_sic) {
        return;
    }

    if (!bno055_imu_write_offsets(&imu_sensor, &k_static_imu_offsets)) {
        printf("Failed to apply stored BNO055 offsets\n");
        imu_offsets_applied = false;
    } else {
        imu_offsets_applied = true;
        imu_offsets_boot_report_pending = true;
    }

    imu_offsets_attempts++;
    imu_offsets_last_attempt_ms = to_ms_since_boot(get_absolute_time());
}

static void setup_imu(void) {
    i2c_init(IMU_I2C_PORT, 400 * 1000);
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);

    if (!bno055_imu_init(&imu_sensor, IMU_I2C_PORT, BNO055_I2C_ADDRESS)) {
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(150);
            gpio_put(LED_PIN, 0);
            sleep_ms(850);
        }
    }

    apply_static_imu_offsets();
}

/* Initialise the micro-ROS node, publishers, subscriber, and executor */
static void setup_microros(void) {
    allocator = rcl_get_default_allocator();
    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK) {
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(200);
            gpio_put(LED_PIN, 0);
            sleep_ms(200);
        }
    }

    rc = rclc_node_init_default(&node, "pico_motor_driver_node", "", &support);
    if (rc != RCL_RET_OK) {
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(500);
            gpio_put(LED_PIN, 0);
            sleep_ms(500);
        }
    }

    rc = rclc_publisher_init_default(
        &motor_state_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "pico_motor_driver");
    if (rc != RCL_RET_OK) {
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(100);
            gpio_put(LED_PIN, 0);
            sleep_ms(900);
        }
    }

    rc = rclc_publisher_init_default(
        &wheel_state_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "wheel_states");
    if (rc != RCL_RET_OK) {
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(350);
            gpio_put(LED_PIN, 0);
            sleep_ms(350);
        }
    }

    rc = rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw");
    if (rc != RCL_RET_OK) {
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(250);
            gpio_put(LED_PIN, 0);
            sleep_ms(250);
        }
    }

    rc = rclc_publisher_init_default(
        &imu_calib_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "imu/calibration");
    if (rc != RCL_RET_OK) {
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(600);
            gpio_put(LED_PIN, 0);
            sleep_ms(400);
        }
    }

    rc = rclc_publisher_init_default(
        &imu_offsets_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "imu/calibration_offsets");
    if (rc != RCL_RET_OK) {
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(700);
            gpio_put(LED_PIN, 0);
            sleep_ms(300);
        }
    }

    rc = rclc_publisher_init_default(
        &imu_offsets_array_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "imu/calibration_offsets_raw");
    if (rc != RCL_RET_OK) {
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(750);
            gpio_put(LED_PIN, 0);
            sleep_ms(250);
        }
    }

    rc = rclc_publisher_init_default(
        &tick_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "tick_counter");
    if (rc != RCL_RET_OK) {
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(400);
            gpio_put(LED_PIN, 0);
            sleep_ms(400);
        }
    }

    rc = rclc_subscription_init_default(
        &wheel_cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "wheel_commands");
    if (rc != RCL_RET_OK) {
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(50);
            gpio_put(LED_PIN, 0);
            sleep_ms(50);
        }
    }

    rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (rc != RCL_RET_OK) {
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(300);
            gpio_put(LED_PIN, 0);
            sleep_ms(300);
        }
    }

    rc = rclc_executor_add_subscription(
        &executor,
        &wheel_cmd_sub,
        &wheel_command_msg,
        &wheel_command_callback,
        ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(150);
            gpio_put(LED_PIN, 0);
            sleep_ms(150);
        }
    }
}

/* Velocity/position publisher mirroring the production driver interface */
static void publish_wheel_state(void) {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        wheel_state_buffer[i] = radians_from_ticks_per_second(motors[i].last_measured_tps);
        wheel_state_buffer[i + MOTOR_COUNT] = radians_from_ticks(motor_get_ticks((motor_index_t)i));
    }

    rcl_publish(&wheel_state_pub, &wheel_state_msg, NULL);
}

/* Read the IMU and publish ROS-standard orientation, angular velocity, and acceleration */
static void publish_imu_state(void) {
    bno055_imu_sample_t sample = {0};
    if (!bno055_imu_read(&imu_sensor, &sample)) {
        return;
    }

    imu_msg.orientation.w = sample.orientation_w;
    imu_msg.orientation.x = sample.orientation_x;
    imu_msg.orientation.y = sample.orientation_y;
    imu_msg.orientation.z = sample.orientation_z;

    imu_msg.angular_velocity.x = sample.angular_velocity_x;
    imu_msg.angular_velocity.y = sample.angular_velocity_y;
    imu_msg.angular_velocity.z = sample.angular_velocity_z;

    imu_msg.linear_acceleration.x = sample.linear_acceleration_x;
    imu_msg.linear_acceleration.y = sample.linear_acceleration_y;
    imu_msg.linear_acceleration.z = sample.linear_acceleration_z;

    int64_t time_ns = rmw_uros_epoch_nanos();
    imu_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000LL);
    imu_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);

    rcl_publish(&imu_pub, &imu_msg, NULL);
}

/* Periodic telemetry payload: per-wheel filtered velocity, PWM, ticks, and encoder diagnostics */
static void publish_motor_state(uint32_t now_ms) {
    (void)now_ms;

    float target_sum = 0.0f;

    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        target_sum += motors[i].active_target_tps;
    }
    float target_avg = target_sum / (float)MOTOR_COUNT;

    static uint32_t last_filtered[MOTOR_COUNT] = {0};
    static uint32_t last_invalid[MOTOR_COUNT] = {0};
    uint32_t filtered_delta[MOTOR_COUNT];
    uint32_t invalid_delta[MOTOR_COUNT];

    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        uint32_t filtered_snapshot = encoder_filtered_events[i];
        uint32_t invalid_snapshot = encoder_invalid_transitions[i];
        filtered_delta[i] = filtered_snapshot - last_filtered[i];
        invalid_delta[i] = invalid_snapshot - last_invalid[i];
        last_filtered[i] = filtered_snapshot;
        last_invalid[i] = invalid_snapshot;
    }

    int written = snprintf(motor_state_buffer, sizeof(motor_state_buffer),
                           "Target: %.0f (tks/s) (tks/sec,pwm,ticks|inv,flt) "
                           "FL(%.0f,%d,%ld|%lu,%lu), FR(%.0f,%d,%ld|%lu,%lu), "
                           "RL(%.0f,%d,%ld|%lu,%lu), RR(%.0f,%d,%ld|%lu,%lu)",
                           target_avg,
                           motors[MOTOR_FRONT_LEFT].last_measured_tps,
                           motors[MOTOR_FRONT_LEFT].pwm_output,
                           (long)motor_get_ticks(MOTOR_FRONT_LEFT),
                           (unsigned long)invalid_delta[MOTOR_FRONT_LEFT],
                           (unsigned long)filtered_delta[MOTOR_FRONT_LEFT],
                           motors[MOTOR_FRONT_RIGHT].last_measured_tps,
                           motors[MOTOR_FRONT_RIGHT].pwm_output,
                           (long)motor_get_ticks(MOTOR_FRONT_RIGHT),
                           (unsigned long)invalid_delta[MOTOR_FRONT_RIGHT],
                           (unsigned long)filtered_delta[MOTOR_FRONT_RIGHT],
                           motors[MOTOR_REAR_LEFT].last_measured_tps,
                           motors[MOTOR_REAR_LEFT].pwm_output,
                           (long)motor_get_ticks(MOTOR_REAR_LEFT),
                           (unsigned long)invalid_delta[MOTOR_REAR_LEFT],
                           (unsigned long)filtered_delta[MOTOR_REAR_LEFT],
                           motors[MOTOR_REAR_RIGHT].last_measured_tps,
                           motors[MOTOR_REAR_RIGHT].pwm_output,
                           (long)motor_get_ticks(MOTOR_REAR_RIGHT),
                           (unsigned long)invalid_delta[MOTOR_REAR_RIGHT],
                           (unsigned long)filtered_delta[MOTOR_REAR_RIGHT]);

    if (written > 0 && written < (int)sizeof(motor_state_buffer)) {
        motor_state_msg.data.size = (size_t)written;
        rcl_publish(&motor_state_pub, &motor_state_msg, NULL);
    }
}

/* Raw tick counter publisher for quick sanity checks via /tick_counter */
static void publish_tick_state(void) {
    int written = snprintf(tick_buffer, sizeof(tick_buffer),
                           "FL(%ld), FR(%ld), RL(%ld), RR(%ld)",
                           (long)motor_get_ticks(MOTOR_FRONT_LEFT),
                           (long)motor_get_ticks(MOTOR_FRONT_RIGHT),
                           (long)motor_get_ticks(MOTOR_REAR_LEFT),
                           (long)motor_get_ticks(MOTOR_REAR_RIGHT));
    if (written > 0 && written < (int)sizeof(tick_buffer)) {
        tick_msg.data.size = (size_t)written;
        rcl_publish(&tick_pub, &tick_msg, NULL);
    }
}

static bool publish_imu_offset_snapshot(void) {
    bno055_imu_offsets_t offsets = {0};
    if (!bno055_imu_read_offsets(&imu_sensor, &offsets)) {
        return false;
    }

    imu_offsets_array_buffer[0] = (int16_t)(offsets.has_accel ? 1 : 0);
    imu_offsets_array_buffer[1] = (int16_t)(offsets.has_gyro ? 1 : 0);
    imu_offsets_array_buffer[2] = (int16_t)(offsets.has_mag ? 1 : 0);
    imu_offsets_array_buffer[3] = (int16_t)(offsets.has_sic ? 1 : 0);
    imu_offsets_array_buffer[4] = offsets.accel.x;
    imu_offsets_array_buffer[5] = offsets.accel.y;
    imu_offsets_array_buffer[6] = offsets.accel.z;
    imu_offsets_array_buffer[7] = offsets.accel.r;
    imu_offsets_array_buffer[8] = offsets.gyro.x;
    imu_offsets_array_buffer[9] = offsets.gyro.y;
    imu_offsets_array_buffer[10] = offsets.gyro.z;
    imu_offsets_array_buffer[11] = offsets.mag.x;
    imu_offsets_array_buffer[12] = offsets.mag.y;
    imu_offsets_array_buffer[13] = offsets.mag.z;
    imu_offsets_array_buffer[14] = offsets.mag.r;
    imu_offsets_array_buffer[15] = offsets.sic.sic_0;
    imu_offsets_array_buffer[16] = offsets.sic.sic_1;
    imu_offsets_array_buffer[17] = offsets.sic.sic_2;
    imu_offsets_array_buffer[18] = offsets.sic.sic_3;
    imu_offsets_array_buffer[19] = offsets.sic.sic_4;
    imu_offsets_array_buffer[20] = offsets.sic.sic_5;
    imu_offsets_array_buffer[21] = offsets.sic.sic_6;
    imu_offsets_array_buffer[22] = offsets.sic.sic_7;
    imu_offsets_array_buffer[23] = offsets.sic.sic_8;

    rcl_publish(&imu_offsets_array_pub, &imu_offsets_array_msg, NULL);

    int written = snprintf(imu_offsets_buffer,
                           sizeof(imu_offsets_buffer),
                           "/* BNO055 offsets captured at %lu ms */ static const bno055_imu_offsets_t k_static_imu_offsets = { .has_accel = %s, .has_gyro = %s, .has_mag = %s, .has_sic = %s, .accel = { .x = %d, .y = %d, .z = %d, .r = %d }, .gyro = { .x = %d, .y = %d, .z = %d }, .mag = { .x = %d, .y = %d, .z = %d, .r = %d }, .sic = { .sic_0 = %d, .sic_1 = %d, .sic_2 = %d, .sic_3 = %d, .sic_4 = %d, .sic_5 = %d, .sic_6 = %d, .sic_7 = %d, .sic_8 = %d } };\n",
                           (unsigned long)to_ms_since_boot(get_absolute_time()),
                           offsets.has_accel ? "true" : "false",
                           offsets.has_gyro ? "true" : "false",
                           offsets.has_mag ? "true" : "false",
                           offsets.has_sic ? "true" : "false",
                           (int)offsets.accel.x,
                           (int)offsets.accel.y,
                           (int)offsets.accel.z,
                           (int)offsets.accel.r,
                           (int)offsets.gyro.x,
                           (int)offsets.gyro.y,
                           (int)offsets.gyro.z,
                           (int)offsets.mag.x,
                           (int)offsets.mag.y,
                           (int)offsets.mag.z,
                           (int)offsets.mag.r,
                           (int)offsets.sic.sic_0,
                           (int)offsets.sic.sic_1,
                           (int)offsets.sic.sic_2,
                           (int)offsets.sic.sic_3,
                           (int)offsets.sic.sic_4,
                           (int)offsets.sic.sic_5,
                           (int)offsets.sic.sic_6,
                           (int)offsets.sic.sic_7,
                           (int)offsets.sic.sic_8);

    if (written <= 0 || written >= (int)sizeof(imu_offsets_buffer)) {
        return false;
    }

    imu_offsets_buffer[written] = '\0';
    imu_offsets_msg.data.size = (size_t)written;
    rcl_publish(&imu_offsets_pub, &imu_offsets_msg, NULL);
    return true;
}

static void publish_imu_calibration(void) {
    uint8_t sys = 0;
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;

    if (!bno055_imu_get_calibration(&imu_sensor, &sys, &gyro, &accel, &mag)) {
        return;
    }

    imu_calib_buffer[0] = (float)sys;
    imu_calib_buffer[1] = (float)gyro;
    imu_calib_buffer[2] = (float)accel;
    imu_calib_buffer[3] = (float)mag;

    rcl_publish(&imu_calib_pub, &imu_calib_msg, NULL);

    if ((sys == 3U) && (gyro == 3U) && (accel == 3U) && (mag == 3U) && !imu_offsets_published) {
        if (publish_imu_offset_snapshot()) {
            imu_offsets_published = true;
        }
    }

    if (mag >= 1U) {
        imu_offsets_applied = true;
    } else if (imu_offsets_attempts > 0U && imu_offsets_attempts < 5U) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if ((now_ms - imu_offsets_last_attempt_ms) >= 1500U) {
            imu_offsets_applied = false;
        }
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(1000); // Wait for hardware to stabilise

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    setup_motors();
    setup_imu();
    init_messages();

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    const int agent_timeout_ms = 1000;
    const uint32_t agent_deadline_ms = to_ms_since_boot(get_absolute_time()) + AGENT_CONNECT_TIMEOUT_MS;

    while (rmw_uros_ping_agent(agent_timeout_ms, 1) != RCL_RET_OK) {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(900);

        if (to_ms_since_boot(get_absolute_time()) >= agent_deadline_ms) {
            goto agent_connection_failure;
        }
    }

    setup_microros();

    gpio_put(LED_PIN, 1);

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    last_command_ms = now_ms;
    uint32_t last_control_ms = now_ms;
    uint32_t last_motor_pub_ms = now_ms;
    uint32_t last_imu_pub_ms = now_ms;
    uint32_t last_imu_calib_pub_ms = now_ms;
    uint32_t last_wheel_state_pub_ms = now_ms;
    uint32_t last_tick_pub_ms = now_ms;

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        now_ms = to_ms_since_boot(get_absolute_time());

        if ((now_ms - last_command_ms) > COMMAND_TIMEOUT_MS) {
            for (size_t i = 0; i < MOTOR_COUNT; ++i) {
                if (motors[i].command_active) {
                    motor_stop(&motors[i]);
                }
            }
            last_command_ms = now_ms;
        }

        if ((now_ms - last_control_ms) >= CONTROL_INTERVAL_MS) {
            last_control_ms = now_ms;
            for (size_t i = 0; i < MOTOR_COUNT; ++i) {
                motor_update(&motors[i], now_ms);
            }
        }

        if ((now_ms - last_motor_pub_ms) >= MOTOR_STATE_PUBLISH_MS) {
            last_motor_pub_ms = now_ms;
            publish_motor_state(now_ms);
        }

        if ((now_ms - last_imu_pub_ms) >= IMU_PUBLISH_MS) {
            last_imu_pub_ms = now_ms;
            publish_imu_state();
        }

        if ((now_ms - last_wheel_state_pub_ms) >= WHEEL_STATE_PUBLISH_MS) {
            last_wheel_state_pub_ms = now_ms;
            publish_wheel_state();
        }

        if ((now_ms - last_tick_pub_ms) >= TICK_PUBLISH_MS) {
            last_tick_pub_ms = now_ms;
            publish_tick_state();
        }

        if (!imu_offsets_applied && (imu_offsets_attempts > 0U) && (imu_offsets_attempts < 5U)) {
            if ((now_ms - imu_offsets_last_attempt_ms) >= 500U) {
                apply_static_imu_offsets();
            }
        }

        if (imu_offsets_boot_report_pending) {
            if ((now_ms - imu_offsets_last_attempt_ms) >= 250U) {
                if (publish_imu_offset_snapshot()) {
                    imu_offsets_boot_report_pending = false;
                }
            }
        }

        if ((now_ms - last_imu_calib_pub_ms) >= 1000U) {
            last_imu_calib_pub_ms = now_ms;
            publish_imu_calibration();
        }
    }

agent_connection_failure:
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(900);
    }

    return 0;
}
