/**
 * @file board.c
 * @brief Pico SDK peripheral subset hosted by the pinned micro T-Kernel BSP.
 * @details No SDK startup, interrupt dispatcher, scheduler or spinlocks are
 * linked. The kernel installs GPIO IRQ 13 on core 0. D-ADAPTER: SDK ABI names,
 * native SDK integer types and compiler intrinsics stay in this adapter.
 * Verify edge loss, current transients and timing on actual hardware.
 */
#include "board.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#include "app_main.h"
#include "robot_config.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/resets.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/psm.h"
#include "hardware/structs/timer.h"
#include "hardware/structs/watchdog.h"

#define BOARD_ACCEL_ADDRESS 0x19U
#define BOARD_ACCEL_CTRL1 0x20U
#define BOARD_ACCEL_CTRL4 0x23U
#define BOARD_ACCEL_STATUS 0x27U
#define BOARD_ACCEL_DATA 0xA8U
#define BOARD_ACCEL_ODR_ENABLE 0x57U
#define BOARD_ACCEL_BDU_HR_2G 0x88U
#define BOARD_ACCEL_AXES 3U
#define BOARD_ACCEL_BYTES 6U
#define BOARD_ADC_TIMEOUT_US 100U
/* IO_BANK0 latches four status bits per GPIO and eight GPIOs per bank. */
#define BOARD_IRQ_BANK(pin) ((pin) / 8U)
#define BOARD_IRQ_SHIFT(pin) (((pin) % 8U) * 4U)
#define BOARD_EDGE_BITS(pin) (0xCU << BOARD_IRQ_SHIFT(pin))
#define BOARD_LEFT_BANK BOARD_IRQ_BANK(ROBOT_PIN_ENCODER_LEFT_A)
#define BOARD_RIGHT_BANK BOARD_IRQ_BANK(ROBOT_PIN_ENCODER_RIGHT_A)
#define BOARD_ECHO_BANK BOARD_IRQ_BANK(ROBOT_PIN_ECHO)
#if ROBOT_ENCODER_CHANNELS == 2
#define BOARD_LEFT_MASK (BOARD_EDGE_BITS(ROBOT_PIN_ENCODER_LEFT_A) | \
                         BOARD_EDGE_BITS(ROBOT_PIN_ENCODER_LEFT_B))
#define BOARD_RIGHT_MASK (BOARD_EDGE_BITS(ROBOT_PIN_ENCODER_RIGHT_A) | \
                          BOARD_EDGE_BITS(ROBOT_PIN_ENCODER_RIGHT_B))
#else
#define BOARD_LEFT_MASK BOARD_EDGE_BITS(ROBOT_PIN_ENCODER_LEFT_A)
#define BOARD_RIGHT_MASK BOARD_EDGE_BITS(ROBOT_PIN_ENCODER_RIGHT_A)
#endif
#define BOARD_ECHO_MASK BOARD_EDGE_BITS(ROBOT_PIN_ECHO)
#if (BOARD_IRQ_BANK(ROBOT_PIN_ENCODER_LEFT_A) != \
     BOARD_IRQ_BANK(ROBOT_PIN_ENCODER_LEFT_B)) || \
    (BOARD_IRQ_BANK(ROBOT_PIN_ENCODER_RIGHT_A) != \
     BOARD_IRQ_BANK(ROBOT_PIN_ENCODER_RIGHT_B))
#error "Both encoder inputs of one wheel must share an IO_BANK0 bank"
#endif
#define BOARD_MOTOR_SLICE_LEFT 4U
#define BOARD_MOTOR_SLICE_RIGHT 5U
#define BOARD_SERVO_SLICE 6U
/* RP2040 GPIO n drives PWM slice (n >> 1) & 7, channel A when n is even.
 * Motor setup loops LEFT_A..RIGHT_B, and board_apply_motor drives A then B.
 */
#define BOARD_PWM_SLICE(pin) (((pin) >> 1U) & 7U)
#if (BOARD_PWM_SLICE(ROBOT_PIN_MOTOR_LEFT_A) != BOARD_MOTOR_SLICE_LEFT) || \
    (BOARD_PWM_SLICE(ROBOT_PIN_MOTOR_RIGHT_A) != BOARD_MOTOR_SLICE_RIGHT) || \
    (0U != (ROBOT_PIN_MOTOR_LEFT_A & 1U)) || \
    (0U != (ROBOT_PIN_MOTOR_RIGHT_A & 1U)) || \
    (ROBOT_PIN_MOTOR_LEFT_B != (ROBOT_PIN_MOTOR_LEFT_A + 1U)) || \
    (ROBOT_PIN_MOTOR_RIGHT_A != (ROBOT_PIN_MOTOR_LEFT_B + 1U)) || \
    (ROBOT_PIN_MOTOR_RIGHT_B != (ROBOT_PIN_MOTOR_RIGHT_A + 1U))
#error "Motor pins must be contiguous A/B channel pairs of PWM slices 4 and 5"
#endif
#if (BOARD_PWM_SLICE(ROBOT_PIN_SERVO) != BOARD_SERVO_SLICE) || \
    (0U != (ROBOT_PIN_SERVO & 1U))
#error "Servo pin must be channel A of PWM slice 6"
#endif
#if (ROBOT_PWM_WRAP != ((ROBOT_CLOCK_HZ / ROBOT_PWM_HZ) - 1U)) || \
    (ROBOT_CLOCK_HZ != (125U * 1000000U))
#error "Motor wrap and the servo divider of 125 assume a 125 MHz clock"
#endif
#define BOARD_ECHO_MIN_US 100U
#define BOARD_ECHO_MAX_US 25000U
#define BOARD_WATCHDOG_MAX_MS 8000U
#define BOARD_WATCHDOG_COUNTS_MS 2000U

typedef enum
{
    BOARD_ECHO_IDLE,
    BOARD_ECHO_WAIT_RISE,
    BOARD_ECHO_WAIT_FALL,
    BOARD_ECHO_READY,
    BOARD_ECHO_TIMEOUT
} board_echo_state_t;

static bool                         gb_initialized;
static bool                         gb_accel_initialized;
static volatile int32_t             g_left_direction;
static volatile int32_t             g_right_direction;
static const char *                gp_init_diagnostic = "not initialized";
static uint16_t                     g_motor_level_limit;
static uint32_t                     g_watchdog_reload;
static volatile int32_t             g_left_delta;
static volatile int32_t             g_right_delta;
static volatile uint32_t            g_left_total;
static volatile uint32_t            g_right_total;
static volatile uint32_t            g_edge_errors;
static volatile uint32_t            g_left_phase;
static volatile uint32_t            g_right_phase;
static volatile board_echo_state_t  g_echo_state;
static volatile uint64_t            g_echo_rise_us;
static volatile uint32_t            g_echo_pulse_us;
static uint64_t                     g_trigger_time_us;
static bool                         gb_triggered_once;

static void board_gpio_isr (uint32_t interrupt_number);
static uint32_t board_wheel_edges (uint32_t pending, uint32_t pin_a,
                                   uint32_t pin_b);
static void board_init_motor (void);
static void board_init_inputs (void);
static bool board_init_accel (void);
static bool board_read_registers (uint8_t reg, uint8_t * p_data,
                                  size_t count);
static bool board_write_register (uint8_t reg, uint8_t value);
static void board_count_encoder (uint32_t pending, uint32_t current,
                                 uint32_t previous, int32_t direction,
                                 volatile int32_t * p_count,
                                 volatile uint32_t * p_total);
#if ROBOT_ENCODER_CHANNELS == 2
static void board_count_transition (uint32_t pending, uint32_t current,
                                    uint32_t previous,
                                    volatile int32_t * p_count,
                                    volatile uint32_t * p_total);
#endif
static void board_add_count (int32_t increment,
                             volatile int32_t * p_count,
                             volatile uint32_t * p_total);
static void board_apply_motor (uint32_t slice, int32_t direction,
                               uint16_t level,
                               volatile int32_t * p_direction);
static uint16_t board_duty_level (core_real_t duty);
static void board_record_error (void);

void board_boot_timebase (void)
{
    /* Same tick programming as SDK 2.2 hardware_ticks/ticks.c on RP2040.
     * This enables the tick generator, not the watchdog reset mechanism.
     */
    watchdog_hw->tick = 12U | WATCHDOG_TICK_ENABLE_BITS;
    unreset_block_num_wait_blocking(RESET_TIMER);
}

bool board_watchdog_start (uint32_t timeout_ms)
{
    uint32_t saved;
    bool b_started;

    if ((!gb_initialized) || (0U != g_watchdog_reload) || (0U == timeout_ms) ||
        (timeout_ms > BOARD_WATCHDOG_MAX_MS))
    {
        return false;
    }
    saved = app_main_lock();
    /* SDK 2.2 watchdog.c uses this domain selection and RP2040-E1 factor.
     * 8000 * 2000 = 16000000 fits the 24-bit LOAD field and uint32_t.
     */
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    hw_set_bits(&psm_hw->wdsel,
        PSM_WDSEL_BITS & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS));
    hw_clear_bits(&watchdog_hw->ctrl,
        WATCHDOG_CTRL_PAUSE_DBG0_BITS | WATCHDOG_CTRL_PAUSE_DBG1_BITS |
        WATCHDOG_CTRL_PAUSE_JTAG_BITS);
    /* Clear any boot-ROM address-redirect magic: restart normal flash. */
    watchdog_hw->scratch[4] = 0U;
    g_watchdog_reload = timeout_ms * BOARD_WATCHDOG_COUNTS_MS;
    watchdog_hw->load = g_watchdog_reload;
    hw_set_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    b_started = (0U != (watchdog_hw->ctrl & WATCHDOG_CTRL_ENABLE_BITS));
    if (!b_started)
    {
        g_watchdog_reload = 0U;
    }
    app_main_unlock(saved);
    return b_started;
}

void board_watchdog_feed (void)
{
    if (0U != g_watchdog_reload)
    {
        watchdog_hw->load = g_watchdog_reload;
    }
}

bool board_watchdog_rebooted (void)
{
    return (0U != (watchdog_hw->reason & WATCHDOG_REASON_TIMER_BITS));
}

void board_boot_brake (void)
{
    uint32_t saved = app_main_lock();

    board_init_motor();
    board_motor_brake();
    app_main_unlock(saved);
}

bool board_init (void)
{
    bool b_duty_valid = (isfinite(ROBOT_MAX_DUTY)) &&
                        (ROBOT_MAX_DUTY > 0.0) &&
                        (ROBOT_MAX_DUTY <= 1.0);

    if (b_duty_valid)
    {
        g_motor_level_limit = board_duty_level(1.0);
    }
    board_boot_brake();
    gp_init_diagnostic = "invalid maximum motor duty";
    if (!b_duty_valid)
    {
        return false;
    }
    gp_init_diagnostic = "ROBOT_WIRING_CONFIRMED=0; review wiring";
    if (0 == ROBOT_WIRING_CONFIRMED)
    {
        return false;
    }
#if ROBOT_ENCODER_CHANNELS == 1
    gp_init_diagnostic = "single-channel direction/counts not commissioned";
    if ((0 == ROBOT_SINGLE_CHANNEL_DIRECTION_CONFIRMED) ||
        (0U == ROBOT_ENCODER_COUNTS_PER_REV))
    {
        return false;
    }
#endif
    board_init_inputs();
    gp_init_diagnostic = "GPIO interrupt registration failed";
    if (!app_main_install_gpio_irq(board_gpio_isr))
    {
        return false;
    }
    io_bank0_hw->intr[BOARD_LEFT_BANK] = BOARD_LEFT_MASK;
    io_bank0_hw->intr[BOARD_RIGHT_BANK] = BOARD_RIGHT_MASK;
    io_bank0_hw->intr[BOARD_ECHO_BANK] = BOARD_ECHO_MASK;
    io_bank0_hw->proc0_irq_ctrl.inte[BOARD_LEFT_BANK] |= BOARD_LEFT_MASK;
    io_bank0_hw->proc0_irq_ctrl.inte[BOARD_RIGHT_BANK] |= BOARD_RIGHT_MASK;
    io_bank0_hw->proc0_irq_ctrl.inte[BOARD_ECHO_BANK] |= BOARD_ECHO_MASK;
    adc_init();
    adc_gpio_init(ROBOT_PIN_IR_LEFT);
    adc_gpio_init(ROBOT_PIN_IR_RIGHT);
    adc_gpio_init(ROBOT_PIN_IR_BARCODE);
    gb_accel_initialized = board_init_accel();
    gb_initialized = true;
    gp_init_diagnostic = gb_accel_initialized ? "ready" :
                         "accelerometer configuration/readback failed";
    return gb_accel_initialized;
}

const char * board_init_diagnostic (void)
{
    return gp_init_diagnostic;
}

uint64_t board_time_us (void)
{
    uint32_t high_before;
    uint32_t low_part;
    uint32_t high_after;

    do
    {
        high_before = timer_hw->timerawh;
        low_part = timer_hw->timerawl;
        high_after = timer_hw->timerawh;
    } while (high_before != high_after);
    /* Widening preserves all 32 bits before the upper-word shift. */
    return (((uint64_t)high_before) << 32U) | low_part;
}

void board_read_encoders (int32_t * p_left, int32_t * p_right,
                          uint32_t * p_errors)
{
    uint32_t saved;

    if ((NULL == p_left) || (NULL == p_right) || (NULL == p_errors))
    {
        return;
    }
    saved = app_main_lock();
    *p_left = g_left_delta;
    *p_right = g_right_delta;
    *p_errors = g_edge_errors;
    g_left_delta = 0;
    g_right_delta = 0;
    g_edge_errors = 0U;
    app_main_unlock(saved);
}

void board_encoder_totals (uint32_t * p_left, uint32_t * p_right)
{
    uint32_t saved;

    if ((NULL == p_left) || (NULL == p_right))
    {
        return;
    }
    saved = app_main_lock();
    *p_left = g_left_total;
    *p_right = g_right_total;
    app_main_unlock(saved);
}

bool board_read_ir (uint16_t samples[3])
{
    uint32_t channel;
    uint64_t deadline;

    if ((NULL == samples) || (!gb_initialized))
    {
        return false;
    }
    for (channel = 0U; channel < BOARD_ACCEL_AXES; channel++)
    {
        adc_select_input(channel);
        hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
        deadline = board_time_us() + BOARD_ADC_TIMEOUT_US;
        while (0U == (adc_hw->cs & ADC_CS_READY_BITS))
        {
            if (board_time_us() >= deadline)
            {
                return false;
            }
        }
        if (0U != (adc_hw->cs & ADC_CS_ERR_BITS))
        {
            return false;
        }
        /* ADC result is a 12-bit unsigned value, masked before narrowing. */
        samples[channel] = (uint16_t)(adc_hw->result & 0xFFFU);
    }
    return true;
}

board_accel_status_t board_read_accel (int16_t milligravity[3])
{
    uint8_t  status;
    uint8_t  bytes[BOARD_ACCEL_BYTES];
    uint32_t axis;
    uint32_t raw_sample;
    int32_t  signed_sample;
    bool     b_valid;
    board_accel_status_t sample;

    if ((NULL == milligravity) || (!gb_accel_initialized))
    {
        return BOARD_ACCEL_INVALID;
    }
    if (!board_read_registers(BOARD_ACCEL_STATUS, &status, 1U))
    {
        return BOARD_ACCEL_INVALID;
    }
    sample = board_accel_classify(status);
    if (BOARD_ACCEL_PENDING == sample)
    {
        return BOARD_ACCEL_PENDING;
    }
    /* An overrun (ZYXOR with ZYXDA) is FRESH: the burst returns the newest
     * sample; BDU keeps each axis's byte pair together (cross-axis coherence
     * and flag clearing: T-09). Read after every flagged status to release
     * the latch.
     */
    b_valid = (BOARD_ACCEL_FRESH == sample);
    if (!board_read_registers(BOARD_ACCEL_DATA, bytes, BOARD_ACCEL_BYTES))
    {
        return BOARD_ACCEL_INVALID;
    }
    for (axis = 0U; axis < BOARD_ACCEL_AXES; axis++)
    {
        /* Both byte values widen exactly into a 16-bit packed sample. */
        raw_sample = ((uint32_t)bytes[2U * axis + 1U]) << 8U;
        raw_sample |= bytes[2U * axis];
        raw_sample >>= 4U;
        /* raw_sample <= 4095; subtraction produces -2048..2047 exactly. */
        signed_sample = (int32_t)raw_sample;
        if (raw_sample >= 2048U)
        {
            signed_sample -= 4096;
        }
        if ((signed_sample <= -2048) || (signed_sample >= 2047))
        {
            b_valid = false;
        }
        /* Signed 12-bit range fits int16_t; HR +/-2g gives 1 mg/LSB. */
        milligravity[axis] = (int16_t)signed_sample;
    }
    return b_valid ? BOARD_ACCEL_FRESH : BOARD_ACCEL_INVALID;
}

bool board_ultrasonic_trigger (void)
{
    uint64_t current_us = board_time_us();
    uint32_t saved;

    if ((!gb_initialized) ||
        (gb_triggered_once &&
         ((current_us - g_trigger_time_us) < ROBOT_TRIGGER_INTERVAL_US)))
    {
        return false;
    }
    saved = app_main_lock();
    if (BOARD_ECHO_IDLE != g_echo_state)
    {
        app_main_unlock(saved);
        return false;
    }
    if (gpio_get(ROBOT_PIN_ECHO))
    {
        g_echo_state = BOARD_ECHO_TIMEOUT;
        app_main_unlock(saved);
        return false;
    }
    g_echo_state = BOARD_ECHO_WAIT_RISE;
    g_trigger_time_us = current_us;
    gb_triggered_once = true;
    app_main_unlock(saved);
    gpio_put(ROBOT_PIN_TRIGGER, true);
    current_us = board_time_us();
    /* No RTOS delay: wait only for pulse minimum, with IRQs still enabled.
     * The start read may fall late in its 1 us tick, so wait one extra tick:
     * the pulse is then at least 10 us (HC-SR04). Preemption can extend it;
     * measure the width on the board.
     */
    while ((board_time_us() - current_us) <= ROBOT_TRIGGER_PULSE_US)
    {
        tight_loop_contents();
    }
    gpio_put(ROBOT_PIN_TRIGGER, false);
    return true;
}

void board_ultrasonic_service (void)
{
    uint64_t current_us = board_time_us();
    uint32_t saved = app_main_lock();

    if (((BOARD_ECHO_WAIT_RISE == g_echo_state) ||
         (BOARD_ECHO_WAIT_FALL == g_echo_state)) &&
        ((current_us - g_trigger_time_us) >= ROBOT_ECHO_TIMEOUT_US))
    {
        g_echo_state = BOARD_ECHO_TIMEOUT;
    }
    app_main_unlock(saved);
}

bool board_ultrasonic_result (uint32_t * p_pulse_us, bool * p_valid)
{
    uint32_t saved;

    if ((NULL == p_pulse_us) || (NULL == p_valid))
    {
        return false;
    }
    saved = app_main_lock();
    if ((BOARD_ECHO_READY != g_echo_state) &&
        (BOARD_ECHO_TIMEOUT != g_echo_state))
    {
        app_main_unlock(saved);
        return false;
    }
    *p_valid = ((BOARD_ECHO_READY == g_echo_state) &&
                (g_echo_pulse_us >= BOARD_ECHO_MIN_US) &&
                (g_echo_pulse_us <= BOARD_ECHO_MAX_US));
    *p_pulse_us = *p_valid ? g_echo_pulse_us : 0U;
    g_echo_state = BOARD_ECHO_IDLE;
    app_main_unlock(saved);
    return true;
}

bool board_servo_us (uint16_t pulse_us)
{
    if ((!gb_initialized) || (0 == ROBOT_SERVO_CONFIRMED) ||
        (pulse_us < ROBOT_SERVO_MIN_US) ||
        (pulse_us > ROBOT_SERVO_MAX_US))
    {
        return false;
    }
    pwm_set_chan_level(BOARD_SERVO_SLICE, PWM_CHAN_A, pulse_us);
    return true;
}

void board_motor_duty (core_real_t left_duty, core_real_t right_duty)
{
    board_motor_command_t command = {0};

    if (!board_motor_prepare(left_duty, right_duty, &command))
    {
        board_motor_brake();
        return;
    }
    board_motor_commit(&command);
}

bool board_motor_prepare (core_real_t left_duty, core_real_t right_duty,
                          board_motor_command_t * p_command)
{
    if (NULL == p_command)
    {
        return false;
    }
    *p_command = (board_motor_command_t){0};

    if ((!gb_initialized) || (0 == ROBOT_CALIBRATION_CONFIRMED) ||
        (0 == ROBOT_WIRING_CONFIRMED) ||
        (!isfinite(left_duty)) || (!isfinite(right_duty)) ||
        (fabs(left_duty) > 1.0) || (fabs(right_duty) > 1.0))
    {
        return false;
    }
    /* Soft floating-point work stays outside the short GPIO critical region. */
    left_duty *= ROBOT_LEFT_MOTOR_SIGN;
    right_duty *= ROBOT_RIGHT_MOTOR_SIGN;
    p_command->left_level = board_duty_level(left_duty);
    p_command->right_level = board_duty_level(right_duty);
    p_command->left_direction = (left_duty > 0.0) ? 1 : -1;
    p_command->right_direction = (right_duty > 0.0) ? 1 : -1;
    if (0U == p_command->left_level)
    {
        p_command->left_direction = 0;
    }
    if (0U == p_command->right_level)
    {
        p_command->right_direction = 0;
    }
    p_command->b_valid = true;
    return true;
}

void board_motor_commit (const board_motor_command_t * p_command)
{
    uint32_t saved = app_main_lock();

    if ((NULL == p_command) || (!p_command->b_valid) ||
        (!gb_initialized) || (0 == ROBOT_CALIBRATION_CONFIRMED) ||
        (0 == ROBOT_WIRING_CONFIRMED) || board_stop_pressed())
    {
        board_motor_brake();
        app_main_unlock(saved);
        return;
    }
    if ((p_command->left_level > g_motor_level_limit) ||
        (p_command->right_level > g_motor_level_limit) ||
        (p_command->left_direction < -1) ||
        (p_command->left_direction > 1) ||
        (p_command->right_direction < -1) ||
        (p_command->right_direction > 1))
    {
        board_motor_brake();
        app_main_unlock(saved);
        return;
    }
    board_apply_motor(BOARD_MOTOR_SLICE_LEFT, p_command->left_direction,
                      p_command->left_level, &g_left_direction);
    board_apply_motor(BOARD_MOTOR_SLICE_RIGHT, p_command->right_direction,
                      p_command->right_level, &g_right_direction);
    app_main_unlock(saved);
}

void board_motor_brake (void)
{
    uint32_t saved = app_main_lock();

    pwm_set_both_levels(BOARD_MOTOR_SLICE_LEFT, 0U, 0U);
    pwm_set_both_levels(BOARD_MOTOR_SLICE_RIGHT, 0U, 0U);
    g_left_direction = 0;
    g_right_direction = 0;
    app_main_unlock(saved);
}

bool board_stop_pressed (void)
{
    return !gpio_get(ROBOT_PIN_STOP);
}

bool board_start_pressed (void)
{
    return !gpio_get(ROBOT_PIN_START);
}

#if !TM_WIFI_CYW43
/* D-ADAPTER: names below are required Pico SDK ABI hooks, not robot APIs.
 * The Wi-Fi profile already supplies these in cyw43_utk_compat.c.
 */
/** Return frequencies configured by the pinned BSP, without changing them. */
uint32_t clock_get_hz (clock_handle_t clock)
{
    if ((clk_sys == clock) || (clk_peri == clock))
    {
        return ROBOT_CLOCK_HZ;
    }
    if ((clk_adc == clock) || (clk_usb == clock))
    {
        return 48000000U;
    }
    return 0U;
}

/** Supply the SDK's timestamp hook using the coherent free-running timer. */
uint64_t time_us_64 (void)
{
    return board_time_us();
}
#endif

/** Initialise four motor PWM outputs low before selecting PWM function. */
static void board_init_motor (void)
{
    uint32_t pin;
    pwm_config config = pwm_get_default_config();

    /* USE_PTMR=0 is enforced by installer; the kernel uses SysTick. */
    unreset_block_num_wait_blocking(RESET_PWM);
    pwm_config_set_clkdiv_int(&config, 1U);
    pwm_config_set_wrap(&config, ROBOT_PWM_WRAP);
    pwm_init(BOARD_MOTOR_SLICE_LEFT, &config, false);
    pwm_init(BOARD_MOTOR_SLICE_RIGHT, &config, false);
    for (pin = ROBOT_PIN_MOTOR_LEFT_A;
         pin <= ROBOT_PIN_MOTOR_RIGHT_B; pin++)
    {
        gpio_init(pin);
        gpio_put(pin, false);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_set_function(pin, GPIO_FUNC_PWM);
    }
    pwm_set_both_levels(BOARD_MOTOR_SLICE_LEFT, 0U, 0U);
    pwm_set_both_levels(BOARD_MOTOR_SLICE_RIGHT, 0U, 0U);
    pwm_set_enabled(BOARD_MOTOR_SLICE_LEFT, true);
    pwm_set_enabled(BOARD_MOTOR_SLICE_RIGHT, true);
}

/** Initialise passive inputs, trigger low and a disabled-pulse servo. */
static void board_init_inputs (void)
{
    static const uint32_t encoder_pins[4] =
    {
        ROBOT_PIN_ENCODER_LEFT_A, ROBOT_PIN_ENCODER_LEFT_B,
        ROBOT_PIN_ENCODER_RIGHT_A, ROBOT_PIN_ENCODER_RIGHT_B
    };
    uint32_t index;
    uint32_t input_levels;
    pwm_config config = pwm_get_default_config();

    for (index = 0U; index < 4U; index++)
    {
        gpio_init(encoder_pins[index]);
        gpio_set_dir(encoder_pins[index], GPIO_IN);
        gpio_pull_up(encoder_pins[index]);
    }
    input_levels = gpio_get_all();
    g_left_phase =
        ((input_levels >> ROBOT_PIN_ENCODER_LEFT_A) & 1U) << 1U;
    g_left_phase |= (input_levels >> ROBOT_PIN_ENCODER_LEFT_B) & 1U;
    g_right_phase =
        ((input_levels >> ROBOT_PIN_ENCODER_RIGHT_A) & 1U) << 1U;
    g_right_phase |= (input_levels >> ROBOT_PIN_ENCODER_RIGHT_B) & 1U;
    gpio_init(ROBOT_PIN_STOP);
    gpio_pull_up(ROBOT_PIN_STOP);
    gpio_init(ROBOT_PIN_START);
    gpio_pull_up(ROBOT_PIN_START);
    gpio_init(ROBOT_PIN_ECHO);
    gpio_pull_down(ROBOT_PIN_ECHO);
    gpio_init(ROBOT_PIN_TRIGGER);
    gpio_put(ROBOT_PIN_TRIGGER, false);
    gpio_set_dir(ROBOT_PIN_TRIGGER, GPIO_OUT);
    gpio_init(ROBOT_PIN_SERVO);
    gpio_put(ROBOT_PIN_SERVO, false);
    gpio_set_dir(ROBOT_PIN_SERVO, GPIO_OUT);
    if (0 != ROBOT_SERVO_CONFIRMED)
    {
        pwm_config_set_clkdiv_int(&config, 125U);
        pwm_config_set_wrap(&config, ROBOT_SERVO_FRAME_US - 1U);
        pwm_init(BOARD_SERVO_SLICE, &config, false);
        pwm_set_chan_level(BOARD_SERVO_SLICE, PWM_CHAN_A, 0U);
        gpio_set_function(ROBOT_PIN_SERVO, GPIO_FUNC_PWM);
        pwm_set_enabled(BOARD_SERVO_SLICE, true);
    }
}

/** Configure only LSM303DLHC acceleration: no nonexistent gyro is assumed. */
static bool board_init_accel (void)
{
    uint8_t control_one;
    uint8_t control_four;
    uint32_t actual_hz;

    gpio_set_function(ROBOT_PIN_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(ROBOT_PIN_IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(ROBOT_PIN_IMU_SDA);
    gpio_pull_up(ROBOT_PIN_IMU_SCL);
    actual_hz = i2c_init(i2c1, ROBOT_I2C_HZ);
    if ((0U == actual_hz) || (actual_hz > 400000U))
    {
        return false;
    }
    if ((!board_write_register(BOARD_ACCEL_CTRL1, BOARD_ACCEL_ODR_ENABLE)) ||
        (!board_write_register(BOARD_ACCEL_CTRL4, BOARD_ACCEL_BDU_HR_2G)))
    {
        return false;
    }
    if ((!board_read_registers(BOARD_ACCEL_CTRL1, &control_one, 1U)) ||
        (!board_read_registers(BOARD_ACCEL_CTRL4, &control_four, 1U)))
    {
        return false;
    }
    return ((BOARD_ACCEL_ODR_ENABLE == control_one) &&
            (BOARD_ACCEL_BDU_HR_2G == control_four));
}

/** Read a small burst with the SDK's whole-transfer timeout per phase. */
static bool board_read_registers (uint8_t reg, uint8_t * p_data,
                                  size_t count)
{
    int written;
    int received;

    if ((NULL == p_data) || (0U == count) || (count > BOARD_ACCEL_BYTES))
    {
        return false;
    }
    written = i2c_write_timeout_us(i2c1, BOARD_ACCEL_ADDRESS, &reg, 1U,
                                  true, ROBOT_I2C_TIMEOUT_US);
    if (1 != written)
    {
        return false;
    }
    received = i2c_read_timeout_us(i2c1, BOARD_ACCEL_ADDRESS, p_data, count,
                                  false, ROBOT_I2C_TIMEOUT_US);
    /* count is checked <= 6, so int conversion is exact. */
    return ((int)count == received);
}

/** Write one LSM303 register, accepting only the complete two-byte result. */
static bool board_write_register (uint8_t reg, uint8_t value)
{
    uint8_t payload[2] = {reg, value};
    int result = i2c_write_timeout_us(i2c1, BOARD_ACCEL_ADDRESS, payload,
                                      sizeof(payload), false,
                                      ROBOT_I2C_TIMEOUT_US);

    return (2 == result);
}

/** GPIO ISR: acknowledge owned edges only; do no kernel waits or I/O. */
static void board_gpio_isr (uint32_t interrupt_number)
{
    uint32_t left_pending;
    uint32_t right_pending;
    uint32_t echo_pending;
    uint32_t echo_edges;
    uint32_t current;
    uint32_t input_levels;
    uint64_t current_us;
    uint64_t elapsed_us;

    /* D-ADAPTER: kernel passes IRQ number; this handler is GPIO-only. */
    (void)interrupt_number;
    left_pending = io_bank0_hw->proc0_irq_ctrl.ints[BOARD_LEFT_BANK] &
                   BOARD_LEFT_MASK;
    right_pending = io_bank0_hw->proc0_irq_ctrl.ints[BOARD_RIGHT_BANK] &
                    BOARD_RIGHT_MASK;
    echo_pending = io_bank0_hw->proc0_irq_ctrl.ints[BOARD_ECHO_BANK] &
                   BOARD_ECHO_MASK;
    io_bank0_hw->intr[BOARD_LEFT_BANK] = left_pending;
    io_bank0_hw->intr[BOARD_RIGHT_BANK] = right_pending;
    io_bank0_hw->intr[BOARD_ECHO_BANK] = echo_pending;
    /* One SIO read keeps all four encoder levels in one coherent sample. */
    input_levels = gpio_get_all();
    current_us = board_time_us();
    if (0U != left_pending)
    {
        current =
            ((input_levels >> ROBOT_PIN_ENCODER_LEFT_A) & 1U) << 1U;
        current |= (input_levels >> ROBOT_PIN_ENCODER_LEFT_B) & 1U;
        board_count_encoder(board_wheel_edges(left_pending,
                                              ROBOT_PIN_ENCODER_LEFT_A,
                                              ROBOT_PIN_ENCODER_LEFT_B),
                            current, g_left_phase, g_left_direction *
                            ROBOT_LEFT_MOTOR_SIGN * ROBOT_LEFT_ENCODER_SIGN,
                            &g_left_delta, &g_left_total);
        g_left_phase = current;
    }
    if (0U != right_pending)
    {
        current =
            ((input_levels >> ROBOT_PIN_ENCODER_RIGHT_A) & 1U) << 1U;
        current |= (input_levels >> ROBOT_PIN_ENCODER_RIGHT_B) & 1U;
        board_count_encoder(board_wheel_edges(right_pending,
                                              ROBOT_PIN_ENCODER_RIGHT_A,
                                              ROBOT_PIN_ENCODER_RIGHT_B),
                            current, g_right_phase, g_right_direction *
                            ROBOT_RIGHT_MOTOR_SIGN * ROBOT_RIGHT_ENCODER_SIGN,
                            &g_right_delta, &g_right_total);
        g_right_phase = current;
    }
    if (0U != echo_pending)
    {
        echo_edges = (echo_pending >> BOARD_IRQ_SHIFT(ROBOT_PIN_ECHO)) & 0xCU;
        if ((BOARD_ECHO_WAIT_RISE == g_echo_state) &&
            ((uint32_t)GPIO_IRQ_EDGE_RISE == echo_edges) &&
            (0U != (input_levels & (1U << ROBOT_PIN_ECHO))))
        {
            g_echo_rise_us = current_us;
            g_echo_state = BOARD_ECHO_WAIT_FALL;
        }
        else if ((BOARD_ECHO_WAIT_FALL == g_echo_state) &&
                 ((uint32_t)GPIO_IRQ_EDGE_FALL == echo_edges) &&
                 (0U == (input_levels & (1U << ROBOT_PIN_ECHO))))
        {
            elapsed_us = current_us - g_echo_rise_us;
            if (elapsed_us <= ROBOT_ECHO_TIMEOUT_US)
            {
                /* Explicit timeout limits elapsed_us to 30000. */
                g_echo_pulse_us = (uint32_t)elapsed_us;
                g_echo_state = BOARD_ECHO_READY;
            }
            else
            {
                g_echo_state = BOARD_ECHO_TIMEOUT;
            }
        }
        else
        {
            /* Both edges coalesced, or unexpected order: no valid timing. */
            if (BOARD_ECHO_IDLE != g_echo_state)
            {
                g_echo_state = BOARD_ECHO_TIMEOUT;
            }
        }
    }
}

/** Normalise one wheel's latched edge bits: A in bits 2..3, B in 6..7. */
static uint32_t board_wheel_edges (uint32_t pending, uint32_t pin_a,
                                   uint32_t pin_b)
{
    return ((pending >> BOARD_IRQ_SHIFT(pin_a)) & 0xCU) |
           (((pending >> BOARD_IRQ_SHIFT(pin_b)) & 0xCU) << 4U);
}

/** Select the explicitly configured acquisition model without guessing B. */
static void board_count_encoder (uint32_t pending, uint32_t current,
                                 uint32_t previous, int32_t direction,
                                 volatile int32_t * p_count,
                                 volatile uint32_t * p_total)
{
#if ROBOT_ENCODER_CHANNELS == 2
    (void)direction;
    board_count_transition(pending, current, previous, p_count, p_total);
#else
    uint32_t expected = (0U != (current & 2U)) ? GPIO_IRQ_EDGE_RISE :
                                               GPIO_IRQ_EDGE_FALL;

    /* Both A edges count. B is not a sensor in this mode. Coalesced edges,
     * inconsistent samples and unknown sign cannot yield safe displacement.
     * Raw sign matches the quadrature contract; app applies encoder polarity.
     */
    if ((0 == ROBOT_SINGLE_CHANNEL_DIRECTION_CONFIRMED) ||
        ((1 != direction) && (-1 != direction)) ||
        (0U == ((current ^ previous) & 2U)) || (pending != expected))
    {
        board_record_error();
        return;
    }
    board_add_count(direction, p_count, p_total);
#endif
}

#if ROBOT_ENCODER_CHANNELS == 2
/** Reject ambiguous latched edges before interpreting the sampled phase. */
static void board_count_transition (uint32_t pending, uint32_t current,
                                    uint32_t previous,
                                    volatile int32_t * p_count,
                                    volatile uint32_t * p_total)
{
    static const int32_t increments[16] =
    {
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0
    };
    int32_t increment = increments[(previous << 2U) | current];
    uint32_t changed = current ^ previous;
    uint32_t expected;

    /* Pending is normalized to A's GPIO nibble then B's nibble. Only one
     * edge consistent with the sampled change can represent an x4 count.
     * Several bits lose ordering; a single inconsistent bit can arise when
     * another edge races the status acknowledgement and SIO sample. Never
     * invent displacement; caller resynchronizes the observed phase.
     */
    if (2U == changed)
    {
        expected = (0U != (current & 2U)) ? GPIO_IRQ_EDGE_RISE :
                                           GPIO_IRQ_EDGE_FALL;
    }
    else
    {
        expected = ((0U != (current & 1U)) ? GPIO_IRQ_EDGE_RISE :
                                            GPIO_IRQ_EDGE_FALL) << 4U;
    }
    if (((1U != changed) && (2U != changed)) || (pending != expected))
    {
        board_record_error();
        return;
    }
    board_add_count(increment, p_count, p_total);
}
#endif

/** Share saturation and modulo-total semantics across acquisition modes. */
static void board_add_count (int32_t increment,
                             volatile int32_t * p_count,
                             volatile uint32_t * p_total)
{
    if (increment > 0)
    {
        /* Unsigned overflow deliberately preserves modulo-2^32 position. */
        (*p_total)++;
    }
    else if (increment < 0)
    {
        /* Unsigned underflow deliberately preserves reverse displacement. */
        (*p_total)--;
    }
    else
    {
        /* A repeated phase does not change either position accumulator. */
    }
    if (((increment > 0) && (INT32_MAX == *p_count)) ||
        ((increment < 0) && (INT32_MIN == *p_count)))
    {
        board_record_error();
        return;
    }
    *p_count += increment;
}

/** Insert one zero-demand cycle before changing polarity; caller masks IRQ. */
static void board_apply_motor (uint32_t slice, int32_t direction,
                               uint16_t level,
                               volatile int32_t * p_direction)
{
    if ((0 != *p_direction) &&
        (direction != *p_direction))
    {
        pwm_set_both_levels(slice, 0U, 0U);
        *p_direction = 0;
        return;
    }
    if (direction > 0)
    {
        pwm_set_both_levels(slice, level, 0U);
    }
    else if (direction < 0)
    {
        pwm_set_both_levels(slice, 0U, level);
    }
    else
    {
        pwm_set_both_levels(slice, 0U, 0U);
    }
    *p_direction = direction;
}

/** Map validated duty to its capped compare level outside the IRQ mask. */
static uint16_t board_duty_level (core_real_t duty)
{
    core_real_t magnitude = fabs(duty);

    if (magnitude > ROBOT_MAX_DUTY)
    {
        magnitude = ROBOT_MAX_DUTY;
    }
    /* Validated duty <= 1 and wrap 6249 bound level to 0..6250. */
    return (uint16_t)(magnitude * (ROBOT_PWM_WRAP + 1U));
}

/** Saturate the diagnostic counter; it is consumed with encoder deltas. */
static void board_record_error (void)
{
    if (UINT32_MAX != g_edge_errors)
    {
        g_edge_errors++;
    }
}

/* End of file. */

