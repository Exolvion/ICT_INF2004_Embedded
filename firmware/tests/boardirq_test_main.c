/**
 * @file boardirq_test_main.c
 * @brief Host-only ISR, packed-word and accelerometer checks with real SDK
 * types.
 * @details MMIO/I2C/PWM are doubles; no timing, ARM or physical sensor is
 * tested. Including the implementation exercises its private registered ISR
 * and packed ISR words without adding a production test hook. Unused board
 * functions are link-discarded. The lock double models PRIMASK: an edge
 * raised while app_main_lock() masks interrupts stays latched, and the ISR
 * runs only when app_main_unlock() clears the mask, as on the Cortex-M0+.
 */
#include "boardirq_test_main.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

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

static io_bank0_hw_t * gp_test_io;
static timer_hw_t * gp_test_timer;
static uint32_t g_test_levels;
static uint32_t g_test_checks;
static uint32_t g_test_failures;
static uint8_t g_test_accel_status;
static uint8_t g_test_accel_register;
static bool g_test_i2c_error;
static bool g_test_accel_saturated;
static uint32_t g_test_accel_bursts;
/* PRIMASK double: mask, outermost regions opened, ISR entries while masked
 * (must stay 0) and one encoder edge batch that arrives inside a region.
 */
static uint32_t g_test_primask;
static uint32_t g_test_regions;
static uint32_t g_test_masked_isr;
static bool gb_test_arm;
static bool gb_test_latched;
static uint32_t g_test_armed_pending;
static uint32_t g_test_armed_left;
static uint32_t g_test_armed_right;
/* Modelled encoder input phases (A << 1 | B) and motor PWM compare levels
 * of slices 4 and 5 (channel A, channel B).
 */
static uint32_t g_test_phase[2];
static uint32_t g_test_pwm[2][2];

static int boardirq_i2c_write (i2c_inst_t * p_i2c, uint8_t address,
    const uint8_t * p_source, size_t length, bool b_nostop,
    uint timeout_us);
static int boardirq_i2c_read (i2c_inst_t * p_i2c, uint8_t address,
    uint8_t * p_destination, size_t length, bool b_nostop,
    uint timeout_us);
static void boardirq_gpio_put (uint32_t pin, bool b_value);
static void boardirq_timer_step (void);
static void boardirq_pwm_levels (uint32_t slice, uint32_t level_a,
                                 uint32_t level_b);

/* One microsecond per busy-wait iteration; the last count ends any loop. */
#define BOARDIRQ_TICK(count) {.timerawl = (count), .timerawh = 0U}
static timer_hw_t g_test_ticks[] =
{
    BOARDIRQ_TICK(5000U), BOARDIRQ_TICK(5001U), BOARDIRQ_TICK(5002U),
    BOARDIRQ_TICK(5003U), BOARDIRQ_TICK(5004U), BOARDIRQ_TICK(5005U),
    BOARDIRQ_TICK(5006U), BOARDIRQ_TICK(5007U), BOARDIRQ_TICK(5008U),
    BOARDIRQ_TICK(5009U), BOARDIRQ_TICK(5010U), BOARDIRQ_TICK(5011U),
    BOARDIRQ_TICK(5012U), BOARDIRQ_TICK(5013U), BOARDIRQ_TICK(0x7FFFFFFFU)
};
static uint32_t g_test_tick;
static uint32_t g_test_rise_us;
static uint32_t g_test_fall_us;

#undef io_bank0_hw
#define io_bank0_hw gp_test_io
#undef timer_hw
#define timer_hw gp_test_timer
#define gpio_get_all() g_test_levels
#define gpio_get(pin) (0U != (g_test_levels & (1U << (pin))))
#define gpio_put(pin, b_value) boardirq_gpio_put((pin), (b_value))
#define tight_loop_contents() boardirq_timer_step()
#define i2c_write_timeout_us boardirq_i2c_write
#define i2c_read_timeout_us boardirq_i2c_read
#define pwm_set_both_levels boardirq_pwm_levels

#include "board.c"

/* Interleaving stress: steps, and the unread count that forces a read so
 * the model never exceeds the 6-bit delta range.
 */
#define BOARDIRQ_STRESS_STEPS 10000U
#define BOARDIRQ_STRESS_LIMIT 30

static void boardirq_check (bool b_pass, const char * p_name);
static void boardirq_reset (void);
static void boardirq_accel (void);
static uint32_t boardirq_bank_bits (uint32_t nibbles, uint32_t pin_a,
                                    uint32_t pin_b);
static void boardirq_inject (uint32_t pending, uint32_t left_phase,
                             uint32_t right_phase);
static void boardirq_arm (uint32_t pending, uint32_t left_phase,
                          uint32_t right_phase);
static int32_t boardirq_delta (uint32_t wheel);
static uint32_t boardirq_read_errors (void);
static void boardirq_inject_echo (uint32_t edge_bits, bool b_level_high,
                                  uint32_t time_us);
static uint32_t boardirq_pulse_field (void);
static void boardirq_echo_pulse (uint32_t rise_us, uint32_t fall_us);
static void boardirq_echo_case (uint32_t width_us, bool b_expect_valid,
                                const char * p_name);
static void boardirq_echo (void);
static void boardirq_echo_coherence (void);
static void boardirq_echo_boundaries (void);
static void boardirq_echo_service (void);
static void boardirq_echo_order (void);
static void boardirq_trigger (void);
static bool boardirq_trigger_refused (uint32_t word);
static void boardirq_trigger_pending (void);
static void boardirq_motor_codes (void);
static void boardirq_motor_setup (void);
static bool boardirq_braked (void);
static void boardirq_motor_command (void);
#if (0 != ROBOT_CALIBRATION_CONFIRMED) && (0 != ROBOT_WIRING_CONFIRMED)
static void boardirq_motor_prepare (void);
static void boardirq_motor_commit (void);
#endif
#if ROBOT_ENCODER_CHANNELS == 2
static uint32_t boardirq_transition (uint32_t wheel, int32_t direction);
static void boardirq_step (uint32_t wheel, int32_t direction);
static void boardirq_normal_cycles (void);
static void boardirq_coalesced (void);
static void boardirq_inconsistent_sample (void);
static void boardirq_resynchronization (void);
static void boardirq_sticky_errors (void);
static void boardirq_constant_b_quadrature (void);
static void boardirq_delta_saturation (void);
static void boardirq_read_keeps_phase (void);
static void boardirq_totals_wrap (void);
static void boardirq_boot_discard (void);
static void boardirq_interleave (void);
static void boardirq_stress (void);
static void boardirq_shared_bank (void);
#else
static void boardirq_single_channel (void);
static void boardirq_single_discard (void);
#endif

int main (void)
{
    boardirq_accel();
    boardirq_echo();
    boardirq_echo_coherence();
    boardirq_echo_boundaries();
    boardirq_echo_service();
    boardirq_echo_order();
    boardirq_trigger();
    boardirq_trigger_pending();
    boardirq_motor_codes();
    boardirq_motor_command();
#if ROBOT_ENCODER_CHANNELS == 2
    boardirq_normal_cycles();
    boardirq_coalesced();
    boardirq_inconsistent_sample();
    boardirq_resynchronization();
    boardirq_sticky_errors();
    boardirq_constant_b_quadrature();
    boardirq_delta_saturation();
    boardirq_read_keeps_phase();
    boardirq_totals_wrap();
    boardirq_boot_discard();
    boardirq_interleave();
    boardirq_stress();
    boardirq_shared_bank();
#else
    boardirq_single_channel();
    boardirq_single_discard();
#endif
    boardirq_check((0U == g_test_primask) && (0U == g_test_masked_isr) &&
                   (!gb_test_latched),
                   "every lock region closed; no ISR ran while masked");
    printf("board ISR/accel host doubles: %u checks, %u failures\n",
           (unsigned int)g_test_checks, (unsigned int)g_test_failures);
    return (0U == g_test_failures) ? 0 : 1;
}

/** @brief PRIMASK double: mask, return the previous mask and count each
 * outermost region. An armed edge arrives as a region opens: the NVIC
 * latches it, so the ISR cannot run until the mask clears.
 */
uint32_t app_main_lock (void)
{
    uint32_t saved = g_test_primask;

    g_test_primask = 1U;
    if (0U == saved)
    {
        ++g_test_regions;
    }
    if (gb_test_arm)
    {
        gb_test_arm = false;
        gb_test_latched = true;
    }
    return saved;
}

/** @brief Restore the mask; a latched edge is taken as soon as it clears. */
void app_main_unlock (uint32_t saved_mask)
{
    g_test_primask = saved_mask;
    if ((0U == g_test_primask) && gb_test_latched)
    {
        gb_test_latched = false;
        boardirq_inject(g_test_armed_pending, g_test_armed_left,
                        g_test_armed_right);
    }
}

/** @brief Intercept actual board register addressing without sensor I/O. */
static int boardirq_i2c_write (i2c_inst_t * p_i2c, uint8_t address,
    const uint8_t * p_source, size_t length, bool b_nostop,
    uint timeout_us)
{
    (void)p_i2c;
    (void)address;
    (void)b_nostop;
    (void)timeout_us;
    if (g_test_i2c_error || (NULL == p_source) || (1U != length))
    {
        return -1;
    }
    g_test_accel_register = p_source[0];
    return 1;
}

/** @brief Return synthetic readiness and high-resolution XYZ register data. */
static int boardirq_i2c_read (i2c_inst_t * p_i2c, uint8_t address,
    uint8_t * p_destination, size_t length, bool b_nostop,
    uint timeout_us)
{
    size_t index;

    (void)p_i2c;
    (void)address;
    (void)b_nostop;
    (void)timeout_us;
    if ((1U == length) && (BOARD_ACCEL_STATUS == g_test_accel_register))
    {
        p_destination[0] = g_test_accel_status;
        return 1;
    }
    if ((BOARD_ACCEL_BYTES != length) ||
        (BOARD_ACCEL_DATA != g_test_accel_register))
    {
        return -1;
    }
    for (index = 0U; index < length; ++index)
    {
        p_destination[index] = 0U;
    }
    /* Z = 1000 mg; optional X = positive 12-bit saturation (2047 mg). */
    p_destination[4] = 0x80U;
    p_destination[5] = 0x3EU;
    if (g_test_accel_saturated)
    {
        p_destination[0] = 0xF0U;
        p_destination[1] = 0x7FU;
    }
    ++g_test_accel_bursts;
    return BOARD_ACCEL_BYTES;
}

/** @brief Exercise actual board status semantics with untouched output age. */
static void boardirq_accel (void)
{
    int16_t values[3] = {111, 222, 333};

    gb_accel_initialized = false;
    boardirq_check(BOARD_ACCEL_INVALID == board_read_accel(values),
                   "uninitialized accelerometer is invalid");
    gb_accel_initialized = true;
    boardirq_check(BOARD_ACCEL_INVALID == board_read_accel(NULL),
                   "null sample destination is invalid");
    g_test_accel_status = 0U;
    boardirq_check(BOARD_ACCEL_PENDING == board_read_accel(values),
                   "no DRDY is pending, not corrupted data");
    boardirq_check((111 == values[0]) && (222 == values[1]) &&
                   (333 == values[2]) && (0U == g_test_accel_bursts),
                   "pending cannot consume or manufacture a sample");
    g_test_accel_status = 0x08U;
    boardirq_check(BOARD_ACCEL_FRESH == board_read_accel(values),
                   "DRDY without overrun is fresh");
    boardirq_check((0 == values[0]) && (0 == values[1]) &&
                   (1000 == values[2]), "signed HR sample converts to mg");
    g_test_accel_status = 0x88U;
    values[0] = 111;
    values[2] = 333;
    boardirq_check(BOARD_ACCEL_FRESH == board_read_accel(values),
                   "overrun with DRDY is the newest sample");
    boardirq_check(2U == g_test_accel_bursts,
                   "overrun drains the BDU latch");
    boardirq_check((0 == values[0]) && (1000 == values[2]),
                   "overrun returns the burst-read sample");
    g_test_accel_status = 0x80U;
    boardirq_check(BOARD_ACCEL_INVALID == board_read_accel(values),
                   "overrun without DRDY is assumed inconsistent (T-09)");
    boardirq_check(3U == g_test_accel_bursts,
                   "overrun without DRDY also drains the BDU latch");
    boardirq_check((BOARD_ACCEL_PENDING == board_accel_classify(0x77U)) &&
                   (BOARD_ACCEL_FRESH == board_accel_classify(0xFFU)),
                   "only ZYXDA and ZYXOR select the sample status");
    g_test_accel_status = 0x08U;
    g_test_accel_saturated = true;
    boardirq_check(BOARD_ACCEL_INVALID == board_read_accel(values),
                   "saturation remains invalid");
    g_test_accel_saturated = false;
    g_test_i2c_error = true;
    boardirq_check(BOARD_ACCEL_INVALID == board_read_accel(values),
                   "I2C transaction failure remains invalid");
    g_test_i2c_error = false;
}

/** @brief Record TRIG edges on the fake microsecond timer; ignore others. */
static void boardirq_gpio_put (uint32_t pin, bool b_value)
{
    if ((ROBOT_PIN_TRIGGER == pin) && (NULL != gp_test_timer))
    {
        if (b_value)
        {
            g_test_rise_us = gp_test_timer->timerawl;
        }
        else
        {
            g_test_fall_us = gp_test_timer->timerawl;
        }
    }
}

/** @brief Each busy-wait iteration advances the fake timer by one count. */
static void boardirq_timer_step (void)
{
    if ((g_test_tick + 1U) < (sizeof(g_test_ticks) / sizeof(g_test_ticks[0])))
    {
        ++g_test_tick;
    }
    gp_test_timer = &g_test_ticks[g_test_tick];
}

/** @brief Record motor compare levels; real PWM MMIO would fault here. */
static void boardirq_pwm_levels (uint32_t slice, uint32_t level_a,
                                 uint32_t level_b)
{
    if ((BOARD_MOTOR_SLICE_LEFT == slice) ||
        (BOARD_MOTOR_SLICE_RIGHT == slice))
    {
        g_test_pwm[slice - BOARD_MOTOR_SLICE_LEFT][0] = level_a;
        g_test_pwm[slice - BOARD_MOTOR_SLICE_LEFT][1] = level_b;
    }
}

/** @brief A start read late in its 1 us count must still give >= 10 us.
 * Counting ROBOT_TRIGGER_PULSE_US counts from that read can end 9.01 us
 * later, so the pulse must span more counts than the datasheet minimum.
 */
static void boardirq_trigger (void)
{
    boardirq_reset();
    g_test_tick = 0U;
    gp_test_timer = &g_test_ticks[0];
    g_test_levels = 0U;
    gb_initialized = true;
    gb_triggered_once = false;
    boardirq_check(board_ultrasonic_trigger(), "idle sonar accepts a trigger");
    boardirq_check((g_test_fall_us - g_test_rise_us) > ROBOT_TRIGGER_PULSE_US,
                   "TRIG spans more counts than the 10 us datasheet minimum");
    boardirq_check((g_test_fall_us - g_test_rise_us) < 20U,
                   "TRIG pulse remains a bounded busy wait");
    boardirq_check(board_echo_word(BOARD_ECHO_WAIT_RISE, 0U) == g_echo_word,
                   "trigger arms the echo measurement with no pulse");
    gb_initialized = false;
    gp_test_timer = NULL;
    boardirq_reset();
}

/** @brief Preserve expected and observed failures without aborting later
 * cases.
 */
static void boardirq_check (bool b_pass, const char * p_name)
{
    g_test_checks++;
    if (!b_pass)
    {
        g_test_failures++;
        printf("FAIL: %s\n", p_name);
    }
}

/** @brief Reset the ISR-shared words, the lock double and the modelled
 * inputs, leaving actual commissioning flags unchanged.
 */
static void boardirq_reset (void)
{
    g_encoder_word = 0U;
    g_encoder_totals = 0U;
    g_encoder_errors = 0U;
    g_motor_directions = 0U;
    g_echo_word = board_echo_word(BOARD_ECHO_IDLE, 0U);
    g_echo_rise_us = 0U;
    g_test_phase[BOARD_LEFT] = 0U;
    g_test_phase[BOARD_RIGHT] = 0U;
    gb_test_arm = false;
    gb_test_latched = false;
}

/** @brief Place one wheel's A/B status nibbles at the configured pins. */
static uint32_t boardirq_bank_bits (uint32_t nibbles, uint32_t pin_a,
                                    uint32_t pin_b)
{
    return ((nibbles & 0xFU) << BOARD_IRQ_SHIFT(pin_a)) |
           (((nibbles >> 4U) & 0xFU) << BOARD_IRQ_SHIFT(pin_b));
}

/** @brief Real SDK read-only fields are initialized, never cast away and
 * written. Test vectors keep one layout: left wheel A/B nibbles in bits
 * 16..23 and right wheel A/B nibbles in bits 24..31. They are relocated
 * here to the IO_BANK0 banks and bit offsets of the configured encoder
 * pins. The caller's register doubles are restored afterwards, so a latched
 * edge can be taken from inside any board call.
 */
static void boardirq_inject (uint32_t pending, uint32_t left_phase,
                             uint32_t right_phase)
{
    io_bank0_hw_t * p_saved_io = gp_test_io;
    timer_hw_t * p_saved_timer = gp_test_timer;
    uint32_t banks[4] = {0U, 0U, 0U, 0U};
    uint32_t expected[4] = {0U, 0U, 0U, 0U};
    uint32_t left_bits = boardirq_bank_bits((pending >> 16U) & 0xFFU,
                                            ROBOT_PIN_ENCODER_LEFT_A,
                                            ROBOT_PIN_ENCODER_LEFT_B);
    uint32_t right_bits = boardirq_bank_bits((pending >> 24U) & 0xFFU,
                                             ROBOT_PIN_ENCODER_RIGHT_A,
                                             ROBOT_PIN_ENCODER_RIGHT_B);
    uint32_t bank;
    bool b_acknowledged = true;

    banks[BOARD_LEFT_BANK] |= left_bits;
    banks[BOARD_RIGHT_BANK] |= right_bits;
    expected[BOARD_LEFT_BANK] |= left_bits & BOARD_LEFT_MASK;
    expected[BOARD_RIGHT_BANK] |= right_bits & BOARD_RIGHT_MASK;
    {
        io_bank0_hw_t registers =
        {
            .proc0_irq_ctrl.ints = {banks[0], banks[1], banks[2], banks[3]}
        };
        timer_hw_t timer = {.timerawl = 1000U, .timerawh = 0U};

        gp_test_io = &registers;
        gp_test_timer = &timer;
        g_test_levels = (((left_phase >> 1U) & 1U) <<
                         ROBOT_PIN_ENCODER_LEFT_A) |
                        ((left_phase & 1U) << ROBOT_PIN_ENCODER_LEFT_B) |
                        (((right_phase >> 1U) & 1U) <<
                         ROBOT_PIN_ENCODER_RIGHT_A) |
                        ((right_phase & 1U) << ROBOT_PIN_ENCODER_RIGHT_B);
        if (0U != g_test_primask)
        {
            ++g_test_masked_isr;
        }
        board_gpio_isr(13U);
        for (bank = 0U; bank < 4U; bank++)
        {
            b_acknowledged = b_acknowledged &&
                             (registers.intr[bank] == expected[bank]);
        }
        gp_test_io = p_saved_io;
        gp_test_timer = p_saved_timer;
    }
    boardirq_check(b_acknowledged,
                   "ISR acknowledges only owned encoder edges");
}

/** @brief Arm one encoder edge batch to arrive inside the next region. */
static void boardirq_arm (uint32_t pending, uint32_t left_phase,
                          uint32_t right_phase)
{
    g_test_armed_pending = pending;
    g_test_armed_left = left_phase;
    g_test_armed_right = right_phase;
    gb_test_arm = true;
}

/** @brief Unconsumed signed delta of one wheel in the ISR word. */
static int32_t boardirq_delta (uint32_t wheel)
{
    return board_delta(g_encoder_word, wheel);
}

/** @brief Consume deltas and errors as the motion owner does. */
static uint32_t boardirq_read_errors (void)
{
    int32_t left = 0;
    int32_t right = 0;
    uint32_t errors = 0U;

    board_read_encoders(&left, &right, &errors);
    return errors;
}

/** @brief Drive the configured ECHO pin's latched edges through the ISR. */
static void boardirq_inject_echo (uint32_t edge_bits, bool b_level_high,
                                  uint32_t time_us)
{
    uint32_t banks[4] = {0U, 0U, 0U, 0U};

    banks[BOARD_ECHO_BANK] = edge_bits << BOARD_IRQ_SHIFT(ROBOT_PIN_ECHO);
    {
        io_bank0_hw_t registers =
        {
            .proc0_irq_ctrl.ints = {banks[0], banks[1], banks[2], banks[3]}
        };
        timer_hw_t timer = {.timerawl = time_us, .timerawh = 0U};

        gp_test_io = &registers;
        gp_test_timer = &timer;
        g_test_levels = b_level_high ? (1U << ROBOT_PIN_ECHO) : 0U;
        board_gpio_isr(13U);
        boardirq_check(registers.intr[BOARD_ECHO_BANK] ==
                       (banks[BOARD_ECHO_BANK] & BOARD_ECHO_MASK),
                       "ISR acknowledges only the owned ECHO edges");
        gp_test_io = NULL;
        gp_test_timer = NULL;
    }
}

/** @brief Pulse field, bits 3-17 of the echo word. */
static uint32_t boardirq_pulse_field (void)
{
    return (g_echo_word >> BOARD_ECHO_PULSE_SHIFT) & BOARD_ECHO_PULSE_MASK;
}

/** @brief Measure one pulse through the ISR: WAIT_RISE, rise, then fall. */
static void boardirq_echo_pulse (uint32_t rise_us, uint32_t fall_us)
{
    g_echo_word = board_echo_word(BOARD_ECHO_WAIT_RISE, 0U);
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_RISE, true, rise_us);
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_FALL, false, fall_us);
}

/** @brief One width through the ISR and the consumer: the stored field, the
 * unchanged 100..25000 us validity window and the reported pulse.
 */
static void boardirq_echo_case (uint32_t width_us, bool b_expect_valid,
                                const char * p_name)
{
    uint32_t pulse_us = 7U;
    bool b_valid = !b_expect_valid;
    bool b_ready = false;
    uint32_t field = 0U;

    boardirq_reset();
    boardirq_echo_pulse(1000U, 1000U + width_us);
    b_ready = (BOARD_ECHO_READY == board_echo_state(g_echo_word));
    field = boardirq_pulse_field();
    boardirq_check(b_ready && (width_us == field) &&
                   board_ultrasonic_result(&pulse_us, &b_valid) &&
                   (b_expect_valid == b_valid) &&
                   (pulse_us == (b_expect_valid ? width_us : 0U)), p_name);
}

/** @brief Echo timing follows the configured pin through its bank. */
static void boardirq_echo (void)
{
    boardirq_reset();
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_RISE, true, 1000U);
    boardirq_check(BOARD_ECHO_IDLE == board_echo_state(g_echo_word),
                   "an ECHO edge without a pending trigger is ignored");
    g_echo_word = board_echo_word(BOARD_ECHO_WAIT_RISE, 0U);
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_RISE, true, 1000U);
    boardirq_check(BOARD_ECHO_WAIT_FALL == board_echo_state(g_echo_word),
                   "rising ECHO edge starts the pulse measurement");
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_FALL, false, 1580U);
    boardirq_check((BOARD_ECHO_READY == board_echo_state(g_echo_word)) &&
                   (580U == boardirq_pulse_field()),
                   "falling ECHO edge completes the 580 us pulse");
    g_echo_word = board_echo_word(BOARD_ECHO_WAIT_RISE, 0U);
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_RISE |
                         (uint32_t)GPIO_IRQ_EDGE_FALL, false, 2000U);
    boardirq_check(BOARD_ECHO_TIMEOUT == board_echo_state(g_echo_word),
                   "coalesced ECHO edges cannot yield a pulse width");
    boardirq_reset();
}

/** @brief State and pulse share one word: READY publishes both with one
 * store, the consumer takes both from one locked load, and no other state
 * carries a stale pulse.
 */
static void boardirq_echo_coherence (void)
{
    uint32_t pulse_us = 7U;
    bool b_valid = false;

    boardirq_reset();
    boardirq_echo_pulse(1000U, 1580U);
    boardirq_check(((uint32_t)BOARD_ECHO_READY | (580U << 3U)) ==
                   g_echo_word,
                   "READY (bits 0-2) and 580 us (bits 3-17) are one word");
    boardirq_check(board_ultrasonic_result(&pulse_us, &b_valid) &&
                   b_valid && (580U == pulse_us),
                   "the consumer takes state and pulse from one read");
    boardirq_check(0U == g_echo_word,
                   "consuming leaves IDLE with the pulse field cleared");
    boardirq_check(!board_ultrasonic_result(&pulse_us, &b_valid),
                   "an IDLE word has no result");
    g_echo_word = board_echo_word(BOARD_ECHO_WAIT_RISE, 0U);
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_RISE, true, 3000U);
    boardirq_check((uint32_t)BOARD_ECHO_WAIT_FALL == g_echo_word,
                   "a pending measurement carries no pulse");
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_FALL, false, 33001U);
    boardirq_check((uint32_t)BOARD_ECHO_TIMEOUT == g_echo_word,
                   "30001 us times out with no pulse");
    pulse_us = 7U;
    boardirq_check(board_ultrasonic_result(&pulse_us, &b_valid) &&
                   (!b_valid) && (0U == pulse_us),
                   "a timeout is UNKNOWN and never the previous pulse");
    g_echo_word = 5U | (580U << BOARD_ECHO_PULSE_SHIFT);
    pulse_us = 7U;
    boardirq_check(board_ultrasonic_result(&pulse_us, &b_valid) &&
                   (!b_valid) && (0U == pulse_us) && (0U == g_echo_word),
                   "state codes 5..7 read as TIMEOUT, never as a pulse");
    boardirq_reset();
}

/** @brief The 15-bit pulse field keeps every board decision exact: the
 * 100..25000 us window, the 30000 us timeout and the 2^32 us timer wrap.
 */
static void boardirq_echo_boundaries (void)
{
    boardirq_echo_case(99U, false, "99 us is below the valid window");
    boardirq_echo_case(100U, true, "100 us is the shortest valid pulse");
    boardirq_echo_case(25000U, true, "25000 us is the longest valid pulse");
    boardirq_echo_case(25001U, false, "25001 us is above the valid window");
    boardirq_echo_case(30000U, false,
                       "30000 us is stored whole in 15 bits, not valid");
    boardirq_reset();
    boardirq_echo_pulse(1000U, 31001U);
    boardirq_check((uint32_t)BOARD_ECHO_TIMEOUT == g_echo_word,
                   "30001 us is a timeout, not a stored pulse");
    boardirq_reset();
    boardirq_echo_pulse(0xFFFFFF00U, 0x00000200U);
    boardirq_check((BOARD_ECHO_READY == board_echo_state(g_echo_word)) &&
                   (768U == boardirq_pulse_field()),
                   "a pulse across the 2^32 us timer wrap measures 768 us");
    boardirq_reset();
}

/** @brief Service times either pending measurement out at exactly
 * 30000 us: WAIT_RISE (no echo rise ever arrives) and WAIT_FALL. It never
 * touches a completed result. A trigger with ECHO already high is refused
 * as TIMEOUT.
 */
static void boardirq_echo_service (void)
{
    timer_hw_t before = {.timerawl = 5000U + 29999U, .timerawh = 0U};
    timer_hw_t limit = {.timerawl = 5000U + 30000U, .timerawh = 0U};
    uint32_t pulse_us = 0U;
    bool b_valid = false;

    boardirq_reset();
    g_trigger_time_us = 5000U;
    g_echo_word = board_echo_word(BOARD_ECHO_WAIT_RISE, 0U);
    gp_test_timer = &before;
    board_ultrasonic_service();
    boardirq_check(board_echo_word(BOARD_ECHO_WAIT_RISE, 0U) == g_echo_word,
                   "service keeps WAIT_RISE younger than 30000 us");
    gp_test_timer = &limit;
    board_ultrasonic_service();
    boardirq_check((uint32_t)BOARD_ECHO_TIMEOUT == g_echo_word,
                   "no echo rise by 30000 us: service times out, no pulse");
    g_echo_word = board_echo_word(BOARD_ECHO_WAIT_FALL, 0U);
    gp_test_timer = &before;
    board_ultrasonic_service();
    boardirq_check(BOARD_ECHO_WAIT_FALL == board_echo_state(g_echo_word),
                   "service keeps a measurement younger than 30000 us");
    gp_test_timer = &limit;
    board_ultrasonic_service();
    boardirq_check((uint32_t)BOARD_ECHO_TIMEOUT == g_echo_word,
                   "service times out at 30000 us with no pulse");
    g_echo_word = board_echo_word(BOARD_ECHO_READY, 580U);
    board_ultrasonic_service();
    boardirq_check(board_ultrasonic_result(&pulse_us, &b_valid) && b_valid &&
                   (580U == pulse_us),
                   "service at 30000 us leaves a READY 580 us result intact");
    boardirq_reset();
    gb_initialized = true;
    gb_triggered_once = false;
    g_test_levels = 1U << ROBOT_PIN_ECHO;
    boardirq_check((!board_ultrasonic_trigger()) &&
                   ((uint32_t)BOARD_ECHO_TIMEOUT == g_echo_word),
                   "a trigger with ECHO already high is refused as TIMEOUT");
    gb_initialized = false;
    gp_test_timer = NULL;
    g_test_levels = 0U;
    boardirq_reset();
}

/** @brief Only a rise sampled high in WAIT_RISE and a fall sampled low in
 * WAIT_FALL advance the echo word. An edge that its sampled level
 * contradicts, an edge out of order and a stray edge after READY each leave
 * TIMEOUT with no pulse: the consumer reads UNKNOWN, never a stale pulse.
 */
static void boardirq_echo_order (void)
{
    uint32_t pulse_us = 7U;
    bool b_valid = true;

    boardirq_reset();
    g_echo_word = board_echo_word(BOARD_ECHO_WAIT_RISE, 0U);
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_RISE, false, 1000U);
    boardirq_check((uint32_t)BOARD_ECHO_TIMEOUT == g_echo_word,
                   "a rise sampled with ECHO low is TIMEOUT");
    g_echo_word = board_echo_word(BOARD_ECHO_WAIT_RISE, 0U);
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_FALL, false, 1000U);
    boardirq_check((uint32_t)BOARD_ECHO_TIMEOUT == g_echo_word,
                   "a fall before any rise is TIMEOUT");
    g_echo_word = board_echo_word(BOARD_ECHO_WAIT_FALL, 0U);
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_FALL, true, 1580U);
    boardirq_check((uint32_t)BOARD_ECHO_TIMEOUT == g_echo_word,
                   "a fall sampled with ECHO high is TIMEOUT");
    g_echo_word = board_echo_word(BOARD_ECHO_WAIT_FALL, 0U);
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_RISE, true, 1580U);
    boardirq_check((uint32_t)BOARD_ECHO_TIMEOUT == g_echo_word,
                   "a second rise before the fall is TIMEOUT");
    boardirq_reset();
    boardirq_echo_pulse(1000U, 1580U);
    boardirq_check(board_echo_word(BOARD_ECHO_READY, 580U) == g_echo_word,
                   "a 580 us pulse is READY before the stray edge");
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_RISE, true, 1880U);
    boardirq_check((uint32_t)BOARD_ECHO_TIMEOUT == g_echo_word,
                   "a stray edge after READY leaves TIMEOUT with no pulse");
    boardirq_check(board_ultrasonic_result(&pulse_us, &b_valid) &&
                   (!b_valid) && (0U == pulse_us),
                   "the stray edge makes the result UNKNOWN, never 580 us");
    boardirq_reset();
}

/** @brief Set a pending echo word, then trigger: true when the trigger is
 * refused and the word is left exactly as it was.
 */
static bool boardirq_trigger_refused (uint32_t word)
{
    g_echo_word = word;
    return (!board_ultrasonic_trigger()) && (word == g_echo_word);
}

/** @brief A trigger due 65000 us after the last one is refused while a
 * measurement or an unconsumed result is pending, and leaves the word as it
 * was, so a READY 580 us result is still delivered. Once the result is
 * consumed, the same trigger is accepted; 64999 us is still too early.
 */
static void boardirq_trigger_pending (void)
{
    timer_hw_t early = {.timerawl = 5000U + ROBOT_TRIGGER_INTERVAL_US - 1U,
                        .timerawh = 0U};
    timer_hw_t due = {.timerawl = 5000U + ROBOT_TRIGGER_INTERVAL_US,
                      .timerawh = 0U};
    uint32_t pulse_us = 0U;
    bool b_valid = false;

    boardirq_reset();
    gb_initialized = true;
    gb_triggered_once = true;
    g_trigger_time_us = 5000U;
    g_test_levels = 0U;
    gp_test_timer = &due;
    boardirq_check(boardirq_trigger_refused(
                       board_echo_word(BOARD_ECHO_WAIT_RISE, 0U)),
                   "a due trigger is refused while WAIT_RISE is pending");
    boardirq_check(boardirq_trigger_refused(
                       board_echo_word(BOARD_ECHO_WAIT_FALL, 0U)),
                   "a due trigger is refused while WAIT_FALL is pending");
    boardirq_check(boardirq_trigger_refused(
                       board_echo_word(BOARD_ECHO_TIMEOUT, 0U)),
                   "a due trigger is refused while a TIMEOUT is unconsumed");
    boardirq_check(boardirq_trigger_refused(
                       board_echo_word(BOARD_ECHO_READY, 580U)),
                   "a due trigger is refused while READY is unconsumed");
    boardirq_check(board_ultrasonic_result(&pulse_us, &b_valid) && b_valid &&
                   (580U == pulse_us),
                   "the refused trigger leaves the 580 us result delivered");
    gp_test_timer = &early;
    boardirq_check((!board_ultrasonic_trigger()) && (0U == g_echo_word),
                   "an idle trigger 64999 us after the last is refused");
    /* The accepted trigger's busy wait steps the tick table from its start. */
    g_test_tick = 0U;
    gp_test_timer = &due;
    boardirq_check(board_ultrasonic_trigger() &&
                   (board_echo_word(BOARD_ECHO_WAIT_RISE, 0U) ==
                    g_echo_word) &&
                   ((5000U + ROBOT_TRIGGER_INTERVAL_US) == g_trigger_time_us),
                   "once consumed, the trigger 65000 us later is accepted");
    gb_initialized = false;
    gp_test_timer = NULL;
    boardirq_reset();
}

/** @brief Direction codes are 2-bit two's complement, as bitpack_signed()
 * and bitpack_extend() with width 2: -1, 0, +1 are 3, 0, 1; 2 is reserved.
 * The command packs 13-bit levels, both codes and b_valid in 31 bits.
 */
static void boardirq_motor_codes (void)
{
    board_motor_command_t command = {0};

    boardirq_check((3U == BOARD_DIRECTION_ENCODE(-1)) &&
                   (0U == BOARD_DIRECTION_ENCODE(0)) &&
                   (1U == BOARD_DIRECTION_ENCODE(1)),
                   "-1, 0 and +1 encode as 3, 0 and 1");
    boardirq_check((-1 == board_direction_decode(3U)) &&
                   (0 == board_direction_decode(0U)) &&
                   (1 == board_direction_decode(1U)) &&
                   (-2 == board_direction_decode(BOARD_DIRECTION_INVALID)),
                   "codes decode back; the reserved code reads -2");
    boardirq_check((-1 == board_direction_decode(0xFFFFFFFFU)) &&
                   (1 == board_direction_decode(0x5U)),
                   "decoding reads only the two code bits");
    command.left_level = (ROBOT_PWM_WRAP + 1U) & BOARD_LEVEL_MASK;
    command.right_level = BOARD_LEVEL_MASK;
    command.left_direction = BOARD_DIRECTION_ENCODE(-1);
    command.right_direction = BOARD_DIRECTION_ENCODE(1);
    command.b_valid = true;
    boardirq_check((6250U == command.left_level) &&
                   (8191U == command.right_level) &&
                   (-1 == board_direction_decode(command.left_direction)) &&
                   (1 == board_direction_decode(command.right_direction)) &&
                   command.b_valid,
                   "13-bit levels hold 0..6250 and codes round-trip");
    boardirq_check(sizeof(uint32_t) == sizeof(board_motor_command_t),
                   "the 31-bit command occupies one word on this ABI");
}

/** @brief Commit preconditions: initialized board, STOP released (high)
 * and the level limit board_init() derives from ROBOT_MAX_DUTY.
 */
static void boardirq_motor_setup (void)
{
    boardirq_reset();
    gb_initialized = true;
    g_motor_level_limit = board_duty_level(1.0);
    g_test_levels = 1U << ROBOT_PIN_STOP;
    g_test_pwm[0][0] = 1U;
    g_test_pwm[0][1] = 1U;
    g_test_pwm[1][0] = 1U;
    g_test_pwm[1][1] = 1U;
}

/** @brief Both slices at zero and both direction codes 0 (BRAKE). */
static bool boardirq_braked (void)
{
    return (0U == g_test_pwm[0][0]) && (0U == g_test_pwm[0][1]) &&
           (0U == g_test_pwm[1][0]) && (0U == g_test_pwm[1][1]) &&
           (0U == g_motor_directions);
}

/** @brief Shipped gates (0) refuse every preparation and brake every
 * commit; commissioned gates run the packing and commit checks.
 */
static void boardirq_motor_command (void)
{
#if (0 != ROBOT_CALIBRATION_CONFIRMED) && (0 != ROBOT_WIRING_CONFIRMED)
    boardirq_motor_prepare();
    boardirq_motor_commit();
#else
    board_motor_command_t command = {0};

    boardirq_motor_setup();
    command.left_level = 100U;
    command.b_valid = true;
    boardirq_check((!board_motor_prepare(0.2, -0.2, &command)) &&
                   (0U == command.left_level) && (!command.b_valid),
                   "unconfirmed gates refuse and leave an invalid command");
    command.left_level = 100U;
    command.left_direction = BOARD_DIRECTION_ENCODE(1);
    command.b_valid = true;
    board_motor_commit(&command);
    boardirq_check(boardirq_braked(), "unconfirmed gates brake a commit");
#endif
    gb_initialized = false;
    g_test_levels = 0U;
    boardirq_reset();
}

#if (0 != ROBOT_CALIBRATION_CONFIRMED) && (0 != ROBOT_WIRING_CONFIRMED)
/** @brief Prepare packs each wheel's capped legacy level and a direction
 * code that is 0 exactly when the level is 0; invalid duty leaves b_valid
 * false. Duties are pre-multiplied by the motor signs, so hardware polarity
 * is left forward and right reverse for any configured sign.
 */
static void boardirq_motor_prepare (void)
{
    board_motor_command_t command = {0};
    double left = 0.2 * ROBOT_LEFT_MOTOR_SIGN;
    double right = -0.3 * ROBOT_RIGHT_MOTOR_SIGN;

    boardirq_motor_setup();
    boardirq_check(board_motor_prepare(left, right, &command) &&
                   (board_duty_level(0.2) == command.left_level) &&
                   (board_duty_level(0.3) == command.right_level) &&
                   (1U == command.left_direction) &&
                   (3U == command.right_direction) && command.b_valid,
                   "prepare packs each wheel's level and direction code");
    boardirq_check(board_motor_prepare(1.0 * ROBOT_LEFT_MOTOR_SIGN,
                                       -1.0 * ROBOT_RIGHT_MOTOR_SIGN,
                                       &command) &&
                   (g_motor_level_limit == command.left_level) &&
                   (g_motor_level_limit == command.right_level),
                   "full duty packs the capped level limit");
    boardirq_check(board_motor_prepare(0.0, -0.0, &command) &&
                   (0U == command.left_level) &&
                   (0U == command.left_direction) &&
                   (0U == command.right_direction) && command.b_valid,
                   "zero duty packs level 0 with direction code 0");
    boardirq_check(board_motor_prepare(1.0e-9, -1.0e-9, &command) &&
                   (0U == command.left_direction) &&
                   (0U == command.right_direction),
                   "a duty below one level count has no direction");
    boardirq_check((!board_motor_prepare(NAN, 0.0, &command)) &&
                   (!command.b_valid) && (0U == command.left_level) &&
                   (0U == command.left_direction),
                   "non-finite duty leaves a zeroed invalid command");
    boardirq_check(!board_motor_prepare(0.0, 1.0000001, &command),
                   "duty magnitude above 1 is refused");
}

/** @brief Commit unpacks codes into PWM channels and the direction byte,
 * inserts one zero cycle before a polarity change and brakes on either
 * wheel's reserved code, b_valid false, NULL or STOP. Both sides of the
 * level limit are pinned on each wheel: the capped full-duty level (equal
 * to the limit) drives, one count above it brakes.
 */
static void boardirq_motor_commit (void)
{
    board_motor_command_t drive = {0};
    board_motor_command_t reverse = {0};
    board_motor_command_t full = {0};
    board_motor_command_t bad = {0};
    uint32_t level = board_duty_level(0.2);
    uint32_t drive_right = board_duty_level(0.3);
    uint32_t above = (uint32_t)g_motor_level_limit + 1U;

    boardirq_motor_setup();
    (void)board_motor_prepare(0.2 * ROBOT_LEFT_MOTOR_SIGN,
                              -0.3 * ROBOT_RIGHT_MOTOR_SIGN, &drive);
    (void)board_motor_prepare(-0.2 * ROBOT_LEFT_MOTOR_SIGN,
                              -0.2 * ROBOT_RIGHT_MOTOR_SIGN, &reverse);
    (void)board_motor_prepare(1.0 * ROBOT_LEFT_MOTOR_SIGN,
                              1.0 * ROBOT_RIGHT_MOTOR_SIGN, &full);
    board_motor_commit(&drive);
    boardirq_check((level == g_test_pwm[0][0]) && (0U == g_test_pwm[0][1]) &&
                   (0U == g_test_pwm[1][0]) &&
                   (drive_right == g_test_pwm[1][1]) &&
                   (0xDU == g_motor_directions),
                   "codes +1/-1 drive each wheel's level, byte 1 | 3 << 2");
    board_motor_commit(&reverse);
    boardirq_check((0U == g_test_pwm[0][0]) && (0U == g_test_pwm[0][1]) &&
                   (level == g_test_pwm[1][1]) &&
                   (0xCU == g_motor_directions),
                   "a polarity change inserts one zero cycle, code 0");
    board_motor_commit(&reverse);
    boardirq_check((0U == g_test_pwm[0][0]) &&
                   (level == g_test_pwm[0][1]) &&
                   (0xFU == g_motor_directions),
                   "the next commit drives the new polarity");
    bad = drive;
    bad.left_direction = BOARD_DIRECTION_INVALID;
    board_motor_commit(&bad);
    boardirq_check(boardirq_braked(), "the left reserved direction brakes");
    board_motor_commit(&drive);
    bad = drive;
    bad.right_direction = BOARD_DIRECTION_INVALID;
    board_motor_commit(&bad);
    boardirq_check(boardirq_braked(), "the right reserved direction brakes");
    /* From BRAKE (codes 0) a commit drives at once, with no zero cycle. */
    board_motor_brake();
    board_motor_commit(&full);
    boardirq_check((g_motor_level_limit == g_test_pwm[0][0]) &&
                   (0U == g_test_pwm[0][1]) &&
                   (g_motor_level_limit == g_test_pwm[1][0]) &&
                   (0U == g_test_pwm[1][1]) && (0x5U == g_motor_directions),
                   "a level equal to the limit drives both wheels");
    bad = full;
    bad.left_level = above & BOARD_LEVEL_MASK;
    board_motor_commit(&bad);
    boardirq_check(boardirq_braked(),
                   "a left level one above the limit brakes");
    board_motor_commit(&full);
    bad = full;
    bad.right_level = above & BOARD_LEVEL_MASK;
    board_motor_commit(&bad);
    boardirq_check(boardirq_braked(),
                   "a right level one above the limit brakes");
    board_motor_commit(&drive);
    bad = drive;
    bad.b_valid = false;
    board_motor_commit(&bad);
    boardirq_check(boardirq_braked(), "an invalid command brakes");
    board_motor_commit(&drive);
    board_motor_commit(NULL);
    boardirq_check(boardirq_braked(), "a null command brakes");
    board_motor_commit(&drive);
    g_test_levels = 0U;
    board_motor_commit(&drive);
    boardirq_check(boardirq_braked(), "a pressed STOP brakes the commit");
#if (ROBOT_ENCODER_CHANNELS == 1) && ROBOT_SINGLE_CHANNEL_DIRECTION_CONFIRMED
    g_test_levels = 1U << ROBOT_PIN_STOP;
    board_motor_commit(&drive);
    boardirq_inject(0x08080000U, 2U, 2U);
    boardirq_check((ROBOT_LEFT_MOTOR_SIGN * ROBOT_LEFT_ENCODER_SIGN ==
                    boardirq_delta(BOARD_LEFT)) &&
                   (-ROBOT_RIGHT_MOTOR_SIGN * ROBOT_RIGHT_ENCODER_SIGN ==
                    boardirq_delta(BOARD_RIGHT)) &&
                   (0U == g_encoder_errors),
                   "the ISR signs one-channel edges from the committed byte");
    board_motor_brake();
    boardirq_inject(0x04040000U, 0U, 0U);
    boardirq_check(BOARD_ENCODER_ERRORS == g_encoder_errors,
                   "after BRAKE the byte gives no direction: both errors");
#endif
}
#endif

#if ROBOT_ENCODER_CHANNELS == 1
/** @brief Constant B counts only in explicitly confirmed one-channel mode.
 * Sticky bits are consumed between cases, so each case shows its own error.
 */
static void boardirq_single_channel (void)
{
    uint16_t left_total = 0U;
    uint16_t right_total = 0U;

    boardirq_reset();
    g_motor_directions = (BOARD_DIRECTION_ENCODE(1) |
                          (BOARD_DIRECTION_ENCODE(-1) << 2U)) & 0xFU;
    boardirq_inject(0x08080000U, 2U, 2U);
    boardirq_inject(0x04040000U, 0U, 0U);
    board_encoder_totals(&left_total, &right_total);
#if ROBOT_SINGLE_CHANNEL_DIRECTION_CONFIRMED
    boardirq_check((2 == boardirq_delta(BOARD_LEFT)) &&
                   (-2 == boardirq_delta(BOARD_RIGHT)) &&
                   (2U == left_total) && (0xFFFEU == right_total) &&
                   (0U == g_encoder_errors),
                   "constant B: both A edges count with applied direction");
    boardirq_inject(0x80800000U, 1U, 1U);
    boardirq_check((2 == boardirq_delta(BOARD_LEFT)) &&
                   (-2 == boardirq_delta(BOARD_RIGHT)) &&
                   (0U == g_encoder_errors), "B-only edges ignored");
    boardirq_reset();
    g_motor_directions = (BOARD_DIRECTION_ENCODE(1) << 2U) & 0xFU;
    boardirq_inject(0x08080000U, 2U, 2U);
    boardirq_check((0 == boardirq_delta(BOARD_LEFT)) &&
                   (0U == (g_encoder_totals & BOARD_TOTAL_MASK)) &&
                   (1 == boardirq_delta(BOARD_RIGHT)) &&
                   (BOARD_ENCODER_ERROR_LEFT == g_encoder_errors),
                   "unknown/braked direction cannot manufacture displacement");
    boardirq_reset();
    g_motor_directions = BOARD_DIRECTION_INVALID;
    boardirq_inject(0x00080000U, 2U, 0U);
    boardirq_check((0 == boardirq_delta(BOARD_LEFT)) &&
                   (BOARD_ENCODER_ERROR_LEFT == g_encoder_errors),
                   "the reserved direction code never counts");
    boardirq_reset();
    g_motor_directions = BOARD_DIRECTION_ENCODE(1);
    boardirq_inject(0x000C0000U, 2U, 0U);
    boardirq_check((0 == boardirq_delta(BOARD_LEFT)) &&
                   (BOARD_ENCODER_ERROR_LEFT == boardirq_read_errors()),
                   "coalesced A edges rejected without a guessed count");
    boardirq_inject(0x00080000U, 0U, 0U);
    boardirq_check((0 == boardirq_delta(BOARD_LEFT)) &&
                   (BOARD_ENCODER_ERROR_LEFT == boardirq_read_errors()),
                   "inconsistent A polarity rejected and resynchronized");
    boardirq_inject(0x00080000U, 2U, 0U);
    boardirq_check((1 == boardirq_delta(BOARD_LEFT)) &&
                   (0U == g_encoder_errors),
                   "next clean A edge resumes after resynchronization");
    g_encoder_word = (g_encoder_word | BOARD_DELTA_MASK) &
                     (BOARD_ENCODER_BITS ^ BOARD_DELTA_SIGN);
    boardirq_inject(0x00040000U, 0U, 0U);
    boardirq_check((31 == boardirq_delta(BOARD_LEFT)) &&
                   (2U == (g_encoder_totals & BOARD_TOTAL_MASK)) &&
                   (BOARD_ENCODER_ERROR_LEFT == g_encoder_errors),
                   "one-channel delta saturates at +31 and sets the bit");
#else
    boardirq_check((0 == boardirq_delta(BOARD_LEFT)) &&
                   (0 == boardirq_delta(BOARD_RIGHT)) &&
                   (0U == left_total) && (0U == right_total) &&
                   (BOARD_ENCODER_ERRORS == boardirq_read_errors()),
                   "unconfirmed inferred direction never yields counts");
    boardirq_inject(0x08080000U, 2U, 2U);
    boardirq_check(BOARD_ENCODER_ERRORS == boardirq_read_errors(),
                   "every unconfirmed edge batch reports both wheels");
#endif
}

/** @brief Boot-window counts (confirmed mode: two A edges per wheel, left
 * forward, right reverse) and errors on both wheels are discarded in one
 * region; the phases and both totals stay, and an A edge latched during the
 * discard is taken after it, so it decodes from the kept phase as the
 * configuration dictates.
 */
static void boardirq_single_discard (void)
{
    uint32_t regions = 0U;
    uint16_t left_before = 0U;
    uint16_t right_before = 0U;
    uint16_t left_total = 0U;
    uint16_t right_total = 0U;

    boardirq_reset();
    g_motor_directions = (BOARD_DIRECTION_ENCODE(1) |
                          (BOARD_DIRECTION_ENCODE(-1) << 2U)) & 0xFU;
    boardirq_inject(0x08080000U, 2U, 2U);
    boardirq_inject(0x04040000U, 0U, 0U);
    boardirq_inject(0x0C0C0000U, 2U, 2U);
    board_encoder_totals(&left_before, &right_before);
    boardirq_check(BOARD_ENCODER_ERRORS == g_encoder_errors,
                   "boot-window coalesced A edges latch both bits");
#if ROBOT_SINGLE_CHANNEL_DIRECTION_CONFIRMED
    boardirq_check((2 == boardirq_delta(BOARD_LEFT)) &&
                   (-2 == boardirq_delta(BOARD_RIGHT)) &&
                   (2U == left_before) && (0xFFFEU == right_before),
                   "boot-window A edges count on both wheels");
#endif
    g_motor_directions = BOARD_DIRECTION_ENCODE(1);
    boardirq_arm(0x00040000U, 0U, 2U);
    regions = g_test_regions;
    board_encoders_discard();
    boardirq_check(((regions + 1U) == g_test_regions) &&
                   ((2U << (BOARD_PHASE_SHIFT + 2U)) ==
                    (g_encoder_word & BOARD_PHASE_BITS)) &&
                   (0 == boardirq_delta(BOARD_RIGHT)),
                   "discard: one region, phases kept, right delta cleared");
    board_encoder_totals(&left_total, &right_total);
    boardirq_check(right_before == right_total,
                   "discard keeps the right total");
#if ROBOT_SINGLE_CHANNEL_DIRECTION_CONFIRMED
    /* The uint16_t cast is the modulo-2^16 total after one more count. */
    boardirq_check((1 == boardirq_delta(BOARD_LEFT)) &&
                   ((uint16_t)(left_before + 1U) == left_total) &&
                   (0U == boardirq_read_errors()),
                   "only the edge latched during the discard counts");
#else
    boardirq_check((0 == boardirq_delta(BOARD_LEFT)) &&
                   (left_before == left_total) &&
                   (BOARD_ENCODER_ERROR_LEFT == boardirq_read_errors()),
                   "unconfirmed mode reports the post-discard edge");
#endif
    boardirq_reset();
}
#endif

#if ROBOT_ENCODER_CHANNELS == 2
/** @brief Pending vector and next modelled phase for one valid x4
 * transition: forward 0-1-3-2-0, reverse 0-2-3-1-0. The changed channel's
 * edge goes to the wheel's A (low) or B (high) nibble.
 */
static uint32_t boardirq_transition (uint32_t wheel, int32_t direction)
{
    static const uint32_t forward[4] = {1U, 3U, 0U, 2U};
    static const uint32_t reverse[4] = {2U, 0U, 3U, 1U};
    uint32_t previous = g_test_phase[wheel];
    uint32_t current = (direction > 0) ? forward[previous] :
                                         reverse[previous];
    uint32_t edge = (uint32_t)GPIO_IRQ_EDGE_FALL;

    if (0U != ((previous ^ current) & 2U))
    {
        if (0U != (current & 2U))
        {
            edge = (uint32_t)GPIO_IRQ_EDGE_RISE;
        }
    }
    else
    {
        if (0U != (current & 1U))
        {
            edge = (uint32_t)GPIO_IRQ_EDGE_RISE;
        }
        edge <<= 4U;
    }
    g_test_phase[wheel] = current;
    return edge << (16U + (8U * wheel));
}

/** @brief Drive one valid transition of the modelled wheel through the ISR.
 */
static void boardirq_step (uint32_t wheel, int32_t direction)
{
    uint32_t pending = boardirq_transition(wheel, direction);

    boardirq_inject(pending, g_test_phase[BOARD_LEFT],
                    g_test_phase[BOARD_RIGHT]);
}

/** @brief Both wheels retain x4 direction and non-consuming totals. */
static void boardirq_normal_cycles (void)
{
    int32_t left = 0;
    int32_t right = 0;
    uint32_t errors = 0U;
    uint16_t left_total = 0U;
    uint16_t right_total = 0U;

    boardirq_reset();
    boardirq_inject(0x80800000U, 1U, 1U);
    boardirq_inject(0x08080000U, 3U, 3U);
    boardirq_inject(0x40400000U, 2U, 2U);
    boardirq_inject(0x04040000U, 0U, 0U);
    board_read_encoders(&left, &right, &errors);
    boardirq_check((4 == left) && (4 == right) && (0U == errors),
                   "four distinct forward transitions count on each wheel");
    board_encoder_totals(&left_total, &right_total);
    boardirq_check((4U == left_total) && (4U == right_total),
                   "consuming deltas preserves independent position totals");
    boardirq_inject(0x08080000U, 2U, 2U);
    boardirq_inject(0x80800000U, 3U, 3U);
    boardirq_inject(0x04040000U, 1U, 1U);
    boardirq_inject(0x40400000U, 0U, 0U);
    board_read_encoders(&left, &right, &errors);
    boardirq_check((-4 == left) && (-4 == right) && (0U == errors),
                   "four distinct reverse transitions count on each wheel");
    board_encoder_totals(&left_total, &right_total);
    boardirq_check((0U == left_total) && (0U == right_total),
                   "reverse transitions return totals to original position");
}

/** @brief One ISR entry with a left encoder edge and an ECHO rise latched
 * together handles both and acknowledges each bank once. With ECHO on GP1
 * both sit in bank 0, so the acknowledgement must merge them: a second
 * write to the same register would leave the first source's bits behind.
 */
static void boardirq_shared_bank (void)
{
    uint32_t banks[4] = {0U, 0U, 0U, 0U};
    uint32_t expected[4] = {0U, 0U, 0U, 0U};
    uint32_t left_bits = boardirq_bank_bits(0x80U, ROBOT_PIN_ENCODER_LEFT_A,
                                            ROBOT_PIN_ENCODER_LEFT_B);
    uint32_t echo_bits = (uint32_t)GPIO_IRQ_EDGE_RISE <<
                         BOARD_IRQ_SHIFT(ROBOT_PIN_ECHO);
    uint32_t bank = 0U;
    int32_t left = 0;
    int32_t right = 0;
    uint32_t errors = 0U;
    bool b_acknowledged = true;

    boardirq_reset();
    g_echo_word = board_echo_word(BOARD_ECHO_WAIT_RISE, 0U);
    banks[BOARD_LEFT_BANK] |= left_bits;
    banks[BOARD_ECHO_BANK] |= echo_bits;
    expected[BOARD_LEFT_BANK] |= left_bits & BOARD_LEFT_MASK;
    expected[BOARD_ECHO_BANK] |= echo_bits & BOARD_ECHO_MASK;
    {
        io_bank0_hw_t registers =
        {
            .proc0_irq_ctrl.ints = {banks[0], banks[1], banks[2], banks[3]}
        };
        timer_hw_t timer = {.timerawl = 1000U, .timerawh = 0U};

        gp_test_io = &registers;
        gp_test_timer = &timer;
        /* Left phase 1 (B high) after the edge; ECHO high after its rise. */
        g_test_levels = (1U << ROBOT_PIN_ENCODER_LEFT_B) |
                        (1U << ROBOT_PIN_ECHO);
        board_gpio_isr(13U);
        for (bank = 0U; bank < 4U; bank++)
        {
            b_acknowledged = b_acknowledged &&
                             (registers.intr[bank] == expected[bank]);
        }
        gp_test_io = NULL;
        gp_test_timer = NULL;
    }
    boardirq_check(b_acknowledged,
                   "shared bank: both sources acknowledged by one write");
    boardirq_check(BOARD_ECHO_WAIT_FALL == board_echo_state(g_echo_word),
                   "shared bank: the ECHO rise starts the measurement");
    board_read_encoders(&left, &right, &errors);
    boardirq_check((1 == left) && (0 == right) && (0U == errors),
                   "shared bank: the left edge counts once");
    boardirq_reset();
}

/** @brief Pending bits are latches, so their ordering and edge count are
 * unknown. Each wheel latches only its own sticky error bit.
 */
static void boardirq_coalesced (void)
{
    boardirq_reset();
    boardirq_inject(0x00CC0000U, 0U, 0U);
    boardirq_check((BOARD_ENCODER_ERROR_LEFT == g_encoder_errors) &&
                   (0 == boardirq_delta(BOARD_LEFT)) &&
                   (0U == (g_encoder_totals & BOARD_TOTAL_MASK)),
                   "coalesced full left cycle reports error, no displacement");
    boardirq_inject(0xCC000000U, 0U, 0U);
    boardirq_check((BOARD_ENCODER_ERRORS == g_encoder_errors) &&
                   (0 == boardirq_delta(BOARD_RIGHT)) &&
                   (0U == (g_encoder_totals >> 16U)),
                   "coalesced full right cycle sets the right bit");
    boardirq_check(BOARD_ENCODER_ERRORS == boardirq_read_errors(),
                   "both sticky bits are reported together");
    boardirq_inject(0x000C0000U, 2U, 0U);
    boardirq_check((BOARD_ENCODER_ERROR_LEFT == g_encoder_errors) &&
                   (0 == boardirq_delta(BOARD_LEFT)) &&
                   (0U == (g_encoder_totals & BOARD_TOTAL_MASK)),
                   "both edges of one pin cannot masquerade as one count");
    boardirq_reset();
    boardirq_inject(0x00880000U, 3U, 0U);
    boardirq_check((BOARD_ENCODER_ERROR_LEFT == g_encoder_errors) &&
                   (0 == boardirq_delta(BOARD_LEFT)),
                   "two channel transitions remain an ambiguous jump");
}

/** @brief A status/sample race must not be accepted as zero or one edge. */
static void boardirq_inconsistent_sample (void)
{
    boardirq_reset();
    boardirq_inject(0x00080000U, 0U, 0U);
    boardirq_check((0 == boardirq_delta(BOARD_LEFT)) &&
                   (BOARD_ENCODER_ERROR_LEFT == boardirq_read_errors()),
                   "pending edge with unchanged phase reports lost evidence");
    boardirq_inject(0x00080000U, 1U, 0U);
    boardirq_check((0 == boardirq_delta(BOARD_LEFT)) &&
                   (BOARD_ENCODER_ERROR_LEFT == boardirq_read_errors()),
                   "pending A edge cannot authorize a B phase transition");
    boardirq_reset();
    boardirq_inject(0x04000000U, 0U, 2U);
    boardirq_check((0 == boardirq_delta(BOARD_RIGHT)) &&
                   (BOARD_ENCODER_ERROR_RIGHT == g_encoder_errors),
                   "pending fall cannot authorize a sampled rising edge");
}

/** @brief A bad wheel must not poison the other wheel or keep an obsolete
 * phase; the next clean edge decodes without a new error.
 */
static void boardirq_resynchronization (void)
{
    int32_t left = 0;
    int32_t right = 0;
    uint32_t errors = 0U;

    boardirq_reset();
    boardirq_inject(0x800C0000U, 2U, 1U);
    boardirq_check((BOARD_ENCODER_ERROR_LEFT == g_encoder_errors) &&
                   (0 == boardirq_delta(BOARD_LEFT)) &&
                   (1 == boardirq_delta(BOARD_RIGHT)),
                   "ambiguous left wheel does not discard a valid right edge");
    boardirq_check(2U == ((g_encoder_word >> BOARD_PHASE_SHIFT) & 3U),
                   "ambiguous acquisition resynchronizes observed phase");
    board_read_encoders(&left, &right, &errors);
    boardirq_check((0 == left) && (1 == right) &&
                   (BOARD_ENCODER_ERROR_LEFT == errors),
                   "the read reports the right edge and the left error");
    boardirq_inject(0x00040000U, 0U, 1U);
    boardirq_check((0U == g_encoder_errors) &&
                   (1 == boardirq_delta(BOARD_LEFT)) &&
                   (1U == (g_encoder_totals & BOARD_TOTAL_MASK)),
                   "next clean edge decodes from resynchronized phase");
}

/** @brief Sticky bits cannot wrap back to zero under persistent overload:
 * 300 ambiguous batches leave both bits set and one read consumes them.
 */
static void boardirq_sticky_errors (void)
{
    int32_t left = 0;
    int32_t right = 0;
    uint32_t errors = 0U;
    uint32_t count = 0U;

    boardirq_reset();
    for (count = 0U; count < 300U; ++count)
    {
        boardirq_inject(0xCCCC0000U, 0U, 0U);
    }
    board_read_encoders(&left, &right, &errors);
    boardirq_check((BOARD_ENCODER_ERRORS == errors) && (0 == left) &&
                   (0 == right),
                   "persistent ambiguous edges leave both sticky bits set");
    board_read_encoders(&left, &right, &errors);
    boardirq_check(0U == errors,
                   "motion owner consumes the error report once");
}

/** @brief Constant B cannot supply measured direction in quadrature mode.
 */
static void boardirq_constant_b_quadrature (void)
{
    boardirq_reset();
    g_motor_directions = (BOARD_DIRECTION_ENCODE(1) |
                          (BOARD_DIRECTION_ENCODE(1) << 2U)) & 0xFU;
    boardirq_inject(0x08080000U, 2U, 2U);
    boardirq_inject(0x04040000U, 0U, 0U);
    boardirq_check((0 == boardirq_delta(BOARD_LEFT)) &&
                   (0 == boardirq_delta(BOARD_RIGHT)) &&
                   (0U == g_encoder_errors),
                   "quadrature preserves measured phase, ignores drive sign");
}

/** @brief Each 6-bit delta holds +/-31 counts per read. One more count
 * keeps the delta at the limit, latches that wheel's bit and still
 * advances the total; the code 0x20 (-32) is never stored.
 */
static void boardirq_delta_saturation (void)
{
    int32_t left = 0;
    int32_t right = 0;
    uint32_t errors = 0U;
    uint32_t count = 0U;
    uint16_t left_total = 0U;
    uint16_t right_total = 0U;

    boardirq_reset();
    for (count = 0U; count < 31U; ++count)
    {
        boardirq_step(BOARD_LEFT, 1);
        boardirq_step(BOARD_RIGHT, -1);
    }
    boardirq_check((31 == boardirq_delta(BOARD_LEFT)) &&
                   (-31 == boardirq_delta(BOARD_RIGHT)) &&
                   (0U == g_encoder_errors),
                   "31 counts per read fit the 6-bit deltas without error");
    boardirq_step(BOARD_LEFT, 1);
    boardirq_check((31 == boardirq_delta(BOARD_LEFT)) &&
                   (BOARD_ENCODER_ERROR_LEFT == g_encoder_errors),
                   "a 32nd forward count saturates at +31, left bit set");
    boardirq_step(BOARD_RIGHT, -1);
    boardirq_check((-31 == boardirq_delta(BOARD_RIGHT)) &&
                   (0x21U == ((g_encoder_word >> 6U) & BOARD_DELTA_MASK)) &&
                   (BOARD_ENCODER_ERRORS == g_encoder_errors),
                   "a 32nd reverse count saturates at -31, right bit set");
    board_encoder_totals(&left_total, &right_total);
    boardirq_check((32U == left_total) && (0xFFE0U == right_total),
                   "totals keep counting through delta saturation");
    board_read_encoders(&left, &right, &errors);
    boardirq_check((31 == left) && (-31 == right) &&
                   (BOARD_ENCODER_ERRORS == errors),
                   "the read reports saturated deltas and both bits");
    board_read_encoders(&left, &right, &errors);
    boardirq_check((0 == left) && (0 == right) && (0U == errors),
                   "the read consumed the deltas and errors once");
}

/** @brief Read-and-clear keeps both phases, so an edge right after a read
 * decodes from them. From a cleared phase (0) these edges would be errors.
 */
static void boardirq_read_keeps_phase (void)
{
    int32_t left = 0;
    int32_t right = 0;
    uint32_t errors = 0U;

    boardirq_reset();
    boardirq_step(BOARD_LEFT, 1);
    boardirq_step(BOARD_LEFT, 1);
    boardirq_step(BOARD_LEFT, 1);
    boardirq_step(BOARD_RIGHT, 1);
    boardirq_step(BOARD_RIGHT, 1);
    board_read_encoders(&left, &right, &errors);
    boardirq_check((3 == left) && (2 == right) && (0U == errors) &&
                   (((2U << BOARD_PHASE_SHIFT) |
                     (3U << (BOARD_PHASE_SHIFT + 2U))) == g_encoder_word),
                   "read-and-clear zeroes deltas and keeps both phases");
    boardirq_step(BOARD_LEFT, 1);
    boardirq_step(BOARD_RIGHT, 1);
    board_read_encoders(&left, &right, &errors);
    boardirq_check((1 == left) && (1 == right) && (0U == errors),
                   "an edge right after a read decodes from the kept phase");
}

/** @brief Totals are two independent modulo-2^16 halves of one word: no
 * carry or borrow crosses between them, and the consumer's modulo
 * difference with a 0x8000 half-range spans the wrap exactly.
 */
static void boardirq_totals_wrap (void)
{
    uint16_t left_total = 0U;
    uint16_t right_total = 0U;
    uint16_t start = 0U;
    uint32_t count = 0U;

    boardirq_reset();
    g_encoder_totals = 0x0000FFFFU;
    boardirq_step(BOARD_LEFT, 1);
    board_encoder_totals(&left_total, &right_total);
    boardirq_check((0U == left_total) && (0U == right_total) &&
                   (0U == g_encoder_totals),
                   "left 0xFFFF -> 0 does not carry into the right half");
    boardirq_step(BOARD_LEFT, -1);
    board_encoder_totals(&left_total, &right_total);
    boardirq_check((0xFFFFU == left_total) && (0U == right_total),
                   "left 0 -> 0xFFFF does not borrow from the right half");
    boardirq_step(BOARD_RIGHT, -1);
    board_encoder_totals(&left_total, &right_total);
    boardirq_check((0xFFFFU == left_total) && (0xFFFFU == right_total),
                   "right 0 -> 0xFFFF leaves the left half unchanged");
    boardirq_step(BOARD_RIGHT, 1);
    board_encoder_totals(&left_total, &right_total);
    boardirq_check((0xFFFFU == left_total) && (0U == right_total),
                   "right 0xFFFF -> 0 leaves the left half unchanged");
    boardirq_check(0U == boardirq_read_errors(), "wrapping is not an error");
    g_encoder_totals = 0xFFF0FFF0U;
    board_encoder_totals(&start, &right_total);
    for (count = 0U; count < 31U; ++count)
    {
        boardirq_step(BOARD_LEFT, 1);
    }
    board_encoder_totals(&left_total, &right_total);
    /* The uint16_t cast is the consumer's modulo-2^16 difference. */
    boardirq_check((0x000FU == left_total) &&
                   (31U == (uint32_t)(uint16_t)(left_total - start)),
                   "a modulo-2^16 difference across 0xFFFF -> 0 is exact");
}

/** @brief BOOT WINDOW, both wheels: an ambiguous batch on each (input
 * settling), then 41 forward left and 42 reverse right counts arrive
 * between board_init() and the first Motion read, so both deltas saturate
 * and both sticky bits latch. The discard drops both deltas and both bits
 * in one region and keeps the non-zero phases (left 1, right 3) and both
 * totals, so the first read reports only post-discard counts, no error.
 */
static void boardirq_boot_discard (void)
{
    int32_t left = 0;
    int32_t right = 0;
    uint32_t errors = 0U;
    uint32_t count = 0U;
    uint32_t regions = 0U;
    uint16_t left_total = 0U;
    uint16_t right_total = 0U;

    boardirq_reset();
    boardirq_inject(0xCCCC0000U, 0U, 0U);
    boardirq_check((BOARD_ENCODER_ERRORS == g_encoder_errors) &&
                   (0U == g_encoder_word) && (0U == g_encoder_totals),
                   "boot-window settling latches both bits, counts nothing");
    for (count = 0U; count < 41U; ++count)
    {
        boardirq_step(BOARD_LEFT, 1);
        boardirq_step(BOARD_RIGHT, -1);
    }
    boardirq_step(BOARD_RIGHT, -1);
    board_encoder_totals(&left_total, &right_total);
    boardirq_check((31 == boardirq_delta(BOARD_LEFT)) &&
                   (-31 == boardirq_delta(BOARD_RIGHT)) &&
                   (41U == left_total) && (0xFFD6U == right_total),
                   "boot-window counts saturate both deltas, totals count");
    regions = g_test_regions;
    board_encoders_discard();
    boardirq_check(((regions + 1U) == g_test_regions) &&
                   (((1U << BOARD_PHASE_SHIFT) |
                     (3U << (BOARD_PHASE_SHIFT + 2U))) == g_encoder_word) &&
                   (0U == g_encoder_errors),
                   "discard clears both deltas and bits, keeps both phases");
    board_encoder_totals(&left_total, &right_total);
    boardirq_check((41U == left_total) && (0xFFD6U == right_total),
                   "discard keeps both position totals");
    boardirq_step(BOARD_LEFT, 1);
    boardirq_step(BOARD_LEFT, 1);
    boardirq_step(BOARD_RIGHT, -1);
    board_read_encoders(&left, &right, &errors);
    board_encoder_totals(&left_total, &right_total);
    boardirq_check((2 == left) && (-1 == right) && (0U == errors) &&
                   (43U == left_total) && (0xFFD5U == right_total),
                   "first read after discard: post-discard counts only");
}

/** @brief An edge raised while the read-and-clear region is open is
 * latched and taken after the unlock: excluded from that read, never
 * cleared unread and returned once by the next read. A split read/clear
 * would run it in between and lose it; an unlocked one opens no region.
 * Likewise an edge latched during the discard survives it, while the
 * deltas counted before it on both wheels are dropped.
 */
static void boardirq_interleave (void)
{
    int32_t left = 0;
    int32_t right = 0;
    uint32_t errors = 0U;
    uint32_t regions = 0U;
    uint32_t pending = 0U;
    uint16_t left_before = 0U;
    uint16_t right_before = 0U;
    uint16_t left_total = 0U;
    uint16_t right_total = 0U;

    boardirq_reset();
    boardirq_step(BOARD_LEFT, 1);
    boardirq_step(BOARD_LEFT, 1);
    boardirq_step(BOARD_RIGHT, -1);
    pending = boardirq_transition(BOARD_LEFT, 1);
    boardirq_arm(pending, g_test_phase[BOARD_LEFT],
                 g_test_phase[BOARD_RIGHT]);
    regions = g_test_regions;
    board_read_encoders(&left, &right, &errors);
    boardirq_check((regions + 1U) == g_test_regions,
                   "read and clear share one masked region");
    boardirq_check((2 == left) && (-1 == right) && (0U == errors),
                   "an edge latched in the region is excluded from it");
    boardirq_check((!gb_test_arm) && (!gb_test_latched) &&
                   (1 == boardirq_delta(BOARD_LEFT)) &&
                   (0 == boardirq_delta(BOARD_RIGHT)),
                   "the latched edge runs after unlock, after the clear");
    board_read_encoders(&left, &right, &errors);
    boardirq_check((1 == left) && (0 == right) && (0U == errors),
                   "the next read returns the latched edge exactly once");
    boardirq_step(BOARD_LEFT, 1);
    boardirq_step(BOARD_RIGHT, -1);
    boardirq_step(BOARD_RIGHT, -1);
    board_encoder_totals(&left_before, &right_before);
    pending = boardirq_transition(BOARD_RIGHT, 1);
    boardirq_arm(pending, g_test_phase[BOARD_LEFT],
                 g_test_phase[BOARD_RIGHT]);
    regions = g_test_regions;
    board_encoders_discard();
    boardirq_check(((regions + 1U) == g_test_regions) &&
                   (0 == boardirq_delta(BOARD_LEFT)) &&
                   (1 == boardirq_delta(BOARD_RIGHT)),
                   "an edge latched during the discard is kept, older not");
    board_encoder_totals(&left_total, &right_total);
    /* The uint16_t cast is the modulo-2^16 total after one more count. */
    boardirq_check((left_before == left_total) &&
                   ((uint16_t)(right_before + 1U) == right_total),
                   "the discard keeps both totals; the latched edge counts");
}

/** @brief Tribunal interleaving test: 10000 valid edges on both wheels at
 * pseudo-random points, with reads in between, some taking an edge inside
 * their region. No count is lost or doubled, no false error appears and
 * the totals equal the net counts across the 0xFFFF -> 0 wrap.
 */
static void boardirq_stress (void)
{
    uint32_t seed = 2026U;
    uint32_t step = 0U;
    uint32_t wheel = 0U;
    uint32_t pending = 0U;
    int32_t direction = 0;
    int32_t taken[2] = {0, 0};
    int32_t unread[2] = {0, 0};
    int32_t net[2] = {0, 0};
    uint32_t errors = 0U;
    uint32_t bad_reads = 0U;
    uint16_t left_total = 0U;
    uint16_t right_total = 0U;

    boardirq_reset();
    g_encoder_totals = 0xFF00FF00U;
    for (step = 0U; step < BOARDIRQ_STRESS_STEPS; ++step)
    {
        seed = (seed * 1103515245U) + 12345U;
        wheel = (seed >> 16U) & 1U;
        direction = (0U != ((seed >> 17U) & 3U)) ? 1 : -1;
        if ((0U == ((seed >> 20U) % 5U)) ||
            (unread[wheel] >= BOARDIRQ_STRESS_LIMIT) ||
            (unread[wheel] <= -BOARDIRQ_STRESS_LIMIT))
        {
            pending = boardirq_transition(wheel, direction);
            boardirq_arm(pending, g_test_phase[BOARD_LEFT],
                         g_test_phase[BOARD_RIGHT]);
            board_read_encoders(&taken[BOARD_LEFT], &taken[BOARD_RIGHT],
                                &errors);
            if ((taken[0] != unread[0]) || (taken[1] != unread[1]) ||
                (0U != errors))
            {
                ++bad_reads;
            }
            unread[0] = 0;
            unread[1] = 0;
        }
        else
        {
            boardirq_step(wheel, direction);
        }
        unread[wheel] += direction;
        net[wheel] += direction;
    }
    board_read_encoders(&taken[BOARD_LEFT], &taken[BOARD_RIGHT], &errors);
    boardirq_check((0U == bad_reads) && (taken[0] == unread[0]) &&
                   (taken[1] == unread[1]) && (0U == errors),
                   "interleaved reads lose, double and invent no count");
    board_encoder_totals(&left_total, &right_total);
    /* Both casts reduce modulo 2^16, as the ISR's wrapping totals do. */
    boardirq_check(((uint16_t)(0xFF00U + (uint32_t)net[0]) == left_total) &&
                   ((uint16_t)(0xFF00U + (uint32_t)net[1]) == right_total) &&
                   (net[0] > 256) && (net[1] > 256),
                   "totals equal the net counts across the wrap");
    boardirq_check(((g_test_phase[BOARD_LEFT] << BOARD_PHASE_SHIFT) |
                    (g_test_phase[BOARD_RIGHT] <<
                     (BOARD_PHASE_SHIFT + 2U))) == g_encoder_word,
                   "phases track the inputs through every clear");
}
#endif

/* End of file. */

