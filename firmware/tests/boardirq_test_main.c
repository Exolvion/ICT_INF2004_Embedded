/**
 * @file boardirq_test_main.c
 * @brief Host-only ISR and accelerometer checks with real SDK types.
 * @details MMIO/I2C are doubles; no timing, ARM or physical sensor is tested.
 * Including the implementation exercises its private registered ISR without
 * adding a production test hook. Unused board functions are link-discarded.
 */
#include "boardirq_test_main.h"

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

static int boardirq_i2c_write (i2c_inst_t * p_i2c, uint8_t address,
    const uint8_t * p_source, size_t length, bool b_nostop,
    uint timeout_us);
static int boardirq_i2c_read (i2c_inst_t * p_i2c, uint8_t address,
    uint8_t * p_destination, size_t length, bool b_nostop,
    uint timeout_us);
static void boardirq_gpio_put (uint32_t pin, bool b_value);
static void boardirq_timer_step (void);

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

#include "board.c"

static void boardirq_check (bool b_pass, const char * p_name);
static void boardirq_reset (void);
static void boardirq_accel (void);
static uint32_t boardirq_bank_bits (uint32_t nibbles, uint32_t pin_a,
                                    uint32_t pin_b);
static void boardirq_inject (uint32_t pending, uint32_t left_phase,
                             uint32_t right_phase);
static void boardirq_inject_echo (uint32_t edge_bits, bool b_level_high,
                                  uint32_t time_us);
static void boardirq_echo (void);
static void boardirq_trigger (void);
#if ROBOT_ENCODER_CHANNELS == 2
static void boardirq_normal_cycles (void);
static void boardirq_coalesced (void);
static void boardirq_inconsistent_sample (void);
static void boardirq_resynchronization (void);
static void boardirq_saturation (void);
static void boardirq_constant_b_quadrature (void);
#else
static void boardirq_single_channel (void);
#endif

int main (void)
{
    boardirq_accel();
    boardirq_echo();
    boardirq_trigger();
#if ROBOT_ENCODER_CHANNELS == 2
    boardirq_normal_cycles();
    boardirq_coalesced();
    boardirq_inconsistent_sample();
    boardirq_resynchronization();
    boardirq_saturation();
    boardirq_constant_b_quadrature();
#else
    boardirq_single_channel();
#endif
    printf("board ISR/accel host doubles: %u checks, %u failures\n",
           (unsigned int)g_test_checks, (unsigned int)g_test_failures);
    return (0U == g_test_failures) ? 0 : 1;
}

/** Lock stubs serve only single-threaded public encoder snapshot calls. */
uint32_t app_main_lock (void)
{
    return 0U;
}

void app_main_unlock (uint32_t saved_mask)
{
    (void)saved_mask;
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
    boardirq_check(BOARD_ECHO_WAIT_RISE == g_echo_state,
                   "trigger arms the echo measurement");
    gb_initialized = false;
    gp_test_timer = NULL;
    boardirq_reset();
}

/** Preserve expected and observed failures without aborting later cases. */
static void boardirq_check (bool b_pass, const char * p_name)
{
    g_test_checks++;
    if (!b_pass)
    {
        g_test_failures++;
        printf("FAIL: %s\n", p_name);
    }
}

/** Reset only ISR-owned state, leaving actual commissioning flags unchanged. */
static void boardirq_reset (void)
{
    g_left_delta = 0;
    g_right_delta = 0;
    g_left_direction = 0;
    g_right_direction = 0;
    g_left_total = 0U;
    g_right_total = 0U;
    g_left_phase = 0U;
    g_right_phase = 0U;
    g_edge_errors = 0U;
    g_echo_state = BOARD_ECHO_IDLE;
}

/** Place one wheel's A/B status nibbles at the configured pins' offsets. */
static uint32_t boardirq_bank_bits (uint32_t nibbles, uint32_t pin_a,
                                    uint32_t pin_b)
{
    return ((nibbles & 0xFU) << BOARD_IRQ_SHIFT(pin_a)) |
           (((nibbles >> 4U) & 0xFU) << BOARD_IRQ_SHIFT(pin_b));
}

/** Real SDK read-only fields are initialized, never cast away and written.
 * Test vectors keep one layout: left wheel A/B nibbles in bits 16..23 and
 * right wheel A/B nibbles in bits 24..31. They are relocated here to the
 * IO_BANK0 banks and bit offsets of the configured encoder pins.
 */
static void boardirq_inject (uint32_t pending, uint32_t left_phase,
                             uint32_t right_phase)
{
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
                   "ISR acknowledges only owned encoder edges");
}

/** Drive the configured ECHO pin's latched edges through the real ISR. */
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

/** Echo timing follows the configured pin through its IO_BANK0 bank. */
static void boardirq_echo (void)
{
    boardirq_reset();
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_RISE, true, 1000U);
    boardirq_check(BOARD_ECHO_IDLE == g_echo_state,
                   "an ECHO edge without a pending trigger is ignored");
    g_echo_state = BOARD_ECHO_WAIT_RISE;
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_RISE, true, 1000U);
    boardirq_check(BOARD_ECHO_WAIT_FALL == g_echo_state,
                   "rising ECHO edge starts the pulse measurement");
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_FALL, false, 1580U);
    boardirq_check((BOARD_ECHO_READY == g_echo_state) &&
                   (580U == g_echo_pulse_us),
                   "falling ECHO edge completes the 580 us pulse");
    g_echo_state = BOARD_ECHO_WAIT_RISE;
    boardirq_inject_echo((uint32_t)GPIO_IRQ_EDGE_RISE |
                         (uint32_t)GPIO_IRQ_EDGE_FALL, false, 2000U);
    boardirq_check(BOARD_ECHO_TIMEOUT == g_echo_state,
                   "coalesced ECHO edges cannot yield a pulse width");
    boardirq_reset();
}

#if ROBOT_ENCODER_CHANNELS == 1
/** Constant B counts only in explicitly confirmed single-channel mode. */
static void boardirq_single_channel (void)
{
    boardirq_reset();
    g_left_direction = 1;
    g_right_direction = -1;
    boardirq_inject(0x08080000U, 2U, 2U);
    boardirq_inject(0x04040000U, 0U, 0U);
#if ROBOT_SINGLE_CHANNEL_DIRECTION_CONFIRMED
    boardirq_check((2 == g_left_delta) && (-2 == g_right_delta) &&
                   (2U == g_left_total) &&
                   ((UINT32_MAX - 1U) == g_right_total) &&
                   (0U == g_edge_errors),
                   "constant B: both A edges count with applied direction");
    boardirq_inject(0x80800000U, 1U, 1U);
    boardirq_check((2 == g_left_delta) && (-2 == g_right_delta) &&
                   (0U == g_edge_errors), "B-only edges ignored");
    boardirq_reset();
    g_right_direction = 1;
    boardirq_inject(0x08080000U, 2U, 2U);
    boardirq_check((0 == g_left_delta) && (0U == g_left_total) &&
                   (1 == g_right_delta) && (1U == g_edge_errors),
                   "unknown/braked direction cannot manufacture displacement");
    boardirq_reset();
    g_left_direction = 1;
    boardirq_inject(0x000C0000U, 2U, 0U);
    boardirq_check((0 == g_left_delta) && (1U == g_edge_errors),
                   "coalesced A edges rejected without a guessed count");
    boardirq_inject(0x00080000U, 0U, 0U);
    boardirq_check((0 == g_left_delta) && (2U == g_edge_errors),
                   "inconsistent A polarity rejected and resynchronized");
    boardirq_inject(0x00080000U, 2U, 0U);
    boardirq_check((1 == g_left_delta) && (2U == g_edge_errors),
                   "next clean A edge resumes after resynchronization");
    g_left_delta = INT32_MAX;
    boardirq_inject(0x00040000U, 0U, 0U);
    boardirq_check((INT32_MAX == g_left_delta) && (3U == g_edge_errors),
                   "single-channel delta overflow saturates and reports error");
#else
    boardirq_check((0 == g_left_delta) && (0 == g_right_delta) &&
                   (0U == g_left_total) && (0U == g_right_total) &&
                   (4U == g_edge_errors),
                   "unconfirmed inferred direction never yields counts");
#endif
}
#endif

#if ROBOT_ENCODER_CHANNELS == 2
/** Both wheels retain x4 direction and non-consuming modulo totals. */
static void boardirq_normal_cycles (void)
{
    int32_t left = 0;
    int32_t right = 0;
    uint32_t errors = 0U;
    uint32_t left_total = 0U;
    uint32_t right_total = 0U;

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

/** Pending bits are latches, so their ordering and edge count are unknown. */
static void boardirq_coalesced (void)
{
    boardirq_reset();
    boardirq_inject(0x00CC0000U, 0U, 0U);
    boardirq_check((1U == g_edge_errors) && (0 == g_left_delta) &&
                   (0U == g_left_total),
                   "coalesced full left cycle reports error, no displacement");
    boardirq_inject(0xCC000000U, 0U, 0U);
    boardirq_check((2U == g_edge_errors) && (0 == g_right_delta) &&
                   (0U == g_right_total),
                   "coalesced full right cycle reports error");
    boardirq_inject(0x000C0000U, 2U, 0U);
    boardirq_check((3U == g_edge_errors) && (0 == g_left_delta) &&
                   (0U == g_left_total),
                   "both edges of one pin cannot masquerade as one count");
    boardirq_reset();
    boardirq_inject(0x00880000U, 3U, 0U);
    boardirq_check((1U == g_edge_errors) && (0 == g_left_delta),
                   "two channel transitions remain an ambiguous jump");
}

/** A status/sample race must not be silently accepted as zero or one edge. */
static void boardirq_inconsistent_sample (void)
{
    boardirq_reset();
    boardirq_inject(0x00080000U, 0U, 0U);
    boardirq_check((1U == g_edge_errors) && (0 == g_left_delta),
                   "pending edge with unchanged phase reports lost evidence");
    boardirq_inject(0x00080000U, 1U, 0U);
    boardirq_check((2U == g_edge_errors) && (0 == g_left_delta),
                   "pending A edge cannot authorize a B phase transition");
    boardirq_reset();
    boardirq_inject(0x04000000U, 0U, 2U);
    boardirq_check((1U == g_edge_errors) && (0 == g_right_delta),
                   "pending fall cannot authorize a sampled rising edge");
}

/** A bad wheel must not poison the other wheel or retain an obsolete phase. */
static void boardirq_resynchronization (void)
{
    boardirq_reset();
    boardirq_inject(0x800C0000U, 2U, 1U);
    boardirq_check((1U == g_edge_errors) && (0 == g_left_delta) &&
                   (1 == g_right_delta),
                   "ambiguous left wheel does not discard a valid right edge");
    boardirq_check(2U == g_left_phase,
                   "ambiguous acquisition resynchronizes observed phase");
    boardirq_inject(0x00040000U, 0U, 1U);
    boardirq_check((1U == g_edge_errors) && (1 == g_left_delta) &&
                   (1U == g_left_total),
                   "next clean edge decodes from resynchronized phase");
}

/** Persistent overload cannot wrap the diagnostic counter back to zero. */
static void boardirq_saturation (void)
{
    int32_t left = 0;
    int32_t right = 0;
    uint32_t errors = 0U;

    boardirq_reset();
    g_edge_errors = UINT32_MAX;
    boardirq_inject(0xCCCC0000U, 0U, 0U);
    board_read_encoders(&left, &right, &errors);
    boardirq_check(UINT32_MAX == errors,
                   "ambiguous-edge diagnostic saturates");
    board_read_encoders(&left, &right, &errors);
    boardirq_check(0U == errors,
                   "motion owner consumes the error report once");
}

/** Constant B cannot supply measured direction in quadrature mode. */
static void boardirq_constant_b_quadrature (void)
{
    boardirq_reset();
    g_left_direction = 1;
    g_right_direction = 1;
    boardirq_inject(0x08080000U, 2U, 2U);
    boardirq_inject(0x04040000U, 0U, 0U);
    boardirq_check((0 == g_left_delta) && (0 == g_right_delta) &&
                   (0U == g_edge_errors),
                   "quadrature preserves measured phase, ignores drive sign");
}
#endif

/* End of file. */

