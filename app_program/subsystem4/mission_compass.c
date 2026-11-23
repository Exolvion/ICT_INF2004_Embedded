/** @file mission_compass.c
 * @brief The mission: subsystem 4: the compass (LSM303DLHC magnetometer).
 * Split out of the one file by tools/reorg/split_c.py (22 Nov 2026); what
 * the parts share is in mission_private.h.
 */
#include "mission_private.h"
#include "mission_compass.h"

/* Compass trial (3 Nov), report only: the LSM303DLHC's magnetometer (I2C
 * 0x1E, beside the accelerometer; datasheet registers) is switched on and
 * its raw X Y Z counts logged: MAG once a second while idle (turn the car
 * round by hand for the motor magnets' offset), " C x y z" after each
 * MISSION pose line, and on each turn check. Nothing steers by it. Device
 * builds only: the host simulations read none.
 */
#define BENCH_MAG_ADDRESS (0x1EU)
#define BENCH_MAG_CRA_REG (0x00U)   /* CRA_REG_M */
#define BENCH_MAG_CRA_30HZ (0x14U)  /* 30 Hz output */
#define BENCH_MAG_CRB_REG (0x01U)   /* CRB_REG_M */
#define BENCH_MAG_CRB_1G3 (0x20U)   /* +-1.3 gauss, 1100 counts/gauss X Y */
#define BENCH_MAG_MR_REG (0x02U)    /* MR_REG_M */
#define BENCH_MAG_MR_CONT (0x00U)   /* continuous conversion */
#define BENCH_MAG_OUT_REG (0x03U)   /* OUT_X_H_M: X, Z, Y, high byte first */
#define BENCH_MAG_OVERFLOW (-4096)  /* an axis out of range */
#define BENCH_MAG_SETUP_REGS (3U)   /* CRA, CRB and MR, written once */
#if ((ROBOT_PIN_IMU_SDA % 4U) == 0U) /* board.c's controller for the pins */
#define BENCH_MAG_I2C i2c0
#endif
#if !(((ROBOT_PIN_IMU_SDA % 4U) == 0U) /* board.c's controller for the pins */)
#define BENCH_MAG_I2C i2c1
#endif

/** @brief Compass trial: "; C x y z" (raw counts) onto the text being
 * built, or nothing when there is no reading.
 */
void bench_mag_say (void)
{
    int16_t  xyz[3] = {0, 0, 0};
    uint32_t axis;

    if (!bench_flag(bench_mag_read(xyz)))
    {
        return;
    }
    bench_text("; C");
    for (axis = 0U; axis < ROBOT_IMU_AXES; ++axis)
    {
        bench_text(" ");
        bench_s((int32_t)xyz[axis], 13U);
    }
}

/** @brief Compass trial: the LSM303DLHC magnetometer's raw X, Y, Z counts
 * (sensor axes) into p_xyz, over I2C. The first call switches it on (30 Hz,
 * +-1.3 gauss, continuous) and reads nothing. False when it is off or
 * missing, an axis is out of range, or on the host simulations (no I2C).
 * The robot app calls it from the IMU task only (mission_mag_poll).
 */
static bool bench_mag_bus (int16_t * p_xyz)
{
#if defined(PICO_ON_DEVICE) && (1 == PICO_ON_DEVICE)
    static const uint8_t setup[BENCH_MAG_SETUP_REGS][2] =
    {
        {BENCH_MAG_CRA_REG, BENCH_MAG_CRA_30HZ},
        {BENCH_MAG_CRB_REG, BENCH_MAG_CRB_1G3},
        {BENCH_MAG_MR_REG, BENCH_MAG_MR_CONT}
    };
    static bool b_tried = false; /* switched on, or tried */
    static bool b_ready = false; /* and it answered */
    uint8_t  reg = BENCH_MAG_OUT_REG;
    uint8_t  data[6] = {0U, 0U, 0U, 0U, 0U, 0U};
    uint32_t reg_index;
    int16_t  x;
    int16_t  y;
    int16_t  z;

    if (!b_tried)
    {
        b_tried = true;
        b_ready = true;
        for (reg_index = 0U; reg_index < BENCH_MAG_SETUP_REGS; ++reg_index)
        {
            if (2 != i2c_write_timeout_us(BENCH_MAG_I2C, BENCH_MAG_ADDRESS,
                                          setup[reg_index], 2U, false,
                                          ROBOT_I2C_TIMEOUT_US))
            {
                b_ready = false;
            }
        }
        return false;
    }
    if ((!b_ready) ||
        (1 != i2c_write_timeout_us(BENCH_MAG_I2C, BENCH_MAG_ADDRESS, &reg, 1U,
                                   true, ROBOT_I2C_TIMEOUT_US)) ||
        (6 != i2c_read_timeout_us(BENCH_MAG_I2C, BENCH_MAG_ADDRESS, data, 6U,
                                  false, ROBOT_I2C_TIMEOUT_US)))
    {
        return false;
    }
    x = (int16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
    z = (int16_t)(((uint16_t)data[2] << 8U) | (uint16_t)data[3]);
    y = (int16_t)(((uint16_t)data[4] << 8U) | (uint16_t)data[5]);
    if ((BENCH_MAG_OVERFLOW == x) || (BENCH_MAG_OVERFLOW == y) ||
        (BENCH_MAG_OVERFLOW == z))
    {
        return false;
    }
    p_xyz[0] = x;
    p_xyz[1] = y;
    p_xyz[2] = z;
    return true;
#else
    (void)p_xyz;
    return false;
#endif
}

/* The IMU task's last compass reading: raw counts (16-bit words), valid. */
typedef struct
{
    uint32_t x       : 16;
    uint32_t y       : 16;
    uint32_t z       : 16;
    bool     b_valid : 1;
} mission_mag_t;

static mission_mag_t g_mag;

/** @brief IMU task (mission.h): one compass read, kept for the mission. */
void mission_mag_poll (void)
{
    int16_t  xyz[3] = {0, 0, 0};
    bool     b_valid = bench_mag_bus(xyz);
    uint32_t saved = app_main_lock();

    /* The fields hold the 16-bit two's complement words. */
    g_mag.x = (uint16_t)xyz[0];
    g_mag.y = (uint16_t)xyz[1];
    g_mag.z = (uint16_t)xyz[2];
    g_mag.b_valid = b_valid;
    app_main_unlock(saved);
}

/** @brief The compass reading the IMU task last took (mission_mag_poll),
 * at most 100 ms old. False if that read failed or none came yet.
 */
bool bench_mag_read (int16_t * p_xyz)
{
    mission_mag_t sample;
    uint32_t      saved = app_main_lock();

    sample = g_mag;
    app_main_unlock(saved);
    if (sample.b_valid)
    {
        p_xyz[0] = (int16_t)(uint16_t)sample.x;
        p_xyz[1] = (int16_t)(uint16_t)sample.y;
        p_xyz[2] = (int16_t)(uint16_t)sample.z;
    }
    return sample.b_valid;
}

/* End of file. */

