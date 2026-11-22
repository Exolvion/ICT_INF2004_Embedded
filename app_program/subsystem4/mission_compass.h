/** @file mission_compass.h
 * @brief The mission: subsystem 4: the compass (LSM303DLHC magnetometer);
 * the calls the other parts make into mission_compass.c. Include
 * mission_private.h first.
 */
#ifndef MISSION_COMPASS_H
#define MISSION_COMPASS_H
bool bench_mag_read (int16_t * p_xyz);
void bench_mag_say (void);

#endif /* MISSION_COMPASS_H */

/* End of file. */

