/** @file robot_imu.h
 * @brief The robot app: subsystem 4: the IMU task (accelerometer, terrain,
 * tilt and collision, its evidence); the calls the other parts make into
 * robot_imu.c. Include robot_app_private.h first.
 */
#ifndef ROBOT_IMU_H
#define ROBOT_IMU_H
void robot_app_terrain_run (const robot_app_shared_t * p_copy,
                                   uint16_t path_hc, uint32_t now_ms);
void robot_app_imu_missed (board_accel_status_t status,
                                  const int16_t raw[3], uint32_t read_us,
                                  vehicle_state_t state, uint32_t now_ms);
void robot_app_imu_judge (bool b_valid, bool b_tilt_valid,
                                 double dt_s, vehicle_state_t state);
void robot_app_imu_note (robot_app_imu_event_t event,
                                uint32_t value);
uint8_t robot_app_count8 (uint32_t count);
bool robot_app_imu_path (const robot_app_shared_t * p_copy,
                                uint32_t now_ms, uint16_t * p_path_hc);
void robot_app_imu_publish (const terrain_output_t * p_output,
                                   const robot_app_shared_t * p_copy,
                                   bool b_valid, uint32_t now_ms);

#endif /* ROBOT_IMU_H */

/* End of file. */

