/** @file robot_line.h
 * @brief The robot app: subsystem 3: the Line task (IR samples, line
 * control); the calls the other parts make into robot_line.c. Include
 * robot_app_private.h first.
 */
#ifndef ROBOT_LINE_H
#define ROBOT_LINE_H
#if !ROBOT_APP_MISSION
bool robot_app_optical_path (void);
#endif
bool robot_app_codebook_ready (void);
#if !(ROBOT_APP_MISSION)
vehicle_nav_t robot_app_nav (barcode_action_t action);
bool robot_app_original_line (const robot_app_shared_t * p_copy,
                                     uint32_t now_ms);
#endif
#if ROBOT_APP_MISSION

/** @brief Line task step: read the IR pair, run the line controller,
 * publish its status, steering and health, and pass the sample on to the
 * mission.
 */
void robot_app_line_step (uint32_t now_ms);
#endif
#if !(ROBOT_APP_MISSION)

/** @brief Line task step: read the IR pair, run the line controller and
 * barcode decoder, and publish the line status, steering and health.
 */
void robot_app_line_step (uint32_t now_ms);
#endif

#endif /* ROBOT_LINE_H */

/* End of file. */

