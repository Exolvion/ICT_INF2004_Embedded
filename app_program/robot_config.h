/**
 * @file robot_config.h
 * @brief Explicit, unconfirmed physical configuration for the supplied kit.
 * @details Values below are design hypotheses, not measured calibration.
 * Record evidence in the validation log before changing confirmation gates.
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Independent gates prevent guessed calibration from energising actuators.
 * CALIBRATION 1 (22 Nov 2026): the wheel, IR and accelerometer values below
 * drove the car on the track for the whole bench MISSION campaign (6-22 Nov,
 * testing/system/logs/2026-11-*_dashboard.txt; the bench image opened this
 * gate in its own copy). The robot app now runs that mission.
 */
#define ROBOT_CALIBRATION_CONFIRMED 1
#define ROBOT_CALIBRATION_VERSION 2U
#define ROBOT_SERVO_CONFIRMED 1
#define ROBOT_BARCODE_CONFIRMED 0
#define ROBOT_WIRING_CONFIRMED 1

/* 2 = x4 quadrature; 1 = both A edges, B ignored. Verify the fitted sensor.
 * A single-channel sensor cannot measure direction. The optional mode uses
 * applied motor direction and must be explicitly accepted after coast and
 * reversal trials. Edges while braked have unknown sign and are rejected.
 */
#define ROBOT_ENCODER_CHANNELS 2
#define ROBOT_SINGLE_CHANNEL_DIRECTION_CONFIRMED 0
#if (ROBOT_ENCODER_CHANNELS != 1) && (ROBOT_ENCODER_CHANNELS != 2)
#error "ROBOT_ENCODER_CHANNELS must be 1 or 2"
#endif

/* Pinned BSP clock; checked against BSP CPU clock configuration at install.
 * ROBOT_KERNEL_TICK_MS is documentation only and is not read by firmware:
 * the installer sets CNF_TIMER_PERIOD to 1 itself. ROBOT_PWM_HZ is read only
 * by the compile-time check in board.c; the PWM uses ROBOT_PWM_WRAP.
 */
#define ROBOT_CLOCK_HZ 125000000U
#define ROBOT_KERNEL_TICK_MS 1U
#define ROBOT_PWM_HZ 20000U
#define ROBOT_PWM_WRAP 6249U
#define ROBOT_SERVO_FRAME_US 20000U
/* Scan servo (S5-U06, 2026-10-27, bench image and top-view photos): 1780 us
 * points the sonar straight ahead and a larger pulse turns it left, about
 * 11.4 us per degree to the left and 9.7 us per degree to the right. The
 * sonar's board meets the Robo Pico's front Grove connectors at about 50
 * degrees either way, so the scan covers +/-ROBOT_SCAN_HALF_ANGLE_DEG and
 * MIN_US / MAX_US are the pulses at -40 (right) and +40 (left) degrees.
 * At the right end the sonar's own cable once pulled it out of its holder:
 * secure the sonar or give its cable more slack.
 */
#define ROBOT_SCAN_HALF_ANGLE_DEG 40
#define ROBOT_SERVO_MIN_US 1390U
#define ROBOT_SERVO_CENTRE_US 1780U
#define ROBOT_SERVO_ANGLE_SIGN 1
#define ROBOT_SERVO_MAX_US 2240U

/* Pico GPIO numbers, not physical header pins. Each device sits on one Robo
 * Pico connector: Grove 1 = HC-SR04 TRIG GP0 / ECHO GP1 (5 V module: the
 * ECHO divider is mandatory), Grove 2 = right wheel encoder A/B, Grove 3 =
 * accelerometer on I2C0 (GP4 SDA, GP5 SCL), Grove 4 = left wheel encoder
 * A/B (as wired on the IS29 car, checked 2026-10-26; swap the two encoder
 * pairs here if the car is rewired), motor driver M1 = GP8/GP9 (left wheel,
 * PWM slice 4) and M2 = GP10/GP11 (right wheel, PWM slice 5): each encoder
 * pair must belong to the wheel of its motor, Grove 5/6/7 = IR
 * left/right/barcode (analogue A0 on ADC0..2,
 * digital D0 on GP6/GP14/GP7) and servo port 1 (GP12) = scan servo. Grove 5
 * and Grove 6 both carry GP26, so the right sensor's D0 is rewired from
 * Grove 6 to GP14 (servo port 3): it was on GP13 (servo port 2) until that
 * header pin broke off on 28 Oct 2026. Line following reads only the
 * analogue outputs. board_init sets the barcode D0 (GP7) as an input with
 * no interrupt; the mission's D0 decoder polls it (board_barcode_d0); the
 * other D0 pins stay inputs from reset.
 * GP15 is kept for servos. GP0/GP1 are also the UART0
 * console pins: the console owns them until board_init() hands them to the
 * sonar, and a start-up refusal returns them to UART0 to print its reason.
 * Kernel fault messages after start-up reach no UART pin: use SWD.
 * TRIG must sit on the console TX pin and ECHO on RX, so the boot console
 * never drives against the sensor's ECHO output; board.c checks this.
 * Wire Grove contacts by silkscreen GPIO label, not pin number.
 * Both encoder inputs of one wheel must share an IO_BANK0 interrupt bank
 * (GPIO 0-7, 8-15, 16-23); board.c checks this.
 */
#define ROBOT_PIN_ENCODER_LEFT_A 16U
#define ROBOT_PIN_ENCODER_LEFT_B 17U
#define ROBOT_PIN_ENCODER_RIGHT_A 2U
#define ROBOT_PIN_ENCODER_RIGHT_B 3U
#define ROBOT_PIN_MOTOR_LEFT_A 8U
#define ROBOT_PIN_MOTOR_LEFT_B 9U
#define ROBOT_PIN_MOTOR_RIGHT_A 10U
#define ROBOT_PIN_MOTOR_RIGHT_B 11U
#define ROBOT_PIN_SERVO 12U
#define ROBOT_PIN_IMU_SDA 4U
#define ROBOT_PIN_IMU_SCL 5U
#define ROBOT_PIN_TRIGGER 0U
#define ROBOT_PIN_ECHO 1U
#define ROBOT_PIN_CONSOLE_TX 0U
#define ROBOT_PIN_CONSOLE_RX 1U
#define ROBOT_PIN_STOP 20U
#define ROBOT_PIN_START 21U
#define ROBOT_PIN_IR_LEFT 26U
#define ROBOT_PIN_IR_RIGHT 27U
#define ROBOT_PIN_IR_BARCODE 28U
/* IR digital D0 outputs: wired; only the barcode one is read (bench). */
#define ROBOT_PIN_IR_LEFT_AUX 6U
#define ROBOT_PIN_IR_RIGHT_AUX 14U
#define ROBOT_PIN_IR_BARCODE_AUX 7U

/* Mechanical hypotheses: replace with measured loaded-wheel parameters.
 * Counts/revolution means decoded edges at the WHEEL shaft: x4 for two
 * channels, BOTH A edges for one channel. No single-channel value is assumed.
 * Counts/revolution measured (S2-U05, 2026-10-27): 10 marked hand turns per
 * wheel, twice each, 2530-2536 x4 edges per turn on both wheels.
 * Wheel diameter 65 mm and track width 130 mm (tyre centres, at the floor)
 * by tape measure (S2-U06, 2026-10-27).
 */
#define ROBOT_WHEEL_RADIUS_M 0.0325
#define ROBOT_TRACK_WIDTH_M 0.130
#if ROBOT_ENCODER_CHANNELS == 2
#define ROBOT_ENCODER_COUNTS_PER_REV 2533U
#else
#define ROBOT_ENCODER_COUNTS_PER_REV 0U
#endif
/* Signs (2026-10-27): pushing the car about 34 cm forward on the floor
 * raised both raw totals (left +4286, right +4080), so both encoder signs
 * are +1 (S2-U04 rerun; the 26 Oct hand-turn result of -1 was wrong). In
 * the S2-U07 duty sweep the left wheel ran backwards at + duty, so
 * LEFT_MOTOR_SIGN is -1; a second sweep with these signs turned both wheels
 * forward together at + duty.
 */
#define ROBOT_LEFT_ENCODER_SIGN 1
#define ROBOT_RIGHT_ENCODER_SIGN 1
#define ROBOT_LEFT_MOTOR_SIGN -1
#define ROBOT_RIGHT_MOTOR_SIGN 1
#define ROBOT_MAX_SPEED_MPS 0.15
#define ROBOT_MAX_DUTY 0.35
/* No acceleration/setpoint ramp exists: wheel targets step directly. The PID
 * integral is held while saturated and bounded by the +/-ROBOT_MAX_DUTY
 * output limits (core.c); there is no separate integral limit.
 */
#define ROBOT_PID_PROPORTIONAL 1.0
#define ROBOT_PID_INTEGRAL 0.3
#define ROBOT_PID_DERIVATIVE 0.0

/* IR (S3-U03 redone 2026-10-28, motion bench image, car still on the
 * floor tiles with the 16-18 mm black tape; 1 s averages): white reads LOW
 * and black HIGH on all three. Redone because the two line sensors were
 * spread to sit over the tape edges and swapped sides. White is all three
 * sensors mid-tile (left 139-217, right 182-264, barcode 109-160); black is
 * the tape under each line sensor alone (left 4025-4089, right 4074-4095)
 * and under the barcode sensor (4017-4095). Grout lines read up to about
 * 1250. The barcode black is tape, not the printed codebook (S3-U04/U05).
 * 27 Oct, old mount and tape: white 178/170/144, black 2800/3281/3950.
 * ROBOT_LINE_SENSOR_SPACING_M and ROBOT_BARCODE_LATERAL_OFFSET_M are
 * geometry records only: documentation, not read by firmware (a barcode
 * coordinate along the side sensor's track, T-56, would need the offset).
 * Measured by ruler 2026-10-29: the barcode sensor 71.5 mm left of the
 * midpoint of the front sensors and 160 mm ahead of the wheel axle; the
 * front sensors 130 mm ahead of it. The spec's bars start 29 mm from the
 * line centre (half the 18 mm line plus the 20 mm gap).
 * Moved 2 Nov 2026 to the car's left side (our measurement): the
 * barcode sensor 80 mm left of the centre line, 70 mm ahead of the axle
 * (60 mm behind the front sensors), about 5 mm above the floor.
 * ROBOT_SENSOR_AXLE_OFFSET_M is the NAVIGATE approach distance (app_cfg.c
 * junction_distance_m). Enter the MEASURED travel from junction-edge
 * detection to the axle over the crossbar centre (about the sensor-to-axle
 * offset + half the line width - the debounce/detection travel), not the
 * raw geometric offset (T-06). 0.060 is the unmeasured offset hypothesis.
 */
#define ROBOT_IR_LEFT_WHITE 190U
#define ROBOT_IR_LEFT_BLACK 4050U
#define ROBOT_IR_RIGHT_WHITE 212U
#define ROBOT_IR_RIGHT_BLACK 4088U
#define ROBOT_IR_BARCODE_WHITE 137U
#define ROBOT_IR_BARCODE_BLACK 4043U
#define ROBOT_LINE_SENSOR_SPACING_M 0.018
#define ROBOT_BARCODE_LATERAL_OFFSET_M 0.080
#define ROBOT_BARCODE_FORWARD_OFFSET_M 0.070
#define ROBOT_LINE_SENSOR_FORWARD_M 0.130
#define ROBOT_SENSOR_AXLE_OFFSET_M 0.060

/* Sensor transport and conservative initial thresholds, still unmeasured.
 * The SDK's I2C timeout is one wall-clock deadline for the whole transfer,
 * so it must outlast the higher-priority tasks that can preempt the IMU
 * task mid-transfer: 2 ms failed a healthy read at START on the car (22 Nov:
 * Vehicle and Motion steps of 2-5 ms; a decode holds 10 ms between yields).
 * Reads then took up to 19.8 ms of wall time in a run (the end report's
 * "slowest read", 22:33), and the mission's slowest step reaches 40 ms, so
 * 50 ms. A stuck bus fails the IMU step's first call after 50 ms.
 * 400 kHz (fast mode, which the GY-511 supports; was 100 kHz): the IMU task
 * busy-waits on the bus, and on 23 Nov it took 3.0 ms a 10 ms step, 15 % of
 * the CPU. The bench image keeps 100 kHz (firmware/bench/build.sh).
 */
#define ROBOT_I2C_HZ 400000U
#define ROBOT_I2C_TIMEOUT_US 50000U
#define ROBOT_ECHO_TIMEOUT_US 30000U
#define ROBOT_TRIGGER_INTERVAL_US 65000U
#define ROBOT_TRIGGER_PULSE_US 10U
#define ROBOT_SERVO_SETTLE_MS 180U
/* Integer limits compare with shared fields that are rounded up (ceil mm,
 * magnitude-up mrad), which keeps the original double decisions exact;
 * MM / 1000.0 is the same double as the former 0.25 literal. */
#define ROBOT_FRONT_STOP_DISTANCE_MM 250U
#define ROBOT_FRONT_STOP_DISTANCE_M (ROBOT_FRONT_STOP_DISTANCE_MM / 1000.0)
#define ROBOT_ROBOT_WIDTH_M 0.18
#define ROBOT_OBSTACLE_MARGIN_M 0.06
#define ROBOT_REACQUIRE_DISTANCE_M 0.60
/* Accelerometer six-face calibration (S4-U03, 2026-10-27, bench image, car
 * still on each face): sensor mg read upright +26/+56/+990, upside down
 * z -1021, nose down y +1075, nose up y -950, left side down x -980, right
 * side down x +1027. Bias and scale fit so all six read 1 g (the simple
 * (up + down) / 2 pairs agree within 3 mg and 0.3%). Sensor axes, before
 * the mount mapping below.
 */
#define ROBOT_ACCEL_BIAS_X_MG 24
#define ROBOT_ACCEL_BIAS_Y_MG 60
#define ROBOT_ACCEL_BIAS_Z_MG -18
#define ROBOT_ACCEL_SCALE_X 0.994
#define ROBOT_ACCEL_SCALE_Y 0.985
#define ROBOT_ACCEL_SCALE_Z 0.992

/* Course bounds require actual track measurements and an approved codebook.
 * Free-corridor confirmation is an environmental assumption: a frontal scan
 * cannot see the rear depth of an obstacle or prove an entire swept path.
 */
#define ROBOT_COURSE_CONFIRMED 0
#define ROBOT_CORRIDOR_CONFIRMED 0
#define ROBOT_MAX_OBSTACLE_DEPTH_M 0.30
#define ROBOT_ROBOT_LENGTH_M 0.22
#define ROBOT_LINE_UNIQUE_CORRIDOR_M 0.03
#define ROBOT_LINE_HEADING_TOL_RAD 0.20
#define ROBOT_LINE_LOCK_MS 100U
#define ROBOT_TURN_RATE_RPS 0.65
#define ROBOT_LINE_GAIN_RPS 0.75
#define ROBOT_LINE_SPEED_MPS 0.08
#define ROBOT_STALL_MS 500U
/* Unmeasured stall hypothesis: a wheel commanded at |duty| >= this value
 * with no encoder count for ROBOT_STALL_MS latches a hard stall fault. The
 * slow PID ramp from rest (integral build-up, not a setpoint ramp) can trip
 * it if breakaway duty exceeds about 0.16; measure breakaway (T-59).
 * S2-U07 (27 Oct, wheels off the ground): the second sweep turned both
 * wheels steadily from 6-10% duty either way, so 0.15 holds there. The
 * first sweep, the motors' first run, needed up to 27% backward: check a
 * cold start before relying on 0.15. Once turning: about 8.2 mm/s per 1%
 * duty, the right wheel about 3% faster.
 */
#define ROBOT_STALL_DUTY 0.15
#define ROBOT_TILT_LIMIT_MRAD 600
#define ROBOT_TILT_LIMIT_RAD (ROBOT_TILT_LIMIT_MRAD / 1000.0)
/* Flat reference in vehicle axes after the fit and mapping: car upright and
 * still on the floor (S4-U03, 2026-10-27, 9 s), about 0.3 degrees off z. */
#define ROBOT_IMU_REFERENCE_X_MG 4
#define ROBOT_IMU_REFERENCE_Y_MG 2
#define ROBOT_IMU_REFERENCE_Z_MG 1000
#define ROBOT_IMU_MOUNT_CONFIRMED 1
/* Axis indices/signs map the fitted sensor to x forward, y left, z up.
 * S4-U04 (2026-10-27): nose down reads sensor +y and left side down -x, so
 * forward is -y, left is +x and up is +z (a proper rotation).
 */
#define ROBOT_IMU_FORWARD_AXIS 1U
#define ROBOT_IMU_LEFT_AXIS 0U
#define ROBOT_IMU_UP_AXIS 2U
#define ROBOT_IMU_AXES 3U  /* x, y and z */
#define ROBOT_IMU_FORWARD_SIGN -1
#define ROBOT_IMU_LEFT_SIGN 1
#define ROBOT_IMU_UP_SIGN 1

#endif /* ROBOT_CONFIG_H */

/* End of file. */

