/** @file course_cfg.c
 * @brief Explicit integration point for instructor-confirmed barcode widths.
 * This is missing input, not a pretend decoder implementation: barcode.c
 * implements spatial decoding and is tested with labelled synthetic fixtures.
 */
#include "course_cfg.h"
#include "robot_config.h"
#include <stddef.h>

/** @brief Empty codebook keeps autonomous START inhibited until configured. */
void course_cfg_load (barcode_config_t * p_config)
{
    barcode_defaults(p_config);
    if (NULL != p_config)
    {
        p_config->b_calibration_confirmed =
            0 != ROBOT_CALIBRATION_CONFIRMED;
        /* Enter measured symbols, framing, tolerances and encoder resolution
         * here before confirming. Do not copy test patterns onto the course.
         * reverse_tolerance_m must exceed one effective encoder count of
         * centre distance (2*pi*R/counts, about 0.567 mm with the current
         * hypotheses): the 0.5 mm default is below it, and the start-up
         * codebook gate in robot_app.c does not check this yet (T-57).
         */
        p_config->b_codebook_confirmed = 0 != ROBOT_BARCODE_CONFIRMED;
    }
}

/* End of file. */

