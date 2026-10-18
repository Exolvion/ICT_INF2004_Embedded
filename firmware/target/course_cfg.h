/** @file course_cfg.h
 * @brief Course-specific contract; never infer encoding from example letters.
 */
#ifndef COURSE_CFG_H
#define COURSE_CFG_H

#include "barcode.h"

/** Load the reviewed codebook. Supplied configuration is empty/inhibited. */
void course_cfg_load (barcode_config_t * p_config);

#endif /* COURSE_CFG_H */

/* End of file. */

