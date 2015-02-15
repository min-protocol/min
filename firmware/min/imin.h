/*
 * imin.h
 *
 *  Created on: May 5, 2014
 *      Author: ken
 */

/* Internal MIN definitions - not part of API */

#ifndef IMIN_H_
#define IMIN_H_

#include "min.h"

/* Maximum size of a MIN frame:
 *
 * 3 (header) + 1 (id) + 1 (control) + 15 (payload) + 2 (checksum) + 1 (EOF) + stuff bytes
 *
 * Stuff bytes can be inserted (in worst case) after every 2 bytes in id/control/payload/checksum
 * = (1 + 1 + 15 + 2) / 2 = 10.
 *
 * Maximum size of a frame is therefore 32 bytes.
 */
#define MAX_FRAME_SIZE					(33U)
#define FRAME_LENGTH_MASK               (0x0fU)

/* Magic bytes */
#define HEADER_BYTE						(0xaaU)
#define STUFF_BYTE						(0x55U)
#define EOF_BYTE						(0x55U)

#endif /* IMIN_H_ */
