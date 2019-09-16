#include "config.h"
/*
 * USB Paths to devices
 */

const unsigned char paths[CTRL_MAXPI+1][8] = {
        { 2, 255, 255, 255, 255, 255, 255, 255 }, /* ClusterCTRL */
        { 4, 255, 255, 255, 255, 255, 255, 255 }, /* P1 */
        { 3, 255, 255, 255, 255, 255, 255, 255 } /* P2 */
};
