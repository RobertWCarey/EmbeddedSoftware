/*!
 * @addtogroup median_module median module documentation
 * @{
 */
/*! @file
 *
 *  @brief Median filter.
 *
 *  This contains the functions for performing a median filter on byte-sized data.
 *
 *  @author PMcL
 *  @date 2015-10-12
 */

#ifndef MEDIAN_H
#define MEDIAN_H

// New types
#include "types.h"

/*! @brief Median filters 3 bytes.
 *
 *  @param n1 is the first  of 3 bytes for which the median is sought.
 *  @param n2 is the second of 3 bytes for which the median is sought.
 *  @param n3 is the third  of 3 bytes for which the median is sought.
 */
float Median_Filter3(const float n1, const float n2, const float n3);

#endif
/*!
 * @}
 */
