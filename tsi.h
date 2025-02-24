/******************************************************************************
 * This file is a part of the Sysytem Microprocessor Tutorial (C).            *
 ******************************************************************************/

/**
 * @file tsi.h
 * @author Koryciak
 * @date Oct 2019
 * @brief File containing enums, structures and declarations.
 * @ver 0.1
 */

#ifndef TSI_H
#define TSI_H

#include "frdm_bsp.h"


/**
 * @brief Touch slider initialization.
 */
void TSI_Init(void);
/**
 * @brief Return value read from the slider (0,100).
 */
uint8_t TSI_ReadSlider (void);

#endif /* TSI_H */
