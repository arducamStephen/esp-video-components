/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * imx298 camera sensor register type definition.
 */
typedef struct {
    uint16_t reg;
    uint32_t val;
} imx298_reginfo_t;

#ifdef __cplusplus
}
#endif
