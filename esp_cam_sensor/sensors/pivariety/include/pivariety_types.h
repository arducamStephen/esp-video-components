/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * pivariety camera sensor register type definition.
 */
struct v4l2_rect {
    int32_t left;
    int32_t top;
    uint32_t width;
    uint32_t height;
};

typedef struct {
    uint16_t reg;
    uint32_t val;
} pivariety_reginfo_t;

#ifdef __cplusplus
}
#endif
