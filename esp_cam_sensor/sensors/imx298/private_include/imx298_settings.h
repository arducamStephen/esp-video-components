/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdio.h>
#include <stdint.h>
#include <sdkconfig.h>
#include "imx298_regs.h"
#include "imx298_types.h"

#ifdef __cplusplus
extern "C" {
#endif

static const imx298_reginfo_t imx298_mipi_stream_on[] = {
    {STREAM_ON,            0x00000001},
    {IMX298_REG_END,       0x00000000},
};

static const imx298_reginfo_t imx298_mipi_stream_off[] = {
    {STREAM_ON,            0x00000000},
    {IMX298_REG_END,       0x00000000},
};

static const imx298_reginfo_t imx298_MIPI_2lane_raw10_1280x720_30fps[] = {
    {PIXFORMAT_INDEX_REG,  0x00000000},
    {RESOLUTION_INDEX_REG, 0x00000001},
    {STREAM_ON,            0x00000001},
    {IMX298_REG_END,       0x00000000},
};

// static const imx298_reginfo_t imx298_MIPI_2lane_raw10_1280x800_45fps[] = {
//     {PIXFORMAT_INDEX_REG,  0x00000000},
//     {RESOLUTION_INDEX_REG, 0x00000001},
//     {STREAM_ON,            0x00000001},
//     {IMX298_REG_END,       0x00000000},
// };

#ifdef __cplusplus
}
#endif
