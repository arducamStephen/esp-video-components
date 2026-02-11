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

enum imx500_state {
	IMX500_STATE_RESET = 0,
	IMX500_STATE_PROGRAM_EMPTY,
	IMX500_STATE_WITHOUT_NETWORK,
	IMX500_STATE_WITH_NETWORK,
};

struct imx500 {
	esp_sccb_io_handle_t sccb_handle;
	i2c_master_bus_handle_t bus_handle;

	unsigned int fmt_code; 

	struct v4l2_rect inference_window;

	/* Current mode */
	// const struct imx500_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	// struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;

	bool loader_and_main_written;
	bool network_written;

	/* Current long exposure factor in use. Set through V4L2_CID_VBLANK */
	// unsigned int long_exp_shift;

	const struct firmware *fw_loader;
	const struct firmware *fw_main;
	const uint8_t *fw_network;
	size_t fw_network_size;
	size_t fw_progress;
	unsigned int fw_stage;

	enum imx500_state fsm_state;

	uint32_t num_inference_lines;
};

#ifdef __cplusplus
}
#endif
