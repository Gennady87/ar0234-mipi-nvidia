/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar0234_mode_tbls.h - ar0234 sensor driver
 *
 * Copyright (c) 2016-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * Copyright (c) 2026, UAB Kurokesu. All rights reserved.
 */

#ifndef __AR0234_MODE_TBLS_H__
#define __AR0234_MODE_TBLS_H__

#include <media/camera_common.h>
#include <linux/miscdevice.h>

#define ar0234_reg struct reg_8

/* AR0234 Start Streaming */
static ar0234_reg ar0234_start[] = {
/* TODO */
};

/* AR0234 Stop Streaming */
static ar0234_reg ar0234_stop[] = {
/* TODO */
};

/* AR0234 Common Mode */
static ar0234_reg ar0234_mode_common[] = {
/* TODO*/
};

/* AR0234 1920x1200 2-lane mode */
static ar0234_reg ar0234_mode_1920x1200[] = {
/* TODO */
};

static ar0234_reg ar0234_mode_test_pattern[] = {
/* TODO */
};

enum { AR0234_MODE_1920X1200,
       AR0234_MODE_COMMON,
       AR0234_START_STREAM,
       AR0234_STOP_STREAM,
       AR0234_MODE_TEST_PATTERN,
};

static ar0234_reg *mode_table[] = {
	[AR0234_MODE_1920X1200] = ar0234_mode_1920x1200,
	[AR0234_MODE_COMMON] = ar0234_mode_common,
	[AR0234_START_STREAM] = ar0234_start,
	[AR0234_STOP_STREAM] = ar0234_stop,
	[AR0234_MODE_TEST_PATTERN] = ar0234_mode_test_pattern,
};

static const int ar0234_30fps[] = {
	30,
};

/*
 * WARNING: frmfmt ordering needs to match mode definition in
 * device tree!
 */
static const struct camera_common_frmfmt ar0234_frmfmt[] = {
	{ { 1920, 1200 }, ar0234_30fps, 1, 0, AR0234_MODE_1920X1200 },
};

#endif /* __AR0234_MODE_TBLS_H__ */
