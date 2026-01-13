// SPDX-License-Identifier: GPL-2.0
/*
 * nv_ar0234.c - ar0234 sensor driver
 *
 * Copyright (c) 2016-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * Copyright (c) 2026, UAB Kurokesu. All rights reserved.
 */

#define DEBUG

#include <nvidia/conftest.h>

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>

#include "../platform/tegra/camera/camera_gpio.h"
#include "ar0234_mode_tbls.h"

/* AR0234 Register Definitions */
/* TODO */

static const struct of_device_id ar0234_of_match[] = {
	{ .compatible = "onnn,ar0234cs" },
	{},
};

MODULE_DEVICE_TABLE(of, ar0234_of_match);

/* TODO */
// static int test_mode;
// module_param(test_mode, int, 0644);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

enum ar0234_config {
	TWO_LANE_CONFIG,
	FOUR_LANE_CONFIG,
};

struct ar0234 {
	struct i2c_client *i2c_client;
	struct v4l2_subdev *subdev;
	u32 frame_length;
	struct camera_common_data *s_data;
	struct tegracam_device *tc_dev;
	enum ar0234_config config;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.cache_type = REGCACHE_RBTREE,
	.use_single_read = true,
	.use_single_write = true,
};

static inline int ar0234_read_reg(struct camera_common_data *s_data, u16 addr,
				  u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xff;

	return err;
}

static inline int ar0234_write_reg(struct camera_common_data *s_data, u16 addr,
				   u8 val)
{
	int err = 0;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(s_data->dev, "%s: i2c write failed, 0x%x = %x",
			__func__, addr, val);

	return err;
}

static int ar0234_write_table(struct ar0234 *priv, const ar0234_reg table[])
{
	int err;

	dev_dbg(priv->s_data->dev, "%s: Writing register table\n", __func__);

	err = regmap_util_write_table_8(priv->s_data->regmap, table, NULL, 0,
					IMX462_TABLE_WAIT_MS, IMX462_TABLE_END);

	if (err) {
		dev_err(priv->s_data->dev, "%s: Failed to write table (%d)\n",
			__func__, err);
	} else {
		dev_dbg(priv->s_data->dev,
			"%s: Register table written successfully\n", __func__);
	}

	return err;
}

static int ar0234_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	/* TODO */
	return 0;
}

static int ar0234_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	/* TODO */
	return 0;
}

static int ar0234_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	/* TODO */
	return 0;
}

static int ar0234_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	/* TODO */
	return 0;
}

static struct tegracam_ctrl_ops ar0234_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = ar0234_set_gain,
	.set_exposure = ar0234_set_exposure,
	.set_frame_rate = ar0234_set_frame_rate,
	.set_group_hold = ar0234_set_group_hold,
};

static int ar0234_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power on\n", __func__);
	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	if (pw->reset_gpio) {
		if (gpiod_cansleep(gpio_to_desc(pw->reset_gpio)))
			gpio_set_value_cansleep(pw->reset_gpio, 0);
		else
			gpio_set_value(pw->reset_gpio, 0);
	}

	if (unlikely(!(pw->avdd || pw->iovdd || pw->dvdd)))
		goto skip_power_seqn;

	usleep_range(10, 20);

	if (pw->avdd) {
		err = regulator_enable(pw->avdd);
		if (err)
			goto ar0234_avdd_fail;
	}

	if (pw->iovdd) {
		err = regulator_enable(pw->iovdd);
		if (err)
			goto ar0234_iovdd_fail;
	}

	if (pw->dvdd) {
		err = regulator_enable(pw->dvdd);
		if (err)
			goto ar0234_dvdd_fail;
	}

	usleep_range(10, 20);

skip_power_seqn:
	if (pw->reset_gpio) {
		if (gpiod_cansleep(gpio_to_desc(pw->reset_gpio)))
			gpio_set_value_cansleep(pw->reset_gpio, 1);
		else
			gpio_set_value(pw->reset_gpio, 1);
	}

	/* TODO */
	/* Need to wait for t4 + t5 + t9 + t10 time as per the data sheet */
	/* t4 - 200us, t5 - 21.2ms, t9 - 1.2ms t10 - 270 ms */
	usleep_range(300000, 300100);

	pw->state = SWITCH_ON;

	return 0;

ar0234_dvdd_fail:
	regulator_disable(pw->iovdd);

ar0234_iovdd_fail:
	regulator_disable(pw->avdd);

ar0234_avdd_fail:
	dev_err(dev, "%s failed.\n", __func__);

	return -ENODEV;
}

static int ar0234_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (err) {
			dev_err(dev, "%s failed.\n", __func__);
			return err;
		}
	} else {
		if (pw->reset_gpio) {
			if (gpiod_cansleep(gpio_to_desc(pw->reset_gpio)))
				gpio_set_value_cansleep(pw->reset_gpio, 0);
			else
				gpio_set_value(pw->reset_gpio, 0);
		}

		usleep_range(10, 10);

		if (pw->dvdd)
			regulator_disable(pw->dvdd);
		if (pw->iovdd)
			regulator_disable(pw->iovdd);
		if (pw->avdd)
			regulator_disable(pw->avdd);
	}

	pw->state = SWITCH_OFF;

	return 0;
}

static int ar0234_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->dvdd))
		devm_regulator_put(pw->dvdd);

	if (likely(pw->avdd))
		devm_regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		devm_regulator_put(pw->iovdd);

	pw->dvdd = NULL;
	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (likely(pw->reset_gpio))
		gpio_free(pw->reset_gpio);

	return 0;
}

static int ar0234_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct clk *parent;
	int err = 0;

	if (!pdata) {
		dev_err(dev, "pdata missing\n");
		return -EFAULT;
	}

	/* Sensor MCLK (aka. EXTCLK) */
	if (pdata->mclk_name) {
		pw->mclk = devm_clk_get(dev, pdata->mclk_name);
		if (IS_ERR(pw->mclk)) {
			dev_err(dev, "unable to get clock %s\n",
				pdata->mclk_name);
			return PTR_ERR(pw->mclk);
		}

		if (pdata->parentclk_name) {
			parent = devm_clk_get(dev, pdata->parentclk_name);
			if (IS_ERR(parent)) {
				dev_err(dev, "unable to get parent clock %s",
					pdata->parentclk_name);
			} else
				clk_set_parent(pw->mclk, parent);
		}
	}

	/* analog 2.8v */
	if (pdata->regulators.avdd)
		err |= camera_common_regulator_get(dev, &pw->avdd,
						   pdata->regulators.avdd);
	/* IO 1.8v */
	if (pdata->regulators.iovdd)
		err |= camera_common_regulator_get(dev, &pw->iovdd,
						   pdata->regulators.iovdd);
	/* dig 1.2v */
	if (pdata->regulators.dvdd)
		err |= camera_common_regulator_get(dev, &pw->dvdd,
						   pdata->regulators.dvdd);
	if (err) {
		dev_err(dev, "%s: unable to get regulator(s)\n", __func__);
		goto done;
	}

	/* Reset or ENABLE GPIO */
	pw->reset_gpio = pdata->reset_gpio;
	err = gpio_request(pw->reset_gpio, "cam_reset_gpio");
	if (err < 0) {
		dev_err(dev, "%s: unable to request reset_gpio (%d)\n",
			__func__, err);
		goto done;
	}

done:
	pw->state = SWITCH_OFF;

	return err;
}

static struct camera_common_pdata *
ar0234_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;
	int err = 0;
	int gpio;

	if (!np)
		return NULL;

	match = of_match_device(ar0234_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata =
		devm_kzalloc(dev, sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "reset-gpios not found\n");
		goto error;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	err = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
	if (err)
		dev_dbg(dev,
			"mclk name not present, assume sensor driven externally\n");

	err = of_property_read_string(np, "avdd-reg",
				      &board_priv_pdata->regulators.avdd);
	err |= of_property_read_string(np, "iovdd-reg",
				       &board_priv_pdata->regulators.iovdd);
	err |= of_property_read_string(np, "dvdd-reg",
				       &board_priv_pdata->regulators.dvdd);
	if (err)
		dev_dbg(dev,
			"avdd, iovdd and/or dvdd reglrs. not present, assume sensor powered independently\n");

	board_priv_pdata->has_eeprom = of_property_read_bool(np, "has-eeprom");

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);

	return ret;
}

static int ar0234_set_mode(struct tegracam_device *tc_dev)
{
	struct ar0234 *priv = (struct ar0234 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	unsigned int mode_index = 0;
	int err = 0;
	const char *config;
	struct device_node *mode;
	uint offset = ARRAY_SIZE(ar0234_frmfmt);

	dev_dbg(tc_dev->dev, "%s:\n", __func__);
	mode = of_get_child_by_name(tc_dev->dev->of_node, "mode0");
	err = of_property_read_string(mode, "num_lanes", &config);

	if (config[0] == '4') {
		priv->config = FOUR_LANE_CONFIG;
		dev_dbg(tc_dev->dev, "Using 4-lane configuration\n");
	} else if (config[0] == '2') {
		priv->config = TWO_LANE_CONFIG;
		dev_dbg(tc_dev->dev, "Using 2-lane configuration\n");
	} else {
		dev_err(tc_dev->dev, "Unsupported config\n");
		return -EINVAL;
	}

	/* TODO */
	err = ar0234_write_table(priv, mode_table[AR0234_MODE_COMMON]);
	if (err)
		return err;
	
	mode_index = s_data->mode;
	if (priv->config == FOUR_LANE_CONFIG)
		err = ar0234_write_table(priv, mode_table[mode_index + offset]);
	else {
		dev_dbg(tc_dev->dev, "Writing mode table %d\n", mode_index);
		err = ar0234_write_table(priv, mode_table[mode_index]);
	}
	if (err)
		return err;

	return 0;
}

static int ar0234_start_streaming(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct ar0234 *priv = (struct ar0234 *)tegracam_get_privdata(tc_dev);
	int err = 0;

	dev_dbg(tc_dev->dev, "%s:\n", __func__);

	if (test_mode) {
		dev_dbg(tc_dev->dev, "Test mode %d\n", test_mode);

		err = ar0234_write_table(priv,
					 mode_table[AR0234_MODE_TEST_PATTERN]);
		if (err)
			return err;

		/* TODO*/
		// err = imx462_write_reg(s_data, IMX462_PGCTRL,
		// 		       IMX462_PGCTRL_REGEN |
		// 			       IMX462_PGCTRL_THRU |
		// 			       IMX462_PGCTRL_MODE(test_mode));

		if (err)
			return err;
	}

	return ar0234_write_table(priv, mode_table[AR0234_START_STREAM]);
}

static int ar0234_stop_streaming(struct tegracam_device *tc_dev)
{
	int err;
	struct ar0234 *priv = (struct ar0234 *)tegracam_get_privdata(tc_dev);

	dev_dbg(tc_dev->dev, "%s:\n", __func__);
	err = ar0234_write_table(priv, mode_table[AR0234_STOP_STREAM]);

	return err;
}

static struct camera_common_sensor_ops ar0234_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ar0234_frmfmt),
	.frmfmt_table = ar0234_frmfmt,
	.power_on = ar0234_power_on,
	.power_off = ar0234_power_off,
	.write_reg = ar0234_write_reg,
	.read_reg = ar0234_read_reg,
	.parse_dt = ar0234_parse_dt,
	.power_get = ar0234_power_get,
	.power_put = ar0234_power_put,
	.set_mode = ar0234_set_mode,
	.start_streaming = ar0234_start_streaming,
	.stop_streaming = ar0234_stop_streaming,
};

static int ar0234_board_setup(struct ar0234 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	u16 reg_val;
	int err = 0;

	/* Skip mclk enable as this camera module has an on-board oscillator */

	err = ar0234_power_on(s_data);
	if (err) {
		dev_err(dev, "error during power on sensor (%d)\n", err);
		goto done;
	}

	/* Probe sensor model id registers */
	err = ar0234_read_reg(s_data, AR0234_MODEL_ID_ADDR, &reg_val);
	if (err) {
		dev_err(dev, "%s: error during i2c read probe (%d)\n", __func__,
			err);
		goto err_reg_probe;
	}

	dev_dbg(dev, "%s: sensor model id: 0x%x\n", __func__, reg_val);

	if (reg_val != AR0234_MODEL_ID_VAL)
		dev_err(dev, "%s: invalid sensor model id: 0x%x\n", __func__,
			reg_val);

err_reg_probe:
	ar0234_power_off(s_data);

done:
	return err;
}

static int ar0234_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops ar0234_subdev_internal_ops = {
	.open = ar0234_open,
};

#if defined(NV_I2C_DRIVER_STRUCT_PROBE_WITHOUT_I2C_DEVICE_ID_ARG) /* Linux 6.3 */
static int ar0234_probe(struct i2c_client *client)
#else
static int ar0234_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
#endif
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct ar0234 *priv;
	int err;

	dev_dbg(dev, "probing v4l2 sensor at addr 0x%0x\n", client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(struct ar0234), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev, sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "ar0234", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &ar0234_common_ops;
	tc_dev->v4l2sd_internal_ops = &ar0234_subdev_internal_ops;
	tc_dev->tcctrl_ops = &ar0234_ctrl_ops;
	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = ar0234_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		tegracam_device_unregister(tc_dev);
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_dbg(dev, "detected ar0234 sensor\n");

	return 0;
}

#if defined(NV_I2C_DRIVER_STRUCT_REMOVE_RETURN_TYPE_INT) /* Linux 6.1 */
static int ar0234_remove(struct i2c_client *client)
#else
static void ar0234_remove(struct i2c_client *client)
#endif
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0234 *priv;

	if (!s_data) {
		dev_err(&client->dev, "camera common data is NULL\n");
#if defined(NV_I2C_DRIVER_STRUCT_REMOVE_RETURN_TYPE_INT) /* Linux 6.1 */
		return -EINVAL;
#else
		return;
#endif
	}
	priv = (struct imx462 *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);
#if defined(NV_I2C_DRIVER_STRUCT_REMOVE_RETURN_TYPE_INT) /* Linux 6.1 */
	return 0;
#endif
}

static const struct i2c_device_id ar0234_id[] = { { "ar0234", 0 }, {} };

MODULE_DEVICE_TABLE(i2c, ar0234_id);

static struct i2c_driver ar0234_i2c_driver = {
	.driver = {
		   .name = "ar0234",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(ar0234_of_match),
		   },
	.probe = ar0234_probe,
	.remove = ar0234_remove,
	.id_table = ar0234_id,
};

module_i2c_driver(ar0234_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for OnSemi AR0234");
MODULE_AUTHOR("UAB Kurokesu");
MODULE_LICENSE("GPL v2");
