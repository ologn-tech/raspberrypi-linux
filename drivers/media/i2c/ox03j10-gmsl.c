// SPDX-License-Identifier: GPL-2.0
/*
 * OX03J10 GMSL2 camera driver
 *
 * SerDes sequence supplied by Shenzhen Sensing World Intelligent Technology.
 * Topology: OX03J10 SoC -> MAX96717 -> GMSL2 link A -> MAX9296 port A -> CSI-2
 * Format: 1920x1536 UYVY 8-bit, 30 fps
 *
 * The pre-stream register table is the QSTS ("quick start") script from the
 * sg26313 ISP firmware (1920x1536 UYVY 30 fps). SYS1/ISPS stay in the
 * module SPI image; this host table only programs PLL/output then stream-on.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/string.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define OX03J10_WIDTH			1920
#define OX03J10_HEIGHT			1536
#define OX03J10_CODE			MEDIA_BUS_FMT_UYVY8_1X16
#define OX03J10_BITS_PER_SAMPLE		16
#define OX03J10_DEFAULT_DATA_LANES	4
#define OX03J10_DEFAULT_LINK_FREQ	1200000000ULL
#define OX03J10_CSI2_DT_YUV422_8B	0x1e

/* Vendor documentation uses 8-bit addresses 0x90, 0x80, 0x6c. */
#define MAX9296_DEFAULT_ADDR		(0x90 >> 1) /* Linux address: 0x48 */
#define MAX96717_DEFAULT_ADDR		(0x80 >> 1) /* Linux address: 0x40 */
#define OX03J10_DEFAULT_ADDR		(0x6c >> 1) /* Linux address: 0x36 */

#define OX03J10_REG_MODE_SELECT		0x0100
#define OX03J10_MODE_STANDBY		0x00
#define OX03J10_MODE_STREAMING		0x01
#define OX03J10_REG_DELAY		0xffff

#define MAX9296_REG_CTRL3		0x0013
#define MAX9296_LOCKED			BIT(3)
#define MAX9296_REG_MIPI_OUT		0x0313
#define MAX9296_MIPI_DISABLE		0x00
#define MAX9296_MIPI_ENABLE_PORT_A	0x02

#define OX03J10_GMSL_LOCK_SLEEP_US	20000
#define OX03J10_GMSL_LOCK_TIMEOUT_US	2000000
#define OX03J10_SER_WRITE_RETRIES	20
#define OX03J10_SER_RETRY_DELAY_MS	50
#define OX03J10_SENSOR_WRITE_RETRIES	5
#define OX03J10_SENSOR_RETRY_DELAY_MS	20

struct ox03j10_gmsl {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	/* Protects the streaming state and SerDes stream enable writes. */
	struct mutex lock;
	struct i2c_client *client;
	struct v4l2_mbus_config_mipi_csi2 csi2;
	u64 link_freq;
	s64 link_freq_menu[1];
	u16 dser_addr;
	u16 ser_addr;
	u16 sensor_addr;
	bool streaming;
};

struct ox03j10_regval {
	u16 addr;
	u8 val;
};

/*
 * QSTS script from the sg26313 firmware. addr 0xffff is a delay in milliseconds.
 * Duplicate addresses are intentional (PLL staged writes).
 */
static const struct ox03j10_regval ox03j10_prestream_regs[] = {
};

static inline struct ox03j10_gmsl *to_ox03j10(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ox03j10_gmsl, sd);
}

static int ox03j10_i2c_xfer_write(struct ox03j10_gmsl *priv, u16 target,
				  u16 reg, u8 value)
{
	u8 data[] = { reg >> 8, reg & 0xff, value };
	struct i2c_msg msg = {
		.addr = target,
		.flags = 0,
		.len = sizeof(data),
		.buf = data,
	};
	int ret;

	ret = i2c_transfer(priv->client->adapter, &msg, 1);
	if (ret == 1)
		return 0;

	return ret < 0 ? ret : -EIO;
}

static int ox03j10_i2c_write(struct ox03j10_gmsl *priv, u16 target,
			     u16 reg, u8 value)
{
	int ret;

	ret = ox03j10_i2c_xfer_write(priv, target, reg, value);
	if (ret)
		dev_err(&priv->client->dev,
			"I2C write failed: addr=0x%02x reg=0x%04x val=0x%02x (%d)\n",
			target, reg, value, ret);
	return ret;
}

static int ox03j10_i2c_read(struct ox03j10_gmsl *priv, u16 target,
			    u16 reg, u8 *value)
{
	u8 addr[] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[] = {
		{
			.addr = target,
			.flags = 0,
			.len = sizeof(addr),
			.buf = addr,
		},
		{
			.addr = target,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = value,
		},
	};
	int ret;

	ret = i2c_transfer(priv->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret == ARRAY_SIZE(msgs))
		return 0;

	return ret < 0 ? ret : -EIO;
}

static int max9296_write(struct ox03j10_gmsl *priv, u16 reg, u8 value)
{
	return ox03j10_i2c_write(priv, priv->dser_addr, reg, value);
}

static int max96717_write(struct ox03j10_gmsl *priv, u16 reg, u8 value)
{
	int ret = 0;
	unsigned int i;

	for (i = 0; i < OX03J10_SER_WRITE_RETRIES; i++) {
		ret = ox03j10_i2c_xfer_write(priv, priv->ser_addr, reg, value);
		if (!ret)
			return 0;
		msleep(OX03J10_SER_RETRY_DELAY_MS);
	}

	dev_err(&priv->client->dev,
		"I2C write failed: addr=0x%02x reg=0x%04x val=0x%02x (%d)\n",
		priv->ser_addr, reg, value, ret);
	return ret;
}

static int ox03j10_sensor_write(struct ox03j10_gmsl *priv, u16 reg, u8 value)
{
	int ret = 0;
	unsigned int i;

	for (i = 0; i < OX03J10_SENSOR_WRITE_RETRIES; i++) {
		ret = ox03j10_i2c_xfer_write(priv, priv->sensor_addr, reg, value);
		if (!ret)
			return 0;
		msleep(OX03J10_SENSOR_RETRY_DELAY_MS);
	}

	dev_err(&priv->client->dev,
		"I2C write failed: addr=0x%02x reg=0x%04x val=0x%02x (%d)\n",
		priv->sensor_addr, reg, value, ret);
	return ret;
}

static int ox03j10_write_regs(struct ox03j10_gmsl *priv,
			      const struct ox03j10_regval *regs,
			      unsigned int nregs)
{
	unsigned int i;
	int ret;

	for (i = 0; i < nregs; i++) {
		if (regs[i].addr == OX03J10_REG_DELAY) {
			msleep(regs[i].val);
			continue;
		}

		ret = ox03j10_sensor_write(priv, regs[i].addr, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static bool ox03j10_gmsl_locked(struct ox03j10_gmsl *priv)
{
	u8 val;
	int ret;

	ret = ox03j10_i2c_read(priv, priv->dser_addr, MAX9296_REG_CTRL3, &val);
	if (ret)
		return false;

	return val & MAX9296_LOCKED;
}

static int ox03j10_wait_gmsl_lock(struct ox03j10_gmsl *priv)
{
	struct device *dev = &priv->client->dev;
	u8 ctrl3 = 0;
	int locked;
	int ret;

	ret = read_poll_timeout(ox03j10_gmsl_locked, locked, locked,
				OX03J10_GMSL_LOCK_SLEEP_US,
				OX03J10_GMSL_LOCK_TIMEOUT_US, false, priv);
	ox03j10_i2c_read(priv, priv->dser_addr, MAX9296_REG_CTRL3, &ctrl3);
	if (ret) {
		dev_err(dev,
			"GMSL link not locked (CTRL3=0x%02x); MAX96717 is unreachable until lock\n",
			ctrl3);
		return -EIO;
	}

	dev_dbg(dev, "GMSL link locked (CTRL3=0x%02x)\n", ctrl3);
	return 0;
}

/*
 * MAX9296 1CH bring-up taken from the working ISX031 sequence on this HAT,
 * then the Sensing World OX03J10 serializer GPIO/trigger steps.
 *
 * Writing 0x0001 (rate) without RESET_ALL, PoC GPIOs, and CTRL1 link enable
 * leaves the reverse I2C tunnel down, so the first MAX96717 access NACKs.
 */
static int ox03j10_serdes_init(struct ox03j10_gmsl *priv)
{
	static const struct {
		u16 reg;
		u8 val;
	} max9296_1ch[] = {
		{ 0x0010, 0x80 }, /* RESET_ALL */
		{ 0x0313, 0x00 }, /* MIPI off */
		{ 0x0005, 0x80 },
		{ 0x02bc, 0x90 }, /* deserializer GPIO / PoC enable */
		{ 0x02bc, 0x80 },
		{ 0x02bd, 0x84 },
		{ 0x0001, 0x01 }, /* 3 Gbps */
		{ 0x0011, 0x0f }, /* enable GMSL link A */
		{ 0x0320, 0x2c }, /* CSI DPLL ~1.2 Gbps */
		{ 0x044a, 0xc0 }, /* 4 lanes, port A */
		{ 0x048a, 0xc0 },
		{ 0x044b, 0x07 },
		{ 0x046d, 0x15 },
		{ 0x044d, 0x1e }, /* YUV422 8-bit */
		{ 0x044e, 0x1e },
		{ 0x044f, 0x00 },
		{ 0x0450, 0x00 },
		{ 0x0451, 0x01 },
		{ 0x0452, 0x01 },
	};
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(max9296_1ch); i++) {
		ret = max9296_write(priv, max9296_1ch[i].reg,
				    max9296_1ch[i].val);
		if (ret)
			return ret;

		if ((max9296_1ch[i].reg == 0x0010 &&
		     max9296_1ch[i].val == 0x80) ||
		    (max9296_1ch[i].reg == 0x02bc &&
		     max9296_1ch[i].val == 0x90))
			msleep(200);
	}

	msleep(500);

	ret = ox03j10_wait_gmsl_lock(priv);
	if (ret)
		return ret;

	/* MAX96717: reset camera, route to port A, select YUV422. */
	ret = max96717_write(priv, 0x02be, 0x10);
	if (ret)
		return ret;
	ret = max96717_write(priv, 0x005b, 0x01);
	if (ret)
		return ret;
	ret = max96717_write(priv, 0x0318, 0x5e);
	if (ret)
		return ret;

	/* Trigger-mode sequence: MFP7/MFP8 low, then high. */
	ret = max96717_write(priv, 0x02d3, 0x00);
	if (ret)
		return ret;
	ret = max96717_write(priv, 0x02d6, 0x00);
	if (ret)
		return ret;
	msleep(1000);
	ret = max96717_write(priv, 0x02d3, 0x10);
	if (ret)
		return ret;
	ret = max96717_write(priv, 0x02d6, 0x10);
	if (ret)
		return ret;
	msleep(500);

	return max9296_write(priv, MAX9296_REG_MIPI_OUT, MAX9296_MIPI_DISABLE);
}

static void ox03j10_fill_format(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = OX03J10_WIDTH;
	fmt->height = OX03J10_HEIGHT;
	fmt->code = OX03J10_CODE;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int ox03j10_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index)
		return -EINVAL;

	code->code = OX03J10_CODE;
	return 0;
}

static int ox03j10_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad || fse->index || fse->code != OX03J10_CODE)
		return -EINVAL;

	fse->min_width = OX03J10_WIDTH;
	fse->max_width = OX03J10_WIDTH;
	fse->min_height = OX03J10_HEIGHT;
	fse->max_height = OX03J10_HEIGHT;
	return 0;
}

static int ox03j10_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
{
	if (fmt->pad)
		return -EINVAL;

	ox03j10_fill_format(&fmt->format);
	return 0;
}

static int ox03j10_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *try_fmt;

	if (fmt->pad)
		return -EINVAL;

	ox03j10_fill_format(&fmt->format);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		try_fmt = v4l2_subdev_state_get_format(state, fmt->pad);
		*try_fmt = fmt->format;
	}

	return 0;
}

static int ox03j10_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				   struct v4l2_mbus_config *config)
{
	struct ox03j10_gmsl *priv = to_ox03j10(sd);

	if (pad)
		return -EINVAL;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->link_freq = priv->link_freq;
	config->bus.mipi_csi2 = priv->csi2;

	return 0;
}

static int ox03j10_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	if (pad)
		return -EINVAL;

	memset(fd, 0, sizeof(*fd));
	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;
	fd->entry[0].pixelcode = OX03J10_CODE;
	fd->entry[0].bus.csi2.dt = OX03J10_CSI2_DT_YUV422_8B;
	return 0;
}

static int ox03j10_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ox03j10_gmsl *priv = to_ox03j10(sd);
	int ret = 0;

	mutex_lock(&priv->lock);
	if (priv->streaming == !!enable)
		goto out;

	if (enable) {
		ret = ox03j10_write_regs(priv, ox03j10_prestream_regs,
					 ARRAY_SIZE(ox03j10_prestream_regs));
		if (ret)
			goto out;

		ret = ox03j10_sensor_write(priv, OX03J10_REG_MODE_SELECT,
					   OX03J10_MODE_STREAMING);
		if (ret)
			goto out;

		msleep(10);

		ret = max9296_write(priv, MAX9296_REG_MIPI_OUT,
				    MAX9296_MIPI_ENABLE_PORT_A);
		if (ret) {
			ox03j10_sensor_write(priv, OX03J10_REG_MODE_SELECT,
					     OX03J10_MODE_STANDBY);
			goto out;
		}
	} else {
		ret = max9296_write(priv, MAX9296_REG_MIPI_OUT,
				    MAX9296_MIPI_DISABLE);
		ox03j10_sensor_write(priv, OX03J10_REG_MODE_SELECT,
				     OX03J10_MODE_STANDBY);
		if (ret)
			goto out;
	}

	priv->streaming = !!enable;
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int ox03j10_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	return ox03j10_s_stream(sd, 1);
}

static int ox03j10_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	return ox03j10_s_stream(sd, 0);
}

static const struct v4l2_subdev_video_ops ox03j10_video_ops = {
	.s_stream = ox03j10_s_stream,
};

static const struct v4l2_subdev_pad_ops ox03j10_pad_ops = {
	.enum_mbus_code = ox03j10_enum_mbus_code,
	.enum_frame_size = ox03j10_enum_frame_size,
	.get_fmt = ox03j10_get_fmt,
	.set_fmt = ox03j10_set_fmt,
	.get_mbus_config = ox03j10_get_mbus_config,
	.get_frame_desc = ox03j10_get_frame_desc,
	.enable_streams = ox03j10_enable_streams,
	.disable_streams = ox03j10_disable_streams,
};

static const struct v4l2_subdev_ops ox03j10_subdev_ops = {
	.video = &ox03j10_video_ops,
	.pad = &ox03j10_pad_ops,
};

static const struct media_entity_operations ox03j10_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* Accept Linux 7-bit values. Also tolerate the vendor's 8-bit notation. */
static int ox03j10_read_i2c_addr(struct device *dev, const char *property,
				 u16 default_addr, u16 *result)
{
	u32 value;

	if (device_property_read_u32(dev, property, &value))
		value = default_addr;

	if (value > 0x7f) {
		if (value > 0xfe || (value & 1))
			return -EINVAL;

		dev_warn(dev,
			 "%s uses 8-bit I2C notation; converting 0x%x to 0x%x\n",
			 property, value, value >> 1);
		value >>= 1;
	}

	*result = value;
	return 0;
}

static int ox03j10_parse_endpoint(struct ox03j10_gmsl *priv)
{
	struct device *dev = &priv->client->dev;
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *endpoint;
	unsigned int lanes;
	unsigned int i;
	int ret = 0;

	priv->csi2.clock_lane = 0;
	priv->csi2.num_data_lanes = OX03J10_DEFAULT_DATA_LANES;
	for (i = 0; i < OX03J10_DEFAULT_DATA_LANES; i++)
		priv->csi2.data_lanes[i] = i + 1;
	priv->link_freq = OX03J10_DEFAULT_LINK_FREQ;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint)
		return 0;

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse endpoint\n");

	lanes = ep.bus.mipi_csi2.num_data_lanes;
	if (!lanes)
		lanes = OX03J10_DEFAULT_DATA_LANES;

	if (lanes != 1 && lanes != 2 && lanes != 4) {
		ret = dev_err_probe(dev, -EINVAL,
				    "unsupported CSI-2 lane count %u\n",
				    lanes);
		goto out_free_endpoint;
	}

	priv->csi2 = ep.bus.mipi_csi2;
	priv->csi2.num_data_lanes = lanes;
	if (ep.nr_of_link_frequencies)
		priv->link_freq = ep.link_frequencies[0];

out_free_endpoint:
	v4l2_fwnode_endpoint_free(&ep);

	return ret;
}

static int ox03j10_init_controls(struct ox03j10_gmsl *priv)
{
	struct v4l2_ctrl_handler *handler = &priv->ctrl_handler;
	u64 pixel_rate;
	int ret;

	ret = v4l2_ctrl_handler_init(handler, 2);
	if (ret)
		return ret;

	priv->link_freq_menu[0] = priv->link_freq;
	v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ, 0, 0,
			       priv->link_freq_menu);

	pixel_rate = div_u64(priv->link_freq * 2 * priv->csi2.num_data_lanes,
			     OX03J10_BITS_PER_SAMPLE);
	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE, pixel_rate,
			  pixel_rate, 1, pixel_rate);

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	priv->sd.ctrl_handler = handler;
	return 0;
}

static int ox03j10_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ox03j10_gmsl *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	ret = ox03j10_read_i2c_addr(dev, "max9296-addr",
				    MAX9296_DEFAULT_ADDR, &priv->dser_addr);
	if (ret)
		return dev_err_probe(dev, ret, "invalid max9296-addr\n");

	ret = ox03j10_read_i2c_addr(dev, "max96717-addr",
				    MAX96717_DEFAULT_ADDR, &priv->ser_addr);
	if (ret)
		return dev_err_probe(dev, ret, "invalid max96717-addr\n");

	ret = ox03j10_read_i2c_addr(dev, "sensor-addr",
				    OX03J10_DEFAULT_ADDR, &priv->sensor_addr);
	if (ret)
		return dev_err_probe(dev, ret, "invalid sensor-addr\n");

	ret = ox03j10_parse_endpoint(priv);
	if (ret)
		return ret;

	v4l2_i2c_subdev_init(&priv->sd, client, &ox03j10_subdev_ops);

	ret = ox03j10_init_controls(priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init controls\n");

	mutex_init(&priv->lock);

	ret = ox03j10_serdes_init(priv);
	if (ret)
		goto err_serdes;

	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	priv->sd.entity.ops = &ox03j10_entity_ops;
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&priv->sd.entity, 1, &priv->pad);
	if (ret)
		goto err_free_ctrls;

	priv->sd.state_lock = priv->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&priv->sd);
	if (ret)
		goto err_entity_cleanup;

	ret = v4l2_async_register_subdev_sensor(&priv->sd);
	if (ret)
		goto err_subdev_cleanup;

	dev_info(dev,
		 "OX03J10 GMSL ready (MAX9296=0x%02x, MAX96717=0x%02x, OX03J10=0x%02x)\n",
		 priv->dser_addr, priv->ser_addr, priv->sensor_addr);
	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(&priv->sd);
err_entity_cleanup:
	media_entity_cleanup(&priv->sd.entity);
err_free_ctrls:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	mutex_destroy(&priv->lock);

	return ret;

err_serdes:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	mutex_destroy(&priv->lock);

	return dev_err_probe(dev, ret, "SerDes initialization failed\n");
}

static void ox03j10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ox03j10_gmsl *priv = to_ox03j10(sd);

	ox03j10_s_stream(sd, 0);
	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	mutex_destroy(&priv->lock);
}

static const struct of_device_id ox03j10_of_match[] = {
	{ .compatible = "sensing-world,ox03j10-gmsl" },
	{ }
};
MODULE_DEVICE_TABLE(of, ox03j10_of_match);

static struct i2c_driver ox03j10_i2c_driver = {
	.driver = {
		.name = "ox03j10-gmsl",
		.of_match_table = ox03j10_of_match,
	},
	.probe = ox03j10_probe,
	.remove = ox03j10_remove,
};
module_i2c_driver(ox03j10_i2c_driver);

MODULE_DESCRIPTION("OX03J10 MAX96717/MAX9296 GMSL2 camera driver");
MODULE_AUTHOR("Reconstructed and adapted from supplied configuration");
MODULE_LICENSE("GPL");
