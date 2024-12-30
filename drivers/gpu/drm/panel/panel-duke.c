// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Girjeu Efim <efimgirjeu3@gmail.com>
 *
 * Generated using linux-mdss-dsi-panel-driver-generator from Lineage OS device tree:
 * https://github.com/Android4Lumia/android_kernel_msft_talkman/blob/lineage-18.1/arch/arm/boot/dts/lumia/msm8992-chi.dtsi
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct duke {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
};

static inline struct duke *to_duke(struct drm_panel *panel)
{
	return container_of(panel, struct duke, panel);
}

static void duke_reset(struct duke *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(17000, 18000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(17000, 18000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(17000, 18000);
}

static int duke_on(struct duke *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbd, 0x05, 0x02, 0x16);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x20);
	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0x0080);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int duke_off(struct duke *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 33);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int duke_prepare(struct drm_panel *panel)
{
	struct duke *ctx = to_duke(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	duke_reset(ctx);

	ret = duke_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	return 0;
}

static int duke_unprepare(struct drm_panel *panel)
{
	struct duke *ctx = to_duke(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = duke_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static const struct drm_display_mode duke_mode = {
	.clock = (1440 + 76 + 16 + 32) * (2560 + 4 + 2 + 2) * 60 / 1000,
	.hdisplay = 1440,
	.hsync_start = 1440 + 76,
	.hsync_end = 1440 + 76 + 16,
	.htotal = 1440 + 76 + 16 + 32,
	.vdisplay = 2560,
	.vsync_start = 2560 + 4,
	.vsync_end = 2560 + 4 + 2,
	.vtotal = 2560 + 4 + 2 + 2,
	.width_mm = 65,
	.height_mm = 115,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int duke_get_modes(struct drm_panel *panel,
			  struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &duke_mode);
}

static const struct drm_panel_funcs duke_panel_funcs = {
	.prepare = duke_prepare,
	.unprepare = duke_unprepare,
	.get_modes = duke_get_modes,
};

static int duke_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

// TODO: Check if /sys/class/backlight/.../actual_brightness actually returns
// correct values. If not, remove this function.
static int duke_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness & 0xff;
}

static const struct backlight_ops duke_bl_ops = {
	.update_status = duke_bl_update_status,
	.get_brightness = duke_bl_get_brightness,
};

static struct backlight_device *
duke_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 255,
		.max_brightness = 255,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &duke_bl_ops, &props);
}

static int duke_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi1;
	struct mipi_dsi_host *dsi1_host;
	struct duke *ctx;
	const struct mipi_dsi_device_info *info;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_VIDEO_HSE |
			  MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	drm_panel_init(&ctx->panel, dev, &duke_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = duke_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);


	return 0;
}

static void duke_remove(struct mipi_dsi_device *dsi)
{
	struct duke *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id duke_of_match[] = {
	{ .compatible = "msft,duke" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, duke_of_match);

static struct mipi_dsi_driver duke_driver = {
	.probe = duke_probe,
	.remove = duke_remove,
	.driver = {
		.name = "panel-duke",
		.of_match_table = duke_of_match,
	},
};
module_mipi_dsi_driver(duke_driver);

MODULE_AUTHOR("snaccy <efimgirjeu3@gmail.com>"); // FIXME
MODULE_DESCRIPTION("DRM driver for Duke 1440p Dual DSI panel, command mode");
MODULE_LICENSE("GPL");
