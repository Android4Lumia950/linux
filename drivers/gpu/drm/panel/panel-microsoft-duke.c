// SPDX-License-Identifier: GPL-2.0
/*
 * Microsoft Lumia 950 (Talkman) Duke 1440p dual-DSI command-mode panel.
 *
 * Commands, timings, lanes, TE, reset, and DCS backlight come from 3.10
 * dsi-panel-duke-wqhd-dualdsi-cmd.dtsi (mmo panels/). Dual-DSI attach
 * follows drivers/gpu/drm/panel/panel-novatek-nt36523.c. Do not use the
 * Cityman Sergej panel on Talkman.
 *
 * 3.10 DSI0 has qcom,cmd-sync-wait-broadcast; DSI1 has that plus
 * qcom,cmd-sync-wait-trigger. Mainline maps that to qcom,dual-dsi-mode +
 * qcom,sync-dual-dsi on both hosts and qcom,master-dsi on DSI0.
 *
 * 3.10 off-command after DCS 28 is the gcdb leftover
 * "00 24 0A 0A 26 25 09". That is not a DCS packet. Do not send it.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>

#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

struct duke_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	struct gpio_desc *reset_gpio;
};

static inline struct duke_panel *to_duke(struct drm_panel *panel)
{
	return container_of(panel, struct duke_panel, panel);
}

static int duke_dcs_write_buf(struct duke_panel *ctx, const void *data, size_t len)
{
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		if (!ctx->dsi[i])
			continue;
		ret = mipi_dsi_dcs_write_buffer(ctx->dsi[i], data, len);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int duke_dcs_write_cmd(struct duke_panel *ctx, u8 cmd)
{
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		if (!ctx->dsi[i])
			continue;
		ret = mipi_dsi_dcs_write(ctx->dsi[i], cmd, NULL, 0);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int duke_reset(struct duke_panel *ctx)
{
	/*
	 * 3.10 qcom,mdss-dsi-reset-sequence = <1 17>, <0 17>, <1 17> with
	 * platform-reset-gpio flags 0 (physical levels). GPIO_ACTIVE_LOW
	 * in DT: gpiod 0/1/0 is physical 1/0/1.
	 */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(17);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(17);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(17);
	return 0;
}

static int duke_prepare(struct drm_panel *panel)
{
	struct duke_panel *ctx = to_duke(panel);
	int ret, i;
	static const u8 unlock[] = { 0xf0, 0x5a, 0x5a };
	static const u8 bd[] = { 0xbd, 0x05, 0x02, 0x16 };
	static const u8 page[] = { 0xff, 0x02 };
	static const u8 lock[] = { 0xf0, 0xa5, 0xa5 };
	static const u8 ctrl[] = { 0x53, 0x20 };
	static const u8 bright[] = { 0x51, 0x80 };
	static const u8 te[] = { 0x35, 0x00 };

	duke_reset(ctx);

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++)
		if (ctx->dsi[i])
			ctx->dsi[i]->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = duke_dcs_write_cmd(ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	if (ret)
		return ret;
	msleep(120);

	ret = duke_dcs_write_buf(ctx, unlock, sizeof(unlock));
	if (ret)
		return ret;
	ret = duke_dcs_write_buf(ctx, bd, sizeof(bd));
	if (ret)
		return ret;
	ret = duke_dcs_write_buf(ctx, page, sizeof(page));
	if (ret)
		return ret;
	ret = duke_dcs_write_buf(ctx, lock, sizeof(lock));
	if (ret)
		return ret;
	ret = duke_dcs_write_buf(ctx, ctrl, sizeof(ctrl));
	if (ret)
		return ret;
	ret = duke_dcs_write_buf(ctx, bright, sizeof(bright));
	if (ret)
		return ret;
	ret = duke_dcs_write_buf(ctx, te, sizeof(te));
	if (ret)
		return ret;
	ret = duke_dcs_write_cmd(ctx, MIPI_DCS_SET_DISPLAY_ON);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++)
		if (ctx->dsi[i])
			ctx->dsi[i]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	return 0;
}

static int duke_unprepare(struct drm_panel *panel)
{
	struct duke_panel *ctx = to_duke(panel);
	int i;

	/*
	 * 3.10 off-command-state is dsi_hs_mode. Wait 0x21 = 33 ms after
	 * display off (gcdb wait field). Do not send the DSI0 trailing
	 * garbage bytes.
	 */
	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++)
		if (ctx->dsi[i])
			ctx->dsi[i]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	duke_dcs_write_cmd(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(33);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	return 0;
}

/*
 * Bonded 1440-wide mode. Each 3.10 host is 720 wide with the same porches.
 * Pixel clock = (1440+76+16+32)*(2560+4+2+2)*60 = 240981000 Hz.
 * msm_dsi bonded path halves hdisplay for each controller.
 */
static const struct drm_display_mode duke_mode = {
	.clock = 240981,
	.hdisplay = 1440,
	.hsync_start = 1440 + 76,
	.hsync_end = 1440 + 76 + 16,
	.htotal = 1440 + 76 + 16 + 32,
	.vdisplay = 2560,
	.vsync_start = 2560 + 4,
	.vsync_end = 2560 + 4 + 2,
	.vtotal = 2560 + 4 + 2 + 2,
};

static int duke_get_modes(struct drm_panel *panel,
			  struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &duke_mode);
	if (!mode)
		return -ENOMEM;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);
	connector->display_info.width_mm = 65;
	connector->display_info.height_mm = 115;
	return 1;
}

static const struct drm_panel_funcs duke_funcs = {
	.prepare = duke_prepare,
	.unprepare = duke_unprepare,
	.get_modes = duke_get_modes,
};

static int duke_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct duke_panel *ctx;
	struct device_node *dsi1;
	struct mipi_dsi_host *dsi1_host;
	const struct mipi_dsi_device_info info = {
		.type = "duke",
		.channel = 0,
		.node = NULL,
	};
	int i, ret;

	ctx = devm_drm_panel_alloc(dev, struct duke_panel, panel,
				   &duke_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio), "reset gpio\n");

	dsi1 = of_graph_get_remote_node(dev->of_node, 1, -1);
	if (!dsi1)
		return dev_err_probe(dev, -ENODEV, "no dsi1 remote\n");
	dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
	of_node_put(dsi1);
	if (!dsi1_host)
		return dev_err_probe(dev, -EPROBE_DEFER, "dsi1 host\n");

	ctx->dsi[1] = devm_mipi_dsi_device_register_full(dev, dsi1_host, &info);
	if (IS_ERR(ctx->dsi[1]))
		return dev_err_probe(dev, PTR_ERR(ctx->dsi[1]), "dsi1 device\n");

	ctx->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->panel.prepare_prev_first = true;
	drm_panel_add(&ctx->panel);

	/*
	 * 3.10: dsi_cmd_mode, 4 lanes, lane_map_0123, lane-hs=1.
	 * traffic-mode burst_mode is a 3.10 host field on this cmd panel;
	 * it is not MIPI_DSI_MODE_VIDEO / VIDEO_BURST. nt35950 command
	 * dual-DSI uses CLOCK_NON_CONTINUOUS | LPM without MODE_VIDEO.
	 */
	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		ctx->dsi[i]->lanes = 4;
		ctx->dsi[i]->format = MIPI_DSI_FMT_RGB888;
		ctx->dsi[i]->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS |
					  MIPI_DSI_MODE_LPM;
		ret = devm_mipi_dsi_attach(dev, ctx->dsi[i]);
		if (ret < 0)
			return dev_err_probe(dev, ret, "attach DSI%d\n", i);
	}

	return 0;
}

static void duke_remove(struct mipi_dsi_device *dsi)
{
	struct duke_panel *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id duke_of_match[] = {
	{ .compatible = "microsoft,duke-wqhd-cmd" },
	{ }
};
MODULE_DEVICE_TABLE(of, duke_of_match);

static struct mipi_dsi_driver duke_driver = {
	.driver = {
		.name = "panel-microsoft-duke",
		.of_match_table = duke_of_match,
	},
	.probe = duke_probe,
	.remove = duke_remove,
};
module_mipi_dsi_driver(duke_driver);

MODULE_DESCRIPTION("Microsoft Lumia 950 Duke dual-DSI panel");
MODULE_LICENSE("GPL");
