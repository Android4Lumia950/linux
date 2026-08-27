// SPDX-License-Identifier: GPL-2.0
/*
 * Microsoft Lumia 950 XL (Cityman) Sergej 1440p dual-DSI command-mode panel.
 *
 * Commands, timings, lanes, TE, reset, and DCS backlight come from 3.10
 * dsi-panel-sergej-wqhd-dualdsi-cmd.dtsi (mmo panels/). Dual-DSI attach
 * follows panel-microsoft-duke.c / panel-novatek-nt36523.c. Do not use the
 * Talkman Duke panel on Cityman.
 *
 * 3.10 DSI0 has qcom,cmd-sync-wait-broadcast; DSI1 has that plus
 * qcom,cmd-sync-wait-trigger. Mainline maps that to qcom,dual-dsi-mode +
 * qcom,sync-dual-dsi on both hosts and qcom,master-dsi on DSI0.
 *
 * Sergej on-command is not Duke: sleep wait is 5 ms (not 120), extra F2 63,
 * BD payload is 11 01 02 16 02 16, lock waits 23 ms. Off-command after DCS
 * 28 is DCS 10 (enter sleep, 120 ms), not the Duke gcdb leftover.
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

struct sergej_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	struct gpio_desc *reset_gpio;
};

static inline struct sergej_panel *to_sergej(struct drm_panel *panel)
{
	return container_of(panel, struct sergej_panel, panel);
}

static int sergej_dcs_write_buf(struct sergej_panel *ctx, const void *data,
				size_t len)
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

static int sergej_dcs_write_cmd(struct sergej_panel *ctx, u8 cmd)
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

static int sergej_reset(struct sergej_panel *ctx)
{
	/*
	 * 3.10 qcom,mdss-dsi-reset-sequence = <1 17>, <0 17>, <1 17> with
	 * platform-reset-gpio flags 0 (physical levels). GPIO_ACTIVE_LOW
	 * in DT: gpiod 0/1/0 is physical 1/0/1. Same gpio78 as octagon
	 * DISPLAY_RESET_N / mmo-common mdss.dtsi.
	 */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(17);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(17);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(17);
	return 0;
}

static int sergej_prepare(struct drm_panel *panel)
{
	struct sergej_panel *ctx = to_sergej(panel);
	int ret, i;
	static const u8 unlock[] = { 0xf0, 0x5a, 0x5a };
	static const u8 f2[] = { 0xf2, 0x63 };
	static const u8 bd[] = { 0xbd, 0x11, 0x01, 0x02, 0x16, 0x02, 0x16 };
	static const u8 lock[] = { 0xf0, 0xa5, 0xa5 };
	static const u8 ctrl[] = { 0x53, 0x20 };
	static const u8 bright[] = { 0x51, 0x80 };
	static const u8 te[] = { 0x35, 0x00 };

	sergej_reset(ctx);

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++)
		if (ctx->dsi[i])
			ctx->dsi[i]->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = sergej_dcs_write_cmd(ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	if (ret)
		return ret;
	/* 3.10 on-command wait field 0x05 after DCS 11 */
	msleep(5);

	ret = sergej_dcs_write_buf(ctx, unlock, sizeof(unlock));
	if (ret)
		return ret;
	ret = sergej_dcs_write_buf(ctx, f2, sizeof(f2));
	if (ret)
		return ret;
	ret = sergej_dcs_write_buf(ctx, bd, sizeof(bd));
	if (ret)
		return ret;
	ret = sergej_dcs_write_buf(ctx, lock, sizeof(lock));
	if (ret)
		return ret;
	/* 3.10 lock packet wait field 0x17 */
	msleep(23);
	ret = sergej_dcs_write_buf(ctx, ctrl, sizeof(ctrl));
	if (ret)
		return ret;
	ret = sergej_dcs_write_buf(ctx, bright, sizeof(bright));
	if (ret)
		return ret;
	ret = sergej_dcs_write_buf(ctx, te, sizeof(te));
	if (ret)
		return ret;
	ret = sergej_dcs_write_cmd(ctx, MIPI_DCS_SET_DISPLAY_ON);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++)
		if (ctx->dsi[i])
			ctx->dsi[i]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	return 0;
}

static int sergej_unprepare(struct drm_panel *panel)
{
	struct sergej_panel *ctx = to_sergej(panel);
	int i;

	/*
	 * 3.10 off-command-state is dsi_hs_mode. DCS 28 wait 0x21 = 33 ms,
	 * then DCS 10 wait 0x78 = 120 ms.
	 */
	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++)
		if (ctx->dsi[i])
			ctx->dsi[i]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	sergej_dcs_write_cmd(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(33);
	sergej_dcs_write_cmd(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(120);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	return 0;
}

/*
 * Bonded 1440-wide mode. Each 3.10 host is 720 wide with the same porches.
 * Pixel clock = (1440+76+16+32)*(2560+4+2+2)*60 = 240981000 Hz.
 * msm_dsi bonded path halves hdisplay for each controller.
 */
static const struct drm_display_mode sergej_mode = {
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

static int sergej_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &sergej_mode);
	if (!mode)
		return -ENOMEM;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);
	/* 3.10 qcom,mdss-pan-physical-*-dimension */
	connector->display_info.width_mm = 65;
	connector->display_info.height_mm = 115;
	return 1;
}

static const struct drm_panel_funcs sergej_funcs = {
	.prepare = sergej_prepare,
	.unprepare = sergej_unprepare,
	.get_modes = sergej_get_modes,
};

static int sergej_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct sergej_panel *ctx;
	struct device_node *dsi1;
	struct mipi_dsi_host *dsi1_host;
	const struct mipi_dsi_device_info info = {
		.type = "sergej",
		.channel = 0,
		.node = NULL,
	};
	int i, ret;

	ctx = devm_drm_panel_alloc(dev, struct sergej_panel, panel,
				   &sergej_funcs, DRM_MODE_CONNECTOR_DSI);
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
	 * it is not MIPI_DSI_MODE_VIDEO / VIDEO_BURST.
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

static void sergej_remove(struct mipi_dsi_device *dsi)
{
	struct sergej_panel *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id sergej_of_match[] = {
	{ .compatible = "microsoft,sergej-wqhd-cmd" },
	{ }
};
MODULE_DEVICE_TABLE(of, sergej_of_match);

static struct mipi_dsi_driver sergej_driver = {
	.driver = {
		.name = "panel-microsoft-sergej",
		.of_match_table = sergej_of_match,
	},
	.probe = sergej_probe,
	.remove = sergej_remove,
};
module_mipi_dsi_driver(sergej_driver);

MODULE_DESCRIPTION("Microsoft Lumia 950 XL Sergej dual-DSI panel");
MODULE_LICENSE("GPL");
