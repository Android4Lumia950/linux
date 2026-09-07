// SPDX-License-Identifier: GPL-2.0
/*
 * WCD9330 (tomtom) SLIMbus codec. Sequences and registers are from
 * 3.10 wcd9330.c / wcd9xxx-core.c / wcd9xxx-resmgr.c / wcd9xxx-mbhc.c
 * / wcd9xxx-slimslave.c. Mainline slim / ASoC / regmap only. This
 * is slim217,130, not WCD9335 (slim217,1a0).
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slimbus.h>
#include <linux/string.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include "../qcom/qdsp6/q6afe.h"
#include "wcd9330.h"

#define WCD9330_RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
				 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 | \
				 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)
#define WCD9330_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

#define TOMTOM_SLIM_RX_CH(p)	{ .port = (p) + TOMTOM_RX_PORT_START, .shift = (p) }

enum {
	AIF1_PB,
	NUM_CODEC_DAIS,
};

enum {
	RX_MIX1_INP_SEL_ZERO = 0,
	RX_MIX1_INP_SEL_SRC1,
	RX_MIX1_INP_SEL_SRC2,
	RX_MIX1_INP_SEL_IIR1,
	RX_MIX1_INP_SEL_IIR2,
	RX_MIX1_INP_SEL_RX1,
	RX_MIX1_INP_SEL_RX2,
	RX_MIX1_INP_SEL_RX3,
	RX_MIX1_INP_SEL_RX4,
	RX_MIX1_INP_SEL_RX5,
	RX_MIX1_INP_SEL_RX6,
	RX_MIX1_INP_SEL_RX7,
};

struct wcd9330_slim_ch {
	u32 ch_num;
	u16 port;
	u16 shift;
	struct list_head list;
};

struct wcd9330_dai_data {
	struct list_head slim_ch_list;
	struct slim_stream_config sconfig;
	struct slim_stream_runtime *sruntime;
	bool slim_prepared;
	bool slim_on;
};

struct wcd9330_codec {
	struct device *dev;
	struct slim_device *slim;
	struct slim_device *slim_ifc_dev;
	struct regmap *regmap;
	struct regmap *if_regmap;
	struct gpio_desc *reset_gpio;
	struct clk *mclk;
	struct snd_soc_component *component;
	struct snd_soc_jack *hs_jack;
	int irq;
	bool insert_inverted;
	int clk_mclk_users;
	int bg_users;
	int rx_bias_users;
	int cp_users;
	u8 clsh_state;
	int clsh_users;
	int buck_users;
	int ncp_users[2];
	u32 rx_port_value[TOMTOM_RX_MAX];
	struct wcd9330_slim_ch rx_chs[TOMTOM_RX_MAX];
	struct wcd9330_dai_data dai[NUM_CODEC_DAIS];
	int num_rx_port;
};

/*
 * 3.10 msm8994.dtsi qcom,cdc-static-supplies. Rails on this board
 * are the existing octagon RPM nodes (s5 2.15 V, s4 1.8 V, l11 1.2 V).
 */
static const char * const wcd9330_supplies[] = {
	"vdd-buck", "vdd-tx-h", "vdd-rx-h", "vdd-px",
	"vdd-a-1p2v", "vdd-cx1", "vdd-cx2",
};

static const struct wcd9330_slim_ch wcd9330_rx_chs[TOMTOM_RX_MAX] = {
	TOMTOM_SLIM_RX_CH(0),
	TOMTOM_SLIM_RX_CH(1),
	TOMTOM_SLIM_RX_CH(2),
	TOMTOM_SLIM_RX_CH(3),
	TOMTOM_SLIM_RX_CH(4),
	TOMTOM_SLIM_RX_CH(5),
	TOMTOM_SLIM_RX_CH(6),
	TOMTOM_SLIM_RX_CH(7),
	TOMTOM_SLIM_RX_CH(8),
	TOMTOM_SLIM_RX_CH(9),
	TOMTOM_SLIM_RX_CH(10),
	TOMTOM_SLIM_RX_CH(11),
	TOMTOM_SLIM_RX_CH(12),
};

/* 3.10 wcd9330.c: digital_gain / line_gain */
static const DECLARE_TLV_DB_SCALE(digital_gain, 0, 1, 0);
static const DECLARE_TLV_DB_SCALE(line_gain, 0, 700, 1);

/* 3.10 rx_digital_gain_reg — RX1/RX2 only on this DAI */
static const u16 wcd9330_rx_digital_gain_reg[] = {
	TOMTOM_A_CDC_RX1_VOL_CTL_B2_CTL,
	TOMTOM_A_CDC_RX2_VOL_CTL_B2_CTL,
};

static int wcd9330_slim_read(void *context, const void *reg, size_t reg_size,
			     void *val, size_t val_size)
{
	struct slim_device *sdev = context;
	unsigned int addr = *(const u16 *)reg;
	int ret;

	ret = slim_read(sdev, TOMTOM_REGISTER_START_OFFSET + addr,
			val_size, val);
	if (ret)
		dev_err_ratelimited(&sdev->dev, "slim read 0x%x: %d\n",
				    addr, ret);
	return ret;
}

static int wcd9330_slim_write(void *context, const void *data, size_t count)
{
	struct slim_device *sdev = context;
	unsigned int addr = *(const u16 *)data;
	const u8 *val = data + sizeof(u16);
	int ret;

	ret = slim_write(sdev, TOMTOM_REGISTER_START_OFFSET + addr,
			 count - sizeof(u16), (u8 *)val);
	if (ret)
		dev_err_ratelimited(&sdev->dev, "slim write 0x%x: %d\n",
				    addr, ret);
	return ret;
}

static const struct regmap_bus wcd9330_slim_bus = {
	.read = wcd9330_slim_read,
	.write = wcd9330_slim_write,
	.reg_format_endian_default = REGMAP_ENDIAN_LITTLE,
	.val_format_endian_default = REGMAP_ENDIAN_LITTLE,
};

static const struct regmap_config wcd9330_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = TOMTOM_MAX_REGISTER,
	/* 3.10 POR table is not in this driver; hit the slim device. */
	.cache_type = REGCACHE_NONE,
};

static int wcd9330_enable_bg(struct wcd9330_codec *wcd, bool enable)
{
	struct regmap *rm = wcd->regmap;

	if (enable) {
		if (++wcd->bg_users != 1)
			return 0;
		/* 3.10 wcd9xxx_enable_bg: slow mode + precharge */
		regmap_update_bits(rm, TOMTOM_A_BIAS_CENTRAL_BG_CTL, 0x80, 0x80);
		regmap_update_bits(rm, TOMTOM_A_BIAS_CENTRAL_BG_CTL, 0x04, 0x04);
		regmap_update_bits(rm, TOMTOM_A_BIAS_CENTRAL_BG_CTL, 0x01, 0x01);
		usleep_range(1000, 1100);
		regmap_update_bits(rm, TOMTOM_A_BIAS_CENTRAL_BG_CTL, 0x80, 0x00);
	} else if (wcd->bg_users > 0 && --wcd->bg_users == 0) {
		regmap_update_bits(rm, TOMTOM_A_BIAS_CENTRAL_BG_CTL, 0x03, 0x00);
		usleep_range(100, 110);
	}

	return 0;
}

static int wcd9330_enable_mclk(struct wcd9330_codec *wcd, bool enable)
{
	struct regmap *rm = wcd->regmap;
	int ret;

	if (enable) {
		if (++wcd->clk_mclk_users != 1)
			return 0;
		ret = clk_prepare_enable(wcd->mclk);
		if (ret) {
			wcd->clk_mclk_users--;
			return ret;
		}
		wcd9330_enable_bg(wcd, true);
		/* 3.10 wcd9xxx_enable_clock_block MCLK */
		regmap_update_bits(rm, TOMTOM_A_CLK_BUFF_EN1, 0x01, 0x01);
		usleep_range(1000, 1200);
		regmap_update_bits(rm, TOMTOM_A_CLK_BUFF_EN2, 0x02, 0x00);
		regmap_update_bits(rm, TOMTOM_A_CLK_BUFF_EN2, 0x04, 0x04);
		regmap_update_bits(rm, TOMTOM_A_CDC_CLK_MCLK_CTL, 0x01, 0x01);
		usleep_range(50, 55);
	} else if (wcd->clk_mclk_users > 0 && --wcd->clk_mclk_users == 0) {
		/* 3.10 wcd9xxx_disable_clock_block TOMTOM */
		regmap_update_bits(rm, TOMTOM_A_CLK_BUFF_EN2, 0x04, 0x00);
		usleep_range(50, 55);
		regmap_update_bits(rm, TOMTOM_A_CLK_BUFF_EN2, 0x02, 0x02);
		regmap_update_bits(rm, TOMTOM_A_CLK_BUFF_EN1, 0x40, 0x40);
		regmap_update_bits(rm, TOMTOM_A_CLK_BUFF_EN1, 0x40, 0x00);
		regmap_update_bits(rm, TOMTOM_A_CLK_BUFF_EN1, 0x01, 0x00);
		usleep_range(50, 55);
		wcd9330_enable_bg(wcd, false);
		clk_disable_unprepare(wcd->mclk);
	}

	return 0;
}

static int wcd9330_codec_enable_mclk(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(comp);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		return wcd9330_enable_mclk(wcd, true);
	case SND_SOC_DAPM_POST_PMD:
		return wcd9330_enable_mclk(wcd, false);
	}
	return 0;
}

static int wcd9330_codec_enable_rx_bias(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(comp);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (++wcd->rx_bias_users == 1)
			snd_soc_component_update_bits(comp, TOMTOM_A_RX_COM_BIAS,
						      0x80, 0x80);
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (wcd->rx_bias_users > 0 && --wcd->rx_bias_users == 0)
			snd_soc_component_update_bits(comp, TOMTOM_A_RX_COM_BIAS,
						      0x80, 0x00);
		break;
	}
	return 0;
}

static int wcd9330_codec_enable_interpolator(struct snd_soc_dapm_widget *w,
					     struct snd_kcontrol *kcontrol,
					     int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_CLK_RX_RESET_CTL,
					      1 << w->shift, 1 << w->shift);
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_CLK_RX_RESET_CTL,
					      1 << w->shift, 0);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/*
		 * 3.10 tomtom_codec_enable_interpolator: apply the
		 * digital gain after the interpolator is enabled.
		 */
		if (w->shift < ARRAY_SIZE(wcd9330_rx_digital_gain_reg))
			snd_soc_component_write(comp,
				wcd9330_rx_digital_gain_reg[w->shift],
				snd_soc_component_read(comp,
					wcd9330_rx_digital_gain_reg[w->shift]));
		break;
	}
	return 0;
}

/*
 * 3.10 tomtom_codec_dsm_mux_event. CLASS_H_DSM bits 5:4 select
 * DSM_HPHL_RX1 / DSM_SPKR_RX7; bits 3:2 are the ZOH feed into
 * Class H. Without ZOH, HPH PAs have no DSM reference.
 */
static int wcd9330_codec_dsm_mux_event(struct snd_soc_dapm_widget *w,
				       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	u8 reg_val, zoh_mux_val = 0x00;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		reg_val = snd_soc_component_read(comp, TOMTOM_A_CDC_CONN_CLSH_CTL);
		if ((reg_val & 0x30) == 0x10)
			zoh_mux_val = 0x04;
		else if ((reg_val & 0x30) == 0x20)
			zoh_mux_val = 0x08;
		if (zoh_mux_val)
			snd_soc_component_update_bits(comp,
						      TOMTOM_A_CDC_CONN_CLSH_CTL,
						      0x0C, zoh_mux_val);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_CONN_CLSH_CTL,
					      0x0C, 0x00);
		break;
	}
	return 0;
}

/*
 * 3.10 wcd9xxx-common.c Class H. high_perf_mode defaults to 0, so
 * HPH uses wcd9xxx_clsh_fsm, not wcd9xxx_enable_high_perf_mode.
 * HPH-only (no EAR/LO). clsh_comp_req's resmgr cond writes
 * CLSH_B1_CTL bits 3/2; that is the register effect here.
 */
#define WCD9330_CLSH_IDLE		0x00
#define WCD9330_CLSH_HPHL		(1 << 1)
#define WCD9330_CLSH_HPHR		(1 << 2)
#define WCD9330_CLSH_HPH_ST		(WCD9330_CLSH_HPHL | WCD9330_CLSH_HPHR)
#define WCD9330_CLSH_PRE_DAC		0x01
#define WCD9330_CLSH_POST_PA		0x02
#define WCD9330_NCP_FCLK8		0
#define WCD9330_NCP_FCLK5		1
#define WCD9330_BUCK_VREF_0P494V	0x3f
#define WCD9330_BUCK_SETTLE_US		50
#define WCD9330_NCP_SETTLE_US		50

struct wcd9330_reg_mask_val {
	u16 reg;
	u8 mask;
	u8 val;
};

static void wcd9330_update_set(struct snd_soc_component *comp,
			       const struct wcd9330_reg_mask_val *set, int n)
{
	int i;

	for (i = 0; i < n; i++)
		snd_soc_component_update_bits(comp, set[i].reg,
					      set[i].mask, set[i].val);
}

static void wcd9330_enable_clsh_block(struct snd_soc_component *comp,
				      struct wcd9330_codec *wcd, bool enable)
{
	if ((enable && ++wcd->clsh_users == 1) ||
	    (!enable && --wcd->clsh_users == 0))
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_CLSH_B1_CTL,
					      0x01, enable ? 0x01 : 0x00);
}

static void wcd9330_enable_anc_delay(struct snd_soc_component *comp, bool on)
{
	snd_soc_component_update_bits(comp, TOMTOM_A_CDC_CLSH_B1_CTL,
				      0x02, on ? 0x02 : 0x00);
}

static void wcd9330_enable_buck(struct snd_soc_component *comp,
				struct wcd9330_codec *wcd, bool enable)
{
	if ((enable && ++wcd->buck_users == 1) ||
	    (!enable && --wcd->buck_users == 0))
		snd_soc_component_update_bits(comp, TOMTOM_A_BUCK_MODE_1,
					      0x80, enable ? 0x80 : 0x00);
}

static void wcd9330_chargepump_request(struct snd_soc_component *comp,
				       struct wcd9330_codec *wcd, bool on)
{
	if (on && ++wcd->cp_users == 1)
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_CLK_OTHR_CTL,
					      0x01, 0x01);
	else if (!on && wcd->cp_users > 0 && --wcd->cp_users == 0)
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_CLK_OTHR_CTL,
					      0x01, 0x00);
}

static void wcd9330_cfg_clsh_param_common(struct snd_soc_component *comp)
{
	static const struct wcd9330_reg_mask_val reg_set[] = {
		{ TOMTOM_A_CDC_CLSH_BUCK_NCP_VARS, 0x03, 0x00 },
		{ TOMTOM_A_CDC_CLSH_BUCK_NCP_VARS, 0x0c, 0x04 },
		{ TOMTOM_A_CDC_CLSH_BUCK_NCP_VARS, 0x10, 0x00 },
		{ TOMTOM_A_CDC_CLSH_B2_CTL, 0x03, 0x01 },
		{ TOMTOM_A_CDC_CLSH_B2_CTL, 0x0c, 0x04 },
		{ TOMTOM_A_CDC_CLSH_B2_CTL, 0xf0, 0x30 },
		{ TOMTOM_A_CDC_CLSH_B3_CTL, 0xf0, 0x30 },
		{ TOMTOM_A_CDC_CLSH_B3_CTL, 0x0f, 0x0b },
		{ TOMTOM_A_CDC_CLSH_B1_CTL, 0x20, 0x20 },
		{ TOMTOM_A_CDC_CLSH_B1_CTL, 0x02, 0x02 },
	};

	wcd9330_update_set(comp, reg_set, ARRAY_SIZE(reg_set));
}

static void wcd9330_cfg_clsh_param_hph(struct snd_soc_component *comp)
{
	static const struct wcd9330_reg_mask_val reg_set[] = {
		{ TOMTOM_A_CDC_CLSH_B1_CTL, 0x40, 0x00 },
		{ TOMTOM_A_CDC_CLSH_V_PA_HD_HPH, 0x3f, 0x0d },
		{ TOMTOM_A_CDC_CLSH_V_PA_MIN_HPH, 0x3f, 0x1d },
		{ TOMTOM_A_CDC_CLSH_IDLE_HPH_THSD, 0x3f, 0x13 },
		{ TOMTOM_A_CDC_CLSH_FCLKONLY_HPH_THSD, 0x1f, 0x19 },
		{ TOMTOM_A_CDC_CLSH_I_PA_FACT_HPH_L, 0xff, 0x97 },
		{ TOMTOM_A_CDC_CLSH_I_PA_FACT_HPH_U, 0xff, 0x05 },
		{ TOMTOM_A_CDC_CLSH_K_ADDR, 0x80, 0x00 },
		{ TOMTOM_A_CDC_CLSH_K_ADDR, 0x0f, 0x00 },
		{ TOMTOM_A_CDC_CLSH_K_DATA, 0xff, 0xae },
		{ TOMTOM_A_CDC_CLSH_K_DATA, 0xff, 0x01 },
		{ TOMTOM_A_CDC_CLSH_K_DATA, 0xff, 0x1c },
		{ TOMTOM_A_CDC_CLSH_K_DATA, 0xff, 0x00 },
		{ TOMTOM_A_CDC_CLSH_K_DATA, 0xff, 0x24 },
		{ TOMTOM_A_CDC_CLSH_K_DATA, 0xff, 0x00 },
		{ TOMTOM_A_CDC_CLSH_K_DATA, 0xff, 0x25 },
		{ TOMTOM_A_CDC_CLSH_K_DATA, 0xff, 0x00 },
	};

	wcd9330_update_set(comp, reg_set, ARRAY_SIZE(reg_set));
}

static void wcd9330_clsh_comp_req(struct snd_soc_component *comp, int shift,
				  bool on)
{
	snd_soc_component_update_bits(comp, TOMTOM_A_CDC_CLSH_B1_CTL,
				      1 << shift, on ? (1 << shift) : 0);
}

static void wcd9330_set_buck_mode(struct snd_soc_component *comp, u8 buck_vref)
{
	static const struct wcd9330_reg_mask_val first[] = {
		{ TOMTOM_A_BUCK_MODE_5, 0x02, 0x02 },
	};

	wcd9330_update_set(comp, first, ARRAY_SIZE(first));
	snd_soc_component_update_bits(comp, TOMTOM_A_BUCK_MODE_4, 0xff, buck_vref);
	snd_soc_component_update_bits(comp, TOMTOM_A_BUCK_MODE_1, 0x04, 0x04);
	snd_soc_component_update_bits(comp, TOMTOM_A_BUCK_MODE_3, 0x04, 0x00);
	snd_soc_component_update_bits(comp, TOMTOM_A_BUCK_MODE_3, 0x08, 0x00);
	usleep_range(WCD9330_BUCK_SETTLE_US, WCD9330_BUCK_SETTLE_US + 10);
}

static void wcd9330_set_fclk_get_ncp(struct snd_soc_component *comp,
				     struct wcd9330_codec *wcd, int fclk)
{
	wcd->ncp_users[fclk]++;
	snd_soc_component_update_bits(comp, TOMTOM_A_NCP_STATIC, 0x10, 0x00);
	if (wcd->ncp_users[WCD9330_NCP_FCLK8] > 0)
		snd_soc_component_update_bits(comp, TOMTOM_A_NCP_STATIC,
					      0x0f, 0x08);
	else if (wcd->ncp_users[WCD9330_NCP_FCLK5] > 0)
		snd_soc_component_update_bits(comp, TOMTOM_A_NCP_STATIC,
					      0x0f, 0x05);
	snd_soc_component_update_bits(comp, TOMTOM_A_NCP_STATIC, 0x20, 0x20);
	if (snd_soc_component_update_bits(comp, TOMTOM_A_NCP_EN, 0x01, 0x01))
		usleep_range(WCD9330_NCP_SETTLE_US, WCD9330_NCP_SETTLE_US + 50);
}

static void wcd9330_set_fclk_put_ncp(struct snd_soc_component *comp,
				     struct wcd9330_codec *wcd, int fclk)
{
	if (wcd->ncp_users[fclk] > 0)
		wcd->ncp_users[fclk]--;
	if (!wcd->ncp_users[WCD9330_NCP_FCLK8] &&
	    !wcd->ncp_users[WCD9330_NCP_FCLK5])
		snd_soc_component_update_bits(comp, TOMTOM_A_NCP_EN, 0x01, 0x00);
	else if (!wcd->ncp_users[WCD9330_NCP_FCLK8])
		snd_soc_component_update_bits(comp, TOMTOM_A_NCP_STATIC,
					      0x0f, 0x05);
}

static void wcd9330_clsh_enable_post_pa(struct snd_soc_component *comp,
					struct wcd9330_codec *wcd)
{
	static const struct wcd9330_reg_mask_val reg_set[] = {
		{ TOMTOM_A_BUCK_MODE_5, 0x02, 0x00 },
		{ TOMTOM_A_NCP_STATIC, 0x20, 0x00 },
		{ TOMTOM_A_BUCK_MODE_3, 0x04, 0x04 },
	};

	wcd9330_update_set(comp, reg_set, ARRAY_SIZE(reg_set));
	/* TomTom: is_dynamic_vdd_cp is false */
	snd_soc_component_update_bits(comp, TOMTOM_A_BUCK_MODE_3, 0x08, 0x08);
}

static void wcd9330_clsh_state_hph(struct snd_soc_component *comp,
				   struct wcd9330_codec *wcd, bool enable)
{
	if (enable) {
		wcd9330_cfg_clsh_param_common(comp);
		wcd9330_cfg_clsh_param_hph(comp);
		wcd9330_enable_clsh_block(comp, wcd, true);
		wcd9330_chargepump_request(comp, wcd, true);
		wcd9330_enable_anc_delay(comp, true);
		wcd9330_clsh_comp_req(comp, 3, true);
		wcd9330_clsh_comp_req(comp, 2, true);
		wcd9330_set_buck_mode(comp, WCD9330_BUCK_VREF_0P494V);
		wcd9330_enable_buck(comp, wcd, true);
		wcd9330_set_fclk_get_ncp(comp, wcd, WCD9330_NCP_FCLK8);
	} else {
		wcd9330_set_fclk_put_ncp(comp, wcd, WCD9330_NCP_FCLK8);
		wcd9330_enable_buck(comp, wcd, false);
		wcd9330_clsh_comp_req(comp, 3, false);
		wcd9330_clsh_comp_req(comp, 2, false);
		wcd9330_enable_clsh_block(comp, wcd, false);
		wcd9330_chargepump_request(comp, wcd, false);
	}
}

/* 3.10 wcd9xxx_clsh_fsm — HPHL / HPHR / HPH_ST only. */
static void wcd9330_clsh_fsm(struct snd_soc_component *comp, u8 req_state,
			     bool enable, u8 event)
{
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(comp);
	u8 old_state = wcd->clsh_state;
	u8 new_state;

	if (event == WCD9330_CLSH_PRE_DAC) {
		if (!enable)
			return;
		new_state = old_state | req_state;
		if (new_state == old_state)
			return;
		/* 3.10 hph_st enable is a no-op; first side programs CLSH. */
		if (old_state == WCD9330_CLSH_IDLE &&
		    (new_state == WCD9330_CLSH_HPHL ||
		     new_state == WCD9330_CLSH_HPHR))
			wcd9330_clsh_state_hph(comp, wcd, true);
		wcd->clsh_state = new_state;
	} else if (event == WCD9330_CLSH_POST_PA) {
		if (enable) {
			wcd9330_clsh_enable_post_pa(comp, wcd);
			return;
		}
		new_state = old_state & ~req_state;
		if (new_state == old_state)
			return;
		if (old_state == WCD9330_CLSH_HPHL ||
		    old_state == WCD9330_CLSH_HPHR)
			wcd9330_clsh_state_hph(comp, wcd, false);
		wcd->clsh_state = new_state;
	}
}

static int wcd9330_hphl_dac_event(struct snd_soc_dapm_widget *w,
				  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd9330_clsh_fsm(comp, WCD9330_CLSH_HPHL, true,
				 WCD9330_CLSH_PRE_DAC);
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_RX1_B3_CTL,
					      0xbc, 0x94);
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_RX1_B4_CTL,
					      0x30, 0x10);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_RX1_B3_CTL,
					      0xbc, 0x00);
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_RX1_B4_CTL,
					      0x30, 0x00);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd9330_clsh_fsm(comp, WCD9330_CLSH_HPHL, false,
				 WCD9330_CLSH_POST_PA);
		break;
	}
	return 0;
}

static int wcd9330_hphr_dac_event(struct snd_soc_dapm_widget *w,
				  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(comp, w->reg, 0x40, 0x40);
		wcd9330_clsh_fsm(comp, WCD9330_CLSH_HPHR, true,
				 WCD9330_CLSH_PRE_DAC);
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_RX2_B3_CTL,
					      0xbc, 0x94);
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_RX2_B4_CTL,
					      0x30, 0x10);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_RX2_B3_CTL,
					      0xbc, 0x00);
		snd_soc_component_update_bits(comp, TOMTOM_A_CDC_RX2_B4_CTL,
					      0x30, 0x00);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(comp, w->reg, 0x40, 0x00);
		wcd9330_clsh_fsm(comp, WCD9330_CLSH_HPHR, false,
				 WCD9330_CLSH_POST_PA);
		break;
	}
	return 0;
}

static int wcd9330_hph_pa_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	u8 req = (w->shift == 5) ? WCD9330_CLSH_HPHL : WCD9330_CLSH_HPHR;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(TOMTOM_HPH_PA_SETTLE_US,
			     TOMTOM_HPH_PA_SETTLE_US + 1000);
		wcd9330_clsh_fsm(comp, req, true, WCD9330_CLSH_POST_PA);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		usleep_range(TOMTOM_HPH_PA_SETTLE_US,
			     TOMTOM_HPH_PA_SETTLE_US + 1000);
		break;
	}
	return 0;
}

static int slim_rx_mux_get(struct snd_kcontrol *kc,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *w = snd_soc_dapm_kcontrol_to_widget(kc);
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(
					snd_soc_dapm_to_component(w->dapm));

	ucontrol->value.enumerated.item[0] = wcd->rx_port_value[w->shift];
	return 0;
}

static int slim_rx_mux_put(struct snd_kcontrol *kc,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *w = snd_soc_dapm_kcontrol_to_widget(kc);
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(
					snd_soc_dapm_to_component(w->dapm));
	struct soc_enum *e = (struct soc_enum *)kc->private_value;
	u32 port_id = w->shift;

	if (wcd->rx_port_value[port_id] == ucontrol->value.enumerated.item[0])
		return 0;

	wcd->rx_port_value[port_id] = ucontrol->value.enumerated.item[0];
	list_del_init(&wcd->rx_chs[port_id].list);

	switch (wcd->rx_port_value[port_id]) {
	case 0:
		break;
	case 1:
		list_add_tail(&wcd->rx_chs[port_id].list,
			      &wcd->dai[AIF1_PB].slim_ch_list);
		break;
	default:
		return -EINVAL;
	}

	snd_soc_dapm_mux_update_power(w->dapm, kc,
				      wcd->rx_port_value[port_id], e, NULL);
	return 0;
}

static const char * const slim_rx_mux_text[] = {
	"ZERO", "AIF1_PB",
};

static const struct soc_enum slim_rx_mux_enum =
	SOC_ENUM_SINGLE_VIRT(ARRAY_SIZE(slim_rx_mux_text), slim_rx_mux_text);

static const struct snd_kcontrol_new slim_rx_mux[2] = {
	SOC_DAPM_ENUM_EXT("SLIM RX1 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX2 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
};

static const char * const rx_mix1_text[] = {
	"ZERO", "SRC1", "SRC2", "IIR1", "IIR2", "RX1", "RX2", "RX3", "RX4",
	"RX5", "RX6", "RX7"
};

static const struct soc_enum rx1_mix1_inp1_enum =
	SOC_ENUM_SINGLE(TOMTOM_A_CDC_CONN_RX1_B1_CTL, 0, 12, rx_mix1_text);
static const struct soc_enum rx1_mix1_inp2_enum =
	SOC_ENUM_SINGLE(TOMTOM_A_CDC_CONN_RX1_B1_CTL, 4, 12, rx_mix1_text);
static const struct soc_enum rx2_mix1_inp1_enum =
	SOC_ENUM_SINGLE(TOMTOM_A_CDC_CONN_RX2_B1_CTL, 0, 12, rx_mix1_text);
static const struct soc_enum rx2_mix1_inp2_enum =
	SOC_ENUM_SINGLE(TOMTOM_A_CDC_CONN_RX2_B1_CTL, 4, 12, rx_mix1_text);

static const struct snd_kcontrol_new rx1_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP1 Mux", rx1_mix1_inp1_enum);
static const struct snd_kcontrol_new rx1_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP2 Mux", rx1_mix1_inp2_enum);
static const struct snd_kcontrol_new rx2_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP1 Mux", rx2_mix1_inp1_enum);
static const struct snd_kcontrol_new rx2_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP2 Mux", rx2_mix1_inp2_enum);

static const char * const class_h_dsm_text[] = {
	"ZERO", "DSM_HPHL_RX1", "DSM_SPKR_RX7"
};

static const struct soc_enum class_h_dsm_enum =
	SOC_ENUM_SINGLE(TOMTOM_A_CDC_CONN_CLSH_CTL, 4, 3, class_h_dsm_text);

static const struct snd_kcontrol_new class_h_dsm_mux =
	SOC_DAPM_ENUM("CLASS_H_DSM MUX Mux", class_h_dsm_enum);

static const char * const rx1_interp_text[] = { "ZERO", "RX1 MIX2" };
static const char * const rx2_interp_text[] = { "ZERO", "RX2 MIX2" };

static const struct soc_enum rx1_interp_enum =
	SOC_ENUM_SINGLE(TOMTOM_A_CDC_CLK_RX_B1_CTL, 0, 2, rx1_interp_text);
static const struct soc_enum rx2_interp_enum =
	SOC_ENUM_SINGLE(TOMTOM_A_CDC_CLK_RX_B1_CTL, 1, 2, rx2_interp_text);

static const struct snd_kcontrol_new rx1_interp_mux =
	SOC_DAPM_ENUM("RX1 INTERP MUX Mux", rx1_interp_enum);
static const struct snd_kcontrol_new rx2_interp_mux =
	SOC_DAPM_ENUM("RX2 INTERP MUX Mux", rx2_interp_enum);

static const struct snd_kcontrol_new hphl_switch[] = {
	SOC_DAPM_SINGLE("Switch", TOMTOM_A_RX_HPH_L_DAC_CTL, 6, 1, 0)
};

static const struct snd_kcontrol_new wcd9330_snd_controls[] = {
	SOC_SINGLE_SX_TLV("RX1 Digital Volume", TOMTOM_A_CDC_RX1_VOL_CTL_B2_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX2 Digital Volume", TOMTOM_A_CDC_RX2_VOL_CTL_B2_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_TLV("HPHL Volume", TOMTOM_A_RX_HPH_L_GAIN, 0, 20, 1, line_gain),
	SOC_SINGLE_TLV("HPHR Volume", TOMTOM_A_RX_HPH_R_GAIN, 0, 20, 1, line_gain),
};

static const struct snd_soc_dapm_widget wcd9330_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIF1 PB", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MUX("SLIM RX1 MUX", SND_SOC_NOPM, 0, 0, &slim_rx_mux[0]),
	SND_SOC_DAPM_MUX("SLIM RX2 MUX", SND_SOC_NOPM, 1, 0, &slim_rx_mux[1]),
	SND_SOC_DAPM_MIXER("SLIM RX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX2", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("RX1 MIX1 INP1", SND_SOC_NOPM, 0, 0, &rx1_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX1 MIX1 INP2", SND_SOC_NOPM, 0, 0, &rx1_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP1", SND_SOC_NOPM, 0, 0, &rx2_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP2", SND_SOC_NOPM, 0, 0, &rx2_mix1_inp2_mux),
	SND_SOC_DAPM_MIXER("RX1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX2 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX1 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX2 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX_E("RX1 INTERP", TOMTOM_A_CDC_CLK_RX_B1_CTL, 0, 0,
			   &rx1_interp_mux, wcd9330_codec_enable_interpolator,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MUX_E("RX2 INTERP", TOMTOM_A_CDC_CLK_RX_B1_CTL, 1, 0,
			   &rx2_interp_mux, wcd9330_codec_enable_interpolator,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER("RX1 CHAIN", TOMTOM_A_CDC_RX1_B6_CTL, 5, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX2 CHAIN", TOMTOM_A_CDC_RX2_B6_CTL, 5, 0, NULL, 0),
	SND_SOC_DAPM_MUX_E("CLASS_H_DSM MUX", SND_SOC_NOPM, 0, 0, &class_h_dsm_mux,
			   wcd9330_codec_dsm_mux_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER_E("HPHL DAC", TOMTOM_A_RX_HPH_L_DAC_CTL, 7, 0,
			     hphl_switch, ARRAY_SIZE(hphl_switch),
			     wcd9330_hphl_dac_event,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			     SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("HPHR DAC", NULL, TOMTOM_A_RX_HPH_R_DAC_CTL, 7, 0,
			   wcd9330_hphr_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("HPHL_PA_MIXER", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("HPHR_PA_MIXER", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA_E("HPHL", TOMTOM_A_RX_HPH_CNP_EN, 5, 0, NULL, 0,
			   wcd9330_hph_pa_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_E("HPHR", TOMTOM_A_RX_HPH_CNP_EN, 4, 0, NULL, 0,
			   wcd9330_hph_pa_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("HEADPHONE"),

	SND_SOC_DAPM_SUPPLY("MCLK", SND_SOC_NOPM, 0, 0,
			    wcd9330_codec_enable_mclk,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("RX_BIAS", SND_SOC_NOPM, 0, 0,
			    wcd9330_codec_enable_rx_bias,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route wcd9330_audio_map[] = {
	{ "AIF1 PB", NULL, "MCLK" },
	{ "RX_BIAS", NULL, "MCLK" },
	{ "SLIM RX1 MUX", "AIF1_PB", "AIF1 PB" },
	{ "SLIM RX2 MUX", "AIF1_PB", "AIF1 PB" },
	{ "SLIM RX1", NULL, "SLIM RX1 MUX" },
	{ "SLIM RX2", NULL, "SLIM RX2 MUX" },

	{ "RX1 MIX1 INP1", "RX1", "SLIM RX1" },
	{ "RX1 MIX1 INP2", "RX2", "SLIM RX2" },
	{ "RX2 MIX1 INP1", "RX2", "SLIM RX2" },
	{ "RX2 MIX1 INP2", "RX1", "SLIM RX1" },
	{ "RX1 MIX1", NULL, "RX1 MIX1 INP1" },
	{ "RX1 MIX1", NULL, "RX1 MIX1 INP2" },
	{ "RX2 MIX1", NULL, "RX2 MIX1 INP1" },
	{ "RX2 MIX1", NULL, "RX2 MIX1 INP2" },
	{ "RX1 MIX2", NULL, "RX1 MIX1" },
	{ "RX2 MIX2", NULL, "RX2 MIX1" },
	{ "RX1 INTERP", "RX1 MIX2", "RX1 MIX2" },
	{ "RX2 INTERP", "RX2 MIX2", "RX2 MIX2" },
	{ "RX1 CHAIN", NULL, "RX1 INTERP" },
	{ "RX2 CHAIN", NULL, "RX2 INTERP" },
	{ "CLASS_H_DSM MUX", "DSM_HPHL_RX1", "RX1 CHAIN" },

	{ "HPHL DAC", "Switch", "CLASS_H_DSM MUX" },
	{ "HPHL DAC", NULL, "RX_BIAS" },
	{ "HPHR DAC", NULL, "RX2 CHAIN" },
	{ "HPHR DAC", NULL, "RX_BIAS" },
	{ "HPHL_PA_MIXER", NULL, "HPHL DAC" },
	{ "HPHR_PA_MIXER", NULL, "HPHR DAC" },
	{ "HPHL", NULL, "HPHL_PA_MIXER" },
	{ "HPHR", NULL, "HPHR_PA_MIXER" },
	{ "HEADPHONE", NULL, "HPHL" },
	{ "HEADPHONE", NULL, "HPHR" },
};

static int wcd9330_set_interpolator_rate(struct snd_soc_dai *dai, u8 rx_fs)
{
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(dai->component);
	struct snd_soc_component *comp = dai->component;
	struct wcd9330_slim_ch *ch;
	u8 rx_mix1_inp;
	u16 mix1, mix2;
	u8 v1, v2;
	int j;

	list_for_each_entry(ch, &wcd->dai[dai->id].slim_ch_list, list) {
		rx_mix1_inp = ch->port + RX_MIX1_INP_SEL_RX1 -
			      TOMTOM_RX_PORT_START;
		mix1 = TOMTOM_A_CDC_CONN_RX1_B1_CTL;
		for (j = 0; j < 7; j++) {
			mix2 = mix1 + 1;
			v1 = snd_soc_component_read(comp, mix1);
			v2 = snd_soc_component_read(comp, mix2);
			if ((v1 & 0x0f) == rx_mix1_inp ||
			    ((v1 >> 4) & 0x0f) == rx_mix1_inp ||
			    (v2 & 0x0f) == rx_mix1_inp)
				snd_soc_component_update_bits(comp,
					TOMTOM_A_CDC_RX1_B5_CTL + 8 * j,
					0xe0, rx_fs);
			if (j < 2)
				mix1 += 3;
			else
				mix1 += 2;
		}
	}
	return 0;
}

/* 3.10 tomtom_codec_enable_int_port — IFD PORT_INT_EN0 for RX ports. */
static void wcd9330_enable_int_port(struct wcd9330_codec *wcd,
				    struct wcd9330_dai_data *dai)
{
	struct wcd9330_slim_ch *ch;
	unsigned int val = 0;
	u16 reg;
	int port_num;

	list_for_each_entry(ch, &dai->slim_ch_list, list) {
		port_num = ch->port - TOMTOM_RX_PORT_START;
		reg = TOMTOM_SB_PGD_PORT_INT_EN0 + (port_num / 8);
		regmap_read(wcd->if_regmap, reg, &val);
		if (!(val & BIT(port_num % 8)))
			regmap_write(wcd->if_regmap, reg,
				     val | BIT(port_num % 8));
	}
}

static void wcd9330_slim_stop(struct wcd9330_dai_data *dai_data)
{
	if (!dai_data->sruntime)
		return;
	if (dai_data->slim_on) {
		slim_stream_disable(dai_data->sruntime);
		dai_data->slim_on = false;
	}
	if (dai_data->slim_prepared) {
		slim_stream_unprepare(dai_data->sruntime);
		dai_data->slim_prepared = false;
	}
}

/*
 * CONNECT_SINK (slim_stream_prepare). 3.10 cfg_slim_sch_rx does
 * this in AIF POST_PMU, which mainline runs after q6afe_dai_prepare
 * already called q6afe_port_start. Pulse can sit in PREPARED with
 * AFE already filling IFD — that is the overflow | PORT_CLOSED
 * (0x05) and the start pop. Connect here from hw_params, before
 * AFE prepare.
 */
static int wcd9330_slim_prepare(struct wcd9330_codec *wcd,
				struct wcd9330_dai_data *dai_data)
{
	int ret;

	if (!dai_data->sruntime)
		return -EINVAL;
	if (dai_data->slim_prepared)
		return 0;

	wcd9330_enable_int_port(wcd, dai_data);
	ret = slim_stream_prepare(dai_data->sruntime, &dai_data->sconfig);
	if (ret) {
		dev_err(wcd->dev, "slim_stream_prepare: %d\n", ret);
		return ret;
	}
	dai_data->slim_prepared = true;
	return 0;
}

/* DEF_ACT_CHAN. Call from component.prepare so it runs before AFE start. */
static int wcd9330_slim_start(struct wcd9330_codec *wcd,
			      struct wcd9330_dai_data *dai_data)
{
	int ret;

	if (!dai_data->sruntime)
		return 0;
	if (dai_data->slim_on)
		return 0;

	ret = wcd9330_slim_prepare(wcd, dai_data);
	if (ret)
		return ret;
	ret = slim_stream_enable(dai_data->sruntime);
	if (ret) {
		dev_err(wcd->dev, "slim_stream_enable: %d\n", ret);
		wcd9330_slim_stop(dai_data);
		return ret;
	}
	dai_data->slim_on = true;
	return 0;
}

static int wcd9330_slim_set_hw_params(struct wcd9330_codec *wcd,
				      struct wcd9330_dai_data *dai_data)
{
	struct slim_stream_config *cfg = &dai_data->sconfig;
	struct wcd9330_slim_ch *ch;
	u16 payload = 0;
	int ret, i;

	wcd9330_slim_stop(dai_data);
	if (dai_data->sruntime) {
		slim_stream_free(dai_data->sruntime);
		dai_data->sruntime = NULL;
	}

	cfg->ch_count = 0;
	cfg->direction = SNDRV_PCM_STREAM_PLAYBACK;
	cfg->port_mask = 0;

	list_for_each_entry(ch, &dai_data->slim_ch_list, list) {
		cfg->ch_count++;
		payload |= 1 << ch->shift;
		cfg->port_mask |= BIT(ch->port);
	}

	cfg->chs = kcalloc(cfg->ch_count, sizeof(unsigned int), GFP_KERNEL);
	if (!cfg->chs)
		return -ENOMEM;

	i = 0;
	list_for_each_entry(ch, &dai_data->slim_ch_list, list) {
		cfg->chs[i++] = ch->ch_num;
		ret = regmap_write(wcd->if_regmap,
				   TOMTOM_SB_PGD_RX_PORT_MULTI_CHNL_0(ch->port),
				   payload);
		if (ret)
			goto err;
		ret = regmap_write(wcd->if_regmap,
				   TOMTOM_SB_PGD_RX_PORT_CFG(ch->port),
				   TOMTOM_SLAVE_PORT_WATER_MARK_VAL);
		if (ret)
			goto err;
	}

	dai_data->sruntime = slim_stream_allocate(wcd->slim, "WCD9330-SLIM");
	if (IS_ERR(dai_data->sruntime)) {
		ret = PTR_ERR(dai_data->sruntime);
		dai_data->sruntime = NULL;
		dev_err(wcd->dev, "slim_stream_allocate failed: %d\n", ret);
		goto err;
	}
	return 0;
err:
	kfree(cfg->chs);
	cfg->chs = NULL;
	return ret;
}

/*
 * 3.10 tomtom_set_rxsb_port_format. POR of CONN_RX_SB_B1/B2 is
 * 0x00 (24-bit). S16_LE must set bit_sel 0x2 per RX slave port
 * or the CDC mis-parses AFE 16-bit slim samples.
 */
static void wcd9330_set_rxsb_port_format(struct snd_pcm_hw_params *params,
					 struct snd_soc_dai *dai)
{
	struct snd_soc_component *comp = dai->component;
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(comp);
	struct wcd9330_slim_ch *ch;
	u8 bit_sel;
	u16 sb_ctl_reg, field_shift;
	int port;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		bit_sel = 0x2;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		bit_sel = 0x0;
		break;
	default:
		dev_err(comp->dev, "Invalid format\n");
		return;
	}

	list_for_each_entry(ch, &wcd->dai[dai->id].slim_ch_list, list) {
		port = ch->ch_num - TOMTOM_SLIM_CH_START;
		if (port < 0 || !TOMTOM_VALIDATE_RX_SBPORT_RANGE(port)) {
			dev_warn(comp->dev,
				 "%s: invalid port ID %d returned for RX DAI\n",
				 __func__, port);
			return;
		}

		port = TOMTOM_CONVERT_RX_SBPORT_ID(port);
		if (port <= 3) {
			sb_ctl_reg = TOMTOM_A_CDC_CONN_RX_SB_B1_CTL;
			field_shift = port << 1;
		} else if (port <= 7) {
			sb_ctl_reg = TOMTOM_A_CDC_CONN_RX_SB_B2_CTL;
			field_shift = (port - 4) << 1;
		} else {
			dev_warn(comp->dev, "%s: bad port ID %d\n",
				 __func__, port);
			return;
		}

		snd_soc_component_update_bits(comp, sb_ctl_reg,
					      0x3 << field_shift,
					      bit_sel << field_shift);
	}
}

static int wcd9330_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(dai->component);
	u8 rx_fs;
	int ret;

	switch (params_rate(params)) {
	case 8000:
		rx_fs = 0x00;
		break;
	case 16000:
		rx_fs = 0x20;
		break;
	case 32000:
		rx_fs = 0x40;
		break;
	case 48000:
		rx_fs = 0x60;
		break;
	case 96000:
		rx_fs = 0x80;
		break;
	case 192000:
		rx_fs = 0xa0;
		break;
	default:
		return -EINVAL;
	}

	wcd9330_set_interpolator_rate(dai, rx_fs);
	wcd9330_set_rxsb_port_format(params, dai);
	wcd->dai[dai->id].sconfig.rate = params_rate(params);
	wcd->dai[dai->id].sconfig.bps = params_width(params);
	ret = wcd9330_slim_set_hw_params(wcd, &wcd->dai[dai->id]);
	if (ret)
		return ret;
	/* CONNECT_SINK before q6afe_dai_prepare starts SLIMBUS_0_RX */
	return wcd9330_slim_prepare(wcd, &wcd->dai[dai->id]);
}

static int wcd9330_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(dai->component);
	struct wcd9330_dai_data *dai_data = &wcd->dai[dai->id];

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		return wcd9330_slim_start(wcd, dai_data);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		wcd9330_slim_stop(dai_data);
		break;
	}
	return 0;
}

/*
 * Runs in __soc_pcm_prepare before snd_soc_pcm_dai_prepare, so
 * DEF_ACT_CHAN is up before q6afe_port_start. 3.10 does the same
 * work in AIF POST_PMU, which is after afe_port_start.
 */
static int wcd9330_component_prepare(struct snd_soc_component *comp,
				     struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(comp);
	struct snd_soc_dai *dai;
	int i, ret;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	for_each_rtd_codec_dais(rtd, i, dai) {
		if (dai->component != comp)
			continue;
		if (dai->id < 0 || dai->id >= NUM_CODEC_DAIS)
			continue;
		ret = wcd9330_slim_start(wcd, &wcd->dai[dai->id]);
		if (ret)
			return ret;
	}
	return 0;
}

static int wcd9330_set_channel_map(struct snd_soc_dai *dai,
				   unsigned int tx_num, const unsigned int *tx_slot,
				   unsigned int rx_num, const unsigned int *rx_slot)
{
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(dai->component);
	int i;

	if (!rx_slot)
		return -EINVAL;

	wcd->num_rx_port = rx_num;
	for (i = 0; i < rx_num && i < TOMTOM_RX_MAX; i++) {
		wcd->rx_chs[i].ch_num = rx_slot[i];
		INIT_LIST_HEAD(&wcd->rx_chs[i].list);
	}
	return 0;
}

static int wcd9330_get_channel_map(const struct snd_soc_dai *dai,
				   unsigned int *tx_num, unsigned int *tx_slot,
				   unsigned int *rx_num, unsigned int *rx_slot)
{
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(dai->component);
	struct wcd9330_slim_ch *ch;
	int i = 0;

	if (!rx_slot || !rx_num)
		return -EINVAL;

	list_for_each_entry(ch, &wcd->dai[dai->id].slim_ch_list, list)
		rx_slot[i++] = ch->ch_num;
	*rx_num = i;
	return 0;
}

static const struct snd_soc_dai_ops wcd9330_dai_ops = {
	.hw_params = wcd9330_hw_params,
	.trigger = wcd9330_trigger,
	.set_channel_map = wcd9330_set_channel_map,
	.get_channel_map = wcd9330_get_channel_map,
};

static struct snd_soc_dai_driver wcd9330_dai[] = {
	{
		.name = "tomtom_rx1",
		.id = AIF1_PB,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = WCD9330_RATES,
			.formats = WCD9330_FORMATS,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd9330_dai_ops,
	},
};

/*
 * 3.10 wcd9xxx_insert_detect_setup. gpio_level_insert = 0 on this
 * board (qcom,mbhc-insert-detect-inverted): write 0x6C, not 0x68.
 */
static void wcd9330_insert_detect_setup(struct wcd9330_codec *wcd, bool ins)
{
	u8 val = wcd->insert_inverted ? 0x6c : 0x68;

	regmap_update_bits(wcd->regmap, TOMTOM_A_MBHC_INSERT_DETECT, 0x01, 0);
	if (ins)
		val |= BIT(1);
	regmap_write(wcd->regmap, TOMTOM_A_MBHC_INSERT_DETECT, val);
	regmap_update_bits(wcd->regmap, TOMTOM_A_MBHC_INSERT_DETECT, 0x01, 0x01);
}

/*
 * 3.10 tomtom_mbhc_ins_rem_status: removed iff bit 4 of
 * INSERT_DET_STATUS is clear. Generic wcd9xxx_swch_level_remove
 * uses bit 2 — that is not tomtom.
 */
static bool wcd9330_jack_removed(struct wcd9330_codec *wcd)
{
	unsigned int status;

	regmap_read(wcd->regmap, TOMTOM_A_MBHC_INSERT_DET_STATUS, &status);
	return !(status & BIT(4));
}

static void wcd9330_mbhc_gnd(struct wcd9330_codec *wcd, bool enable)
{
	u8 v = enable ? 0x01 : 0x00;

	/* 3.10 init_and_calibrate / swch_irq_handler */
	regmap_update_bits(wcd->regmap, TOMTOM_A_MICB_2_CTL, 0x01, v);
	regmap_update_bits(wcd->regmap, TOMTOM_A_MBHC_HPH, 0x01, v);
}

static void wcd9330_jack_sync(struct wcd9330_codec *wcd)
{
	unsigned int det = 0, st0 = 0, st2 = 0;
	bool removed = wcd9330_jack_removed(wcd);

	regmap_read(wcd->regmap, TOMTOM_A_MBHC_INSERT_DET_STATUS, &det);
	regmap_read(wcd->regmap, TOMTOM_A_INTR_STATUS0, &st0);
	regmap_read(wcd->regmap, TOMTOM_A_INTR_STATUS2, &st2);
	dev_info(wcd->dev,
		 "MBHC %s det=0x%x st0=0x%x st2=0x%x\n",
		 removed ? "out" : "in", det, st0, st2);

	wcd9330_mbhc_gnd(wcd, removed);
	if (wcd->hs_jack)
		snd_soc_jack_report(wcd->hs_jack,
				    removed ? 0 : SND_JACK_HEADPHONE,
				    SND_JACK_HEADPHONE);
	wcd9330_insert_detect_setup(wcd, removed);
}

static irqreturn_t wcd9330_irq(int irq, void *data)
{
	struct wcd9330_codec *wcd = data;
	unsigned int status2 = 0;

	if (wcd9330_enable_mclk(wcd, true))
		return IRQ_NONE;

	/* 3.10 SWCH_IRQ_DEBOUNCE_TIME_US */
	usleep_range(TOMTOM_SWCH_DEBOUNCE_US, TOMTOM_SWCH_DEBOUNCE_US + 1000);

	regmap_read(wcd->regmap, TOMTOM_A_INTR_STATUS2, &status2);
	wcd9330_jack_sync(wcd);
	regmap_write(wcd->regmap, TOMTOM_A_INTR_CLEAR2,
		     BIT(TOMTOM_IRQ_MBHC_JACK_SWITCH % 8) | status2);
	regmap_update_bits(wcd->regmap, TOMTOM_A_INTR_MODE, 0x02, 0x02);
	wcd9330_enable_mclk(wcd, false);
	return IRQ_HANDLED;
}

static int wcd9330_mbhc_start(struct wcd9330_codec *wcd)
{
	int ret;

	ret = wcd9330_enable_mclk(wcd, true);
	if (ret)
		return ret;

	/*
	 * 3.10 wcd9xxx_setup_jack_detect_irq for insert_detect:
	 * HPHL_10K_SW is OCP_CTL bit 1. init_reg mask 0xe1 does
	 * not touch that bit; POR leaves it off.
	 */
	regmap_update_bits(wcd->regmap, TOMTOM_A_RX_HPH_OCP_CTL, BIT(1),
			   BIT(1));
	wcd9330_mbhc_gnd(wcd, true);
	wcd9330_insert_detect_setup(wcd, true);
	/* 3.10 irq_init masks every bit, then request_irq unmasks JACK_SWITCH */
	regmap_write(wcd->regmap, TOMTOM_A_INTR_MASK0, 0xff);
	regmap_write(wcd->regmap, TOMTOM_A_INTR_MASK1, 0xff);
	regmap_write(wcd->regmap, TOMTOM_A_INTR_MASK2, 0xff);
	regmap_write(wcd->regmap, TOMTOM_A_INTR_MASK3, 0xff);
	regmap_update_bits(wcd->regmap, TOMTOM_A_INTR_MASK2,
			   BIT(TOMTOM_IRQ_MBHC_JACK_SWITCH % 8), 0);
	wcd9330_jack_sync(wcd);
	wcd9330_enable_mclk(wcd, false);
	return 0;
}

static int wcd9330_set_jack(struct snd_soc_component *component,
			    struct snd_soc_jack *jack, void *data)
{
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(component);

	wcd->hs_jack = jack;
	if (!jack)
		return 0;
	return wcd9330_mbhc_start(wcd);
}

static void wcd9330_codec_init_reg(struct snd_soc_component *comp)
{
	static const struct wcd9330_reg_mask_val init[] = {
		{ TOMTOM_A_RX_HPH_OCP_CTL, 0xe1, 0x61 },
		{ TOMTOM_A_RX_COM_OCP_COUNT, 0xff, 0xff },
		{ TOMTOM_A_RX_HPH_L_TEST, 0x01, 0x01 },
		{ TOMTOM_A_RX_HPH_R_TEST, 0x01, 0x01 },
		{ TOMTOM_A_RX_HPH_L_GAIN, 0x20, 0x20 },
		{ TOMTOM_A_RX_HPH_R_GAIN, 0x20, 0x20 },
		{ TOMTOM_A_RX_HPH_CNP_WG_CTL, 0xff, 0xdb },
		{ TOMTOM_A_RX_HPH_CNP_WG_TIME, 0xff, 0x58 },
		{ TOMTOM_A_RX_HPH_BIAS_WG_OCP, 0xff, 0x1a },
		{ TOMTOM_A_RX_HPH_CHOP_CTL, 0xff, 0x24 },
		{ TOMTOM_A_NCP_CLK, 0xff, 0xfc },
		{ TOMTOM_A_BIAS_CURR_CTL_2, 0xff, 0x04 },
		{ TOMTOM_A_INTR_MODE, 0x04, 0x04 },
	};

	wcd9330_update_set(comp, init, ARRAY_SIZE(init));
}

/*
 * 3.10 sound/soc/codecs/wcd9xxx-common.h — same order, same values.
 * ADSP AFE_PARAM_ID_CDC_REG_CFG.reg_field_type.
 */
enum {
	RESERVED = 0,
	AANC_LPF_FF_FB = 1,
	AANC_LPF_COEFF_MSB,
	AANC_LPF_COEFF_LSB,
	HW_MAD_AUDIO_ENABLE,
	HW_MAD_ULTR_ENABLE,
	HW_MAD_BEACON_ENABLE,
	HW_MAD_AUDIO_SLEEP_TIME,
	HW_MAD_ULTR_SLEEP_TIME,
	HW_MAD_BEACON_SLEEP_TIME,
	HW_MAD_TX_AUDIO_SWITCH_OFF,
	HW_MAD_TX_ULTR_SWITCH_OFF,
	HW_MAD_TX_BEACON_SWITCH_OFF,
	MAD_AUDIO_INT_DEST_SELECT_REG,
	MAD_ULT_INT_DEST_SELECT_REG,
	MAD_BEACON_INT_DEST_SELECT_REG,
	MAD_CLIP_INT_DEST_SELECT_REG,
	MAD_VBAT_INT_DEST_SELECT_REG,
	MAD_AUDIO_INT_MASK_REG,
	MAD_ULT_INT_MASK_REG,
	MAD_BEACON_INT_MASK_REG,
	MAD_CLIP_INT_MASK_REG,
	MAD_VBAT_INT_MASK_REG,
	MAD_AUDIO_INT_STATUS_REG,
	MAD_ULT_INT_STATUS_REG,
	MAD_BEACON_INT_STATUS_REG,
	MAD_CLIP_INT_STATUS_REG,
	MAD_VBAT_INT_STATUS_REG,
	MAD_AUDIO_INT_CLEAR_REG,
	MAD_ULT_INT_CLEAR_REG,
	MAD_BEACON_INT_CLEAR_REG,
	MAD_CLIP_INT_CLEAR_REG,
	MAD_VBAT_INT_CLEAR_REG,
	SB_PGD_PORT_TX_WATERMARK_N,
	SB_PGD_PORT_TX_ENABLE_N,
	SB_PGD_PORT_RX_WATERMARK_N,
	SB_PGD_PORT_RX_ENABLE_N,
	SB_PGD_TX_PORTn_MULTI_CHNL_0,
	SB_PGD_TX_PORTn_MULTI_CHNL_1,
	SB_PGD_RX_PORTn_MULTI_CHNL_0,
	SB_PGD_RX_PORTn_MULTI_CHNL_1,
	AANC_FF_GAIN_ADAPTIVE,
	AANC_FFGAIN_ADAPTIVE_EN,
	AANC_GAIN_CONTROL,
	SPKR_CLIP_PIPE_BANK_SEL,
	SPKR_CLIPDET_VAL0,
	SPKR_CLIPDET_VAL1,
	SPKR_CLIPDET_VAL2,
	SPKR_CLIPDET_VAL3,
	SPKR_CLIPDET_VAL4,
	SPKR_CLIPDET_VAL5,
	SPKR_CLIPDET_VAL6,
	SPKR_CLIPDET_VAL7,
	VBAT_RELEASE_INT_DEST_SELECT_REG,
	VBAT_RELEASE_INT_MASK_REG,
	VBAT_RELEASE_INT_STATUS_REG,
	VBAT_RELEASE_INT_CLEAR_REG,
	MAD2_CLIP_INT_DEST_SELECT_REG,
	MAD2_CLIP_INT_MASK_REG,
	MAD2_CLIP_INT_STATUS_REG,
	MAD2_CLIP_INT_CLEAR_REG,
};

/*
 * 3.10 tomtom audio_reg_cfg[] — 20 entries, AFE_MAX_CDC_REGISTERS_TO_CONFIG.
 * Addresses are TOMTOM_REGISTER_START_OFFSET + register.
 */
static const struct q6afe_cdc_reg_cfg wcd9330_audio_reg_cfg[] = {
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_CDC_MAD_MAIN_CTL_1,
	  HW_MAD_AUDIO_ENABLE, 0x1, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_CDC_MAD_AUDIO_CTL_3,
	  HW_MAD_AUDIO_SLEEP_TIME, 0xf, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_CDC_MAD_AUDIO_CTL_4,
	  HW_MAD_TX_AUDIO_SWITCH_OFF, 0x1, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_INTR_MODE,
	  MAD_AUDIO_INT_DEST_SELECT_REG, 0x4, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_INTR2_MASK0,
	  MAD_AUDIO_INT_MASK_REG, 0x2, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_INTR2_STATUS0,
	  MAD_AUDIO_INT_STATUS_REG, 0x2, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_INTR2_CLEAR0,
	  MAD_AUDIO_INT_CLEAR_REG, 0x2, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_SB_PGD_PORT_TX_BASE,
	  SB_PGD_PORT_TX_WATERMARK_N, 0x1e, 8, 0x1 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_SB_PGD_PORT_TX_BASE,
	  SB_PGD_PORT_TX_ENABLE_N, 0x1, 8, 0x1 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_SB_PGD_PORT_RX_BASE,
	  SB_PGD_PORT_RX_WATERMARK_N, 0x1e, 8, 0x1 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_SB_PGD_PORT_RX_BASE,
	  SB_PGD_PORT_RX_ENABLE_N, 0x1, 8, 0x1 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_CDC_ANC1_IIR_B1_CTL,
	  AANC_FF_GAIN_ADAPTIVE, 0x4, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_CDC_ANC1_IIR_B1_CTL,
	  AANC_FFGAIN_ADAPTIVE_EN, 0x8, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_CDC_ANC1_GAIN_CTL,
	  AANC_GAIN_CONTROL, 0xff, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_INTR2_MASK0,
	  MAD_CLIP_INT_MASK_REG, 0x10, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_INTR2_MASK0,
	  MAD2_CLIP_INT_MASK_REG, 0x20, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_INTR2_STATUS0,
	  MAD_CLIP_INT_STATUS_REG, 0x10, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_INTR2_STATUS0,
	  MAD2_CLIP_INT_STATUS_REG, 0x20, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_INTR2_CLEAR0,
	  MAD_CLIP_INT_CLEAR_REG, 0x10, 8, 0 },
	{ TOMTOM_REGISTER_START_OFFSET + TOMTOM_A_INTR2_CLEAR0,
	  MAD2_CLIP_INT_CLEAR_REG, 0x20, 8, 0 },
};

static_assert(ARRAY_SIZE(wcd9330_audio_reg_cfg) == 20);

/*
 * 3.10 tomtom_init_slim_slave_cfg + msm_afe_set_config.
 * Pack PGD EA the same way 3.10 memcpy'd slim_eaddr into u64.
 */
int wcd9330_afe_set_config(struct snd_soc_component *comp, struct device *afe_dev)
{
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(comp);
	u64 eaddr = 0;
	u32 lsw, msw;
	int ret;

	if (!wcd->slim || !afe_dev)
		return -EINVAL;

	/*
	 * 3.10 tomtom_init_slim_slave_cfg: memcpy 6-byte e_addr into u64.
	 * Qualcomm stores [instance, devid, prod_le, manf_le] — same as
	 * mainline packed slim_eaddr. PGD on this board is
	 * 00 01 30 01 17 02 → lsw 0x01300100 msw 0x00000217.
	 */
	BUILD_BUG_ON(sizeof(wcd->slim->e_addr) != 6);
	memcpy(&eaddr, &wcd->slim->e_addr, sizeof(wcd->slim->e_addr));
	lsw = eaddr & 0xffffffff;
	msw = eaddr >> 32;

	dev_info(wcd->dev,
		 "WCD9330 AFE slim slave PGD ea %02x%02x%02x%02x%02x%02x lsw 0x%x msw 0x%x\n",
		 wcd->slim->e_addr.instance, wcd->slim->e_addr.dev_index,
		 wcd->slim->e_addr.prod_code & 0xff,
		 wcd->slim->e_addr.prod_code >> 8,
		 wcd->slim->e_addr.manf_id & 0xff,
		 wcd->slim->e_addr.manf_id >> 8, lsw, msw);

	ret = q6afe_cdc_reg_cfg(afe_dev, wcd9330_audio_reg_cfg,
				ARRAY_SIZE(wcd9330_audio_reg_cfg));
	if (ret) {
		dev_err(wcd->dev, "AFE_CDC_REGISTERS_CONFIG %d\n", ret);
		return ret;
	}

	ret = q6afe_cdc_slimbus_slave_cfg(afe_dev, lsw, msw, 0, 16);
	if (ret) {
		dev_err(wcd->dev, "AFE_SLIMBUS_SLAVE_CONFIG %d\n", ret);
		return ret;
	}

	ret = q6afe_cdc_reg_cfg_init(afe_dev);
	if (ret) {
		dev_err(wcd->dev, "AFE_PARAM_ID_CDC_REG_CFG_INIT %d\n", ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(wcd9330_afe_set_config);

static int wcd9330_codec_probe(struct snd_soc_component *component)
{
	struct wcd9330_codec *wcd = snd_soc_component_get_drvdata(component);
	int i, ret;

	wcd->component = component;
	wcd->clsh_state = WCD9330_CLSH_IDLE;
	snd_soc_component_init_regmap(component, wcd->regmap);

	for (i = 0; i < NUM_CODEC_DAIS; i++)
		INIT_LIST_HEAD(&wcd->dai[i].slim_ch_list);
	memcpy(wcd->rx_chs, wcd9330_rx_chs, sizeof(wcd9330_rx_chs));
	for (i = 0; i < TOMTOM_RX_MAX; i++)
		INIT_LIST_HEAD(&wcd->rx_chs[i].list);

	ret = wcd9330_enable_mclk(wcd, true);
	if (ret)
		return ret;

	/* 3.10 tomtom_codec_probe: 9.6 MHz in CHIP_CTL[2:1] */
	snd_soc_component_update_bits(component, TOMTOM_A_CHIP_CTL, 0x06, 0x02);
	wcd9330_codec_init_reg(component);

	/* 3.10 irq_init: all sources masked until set_jack unmasks JACK_SWITCH */
	regmap_write(wcd->regmap, TOMTOM_A_INTR_MASK0, 0xff);
	regmap_write(wcd->regmap, TOMTOM_A_INTR_MASK1, 0xff);
	regmap_write(wcd->regmap, TOMTOM_A_INTR_MASK2, 0xff);
	regmap_write(wcd->regmap, TOMTOM_A_INTR_MASK3, 0xff);

	wcd9330_enable_mclk(wcd, false);
	return 0;
}

static const struct snd_soc_component_driver wcd9330_component_drv = {
	.probe = wcd9330_codec_probe,
	.prepare = wcd9330_component_prepare,
	.controls = wcd9330_snd_controls,
	.num_controls = ARRAY_SIZE(wcd9330_snd_controls),
	.dapm_widgets = wcd9330_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(wcd9330_dapm_widgets),
	.dapm_routes = wcd9330_audio_map,
	.num_dapm_routes = ARRAY_SIZE(wcd9330_audio_map),
	.set_jack = wcd9330_set_jack,
	.endianness = 1,
};

static int wcd9330_parse_dt(struct wcd9330_codec *wcd)
{
	struct device *dev = wcd->dev;
	int ret;

	wcd->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(wcd->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(wcd->reset_gpio),
				     "reset GPIO\n");

	wcd->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(wcd->mclk))
		return dev_err_probe(dev, PTR_ERR(wcd->mclk), "mclk\n");

	ret = clk_set_rate(wcd->mclk, TOMTOM_MCLK_RATE);
	if (ret)
		return dev_err_probe(dev, ret, "mclk 9.6 MHz\n");

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(wcd9330_supplies),
					     wcd9330_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "static supplies\n");

	wcd->insert_inverted = of_property_read_bool(dev->of_node,
					"qcom,mbhc-insert-detect-inverted");
	wcd->irq = of_irq_get(dev->of_node, 0);
	return 0;
}

static int wcd9330_power_on_reset(struct wcd9330_codec *wcd)
{
	/*
	 * 3.10 wcd9xxx_reset: GPIO 0 for 20 ms, then 1 for 20 ms.
	 * reset-gpios is GPIO_ACTIVE_LOW, so gpiod 1 = physical 0.
	 */
	gpiod_set_value_cansleep(wcd->reset_gpio, 1);
	msleep(20);
	gpiod_set_value_cansleep(wcd->reset_gpio, 0);
	msleep(20);
	return 0;
}

static int wcd9330_bring_up(struct wcd9330_codec *wcd)
{
	struct regmap *rm = wcd->regmap;
	unsigned int b0 = 0, b1 = 0, b2 = 0, b3 = 0;
	int tries, ret, wr;
	u16 major;

	/*
	 * 3.10 wcd9xxx_device_init: wcd9xxx_bring_up then
	 * wcd9xxx_check_codec_type. WCD9330 leakage/CDC_CTL is
	 * 0x03C / 0x034, not the older 0x88 / 0x80.
	 */
	wr = regmap_write(rm, TOMTOM_A_LEAKAGE_CTL, 0x4);
	wr |= regmap_write(rm, TOMTOM_A_CDC_CTL, 0);
	usleep_range(5000, 5100);
	wr |= regmap_write(rm, TOMTOM_A_CDC_CTL, 0x1);
	wr |= regmap_write(rm, TOMTOM_A_LEAKAGE_CTL, 0x3);
	wr |= regmap_write(rm, TOMTOM_A_CDC_CTL, 0x3);
	if (wr)
		dev_err(wcd->dev, "WCD9330 bring_up slim write %d\n", wr);

	/* 3.10 wcd9xxx_slim_read_device: up to 3 tries, 5 ms apart */
	for (tries = 0; tries < 3; tries++) {
		ret = regmap_read(wcd->regmap, TOMTOM_A_CHIP_ID_BYTE_0, &b0);
		ret |= regmap_read(wcd->regmap, TOMTOM_A_CHIP_ID_BYTE_1, &b1);
		ret |= regmap_read(wcd->regmap, TOMTOM_A_CHIP_ID_BYTE_2, &b2);
		ret |= regmap_read(wcd->regmap, TOMTOM_A_CHIP_ID_BYTE_3, &b3);
		if (!ret && (b0 | b1 | b2 | b3))
			break;
		dev_info(wcd->dev,
			 "WCD9330 chip id try %d ret %d bytes %02x%02x%02x%02x\n",
			 tries, ret, b3, b2, b1, b0);
		usleep_range(5000, 5100);
	}
	major = (b3 << 8) | b2;

	dev_info(wcd->dev, "WCD9330 chip id %02x%02x%02x%02x major 0x%x\n",
		 b3, b2, b1, b0, major);

	if (major != TOMTOM_MAJOR) {
		dev_err(wcd->dev, "not tomtom (major 0x%x, expected 0x%x)\n",
			major, TOMTOM_MAJOR);
		return -ENODEV;
	}
	return 0;
}

static int wcd9330_slim_probe(struct slim_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct wcd9330_codec *wcd;
	int ret;

	wcd = devm_kzalloc(dev, sizeof(*wcd), GFP_KERNEL);
	if (!wcd)
		return -ENOMEM;

	wcd->dev = dev;
	dev_set_drvdata(dev, wcd);

	ret = wcd9330_parse_dt(wcd);
	if (ret)
		return ret;

	return wcd9330_power_on_reset(wcd);
}

static int wcd9330_slim_status(struct slim_device *sdev,
			       enum slim_device_status status)
{
	struct device *dev = &sdev->dev;
	struct wcd9330_codec *wcd = dev_get_drvdata(dev);
	struct device_node *ifc_np;
	int ret;

	ifc_np = of_parse_phandle(dev->of_node, "slim-ifc-dev", 0);
	if (!ifc_np)
		return dev_err_probe(dev, -EINVAL, "slim-ifc-dev\n");

	wcd->slim = sdev;
	wcd->slim_ifc_dev = of_slim_get_device(sdev->ctrl, ifc_np);
	of_node_put(ifc_np);
	if (!wcd->slim_ifc_dev)
		return dev_err_probe(dev, -EPROBE_DEFER, "IFD\n");

	slim_get_logical_addr(sdev);
	slim_get_logical_addr(wcd->slim_ifc_dev);
	dev_info(dev, "WCD9330 PGD laddr %u IFD laddr %u\n",
		 sdev->laddr, wcd->slim_ifc_dev->laddr);

	wcd->regmap = devm_regmap_init(dev, &wcd9330_slim_bus, sdev,
				       &wcd9330_regmap_config);
	if (IS_ERR(wcd->regmap))
		return PTR_ERR(wcd->regmap);

	wcd->if_regmap = devm_regmap_init(&wcd->slim_ifc_dev->dev,
					  &wcd9330_slim_bus, wcd->slim_ifc_dev,
					  &wcd9330_regmap_config);
	if (IS_ERR(wcd->if_regmap))
		return PTR_ERR(wcd->if_regmap);

	ret = wcd9330_bring_up(wcd);
	if (ret)
		return ret;

	if (wcd->irq > 0) {
		ret = devm_request_threaded_irq(dev, wcd->irq, NULL,
						wcd9330_irq,
						IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
						"wcd9330", wcd);
		if (ret)
			return dev_err_probe(dev, ret, "cdc-int\n");
	}

	return devm_snd_soc_register_component(dev, &wcd9330_component_drv,
					       wcd9330_dai,
					       ARRAY_SIZE(wcd9330_dai));
}

static const struct slim_device_id wcd9330_slim_id[] = {
	{ SLIM_MANF_ID_QCOM, SLIM_PROD_CODE_WCD9330, 0x1, 0x0 },
	{ }
};
MODULE_DEVICE_TABLE(slim, wcd9330_slim_id);

static struct slim_driver wcd9330_slim_driver = {
	.driver = {
		.name = "wcd9330-slim",
	},
	.probe = wcd9330_slim_probe,
	.device_status = wcd9330_slim_status,
	.id_table = wcd9330_slim_id,
};

module_slim_driver(wcd9330_slim_driver);
MODULE_DESCRIPTION("WCD9330 slim driver");
MODULE_LICENSE("GPL");
