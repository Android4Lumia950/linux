// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * 20nm PHY analog: Hai Li 2015 drm/msm/dsi: Add support for msm8x94.
 * 20nm PLL CCF lives in this PHY (dsi_phy.c pll_init), like 28nm.
 * Analog from Talkman 3.10 mdss-dsi-pll-20nm.c. Clock names/indices
 * follow dsi_phy_28nm.c. Do not use 28nm analog post-dividers.
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include "dsi_phy.h"
#include "dsi.xml.h"
#include "dsi_phy_20nm.xml.h"

/*
 * MSM8992/8994 20nm DSI PHY PLL offsets. Copied from Talkman 3.10
 * mdss-dsi-20nm-pll-util.c. Used range 0x0000..0x0168. Mainline dsi_pll
 * window 0x200 covers it. Do not invent offsets.
 */
#define MMSS_DSI_PHY_PLL_SYS_CLK_CTRL			0x0000
#define MMSS_DSI_PHY_PLL_PLL_VCOTAIL_EN			0x0004
#define MMSS_DSI_PHY_PLL_CMN_MODE			0x0008
#define MMSS_DSI_PHY_PLL_IE_TRIM			0x000c
#define MMSS_DSI_PHY_PLL_IP_TRIM			0x0010
#define MMSS_DSI_PHY_PLL_PLL_CNTRL			0x0014
#define MMSS_DSI_PHY_PLL_PLL_PHSEL_CONTROL		0x0018
#define MMSS_DSI_PHY_PLL_IPTAT_TRIM_VCCA_TX_SEL		0x001c
#define MMSS_DSI_PHY_PLL_PLL_IP_SETI			0x0024
#define MMSS_DSI_PHY_PLL_PLL_BKG_KVCO_CAL_EN		0x002c
#define MMSS_DSI_PHY_PLL_BIAS_EN_CLKBUFLR_EN		0x0030
#define MMSS_DSI_PHY_PLL_PLL_CP_SETI			0x0034
#define MMSS_DSI_PHY_PLL_PLL_IP_SETP			0x0038
#define MMSS_DSI_PHY_PLL_PLL_CP_SETP			0x003c
#define MMSS_DSI_PHY_PLL_SYSCLK_EN_SEL_TXBAND		0x0048
#define MMSS_DSI_PHY_PLL_RESETSM_CNTRL			0x004c
#define MMSS_DSI_PHY_PLL_RESETSM_CNTRL2			0x0050
#define MMSS_DSI_PHY_PLL_RESETSM_CNTRL3			0x0054
#define MMSS_DSI_PHY_PLL_DIV_REF1			0x0060
#define MMSS_DSI_PHY_PLL_DIV_REF2			0x0064
#define MMSS_DSI_PHY_PLL_KVCO_COUNT1			0x0068
#define MMSS_DSI_PHY_PLL_KVCO_CAL_CNTRL			0x0070
#define MMSS_DSI_PHY_PLL_KVCO_CODE			0x0074
#define MMSS_DSI_PHY_PLL_VREF_CFG3			0x0080
#define MMSS_DSI_PHY_PLL_PLLLOCK_CMP1			0x0090
#define MMSS_DSI_PHY_PLL_PLLLOCK_CMP2			0x0094
#define MMSS_DSI_PHY_PLL_PLLLOCK_CMP3			0x0098
#define MMSS_DSI_PHY_PLL_PLLLOCK_CMP_EN			0x009c
#define MMSS_DSI_PHY_PLL_PLL_VCO_TUNE			0x00a8
#define MMSS_DSI_PHY_PLL_DEC_START1			0x00ac
#define MMSS_DSI_PHY_PLL_SSC_EN_CENTER			0x00b4
#define MMSS_DSI_PHY_PLL_FAUX_EN				0x00fc
#define MMSS_DSI_PHY_PLL_DIV_FRAC_START1			0x0100
#define MMSS_DSI_PHY_PLL_DIV_FRAC_START2			0x0104
#define MMSS_DSI_PHY_PLL_DIV_FRAC_START3			0x0108
#define MMSS_DSI_PHY_PLL_DEC_START2			0x010c
#define MMSS_DSI_PHY_PLL_PLL_RXTXEPCLK_EN		0x0110
#define MMSS_DSI_PHY_PLL_PLL_CRCTRL			0x0114
#define MMSS_DSI_PHY_PLL_LOW_POWER_RO_CONTROL		0x013c
#define MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL		0x0140
#define MMSS_DSI_PHY_PLL_HR_OCLK2_DIVIDER		0x0144
#define MMSS_DSI_PHY_PLL_HR_OCLK3_DIVIDER		0x0148
#define MMSS_DSI_PHY_PLL_RESET_SM			0x0150
#define MMSS_DSI_PHY_PLL_CORE_VCO_TUNE			0x0160
#define MMSS_DSI_PHY_PLL_CORE_KVCO_CODE			0x0168

/*
 * Analog MMIO at clk_hw_register SErrors on this 20nm PLL even when
 * MDSS GDSC/AHB/MDP are on (pll50-pll57). 3.10 skipped that MMIO
 * because the sibling probed with GDSC off. Mainline pll_init runs
 * from the PHY after MDSS probe, so skip analog until CCF register
 * returns (clk_registering) and, for recalc/get_parent, until VCO
 * prepare has run enable_seq (phy->pll_on). set_rate still programs
 * analog after register — enable_seq does not write HR_OCLK3.
 *
 * PLL1 analog window is optional DT "dsi_pll_1" on DSI0 PHY
 * (3.10 pll_1_base). Do not ioremap a hardcoded phys.
 */

#define DSI_PLL_POLL_MAX_READS		15
#define DSI_PLL_POLL_TIMEOUT_US		1000

#define VCO_REF_CLK_RATE		19200000
/* 3.10: pll_en_90_phase on 8992/8994 overrides 300MHz–1.5GHz to 1–2 GHz */
#define VCO_MIN_RATE			1000000000UL
#define VCO_MAX_RATE			2000000000UL

struct dsi_pll_20nm_vco_calc {
	u32 div_frac_start1;
	u32 div_frac_start2;
	u32 div_frac_start3;
	u32 dec_start1;
	u32 dec_start2;
	u32 pll_plllock_cmp1;
	u32 pll_plllock_cmp2;
	u32 pll_plllock_cmp3;
};

struct dsi_pll_20nm {
	struct clk_hw vco_hw;
	struct clk_hw mux_hw;
	struct clk_hw ndiv_hw;
	struct clk_hw hr_oclk3_hw;
	struct clk_hw *byte_hw;
	struct clk_hw *pixel_hw;
	struct platform_device *pdev;
	struct msm_dsi_phy *phy;
	void __iomem *pll_base;
	void __iomem *pll_1_base;
	int index;
	bool clk_registering;
	struct mutex res_lock;
	spinlock_t lock;
	unsigned long vco_current_rate;
	unsigned long vco_ref_clk_rate;
	unsigned long vco_locking_rate;
	unsigned long vco_cached_rate;
	u32 cache_pll_trim_codes[2];
	u32 ndiv;	/* 3.10 min_div=1 max_div=15 */
	u32 hr_oclk3;	/* 3.10 min_div=1 max_div=255; HW stores div-1 */
	int resource_ref_cnt;
	bool resource_enable;
	bool is_init_locked;
	bool pll_en_90_phase;
};

#define to_pll_20nm_vco(x) container_of(x, struct dsi_pll_20nm, vco_hw)
#define to_pll_20nm_mux(x) container_of(x, struct dsi_pll_20nm, mux_hw)
#define to_pll_20nm_ndiv(x) container_of(x, struct dsi_pll_20nm, ndiv_hw)
#define to_pll_20nm_hr_oclk3(x) container_of(x, struct dsi_pll_20nm, hr_oclk3_hw)

static bool pll_20nm_analog_readable(struct dsi_pll_20nm *pll)
{
	return pll->phy && pll->phy->pll_on && !pll->clk_registering;
}

static bool pll_20nm_analog_writable(struct dsi_pll_20nm *pll)
{
	return pll->phy && !pll->clk_registering;
}

/*
 * 3.10 mdss_pll_util_resource_enable: msm_dss_enable_vreg then
 * msm_dss_enable_clk. PLL0 DT vregs are gdsc + vddio L12 + vcca L28.
 * Mainline: MDSS parent resume = GDSC genpd; PHY supplies = vddio/vcca
 * (SPMI talkman_l12 / talkman_l28, not a dummy fixed regulator); PHY pm_clk
 * iface = MDSS_AHB. Analog at clk_register still SErrors with those
 * votes on (pll50-pll57); skip MMIO via clk_registering instead of
 * collapsing leftover GDSCR. Do not clk_get unused mdp/mmss_misc.
 */
static int pll_20nm_resource_enable(struct dsi_pll_20nm *pll, bool enable)
{
	struct msm_dsi_phy *phy = pll->phy;
	struct device *dev;
	struct device *mdss_dev;
	int rc = 0;
	int changed = 0;

	if (!phy)
		return -ENODEV;
	dev = &phy->pdev->dev;
	mdss_dev = dev->parent;

	mutex_lock(&pll->res_lock);
	if (enable) {
		if (pll->resource_ref_cnt == 0)
			changed++;
		pll->resource_ref_cnt++;
	} else if (pll->resource_ref_cnt) {
		pll->resource_ref_cnt--;
		if (pll->resource_ref_cnt == 0)
			changed++;
	}

	if (changed) {
		if (enable) {
			if (mdss_dev) {
				rc = pm_runtime_resume_and_get(mdss_dev);
				if (rc < 0) {
					pll->resource_ref_cnt--;
					mutex_unlock(&pll->res_lock);
					return rc;
				}
			}
			rc = regulator_bulk_enable(phy->cfg->num_regulators,
						   phy->supplies);
			if (rc) {
				if (mdss_dev)
					pm_runtime_put(mdss_dev);
				pll->resource_ref_cnt--;
				mutex_unlock(&pll->res_lock);
				return rc;
			}
			rc = pm_runtime_resume_and_get(dev);
			if (rc < 0) {
				regulator_bulk_disable(phy->cfg->num_regulators,
						       phy->supplies);
				if (mdss_dev)
					pm_runtime_put(mdss_dev);
				pll->resource_ref_cnt--;
				mutex_unlock(&pll->res_lock);
				return rc;
			}
			pll->resource_enable = true;
		} else {
			pm_runtime_put(dev);
			regulator_bulk_disable(phy->cfg->num_regulators,
					       phy->supplies);
			if (mdss_dev)
				pm_runtime_put(mdss_dev);
			pll->resource_enable = false;
		}
	}
	mutex_unlock(&pll->res_lock);
	return 0;
}

static void pll_20nm_config_common_block_1(void __iomem *pll_base)
{
	if (!pll_base)
		return;

	writel(0x82, pll_base + MMSS_DSI_PHY_PLL_PLL_VCOTAIL_EN);
	writel(0x2a, pll_base + MMSS_DSI_PHY_PLL_BIAS_EN_CLKBUFLR_EN);
	writel(0x2b, pll_base + MMSS_DSI_PHY_PLL_BIAS_EN_CLKBUFLR_EN);
	writel(0x02, pll_base + MMSS_DSI_PHY_PLL_RESETSM_CNTRL3);
}

static void pll_20nm_config_common_block_2(void __iomem *pll_base)
{
	writel(0x40, pll_base + MMSS_DSI_PHY_PLL_SYS_CLK_CTRL);
	writel(0x0f, pll_base + MMSS_DSI_PHY_PLL_IE_TRIM);
	writel(0x0f, pll_base + MMSS_DSI_PHY_PLL_IP_TRIM);
	writel(0x08, pll_base + MMSS_DSI_PHY_PLL_PLL_PHSEL_CONTROL);
	writel(0x0e, pll_base + MMSS_DSI_PHY_PLL_IPTAT_TRIM_VCCA_TX_SEL);
	writel(0x08, pll_base + MMSS_DSI_PHY_PLL_PLL_BKG_KVCO_CAL_EN);
	writel(0x4a, pll_base + MMSS_DSI_PHY_PLL_SYSCLK_EN_SEL_TXBAND);
	writel(0x00, pll_base + MMSS_DSI_PHY_PLL_DIV_REF1);
	writel(0x01, pll_base + MMSS_DSI_PHY_PLL_DIV_REF2);
	writel(0x07, pll_base + MMSS_DSI_PHY_PLL_PLL_CNTRL);
	writel(0x1f, pll_base + MMSS_DSI_PHY_PLL_KVCO_CAL_CNTRL);
	writel(0x8a, pll_base + MMSS_DSI_PHY_PLL_KVCO_COUNT1);
	writel(0x10, pll_base + MMSS_DSI_PHY_PLL_VREF_CFG3);
	writel(0x00, pll_base + MMSS_DSI_PHY_PLL_SSC_EN_CENTER);
	writel(0x0c, pll_base + MMSS_DSI_PHY_PLL_FAUX_EN);
	writel(0x0a, pll_base + MMSS_DSI_PHY_PLL_PLL_RXTXEPCLK_EN);
	writel(0x0f, pll_base + MMSS_DSI_PHY_PLL_LOW_POWER_RO_CONTROL);
	writel(0x00, pll_base + MMSS_DSI_PHY_PLL_CMN_MODE);
}

static void pll_20nm_config_loop_bw(void __iomem *pll_base)
{
	writel(0x03, pll_base + MMSS_DSI_PHY_PLL_PLL_IP_SETI);
	writel(0x3f, pll_base + MMSS_DSI_PHY_PLL_PLL_CP_SETI);
	writel(0x03, pll_base + MMSS_DSI_PHY_PLL_PLL_IP_SETP);
	writel(0x1f, pll_base + MMSS_DSI_PHY_PLL_PLL_CP_SETP);
	writel(0x77, pll_base + MMSS_DSI_PHY_PLL_PLL_CRCTRL);
}

static void pll_20nm_config_powerdown(void __iomem *pll_base)
{
	if (!pll_base)
		return;

	writel(0x00, pll_base + MMSS_DSI_PHY_PLL_SYS_CLK_CTRL);
	writel(0x01, pll_base + MMSS_DSI_PHY_PLL_CMN_MODE);
	writel(0x82, pll_base + MMSS_DSI_PHY_PLL_PLL_VCOTAIL_EN);
	writel(0x02, pll_base + MMSS_DSI_PHY_PLL_BIAS_EN_CLKBUFLR_EN);
	writel(0x06, pll_base + MMSS_DSI_PHY_PLL_RESETSM_CNTRL3);
}

static void pll_20nm_vco_rate_calc(struct dsi_pll_20nm_vco_calc *vco_calc,
				   s64 vco_clk_rate, s64 ref_clk_rate,
				   bool pll_en_90_phase)
{
	s64 multiplier = (1 << 20);
	s64 duration, pll_comp_val;
	s64 dec_start_multiple, dec_start;
	s32 div_frac_start;
	s64 dec_start1, dec_start2;
	s32 div_frac_start1, div_frac_start2, div_frac_start3;
	s64 pll_plllock_cmp1, pll_plllock_cmp2, pll_plllock_cmp3;

	if (pll_en_90_phase)
		duration = 128;
	else
		duration = 1024;

	memset(vco_calc, 0, sizeof(*vco_calc));

	dec_start_multiple = div_s64(vco_clk_rate * multiplier,
				     2 * ref_clk_rate);
	div_s64_rem(dec_start_multiple, multiplier, &div_frac_start);

	dec_start = div_s64(dec_start_multiple, multiplier);
	dec_start1 = (dec_start & 0x7f) | BIT(7);
	dec_start2 = ((dec_start & 0x80) >> 7) | BIT(1);
	div_frac_start1 = (div_frac_start & 0x7f) | BIT(7);
	div_frac_start2 = ((div_frac_start >> 7) & 0x7f) | BIT(7);
	div_frac_start3 = ((div_frac_start >> 14) & 0x3f) | BIT(6);
	if (pll_en_90_phase)
		pll_comp_val = div_s64(dec_start_multiple * 2 * (duration - 1),
				       10 * multiplier);
	else
		pll_comp_val = div_s64(dec_start_multiple * 2 * duration,
				       10 * multiplier) - 1;
	pll_plllock_cmp1 = pll_comp_val & 0xff;
	pll_plllock_cmp2 = (pll_comp_val >> 8) & 0xff;
	pll_plllock_cmp3 = (pll_comp_val >> 16) & 0xff;

	vco_calc->div_frac_start1 = div_frac_start1;
	vco_calc->div_frac_start2 = div_frac_start2;
	vco_calc->div_frac_start3 = div_frac_start3;
	vco_calc->dec_start1 = dec_start1;
	vco_calc->dec_start2 = dec_start2;
	vco_calc->pll_plllock_cmp1 = pll_plllock_cmp1;
	vco_calc->pll_plllock_cmp2 = pll_plllock_cmp2;
	vco_calc->pll_plllock_cmp3 = pll_plllock_cmp3;
}

static void pll_20nm_config_vco_rate(void __iomem *pll_base,
				     struct dsi_pll_20nm_vco_calc *vco_calc,
				     bool pll_en_90_phase)
{
	writel(vco_calc->div_frac_start1, pll_base + MMSS_DSI_PHY_PLL_DIV_FRAC_START1);
	writel(vco_calc->div_frac_start2, pll_base + MMSS_DSI_PHY_PLL_DIV_FRAC_START2);
	writel(vco_calc->div_frac_start3, pll_base + MMSS_DSI_PHY_PLL_DIV_FRAC_START3);
	writel(vco_calc->dec_start1, pll_base + MMSS_DSI_PHY_PLL_DEC_START1);
	writel(vco_calc->dec_start2, pll_base + MMSS_DSI_PHY_PLL_DEC_START2);
	writel(vco_calc->pll_plllock_cmp1, pll_base + MMSS_DSI_PHY_PLL_PLLLOCK_CMP1);
	writel(vco_calc->pll_plllock_cmp2, pll_base + MMSS_DSI_PHY_PLL_PLLLOCK_CMP2);
	writel(vco_calc->pll_plllock_cmp3, pll_base + MMSS_DSI_PHY_PLL_PLLLOCK_CMP3);
	if (pll_en_90_phase)
		writel(0x0d, pll_base + MMSS_DSI_PHY_PLL_PLLLOCK_CMP_EN);
	else
		writel(0x01, pll_base + MMSS_DSI_PHY_PLL_PLLLOCK_CMP_EN);
}

static bool pll_20nm_is_pll_locked(struct dsi_pll_20nm *pll)
{
	u32 status;
	/*
	 * 3.10 uses readl_poll_timeout_noirq(addr, val, cond,
	 * DSI_PLL_POLL_MAX_READS, DSI_PLL_POLL_TIMEOUT_US):
	 * 15 reads with udelay(1000) between them (iopoll.h).
	 * Mainline readl_poll_timeout is (sleep_us, timeout_us),
	 * not (max_reads, timeout_us). Passing 15, 1000 waits 1 ms.
	 */
	unsigned long sleep_us = DSI_PLL_POLL_TIMEOUT_US;
	unsigned long timeout_us = DSI_PLL_POLL_MAX_READS * DSI_PLL_POLL_TIMEOUT_US;

	if (readl_poll_timeout(pll->pll_base + MMSS_DSI_PHY_PLL_RESET_SM,
			       status, status & BIT(5),
			       sleep_us, timeout_us)) {
		pr_err("talkman-pll20: PLL status=%x failed to Lock\n", status);
		return false;
	}

	if (readl_poll_timeout(pll->pll_base + MMSS_DSI_PHY_PLL_RESET_SM,
			       status, status & BIT(6),
			       sleep_us, timeout_us)) {
		pr_err("talkman-pll20: PLL status=%x PLL not ready\n", status);
		return false;
	}

	return true;
}

static void pll_20nm_cache_trim_codes(struct dsi_pll_20nm *pll)
{
	pll->cache_pll_trim_codes[0] =
		readl(pll->pll_base + MMSS_DSI_PHY_PLL_CORE_KVCO_CODE);
	pll->cache_pll_trim_codes[1] =
		readl(pll->pll_base + MMSS_DSI_PHY_PLL_CORE_VCO_TUNE);
}

static void pll_20nm_override_trim_codes(struct dsi_pll_20nm *pll)
{
	void __iomem *pll_base = pll->pll_base;
	u32 reg_data;

	reg_data = (pll->cache_pll_trim_codes[0] & 0x3f) | BIT(5);
	writel(reg_data, pll_base + MMSS_DSI_PHY_PLL_KVCO_CODE);
	reg_data = (pll->cache_pll_trim_codes[1] & 0x7f) | BIT(7);
	writel(reg_data, pll_base + MMSS_DSI_PHY_PLL_PLL_VCO_TUNE);
}

static void pll_20nm_config_resetsm(void __iomem *pll_base)
{
	writel(0x00, pll_base + MMSS_DSI_PHY_PLL_KVCO_CODE);
	writel(0x00, pll_base + MMSS_DSI_PHY_PLL_PLL_VCO_TUNE);
	writel(0x24, pll_base + MMSS_DSI_PHY_PLL_RESETSM_CNTRL);
	writel(0x07, pll_base + MMSS_DSI_PHY_PLL_RESETSM_CNTRL2);
}

static void pll_20nm_config_vco_start(void __iomem *pll_base)
{
	writel(0x03, pll_base + MMSS_DSI_PHY_PLL_PLL_VCOTAIL_EN);
	writel(0x02, pll_base + MMSS_DSI_PHY_PLL_RESETSM_CNTRL3);
	udelay(10);
	writel(0x03, pll_base + MMSS_DSI_PHY_PLL_RESETSM_CNTRL3);
}

static void pll_20nm_config_bypass_cal(void __iomem *pll_base)
{
	writel(0xac, pll_base + MMSS_DSI_PHY_PLL_RESETSM_CNTRL);
	writel(0x28, pll_base + MMSS_DSI_PHY_PLL_PLL_BKG_KVCO_CAL_EN);
}

static int pll_20nm_vco_init_lock(struct dsi_pll_20nm *pll)
{
	pll_20nm_config_resetsm(pll->pll_base);
	pll_20nm_config_vco_start(pll->pll_base);

	if (!pll_20nm_is_pll_locked(pll))
		return -EINVAL;

	pll_20nm_cache_trim_codes(pll);
	return 0;
}

static int pll_20nm_vco_relock(struct dsi_pll_20nm *pll)
{
	pll_20nm_override_trim_codes(pll);
	pll_20nm_config_bypass_cal(pll->pll_base);
	pll_20nm_config_vco_start(pll->pll_base);

	if (!pll_20nm_is_pll_locked(pll))
		return -EINVAL;

	return 0;
}

static int pll_20nm_vco_enable_seq(struct dsi_pll_20nm *pll)
{
	struct dsi_pll_20nm_vco_calc vco_calc;
	int rc;

	pll_20nm_config_common_block_1(pll->pll_1_base);
	pll_20nm_config_common_block_1(pll->pll_base);
	pll_20nm_config_common_block_2(pll->pll_base);
	pll_20nm_config_loop_bw(pll->pll_base);

	pll_20nm_vco_rate_calc(&vco_calc, pll->vco_current_rate,
			       pll->vco_ref_clk_rate, pll->pll_en_90_phase);
	pll_20nm_config_vco_rate(pll->pll_base, &vco_calc,
				 pll->pll_en_90_phase);

	/* 3.10 fixed_hr_oclk2 min=max=4 writes (div-1) into HR_OCLK2_DIVIDER */
	writel(3, pll->pll_base + MMSS_DSI_PHY_PLL_HR_OCLK2_DIVIDER);

	if (!pll->is_init_locked ||
	    pll->vco_locking_rate != pll->vco_current_rate) {
		rc = pll_20nm_vco_init_lock(pll);
		pll->is_init_locked = !rc;
	} else {
		rc = pll_20nm_vco_relock(pll);
	}

	pll->vco_locking_rate = rc ? 0 : pll->vco_current_rate;
	return rc;
}

static int dsi_pll_20nm_clk_determine_rate(struct clk_hw *hw,
					   struct clk_rate_request *req)
{
	req->rate = clamp_t(unsigned long, req->rate, VCO_MIN_RATE, VCO_MAX_RATE);
	return 0;
}

static int dsi_pll_20nm_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long parent_rate)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_vco(hw);

	/* 3.10 pll_20nm_vco_set_rate only caches; analog is enable_seq */
	pll->vco_current_rate = rate;
	pll->vco_ref_clk_rate = parent_rate ? parent_rate : VCO_REF_CLK_RATE;
	return 0;
}

static unsigned long dsi_pll_20nm_clk_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_vco(hw);

	(void)parent_rate;
	/*
	 * 3.10 clk_ops_dsi_vco has no get_rate. CCF always calls
	 * recalc_rate from __clk_register. #50/#51 SError'd on
	 * readl(DEC_START2) here. pll_20nm_vco_get_rate exists only
	 * for handoff after lock, not for register. Return the
	 * cached set_rate value (0 until prepare).
	 */
	return pll->vco_current_rate;
}

static int dsi_pll_20nm_vco_prepare(struct clk_hw *hw)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_vco(hw);
	int rc;

	if (!pll->phy || unlikely(pll->phy->pll_on))
		return pll->phy ? 0 : -ENODEV;

	if (!pll->vco_current_rate)
		return -EINVAL;

	rc = pll_20nm_resource_enable(pll, true);
	if (rc)
		return rc;

	if (pll->vco_cached_rate && pll->vco_cached_rate == pll->vco_current_rate)
		dsi_pll_20nm_clk_set_rate(hw, pll->vco_cached_rate,
					  pll->vco_ref_clk_rate);

	rc = pll_20nm_vco_enable_seq(pll);
	/* 3.10 dsi_pll_enable: power down PLL1 after the lock attempt */
	pll_20nm_config_powerdown(pll->pll_1_base);
	if (rc) {
		pll_20nm_resource_enable(pll, false);
		DRM_DEV_ERROR(&pll->pdev->dev,
			      "talkman-pll20: VCO lock failed rate=%lu\n",
			      pll->vco_current_rate);
		return rc;
	}

	pll->phy->pll_on = true;
	pr_info("talkman-pll20: locked vco=%lu\n", pll->vco_current_rate);
	return 0;
}

static void dsi_pll_20nm_vco_unprepare(struct clk_hw *hw)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_vco(hw);

	if (!pll->phy || unlikely(!pll->phy->pll_on))
		return;

	pll->vco_cached_rate = pll->vco_current_rate;
	pll_20nm_config_powerdown(pll->pll_1_base);
	pll_20nm_config_powerdown(pll->pll_base);
	pll_20nm_resource_enable(pll, false);
	pll->phy->pll_on = false;
}

/*
 * 3.10 clk_ops_dsi_vco has no is_enabled. Do not read RESET_SM at
 * clk_register.
 */
static const struct clk_ops clk_ops_dsi_pll_20nm_vco = {
	.determine_rate = dsi_pll_20nm_clk_determine_rate,
	.set_rate = dsi_pll_20nm_clk_set_rate,
	.recalc_rate = dsi_pll_20nm_clk_recalc_rate,
	.prepare = dsi_pll_20nm_vco_prepare,
	.unprepare = dsi_pll_20nm_vco_unprepare,
};

/*
 * Bypass mux shares POST_DIVIDER_CONTROL with ndiv (bits [3:0]).
 * Bit 5 = parent (0 VCO, 1 ndiv/2). 3.10 also sets bit 7 on every
 * set_mux_sel. Standard clk_mux would not set bit 7.
 */
static u8 dsi_20nm_byte_mux_get_parent(struct clk_hw *hw)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_mux(hw);
	u32 val;

	/* 3.10 get_bypass_lp_div_mux_sel */
	if (!pll_20nm_analog_readable(pll))
		return 0;

	if (pll_20nm_resource_enable(pll, true))
		return 0;

	val = readl(pll->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL);
	pll_20nm_resource_enable(pll, false);
	return !!(val & BIT(5));
}

static int dsi_20nm_byte_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_mux(hw);
	unsigned long flags;
	u32 val;
	int rc;

	/* 3.10 GET skips analog when GDSC is off. Skip analog SET
	 * at clk_register (clk_registering) the same way.
	 */
	if (!pll_20nm_analog_writable(pll))
		return 0;

	rc = pll_20nm_resource_enable(pll, true);
	if (rc)
		return rc;

	spin_lock_irqsave(&pll->lock, flags);
	val = readl(pll->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL);
	val |= BIT(7);
	val &= ~BIT(5);
	val |= (index << 5);
	writel(val, pll->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL);
	spin_unlock_irqrestore(&pll->lock, flags);
	pll_20nm_resource_enable(pll, false);
	return 0;
}

static const struct clk_ops clk_ops_dsi_20nm_byte_mux = {
	.determine_rate = __clk_mux_determine_rate_closest,
	.set_parent = dsi_20nm_byte_mux_set_parent,
	.get_parent = dsi_20nm_byte_mux_get_parent,
};

/*
 * 3.10 ndiv_get_div returns 0 when analog is unreadable. CCF ONE_BASED cannot
 * use divisor 0; cached min_div=1 is that value, not a stub BYTE.
 */
static unsigned long pll_20nm_div_rate(unsigned long parent_rate, u32 div)
{
	if (!div)
		div = 1;
	return DIV_ROUND_UP_ULL((u64)parent_rate, div);
}

static unsigned long dsi_20nm_ndiv_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_ndiv(hw);
	u32 div;

	if (!pll_20nm_analog_readable(pll))
		return pll_20nm_div_rate(parent_rate, pll->ndiv);

	if (pll_20nm_resource_enable(pll, true))
		return pll_20nm_div_rate(parent_rate, pll->ndiv);

	div = readl(pll->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL) &
	      0x0f;
	pll_20nm_resource_enable(pll, false);
	if (div)
		pll->ndiv = div;
	return pll_20nm_div_rate(parent_rate, pll->ndiv);
}

static int dsi_20nm_ndiv_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_ndiv(hw);
	unsigned long flags;
	u32 val;
	int div, rc;

	div = divider_get_val(rate, parent_rate, NULL, 4, CLK_DIVIDER_ONE_BASED);
	if (div < 0)
		return div;
	if (div < 1)
		div = 1;
	if (div > 15)
		div = 15;
	pll->ndiv = div;

	if (!pll_20nm_analog_writable(pll))
		return 0;

	/* 3.10 ndiv_set_div: always resource_enable, then reg | div */
	rc = pll_20nm_resource_enable(pll, true);
	if (rc)
		return rc;

	spin_lock_irqsave(&pll->lock, flags);
	val = readl(pll->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL);
	writel(val | div, pll->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL);
	spin_unlock_irqrestore(&pll->lock, flags);
	pll_20nm_resource_enable(pll, false);
	return 0;
}

static int dsi_20nm_ndiv_determine_rate(struct clk_hw *hw,
					struct clk_rate_request *req)
{
	return divider_determine_rate(hw, req, NULL, 4, CLK_DIVIDER_ONE_BASED);
}

static const struct clk_ops clk_ops_dsi_20nm_ndiv = {
	.recalc_rate = dsi_20nm_ndiv_recalc_rate,
	.set_rate = dsi_20nm_ndiv_set_rate,
	.determine_rate = dsi_20nm_ndiv_determine_rate,
};

static unsigned long dsi_20nm_hr_oclk3_recalc_rate(struct clk_hw *hw,
						   unsigned long parent_rate)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_hr_oclk3(hw);
	u32 div;

	if (!pll_20nm_analog_readable(pll))
		return pll_20nm_div_rate(parent_rate, pll->hr_oclk3);

	if (pll_20nm_resource_enable(pll, true))
		return pll_20nm_div_rate(parent_rate, pll->hr_oclk3);

	div = readl(pll->pll_base + MMSS_DSI_PHY_PLL_HR_OCLK3_DIVIDER) + 1;
	pll_20nm_resource_enable(pll, false);
	if (div)
		pll->hr_oclk3 = div;
	return pll_20nm_div_rate(parent_rate, pll->hr_oclk3);
}

static int dsi_20nm_hr_oclk3_set_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long parent_rate)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_hr_oclk3(hw);
	int div, rc;

	div = divider_get_val(rate, parent_rate, NULL, 8, 0);
	if (div < 0)
		return div;
	/* flags 0: divider_get_val is the HW field (div-1). 3.10 writes that. */
	if (div > 254)
		div = 254;
	pll->hr_oclk3 = div + 1;

	pr_info("talkman-pll20: hr_oclk3 set_rate rate=%lu parent=%lu div=%d phy=%d pll_on=%d\n",
		rate, parent_rate, pll->hr_oclk3,
		!!pll->phy, pll->phy && pll->phy->pll_on);

	if (!pll_20nm_analog_writable(pll))
		return 0;

	rc = pll_20nm_resource_enable(pll, true);
	if (rc)
		return rc;

	writel(div, pll->pll_base + MMSS_DSI_PHY_PLL_HR_OCLK3_DIVIDER);
	pll_20nm_resource_enable(pll, false);
	return 0;
}

static int dsi_20nm_hr_oclk3_determine_rate(struct clk_hw *hw,
					    struct clk_rate_request *req)
{
	return divider_determine_rate(hw, req, NULL, 8, 0);
}

static const struct clk_ops clk_ops_dsi_20nm_hr_oclk3 = {
	.recalc_rate = dsi_20nm_hr_oclk3_recalc_rate,
	.set_rate = dsi_20nm_hr_oclk3_set_rate,
	.determine_rate = dsi_20nm_hr_oclk3_determine_rate,
};

static int pll_20nm_register(struct dsi_pll_20nm *pll, struct clk_hw **provided_clocks)
{
	char clk_name[32], mux_name[32], ndiv_name[32], hr_oclk3_name[32];
	struct clk_init_data vco_init = {
		.parent_data = &(const struct clk_parent_data) {
			.fw_name = "ref", .name = "xo",
		},
		.num_parents = 1,
		.name = clk_name,
		.ops = &clk_ops_dsi_pll_20nm_vco,
		.flags = CLK_IGNORE_UNUSED,
	};
	struct device *dev = &pll->pdev->dev;
	struct clk_hw *hw, *indirect_path_div2, *hr_oclk2;
	struct clk_init_data mux_init, ndiv_init, hr_oclk3_init;
	const struct clk_hw *mux_parents[2];
	const struct clk_hw *ndiv_parent[1];
	const struct clk_hw *hr_oclk3_parent[1];
	int ret;
	int id = pll->index;

	if (!pll->pdev)
		return -ENODEV;

	snprintf(clk_name, sizeof(clk_name), "dsi%dvco_clk", id);
	pll->vco_hw.init = &vco_init;
	ret = devm_clk_hw_register(dev, &pll->vco_hw);
	if (ret)
		return ret;

	/*
	 * ndiv 1–15 in POST_DIVIDER_CONTROL [3:0]. 3.10 wrote (reg | div)
	 * without masking; the field is bits [3:0] (get_div uses & 0x0f).
	 * Custom ops: 3.10 get skips PLL MMIO when analog is unreadable. Do not use
	 * CCF clk_divider — it readl's at clk_register.
	 */
	snprintf(ndiv_name, sizeof(ndiv_name), "dsi%dndiv_clk", id);
	ndiv_parent[0] = &pll->vco_hw;
	ndiv_init = (struct clk_init_data) {
		.name = ndiv_name,
		.ops = &clk_ops_dsi_20nm_ndiv,
		.parent_hws = ndiv_parent,
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	};
	pll->ndiv_hw.init = &ndiv_init;
	ret = devm_clk_hw_register(dev, &pll->ndiv_hw);
	if (ret)
		return ret;

	snprintf(clk_name, sizeof(clk_name), "dsi%dindirect_path_div2_clk",
		 id);
	indirect_path_div2 = devm_clk_hw_register_fixed_factor_parent_hw(dev,
		clk_name, &pll->ndiv_hw, CLK_SET_RATE_PARENT, 1, 2);
	if (IS_ERR(indirect_path_div2))
		return PTR_ERR(indirect_path_div2);

	snprintf(mux_name, sizeof(mux_name), "dsi%dbyte_mux", id);
	mux_parents[0] = &pll->vco_hw;
	mux_parents[1] = indirect_path_div2;
	mux_init = (struct clk_init_data) {
		.name = mux_name,
		.ops = &clk_ops_dsi_20nm_byte_mux,
		.parent_hws = mux_parents,
		.num_parents = 2,
		.flags = CLK_SET_RATE_PARENT,
	};
	pll->mux_hw.init = &mux_init;
	ret = devm_clk_hw_register(dev, &pll->mux_hw);
	if (ret)
		return ret;

	snprintf(clk_name, sizeof(clk_name), "dsi%dhr_oclk2_clk", id);
	hr_oclk2 = devm_clk_hw_register_fixed_factor_parent_hw(dev, clk_name,
		&pll->mux_hw, CLK_SET_RATE_PARENT, 1, 4);
	if (IS_ERR(hr_oclk2))
		return PTR_ERR(hr_oclk2);

	snprintf(clk_name, sizeof(clk_name), "dsi%dpllbyte", id);
	hw = devm_clk_hw_register_fixed_factor_parent_hw(dev, clk_name,
		hr_oclk2, CLK_SET_RATE_PARENT, 1, 2);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	pll->byte_hw = hw;
	if (provided_clocks)
		provided_clocks[DSI_BYTE_PLL_CLK] = hw;

	/*
	 * 3.10 hr_oclk3.c.ops = clk_ops_slave_div (no parent set_rate).
	 * 3.10 pixel_clk_src is clk_ops_div min=max=2 of hr_oclk3
	 * (SET_RATE_PARENT). Do not put slave_div on dsi0pll: mmcc
	 * pclk0 uses clk_pixel_ops + CLK_SET_RATE_PARENT, frac 1/1
	 * needs dsi0pll to round to the pixel rate.
	 */
	snprintf(hr_oclk3_name, sizeof(hr_oclk3_name), "dsi%dhr_oclk3_clk",
		 id);
	hr_oclk3_parent[0] = &pll->vco_hw;
	hr_oclk3_init = (struct clk_init_data) {
		.name = hr_oclk3_name,
		.ops = &clk_ops_dsi_20nm_hr_oclk3,
		.parent_hws = hr_oclk3_parent,
		.num_parents = 1,
	};
	pll->hr_oclk3_hw.init = &hr_oclk3_init;
	ret = devm_clk_hw_register(dev, &pll->hr_oclk3_hw);
	if (ret)
		return ret;

	snprintf(clk_name, sizeof(clk_name), "dsi%dpll", id);
	hw = devm_clk_hw_register_fixed_factor_parent_hw(dev, clk_name,
		&pll->hr_oclk3_hw, CLK_SET_RATE_PARENT, 1, 2);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	pr_info("talkman-pll20: dsi0pll CLK_SET_RATE_PARENT (3.10 clk_ops_div of hr_oclk3)\n");
	pll->pixel_hw = hw;
	if (provided_clocks)
		provided_clocks[DSI_PIXEL_PLL_CLK] = hw;

	return 0;
}

static int dsi_pll_20nm_init(struct msm_dsi_phy *phy)
{
	struct platform_device *pdev = phy->pdev;
	struct dsi_pll_20nm *pll;
	struct resource *res;
	int ret;

	if (!pdev)
		return -ENODEV;

	/*
	 * 3.10 dsi_pll_clock_register_20nm: full byte/pixel tree only on
	 * PLL0 (index==0). CAF mmsscc parents BYTE1/PCLK1 from PLL0.
	 * Do not run enable_seq before ndiv. Analog is
	 * pll_20nm_vco_enable_seq at VCO prepare after clocks exist.
	 */
	if (phy->id != DSI_0) {
		pr_info("talkman-pll20: id=%d skip byte/pixel tree (CAF PLL1)\n",
			phy->id);
		return 0;
	}

	pll = devm_kzalloc(&pdev->dev, sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return -ENOMEM;

	pll->pdev = pdev;
	pll->phy = phy;
	pll->index = phy->id;
	pll->vco_ref_clk_rate = VCO_REF_CLK_RATE;
	pll->pll_en_90_phase = true;
	pll->ndiv = 1;
	pll->hr_oclk3 = 1;
	pll->pll_base = phy->pll_base;
	spin_lock_init(&pll->lock);
	mutex_init(&pll->res_lock);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dsi_pll_1");
	if (res) {
		pll->pll_1_base = devm_ioremap(&pdev->dev, res->start,
					       resource_size(res));
		if (!pll->pll_1_base)
			return -ENOMEM;
	}

	pll->clk_registering = true;
	ret = pll_20nm_register(pll, phy->provided_clocks->hws);
	pll->clk_registering = false;
	if (ret) {
		DRM_DEV_ERROR(&pdev->dev, "failed to register PLL: %d\n", ret);
		return ret;
	}

	phy->vco_hw = &pll->vco_hw;
	pr_info("talkman-pll20: id=0 dsi0pllbyte/dsi0pll registered vco=%lu-%lu 90-phase\n",
		VCO_MIN_RATE, VCO_MAX_RATE);
	return 0;
}

static void dsi_20nm_dphy_set_timing(struct msm_dsi_phy *phy,
				     struct msm_dsi_dphy_timing *timing)
{
	void __iomem *base = phy->base;
	/*
	 * 3.10 mdss_dsi_20nm_phy_config writes pd->timing[i] to
	 * TIMING_CTRL_0 + i*4. Those bytes come from the panel DT
	 * qcom,mdss-dsi-panel-timings (copied in dsi_manager enable_phy),
	 * not dphy_timing_calc. calc still fills phy->timing for host
	 * clk_pre/post. STRENGTH_0 stays 0xff (CAF 8992 is 0x77).
	 */
	if (phy->has_dphy_panel_timings) {
		const u8 *t = phy->dphy_panel_timings;

		pr_info("talkman-pll20: 3.10 phy timings %*ph\n", 12, t);
		writel(t[0], base + REG_DSI_20nm_PHY_TIMING_CTRL_0);
		writel(t[1], base + REG_DSI_20nm_PHY_TIMING_CTRL_1);
		writel(t[2], base + REG_DSI_20nm_PHY_TIMING_CTRL_2);
		writel(t[3], base + REG_DSI_20nm_PHY_TIMING_CTRL_3);
		writel(t[4], base + REG_DSI_20nm_PHY_TIMING_CTRL_4);
		writel(t[5], base + REG_DSI_20nm_PHY_TIMING_CTRL_5);
		writel(t[6], base + REG_DSI_20nm_PHY_TIMING_CTRL_6);
		writel(t[7], base + REG_DSI_20nm_PHY_TIMING_CTRL_7);
		writel(t[8], base + REG_DSI_20nm_PHY_TIMING_CTRL_8);
		writel(t[9], base + REG_DSI_20nm_PHY_TIMING_CTRL_9);
		writel(t[10], base + REG_DSI_20nm_PHY_TIMING_CTRL_10);
		writel(t[11], base + REG_DSI_20nm_PHY_TIMING_CTRL_11);
	} else {
		writel(DSI_20nm_PHY_TIMING_CTRL_0_CLK_ZERO(timing->clk_zero),
		       base + REG_DSI_20nm_PHY_TIMING_CTRL_0);
		writel(DSI_20nm_PHY_TIMING_CTRL_1_CLK_TRAIL(timing->clk_trail),
		       base + REG_DSI_20nm_PHY_TIMING_CTRL_1);
		writel(DSI_20nm_PHY_TIMING_CTRL_2_CLK_PREPARE(timing->clk_prepare),
		       base + REG_DSI_20nm_PHY_TIMING_CTRL_2);
		if (timing->clk_zero & BIT(8))
			writel(DSI_20nm_PHY_TIMING_CTRL_3_CLK_ZERO_8,
			       base + REG_DSI_20nm_PHY_TIMING_CTRL_3);
		else
			writel(0, base + REG_DSI_20nm_PHY_TIMING_CTRL_3);
		writel(DSI_20nm_PHY_TIMING_CTRL_4_HS_EXIT(timing->hs_exit),
		       base + REG_DSI_20nm_PHY_TIMING_CTRL_4);
		writel(DSI_20nm_PHY_TIMING_CTRL_5_HS_ZERO(timing->hs_zero),
		       base + REG_DSI_20nm_PHY_TIMING_CTRL_5);
		writel(DSI_20nm_PHY_TIMING_CTRL_6_HS_PREPARE(timing->hs_prepare),
		       base + REG_DSI_20nm_PHY_TIMING_CTRL_6);
		writel(DSI_20nm_PHY_TIMING_CTRL_7_HS_TRAIL(timing->hs_trail),
		       base + REG_DSI_20nm_PHY_TIMING_CTRL_7);
		writel(DSI_20nm_PHY_TIMING_CTRL_8_HS_RQST(timing->hs_rqst),
		       base + REG_DSI_20nm_PHY_TIMING_CTRL_8);
		writel(DSI_20nm_PHY_TIMING_CTRL_9_TA_GO(timing->ta_go) |
		       DSI_20nm_PHY_TIMING_CTRL_9_TA_SURE(timing->ta_sure),
		       base + REG_DSI_20nm_PHY_TIMING_CTRL_9);
		writel(DSI_20nm_PHY_TIMING_CTRL_10_TA_GET(timing->ta_get),
		       base + REG_DSI_20nm_PHY_TIMING_CTRL_10);
		writel(DSI_20nm_PHY_TIMING_CTRL_11_TRIG3_CMD(0),
		       base + REG_DSI_20nm_PHY_TIMING_CTRL_11);
	}
}

static void dsi_20nm_phy_regulator_ctrl(struct msm_dsi_phy *phy, bool enable)
{
	void __iomem *base = phy->reg_base;

	if (!enable) {
		writel(0, base + REG_DSI_20nm_PHY_REGULATOR_CAL_PWR_CFG);
		return;
	}

	if (phy->regulator_ldo_mode) {
		writel(0x1d, phy->base + REG_DSI_20nm_PHY_LDO_CNTRL);
		return;
	}

	/* non LDO mode */
	writel(0x03, base + REG_DSI_20nm_PHY_REGULATOR_CTRL_1);
	writel(0x03, base + REG_DSI_20nm_PHY_REGULATOR_CTRL_2);
	writel(0x00, base + REG_DSI_20nm_PHY_REGULATOR_CTRL_3);
	writel(0x20, base + REG_DSI_20nm_PHY_REGULATOR_CTRL_4);
	writel(0x01, base + REG_DSI_20nm_PHY_REGULATOR_CAL_PWR_CFG);
	writel(0x00, phy->base + REG_DSI_20nm_PHY_LDO_CNTRL);
	writel(0x03, base + REG_DSI_20nm_PHY_REGULATOR_CTRL_0);
}

static int dsi_20nm_phy_enable(struct msm_dsi_phy *phy,
			       struct msm_dsi_phy_clk_request *clk_req)
{
	struct msm_dsi_dphy_timing *timing = &phy->timing;
	int i;
	void __iomem *base = phy->base;
	u32 cfg_4[4] = {0x20, 0x40, 0x20, 0x00};
	u32 val;

	DBG("");

	if (msm_dsi_dphy_timing_calc(timing, clk_req)) {
		DRM_DEV_ERROR(&phy->pdev->dev,
			      "%s: D-PHY timing calculation failed\n", __func__);
		return -EINVAL;
	}

	dsi_20nm_phy_regulator_ctrl(phy, true);

	writel(0xff, base + REG_DSI_20nm_PHY_STRENGTH_0);

	val = readl(base + REG_DSI_20nm_PHY_GLBL_TEST_CTRL);
	if (phy->id == DSI_1 && phy->usecase == MSM_DSI_PHY_STANDALONE)
		val |= DSI_20nm_PHY_GLBL_TEST_CTRL_BITCLK_HS_SEL;
	else
		val &= ~DSI_20nm_PHY_GLBL_TEST_CTRL_BITCLK_HS_SEL;
	writel(val, base + REG_DSI_20nm_PHY_GLBL_TEST_CTRL);

	for (i = 0; i < 4; i++) {
		writel((i >> 1) * 0x40, base + REG_DSI_20nm_PHY_LN_CFG_3(i));
		writel(0x01, base + REG_DSI_20nm_PHY_LN_TEST_STR_0(i));
		writel(0x46, base + REG_DSI_20nm_PHY_LN_TEST_STR_1(i));
		writel(0x02, base + REG_DSI_20nm_PHY_LN_CFG_0(i));
		writel(0xa0, base + REG_DSI_20nm_PHY_LN_CFG_1(i));
		writel(cfg_4[i], base + REG_DSI_20nm_PHY_LN_CFG_4(i));
	}

	writel(0x80, base + REG_DSI_20nm_PHY_LNCK_CFG_3);
	writel(0x01, base + REG_DSI_20nm_PHY_LNCK_TEST_STR0);
	writel(0x46, base + REG_DSI_20nm_PHY_LNCK_TEST_STR1);
	writel(0x00, base + REG_DSI_20nm_PHY_LNCK_CFG_0);
	writel(0xa0, base + REG_DSI_20nm_PHY_LNCK_CFG_1);
	writel(0x00, base + REG_DSI_20nm_PHY_LNCK_CFG_2);
	writel(0x00, base + REG_DSI_20nm_PHY_LNCK_CFG_4);

	dsi_20nm_dphy_set_timing(phy, timing);

	writel(0x00, base + REG_DSI_20nm_PHY_CTRL_1);

	writel(0x06, base + REG_DSI_20nm_PHY_STRENGTH_1);

	/* make sure everything is written before enable */
	wmb();
	writel(0x7f, base + REG_DSI_20nm_PHY_CTRL_0);

	return 0;
}

static void dsi_20nm_phy_disable(struct msm_dsi_phy *phy)
{
	writel(0, phy->base + REG_DSI_20nm_PHY_CTRL_0);
	dsi_20nm_phy_regulator_ctrl(phy, false);
}

static const struct regulator_bulk_data dsi_phy_20nm_regulators[] = {
	{ .supply = "vddio", .init_load_uA = 100000 }, /* 1.8 V */
	{ .supply = "vcca", .init_load_uA = 10000 }, /* 1.0 V */
};

const struct msm_dsi_phy_cfg dsi_phy_20nm_cfgs = {
	.has_phy_regulator = true,
	.regulator_data = dsi_phy_20nm_regulators,
	.num_regulators = ARRAY_SIZE(dsi_phy_20nm_regulators),
	.ops = {
		.enable = dsi_20nm_phy_enable,
		.disable = dsi_20nm_phy_disable,
		.pll_init = dsi_pll_20nm_init,
	},
	.min_pll_rate = VCO_MIN_RATE,
	.max_pll_rate = VCO_MAX_RATE,
	.io_start = { 0xfd998500, 0xfd9a0500 },
	.num_dsi_phy = 2,
};

const struct msm_dsi_phy_cfg dsi_phy_20nm_8992_cfgs = {
	.has_phy_regulator = true,
	.regulator_data = dsi_phy_20nm_regulators,
	.num_regulators = ARRAY_SIZE(dsi_phy_20nm_regulators),
	.ops = {
		.enable = dsi_20nm_phy_enable,
		.disable = dsi_20nm_phy_disable,
		.pll_init = dsi_pll_20nm_init,
	},
	.min_pll_rate = VCO_MIN_RATE,
	.max_pll_rate = VCO_MAX_RATE,
	.io_start = { 0xfd994500, 0xfd996500 },
	.num_dsi_phy = 2,
};
