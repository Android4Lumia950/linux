// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/pm_runtime.h>

#include "dsi_phy.h"
#include "dsi.xml.h"
#include "dsi_phy_20nm.xml.h"

/*
 * DSI PLL 20nm - clock diagram (eg: DSI0):
 *
 *                            +------+
 *  dsi0vco_clk ------+-------| ndiv |--- dsi0ndiv_clk
 *                     |       +------+
 *                     |          |
 *                     |          |     +-----+
 *                     |          o-----| /2  |--- dsi0indirect_path_div2_clk
 *                     |                +-----+
 *                     |                   |
 *                +----+----+              |
 *                | byte mux|--------------o
 *                +----+----+
 *                     |
 *                 +---+---+     +-----+
 *                 |  /4   |-----| /2  |--- dsi0pllbyte
 *                 +-------+     +-----+
 *
 *  dsi0vco_clk ---------------------------+
 *                                         |
 *                                    +---------+     +-----+
 *                                    |hr_oclk3 |-----| /2  |--- dsi0pll
 *                                    +---------+     +-----+
 */

#define POLL_MAX_READS			15
#define POLL_TIMEOUT_US			1000

#define VCO_REF_CLK_RATE		19200000
#define VCO_MIN_RATE			1000000000UL
#define VCO_MAX_RATE			2000000000UL

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

	struct msm_dsi_phy *phy;

	/* optional second analog window shared by the PLL0/PLL1 pair */
	void __iomem *pll_1_base;

	/* protects MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL (shared by mux/ndiv) */
	spinlock_t postdiv_lock;

	/* serializes analog resource enable/disable across the clk_ops */
	struct mutex res_lock;
	int resource_ref_cnt;

	/*
	 * Set while pll_20nm_register() is running. __clk_register() calls
	 * .recalc_rate/.get_parent on each clock as it registers, before
	 * the PLL has ever locked; ndiv/hr_oclk3/byte_mux gate hardware
	 * reads on this so those calls return cached defaults instead of
	 * touching an unpowered analog block.
	 */
	bool clk_registering;

	unsigned long vco_current_rate;
	unsigned long vco_ref_clk_rate;
	unsigned long vco_locking_rate;
	unsigned long vco_cached_rate;

	u32 cache_pll_trim_codes[2];
	u32 ndiv;		/* 1..15 */
	u32 hr_oclk3;		/* 1..255, HW stores div - 1 */
	u8 mux_index;

	bool is_init_locked;
	bool pll_en_90_phase;
};

#define to_pll_20nm_vco(x)	container_of(x, struct dsi_pll_20nm, vco_hw)
#define to_pll_20nm_mux(x)	container_of(x, struct dsi_pll_20nm, mux_hw)
#define to_pll_20nm_ndiv(x)	container_of(x, struct dsi_pll_20nm, ndiv_hw)
#define to_pll_20nm_hr_oclk3(x)	container_of(x, struct dsi_pll_20nm, hr_oclk3_hw)

/*
 * ndiv/hr_oclk3/byte_mux reads require the PLL to have locked at least
 * once; before that the analog block may be unpowered and a readl()
 * on it aborts. Writes only need to avoid the registration window,
 * since set_rate/set_parent take their own resource reference.
 */
static bool pll_20nm_analog_readable(struct dsi_pll_20nm *pll)
{
	return pll->phy && pll->phy->pll_on && !pll->clk_registering;
}

static bool pll_20nm_analog_writable(struct dsi_pll_20nm *pll)
{
	return pll->phy && !pll->clk_registering;
}

/*
 * Regulator/GDSC/AHB resources the analog block needs before its
 * registers are touched. Each accessor takes its own reference rather
 * than assuming the PHY's prepare/unprepare state.
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
				if (rc < 0)
					goto fail_enable;
			}
			rc = regulator_bulk_enable(phy->cfg->num_regulators,
						   phy->supplies);
			if (rc)
				goto fail_regulator;
			rc = pm_runtime_resume_and_get(dev);
			if (rc < 0)
				goto fail_runtime;
		} else {
			pm_runtime_put(dev);
			regulator_bulk_disable(phy->cfg->num_regulators,
					       phy->supplies);
			if (mdss_dev)
				pm_runtime_put(mdss_dev);
		}
	}
	mutex_unlock(&pll->res_lock);
	return 0;

fail_runtime:
	regulator_bulk_disable(phy->cfg->num_regulators, phy->supplies);
fail_regulator:
	if (mdss_dev)
		pm_runtime_put(mdss_dev);
fail_enable:
	pll->resource_ref_cnt--;
	mutex_unlock(&pll->res_lock);
	return rc;
}

static bool pll_20nm_poll_for_ready(struct dsi_pll_20nm *pll,
				    u32 nb_tries, u32 timeout_us)
{
	void __iomem *base = pll->phy->pll_base;
	bool pll_locked = false, pll_ready = false;
	u32 tries, val;

	tries = nb_tries;
	while (tries--) {
		val = readl(base + MMSS_DSI_PHY_PLL_RESET_SM);
		pll_locked = !!(val & BIT(5));

		if (pll_locked)
			break;

		udelay(timeout_us);
	}

	if (!pll_locked)
		goto out;

	tries = nb_tries;
	while (tries--) {
		val = readl(base + MMSS_DSI_PHY_PLL_RESET_SM);
		pll_ready = !!(val & BIT(6));

		if (pll_ready)
			break;

		udelay(timeout_us);
	}

out:
	DBG("DSI PLL is %slocked, %sready", pll_locked ? "" : "*not* ", pll_ready ? "" : "*not* ");

	return pll_locked && pll_ready;
}

static void pll_20nm_commit_common_block(void __iomem *pll_base)
{
	if (!pll_base)
		return;

	writel(0x82, pll_base + MMSS_DSI_PHY_PLL_PLL_VCOTAIL_EN);
	writel(0x2a, pll_base + MMSS_DSI_PHY_PLL_BIAS_EN_CLKBUFLR_EN);
	writel(0x2b, pll_base + MMSS_DSI_PHY_PLL_BIAS_EN_CLKBUFLR_EN);
	writel(0x02, pll_base + MMSS_DSI_PHY_PLL_RESETSM_CNTRL3);
}

static void pll_20nm_commit_common(void __iomem *pll_base)
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

static void pll_20nm_commit_loop_bw(void __iomem *pll_base)
{
	writel(0x03, pll_base + MMSS_DSI_PHY_PLL_PLL_IP_SETI);
	writel(0x3f, pll_base + MMSS_DSI_PHY_PLL_PLL_CP_SETI);
	writel(0x03, pll_base + MMSS_DSI_PHY_PLL_PLL_IP_SETP);
	writel(0x1f, pll_base + MMSS_DSI_PHY_PLL_PLL_CP_SETP);
	writel(0x77, pll_base + MMSS_DSI_PHY_PLL_PLL_CRCTRL);
}

static void pll_20nm_commit_powerdown(void __iomem *pll_base)
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
	s64 multiplier = BIT(20);
	s64 duration, pll_comp_val;
	s64 dec_start_multiple, dec_start;
	s32 div_frac_start;
	s64 dec_start1, dec_start2;
	s32 div_frac_start1, div_frac_start2, div_frac_start3;
	s64 pll_plllock_cmp1, pll_plllock_cmp2, pll_plllock_cmp3;

	duration = pll_en_90_phase ? 128 : 1024;

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

static void pll_20nm_commit_vco_rate(void __iomem *pll_base,
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

	writel(pll_en_90_phase ? 0x0d : 0x01,
	       pll_base + MMSS_DSI_PHY_PLL_PLLLOCK_CMP_EN);

	/* fixed_hr_oclk2: divide by 4 (HW stores div - 1) */
	writel(3, pll_base + MMSS_DSI_PHY_PLL_HR_OCLK2_DIVIDER);
}

static void pll_20nm_cache_trim_codes(struct dsi_pll_20nm *pll)
{
	void __iomem *base = pll->phy->pll_base;

	pll->cache_pll_trim_codes[0] = readl(base + MMSS_DSI_PHY_PLL_CORE_KVCO_CODE);
	pll->cache_pll_trim_codes[1] = readl(base + MMSS_DSI_PHY_PLL_CORE_VCO_TUNE);
}

static void pll_20nm_override_trim_codes(struct dsi_pll_20nm *pll)
{
	void __iomem *base = pll->phy->pll_base;
	u32 reg_data;

	reg_data = (pll->cache_pll_trim_codes[0] & 0x3f) | BIT(5);
	writel(reg_data, base + MMSS_DSI_PHY_PLL_KVCO_CODE);
	reg_data = (pll->cache_pll_trim_codes[1] & 0x7f) | BIT(7);
	writel(reg_data, base + MMSS_DSI_PHY_PLL_PLL_VCO_TUNE);
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

/* First lock at a given rate: run the full KVCO background calibration */
static int pll_20nm_vco_init_lock(struct dsi_pll_20nm *pll)
{
	pll_20nm_config_resetsm(pll->phy->pll_base);
	pll_20nm_config_vco_start(pll->phy->pll_base);

	if (!pll_20nm_poll_for_ready(pll, POLL_MAX_READS, POLL_TIMEOUT_US))
		return -EINVAL;

	pll_20nm_cache_trim_codes(pll);
	return 0;
}

/* Re-lock at an already-calibrated rate: reuse the cached trim codes */
static int pll_20nm_vco_relock(struct dsi_pll_20nm *pll)
{
	pll_20nm_override_trim_codes(pll);
	pll_20nm_config_bypass_cal(pll->phy->pll_base);
	pll_20nm_config_vco_start(pll->phy->pll_base);

	if (!pll_20nm_poll_for_ready(pll, POLL_MAX_READS, POLL_TIMEOUT_US))
		return -EINVAL;

	return 0;
}

static int pll_20nm_vco_enable_seq(struct dsi_pll_20nm *pll)
{
	struct dsi_pll_20nm_vco_calc vco_calc;
	int rc;

	pll_20nm_commit_common_block(pll->pll_1_base);
	pll_20nm_commit_common_block(pll->phy->pll_base);
	pll_20nm_commit_common(pll->phy->pll_base);
	pll_20nm_commit_loop_bw(pll->phy->pll_base);

	pll_20nm_vco_rate_calc(&vco_calc, pll->vco_current_rate,
			       pll->vco_ref_clk_rate, pll->pll_en_90_phase);
	pll_20nm_commit_vco_rate(pll->phy->pll_base, &vco_calc,
				 pll->pll_en_90_phase);

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

/*
 * VCO clock callbacks
 */
static int dsi_pll_20nm_vco_set_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long parent_rate)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_vco(hw);

	DBG("DSI PLL%d rate=%lu, parent's=%lu", pll->phy->id, rate, parent_rate);

	/* analog is programmed from vco_prepare(); this only caches the rate */
	pll->vco_current_rate = rate;
	pll->vco_ref_clk_rate = parent_rate ? parent_rate : VCO_REF_CLK_RATE;

	return 0;
}

static unsigned long dsi_pll_20nm_vco_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_vco(hw);

	return pll->vco_current_rate;
}

static int dsi_pll_20nm_vco_prepare(struct clk_hw *hw)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_vco(hw);
	int rc;

	DBG("");

	if (unlikely(pll->phy->pll_on))
		return 0;

	if (!pll->vco_current_rate)
		return -EINVAL;

	rc = pll_20nm_resource_enable(pll, true);
	if (rc)
		return rc;

	if (pll->vco_cached_rate && pll->vco_cached_rate == pll->vco_current_rate)
		dsi_pll_20nm_vco_set_rate(hw, pll->vco_cached_rate,
					  pll->vco_ref_clk_rate);

	rc = pll_20nm_vco_enable_seq(pll);

	/* power down the shared PLL1 analog once locking is done */
	pll_20nm_commit_powerdown(pll->pll_1_base);

	if (rc) {
		pll_20nm_resource_enable(pll, false);
		DRM_DEV_ERROR(&pll->phy->pdev->dev, "DSI PLL lock failed\n");
		return rc;
	}

	DBG("DSI PLL lock success");
	pll->phy->pll_on = true;

	return 0;
}

static void dsi_pll_20nm_vco_unprepare(struct clk_hw *hw)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_vco(hw);

	DBG("");

	if (unlikely(!pll->phy->pll_on))
		return;

	pll->vco_cached_rate = pll->vco_current_rate;
	pll_20nm_commit_powerdown(pll->pll_1_base);
	pll_20nm_commit_powerdown(pll->phy->pll_base);
	pll_20nm_resource_enable(pll, false);

	pll->phy->pll_on = false;
}

static int dsi_pll_20nm_vco_determine_rate(struct clk_hw *hw,
					   struct clk_rate_request *req)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_vco(hw);

	req->rate = clamp_t(unsigned long, req->rate,
			    pll->phy->cfg->min_pll_rate, pll->phy->cfg->max_pll_rate);

	return 0;
}

static const struct clk_ops clk_ops_dsi_pll_20nm_vco = {
	.determine_rate = dsi_pll_20nm_vco_determine_rate,
	.set_rate = dsi_pll_20nm_vco_set_rate,
	.recalc_rate = dsi_pll_20nm_vco_recalc_rate,
	.prepare = dsi_pll_20nm_vco_prepare,
	.unprepare = dsi_pll_20nm_vco_unprepare,
};

/*
 * Byte-clock source mux. Shares MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL
 * with the ndiv post-divider: bits[3:0] are ndiv, bit 5 selects the mux
 * parent (0 = VCO, 1 = ndiv/2), bit 7 must be set on every write.
 */
static u8 dsi_20nm_byte_mux_get_parent(struct clk_hw *hw)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_mux(hw);
	u32 val;

	if (!pll_20nm_analog_readable(pll))
		return 0;

	if (pll_20nm_resource_enable(pll, true))
		return 0;

	val = readl(pll->phy->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL);
	pll_20nm_resource_enable(pll, false);

	return !!(val & BIT(5));
}

static int dsi_20nm_byte_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_mux(hw);
	unsigned long flags;
	u32 val;
	int rc;

	pll->mux_index = index;

	if (!pll_20nm_analog_writable(pll))
		return 0;

	rc = pll_20nm_resource_enable(pll, true);
	if (rc)
		return rc;

	spin_lock_irqsave(&pll->postdiv_lock, flags);
	val = readl(pll->phy->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL);
	val |= BIT(7);
	val &= ~BIT(5);
	val |= index << 5;
	writel(val, pll->phy->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL);
	spin_unlock_irqrestore(&pll->postdiv_lock, flags);

	pll_20nm_resource_enable(pll, false);
	return 0;
}

static int dsi_20nm_byte_mux_prepare(struct clk_hw *hw)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_mux(hw);

	return dsi_20nm_byte_mux_set_parent(hw, pll->mux_index);
}

static const struct clk_ops clk_ops_dsi_20nm_byte_mux = {
	.determine_rate = __clk_mux_determine_rate_closest,
	.set_parent = dsi_20nm_byte_mux_set_parent,
	.get_parent = dsi_20nm_byte_mux_get_parent,
	.prepare = dsi_20nm_byte_mux_prepare,
};

/*
 * ndiv post-divider clock callbacks
 */
#define div_mask(width)	((1 << (width)) - 1)

static unsigned long dsi_20nm_ndiv_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_ndiv(hw);
	u32 div;

	if (!pll_20nm_analog_readable(pll))
		return DIV_ROUND_UP_ULL((u64)parent_rate, pll->ndiv ?: 1);

	if (pll_20nm_resource_enable(pll, true))
		return DIV_ROUND_UP_ULL((u64)parent_rate, pll->ndiv ?: 1);

	div = readl(pll->phy->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL) &
	      div_mask(4);
	pll_20nm_resource_enable(pll, false);
	if (div)
		pll->ndiv = div;

	return DIV_ROUND_UP_ULL((u64)parent_rate, pll->ndiv ?: 1);
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
	pll->ndiv = clamp(div, 1, 15);

	if (!pll_20nm_analog_writable(pll))
		return 0;

	rc = pll_20nm_resource_enable(pll, true);
	if (rc)
		return rc;

	spin_lock_irqsave(&pll->postdiv_lock, flags);
	val = readl(pll->phy->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL);
	val &= ~div_mask(4);
	val |= pll->ndiv;
	writel(val, pll->phy->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL);
	spin_unlock_irqrestore(&pll->postdiv_lock, flags);

	pll_20nm_resource_enable(pll, false);
	return 0;
}

static int dsi_20nm_ndiv_determine_rate(struct clk_hw *hw,
					struct clk_rate_request *req)
{
	return divider_determine_rate(hw, req, NULL, 4, CLK_DIVIDER_ONE_BASED);
}

static int dsi_20nm_ndiv_prepare(struct clk_hw *hw)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_ndiv(hw);
	unsigned long flags;
	u32 val;
	int rc;

	if (!pll_20nm_analog_writable(pll))
		return 0;

	rc = pll_20nm_resource_enable(pll, true);
	if (rc)
		return rc;

	spin_lock_irqsave(&pll->postdiv_lock, flags);
	val = readl(pll->phy->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL);
	val &= ~div_mask(4);
	val |= pll->ndiv;
	writel(val, pll->phy->pll_base + MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL);
	spin_unlock_irqrestore(&pll->postdiv_lock, flags);

	pll_20nm_resource_enable(pll, false);
	return 0;
}

static const struct clk_ops clk_ops_dsi_20nm_ndiv = {
	.recalc_rate = dsi_20nm_ndiv_recalc_rate,
	.set_rate = dsi_20nm_ndiv_set_rate,
	.determine_rate = dsi_20nm_ndiv_determine_rate,
	.prepare = dsi_20nm_ndiv_prepare,
};

/*
 * hr_oclk3 post-divider clock callbacks (dedicated register, div - 1
 * stored in HW, 1..255)
 */
static unsigned long dsi_20nm_hr_oclk3_recalc_rate(struct clk_hw *hw,
						   unsigned long parent_rate)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_hr_oclk3(hw);
	u32 div;

	if (!pll_20nm_analog_readable(pll))
		return DIV_ROUND_UP_ULL((u64)parent_rate, pll->hr_oclk3 ?: 1);

	if (pll_20nm_resource_enable(pll, true))
		return DIV_ROUND_UP_ULL((u64)parent_rate, pll->hr_oclk3 ?: 1);

	div = readl(pll->phy->pll_base + MMSS_DSI_PHY_PLL_HR_OCLK3_DIVIDER) + 1;
	pll_20nm_resource_enable(pll, false);
	if (div > 1)
		pll->hr_oclk3 = div;

	return DIV_ROUND_UP_ULL((u64)parent_rate, pll->hr_oclk3 ?: 1);
}

static int dsi_20nm_hr_oclk3_set_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long parent_rate)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_hr_oclk3(hw);
	int div, rc;

	div = divider_get_val(rate, parent_rate, NULL, 8, 0);
	if (div < 0)
		return div;
	div = min(div, 254);
	pll->hr_oclk3 = div + 1;

	if (!pll_20nm_analog_writable(pll))
		return 0;

	rc = pll_20nm_resource_enable(pll, true);
	if (rc)
		return rc;

	writel(div, pll->phy->pll_base + MMSS_DSI_PHY_PLL_HR_OCLK3_DIVIDER);
	pll_20nm_resource_enable(pll, false);
	return 0;
}

static int dsi_20nm_hr_oclk3_determine_rate(struct clk_hw *hw,
					    struct clk_rate_request *req)
{
	return divider_determine_rate(hw, req, NULL, 8, 0);
}

static int dsi_20nm_hr_oclk3_prepare(struct clk_hw *hw)
{
	struct dsi_pll_20nm *pll = to_pll_20nm_hr_oclk3(hw);
	int rc;

	if (!pll_20nm_analog_writable(pll) || !pll->hr_oclk3)
		return 0;

	rc = pll_20nm_resource_enable(pll, true);
	if (rc)
		return rc;

	writel(pll->hr_oclk3 - 1,
	       pll->phy->pll_base + MMSS_DSI_PHY_PLL_HR_OCLK3_DIVIDER);
	pll_20nm_resource_enable(pll, false);
	return 0;
}

static const struct clk_ops clk_ops_dsi_20nm_hr_oclk3 = {
	.recalc_rate = dsi_20nm_hr_oclk3_recalc_rate,
	.set_rate = dsi_20nm_hr_oclk3_set_rate,
	.determine_rate = dsi_20nm_hr_oclk3_determine_rate,
	.prepare = dsi_20nm_hr_oclk3_prepare,
};

static int pll_20nm_register(struct dsi_pll_20nm *pll, struct clk_hw **provided_clocks)
{
	char clk_name[32], mux_name[32], ndiv_name[32], hr_oclk3_name[32];
	struct clk_init_data vco_init = {
		.parent_data = &(const struct clk_parent_data) {
			.fw_name = "ref",
		},
		.num_parents = 1,
		.name = clk_name,
		.flags = CLK_IGNORE_UNUSED,
		.ops = &clk_ops_dsi_pll_20nm_vco,
	};
	struct device *dev = &pll->phy->pdev->dev;
	struct clk_hw *hw, *indirect_path_div2, *hr_oclk2;
	struct clk_init_data mux_init, ndiv_init, hr_oclk3_init;
	const struct clk_hw *mux_parents[2];
	const struct clk_hw *ndiv_parent[1];
	const struct clk_hw *hr_oclk3_parent[1];
	int ret;

	DBG("DSI%d", pll->phy->id);

	snprintf(clk_name, sizeof(clk_name), "dsi%dvco_clk", pll->phy->id);
	pll->vco_hw.init = &vco_init;
	ret = devm_clk_hw_register(dev, &pll->vco_hw);
	if (ret)
		return ret;

	/* ndiv, bits 0-3 in MMSS_DSI_PHY_PLL_POST_DIVIDER_CONTROL */
	snprintf(ndiv_name, sizeof(ndiv_name), "dsi%dndiv_clk", pll->phy->id);
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
		 pll->phy->id);
	indirect_path_div2 = devm_clk_hw_register_fixed_factor_parent_hw(dev,
			clk_name, &pll->ndiv_hw, CLK_SET_RATE_PARENT, 1, 2);
	if (IS_ERR(indirect_path_div2))
		return PTR_ERR(indirect_path_div2);

	snprintf(mux_name, sizeof(mux_name), "dsi%dbyte_mux", pll->phy->id);
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

	snprintf(clk_name, sizeof(clk_name), "dsi%dhr_oclk2_clk", pll->phy->id);
	hr_oclk2 = devm_clk_hw_register_fixed_factor_parent_hw(dev, clk_name,
			&pll->mux_hw, CLK_SET_RATE_PARENT, 1, 4);
	if (IS_ERR(hr_oclk2))
		return PTR_ERR(hr_oclk2);

	snprintf(clk_name, sizeof(clk_name), "dsi%dpllbyte", pll->phy->id);

	/* DSI Byte clock = VCO_CLK / byte_mux / 4 / 2 */
	hw = devm_clk_hw_register_fixed_factor_parent_hw(dev, clk_name,
			hr_oclk2, CLK_SET_RATE_PARENT, 1, 2);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	provided_clocks[DSI_BYTE_PLL_CLK] = hw;

	/*
	 * hr_oclk3 is a dedicated post-divider (own register, not shared
	 * with ndiv/byte_mux) feeding the pixel clock.
	 */
	snprintf(hr_oclk3_name, sizeof(hr_oclk3_name), "dsi%dhr_oclk3_clk",
		 pll->phy->id);
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

	snprintf(clk_name, sizeof(clk_name), "dsi%dpll", pll->phy->id);

	/* DSI pixel clock = VCO_CLK / hr_oclk3 / 2 */
	hw = devm_clk_hw_register_fixed_factor_parent_hw(dev, clk_name,
			&pll->hr_oclk3_hw, CLK_SET_RATE_PARENT, 1, 2);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
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
	 * Only DSI0's PLL provides the byte/pixel clock tree; DSI1's PLL
	 * registers exist but aren't consumed as clock sources.
	 */
	if (phy->id != DSI_0)
		return 0;

	pll = devm_kzalloc(&pdev->dev, sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return -ENOMEM;

	DBG("PLL%d", phy->id);

	pll->phy = phy;
	pll->vco_ref_clk_rate = VCO_REF_CLK_RATE;
	pll->pll_en_90_phase = true;
	pll->ndiv = 1;
	pll->hr_oclk3 = 1;
	pll->mux_index = 1;
	spin_lock_init(&pll->postdiv_lock);
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

	return 0;
}

static void dsi_20nm_dphy_set_timing(struct msm_dsi_phy *phy,
		struct msm_dsi_dphy_timing *timing)
{
	void __iomem *base = phy->base;

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
	{ .supply = "vddio", .init_load_uA = 100000 },	/* 1.8 V */
	{ .supply = "vcca", .init_load_uA = 10000 },	/* 1.0 V */
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
