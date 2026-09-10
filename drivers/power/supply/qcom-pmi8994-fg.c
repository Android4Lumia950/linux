// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PMI8994 QPNP fuel gauge.
 *
 * SRAM is reached through the MEMIF peripheral (0x4400). Register map
 * and LSB weights match the QPNP FG block in PMI8994 / PMI8950.
 */

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>

#define FG_SOC_BASE			0x4000
#define FG_BATT_BASE			0x4100
#define FG_MEMIF_BASE			0x4400

#define REG_PERPH_TYPE			0x04
#define REG_PERPH_SUBTYPE		0x05
#define REG_INT_RT_STS			0x10

#define FG_SUBTYPE_SOC			0x09
#define FG_SUBTYPE_BATT			0x0a
#define FG_SUBTYPE_MEMIF		0x0c

#define SOC_MONOTONIC_SOC		0x09
#define SOC_BOOT_MOD			0x50
#define SOC_RESTART			0x51
#define NO_OTP_PROF_RELOAD		BIT(6)
#define REDO_FIRST_ESTIMATE		BIT(3)
#define RESTART_GO			BIT(0)
#define FIRST_EST_DONE_BIT		BIT(5)

#define MEM_INTF_CFG			0x40
#define MEM_INTF_CTL			0x41
#define MEM_INTF_ADDR_LSB		0x42
#define MEM_INTF_WR_DATA0		0x48
#define MEM_INTF_RD_DATA0		0x4c
#define RIF_MEM_ACCESS_REQ		BIT(7)
#define INTF_CTL_BURST			BIT(7)
#define INTF_CTL_WR_EN			BIT(6)
#define MEM_AVAIL_BIT			BIT(0)

#define RAM_OFFSET			0x400
#define BATT_TEMP_ADDR			0x550
#define BATT_TEMP_OFF			2
#define VOLTAGE_ADDR			0x5cc
#define VOLTAGE_OFF			1
#define CURRENT_OFF			3
#define BATT_ID_ADDR			0x594
#define BATT_ID_OFF			1
#define BATT_ID_INFO_OFF		3
#define BATT_PROFILE_OFFSET		0x4c0
#define PROFILE_INTEGRITY_REG		0x53c
#define PROFILE_INTEGRITY_BIT		BIT(0)
#define THERMAL_COEFF_ADDR		0x444
#define THERMAL_COEFF_OFF		2
#define THERMAL_COEFF_N_BYTES		6
#define JEITA_ADDR			0x454
#define CUTOFF_VOLTAGE_ADDR		0x40c
#define TERM_CURRENT_ADDR		0x40c
#define TERM_CURRENT_OFF		2
#define CHG_TERM_ADDR			0x4f8
#define CHG_TERM_OFF			2
#define RESUME_SOC_ADDR			0x45c
#define RESUME_SOC_OFF			1
#define EXTERNAL_SENSE_SELECT		0x4ac
#define EXTERNAL_SENSE_OFF		2
#define BATT_TEMP_CNTRL_OFF		3
#define BATT_TEMP_CNTRL_MASK		0x17
#define TEMP_SENSE_ALWAYS_BIT		BIT(1)
#define PATCH_NEG_CURRENT_BIT		BIT(3)
#define BATT_IDED			BIT(3)
#define FG_PROFILE_LEN			128
#define PROFILE_COMPARE_LEN		32

#define LSB_16B_NUM			152587
#define LSB_16B_DEN			1000
#define LSB_8B				9800
#define TEMP_LSB_16B			625
#define DECIKELVIN			2730

#define FULL_SOC_RAW			0xff
#define MEMIF_TIMEOUT_MS		1500
#define FIRST_EST_TIMEOUT_MS		2000
#define BATT_ID_TIMEOUT_MS		250
#define TELEM_CACHE_MS			1000
#define MAX_SOC_TRIES			5

static const u8 bias_ua[] = {
	[1] = 5,
	[2] = 15,
	[3] = 150,
};

struct pmi8994_fg {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct mutex lock;
	u32 soc_base;
	u32 batt_base;
	u32 mem_base;
	bool unknown_battery;
	int id_kohm;
	int max_voltage_uv;
	int nom_cap_uah;
	const char *batt_type;
	u8 thermal_coeff[THERMAL_COEFF_N_BYTES];
	bool has_thermal_coeff;
	s32 jeita_decidegc[4];
	bool has_jeita;
	u32 cutoff_mv;
	u32 iterm_ma;
	u32 chg_iterm_ma;
	u32 resume_soc;
	unsigned long telem_jiffies;
	int voltage_uv;
	int current_ua;
	int temp_decidegc;
	bool telem_valid;
};

static int fg_read(struct pmi8994_fg *fg, u16 addr, u8 *val, int len)
{
	return regmap_bulk_read(fg->regmap, addr, val, len);
}

static int fg_write(struct pmi8994_fg *fg, u16 addr, const u8 *val, int len)
{
	return regmap_bulk_write(fg->regmap, addr, val, len);
}

static int fg_masked_write(struct pmi8994_fg *fg, u16 addr, u8 mask, u8 val)
{
	return regmap_update_bits(fg->regmap, addr, mask, val);
}

static bool fg_mem_available(struct pmi8994_fg *fg)
{
	unsigned int sts, cfg;
	int rc;

	rc = regmap_read(fg->regmap, fg->mem_base + REG_INT_RT_STS, &sts);
	if (rc || !(sts & MEM_AVAIL_BIT))
		return false;

	rc = regmap_read(fg->regmap, fg->mem_base + MEM_INTF_CFG, &cfg);
	if (rc || !(cfg & RIF_MEM_ACCESS_REQ))
		return false;

	return true;
}

static int fg_req_access(struct pmi8994_fg *fg)
{
	unsigned long timeout;
	int rc;

	if (fg_mem_available(fg))
		return 0;

	rc = fg_masked_write(fg, fg->mem_base + MEM_INTF_CFG,
			     RIF_MEM_ACCESS_REQ, RIF_MEM_ACCESS_REQ);
	if (rc)
		return rc;

	timeout = jiffies + msecs_to_jiffies(MEMIF_TIMEOUT_MS);
	while (!fg_mem_available(fg)) {
		if (time_after(jiffies, timeout)) {
			dev_info_ratelimited(fg->dev,
					     "MEMIF access timed out\n");
			return -ETIMEDOUT;
		}
		usleep_range(1000, 2000);
	}

	return 0;
}

static int fg_release_access(struct pmi8994_fg *fg)
{
	return fg_masked_write(fg, fg->mem_base + MEM_INTF_CFG,
			       RIF_MEM_ACCESS_REQ, 0);
}

static int fg_set_ram_addr(struct pmi8994_fg *fg, u16 address)
{
	u8 buf[2] = { address & 0xff, address >> 8 };

	return fg_write(fg, fg->mem_base + MEM_INTF_ADDR_LSB, buf, 2);
}

static int fg_config_access(struct pmi8994_fg *fg, bool write, bool burst)
{
	u8 ctl = (write ? INTF_CTL_WR_EN : 0) | (burst ? INTF_CTL_BURST : 0);

	return fg_write(fg, fg->mem_base + MEM_INTF_CTL, &ctl, 1);
}

static int fg_mem_read(struct pmi8994_fg *fg, u16 address, int offset,
		       u8 *val, int len)
{
	int rc, chunk;
	u8 *dst = val;

	if (offset > 3)
		return -EINVAL;

	address += offset;
	offset = address % 4;
	address = (address / 4) * 4;

	guard(mutex)(&fg->lock);

	rc = fg_req_access(fg);
	if (rc)
		return rc;

	rc = fg_config_access(fg, false, len > 4);
	if (rc)
		goto out;

	rc = fg_set_ram_addr(fg, address);
	if (rc)
		goto out;

	while (len > 0) {
		chunk = min(len, 4 - offset);
		rc = fg_read(fg, fg->mem_base + MEM_INTF_RD_DATA0 + offset,
			     dst, chunk);
		if (rc)
			goto out;
		dst += chunk;
		len -= chunk;
		if (len > 0) {
			address += 4;
			offset = 0;
			rc = fg_set_ram_addr(fg, address);
			if (rc)
				goto out;
		}
	}

out:
	fg_release_access(fg);
	return rc;
}

static int fg_mem_write(struct pmi8994_fg *fg, u16 address, int offset,
			const u8 *val, int len)
{
	int rc, chunk;
	u8 word[4];
	const u8 *src = val;

	if (address < RAM_OFFSET || offset > 3)
		return -EINVAL;

	address += offset;
	offset = address % 4;
	address = (address / 4) * 4;

	guard(mutex)(&fg->lock);

	rc = fg_req_access(fg);
	if (rc)
		return rc;

	while (len > 0) {
		if (offset || len < 4) {
			rc = fg_config_access(fg, false, false);
			if (rc)
				goto out;
			rc = fg_set_ram_addr(fg, address);
			if (rc)
				goto out;
			rc = fg_read(fg, fg->mem_base + MEM_INTF_RD_DATA0,
				     word, 4);
			if (rc)
				goto out;
			chunk = min(len, 4 - offset);
			memcpy(word + offset, src, chunk);
		} else {
			chunk = 4;
			memcpy(word, src, 4);
		}

		rc = fg_config_access(fg, true, len - chunk > 0);
		if (rc)
			goto out;
		rc = fg_set_ram_addr(fg, address);
		if (rc)
			goto out;
		rc = fg_write(fg, fg->mem_base + MEM_INTF_WR_DATA0, word, 4);
		if (rc)
			goto out;

		src += chunk;
		len -= chunk;
		address += 4;
		offset = 0;
	}

out:
	fg_release_access(fg);
	return rc;
}

static int fg_mem_masked_write(struct pmi8994_fg *fg, u16 address, int offset,
			       u8 mask, u8 val)
{
	u8 reg;
	int rc;

	rc = fg_mem_read(fg, address, offset, &reg, 1);
	if (rc)
		return rc;
	reg = (reg & ~mask) | (val & mask);
	return fg_mem_write(fg, address, offset, &reg, 1);
}

static int fg_read_u16_sram(struct pmi8994_fg *fg, u16 address, int offset,
			    u16 *out)
{
	u8 buf[2];
	int rc;

	rc = fg_mem_read(fg, address, offset, buf, 2);
	if (rc)
		return rc;
	*out = buf[0] | (buf[1] << 8);
	return 0;
}

static int fg_get_monotonic_soc(struct pmi8994_fg *fg)
{
	u8 cap[2];
	int rc, tries = 0;

	while (tries < MAX_SOC_TRIES) {
		rc = fg_read(fg, fg->soc_base + SOC_MONOTONIC_SOC, cap, 2);
		if (rc)
			return rc;
		if (cap[0] == cap[1])
			return cap[0];
		tries++;
	}

	return -EINVAL;
}

static int fg_capacity_pct(struct pmi8994_fg *fg)
{
	int msoc = fg_get_monotonic_soc(fg);

	if (msoc < 0)
		return msoc;
	if (msoc == 0)
		return 0;
	if (msoc == FULL_SOC_RAW)
		return 100;
	return DIV_ROUND_CLOSEST((msoc - 1) * 98, FULL_SOC_RAW - 2) + 1;
}

static int fg_batt_temp_decidegc(struct pmi8994_fg *fg, int *temp)
{
	u16 raw;
	int rc;

	rc = fg_read_u16_sram(fg, BATT_TEMP_ADDR, BATT_TEMP_OFF, &raw);
	if (rc)
		return rc;
	*temp = (raw * TEMP_LSB_16B / 1000) - DECIKELVIN;
	return 0;
}

static int fg_voltage_uv(struct pmi8994_fg *fg, int *uv)
{
	u16 raw;
	int rc;

	rc = fg_read_u16_sram(fg, VOLTAGE_ADDR, VOLTAGE_OFF, &raw);
	if (rc)
		return rc;
	*uv = div_u64((u64)raw * LSB_16B_NUM, LSB_16B_DEN);
	return 0;
}

static int fg_current_ua(struct pmi8994_fg *fg, int *ua)
{
	u16 raw;
	s16 sraw;
	int rc;

	rc = fg_read_u16_sram(fg, VOLTAGE_ADDR, CURRENT_OFF, &raw);
	if (rc)
		return rc;
	sraw = (s16)raw;
	*ua = div_s64((s64)sraw * LSB_16B_NUM, LSB_16B_DEN);
	return 0;
}

static int fg_wait_batt_ided(struct pmi8994_fg *fg)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(BATT_ID_TIMEOUT_MS);
	unsigned int sts;
	int rc;

	do {
		rc = regmap_read(fg->regmap, fg->batt_base + REG_INT_RT_STS,
				 &sts);
		if (rc)
			return rc;
		if (sts & BATT_IDED)
			return 0;
		usleep_range(5000, 10000);
	} while (!time_after(jiffies, timeout));

	return -ETIMEDOUT;
}

static int fg_read_batt_id(struct pmi8994_fg *fg)
{
	u8 id, info;
	int rc, uv, bias;
	int64_t ohm;

	rc = fg_wait_batt_ided(fg);
	if (rc)
		return rc;

	rc = fg_mem_read(fg, BATT_ID_ADDR, BATT_ID_OFF, &id, 1);
	if (rc)
		return rc;
	rc = fg_mem_read(fg, BATT_ID_ADDR, BATT_ID_INFO_OFF, &info, 1);
	if (rc)
		return rc;

	bias = info & 0x3;
	if (!bias || bias > 3)
		return -EINVAL;

	uv = id * LSB_8B;
	ohm = div_u64((u64)uv, bias_ua[bias]);
	fg->id_kohm = DIV_ROUND_CLOSEST(ohm, 1000);
	return 0;
}

static bool fg_id_in_range(struct pmi8994_fg *fg, u32 expect_kohm, u32 range_pct)
{
	u32 limit = (expect_kohm * range_pct) / 100;
	u32 delta = abs((int)expect_kohm - fg->id_kohm);

	return delta <= limit;
}

static int fg_wait_first_est(struct pmi8994_fg *fg)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(FIRST_EST_TIMEOUT_MS);
	unsigned int sts;
	int rc;

	do {
		rc = regmap_read(fg->regmap, fg->soc_base + REG_INT_RT_STS, &sts);
		if (rc)
			return rc;
		if (sts & FIRST_EST_DONE_BIT)
			return 0;
		msleep(50);
	} while (!time_after(jiffies, timeout));

	return -ETIMEDOUT;
}

static int fg_load_profile(struct pmi8994_fg *fg, const u8 *profile, int len)
{
	u8 integrity = 0, present[FG_PROFILE_LEN];
	u8 restart = REDO_FIRST_ESTIMATE | RESTART_GO;
	int rc;

	if (len != FG_PROFILE_LEN)
		return -EINVAL;

	rc = fg_mem_read(fg, PROFILE_INTEGRITY_REG, 0, &integrity, 1);
	if (rc)
		return rc;

	rc = fg_mem_read(fg, BATT_PROFILE_OFFSET, 0, present, len);
	if (rc)
		return rc;

	if ((integrity & PROFILE_INTEGRITY_BIT) &&
	    !memcmp(present, profile, PROFILE_COMPARE_LEN)) {
		dev_dbg(fg->dev, "FG profile already loaded\n");
		return 0;
	}

	mutex_lock(&fg->lock);
	fg_release_access(fg);
	rc = fg_masked_write(fg, fg->soc_base + SOC_BOOT_MOD,
			     NO_OTP_PROF_RELOAD, 0);
	if (!rc)
		rc = fg_masked_write(fg, fg->soc_base + SOC_RESTART, restart, 0);
	mutex_unlock(&fg->lock);
	if (rc)
		return rc;

	rc = fg_mem_write(fg, BATT_PROFILE_OFFSET, 0, profile, len);
	if (rc)
		return rc;

	rc = fg_mem_masked_write(fg, PROFILE_INTEGRITY_REG, 0,
				 PROFILE_INTEGRITY_BIT, PROFILE_INTEGRITY_BIT);
	if (rc)
		return rc;

	mutex_lock(&fg->lock);
	rc = fg_masked_write(fg, fg->soc_base + SOC_BOOT_MOD,
			     NO_OTP_PROF_RELOAD, NO_OTP_PROF_RELOAD);
	if (!rc)
		rc = fg_masked_write(fg, fg->soc_base + SOC_RESTART,
				     restart, restart);
	mutex_unlock(&fg->lock);
	if (rc)
		return rc;

	rc = fg_wait_first_est(fg);
	if (rc)
		dev_err(fg->dev, "first estimate after profile load timed out\n");

	mutex_lock(&fg->lock);
	fg_masked_write(fg, fg->soc_base + SOC_BOOT_MOD, NO_OTP_PROF_RELOAD, 0);
	fg_masked_write(fg, fg->soc_base + SOC_RESTART, restart, 0);
	mutex_unlock(&fg->lock);

	return rc;
}

static void fg_uv_to_adc(s64 uv, u8 data[2])
{
	s16 raw = (s16)div64_s64(uv * LSB_16B_DEN, LSB_16B_NUM);

	data[0] = raw & 0xff;
	data[1] = (raw >> 8) & 0xff;
}

static int fg_apply_board_settings(struct pmi8994_fg *fg)
{
	u8 jeita[4], data[2];
	int i, rc;

	if (fg->has_thermal_coeff) {
		rc = fg_mem_write(fg, THERMAL_COEFF_ADDR, THERMAL_COEFF_OFF,
				  fg->thermal_coeff, THERMAL_COEFF_N_BYTES);
		if (rc)
			return rc;
	}

	if (fg->has_jeita) {
		for (i = 0; i < 4; i++)
			jeita[i] = (fg->jeita_decidegc[i] / 10) + 30;
		rc = fg_mem_write(fg, JEITA_ADDR, 0, jeita, 4);
		if (rc)
			return rc;
	}

	if (fg->cutoff_mv) {
		fg_uv_to_adc((s64)fg->cutoff_mv * 1000, data);
		rc = fg_mem_write(fg, CUTOFF_VOLTAGE_ADDR, 0, data, 2);
		if (rc)
			return rc;
	}

	if (fg->iterm_ma) {
		fg_uv_to_adc((s64)(-(s32)fg->iterm_ma) * 1000, data);
		rc = fg_mem_write(fg, TERM_CURRENT_ADDR, TERM_CURRENT_OFF,
				  data, 2);
		if (rc)
			return rc;
	}

	if (fg->chg_iterm_ma) {
		fg_uv_to_adc((s64)(-(s32)fg->chg_iterm_ma) * 1000, data);
		rc = fg_mem_write(fg, CHG_TERM_ADDR, CHG_TERM_OFF, data, 2);
		if (rc)
			return rc;
	}

	if (fg->resume_soc && fg->resume_soc < 100) {
		data[0] = DIV_ROUND_CLOSEST(fg->resume_soc * FULL_SOC_RAW, 100);
		rc = fg_mem_write(fg, RESUME_SOC_ADDR, RESUME_SOC_OFF, data, 1);
		if (rc)
			return rc;
	}

	rc = fg_mem_masked_write(fg, EXTERNAL_SENSE_SELECT, BATT_TEMP_CNTRL_OFF,
				 BATT_TEMP_CNTRL_MASK, TEMP_SENSE_ALWAYS_BIT);
	if (rc)
		return rc;

	return fg_mem_masked_write(fg, EXTERNAL_SENSE_SELECT, EXTERNAL_SENSE_OFF,
				   PATCH_NEG_CURRENT_BIT, PATCH_NEG_CURRENT_BIT);
}

static int fg_refresh_telem(struct pmi8994_fg *fg)
{
	int rc, uv, ua, temp;

	if (fg->telem_valid &&
	    time_before(jiffies,
			fg->telem_jiffies + msecs_to_jiffies(TELEM_CACHE_MS)))
		return 0;

	/* Keep the last sample if MEMIF is not already open. */
	if (fg->telem_valid) {
		unsigned long deadline = jiffies + msecs_to_jiffies(50);

		mutex_lock(&fg->lock);
		if (!fg_mem_available(fg)) {
			fg_masked_write(fg, fg->mem_base + MEM_INTF_CFG,
					RIF_MEM_ACCESS_REQ, RIF_MEM_ACCESS_REQ);
			while (!fg_mem_available(fg) &&
			       time_before(jiffies, deadline))
				usleep_range(1000, 2000);
			if (!fg_mem_available(fg)) {
				mutex_unlock(&fg->lock);
				return 0;
			}
		}
		mutex_unlock(&fg->lock);
	}

	rc = fg_batt_temp_decidegc(fg, &temp);
	if (rc)
		return rc;
	rc = fg_voltage_uv(fg, &uv);
	if (rc)
		return rc;
	rc = fg_current_ua(fg, &ua);
	if (rc)
		return rc;

	fg->temp_decidegc = temp;
	fg->voltage_uv = uv;
	fg->current_ua = ua;
	fg->telem_jiffies = jiffies;
	fg->telem_valid = true;
	return 0;
}

static int fg_get_status(struct pmi8994_fg *fg)
{
	struct power_supply *usb;
	union power_supply_propval val;
	int rc;

	(void)fg;
	usb = power_supply_get_by_name("qcom-smbchg-usb");
	if (!usb)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	rc = power_supply_get_property(usb, POWER_SUPPLY_PROP_STATUS, &val);
	power_supply_put(usb);
	if (rc)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	return val.intval;
}

static void fg_external_power_changed(struct power_supply *psy)
{
	power_supply_changed(psy);
}

static int fg_get_property(struct power_supply *psy,
			   enum power_supply_property psp,
			   union power_supply_propval *val)
{
	struct pmi8994_fg *fg = power_supply_get_drvdata(psy);
	int rc;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fg_get_status(fg);
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = fg_capacity_pct(fg);
		if (rc < 0)
			return rc;
		val->intval = rc;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = fg_refresh_telem(fg);
		if (rc)
			return rc;
		val->intval = fg->voltage_uv;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = fg_refresh_telem(fg);
		if (rc)
			return rc;
		/* Magnitude: UPower 1.91 treats current_now < 0 as discharge. */
		val->intval = fg->current_ua < 0 ? -fg->current_ua
						 : fg->current_ua;
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		rc = fg_refresh_telem(fg);
		if (rc)
			return rc;
		val->intval = fg->temp_decidegc;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		if (fg->max_voltage_uv <= 0)
			return -ENODATA;
		val->intval = fg->max_voltage_uv;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (fg->nom_cap_uah <= 0)
			return -ENODATA;
		val->intval = fg->nom_cap_uah;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = fg->unknown_battery ? "unknown" : fg->batt_type;
		return 0;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static const struct power_supply_desc fg_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = fg_props,
	.num_properties = ARRAY_SIZE(fg_props),
	.get_property = fg_get_property,
	.external_power_changed = fg_external_power_changed,
};

static int fg_parse_dt(struct pmi8994_fg *fg)
{
	struct device *dev = fg->dev;
	int rc, len;

	rc = device_property_read_u32(dev, "reg", &fg->soc_base);
	if (rc)
		fg->soc_base = FG_SOC_BASE;
	fg->batt_base = FG_BATT_BASE;
	fg->mem_base = FG_MEMIF_BASE;

	device_property_read_u32(dev, "qcom,fg-cutoff-voltage-mv", &fg->cutoff_mv);
	device_property_read_u32(dev, "qcom,fg-iterm-ma", &fg->iterm_ma);
	device_property_read_u32(dev, "qcom,fg-chg-iterm-ma", &fg->chg_iterm_ma);
	device_property_read_u32(dev, "qcom,resume-soc", &fg->resume_soc);

	len = device_property_count_u8(dev, "qcom,thermal-coefficients");
	if (len == THERMAL_COEFF_N_BYTES) {
		rc = device_property_read_u8_array(dev,
						   "qcom,thermal-coefficients",
						   fg->thermal_coeff,
						   THERMAL_COEFF_N_BYTES);
		if (!rc)
			fg->has_thermal_coeff = true;
	}

	if (dev->of_node &&
	    !of_property_read_s32(dev->of_node, "qcom,cool-bat-decidegc",
				  &fg->jeita_decidegc[0]) &&
	    !of_property_read_s32(dev->of_node, "qcom,warm-bat-decidegc",
				  &fg->jeita_decidegc[1]) &&
	    !of_property_read_s32(dev->of_node, "qcom,cold-bat-decidegc",
				  &fg->jeita_decidegc[2]) &&
	    !of_property_read_s32(dev->of_node, "qcom,hot-bat-decidegc",
				  &fg->jeita_decidegc[3]))
		fg->has_jeita = true;

	return 0;
}

static int fg_setup_profile(struct pmi8994_fg *fg)
{
	struct device *dev = fg->dev;
	u32 expect_kohm = 0, range_pct = 0;
	u8 *profile;
	int len, rc;

	device_property_read_string(dev, "qcom,battery-type", &fg->batt_type);
	if (!fg->batt_type)
		fg->batt_type = "qcom,pmi8994-fg";

	device_property_read_u32(dev, "qcom,batt-id-kohm", &expect_kohm);
	device_property_read_u32(dev, "qcom,batt-id-range-pct", &range_pct);

	if (expect_kohm && range_pct)
		rc = fg_read_batt_id(fg);
	else
		rc = -ENODATA;
	if (rc) {
		dev_warn(dev, "battery ID unreadable (%d), leaving SRAM profile\n",
			 rc);
		fg->unknown_battery = true;
		return fg_apply_board_settings(fg);
	}

	dev_info(dev, "battery ID %d kΩ (expect %u ±%u%%)\n",
		 fg->id_kohm, expect_kohm, range_pct);

	if (!expect_kohm || !range_pct ||
	    !fg_id_in_range(fg, expect_kohm, range_pct)) {
		fg->unknown_battery = true;
		dev_info(dev, "ID out of range; not loading profile\n");
		return fg_apply_board_settings(fg);
	}

	len = device_property_count_u8(dev, "qcom,fg-profile-data");
	if (len != FG_PROFILE_LEN) {
		dev_dbg(dev, "no 128-byte fg-profile-data, using SRAM as-is\n");
		return fg_apply_board_settings(fg);
	}

	profile = kcalloc(FG_PROFILE_LEN, sizeof(*profile), GFP_KERNEL);
	if (!profile)
		return -ENOMEM;
	rc = device_property_read_u8_array(dev, "qcom,fg-profile-data",
					   profile, FG_PROFILE_LEN);
	if (rc) {
		kfree(profile);
		return rc;
	}

	rc = fg_load_profile(fg, profile, len);
	kfree(profile);
	if (rc)
		dev_err(dev, "profile load failed (%d)\n", rc);

	return fg_apply_board_settings(fg);
}

static int fg_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = {};
	struct power_supply_battery_info *info;
	struct pmi8994_fg *fg;
	unsigned int type, subtype;
	int rc;

	fg = devm_kzalloc(&pdev->dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->dev = &pdev->dev;
	mutex_init(&fg->lock);

	fg->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!fg->regmap)
		return dev_err_probe(&pdev->dev, -ENODEV, "no parent regmap\n");

	rc = fg_parse_dt(fg);
	if (rc)
		return rc;

	rc = regmap_read(fg->regmap, fg->soc_base + REG_PERPH_TYPE, &type);
	if (rc)
		return dev_err_probe(&pdev->dev, rc, "SOC type read failed\n");
	rc = regmap_read(fg->regmap, fg->soc_base + REG_PERPH_SUBTYPE, &subtype);
	if (rc)
		return rc;
	if (subtype != FG_SUBTYPE_SOC)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "not FG SOC (type %02x subtype %02x)\n",
				     type, subtype);

	rc = regmap_read(fg->regmap, fg->batt_base + REG_PERPH_SUBTYPE,
			 &subtype);
	if (rc)
		return rc;
	if (subtype != FG_SUBTYPE_BATT)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "not FG BATT (subtype %02x)\n", subtype);

	rc = regmap_read(fg->regmap, fg->mem_base + REG_PERPH_TYPE, &type);
	if (!rc)
		rc = regmap_read(fg->regmap, fg->mem_base + REG_PERPH_SUBTYPE,
				 &subtype);
	if (rc)
		return rc;
	if (subtype != FG_SUBTYPE_MEMIF)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "not FG MEMIF (type %02x subtype %02x)\n",
				     type, subtype);

	rc = fg_setup_profile(fg);
	if (rc)
		return dev_err_probe(&pdev->dev, rc, "profile setup failed\n");

	psy_cfg.drv_data = fg;
	psy_cfg.fwnode = dev_fwnode(&pdev->dev);
	fg->psy = devm_power_supply_register(&pdev->dev, &fg_desc, &psy_cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(fg->psy),
				     "failed to register battery\n");

	if (!power_supply_get_battery_info(fg->psy, &info)) {
		if (info->voltage_max_design_uv > 0)
			fg->max_voltage_uv = info->voltage_max_design_uv;
		if (info->charge_full_design_uah > 0)
			fg->nom_cap_uah = info->charge_full_design_uah;
		power_supply_put_battery_info(fg->psy, info);
	}

	platform_set_drvdata(pdev, fg);
	return 0;
}

static const struct of_device_id fg_of_match[] = {
	{ .compatible = "qcom,pmi8994-fg" },
	{ }
};
MODULE_DEVICE_TABLE(of, fg_of_match);

static struct platform_driver fg_driver = {
	.driver = {
		.name = "qcom-pmi8994-fg",
		.of_match_table = fg_of_match,
	},
	.probe = fg_probe,
};
module_platform_driver(fg_driver);

MODULE_DESCRIPTION("Qualcomm PMI8994 fuel gauge");
MODULE_LICENSE("GPL");
