// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lattice UC120 USB Type-C CC PHY (iCE5LP2K)
 *
 * SPI register protocol implementation based on the WOA-Project Ice5Lp2k clean-room
 * ice5lp_2k.sys replacement.
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/usb/typec.h>
#include <linux/workqueue.h>

#define UC120_REG_2			2
#define UC120_REG_3			3
#define UC120_REG_4			4
#define UC120_REG_5			5
#define UC120_REG_7			7
#define UC120_REG_13			13
#define UC120_REG_CAL_BASE		18

#define UC120_REG2_STATE		GENMASK(5, 2)
#define UC120_REG2_CUR_CHG		GENMASK(7, 6)

#define UC120_REG7_CURRENT		GENMASK(5, 4)
#define UC120_REG7_POLARITY		BIT(6)

#define UC120_STATE_DETACH_A		1
#define UC120_STATE_DFP_VBUS		2
#define UC120_STATE_DFP_CABLE		3
#define UC120_STATE_DETACH_B		4
#define UC120_STATE_UFP			5
#define UC120_STATE_AUDIO		6
#define UC120_STATE_DEBUG		7
#define UC120_STATE_PWR_ACC		8

#define UC120_CUR_USB			1
#define UC120_CUR_1_5A			2
#define UC120_CUR_3_0A			3

#define UC120_REG4_IRQ_EN		BIT(0)
#define UC120_REG4_UNK1			BIT(1)
#define UC120_REG4_PD_EN		BIT(7)
#define UC120_REG4_D0			0x06
#define UC120_REG5_D0			0x88
#define UC120_REG5_PWR_SRC		BIT(0)
#define UC120_REG5_VCONN		BIT(5)
#define UC120_REG5_VBUS			BIT(6)
#define UC120_REG13_D0_MASK		0x03
#define UC120_REG13_D0			0x02

#define UC120_FW_NAME			"lattice/ice5lp_2k_cal.bin"
#define UC120_SPI_RETRIES		3
#define UC120_CC_DEBOUNCE_MS		150

struct uc120 {
	struct spi_device *spi;
	struct mutex lock;
	struct typec_capability cap;
	struct typec_port *port;
	struct typec_partner *partner;
	struct gpio_desc *orient_gpio;
	struct power_supply *usb_psy;
	const char *usb_psy_name;
	u8 last_state;
	u8 last_cur;
	u8 last_reverse;
	u8 reg4;
	u8 reg5;
	int typec_icl_ua;
	bool calibrated;
	struct notifier_block psy_nb;
	struct delayed_work vbus_off_work;
	struct delayed_work pd_en_work;
};

static int uc120_spi_read(struct uc120 *uc, u8 reg, u8 *val, size_t len)
{
	u8 cmd = reg << 3;
	u8 *rx;
	struct spi_transfer xfers[2] = { };
	struct spi_message msg;
	int ret, tries;

	rx = kzalloc(2 + len, GFP_KERNEL);
	if (!rx)
		return -ENOMEM;

	xfers[0].tx_buf = &cmd;
	xfers[0].len = 1;
	xfers[1].rx_buf = rx;
	xfers[1].len = 2 + len;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	/*
	 * Ice5Lp2k UC120SpiRead retries the SPB transfer three times on
	 * failure. Do not retry a successful read that returned 0xfc —
	 * that is an undecodable CC latch, not an SPI error.
	 */
	ret = -EIO;
	for (tries = 0; tries < UC120_SPI_RETRIES; tries++) {
		ret = spi_sync(uc->spi, &msg);
		if (!ret)
			break;
	}
	if (!ret)
		memcpy(val, rx + 2, len);

	kfree(rx);
	return ret;
}

static int uc120_spi_write(struct uc120 *uc, u8 reg, const u8 *val, size_t len)
{
	u8 *tx;
	int ret;

	tx = kmalloc(1 + len, GFP_KERNEL);
	if (!tx)
		return -ENOMEM;

	tx[0] = (reg << 3) | 1;
	memcpy(tx + 1, val, len);
	ret = spi_write(uc->spi, tx, 1 + len);
	kfree(tx);
	return ret;
}

static int uc120_read8(struct uc120 *uc, u8 reg, u8 *val)
{
	return uc120_spi_read(uc, reg, val, 1);
}

static int uc120_write8(struct uc120 *uc, u8 reg, u8 val)
{
	return uc120_spi_write(uc, reg, &val, 1);
}

static int uc120_current_ua(u8 code)
{
	switch (code) {
	case UC120_CUR_1_5A:
		return 1500000;
	case UC120_CUR_3_0A:
		/* 3 A Rp sags VBUS on these bricks; stay at 1.5 A. */
		return 1500000;
	default:
		/* Default USB Rp: leave ICL to smbchg AICL. */
		return 0;
	}
}

static enum typec_pwr_opmode uc120_opmode(u8 code)
{
	switch (code) {
	case UC120_CUR_1_5A:
		return TYPEC_PWR_MODE_1_5A;
	case UC120_CUR_3_0A:
		return TYPEC_PWR_MODE_3_0A;
	default:
		return TYPEC_PWR_MODE_USB;
	}
}

static void uc120_remember(struct uc120 *uc, u8 state, u8 cur, bool reverse)
{
	uc->last_state = state;
	uc->last_cur = cur;
	uc->last_reverse = reverse;
}

static void uc120_set_charger_icl(struct uc120 *uc, int ua)
{
	union power_supply_propval val = { .intval = ua };
	struct power_supply *psy = uc->usb_psy;

	if (!psy && uc->usb_psy_name) {
		psy = power_supply_get_by_name(uc->usb_psy_name);
		if (psy)
			uc->usb_psy = psy;
	}
	if (!psy)
		return;

	uc->typec_icl_ua = ua;
	power_supply_set_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
}

/* Ice5Lp2k UC120PDMessagingEnable: Register4 bit 7 = GoodCRC. */
static int uc120_pd_enable(struct uc120 *uc, bool enable)
{
	u8 val = enable ? (uc->reg4 | UC120_REG4_PD_EN) :
			  (uc->reg4 & ~UC120_REG4_PD_EN);
	int ret;

	if (val == uc->reg4)
		return 0;
	ret = uc120_write8(uc, UC120_REG_4, val);
	if (!ret)
		uc->reg4 = val;
	return ret;
}

static void uc120_pd_en_work(struct work_struct *work)
{
	struct uc120 *uc = container_of(work, struct uc120, pd_en_work.work);

	mutex_lock(&uc->lock);
	if (uc->last_state == UC120_STATE_UFP ||
	    uc->last_state == UC120_STATE_PWR_ACC)
		uc120_pd_enable(uc, true);
	mutex_unlock(&uc->lock);
}

static void uc120_set_orientation(struct uc120 *uc, bool reverse)
{
	if (uc->orient_gpio)
		gpiod_set_value_cansleep(uc->orient_gpio, reverse);
	if (uc->port)
		typec_set_orientation(uc->port,
				      reverse ? TYPEC_ORIENTATION_REVERSE :
						TYPEC_ORIENTATION_NORMAL);
}

/*
 * WOA UC120ToggleReg4YetUnknown: attach clears register 4 bit 1,
 * detach sets it. D0 sets bits 1 and 2 (0x06) before the first attach.
 */
static int uc120_reg4_unk1(struct uc120 *uc, bool set)
{
	u8 val = set ? (uc->reg4 | UC120_REG4_UNK1) :
		       (uc->reg4 & ~UC120_REG4_UNK1);
	int ret;

	if (val == uc->reg4)
		return 0;
	ret = uc120_write8(uc, UC120_REG_4, val);
	if (!ret)
		uc->reg4 = val;
	return ret;
}

/*
 * Ice5Lp2k SetVConn / SetPowerRole read-modify-write register 5.
 * A live read returns 0 on this silicon, so keep the software cache
 * (same as Pmic1 / Pmic2).
 */
static int uc120_set_vconn(struct uc120 *uc, bool enable)
{
	u8 val = enable ? (uc->reg5 | UC120_REG5_VCONN) :
			  (uc->reg5 & ~UC120_REG5_VCONN);
	int ret;

	if (val == uc->reg5)
		return 0;
	ret = uc120_write8(uc, UC120_REG_5, val);
	if (!ret)
		uc->reg5 = val;
	return ret;
}

static int uc120_set_pwr_src(struct uc120 *uc, bool source)
{
	u8 val = source ? (uc->reg5 | UC120_REG5_PWR_SRC) :
			  (uc->reg5 & ~UC120_REG5_PWR_SRC);
	int ret;

	if (val == uc->reg5)
		return 0;
	ret = uc120_write8(uc, UC120_REG_5, val);
	if (!ret)
		uc->reg5 = val;
	return ret;
}

static void uc120_unregister_partner(struct uc120 *uc)
{
	if (!uc->partner)
		return;
	typec_unregister_partner(uc->partner);
	uc->partner = NULL;
}

static void uc120_register_partner(struct uc120 *uc,
				   enum typec_accessory accessory)
{
	struct typec_partner_desc desc = { };

	uc120_unregister_partner(uc);
	desc.accessory = accessory;
	uc->partner = typec_register_partner(uc->port, &desc);
	if (IS_ERR(uc->partner)) {
		dev_err(&uc->spi->dev, "partner register failed: %ld\n",
			PTR_ERR(uc->partner));
		uc->partner = NULL;
	}
}

static void uc120_apply_state(struct uc120 *uc, u8 state, u8 cur, bool reverse)
{
	enum typec_accessory acc = TYPEC_ACCESSORY_NONE;
	bool sink = false;
	bool dfp = false;

	dev_dbg(&uc->spi->dev, "CC state %u current %u reverse %u\n",
		state, cur, reverse);

	switch (state) {
	case UC120_STATE_DETACH_A:
	case UC120_STATE_DETACH_B:
		uc120_unregister_partner(uc);
		if (uc->port) {
			typec_set_pwr_role(uc->port, TYPEC_SINK);
			typec_set_data_role(uc->port, TYPEC_DEVICE);
			typec_set_pwr_opmode(uc->port, TYPEC_PWR_MODE_USB);
			typec_set_orientation(uc->port, TYPEC_ORIENTATION_NONE);
		}
		cancel_delayed_work(&uc->pd_en_work);
		uc120_pd_enable(uc, false);
		uc120_set_charger_icl(uc, 0);
		uc120_reg4_unk1(uc, true);
		uc120_set_vconn(uc, false);
		uc120_set_pwr_src(uc, false);
		return;
	case UC120_STATE_UFP:
	case UC120_STATE_PWR_ACC:
		sink = true;
		break;
	case UC120_STATE_AUDIO:
		acc = TYPEC_ACCESSORY_AUDIO;
		break;
	case UC120_STATE_DEBUG:
		acc = TYPEC_ACCESSORY_DEBUG;
		break;
	case UC120_STATE_DFP_VBUS:
	case UC120_STATE_DFP_CABLE:
		dfp = true;
		break;
	default:
		return;
	}

	if (dfp || state == UC120_STATE_PWR_ACC) {
		uc120_set_vconn(uc, true);
		uc120_set_pwr_src(uc, true);
	}

	uc120_set_orientation(uc, reverse);

	if (uc->last_state != state)
		uc120_register_partner(uc, acc);

	if (uc->port) {
		typec_set_pwr_role(uc->port, sink ? TYPEC_SINK : TYPEC_SOURCE);
		typec_set_data_role(uc->port, sink ? TYPEC_DEVICE : TYPEC_HOST);
		typec_set_pwr_opmode(uc->port, uc120_opmode(cur));
	}

	uc120_reg4_unk1(uc, false);
	if (sink) {
		int ua = uc120_current_ua(cur);

		if (ua)
			uc120_set_charger_icl(uc, ua);
		/* GoodCRC after tCCDebounce; enabling in apply_state drops UFP. */
		if (uc->last_state != state)
			mod_delayed_work(system_dfl_wq, &uc->pd_en_work,
					 msecs_to_jiffies(UC120_CC_DEBOUNCE_MS));
		return;
	}
	uc120_set_charger_icl(uc, 0);
}

/* Ignore leftover 0xfc (state 15). Vote only a real 1.5/3 A Rp on UFP. */
static void uc120_apply_current_change(struct uc120 *uc, u8 state, u8 cur_chg)
{
	int ua;

	if (cur_chg < UC120_CUR_USB || cur_chg > UC120_CUR_3_0A)
		return;
	if (state && (state < UC120_STATE_DETACH_A ||
		      state > UC120_STATE_PWR_ACC))
		return;
	if (uc->last_state != UC120_STATE_UFP &&
	    uc->last_state != UC120_STATE_PWR_ACC)
		return;
	ua = uc120_current_ua(cur_chg);
	if (ua == uc->typec_icl_ua)
		return;
	if (uc->port)
		typec_set_pwr_opmode(uc->port, uc120_opmode(cur_chg));
	uc120_set_charger_icl(uc, ua);
	uc120_remember(uc, uc->last_state, cur_chg, uc->last_reverse);
}

static int uc120_sync(struct uc120 *uc, bool ack)
{
	u8 reg2, reg7, ackval = 0xff;
	u8 state, cur, cur_chg;
	int ret;

	ret = uc120_read8(uc, UC120_REG_2, &reg2);
	if (ret)
		return ret;

	state = FIELD_GET(UC120_REG2_STATE, reg2);
	cur_chg = FIELD_GET(UC120_REG2_CUR_CHG, reg2);
	reg7 = 0;
	cur = 0;

	if (state || cur_chg) {
		ret = uc120_read8(uc, UC120_REG_7, &reg7);
		if (ret)
			return ret;
		cur = FIELD_GET(UC120_REG7_CURRENT, reg7);
		if (state >= UC120_STATE_DETACH_A &&
		    state <= UC120_STATE_PWR_ACC) {
			uc120_apply_state(uc, state, cur,
					  !!(reg7 & UC120_REG7_POLARITY));
			uc120_remember(uc, state, cur,
				       !!(reg7 & UC120_REG7_POLARITY));
		} else if (state) {
			dev_dbg(&uc->spi->dev,
				"ignored CC state %u reg2=0x%02x\n",
				state, reg2);
		}
		uc120_apply_current_change(uc, state, cur_chg);
	}

	/* Ice5Lp2k ISR always ACKs register 2 with 0xFF after the read. */
	if (ack)
		return uc120_write8(uc, UC120_REG_2, ackval);
	return 0;
}

/* Re-read while INT stays low so a UFP nibble after leftover 0xfc is kept. */
#define UC120_IRQ_DRAIN 4

static bool uc120_irq_still_low(int irq)
{
	bool high;
	int ret;

	ret = irq_get_irqchip_state(irq, IRQCHIP_STATE_LINE_LEVEL, &high);
	if (ret)
		return false;
	return !high;
}

static irqreturn_t uc120_irq_thread(int irq, void *data)
{
	struct uc120 *uc = data;
	int i;

	mutex_lock(&uc->lock);
	for (i = 0; i < UC120_IRQ_DRAIN; i++) {
		if (uc120_sync(uc, true)) {
			dev_err_ratelimited(&uc->spi->dev, "IRQ SPI failed\n");
			break;
		}
		if (!uc120_irq_still_low(irq))
			break;
	}
	mutex_unlock(&uc->lock);
	return IRQ_HANDLED;
}

static void uc120_pmic2_detach(struct uc120 *uc)
{
	uc->reg5 &= ~UC120_REG5_VBUS;
	uc120_write8(uc, UC120_REG_5, uc->reg5);
	cancel_delayed_work(&uc->pd_en_work);
	uc120_pd_enable(uc, false);
	uc120_remember(uc, UC120_STATE_DETACH_A, 0, false);
	uc120_unregister_partner(uc);
	if (uc->port) {
		typec_set_pwr_role(uc->port, TYPEC_SINK);
		typec_set_data_role(uc->port, TYPEC_DEVICE);
		typec_set_pwr_opmode(uc->port, TYPEC_PWR_MODE_USB);
		typec_set_orientation(uc->port, TYPEC_ORIENTATION_NONE);
	}
	uc120_set_charger_icl(uc, 0);
	uc120_reg4_unk1(uc, true);
}

/* Pmic1/2 follow charger PRESENT (src-det), not INPUT_STS. */
static void uc120_vbus_off_work(struct work_struct *work)
{
	struct uc120 *uc = container_of(work, struct uc120, vbus_off_work.work);
	union power_supply_propval val = { };
	struct power_supply *psy;

	mutex_lock(&uc->lock);
	psy = uc->usb_psy;
	if (psy && !power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,
					      &val) &&
	    val.intval)
		goto out;
	if (uc->reg5 & UC120_REG5_VBUS)
		uc120_pmic2_detach(uc);
out:
	mutex_unlock(&uc->lock);
}
static void uc120_vbus_from_psy(struct uc120 *uc)
{
	union power_supply_propval val = { };
	struct power_supply *psy = uc->usb_psy;
	unsigned int delay;

	if (!psy && uc->usb_psy_name) {
		psy = power_supply_get_by_name(uc->usb_psy_name);
		if (psy)
			uc->usb_psy = psy;
	}
	if (!psy)
		return;
	if (power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT, &val))
		return;

	/* Register 5 reads as 0; write the cached value (Pmic1 bit 6). */
	if (val.intval) {
		cancel_delayed_work(&uc->vbus_off_work);
		if (uc->reg5 & UC120_REG5_VBUS)
			return;
		uc->reg5 |= UC120_REG5_VBUS;
		uc120_write8(uc, UC120_REG_5, uc->reg5);
		return;
	}

	if (!(uc->reg5 & UC120_REG5_VBUS))
		return;
	/* Brief PRESENT=0 (APSD) must not drop FPGA Rd. */
	delay = (uc->last_state == UC120_STATE_UFP ||
		 uc->last_state == UC120_STATE_PWR_ACC ||
		 uc->typec_icl_ua) ? 500 : UC120_CC_DEBOUNCE_MS;
	mod_delayed_work(system_dfl_wq, &uc->vbus_off_work,
			 msecs_to_jiffies(delay));
}

static int uc120_psy_notifier(struct notifier_block *nb, unsigned long event,
			      void *data)
{
	struct uc120 *uc = container_of(nb, struct uc120, psy_nb);
	struct power_supply *psy = data;

	if (event != PSY_EVENT_PROP_CHANGED || !psy || !psy->desc ||
	    !psy->desc->name || !uc->usb_psy_name)
		return NOTIFY_DONE;
	if (strcmp(psy->desc->name, uc->usb_psy_name))
		return NOTIFY_DONE;

	mutex_lock(&uc->lock);
	uc120_vbus_from_psy(uc);
	mutex_unlock(&uc->lock);
	return NOTIFY_OK;
}

static void uc120_unreg_psy_notifier(void *data)
{
	power_supply_unreg_notifier(data);
}

static int uc120_calibrate(struct uc120 *uc)
{
	const struct firmware *fw;
	const u8 *p;
	size_t n;
	int ret = 0, i;
	static const u8 map11[] = { 18, 19, 20, 21, 26, 22, 23, 24, 25, 27 };

	ret = request_firmware_direct(&fw, UC120_FW_NAME, &uc->spi->dev);
	if (ret) {
		dev_info(&uc->spi->dev,
			 "no %s (%d), continuing uncalibrated\n",
			 UC120_FW_NAME, ret);
		return 0;
	}

	if (fw->size == 8) {
		p = fw->data;
		for (i = 0; i < 8 && !ret; i++)
			ret = uc120_write8(uc, UC120_REG_CAL_BASE + i, p[i]);
	} else if (fw->size > 8 && fw->data[0] == 0x02) {
		n = min_t(size_t, fw->size - 1, ARRAY_SIZE(map11));
		p = fw->data + 1;
		for (i = 0; i < n && !ret; i++)
			ret = uc120_write8(uc, map11[i], p[i]);
	} else {
		dev_err(&uc->spi->dev, "unrecognized cal size %zu\n", fw->size);
		release_firmware(fw);
		return 0;
	}

	release_firmware(fw);
	if (ret)
		return ret;

	uc->calibrated = true;
	dev_info(&uc->spi->dev, "applied %s\n", UC120_FW_NAME);
	return 0;
}

static int uc120_hw_init(struct uc120 *uc)
{
	int ret;

	/* Ice5Lp2k D0: cached Register4 |= 6, Register5 = 0x88, Register13 = (x & 0xFC) | 2. */
	uc->reg4 |= UC120_REG4_D0;
	ret = uc120_write8(uc, UC120_REG_4, uc->reg4);
	if (ret)
		return ret;
	uc->reg5 = UC120_REG5_D0;
	ret = uc120_write8(uc, UC120_REG_5, uc->reg5);
	if (ret)
		return ret;
	ret = uc120_write8(uc, UC120_REG_13, UC120_REG13_D0);
	if (ret)
		return ret;

	ret = uc120_calibrate(uc);
	if (ret)
		return ret;

	return 0;
}

/*
 * WOA UC120InterruptEnable: runs after the IRQ is already hooked
 * (WdfInterruptCreate in AddDevice, enable after D0). Unmasking the
 * FPGA before request_irq drops the falling edge on GPIO 95.
 */
static int uc120_irq_enable(struct uc120 *uc)
{
	int ret;

	ret = uc120_write8(uc, UC120_REG_2, 0xff);
	if (ret)
		return ret;
	ret = uc120_write8(uc, UC120_REG_3, 0xff);
	if (ret)
		return ret;

	uc->reg4 |= UC120_REG4_IRQ_EN;
	ret = uc120_write8(uc, UC120_REG_4, uc->reg4);
	if (ret)
		return ret;
	uc->reg5 &= 0x7f;
	ret = uc120_write8(uc, UC120_REG_5, uc->reg5);
	if (ret)
		return ret;

	return uc120_sync(uc, true);
}

static int uc120_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct fwnode_handle *connector;
	struct uc120 *uc;
	int ret;

	if (!spi->irq)
		return dev_err_probe(dev, -EINVAL, "UC120 IRQ missing\n");

	uc = devm_kzalloc(dev, sizeof(*uc), GFP_KERNEL);
	if (!uc)
		return -ENOMEM;

	uc->spi = spi;
	mutex_init(&uc->lock);
	INIT_DELAYED_WORK(&uc->vbus_off_work, uc120_vbus_off_work);
	INIT_DELAYED_WORK(&uc->pd_en_work, uc120_pd_en_work);
	spi_set_drvdata(spi, uc);

	device_property_read_string(dev, "usb-psy-name", &uc->usb_psy_name);
	uc->orient_gpio = devm_gpiod_get_optional(dev, "orientation",
						  GPIOD_OUT_LOW);
	if (IS_ERR(uc->orient_gpio))
		return PTR_ERR(uc->orient_gpio);

	connector = device_get_named_child_node(dev, "connector");
	if (!connector)
		return -ENODEV;

	ret = typec_get_fw_cap(&uc->cap, connector);
	if (ret)
		goto err_put;

	uc->cap.revision = USB_TYPEC_REV_1_2;
	uc->cap.pd_revision = 0;
	uc->cap.accessory[0] = TYPEC_ACCESSORY_AUDIO;
	uc->cap.accessory[1] = TYPEC_ACCESSORY_DEBUG;
	uc->cap.orientation_aware = true;
	uc->cap.driver_data = uc;

	uc->port = typec_register_port(dev, &uc->cap);
	if (IS_ERR(uc->port)) {
		ret = PTR_ERR(uc->port);
		goto err_put;
	}

	mutex_lock(&uc->lock);
	ret = uc120_hw_init(uc);
	mutex_unlock(&uc->lock);
	if (ret) {
		dev_err(dev, "UC120 init failed: %d\n", ret);
		goto err_port;
	}

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, uc120_irq_thread,
					IRQF_ONESHOT, "uc120", uc);
	if (ret)
		goto err_port;

	mutex_lock(&uc->lock);
	ret = uc120_irq_enable(uc);
	mutex_unlock(&uc->lock);
	if (ret) {
		dev_err(dev, "UC120 IRQ enable failed: %d\n", ret);
		goto err_port;
	}

	uc->psy_nb.notifier_call = uc120_psy_notifier;
	ret = power_supply_reg_notifier(&uc->psy_nb);
	if (ret)
		goto err_port;
	ret = devm_add_action_or_reset(dev, uc120_unreg_psy_notifier,
				       &uc->psy_nb);
	if (ret)
		goto err_port;

	mutex_lock(&uc->lock);
	uc120_vbus_from_psy(uc);
	mutex_unlock(&uc->lock);

	fwnode_handle_put(connector);
	return 0;

err_port:
	cancel_delayed_work_sync(&uc->pd_en_work);
	cancel_delayed_work_sync(&uc->vbus_off_work);
	typec_unregister_port(uc->port);
err_put:
	fwnode_handle_put(connector);
	return ret;
}

static void uc120_remove(struct spi_device *spi)
{
	struct uc120 *uc = spi_get_drvdata(spi);

	cancel_delayed_work_sync(&uc->pd_en_work);
	cancel_delayed_work_sync(&uc->vbus_off_work);
	mutex_lock(&uc->lock);
	uc->reg4 &= ~UC120_REG4_IRQ_EN;
	uc120_write8(uc, UC120_REG_4, uc->reg4);
	uc120_set_charger_icl(uc, 0);
	uc120_unregister_partner(uc);
	mutex_unlock(&uc->lock);
	typec_unregister_port(uc->port);
	if (uc->usb_psy)
		power_supply_put(uc->usb_psy);
}

static const struct of_device_id uc120_of_match[] = {
	{ .compatible = "lattice,uc120" },
	{ }
};
MODULE_DEVICE_TABLE(of, uc120_of_match);

static struct spi_driver uc120_driver = {
	.probe = uc120_probe,
	.remove = uc120_remove,
	.driver = {
		.name = "uc120",
		.of_match_table = uc120_of_match,
	},
};
module_spi_driver(uc120_driver);

MODULE_DESCRIPTION("Lattice UC120 USB Type-C CC PHY");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(UC120_FW_NAME);
