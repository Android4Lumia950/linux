/* SPDX-License-Identifier: GPL-2.0 */
/*
 * WCD9330 (tomtom) registers. Addresses and POR values are from
 * 3.10 include/linux/mfd/wcd9xxx/wcd9330_registers.h and
 * wcd9xxx_registers.h. Slim element offset is
 * TOMTOM_REGISTER_START_OFFSET (wcd9330.h / wcd9xxx-core.c).
 */
#ifndef __WCD9330_H__
#define __WCD9330_H__

#define TOMTOM_REGISTER_START_OFFSET		0x800
#define TOMTOM_NUM_REGISTERS			0x400
#define TOMTOM_MAX_REGISTER			(TOMTOM_NUM_REGISTERS - 1)

#define TOMTOM_MAJOR				0x0105

#define TOMTOM_A_CHIP_CTL			0x000
#define TOMTOM_A_CHIP_ID_BYTE_0			0x004
#define TOMTOM_A_CHIP_ID_BYTE_1			0x005
#define TOMTOM_A_CHIP_ID_BYTE_2			0x006
#define TOMTOM_A_CHIP_ID_BYTE_3			0x007
#define TOMTOM_A_CDC_CTL				0x034
#define TOMTOM_A_LEAKAGE_CTL			0x03c

#define TOMTOM_A_INTR_MODE			0x090
#define TOMTOM_A_INTR2_MASK0			0x0b0
#define TOMTOM_A_INTR2_STATUS0			0x0b2
#define TOMTOM_A_INTR2_CLEAR0			0x0b4
#define TOMTOM_A_CDC_MAD_MAIN_CTL_1		0x0e0
#define TOMTOM_A_CDC_MAD_AUDIO_CTL_3		0x0e4
#define TOMTOM_A_CDC_MAD_AUDIO_CTL_4		0x0e5
#define TOMTOM_A_CDC_ANC1_IIR_B1_CTL		0x202
#define TOMTOM_A_CDC_ANC1_GAIN_CTL		0x20c
#define TOMTOM_SB_PGD_PORT_RX_BASE		0x40
#define TOMTOM_SB_PGD_PORT_TX_BASE		0x50
#define TOMTOM_A_INTR_MASK0			0x094
#define TOMTOM_A_INTR_MASK1			0x095
#define TOMTOM_A_INTR_MASK2			0x096
#define TOMTOM_A_INTR_MASK3			0x097
#define TOMTOM_A_INTR_STATUS0			0x098
#define TOMTOM_A_INTR_STATUS2			0x09a
#define TOMTOM_A_INTR_CLEAR0			0x09c
#define TOMTOM_A_INTR_CLEAR2			0x09e

#define TOMTOM_A_BIAS_CENTRAL_BG_CTL		0x101
#define TOMTOM_A_BIAS_CURR_CTL_2			0x104
#define TOMTOM_A_CLK_BUFF_EN1			0x108
#define TOMTOM_A_CLK_BUFF_EN2			0x109

#define TOMTOM_A_MICB_2_CTL			0x131

#define TOMTOM_A_MBHC_INSERT_DETECT		0x14a
#define TOMTOM_A_MBHC_INSERT_DET_STATUS		0x14b
#define TOMTOM_A_MBHC_HPH			0x1fe

#define TOMTOM_A_BUCK_MODE_1			0x181
#define TOMTOM_A_BUCK_MODE_3			0x183
#define TOMTOM_A_BUCK_MODE_4			0x184
#define TOMTOM_A_BUCK_MODE_5			0x185

#define TOMTOM_A_NCP_EN				0x192
#define TOMTOM_A_NCP_CLK				0x193
#define TOMTOM_A_NCP_STATIC			0x194

#define TOMTOM_A_RX_COM_OCP_COUNT		0x1a0
#define TOMTOM_A_RX_COM_BIAS			0x1a2
#define TOMTOM_A_RX_HPH_CHOP_CTL		0x1a5
#define TOMTOM_A_RX_HPH_BIAS_PA			0x1a6
#define TOMTOM_A_RX_HPH_BIAS_PA__POR		0x7a
#define TOMTOM_A_RX_HPH_BIAS_WG_OCP		0x1a9
#define TOMTOM_A_RX_HPH_OCP_CTL			0x1aa
#define TOMTOM_A_RX_HPH_CNP_EN			0x1ab
#define TOMTOM_A_RX_HPH_CNP_WG_CTL		0x1ac
#define TOMTOM_A_RX_HPH_CNP_WG_TIME		0x1ad
#define TOMTOM_A_RX_HPH_L_GAIN			0x1ae
#define TOMTOM_A_RX_HPH_L_TEST			0x1af
#define TOMTOM_A_RX_HPH_L_PA_CTL		0x1b0
#define TOMTOM_A_RX_HPH_L_PA_CTL__POR		0x42
#define TOMTOM_A_RX_HPH_L_DAC_CTL		0x1b1
#define TOMTOM_A_RX_HPH_R_GAIN			0x1b4
#define TOMTOM_A_RX_HPH_R_TEST			0x1b5
#define TOMTOM_A_RX_HPH_R_PA_CTL		0x1b6
#define TOMTOM_A_RX_HPH_R_PA_CTL__POR		0x42
#define TOMTOM_A_RX_HPH_R_DAC_CTL		0x1b7

#define TOMTOM_A_CDC_RX1_B3_CTL			0x2b2
#define TOMTOM_A_CDC_RX1_B4_CTL			0x2b3
#define TOMTOM_A_CDC_RX1_B5_CTL			0x2b4
#define TOMTOM_A_CDC_RX1_B6_CTL			0x2b5
#define TOMTOM_A_CDC_RX1_VOL_CTL_B2_CTL		0x2b7
#define TOMTOM_A_CDC_RX2_B3_CTL			0x2ba
#define TOMTOM_A_CDC_RX2_B4_CTL			0x2bb
#define TOMTOM_A_CDC_RX2_B5_CTL			0x2bc
#define TOMTOM_A_CDC_RX2_B6_CTL			0x2bd
#define TOMTOM_A_CDC_RX2_VOL_CTL_B2_CTL		0x2bf

#define TOMTOM_A_CDC_CLK_RX_RESET_CTL		0x301
#define TOMTOM_A_CDC_CLK_OTHR_CTL		0x30c
#define TOMTOM_A_CDC_CLK_RX_B1_CTL		0x30f
#define TOMTOM_A_CDC_CLK_MCLK_CTL		0x311

#define TOMTOM_A_CDC_CLSH_B1_CTL			0x320
#define TOMTOM_A_CDC_CLSH_B2_CTL			0x321
#define TOMTOM_A_CDC_CLSH_B3_CTL			0x322
#define TOMTOM_A_CDC_CLSH_BUCK_NCP_VARS		0x323
#define TOMTOM_A_CDC_CLSH_IDLE_HPH_THSD		0x324
#define TOMTOM_A_CDC_CLSH_FCLKONLY_HPH_THSD	0x326
#define TOMTOM_A_CDC_CLSH_K_ADDR			0x328
#define TOMTOM_A_CDC_CLSH_K_DATA			0x329
#define TOMTOM_A_CDC_CLSH_I_PA_FACT_HPH_L	0x32a
#define TOMTOM_A_CDC_CLSH_I_PA_FACT_HPH_U	0x32b
#define TOMTOM_A_CDC_CLSH_V_PA_HD_HPH		0x32f
#define TOMTOM_A_CDC_CLSH_V_PA_MIN_HPH		0x331

#define TOMTOM_A_CDC_CONN_RX1_B1_CTL		0x380
#define TOMTOM_A_CDC_CONN_RX1_B2_CTL		0x381
#define TOMTOM_A_CDC_CONN_RX2_B1_CTL		0x383
#define TOMTOM_A_CDC_CONN_RX_SB_B1_CTL		0x3ae
#define TOMTOM_A_CDC_CONN_RX_SB_B2_CTL		0x3af
#define TOMTOM_A_CDC_CONN_CLSH_CTL		0x3b0

/* 3.10 wcd9330.c: slave port = ch_num - 128; RX ports 16..23 */
#define TOMTOM_VALIDATE_RX_SBPORT_RANGE(p)	((p) >= 16 && (p) <= 23)
#define TOMTOM_CONVERT_RX_SBPORT_ID(p)		((p) - 16)

/*
 * 3.10 wcd9xxx-slimslave.c TAIKO slave (tomtom uses that type):
 * rx_port_ch_reg_base = 0x140, port_rx_cfg_reg_base = 0x030.
 * WATER_MARK_VAL is 12-byte watermark | enable.
 */
#define TOMTOM_SB_PGD_RX_PORT_CFG(p)		(0x030 + (p))
#define TOMTOM_SB_PGD_RX_PORT_MULTI_CHNL_0(p)	(0x140 + 4 * (p))
#define TOMTOM_SB_PGD_PORT_INT_EN0		0x30
#define TOMTOM_SLAVE_PORT_WATER_MARK_VAL	0x05

#define TOMTOM_RX_PORT_START			16
#define TOMTOM_RX_MAX				13
#define TOMTOM_SLIM_CH_START			128

#define SLIM_MANF_ID_QCOM			0x217
#define SLIM_PROD_CODE_WCD9330			0x130

#define TOMTOM_MCLK_RATE			9600000
#define TOMTOM_HPH_PA_SETTLE_US			13000

#define TOMTOM_IRQ_MBHC_REMOVAL			1
#define TOMTOM_IRQ_MBHC_INSERTION		6
/*
 * 3.10 include/linux/mfd/wcd9xxx/core.h: JACK_SWITCH is irq 23
 * (INTR_REG 2 bit 7). Codec-own insert_detect uses this, not
 * INSERTION/REMOVAL. wcd9xxx_setup_jack_detect_irq requests it.
 */
#define TOMTOM_IRQ_MBHC_JACK_SWITCH		23
#define TOMTOM_SWCH_DEBOUNCE_US			5000

struct snd_soc_component;
struct device;

int wcd9330_afe_set_config(struct snd_soc_component *comp,
			   struct device *afe_dev);

#endif
