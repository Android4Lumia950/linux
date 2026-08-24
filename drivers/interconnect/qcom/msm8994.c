// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Linaro Limited.
 *
 * Based on msm8974.c
 * Copyright (C) 2019 Brian Masney <masneyb@onstation.org>
 *
 * Based on MSM bus code from downstream MSM kernel sources.
 * Copyright (c) 2014 The Linux Foundation. All rights reserved.
 *
 * Here's a rough representation that shows the various buses that form the
 * Network On Chip (NOC) for the msm8994:
 *
 *                         Multimedia Subsystem (MMSS)
 *         |----------+-----------------------------------+-----------|
 *                    |                                   |
 *                    |                                   |
 *        Config      |                     Bus Interface | Memory Controller
 *       |------------+-+-----------|        |------------+-+-----------|
 *                      |                                   |
 *                      |                                   |
 *                      |             System                |
 *     |--------------+-+---------------------------------+-+-------------|
 *                    |                                   |
 *                    |                                   |
 *        Peripheral  |                           On Chip | Memory (OCMEM)
 *       |------------+-------------|        |------------+-------------|
 */

#include <linux/args.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interconnect-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "icc-rpm.h"

/*
 * Per-provider DT indices for qnoc_probe() xlate. Must stay identical to
 * include/dt-bindings/interconnect/qcom,msm8994.h.
 *
 * Defined here on purpose: several 8994 trees still ship a copy of
 * qcom,msm8974.h under the msm8994 name. Including that header makes
 * 8994-only nodes disappear and silently remaps shared IDs.
 */
enum {
	BIMC_MAS_AMPSS_M0 = 0,
	BIMC_MAS_GRAPHICS_3D,
	BIMC_TO_MNOC,
	BIMC_TO_SNOC,
	BIMC_SLV_EBI_CH0,
	BIMC_SLV_AMPSS_L2,

	CNOC_MAS_RPM_INST = 0,
	CNOC_MAS_RPM_SYS,
	CNOC_MAS_DEHR,
	CNOC_MAS_QDSS_DAP,
	CNOC_MAS_SPDM,
	CNOC_MAS_TIC,
	CNOC_TO_SNOC,
	CNOC_SLV_CLK_CTL,
	CNOC_SLV_SECURITY,
	CNOC_SLV_TCSR,
	CNOC_SLV_TLMM,
	CNOC_SLV_CRYPTO_0_CFG,
	CNOC_SLV_CRYPTO_1_CFG,
	CNOC_SLV_CRYPTO_2_CFG,
	CNOC_SLV_IMEM_CFG,
	CNOC_SLV_MESSAGE_RAM,
	CNOC_SLV_BIMC_CFG,
	CNOC_SLV_BOOT_ROM,
	CNOC_SLV_PMIC_ARB,
	CNOC_SLV_SPDM_WRAPPER,
	CNOC_SLV_DEHR_CFG,
	CNOC_SLV_MPM,
	CNOC_SLV_QDSS_CFG,
	CNOC_SLV_RBCPR_CFG,
	CNOC_SLV_RBCPR_QDSS_APU_CFG,
	CNOC_SLV_CNOC_MNOC_MMSS_CFG,
	CNOC_SLV_CNOC_MNOC_CFG,
	CNOC_SLV_PNOC_CFG,
	CNOC_SLV_SNOC_MPU_CFG,
	CNOC_SLV_SNOC_CFG,
	CNOC_SLV_EBI1_DLL_CFG,
	CNOC_SLV_PHY_APU_CFG,
	CNOC_SLV_EBI1_PHY_CFG,
	CNOC_SLV_RPM,
	CNOC_SLV_PCIE_0_CFG,
	CNOC_SLV_PCIE_1_CFG,
	CNOC_SLV_GENI_IR_CFG,
	CNOC_SLV_UFS_CFG,

	MNOC_MAS_JPEG = 0,
	MNOC_MAS_MDP_PORT0,
	MNOC_MAS_MDP_PORT1,
	MNOC_MAS_VIDEO_P0,
	MNOC_MAS_VIDEO_P1,
	MNOC_MAS_VFE,
	MNOC_MAS_CPP,
	MNOC_MAS_VPU,
	MNOC_MAS_CNOC_MMSS_CFG,
	MNOC_MAS_CNOC_CFG,
	MNOC_TO_BIMC,
	MNOC_SLV_CAMERA_CFG,
	MNOC_SLV_DISPLAY_CFG,
	MNOC_SLV_OCMEM_CFG,
	MNOC_SLV_CPR_CFG,
	MNOC_SLV_CPR_XPU_CFG,
	MNOC_SLV_MISC_CFG,
	MNOC_SLV_MISC_XPU_CFG,
	MNOC_SLV_VENUS_CFG,
	MNOC_SLV_GRAPHICS_3D_CFG,
	MNOC_SLV_MMSS_CLK_CFG,
	MNOC_SLV_MMSS_CLK_XPU_CFG,
	MNOC_SLV_MNOC_MPU_CFG,
	MNOC_SLV_AVSYNC_CFG,
	MNOC_SLV_VPU_CFG,
	MNOC_SLV_SERVICE_MNOC,

	OCMEM_MAS_SNOC = 0,
	OCMEM_MAS_OCMEM_DMA,
	OCMEM_MAS_GFX3D,
	OCMEM_MAS_VIDEO_P0_OCMEM,
	OCMEM_SLV_OCMEM,
	OCMEM_SLV_OCMEM_GFX,
	OCMEM_TO_SNOC,

	PNOC_MAS_PNOC_CFG = 0,
	PNOC_MAS_SDCC_1,
	PNOC_MAS_SDCC_3,
	PNOC_MAS_SDCC_4,
	PNOC_MAS_SDCC_2,
	PNOC_MAS_TSIF,
	PNOC_MAS_BAM_DMA,
	PNOC_MAS_BLSP_2,
	PNOC_MAS_BLSP_1,
	PNOC_MAS_USB_HS,
	PNOC_TO_SNOC,
	PNOC_SLV_SDCC_1,
	PNOC_SLV_SDCC_3,
	PNOC_SLV_SDCC_2,
	PNOC_SLV_SDCC_4,
	PNOC_SLV_TSIF,
	PNOC_SLV_BAM_DMA,
	PNOC_SLV_BLSP_2,
	PNOC_SLV_BLSP_1,
	PNOC_SLV_USB_HS,
	PNOC_SLV_PDM,
	PNOC_SLV_PRNG,

	SNOC_MAS_LPASS_AHB = 0,
	SNOC_MAS_QDSS_BAM,
	SNOC_TO_BIMC,
	SNOC_TO_CNOC,
	SNOC_TO_PNOC,
	SNOC_TO_OCMEM_VNOC,
	SNOC_MAS_CRYPTO_CORE0,
	SNOC_MAS_CRYPTO_CORE1,
	SNOC_MAS_CRYPTO_CORE2,
	SNOC_MAS_LPASS_PROC,
	SNOC_MAS_QDSS_ETR,
	SNOC_MAS_USB3,
	SNOC_MAS_PCIE,
	SNOC_MAS_PCIE_1,
	SNOC_MAS_UFS,
	SNOC_MAS_IPA,
	SNOC_MAS_OVNOC,
	SNOC_SLV_AMPSS,
	SNOC_SLV_LPASS,
	SNOC_SLV_USB3,
	SNOC_SLV_OCIMEM,
	SNOC_SLV_QDSS_STM,
	SNOC_SLV_PCIE_0,
	SNOC_SLV_PCIE_1,
};

enum {
	MSM8994_BIMC_MAS_AMPSS_M0 = 1,
	MSM8994_BIMC_MAS_GRAPHICS_3D,
	MSM8994_BIMC_TO_MNOC,
	MSM8994_BIMC_TO_SNOC,
	MSM8994_BIMC_SLV_EBI_CH0,
	MSM8994_BIMC_SLV_AMPSS_L2,
	MSM8994_CNOC_MAS_RPM_INST,
	MSM8994_CNOC_MAS_RPM_SYS,
	MSM8994_CNOC_MAS_DEHR,
	MSM8994_CNOC_MAS_QDSS_DAP,
	MSM8994_CNOC_MAS_SPDM,
	MSM8994_CNOC_MAS_TIC,
	MSM8994_CNOC_TO_SNOC,
	MSM8994_CNOC_SLV_CLK_CTL,
	MSM8994_CNOC_SLV_SECURITY,
	MSM8994_CNOC_SLV_TCSR,
	MSM8994_CNOC_SLV_TLMM,
	MSM8994_CNOC_SLV_CRYPTO_0_CFG,
	MSM8994_CNOC_SLV_CRYPTO_1_CFG,
	MSM8994_CNOC_SLV_CRYPTO_2_CFG,
	MSM8994_CNOC_SLV_IMEM_CFG,
	MSM8994_CNOC_SLV_MESSAGE_RAM,
	MSM8994_CNOC_SLV_BIMC_CFG,
	MSM8994_CNOC_SLV_BOOT_ROM,
	MSM8994_CNOC_SLV_PMIC_ARB,
	MSM8994_CNOC_SLV_SPDM_WRAPPER,
	MSM8994_CNOC_SLV_DEHR_CFG,
	MSM8994_CNOC_SLV_MPM,
	MSM8994_CNOC_SLV_QDSS_CFG,
	MSM8994_CNOC_SLV_RBCPR_CFG,
	MSM8994_CNOC_SLV_RBCPR_QDSS_APU_CFG,
	MSM8994_CNOC_SLV_CNOC_MNOC_MMSS_CFG,
	MSM8994_CNOC_SLV_CNOC_MNOC_CFG,
	MSM8994_CNOC_SLV_PNOC_CFG,
	MSM8994_CNOC_SLV_SNOC_MPU_CFG,
	MSM8994_CNOC_SLV_SNOC_CFG,
	MSM8994_CNOC_SLV_EBI1_DLL_CFG,
	MSM8994_CNOC_SLV_PHY_APU_CFG,
	MSM8994_CNOC_SLV_EBI1_PHY_CFG,
	MSM8994_CNOC_SLV_RPM,
	MSM8994_CNOC_SLV_PCIE_0_CFG,
	MSM8994_CNOC_SLV_PCIE_1_CFG,
	MSM8994_CNOC_SLV_GENI_IR_CFG,
	MSM8994_CNOC_SLV_UFS_CFG,
	MSM8994_MNOC_MAS_JPEG,
	MSM8994_MNOC_MAS_MDP_PORT0,
	MSM8994_MNOC_MAS_MDP_PORT1,
	MSM8994_MNOC_MAS_VIDEO_P0,
	MSM8994_MNOC_MAS_VIDEO_P1,
	MSM8994_MNOC_MAS_VFE,
	MSM8994_MNOC_MAS_CPP,
	MSM8994_MNOC_MAS_VPU,
	MSM8994_MNOC_MAS_CNOC_MMSS_CFG,
	MSM8994_MNOC_MAS_CNOC_CFG,
	MSM8994_MNOC_TO_BIMC,
	MSM8994_MNOC_SLV_CAMERA_CFG,
	MSM8994_MNOC_SLV_DISPLAY_CFG,
	MSM8994_MNOC_SLV_OCMEM_CFG,
	MSM8994_MNOC_SLV_CPR_CFG,
	MSM8994_MNOC_SLV_CPR_XPU_CFG,
	MSM8994_MNOC_SLV_MISC_CFG,
	MSM8994_MNOC_SLV_MISC_XPU_CFG,
	MSM8994_MNOC_SLV_VENUS_CFG,
	MSM8994_MNOC_SLV_GRAPHICS_3D_CFG,
	MSM8994_MNOC_SLV_MMSS_CLK_CFG,
	MSM8994_MNOC_SLV_MMSS_CLK_XPU_CFG,
	MSM8994_MNOC_SLV_MNOC_MPU_CFG,
	MSM8994_MNOC_SLV_AVSYNC_CFG,
	MSM8994_MNOC_SLV_VPU_CFG,
	MSM8994_MNOC_SLV_SERVICE_MNOC,
	MSM8994_OCMEM_MAS_SNOC,
	MSM8994_OCMEM_MAS_OCMEM_DMA,
	MSM8994_OCMEM_MAS_GFX3D,
	MSM8994_OCMEM_MAS_VIDEO_P0_OCMEM,
	MSM8994_OCMEM_SLV_OCMEM,
	MSM8994_OCMEM_SLV_OCMEM_GFX,
	MSM8994_OCMEM_TO_SNOC,
	MSM8994_PNOC_MAS_PNOC_CFG,
	MSM8994_PNOC_MAS_SDCC_1,
	MSM8994_PNOC_MAS_SDCC_3,
	MSM8994_PNOC_MAS_SDCC_4,
	MSM8994_PNOC_MAS_SDCC_2,
	MSM8994_PNOC_MAS_TSIF,
	MSM8994_PNOC_MAS_BAM_DMA,
	MSM8994_PNOC_MAS_BLSP_2,
	MSM8994_PNOC_MAS_BLSP_1,
	MSM8994_PNOC_MAS_USB_HS,
	MSM8994_PNOC_TO_SNOC,
	MSM8994_PNOC_SLV_SDCC_1,
	MSM8994_PNOC_SLV_SDCC_3,
	MSM8994_PNOC_SLV_SDCC_2,
	MSM8994_PNOC_SLV_SDCC_4,
	MSM8994_PNOC_SLV_TSIF,
	MSM8994_PNOC_SLV_BAM_DMA,
	MSM8994_PNOC_SLV_BLSP_2,
	MSM8994_PNOC_SLV_BLSP_1,
	MSM8994_PNOC_SLV_USB_HS,
	MSM8994_PNOC_SLV_PDM,
	MSM8994_PNOC_SLV_PRNG,
	MSM8994_SNOC_MAS_LPASS_AHB,
	MSM8994_SNOC_MAS_QDSS_BAM,
	MSM8994_SNOC_TO_BIMC,
	MSM8994_SNOC_TO_CNOC,
	MSM8994_SNOC_TO_PNOC,
	MSM8994_SNOC_TO_OCMEM_VNOC,
	MSM8994_SNOC_MAS_CRYPTO_CORE0,
	MSM8994_SNOC_MAS_CRYPTO_CORE1,
	MSM8994_SNOC_MAS_CRYPTO_CORE2,
	MSM8994_SNOC_MAS_LPASS_PROC,
	MSM8994_SNOC_MAS_QDSS_ETR,
	MSM8994_SNOC_MAS_USB3,
	MSM8994_SNOC_MAS_PCIE,
	MSM8994_SNOC_MAS_PCIE_1,
	MSM8994_SNOC_MAS_UFS,
	MSM8994_SNOC_MAS_IPA,
	MSM8994_SNOC_MAS_OVNOC,
	MSM8994_SNOC_SLV_AMPSS,
	MSM8994_SNOC_SLV_LPASS,
	MSM8994_SNOC_SLV_USB3,
	MSM8994_SNOC_SLV_OCIMEM,
	MSM8994_SNOC_SLV_QDSS_STM,
	MSM8994_SNOC_SLV_PCIE_0,
	MSM8994_SNOC_SLV_PCIE_1,
};

static int msm8994_get_bw(struct icc_node *node, u32 *avg, u32 *peak)
{
	*avg = 0;
	*peak = 0;

	return 0;
}

static const u16 mas_ampss_m0_links[] = {
	MSM8994_BIMC_SLV_EBI_CH0
};

static struct qcom_icc_node mas_ampss_m0 = {
	.name = "mas_ampss_m0",
	.id = MSM8994_BIMC_MAS_AMPSS_M0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_ampss_m0_links),
	.links = mas_ampss_m0_links,
};

static const u16 mas_oxili_links[] = {
	MSM8994_BIMC_SLV_EBI_CH0
};

static struct qcom_icc_node mas_oxili = {
	.name = "mas_oxili",
	.id = MSM8994_BIMC_MAS_GRAPHICS_3D,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_oxili_links),
	.links = mas_oxili_links,
};

static const u16 bimc_to_mnoc_links[] = {
	MSM8994_BIMC_SLV_EBI_CH0
};

static struct qcom_icc_node bimc_to_mnoc = {
	.name = "bimc_to_mnoc",
	.id = MSM8994_BIMC_TO_MNOC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(bimc_to_mnoc_links),
	.links = bimc_to_mnoc_links,
};

static const u16 bimc_to_snoc_links[] = {
	MSM8994_SNOC_TO_BIMC,
	MSM8994_BIMC_SLV_EBI_CH0,
	MSM8994_BIMC_MAS_AMPSS_M0
};

static struct qcom_icc_node bimc_to_snoc = {
	.name = "bimc_to_snoc",
	.id = MSM8994_BIMC_TO_SNOC,
	.buswidth = 8,
	.mas_rpm_id = 3,
	.slv_rpm_id = 2,
	.num_links = ARRAY_SIZE(bimc_to_snoc_links),
	.links = bimc_to_snoc_links,
};

static struct qcom_icc_node slv_ebi_ch0 = {
	.name = "slv_ebi_ch0",
	.id = MSM8994_BIMC_SLV_EBI_CH0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 0,
};

static struct qcom_icc_node slv_ampss_l2 = {
	.name = "slv_ampss_l2",
	.id = MSM8994_BIMC_SLV_AMPSS_L2,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 1,
};

static struct qcom_icc_node * const msm8994_bimc_nodes[] = {
	[BIMC_MAS_AMPSS_M0] = &mas_ampss_m0,
	[BIMC_MAS_GRAPHICS_3D] = &mas_oxili,
	[BIMC_TO_MNOC] = &bimc_to_mnoc,
	[BIMC_TO_SNOC] = &bimc_to_snoc,
	[BIMC_SLV_EBI_CH0] = &slv_ebi_ch0,
	[BIMC_SLV_AMPSS_L2] = &slv_ampss_l2,
};

static const struct qcom_icc_desc msm8994_bimc = {
	.nodes = msm8994_bimc_nodes,
	.num_nodes = ARRAY_SIZE(msm8994_bimc_nodes),
	.bus_clk_desc = &bimc_clk,
	.get_bw = msm8994_get_bw,
	.ignore_enxio = true,
};

static const u16 mas_rpm_inst_links[] = {
	MSM8994_CNOC_SLV_BOOT_ROM
};

static struct qcom_icc_node mas_rpm_inst = {
	.name = "mas_rpm_inst",
	.id = MSM8994_CNOC_MAS_RPM_INST,
	.buswidth = 8,
	.mas_rpm_id = 45,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_rpm_inst_links),
	.links = mas_rpm_inst_links,
};

static struct qcom_icc_node mas_rpm_sys = {
	.name = "mas_rpm_sys",
	.id = MSM8994_CNOC_MAS_RPM_SYS,
	.buswidth = 8,
	.mas_rpm_id = 47,
	.slv_rpm_id = -1,
};

static const u16 mas_dehr_links[] = {
	MSM8994_CNOC_SLV_BIMC_CFG
};

static struct qcom_icc_node mas_dehr = {
	.name = "mas_dehr",
	.id = MSM8994_CNOC_MAS_DEHR,
	.buswidth = 8,
	.mas_rpm_id = 48,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_dehr_links),
	.links = mas_dehr_links,
};

static struct qcom_icc_node mas_qdss_dap = {
	.name = "mas_qdss_dap",
	.id = MSM8994_CNOC_MAS_QDSS_DAP,
	.buswidth = 8,
	.mas_rpm_id = 49,
	.slv_rpm_id = -1,
};

static const u16 mas_spdm_links[] = {
	MSM8994_CNOC_TO_SNOC
};

static struct qcom_icc_node mas_spdm = {
	.name = "mas_spdm",
	.id = MSM8994_CNOC_MAS_SPDM,
	.buswidth = 8,
	.mas_rpm_id = 50,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_spdm_links),
	.links = mas_spdm_links,
};

static struct qcom_icc_node mas_tic = {
	.name = "mas_tic",
	.id = MSM8994_CNOC_MAS_TIC,
	.buswidth = 8,
	.mas_rpm_id = 51,
	.slv_rpm_id = -1,
};

static const u16 cnoc_to_snoc_links[] = {
	MSM8994_SNOC_TO_CNOC
};

static struct qcom_icc_node cnoc_to_snoc = {
	.name = "cnoc_to_snoc",
	.id = MSM8994_CNOC_TO_SNOC,
	.buswidth = 8,
	.mas_rpm_id = 52,
	.slv_rpm_id = 75,
	.num_links = ARRAY_SIZE(cnoc_to_snoc_links),
	.links = cnoc_to_snoc_links,
};

static struct qcom_icc_node slv_clk_ctl = {
	.name = "slv_clk_ctl",
	.id = MSM8994_CNOC_SLV_CLK_CTL,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 47,
};

static struct qcom_icc_node slv_security = {
	.name = "slv_security",
	.id = MSM8994_CNOC_SLV_SECURITY,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 49,
};

static struct qcom_icc_node slv_tcsr = {
	.name = "slv_tcsr",
	.id = MSM8994_CNOC_SLV_TCSR,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 50,
};

static struct qcom_icc_node slv_tlmm = {
	.name = "slv_tlmm",
	.id = MSM8994_CNOC_SLV_TLMM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 51,
};

static struct qcom_icc_node slv_crypto_0_cfg = {
	.name = "slv_crypto_0_cfg",
	.id = MSM8994_CNOC_SLV_CRYPTO_0_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 52,
};

static struct qcom_icc_node slv_crypto_1_cfg = {
	.name = "slv_crypto_1_cfg",
	.id = MSM8994_CNOC_SLV_CRYPTO_1_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 53,
};

static struct qcom_icc_node slv_crypto_2_cfg = {
	.name = "slv_crypto_2_cfg",
	.id = MSM8994_CNOC_SLV_CRYPTO_2_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 87,
};

static struct qcom_icc_node slv_imem_cfg = {
	.name = "slv_imem_cfg",
	.id = MSM8994_CNOC_SLV_IMEM_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 54,
};

static struct qcom_icc_node slv_message_ram = {
	.name = "slv_message_ram",
	.id = MSM8994_CNOC_SLV_MESSAGE_RAM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 55,
};

static struct qcom_icc_node slv_bimc_cfg = {
	.name = "slv_bimc_cfg",
	.id = MSM8994_CNOC_SLV_BIMC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 56,
};

static struct qcom_icc_node slv_boot_rom = {
	.name = "slv_boot_rom",
	.id = MSM8994_CNOC_SLV_BOOT_ROM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 57,
};

static const u16 slv_cnoc_mnoc_mmss_cfg_links[] = {
	MSM8994_MNOC_MAS_CNOC_MMSS_CFG
};

static struct qcom_icc_node slv_cnoc_mnoc_mmss_cfg = {
	.name = "slv_cnoc_mnoc_mmss_cfg",
	.id = MSM8994_CNOC_SLV_CNOC_MNOC_MMSS_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 58,
	.num_links = ARRAY_SIZE(slv_cnoc_mnoc_mmss_cfg_links),
	.links = slv_cnoc_mnoc_mmss_cfg_links,
};

static struct qcom_icc_node slv_pmic_arb = {
	.name = "slv_pmic_arb",
	.id = MSM8994_CNOC_SLV_PMIC_ARB,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 59,
};

static struct qcom_icc_node slv_spdm_wrapper = {
	.name = "slv_spdm_wrapper",
	.id = MSM8994_CNOC_SLV_SPDM_WRAPPER,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 60,
};

static struct qcom_icc_node slv_dehr_cfg = {
	.name = "slv_dehr_cfg",
	.id = MSM8994_CNOC_SLV_DEHR_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 61,
};

static struct qcom_icc_node slv_mpm = {
	.name = "slv_mpm",
	.id = MSM8994_CNOC_SLV_MPM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 62,
};

static struct qcom_icc_node slv_qdss_cfg = {
	.name = "slv_qdss_cfg",
	.id = MSM8994_CNOC_SLV_QDSS_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 63,
};

static struct qcom_icc_node slv_rbcpr_cfg = {
	.name = "slv_rbcpr_cfg",
	.id = MSM8994_CNOC_SLV_RBCPR_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 64,
};

static struct qcom_icc_node slv_rbcpr_qdss_apu_cfg = {
	.name = "slv_rbcpr_qdss_apu_cfg",
	.id = MSM8994_CNOC_SLV_RBCPR_QDSS_APU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 65,
};

static const u16 slv_cnoc_mnoc_cfg_links[] = {
	MSM8994_MNOC_MAS_CNOC_CFG
};

static struct qcom_icc_node slv_cnoc_mnoc_cfg = {
	.name = "slv_cnoc_mnoc_cfg",
	.id = MSM8994_CNOC_SLV_CNOC_MNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 66,
	.num_links = ARRAY_SIZE(slv_cnoc_mnoc_cfg_links),
	.links = slv_cnoc_mnoc_cfg_links,
};

static const u16 slv_pnoc_cfg_links[] = {
	MSM8994_PNOC_MAS_PNOC_CFG
};

static struct qcom_icc_node slv_pnoc_cfg = {
	.name = "slv_pnoc_cfg",
	.id = MSM8994_CNOC_SLV_PNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 69,
	.num_links = ARRAY_SIZE(slv_pnoc_cfg_links),
	.links = slv_pnoc_cfg_links,
};

static struct qcom_icc_node slv_snoc_mpu_cfg = {
	.name = "slv_snoc_mpu_cfg",
	.id = MSM8994_CNOC_SLV_SNOC_MPU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 67,
};

static struct qcom_icc_node slv_snoc_cfg = {
	.name = "slv_snoc_cfg",
	.id = MSM8994_CNOC_SLV_SNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 70,
};

static struct qcom_icc_node slv_ebi1_dll_cfg = {
	.name = "slv_ebi1_dll_cfg",
	.id = MSM8994_CNOC_SLV_EBI1_DLL_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 71,
};

static struct qcom_icc_node slv_phy_apu_cfg = {
	.name = "slv_phy_apu_cfg",
	.id = MSM8994_CNOC_SLV_PHY_APU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 72,
};

static struct qcom_icc_node slv_ebi1_phy_cfg = {
	.name = "slv_ebi1_phy_cfg",
	.id = MSM8994_CNOC_SLV_EBI1_PHY_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 73,
};

static struct qcom_icc_node slv_rpm = {
	.name = "slv_rpm",
	.id = MSM8994_CNOC_SLV_RPM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 74,
};

static struct qcom_icc_node slv_pcie_0_cfg = {
	.name = "slv_pcie_0_cfg",
	.id = MSM8994_CNOC_SLV_PCIE_0_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 88,
};

static struct qcom_icc_node slv_pcie_1_cfg = {
	.name = "slv_pcie_1_cfg",
	.id = MSM8994_CNOC_SLV_PCIE_1_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 89,
};

static struct qcom_icc_node slv_geni_ir_cfg = {
	.name = "slv_geni_ir_cfg",
	.id = MSM8994_CNOC_SLV_GENI_IR_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 91,
};

static struct qcom_icc_node slv_ufs_cfg = {
	.name = "slv_ufs_cfg",
	.id = MSM8994_CNOC_SLV_UFS_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 92,
};

static struct qcom_icc_node * const msm8994_cnoc_nodes[] = {
	[CNOC_MAS_RPM_INST] = &mas_rpm_inst,
	[CNOC_MAS_RPM_SYS] = &mas_rpm_sys,
	[CNOC_MAS_DEHR] = &mas_dehr,
	[CNOC_MAS_QDSS_DAP] = &mas_qdss_dap,
	[CNOC_MAS_SPDM] = &mas_spdm,
	[CNOC_MAS_TIC] = &mas_tic,
	[CNOC_TO_SNOC] = &cnoc_to_snoc,
	[CNOC_SLV_CLK_CTL] = &slv_clk_ctl,
	[CNOC_SLV_SECURITY] = &slv_security,
	[CNOC_SLV_TCSR] = &slv_tcsr,
	[CNOC_SLV_TLMM] = &slv_tlmm,
	[CNOC_SLV_CRYPTO_0_CFG] = &slv_crypto_0_cfg,
	[CNOC_SLV_CRYPTO_1_CFG] = &slv_crypto_1_cfg,
	[CNOC_SLV_CRYPTO_2_CFG] = &slv_crypto_2_cfg,
	[CNOC_SLV_IMEM_CFG] = &slv_imem_cfg,
	[CNOC_SLV_MESSAGE_RAM] = &slv_message_ram,
	[CNOC_SLV_BIMC_CFG] = &slv_bimc_cfg,
	[CNOC_SLV_BOOT_ROM] = &slv_boot_rom,
	[CNOC_SLV_PMIC_ARB] = &slv_pmic_arb,
	[CNOC_SLV_SPDM_WRAPPER] = &slv_spdm_wrapper,
	[CNOC_SLV_DEHR_CFG] = &slv_dehr_cfg,
	[CNOC_SLV_MPM] = &slv_mpm,
	[CNOC_SLV_QDSS_CFG] = &slv_qdss_cfg,
	[CNOC_SLV_RBCPR_CFG] = &slv_rbcpr_cfg,
	[CNOC_SLV_RBCPR_QDSS_APU_CFG] = &slv_rbcpr_qdss_apu_cfg,
	[CNOC_SLV_CNOC_MNOC_MMSS_CFG] = &slv_cnoc_mnoc_mmss_cfg,
	[CNOC_SLV_CNOC_MNOC_CFG] = &slv_cnoc_mnoc_cfg,
	[CNOC_SLV_PNOC_CFG] = &slv_pnoc_cfg,
	[CNOC_SLV_SNOC_MPU_CFG] = &slv_snoc_mpu_cfg,
	[CNOC_SLV_SNOC_CFG] = &slv_snoc_cfg,
	[CNOC_SLV_EBI1_DLL_CFG] = &slv_ebi1_dll_cfg,
	[CNOC_SLV_PHY_APU_CFG] = &slv_phy_apu_cfg,
	[CNOC_SLV_EBI1_PHY_CFG] = &slv_ebi1_phy_cfg,
	[CNOC_SLV_RPM] = &slv_rpm,
	[CNOC_SLV_PCIE_0_CFG] = &slv_pcie_0_cfg,
	[CNOC_SLV_PCIE_1_CFG] = &slv_pcie_1_cfg,
	[CNOC_SLV_GENI_IR_CFG] = &slv_geni_ir_cfg,
	[CNOC_SLV_UFS_CFG] = &slv_ufs_cfg,
};

static const struct qcom_icc_desc msm8994_cnoc = {
	.nodes = msm8994_cnoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8994_cnoc_nodes),
	.bus_clk_desc = &bus_2_clk,
	.get_bw = msm8994_get_bw,
	.ignore_enxio = true,
};

static const u16 mas_jpeg_links[] = {
	MSM8994_MNOC_TO_BIMC
};

static struct qcom_icc_node mas_jpeg = {
	.name = "mas_jpeg",
	.id = MSM8994_MNOC_MAS_JPEG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_jpeg_links),
	.links = mas_jpeg_links,
};

static const u16 mas_mdp_port0_links[] = {
	MSM8994_MNOC_TO_BIMC
};

static struct qcom_icc_node mas_mdp_port0 = {
	.name = "mas_mdp_port0",
	.id = MSM8994_MNOC_MAS_MDP_PORT0,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_mdp_port0_links),
	.links = mas_mdp_port0_links,
};

static const u16 mas_mdp_port1_links[] = {
	MSM8994_MNOC_TO_BIMC
};

static struct qcom_icc_node mas_mdp_port1 = {
	.name = "mas_mdp_port1",
	.id = MSM8994_MNOC_MAS_MDP_PORT1,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_mdp_port1_links),
	.links = mas_mdp_port1_links,
};

static const u16 mas_video_p0_links[] = {
	MSM8994_MNOC_TO_BIMC
};

static struct qcom_icc_node mas_video_p0 = {
	.name = "mas_video_p0",
	.id = MSM8994_MNOC_MAS_VIDEO_P0,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_video_p0_links),
	.links = mas_video_p0_links,
};

static const u16 mas_video_p1_links[] = {
	MSM8994_MNOC_TO_BIMC
};

static struct qcom_icc_node mas_video_p1 = {
	.name = "mas_video_p1",
	.id = MSM8994_MNOC_MAS_VIDEO_P1,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_video_p1_links),
	.links = mas_video_p1_links,
};

static const u16 mas_vfe_links[] = {
	MSM8994_MNOC_TO_BIMC
};

static struct qcom_icc_node mas_vfe = {
	.name = "mas_vfe",
	.id = MSM8994_MNOC_MAS_VFE,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_vfe_links),
	.links = mas_vfe_links,
};

static const u16 mas_cpp_links[] = {
	MSM8994_MNOC_TO_BIMC
};

static struct qcom_icc_node mas_cpp = {
	.name = "mas_cpp",
	.id = MSM8994_MNOC_MAS_CPP,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_cpp_links),
	.links = mas_cpp_links,
};

static const u16 mas_vpu_links[] = {
	MSM8994_MNOC_TO_BIMC
};

static struct qcom_icc_node mas_vpu = {
	.name = "mas_vpu",
	.id = MSM8994_MNOC_MAS_VPU,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_vpu_links),
	.links = mas_vpu_links,
};

static struct qcom_icc_node mas_cnoc_mnoc_mmss_cfg = {
	.name = "mas_cnoc_mnoc_mmss_cfg",
	.id = MSM8994_MNOC_MAS_CNOC_MMSS_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
};

static const u16 mas_cnoc_mnoc_cfg_links[] = {
	MSM8994_MNOC_SLV_SERVICE_MNOC
};

static struct qcom_icc_node mas_cnoc_mnoc_cfg = {
	.name = "mas_cnoc_mnoc_cfg",
	.id = MSM8994_MNOC_MAS_CNOC_CFG,
	.buswidth = 16,
	.mas_rpm_id = 5,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_cnoc_mnoc_cfg_links),
	.links = mas_cnoc_mnoc_cfg_links,
};

static const u16 mnoc_to_bimc_links[] = {
	MSM8994_BIMC_TO_MNOC
};

static struct qcom_icc_node mnoc_to_bimc = {
	.name = "mnoc_to_bimc",
	.id = MSM8994_MNOC_TO_BIMC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mnoc_to_bimc_links),
	.links = mnoc_to_bimc_links,
};

static struct qcom_icc_node slv_camera_cfg = {
	.name = "slv_camera_cfg",
	.id = MSM8994_MNOC_SLV_CAMERA_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 3,
};

static struct qcom_icc_node slv_display_cfg = {
	.name = "slv_display_cfg",
	.id = MSM8994_MNOC_SLV_DISPLAY_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 4,
};

static struct qcom_icc_node slv_ocmem_cfg = {
	.name = "slv_ocmem_cfg",
	.id = MSM8994_MNOC_SLV_OCMEM_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 5,
};

static struct qcom_icc_node slv_cpr_cfg = {
	.name = "slv_cpr_cfg",
	.id = MSM8994_MNOC_SLV_CPR_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 6,
};

static struct qcom_icc_node slv_cpr_xpu_cfg = {
	.name = "slv_cpr_xpu_cfg",
	.id = MSM8994_MNOC_SLV_CPR_XPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 7,
};

static struct qcom_icc_node slv_misc_cfg = {
	.name = "slv_misc_cfg",
	.id = MSM8994_MNOC_SLV_MISC_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 8,
};

static struct qcom_icc_node slv_misc_xpu_cfg = {
	.name = "slv_misc_xpu_cfg",
	.id = MSM8994_MNOC_SLV_MISC_XPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 9,
};

static struct qcom_icc_node slv_venus_cfg = {
	.name = "slv_venus_cfg",
	.id = MSM8994_MNOC_SLV_VENUS_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 10,
};

static struct qcom_icc_node slv_graphics_3d_cfg = {
	.name = "slv_graphics_3d_cfg",
	.id = MSM8994_MNOC_SLV_GRAPHICS_3D_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 11,
};

static struct qcom_icc_node slv_mmss_clk_cfg = {
	.name = "slv_mmss_clk_cfg",
	.id = MSM8994_MNOC_SLV_MMSS_CLK_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 12,
};

static struct qcom_icc_node slv_mmss_clk_xpu_cfg = {
	.name = "slv_mmss_clk_xpu_cfg",
	.id = MSM8994_MNOC_SLV_MMSS_CLK_XPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 13,
};

static struct qcom_icc_node slv_mnoc_mpu_cfg = {
	.name = "slv_mnoc_mpu_cfg",
	.id = MSM8994_MNOC_SLV_MNOC_MPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 14,
};

static struct qcom_icc_node slv_avsync_cfg = {
	.name = "slv_avsync_cfg",
	.id = MSM8994_MNOC_SLV_AVSYNC_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 93,
};

static struct qcom_icc_node slv_vpu_cfg = {
	.name = "slv_vpu_cfg",
	.id = MSM8994_MNOC_SLV_VPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 94,
};

static struct qcom_icc_node slv_service_mnoc = {
	.name = "slv_service_mnoc",
	.id = MSM8994_MNOC_SLV_SERVICE_MNOC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 17,
};

static struct qcom_icc_node * const msm8994_mnoc_nodes[] = {
	[MNOC_MAS_JPEG] = &mas_jpeg,
	[MNOC_MAS_MDP_PORT0] = &mas_mdp_port0,
	[MNOC_MAS_MDP_PORT1] = &mas_mdp_port1,
	[MNOC_MAS_VIDEO_P0] = &mas_video_p0,
	[MNOC_MAS_VIDEO_P1] = &mas_video_p1,
	[MNOC_MAS_VFE] = &mas_vfe,
	[MNOC_MAS_CPP] = &mas_cpp,
	[MNOC_MAS_VPU] = &mas_vpu,
	[MNOC_MAS_CNOC_MMSS_CFG] = &mas_cnoc_mnoc_mmss_cfg,
	[MNOC_MAS_CNOC_CFG] = &mas_cnoc_mnoc_cfg,
	[MNOC_TO_BIMC] = &mnoc_to_bimc,
	[MNOC_SLV_CAMERA_CFG] = &slv_camera_cfg,
	[MNOC_SLV_DISPLAY_CFG] = &slv_display_cfg,
	[MNOC_SLV_OCMEM_CFG] = &slv_ocmem_cfg,
	[MNOC_SLV_CPR_CFG] = &slv_cpr_cfg,
	[MNOC_SLV_CPR_XPU_CFG] = &slv_cpr_xpu_cfg,
	[MNOC_SLV_MISC_CFG] = &slv_misc_cfg,
	[MNOC_SLV_MISC_XPU_CFG] = &slv_misc_xpu_cfg,
	[MNOC_SLV_VENUS_CFG] = &slv_venus_cfg,
	[MNOC_SLV_GRAPHICS_3D_CFG] = &slv_graphics_3d_cfg,
	[MNOC_SLV_MMSS_CLK_CFG] = &slv_mmss_clk_cfg,
	[MNOC_SLV_MMSS_CLK_XPU_CFG] = &slv_mmss_clk_xpu_cfg,
	[MNOC_SLV_MNOC_MPU_CFG] = &slv_mnoc_mpu_cfg,
	[MNOC_SLV_AVSYNC_CFG] = &slv_avsync_cfg,
	[MNOC_SLV_VPU_CFG] = &slv_vpu_cfg,
	[MNOC_SLV_SERVICE_MNOC] = &slv_service_mnoc,
};

static const struct qcom_icc_desc msm8994_mnoc = {
	.nodes = msm8994_mnoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8994_mnoc_nodes),
	.get_bw = msm8994_get_bw,
	.ignore_enxio = true,
};

static const u16 mas_snoc_ovirt_links[] = {
	MSM8994_OCMEM_SLV_OCMEM
};

static struct qcom_icc_node mas_snoc_ovirt = {
	.name = "mas_snoc_ovirt",
	.id = MSM8994_OCMEM_MAS_SNOC,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_snoc_ovirt_links),
	.links = mas_snoc_ovirt_links,
};

static const u16 mas_ocmem_dma_links[] = {
	MSM8994_OCMEM_SLV_OCMEM,
	MSM8994_OCMEM_TO_SNOC
};

static struct qcom_icc_node mas_ocmem_dma = {
	.name = "mas_ocmem_dma",
	.id = MSM8994_OCMEM_MAS_OCMEM_DMA,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_ocmem_dma_links),
	.links = mas_ocmem_dma_links,
};

static const u16 mas_oxili_ocmem_links[] = {
	MSM8994_OCMEM_SLV_OCMEM_GFX
};

static struct qcom_icc_node mas_oxili_ocmem = {
	.name = "mas_oxili_ocmem",
	.id = MSM8994_OCMEM_MAS_GFX3D,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_oxili_ocmem_links),
	.links = mas_oxili_ocmem_links,
};

static const u16 mas_venus_ocmem_links[] = {
	MSM8994_OCMEM_SLV_OCMEM
};

static struct qcom_icc_node mas_venus_ocmem = {
	.name = "mas_venus_ocmem",
	.id = MSM8994_OCMEM_MAS_VIDEO_P0_OCMEM,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_venus_ocmem_links),
	.links = mas_venus_ocmem_links,
};

static struct qcom_icc_node slv_ocmem = {
	.name = "slv_ocmem",
	.id = MSM8994_OCMEM_SLV_OCMEM,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node slv_ocmem_gfx = {
	.name = "slv_ocmem_gfx",
	.id = MSM8994_OCMEM_SLV_OCMEM_GFX,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
};

static const u16 ocmem_to_snoc_links[] = {
	MSM8994_SNOC_MAS_OVNOC
};

static struct qcom_icc_node ocmem_to_snoc = {
	.name = "ocmem_to_snoc",
	.id = MSM8994_OCMEM_TO_SNOC,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = 77,
	.num_links = ARRAY_SIZE(ocmem_to_snoc_links),
	.links = ocmem_to_snoc_links,
};

static struct qcom_icc_node * const msm8994_ovirt_nodes[] = {
	[OCMEM_MAS_SNOC] = &mas_snoc_ovirt,
	[OCMEM_MAS_OCMEM_DMA] = &mas_ocmem_dma,
	[OCMEM_MAS_GFX3D] = &mas_oxili_ocmem,
	[OCMEM_MAS_VIDEO_P0_OCMEM] = &mas_venus_ocmem,
	[OCMEM_SLV_OCMEM] = &slv_ocmem,
	[OCMEM_SLV_OCMEM_GFX] = &slv_ocmem_gfx,
	[OCMEM_TO_SNOC] = &ocmem_to_snoc,
};

static const struct qcom_icc_desc msm8994_ovirt = {
	.nodes = msm8994_ovirt_nodes,
	.num_nodes = ARRAY_SIZE(msm8994_ovirt_nodes),
	.bus_clk_desc = &gpu_mem_2_clk,
	.get_bw = msm8994_get_bw,
	.ignore_enxio = true,
};

static const u16 mas_pnoc_cfg_links[] = {
	MSM8994_PNOC_SLV_PRNG
};

static struct qcom_icc_node mas_pnoc_cfg = {
	.name = "mas_pnoc_cfg",
	.id = MSM8994_PNOC_MAS_PNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = 43,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_pnoc_cfg_links),
	.links = mas_pnoc_cfg_links,
};

static const u16 mas_sdcc_1_links[] = {
	MSM8994_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_sdcc_1 = {
	.name = "mas_sdcc_1",
	.id = MSM8994_PNOC_MAS_SDCC_1,
	.buswidth = 8,
	.mas_rpm_id = 33,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_1_links),
	.links = mas_sdcc_1_links,
};

static const u16 mas_sdcc_3_links[] = {
	MSM8994_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_sdcc_3 = {
	.name = "mas_sdcc_3",
	.id = MSM8994_PNOC_MAS_SDCC_3,
	.buswidth = 8,
	.mas_rpm_id = 34,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_3_links),
	.links = mas_sdcc_3_links,
};

static const u16 mas_sdcc_4_links[] = {
	MSM8994_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_sdcc_4 = {
	.name = "mas_sdcc_4",
	.id = MSM8994_PNOC_MAS_SDCC_4,
	.buswidth = 8,
	.mas_rpm_id = 36,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_4_links),
	.links = mas_sdcc_4_links,
};

static const u16 mas_sdcc_2_links[] = {
	MSM8994_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_sdcc_2 = {
	.name = "mas_sdcc_2",
	.id = MSM8994_PNOC_MAS_SDCC_2,
	.buswidth = 8,
	.mas_rpm_id = 35,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_2_links),
	.links = mas_sdcc_2_links,
};

static const u16 mas_tsif_links[] = {
	MSM8994_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_tsif = {
	.name = "mas_tsif",
	.id = MSM8994_PNOC_MAS_TSIF,
	.buswidth = 8,
	.mas_rpm_id = 37,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_tsif_links),
	.links = mas_tsif_links,
};

static const u16 mas_bam_dma_links[] = {
	MSM8994_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_bam_dma = {
	.name = "mas_bam_dma",
	.id = MSM8994_PNOC_MAS_BAM_DMA,
	.buswidth = 8,
	.mas_rpm_id = 38,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_bam_dma_links),
	.links = mas_bam_dma_links,
};

static const u16 mas_blsp_2_links[] = {
	MSM8994_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_blsp_2 = {
	.name = "mas_blsp_2",
	.id = MSM8994_PNOC_MAS_BLSP_2,
	.buswidth = 8,
	.mas_rpm_id = 39,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_blsp_2_links),
	.links = mas_blsp_2_links,
};

static const u16 mas_blsp_1_links[] = {
	MSM8994_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_blsp_1 = {
	.name = "mas_blsp_1",
	.id = MSM8994_PNOC_MAS_BLSP_1,
	.buswidth = 8,
	.mas_rpm_id = 41,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_blsp_1_links),
	.links = mas_blsp_1_links,
};

static const u16 mas_usb_hs_links[] = {
	MSM8994_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_usb_hs = {
	.name = "mas_usb_hs",
	.id = MSM8994_PNOC_MAS_USB_HS,
	.buswidth = 8,
	.mas_rpm_id = 42,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_usb_hs_links),
	.links = mas_usb_hs_links,
};

static const u16 pnoc_to_snoc_links[] = {
	MSM8994_SNOC_TO_PNOC,
	MSM8994_PNOC_SLV_PRNG
};

static struct qcom_icc_node pnoc_to_snoc = {
	.name = "pnoc_to_snoc",
	.id = MSM8994_PNOC_TO_SNOC,
	.buswidth = 8,
	.mas_rpm_id = 44,
	.slv_rpm_id = 45,
	.num_links = ARRAY_SIZE(pnoc_to_snoc_links),
	.links = pnoc_to_snoc_links,
};

static struct qcom_icc_node slv_sdcc_1 = {
	.name = "slv_sdcc_1",
	.id = MSM8994_PNOC_SLV_SDCC_1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 31,
};

static struct qcom_icc_node slv_sdcc_3 = {
	.name = "slv_sdcc_3",
	.id = MSM8994_PNOC_SLV_SDCC_3,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 32,
};

static struct qcom_icc_node slv_sdcc_2 = {
	.name = "slv_sdcc_2",
	.id = MSM8994_PNOC_SLV_SDCC_2,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 33,
};

static struct qcom_icc_node slv_sdcc_4 = {
	.name = "slv_sdcc_4",
	.id = MSM8994_PNOC_SLV_SDCC_4,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 34,
};

static struct qcom_icc_node slv_tsif = {
	.name = "slv_tsif",
	.id = MSM8994_PNOC_SLV_TSIF,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 35,
};

static struct qcom_icc_node slv_bam_dma = {
	.name = "slv_bam_dma",
	.id = MSM8994_PNOC_SLV_BAM_DMA,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 36,
};

static struct qcom_icc_node slv_blsp_2 = {
	.name = "slv_blsp_2",
	.id = MSM8994_PNOC_SLV_BLSP_2,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 37,
};

static struct qcom_icc_node slv_blsp_1 = {
	.name = "slv_blsp_1",
	.id = MSM8994_PNOC_SLV_BLSP_1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 39,
};

static struct qcom_icc_node slv_usb_hs = {
	.name = "slv_usb_hs",
	.id = MSM8994_PNOC_SLV_USB_HS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 40,
};

static struct qcom_icc_node slv_pdm = {
	.name = "slv_pdm",
	.id = MSM8994_PNOC_SLV_PDM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 41,
};

static const u16 slv_prng_links[] = {
	MSM8994_PNOC_TO_SNOC
};

static struct qcom_icc_node slv_prng = {
	.name = "slv_prng",
	.id = MSM8994_PNOC_SLV_PRNG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 44,
	.num_links = ARRAY_SIZE(slv_prng_links),
	.links = slv_prng_links,
};

static struct qcom_icc_node * const msm8994_pnoc_nodes[] = {
	[PNOC_MAS_PNOC_CFG] = &mas_pnoc_cfg,
	[PNOC_MAS_SDCC_1] = &mas_sdcc_1,
	[PNOC_MAS_SDCC_3] = &mas_sdcc_3,
	[PNOC_MAS_SDCC_4] = &mas_sdcc_4,
	[PNOC_MAS_SDCC_2] = &mas_sdcc_2,
	[PNOC_MAS_TSIF] = &mas_tsif,
	[PNOC_MAS_BAM_DMA] = &mas_bam_dma,
	[PNOC_MAS_BLSP_2] = &mas_blsp_2,
	[PNOC_MAS_BLSP_1] = &mas_blsp_1,
	[PNOC_MAS_USB_HS] = &mas_usb_hs,
	[PNOC_TO_SNOC] = &pnoc_to_snoc,
	[PNOC_SLV_SDCC_1] = &slv_sdcc_1,
	[PNOC_SLV_SDCC_3] = &slv_sdcc_3,
	[PNOC_SLV_SDCC_2] = &slv_sdcc_2,
	[PNOC_SLV_SDCC_4] = &slv_sdcc_4,
	[PNOC_SLV_TSIF] = &slv_tsif,
	[PNOC_SLV_BAM_DMA] = &slv_bam_dma,
	[PNOC_SLV_BLSP_2] = &slv_blsp_2,
	[PNOC_SLV_BLSP_1] = &slv_blsp_1,
	[PNOC_SLV_USB_HS] = &slv_usb_hs,
	[PNOC_SLV_PDM] = &slv_pdm,
	[PNOC_SLV_PRNG] = &slv_prng,
};

static const struct qcom_icc_desc msm8994_pnoc = {
	.nodes = msm8994_pnoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8994_pnoc_nodes),
	.bus_clk_desc = &bus_0_clk,
	.get_bw = msm8994_get_bw,
	.keep_alive = true,
	.ignore_enxio = true,
};

static const u16 mas_lpass_ahb_links[] = {
	MSM8994_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_lpass_ahb = {
	.name = "mas_lpass_ahb",
	.id = MSM8994_SNOC_MAS_LPASS_AHB,
	.buswidth = 8,
	.mas_rpm_id = 18,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_lpass_ahb_links),
	.links = mas_lpass_ahb_links,
};

static const u16 mas_qdss_bam_links[] = {
	MSM8994_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_qdss_bam = {
	.name = "mas_qdss_bam",
	.id = MSM8994_SNOC_MAS_QDSS_BAM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_qdss_bam_links),
	.links = mas_qdss_bam_links,
};

static const u16 snoc_to_bimc_links[] = {
	MSM8994_BIMC_TO_SNOC
};

static struct qcom_icc_node snoc_to_bimc = {
	.name = "snoc_to_bimc",
	.id = MSM8994_SNOC_TO_BIMC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 24,
	.num_links = ARRAY_SIZE(snoc_to_bimc_links),
	.links = snoc_to_bimc_links,
};

static const u16 snoc_to_cnoc_links[] = {
	MSM8994_CNOC_TO_SNOC
};

static struct qcom_icc_node snoc_to_cnoc = {
	.name = "snoc_to_cnoc",
	.id = MSM8994_SNOC_TO_CNOC,
	.buswidth = 8,
	.mas_rpm_id = 22,
	.slv_rpm_id = 25,
	.num_links = ARRAY_SIZE(snoc_to_cnoc_links),
	.links = snoc_to_cnoc_links,
};

static const u16 snoc_to_pnoc_links[] = {
	MSM8994_PNOC_TO_SNOC,
	MSM8994_SNOC_TO_BIMC
};

static struct qcom_icc_node snoc_to_pnoc = {
	.name = "snoc_to_pnoc",
	.id = MSM8994_SNOC_TO_PNOC,
	.buswidth = 8,
	.mas_rpm_id = 29,
	.slv_rpm_id = 28,
	.num_links = ARRAY_SIZE(snoc_to_pnoc_links),
	.links = snoc_to_pnoc_links,
};

static const u16 snoc_to_ocmem_vnoc_links[] = {
	MSM8994_OCMEM_MAS_SNOC
};

static struct qcom_icc_node snoc_to_ocmem_vnoc = {
	.name = "snoc_to_ocmem_vnoc",
	.id = MSM8994_SNOC_TO_OCMEM_VNOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 27,
	.num_links = ARRAY_SIZE(snoc_to_ocmem_vnoc_links),
	.links = snoc_to_ocmem_vnoc_links,
};

static const u16 mas_crypto_core0_links[] = {
	MSM8994_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_crypto_core0 = {
	.name = "mas_crypto_core0",
	.id = MSM8994_SNOC_MAS_CRYPTO_CORE0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_crypto_core0_links),
	.links = mas_crypto_core0_links,
};

static const u16 mas_crypto_core1_links[] = {
	MSM8994_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_crypto_core1 = {
	.name = "mas_crypto_core1",
	.id = MSM8994_SNOC_MAS_CRYPTO_CORE1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_crypto_core1_links),
	.links = mas_crypto_core1_links,
};

static const u16 mas_crypto_core2_links[] = {
	MSM8994_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_crypto_core2 = {
	.name = "mas_crypto_core2",
	.id = MSM8994_SNOC_MAS_CRYPTO_CORE2,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_crypto_core2_links),
	.links = mas_crypto_core2_links,
};

static const u16 mas_lpass_proc_links[] = {
	MSM8994_SNOC_TO_BIMC,
	MSM8994_SNOC_TO_OCMEM_VNOC
};

static struct qcom_icc_node mas_lpass_proc = {
	.name = "mas_lpass_proc",
	.id = MSM8994_SNOC_MAS_LPASS_PROC,
	.buswidth = 8,
	.mas_rpm_id = 25,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_lpass_proc_links),
	.links = mas_lpass_proc_links,
};

static const u16 mas_qdss_etr_links[] = {
	MSM8994_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_qdss_etr = {
	.name = "mas_qdss_etr",
	.id = MSM8994_SNOC_MAS_QDSS_ETR,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_qdss_etr_links),
	.links = mas_qdss_etr_links,
};

static const u16 mas_usb3_links[] = {
	MSM8994_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_usb3 = {
	.name = "mas_usb3",
	.id = MSM8994_SNOC_MAS_USB3,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_usb3_links),
	.links = mas_usb3_links,
};

static const u16 mas_pcie_0_links[] = {
	MSM8994_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_pcie_0 = {
	.name = "mas_pcie_0",
	.id = MSM8994_SNOC_MAS_PCIE,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_pcie_0_links),
	.links = mas_pcie_0_links,
};

static const u16 mas_pcie_1_links[] = {
	MSM8994_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_pcie_1 = {
	.name = "mas_pcie_1",
	.id = MSM8994_SNOC_MAS_PCIE_1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_pcie_1_links),
	.links = mas_pcie_1_links,
};

static const u16 mas_ufs_links[] = {
	MSM8994_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_ufs = {
	.name = "mas_ufs",
	.id = MSM8994_SNOC_MAS_UFS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_ufs_links),
	.links = mas_ufs_links,
};

static const u16 mas_ipa_links[] = {
	MSM8994_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_ipa = {
	.name = "mas_ipa",
	.id = MSM8994_SNOC_MAS_IPA,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_ipa_links),
	.links = mas_ipa_links,
};

static const u16 mas_ovirt_snoc_links[] = {
	MSM8994_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_ovirt_snoc = {
	.name = "mas_ovirt_snoc",
	.id = MSM8994_SNOC_MAS_OVNOC,
	.buswidth = 8,
	.mas_rpm_id = 54,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_ovirt_snoc_links),
	.links = mas_ovirt_snoc_links,
};

static struct qcom_icc_node slv_ampss = {
	.name = "slv_ampss",
	.id = MSM8994_SNOC_SLV_AMPSS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 20,
};

static struct qcom_icc_node slv_lpass = {
	.name = "slv_lpass",
	.id = MSM8994_SNOC_SLV_LPASS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 21,
};

static struct qcom_icc_node slv_usb3 = {
	.name = "slv_usb3",
	.id = MSM8994_SNOC_SLV_USB3,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 22,
};

static struct qcom_icc_node slv_ocimem = {
	.name = "slv_ocimem",
	.id = MSM8994_SNOC_SLV_OCIMEM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 26,
};

static struct qcom_icc_node slv_qdss_stm = {
	.name = "slv_qdss_stm",
	.id = MSM8994_SNOC_SLV_QDSS_STM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 30,
};

static struct qcom_icc_node slv_pcie_0 = {
	.name = "slv_pcie_0",
	.id = MSM8994_SNOC_SLV_PCIE_0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 84,
};

static struct qcom_icc_node slv_pcie_1 = {
	.name = "slv_pcie_1",
	.id = MSM8994_SNOC_SLV_PCIE_1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 85,
};

static struct qcom_icc_node * const msm8994_snoc_nodes[] = {
	[SNOC_MAS_LPASS_AHB] = &mas_lpass_ahb,
	[SNOC_MAS_QDSS_BAM] = &mas_qdss_bam,
	[SNOC_TO_BIMC] = &snoc_to_bimc,
	[SNOC_TO_CNOC] = &snoc_to_cnoc,
	[SNOC_TO_PNOC] = &snoc_to_pnoc,
	[SNOC_TO_OCMEM_VNOC] = &snoc_to_ocmem_vnoc,
	[SNOC_MAS_CRYPTO_CORE0] = &mas_crypto_core0,
	[SNOC_MAS_CRYPTO_CORE1] = &mas_crypto_core1,
	[SNOC_MAS_CRYPTO_CORE2] = &mas_crypto_core2,
	[SNOC_MAS_LPASS_PROC] = &mas_lpass_proc,
	[SNOC_MAS_QDSS_ETR] = &mas_qdss_etr,
	[SNOC_MAS_USB3] = &mas_usb3,
	[SNOC_MAS_PCIE] = &mas_pcie_0,
	[SNOC_MAS_PCIE_1] = &mas_pcie_1,
	[SNOC_MAS_UFS] = &mas_ufs,
	[SNOC_MAS_IPA] = &mas_ipa,
	[SNOC_MAS_OVNOC] = &mas_ovirt_snoc,
	[SNOC_SLV_AMPSS] = &slv_ampss,
	[SNOC_SLV_LPASS] = &slv_lpass,
	[SNOC_SLV_USB3] = &slv_usb3,
	[SNOC_SLV_OCIMEM] = &slv_ocimem,
	[SNOC_SLV_QDSS_STM] = &slv_qdss_stm,
	[SNOC_SLV_PCIE_0] = &slv_pcie_0,
	[SNOC_SLV_PCIE_1] = &slv_pcie_1,
};

static const struct qcom_icc_desc msm8994_snoc = {
	.nodes = msm8994_snoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8994_snoc_nodes),
	.bus_clk_desc = &bus_1_clk,
	.get_bw = msm8994_get_bw,
	.ignore_enxio = true,
};

static const struct of_device_id msm8994_noc_of_match[] = {
	{ .compatible = "qcom,msm8994-bimc", .data = &msm8994_bimc },
	{ .compatible = "qcom,msm8994-cnoc", .data = &msm8994_cnoc },
	{ .compatible = "qcom,msm8994-mnoc", .data = &msm8994_mnoc },
	{ .compatible = "qcom,msm8994-ovirt", .data = &msm8994_ovirt },
	{ .compatible = "qcom,msm8994-pnoc", .data = &msm8994_pnoc },
	{ .compatible = "qcom,msm8994-snoc", .data = &msm8994_snoc },
	{ },
};
MODULE_DEVICE_TABLE(of, msm8994_noc_of_match);

static struct platform_driver msm8994_noc_driver = {
	.probe = qnoc_probe,
	.remove = qnoc_remove,
	.driver = {
		.name = "qnoc-msm8994",
		.of_match_table = msm8994_noc_of_match,
		.sync_state = icc_sync_state,
	},
};
module_platform_driver(msm8994_noc_driver);
MODULE_DESCRIPTION("Qualcomm MSM8994 NoC driver");
MODULE_LICENSE("GPL v2");
