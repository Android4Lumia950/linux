/* SPDX-License-Identifier: (GPL-2.0 OR BSD-2-Clause) */
/*
 * Qualcomm MSM8994 interconnect IDs
 *
 * Install at include/dt-bindings/interconnect/qcom,msm8994.h
 * (replace any copy of qcom,msm8974.h that was left at that path).
 *
 * Copyright (c) 2022, The Linux Foundation. All rights reserved.
 */

#ifndef __DT_BINDINGS_INTERCONNECT_QCOM_MSM8994_H
#define __DT_BINDINGS_INTERCONNECT_QCOM_MSM8994_H

#define BIMC_MAS_AMPSS_M0		0
#define BIMC_MAS_GRAPHICS_3D		1
#define BIMC_TO_MNOC			2
#define BIMC_TO_SNOC			3
#define BIMC_SLV_EBI_CH0		4
#define BIMC_SLV_AMPSS_L2		5

#define CNOC_MAS_RPM_INST		0
#define CNOC_MAS_RPM_SYS		1
#define CNOC_MAS_DEHR			2
#define CNOC_MAS_QDSS_DAP		3
#define CNOC_MAS_SPDM			4
#define CNOC_MAS_TIC			5
#define CNOC_TO_SNOC			6
#define CNOC_SLV_CLK_CTL		7
#define CNOC_SLV_SECURITY		8
#define CNOC_SLV_TCSR			9
#define CNOC_SLV_TLMM			10
#define CNOC_SLV_CRYPTO_0_CFG		11
#define CNOC_SLV_CRYPTO_1_CFG		12
#define CNOC_SLV_CRYPTO_2_CFG		13
#define CNOC_SLV_IMEM_CFG		14
#define CNOC_SLV_MESSAGE_RAM		15
#define CNOC_SLV_BIMC_CFG		16
#define CNOC_SLV_BOOT_ROM		17
#define CNOC_SLV_PMIC_ARB		18
#define CNOC_SLV_SPDM_WRAPPER		19
#define CNOC_SLV_DEHR_CFG		20
#define CNOC_SLV_MPM			21
#define CNOC_SLV_QDSS_CFG		22
#define CNOC_SLV_RBCPR_CFG		23
#define CNOC_SLV_RBCPR_QDSS_APU_CFG	24
#define CNOC_SLV_CNOC_MNOC_MMSS_CFG	25
#define CNOC_SLV_CNOC_MNOC_CFG		26
#define CNOC_SLV_PNOC_CFG		27
#define CNOC_SLV_SNOC_MPU_CFG		28
#define CNOC_SLV_SNOC_CFG		29
#define CNOC_SLV_EBI1_DLL_CFG		30
#define CNOC_SLV_PHY_APU_CFG		31
#define CNOC_SLV_EBI1_PHY_CFG		32
#define CNOC_SLV_RPM			33
#define CNOC_SLV_PCIE_0_CFG		34
#define CNOC_SLV_PCIE_1_CFG		35
#define CNOC_SLV_GENI_IR_CFG		36
#define CNOC_SLV_UFS_CFG		37

#define MNOC_MAS_JPEG			0
#define MNOC_MAS_MDP_PORT0		1
#define MNOC_MAS_MDP_PORT1		2
#define MNOC_MAS_VIDEO_P0		3
#define MNOC_MAS_VIDEO_P1		4
#define MNOC_MAS_VFE			5
#define MNOC_MAS_CPP			6
#define MNOC_MAS_VPU			7
#define MNOC_MAS_CNOC_MMSS_CFG		8
#define MNOC_MAS_CNOC_CFG		9
#define MNOC_TO_BIMC			10
#define MNOC_SLV_CAMERA_CFG		11
#define MNOC_SLV_DISPLAY_CFG		12
#define MNOC_SLV_OCMEM_CFG		13
#define MNOC_SLV_CPR_CFG			14
#define MNOC_SLV_CPR_XPU_CFG		15
#define MNOC_SLV_MISC_CFG		16
#define MNOC_SLV_MISC_XPU_CFG		17
#define MNOC_SLV_VENUS_CFG		18
#define MNOC_SLV_GRAPHICS_3D_CFG		19
#define MNOC_SLV_MMSS_CLK_CFG		20
#define MNOC_SLV_MMSS_CLK_XPU_CFG	21
#define MNOC_SLV_MNOC_MPU_CFG		22
#define MNOC_SLV_AVSYNC_CFG		23
#define MNOC_SLV_VPU_CFG			24
#define MNOC_SLV_SERVICE_MNOC		25

#define OCMEM_MAS_SNOC			0
#define OCMEM_MAS_OCMEM_DMA		1
#define OCMEM_MAS_GFX3D			2
#define OCMEM_MAS_VIDEO_P0_OCMEM	3
#define OCMEM_SLV_OCMEM			4
#define OCMEM_SLV_OCMEM_GFX		5
#define OCMEM_TO_SNOC			6

#define PNOC_MAS_PNOC_CFG		0
#define PNOC_MAS_SDCC_1			1
#define PNOC_MAS_SDCC_3			2
#define PNOC_MAS_SDCC_4			3
#define PNOC_MAS_SDCC_2			4
#define PNOC_MAS_TSIF			5
#define PNOC_MAS_BAM_DMA			6
#define PNOC_MAS_BLSP_2			7
#define PNOC_MAS_BLSP_1			8
#define PNOC_MAS_USB_HS			9
#define PNOC_TO_SNOC			10
#define PNOC_SLV_SDCC_1			11
#define PNOC_SLV_SDCC_3			12
#define PNOC_SLV_SDCC_2			13
#define PNOC_SLV_SDCC_4			14
#define PNOC_SLV_TSIF			15
#define PNOC_SLV_BAM_DMA			16
#define PNOC_SLV_BLSP_2			17
#define PNOC_SLV_BLSP_1			18
#define PNOC_SLV_USB_HS			19
#define PNOC_SLV_PDM			20
#define PNOC_SLV_PRNG			21

#define SNOC_MAS_LPASS_AHB		0
#define SNOC_MAS_QDSS_BAM		1
#define SNOC_TO_BIMC			2
#define SNOC_TO_CNOC			3
#define SNOC_TO_PNOC			4
#define SNOC_TO_OCMEM_VNOC		5
#define SNOC_MAS_CRYPTO_CORE0		6
#define SNOC_MAS_CRYPTO_CORE1		7
#define SNOC_MAS_CRYPTO_CORE2		8
#define SNOC_MAS_LPASS_PROC		9
#define SNOC_MAS_QDSS_ETR		10
#define SNOC_MAS_USB3			11
#define SNOC_MAS_PCIE			12
#define SNOC_MAS_PCIE_1			13
#define SNOC_MAS_UFS			14
#define SNOC_MAS_IPA			15
#define SNOC_MAS_OVNOC			16
#define SNOC_SLV_AMPSS			17
#define SNOC_SLV_LPASS			18
#define SNOC_SLV_USB3			19
#define SNOC_SLV_OCIMEM			20
#define SNOC_SLV_QDSS_STM		21
#define SNOC_SLV_PCIE_0			22
#define SNOC_SLV_PCIE_1			23

#endif
