/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SOC_QCOM_MSM8994_OXILI_H
#define __SOC_QCOM_MSM8994_OXILI_H

/*
 * MSM8994 A430: GX+CX must be voted before the GPU SMMU (tbu=gfx3d)
 * and before restore_sec_cfg(18). Implemented in mmcc-msm8994.
 */
int msm8994_oxili_pre_gpu_power(void);
int msm8994_oxili_pre_gpu_power_if_live(void);
void msm8994_oxili_mark_gpu_live(void);
bool msm8994_oxili_pre_gpu_voted(void);

#endif
