// SPDX-License-Identifier: GPL-2.0-only
/*
 * IOMMU API for QCOM secure IOMMUs.  Somewhat based on arm-smmu.c
 *
 * Copyright (C) 2013 ARM Limited
 * Copyright (C) 2017 Red Hat
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/io-64-nonatomic-hi-lo.h>
#include <linux/io-pgtable.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/kconfig.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "arm-smmu.h"

#define SMMU_INTR_SEL_NS     0x2000

/* GR1 sits one 4K page above GR0 on the msm8974 QSMMU */
#define QCOM_IOMMU_GR1			0x1000

/* The implementation-defined space sits two 4K pages above GR0 */
#define QCOM_IOMMU_IMPL_DEF		0x2000

/* Micro-MMU control, at the start of the implementation-defined space */
#define QCOM_IOMMU_MICRO_MMU_CTRL	(QCOM_IOMMU_IMPL_DEF + 0x000)
#define MICRO_MMU_CTRL_HALT_REQ		BIT(2)
#define MICRO_MMU_CTRL_IDLE		BIT(3)

/* Redirect all cacheable requests to the L2 slave port */
#define QCOM_IOMMU_ACTLR_BPRC		(BIT(28) | BIT(29) | BIT(30))

enum qcom_iommu_clk {
	CLK_IFACE,
	CLK_BUS,
	CLK_TBU,
	CLK_NUM,
};

struct qcom_iommu_ctx;

struct qcom_iommu_bfb_reg {
	u32			 offset;
	u32			 value;
};

struct qcom_iommu_sid {
	u8			 cbndx;
	u8			 sid;
};

/*
 * Per-instance configuration for instances whose global register space
 * is at least partially OS-managed (reg points at the global space and
 * SMMU_INTR_SEL_NS must not be written). The stream ID map is only used
 * on non-secured instances.
 */
struct qcom_iommu_cfg {
	enum io_pgtable_fmt		 fmt;
	/* the walker faults on the AF bit despite it being set */
	bool				 no_afe;
	/* context banks lose their state over GDSC power collapse */
	bool				 ctx_restore;
	/* the micro-MMU must be halted while its registers are programmed */
	bool				 halt;
	/* faulting transactions must terminate rather than stall */
	bool				 no_stall;
	const struct qcom_iommu_bfb_reg	*bfb;
	unsigned int			 num_bfb;
	const struct qcom_iommu_sid	*sids;	/* one SMR slot per entry */
	unsigned int			 num_sids;
};

struct qcom_iommu_dev {
	/* IOMMU core code handle */
	struct iommu_device	 iommu;
	struct device		*dev;
	const struct qcom_iommu_cfg *cfg;
	struct clk_bulk_data clks[CLK_NUM];
	void __iomem		*local_base;
	void __iomem		*global_base;
	u32			 sec_id;
	bool			 non_secure;
	u8			 max_asid;
	struct qcom_iommu_ctx	*ctxs[];   /* indexed by asid */
};

static enum io_pgtable_fmt qcom_iommu_pgtbl_fmt(struct qcom_iommu_dev *qcom_iommu)
{
	return qcom_iommu->cfg ? qcom_iommu->cfg->fmt : ARM_32_LPAE_S1;
}

struct qcom_iommu_ctx {
	struct device		*dev;
	void __iomem		*base;
	bool			 secure_init;
	bool			 secured_ctx;
	u8			 asid;      /* asid and ctx bank # are 1:1 */
	struct iommu_domain	*domain;
	/* CB state replayed after power collapse on OS-managed instances */
	u64			 ttbr0;
	u32			 tcr[2];
	u32			 mair[2];
	u32			 contextidr;
	u32			 sctlr;
};

struct qcom_iommu_domain {
	struct io_pgtable_ops	*pgtbl_ops;
	spinlock_t		 pgtbl_lock;
	struct mutex		 init_mutex; /* Protects iommu pointer */
	struct iommu_domain	 domain;
	struct qcom_iommu_dev	*iommu;
	struct iommu_fwspec	*fwspec;
};

static struct qcom_iommu_domain *to_qcom_iommu_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct qcom_iommu_domain, domain);
}

static const struct iommu_ops qcom_iommu_ops;

static struct qcom_iommu_ctx * to_ctx(struct qcom_iommu_domain *d, unsigned asid)
{
	struct qcom_iommu_dev *qcom_iommu = d->iommu;
	if (!qcom_iommu)
		return NULL;
	return qcom_iommu->ctxs[asid];
}

static inline void
iommu_writel(struct qcom_iommu_ctx *ctx, unsigned reg, u32 val)
{
	writel_relaxed(val, ctx->base + reg);
}

static inline void
iommu_writeq(struct qcom_iommu_ctx *ctx, unsigned reg, u64 val)
{
	writeq_relaxed(val, ctx->base + reg);
}

static inline u32
iommu_readl(struct qcom_iommu_ctx *ctx, unsigned reg)
{
	return readl_relaxed(ctx->base + reg);
}

static inline u64
iommu_readq(struct qcom_iommu_ctx *ctx, unsigned reg)
{
	return readq_relaxed(ctx->base + reg);
}

static inline void
qcom_iommu_gr0_write(struct qcom_iommu_dev *qcom_iommu, unsigned int reg,
		      u32 val)
{
	writel_relaxed(val, qcom_iommu->global_base + reg);
}

static inline u32
qcom_iommu_gr0_read(struct qcom_iommu_dev *qcom_iommu, unsigned int reg)
{
	return readl_relaxed(qcom_iommu->global_base + reg);
}

static inline void
qcom_iommu_gr1_write(struct qcom_iommu_dev *qcom_iommu, unsigned int reg,
		      u32 val)
{
	writel_relaxed(val, qcom_iommu->global_base + QCOM_IOMMU_GR1 + reg);
}

static void qcom_iommu_tlb_sync(void *cookie)
{
	struct qcom_iommu_domain *qcom_domain = cookie;
	struct iommu_fwspec *fwspec = qcom_domain->fwspec;
	unsigned i;

	for (i = 0; i < fwspec->num_ids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain, fwspec->ids[i]);
		unsigned int val, ret;

		iommu_writel(ctx, ARM_SMMU_CB_TLBSYNC, 0);

		ret = readl_poll_timeout(ctx->base + ARM_SMMU_CB_TLBSTATUS, val,
					 (val & 0x1) == 0, 0, 5000000);
		if (ret)
			dev_err(ctx->dev, "timeout waiting for TLB SYNC\n");
	}
}

static void qcom_iommu_tlb_inv_context(void *cookie)
{
	struct qcom_iommu_domain *qcom_domain = cookie;
	struct iommu_fwspec *fwspec = qcom_domain->fwspec;
	unsigned i;

	for (i = 0; i < fwspec->num_ids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain, fwspec->ids[i]);
		iommu_writel(ctx, ARM_SMMU_CB_S1_TLBIASID, ctx->asid);
	}

	qcom_iommu_tlb_sync(cookie);
}

static void qcom_iommu_tlb_inv_range_nosync(unsigned long iova, size_t size,
					    size_t granule, bool leaf, void *cookie)
{
	struct qcom_iommu_domain *qcom_domain = cookie;
	struct iommu_fwspec *fwspec = qcom_domain->fwspec;
	unsigned i, reg;

	reg = leaf ? ARM_SMMU_CB_S1_TLBIVAL : ARM_SMMU_CB_S1_TLBIVA;

	for (i = 0; i < fwspec->num_ids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain, fwspec->ids[i]);
		size_t s = size;

		iova = (iova >> 12) << 12;
		iova |= ctx->asid;
		do {
			iommu_writel(ctx, reg, iova);
			iova += granule;
		} while (s -= granule);
	}
}

static void qcom_iommu_tlb_flush_walk(unsigned long iova, size_t size,
				      size_t granule, void *cookie)
{
	qcom_iommu_tlb_inv_range_nosync(iova, size, granule, false, cookie);
	qcom_iommu_tlb_sync(cookie);
}

static void qcom_iommu_tlb_add_page(struct iommu_iotlb_gather *gather,
				    unsigned long iova, size_t granule,
				    void *cookie)
{
	qcom_iommu_tlb_inv_range_nosync(iova, granule, granule, true, cookie);
}

static const struct iommu_flush_ops qcom_flush_ops = {
	.tlb_flush_all	= qcom_iommu_tlb_inv_context,
	.tlb_flush_walk = qcom_iommu_tlb_flush_walk,
	.tlb_add_page	= qcom_iommu_tlb_add_page,
};

static irqreturn_t qcom_iommu_fault(int irq, void *dev)
{
	struct qcom_iommu_ctx *ctx = dev;
	u32 fsr, fsynr;
	u64 iova;

	fsr = iommu_readl(ctx, ARM_SMMU_CB_FSR);

	if (!(fsr & ARM_SMMU_CB_FSR_FAULT))
		return IRQ_NONE;

	fsynr = iommu_readl(ctx, ARM_SMMU_CB_FSYNR0);
	iova = iommu_readq(ctx, ARM_SMMU_CB_FAR);

	if (report_iommu_fault(ctx->domain, ctx->dev, iova, 0)) {
		dev_err_ratelimited(ctx->dev,
				    "Unhandled context fault: fsr=0x%x, "
				    "iova=0x%016llx, fsynr=0x%x, cb=%d\n",
				    fsr, iova, fsynr, ctx->asid);
	}

	iommu_writel(ctx, ARM_SMMU_CB_FSR, fsr);
	iommu_writel(ctx, ARM_SMMU_CB_RESUME, ARM_SMMU_RESUME_TERMINATE);

	return IRQ_HANDLED;
}

static void qcom_iommu_bfb_setup(struct qcom_iommu_dev *qcom_iommu)
{
	const struct qcom_iommu_cfg *cfg = qcom_iommu->cfg;
	unsigned int i;

	if (!cfg)
		return;

	/* The BFB registers sit in the implementation-defined space */
	for (i = 0; i < cfg->num_bfb; i++) {
		if (WARN_ON_ONCE(cfg->bfb[i].offset >= SZ_4K))
			continue;

		writel_relaxed(cfg->bfb[i].value,
			       qcom_iommu->global_base + QCOM_IOMMU_IMPL_DEF +
			       cfg->bfb[i].offset);
	}
}

/*
 * Reset and configure the global register space of an instance the
 * secure world does not manage: global fault state, TLB, stream
 * mapping (SMR/S2CR/CBAR) and the global configuration register.
 */
static int qcom_iommu_reset_ns(struct qcom_iommu_dev *qcom_iommu)
{
	const struct qcom_iommu_cfg *cfg = qcom_iommu->cfg;
	unsigned int i, num_smr;
	u32 reg;
	int ret;

	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sACR, 0);
	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sCR2, 0);
	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sGFAR, 0);
	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sGFAR + 4, 0);
	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sGFSRRESTORE, 0);

	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_TLBIALLNSNH, 0);
	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sTLBGSYNC, 0);
	ret = read_poll_timeout(qcom_iommu_gr0_read, reg,
				!(reg & ARM_SMMU_sTLBGSTATUS_GSACTIVE),
				0, 5000000, false,
				qcom_iommu, ARM_SMMU_GR0_sTLBGSTATUS);
	if (ret) {
		dev_err(qcom_iommu->dev,
			"timeout waiting for global TLB SYNC\n");
		return ret;
	}

	num_smr = FIELD_GET(ARM_SMMU_ID0_NUMSMRG,
			    qcom_iommu_gr0_read(qcom_iommu, ARM_SMMU_GR0_ID0));
	for (i = 0; i < num_smr; i++)
		qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_SMR(i), 0);

	for (i = 0; i < cfg->num_sids; i++) {
		const struct qcom_iommu_sid *sid = &cfg->sids[i];

		qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_SMR(i),
				      ARM_SMMU_SMR_VALID |
				      FIELD_PREP(ARM_SMMU_SMR_ID, sid->sid));
		qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_S2CR(i),
				      FIELD_PREP(ARM_SMMU_S2CR_TYPE, S2CR_TYPE_TRANS) |
				      FIELD_PREP(ARM_SMMU_S2CR_CBNDX, sid->cbndx) |
				      FIELD_PREP(ARM_SMMU_S2CR_MEMATTR, 0xa) |
				      FIELD_PREP(ARM_SMMU_S2CR_NSCFG, 3));
		qcom_iommu_gr1_write(qcom_iommu,
				      ARM_SMMU_GR1_CBAR(sid->cbndx),
				      FIELD_PREP(ARM_SMMU_CBAR_TYPE,
						 CBAR_TYPE_S1_TRANS_S2_BYPASS) |
				      FIELD_PREP(ARM_SMMU_CBAR_IRPTNDX, 1) |
				      FIELD_PREP(ARM_SMMU_CBAR_VMID, 3) |
				      FIELD_PREP(ARM_SMMU_CBAR_S1_BPSHCFG, 2) |
				      FIELD_PREP(ARM_SMMU_CBAR_S1_MEMATTR, 0xa));
	}

	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sCR0,
			      ARM_SMMU_sCR0_SMCFCFG | ARM_SMMU_sCR0_USFCFG |
			      ARM_SMMU_sCR0_STALLD | ARM_SMMU_sCR0_GCFGFIE |
			      ARM_SMMU_sCR0_GCFGFRE | ARM_SMMU_sCR0_GFIE |
			      ARM_SMMU_sCR0_GFRE);

	return 0;
}

/*
 * Halting the micro-MMU quiesces the translation front-end: it stops new
 * client transactions being accepted and waits for the outstanding ones to
 * retire, so that the context bank registers can be reprogrammed without
 * in-flight traffic racing the change.
 */
static void qcom_iommu_halt(struct qcom_iommu_dev *qcom_iommu)
{
	void __iomem *reg = qcom_iommu->global_base + QCOM_IOMMU_MICRO_MMU_CTRL;
	u32 val;

	if (!qcom_iommu->cfg || !qcom_iommu->cfg->halt)
		return;

	writel_relaxed(readl_relaxed(reg) | MICRO_MMU_CTRL_HALT_REQ, reg);

	if (readl_poll_timeout(reg, val, val & MICRO_MMU_CTRL_IDLE, 0, 100000))
		dev_err(qcom_iommu->dev, "timeout waiting for micro-MMU halt\n");
}

static void qcom_iommu_unhalt(struct qcom_iommu_dev *qcom_iommu)
{
	void __iomem *reg = qcom_iommu->global_base + QCOM_IOMMU_MICRO_MMU_CTRL;

	if (!qcom_iommu->cfg || !qcom_iommu->cfg->halt)
		return;

	writel_relaxed(readl_relaxed(reg) & ~MICRO_MMU_CTRL_HALT_REQ, reg);
}

static void qcom_iommu_program_ctx(struct qcom_iommu_dev *qcom_iommu,
				   struct qcom_iommu_ctx *ctx)
{
	qcom_iommu_halt(qcom_iommu);

	/* Disable context bank before programming */
	iommu_writel(ctx, ARM_SMMU_CB_SCTLR, 0);

	/* Clear context bank fault address fault status registers */
	iommu_writel(ctx, ARM_SMMU_CB_FAR, 0);
	iommu_writel(ctx, ARM_SMMU_CB_FSR, ARM_SMMU_CB_FSR_FAULT);

	if (qcom_iommu->cfg)
		iommu_writel(ctx, ARM_SMMU_CB_ACTLR, QCOM_IOMMU_ACTLR_BPRC);

	/* TTBRs */
	iommu_writeq(ctx, ARM_SMMU_CB_TTBR0, ctx->ttbr0);
	iommu_writeq(ctx, ARM_SMMU_CB_TTBR1, 0);

	/* TCR; the v7s ASID lives in CONTEXTIDR instead of TTBR0 */
	if (qcom_iommu_pgtbl_fmt(qcom_iommu) == ARM_V7S)
		iommu_writel(ctx, ARM_SMMU_CB_CONTEXTIDR, ctx->contextidr);
	else
		iommu_writel(ctx, ARM_SMMU_CB_TCR2, ctx->tcr[1]);
	iommu_writel(ctx, ARM_SMMU_CB_TCR, ctx->tcr[0]);

	/* MAIRs (stage-1 only) */
	iommu_writel(ctx, ARM_SMMU_CB_S1_MAIR0, ctx->mair[0]);
	iommu_writel(ctx, ARM_SMMU_CB_S1_MAIR1, ctx->mair[1]);

	/* SCTLR */
	iommu_writel(ctx, ARM_SMMU_CB_SCTLR, ctx->sctlr);

	qcom_iommu_unhalt(qcom_iommu);
}

static int qcom_iommu_init_domain(struct iommu_domain *domain,
				  struct qcom_iommu_dev *qcom_iommu,
				  struct device *dev)
{
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct io_pgtable_ops *pgtbl_ops;
	struct io_pgtable_cfg pgtbl_cfg;
	enum io_pgtable_fmt fmt;
	int i, ret = 0;
	u32 reg;

	mutex_lock(&qcom_domain->init_mutex);
	if (qcom_domain->iommu)
		goto out_unlock;

	fmt = qcom_iommu_pgtbl_fmt(qcom_iommu);

	pgtbl_cfg = (struct io_pgtable_cfg) {
		.pgsize_bitmap	= domain->pgsize_bitmap,
		.ias		= 32,
		.oas		= fmt == ARM_V7S ? 32 : 40,
		.tlb		= &qcom_flush_ops,
		.iommu_dev	= qcom_iommu->dev,
	};

	qcom_domain->iommu = qcom_iommu;
	qcom_domain->fwspec = fwspec;

	pgtbl_ops = alloc_io_pgtable_ops(fmt, &pgtbl_cfg, qcom_domain);
	if (!pgtbl_ops) {
		dev_err(qcom_iommu->dev, "failed to allocate pagetable ops\n");
		ret = -ENOMEM;
		goto out_clear_iommu;
	}

	domain->geometry.aperture_end = (1ULL << pgtbl_cfg.ias) - 1;
	domain->geometry.force_aperture = true;

	for (i = 0; i < fwspec->num_ids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain, fwspec->ids[i]);

		if (!qcom_iommu->non_secure && !ctx->secure_init) {
			ret = qcom_scm_restore_sec_cfg(qcom_iommu->sec_id, ctx->asid);
			if (ret) {
				dev_err(qcom_iommu->dev, "secure init failed: %d\n", ret);
				goto out_clear_iommu;
			}
			ctx->secure_init = true;
		}

		/* Secured QSMMU-500/QSMMU-v2 contexts cannot be programmed */
		if (ctx->secured_ctx) {
			ctx->domain = domain;
			continue;
		}

		if (fmt == ARM_V7S) {
			ctx->ttbr0 = pgtbl_cfg.arm_v7s_cfg.ttbr;
			ctx->tcr[0] = pgtbl_cfg.arm_v7s_cfg.tcr;
			ctx->tcr[1] = 0;
			/* PRRR/NMRR share the MAIR0/MAIR1 offsets */
			ctx->mair[0] = pgtbl_cfg.arm_v7s_cfg.prrr;
			ctx->mair[1] = pgtbl_cfg.arm_v7s_cfg.nmrr;
			ctx->contextidr = ctx->asid;
		} else {
			ctx->ttbr0 = pgtbl_cfg.arm_lpae_s1_cfg.ttbr |
				     FIELD_PREP(ARM_SMMU_TTBRn_ASID, ctx->asid);
			ctx->tcr[0] = arm_smmu_lpae_tcr(&pgtbl_cfg) | ARM_SMMU_TCR_EAE;
			ctx->tcr[1] = arm_smmu_lpae_tcr2(&pgtbl_cfg);
			ctx->mair[0] = pgtbl_cfg.arm_lpae_s1_cfg.mair;
			ctx->mair[1] = pgtbl_cfg.arm_lpae_s1_cfg.mair >> 32;
		}

		reg = ARM_SMMU_SCTLR_CFIE | ARM_SMMU_SCTLR_CFRE |
		      ARM_SMMU_SCTLR_AFE | ARM_SMMU_SCTLR_TRE |
		      ARM_SMMU_SCTLR_M | ARM_SMMU_SCTLR_S1_ASIDPNE |
		      ARM_SMMU_SCTLR_CFCFG;

		if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
			reg |= ARM_SMMU_SCTLR_E;

		if (qcom_iommu->cfg && qcom_iommu->cfg->no_afe)
			reg &= ~ARM_SMMU_SCTLR_AFE;

		if (qcom_iommu->cfg && qcom_iommu->cfg->no_stall)
			reg &= ~ARM_SMMU_SCTLR_CFCFG;

		ctx->sctlr = reg;

		qcom_iommu_program_ctx(qcom_iommu, ctx);

		ctx->domain = domain;
	}

	/* Publish page table ops for map/unmap */
	qcom_domain->pgtbl_ops = pgtbl_ops;

	mutex_unlock(&qcom_domain->init_mutex);

	return 0;
out_clear_iommu:
	free_io_pgtable_ops(pgtbl_ops);
	qcom_domain->iommu = NULL;
out_unlock:
	mutex_unlock(&qcom_domain->init_mutex);
	return ret;
}

static struct iommu_domain *qcom_iommu_domain_alloc_paging(struct device *dev)
{
	struct qcom_iommu_domain *qcom_domain;

	/*
	 * Allocate the domain and initialise some of its data structures.
	 * We can't really do anything meaningful until we've added a
	 * master.
	 */
	qcom_domain = kzalloc_obj(*qcom_domain);
	if (!qcom_domain)
		return NULL;

	mutex_init(&qcom_domain->init_mutex);
	spin_lock_init(&qcom_domain->pgtbl_lock);
	qcom_domain->domain.pgsize_bitmap = SZ_4K;

	return &qcom_domain->domain;
}

static void qcom_iommu_domain_free(struct iommu_domain *domain)
{
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);

	if (qcom_domain->iommu) {
		/*
		 * NOTE: unmap can be called after client device is powered
		 * off, for example, with GPUs or anything involving dma-buf.
		 * So we cannot rely on the device_link.  Make sure the IOMMU
		 * is on to avoid unclocked accesses in the TLB inv path:
		 */
		pm_runtime_get_sync(qcom_domain->iommu->dev);
		free_io_pgtable_ops(qcom_domain->pgtbl_ops);
		pm_runtime_put_sync(qcom_domain->iommu->dev);
	}

	kfree(qcom_domain);
}

static int qcom_iommu_attach_dev(struct iommu_domain *domain,
				 struct device *dev, struct iommu_domain *old)
{
	struct qcom_iommu_dev *qcom_iommu = dev_iommu_priv_get(dev);
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	int ret;

	if (!qcom_iommu) {
		dev_err(dev, "cannot attach to IOMMU, is it on the same bus?\n");
		return -ENXIO;
	}

	/* Ensure that the domain is finalized */
	pm_runtime_get_sync(qcom_iommu->dev);
	ret = qcom_iommu_init_domain(domain, qcom_iommu, dev);
	pm_runtime_put_sync(qcom_iommu->dev);
	if (ret < 0)
		return ret;

	/*
	 * Sanity check the domain. We don't support domains across
	 * different IOMMUs.
	 */
	if (qcom_domain->iommu != qcom_iommu)
		return -EINVAL;

	return 0;
}

static int qcom_iommu_identity_attach(struct iommu_domain *identity_domain,
				      struct device *dev,
				      struct iommu_domain *old)
{
	struct qcom_iommu_domain *qcom_domain;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct qcom_iommu_dev *qcom_iommu = dev_iommu_priv_get(dev);
	unsigned int i;

	if (old == identity_domain || !old)
		return 0;

	qcom_domain = to_qcom_iommu_domain(old);
	if (WARN_ON(!qcom_domain->iommu))
		return -EINVAL;

	pm_runtime_get_sync(qcom_iommu->dev);
	for (i = 0; i < fwspec->num_ids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain, fwspec->ids[i]);

		/* Disable the context bank: */
		iommu_writel(ctx, ARM_SMMU_CB_SCTLR, 0);

		ctx->domain = NULL;
	}
	pm_runtime_put_sync(qcom_iommu->dev);
	return 0;
}

static struct iommu_domain_ops qcom_iommu_identity_ops = {
	.attach_dev = qcom_iommu_identity_attach,
};

static struct iommu_domain qcom_iommu_identity_domain = {
	.type = IOMMU_DOMAIN_IDENTITY,
	.ops = &qcom_iommu_identity_ops,
};

static int qcom_iommu_map(struct iommu_domain *domain, unsigned long iova,
			  phys_addr_t paddr, size_t pgsize, size_t pgcount,
			  int prot, gfp_t gfp, size_t *mapped)
{
	int ret;
	unsigned long flags;
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct io_pgtable_ops *ops = qcom_domain->pgtbl_ops;

	if (!ops)
		return -ENODEV;

	spin_lock_irqsave(&qcom_domain->pgtbl_lock, flags);
	ret = ops->map_pages(ops, iova, paddr, pgsize, pgcount, prot, GFP_ATOMIC, mapped);
	spin_unlock_irqrestore(&qcom_domain->pgtbl_lock, flags);
	return ret;
}

static size_t qcom_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
			       size_t pgsize, size_t pgcount,
			       struct iommu_iotlb_gather *gather)
{
	size_t ret;
	unsigned long flags;
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct io_pgtable_ops *ops = qcom_domain->pgtbl_ops;

	if (!ops)
		return 0;

	/* NOTE: unmap can be called after client device is powered off,
	 * for example, with GPUs or anything involving dma-buf.  So we
	 * cannot rely on the device_link.  Make sure the IOMMU is on to
	 * avoid unclocked accesses in the TLB inv path:
	 */
	pm_runtime_get_sync(qcom_domain->iommu->dev);
	spin_lock_irqsave(&qcom_domain->pgtbl_lock, flags);
	ret = ops->unmap_pages(ops, iova, pgsize, pgcount, gather);
	spin_unlock_irqrestore(&qcom_domain->pgtbl_lock, flags);
	pm_runtime_put_sync(qcom_domain->iommu->dev);

	return ret;
}

static void qcom_iommu_flush_iotlb_all(struct iommu_domain *domain)
{
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct io_pgtable *pgtable = container_of(qcom_domain->pgtbl_ops,
						  struct io_pgtable, ops);
	if (!qcom_domain->pgtbl_ops)
		return;

	pm_runtime_get_sync(qcom_domain->iommu->dev);
	qcom_iommu_tlb_sync(pgtable->cookie);
	pm_runtime_put_sync(qcom_domain->iommu->dev);
}

static void qcom_iommu_iotlb_sync(struct iommu_domain *domain,
				  struct iommu_iotlb_gather *gather)
{
	qcom_iommu_flush_iotlb_all(domain);
}

static phys_addr_t qcom_iommu_iova_to_phys(struct iommu_domain *domain,
					   dma_addr_t iova)
{
	phys_addr_t ret;
	unsigned long flags;
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct io_pgtable_ops *ops = qcom_domain->pgtbl_ops;

	if (!ops)
		return 0;

	spin_lock_irqsave(&qcom_domain->pgtbl_lock, flags);
	ret = ops->iova_to_phys(ops, iova);
	spin_unlock_irqrestore(&qcom_domain->pgtbl_lock, flags);

	return ret;
}

static bool qcom_iommu_capable(struct device *dev, enum iommu_cap cap)
{
	switch (cap) {
	case IOMMU_CAP_CACHE_COHERENCY:
		/*
		 * Return true here as the SMMU can always send out coherent
		 * requests.
		 */
		return true;
	case IOMMU_CAP_NOEXEC:
		return true;
	default:
		return false;
	}
}

static struct iommu_device *qcom_iommu_probe_device(struct device *dev)
{
	struct qcom_iommu_dev *qcom_iommu = dev_iommu_priv_get(dev);
	struct device_link *link;

	if (!qcom_iommu)
		return ERR_PTR(-ENODEV);

	/*
	 * Establish the link between iommu and master, so that the
	 * iommu gets runtime enabled/disabled as per the master's
	 * needs.
	 */
	link = device_link_add(dev, qcom_iommu->dev, DL_FLAG_PM_RUNTIME);
	if (!link) {
		dev_err(qcom_iommu->dev, "Unable to create device link between %s and %s\n",
			dev_name(qcom_iommu->dev), dev_name(dev));
		return ERR_PTR(-ENODEV);
	}

	return &qcom_iommu->iommu;
}

static int qcom_iommu_of_xlate(struct device *dev,
			       const struct of_phandle_args *args)
{
	struct qcom_iommu_dev *qcom_iommu;
	struct platform_device *iommu_pdev;
	unsigned asid = args->args[0];

	if (args->args_count != 1) {
		dev_err(dev, "incorrect number of iommu params found for %s "
			"(found %d, expected 1)\n",
			args->np->full_name, args->args_count);
		return -EINVAL;
	}

	iommu_pdev = of_find_device_by_node(args->np);
	if (WARN_ON(!iommu_pdev))
		return -EINVAL;

	qcom_iommu = platform_get_drvdata(iommu_pdev);

	put_device(&iommu_pdev->dev);

	/* make sure the asid specified in dt is valid, so we don't have
	 * to sanity check this elsewhere:
	 */
	if (WARN_ON(asid > qcom_iommu->max_asid) ||
	    WARN_ON(qcom_iommu->ctxs[asid] == NULL))
		return -EINVAL;

	if (!dev_iommu_priv_get(dev)) {
		dev_iommu_priv_set(dev, qcom_iommu);
	} else {
		/* make sure devices iommus dt node isn't referring to
		 * multiple different iommu devices.  Multiple context
		 * banks are ok, but multiple devices are not:
		 */
		if (WARN_ON(qcom_iommu != dev_iommu_priv_get(dev)))
			return -EINVAL;
	}

	return iommu_fwspec_add_ids(dev, &asid, 1);
}

static const struct iommu_ops qcom_iommu_ops = {
	.identity_domain = &qcom_iommu_identity_domain,
	.capable	= qcom_iommu_capable,
	.domain_alloc_paging = qcom_iommu_domain_alloc_paging,
	.probe_device	= qcom_iommu_probe_device,
	.device_group	= generic_device_group,
	.of_xlate	= qcom_iommu_of_xlate,
	.default_domain_ops = &(const struct iommu_domain_ops) {
		.attach_dev	= qcom_iommu_attach_dev,
		.map_pages	= qcom_iommu_map,
		.unmap_pages	= qcom_iommu_unmap,
		.flush_iotlb_all = qcom_iommu_flush_iotlb_all,
		.iotlb_sync	= qcom_iommu_iotlb_sync,
		.iova_to_phys	= qcom_iommu_iova_to_phys,
		.free		= qcom_iommu_domain_free,
	}
};

static int qcom_iommu_sec_ptbl_init(struct device *dev)
{
	size_t psize = 0;
	unsigned int spare = 0;
	void *cpu_addr;
	dma_addr_t paddr;
	unsigned long attrs;
	static bool allocated = false;
	int ret;

	if (allocated)
		return 0;

	ret = qcom_scm_iommu_secure_ptbl_size(spare, &psize);
	if (ret) {
		dev_err(dev, "failed to get iommu secure pgtable size (%d)\n",
			ret);
		return ret;
	}

	dev_info(dev, "iommu sec: pgtable size: %zu\n", psize);

	attrs = DMA_ATTR_NO_KERNEL_MAPPING;

	cpu_addr = dma_alloc_attrs(dev, psize, &paddr, GFP_KERNEL, attrs);
	if (!cpu_addr) {
		dev_err(dev, "failed to allocate %zu bytes for pgtable\n",
			psize);
		return -ENOMEM;
	}

	ret = qcom_scm_iommu_secure_ptbl_init(paddr, psize, spare);
	if (ret) {
		dev_err(dev, "failed to init iommu pgtable (%d)\n", ret);
		goto free_mem;
	}

	allocated = true;
	return 0;

free_mem:
	dma_free_attrs(dev, psize, cpu_addr, paddr, attrs);
	return ret;
}

static int get_asid(const struct device_node *np)
{
	u32 reg, val;
	int asid;

	/* read the "reg" property directly to get the relative address
	 * of the context bank, and calculate the asid from that:
	 */
	if (of_property_read_u32_index(np, "reg", 0, &reg))
		return -ENODEV;

	/*
	 * Context banks are 0x1000 apart but, in some cases, the ASID
	 * number doesn't match to this logic and needs to be passed
	 * from the DT configuration explicitly.
	 */
	if (!of_property_read_u32(np, "qcom,ctx-asid", &val))
		asid = val;
	else
		asid = reg / 0x1000;

	return asid;
}

static bool qcom_iommu_mdp_mapafter(const struct device *parent)
{
	return of_device_is_compatible(parent->of_node,
				       "qcom,msm8974-mdp-iommu") ||
	       of_device_is_compatible(parent->of_node,
				       "qcom,msm8992-mdp-iommu") ||
	       of_device_is_compatible(parent->of_node,
				       "qcom,msm8994-mdp-iommu");
}

static int qcom_iommu_ctx_probe(struct platform_device *pdev)
{
	struct qcom_iommu_ctx *ctx;
	struct device *dev = &pdev->dev;
	struct qcom_iommu_dev *qcom_iommu = dev_get_drvdata(dev->parent);
	int ret, irq;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	platform_set_drvdata(pdev, ctx);

	/*
	 * MDP QSMMU v1 SoC-resets if the context bank is ioremapped
	 * before the parent is runtime-resumed (MDSS GDSC + clocks).
	 * MDP ctx is a child of the MDP SMMU. Map after parent resume
	 * so the parent's clocks/GDSC are on. GPU/Venus ctxs do not
	 * need this.
	 */
	if (qcom_iommu_mdp_mapafter(dev->parent)) {
		dev_info(dev, "MDP ctx map after parent resume\n");
		ret = pm_runtime_resume_and_get(dev->parent);
		if (ret)
			return ret;
	}

	ctx->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctx->base)) {
		if (qcom_iommu_mdp_mapafter(dev->parent))
			pm_runtime_put(dev->parent);
		return PTR_ERR(ctx->base);
	}

	if (qcom_iommu_mdp_mapafter(dev->parent))
		pm_runtime_put(dev->parent);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	if (of_device_is_compatible(dev->of_node, "qcom,msm-iommu-v2-sec"))
		ctx->secured_ctx = true;

	/* clear IRQs before registering fault handler, just in case the
	 * boot-loader left us a surprise.  Instances with a power domain
	 * may not be accessible yet; they are reset at first resume.
	 */
	if (!ctx->secured_ctx && !qcom_iommu->cfg) {
		ret = pm_runtime_resume_and_get(dev->parent);
		if (ret)
			return ret;
		iommu_writel(ctx, ARM_SMMU_CB_FSR, iommu_readl(ctx, ARM_SMMU_CB_FSR));
		pm_runtime_put_sync(dev->parent);
	}

	ret = devm_request_irq(dev, irq,
			       qcom_iommu_fault,
			       IRQF_SHARED,
			       "qcom-iommu-fault",
			       ctx);
	if (ret)
		return ret;

	ret = get_asid(dev->of_node);
	if (ret < 0) {
		dev_err(dev, "missing reg property\n");
		return ret;
	}

	ctx->asid = ret;

	dev_dbg(dev, "found asid %u\n", ctx->asid);

	qcom_iommu->ctxs[ctx->asid] = ctx;

	return 0;
}

static void qcom_iommu_ctx_remove(struct platform_device *pdev)
{
	struct qcom_iommu_dev *qcom_iommu = dev_get_drvdata(pdev->dev.parent);
	struct qcom_iommu_ctx *ctx = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	qcom_iommu->ctxs[ctx->asid] = NULL;
}

static const struct of_device_id ctx_of_match[] = {
	{ .compatible = "qcom,msm-iommu-v1-ns" },
	{ .compatible = "qcom,msm-iommu-v1-sec" },
	{ .compatible = "qcom,msm-iommu-v2-ns" },
	{ .compatible = "qcom,msm-iommu-v2-sec" },
	{ /* sentinel */ }
};

static struct platform_driver qcom_iommu_ctx_driver = {
	.driver	= {
		.name		= "qcom-iommu-ctx",
		.of_match_table	= ctx_of_match,
	},
	.probe	= qcom_iommu_ctx_probe,
	.remove = qcom_iommu_ctx_remove,
};

static bool qcom_iommu_has_secure_context(struct qcom_iommu_dev *qcom_iommu)
{
	for_each_child_of_node_scoped(qcom_iommu->dev->of_node, child) {
		if (of_device_is_compatible(child, "qcom,msm-iommu-v1-sec") ||
		    of_device_is_compatible(child, "qcom,msm-iommu-v2-sec"))
			return true;
	}

	return false;
}

static int qcom_iommu_device_probe(struct platform_device *pdev)
{
	struct device_node *child;
	struct qcom_iommu_dev *qcom_iommu;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct clk *clk;
	int ret, max_asid = 0;

	/* find the max asid (which is 1:1 to ctx bank idx), so we know how
	 * many child ctx devices we have:
	 */
	for_each_child_of_node(dev->of_node, child)
		max_asid = max(max_asid, get_asid(child));

	qcom_iommu = devm_kzalloc(dev, struct_size(qcom_iommu, ctxs, max_asid + 1),
				  GFP_KERNEL);
	if (!qcom_iommu)
		return -ENOMEM;
	qcom_iommu->max_asid = max_asid;
	qcom_iommu->dev = dev;
	qcom_iommu->cfg = of_device_get_match_data(dev);
	if (of_device_is_compatible(dev->of_node, "qcom,msm8992-mdp-iommu"))
		dev_info(dev, "talkman-iommu: 8992 mdp V7S BFB\n");
	if (of_device_is_compatible(dev->of_node, "qcom,msm8994-mdp-iommu"))
		dev_info(dev, "talkman-iommu: 8994 mdp V7S BFB\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		if (qcom_iommu->cfg) {
			qcom_iommu->global_base = devm_ioremap_resource(dev, res);
			if (IS_ERR(qcom_iommu->global_base))
				return PTR_ERR(qcom_iommu->global_base);
		} else {
			qcom_iommu->local_base = devm_ioremap_resource(dev, res);
			if (IS_ERR(qcom_iommu->local_base))
				return PTR_ERR(qcom_iommu->local_base);
		}
	} else if (qcom_iommu->cfg) {
		return dev_err_probe(dev, -EINVAL,
				     "missing global register space\n");
	}

	clk = devm_clk_get(dev, "iface");
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get iface clock\n");
		return PTR_ERR(clk);
	}
	qcom_iommu->clks[CLK_IFACE].clk = clk;

	clk = devm_clk_get(dev, "bus");
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get bus clock\n");
		return PTR_ERR(clk);
	}
	qcom_iommu->clks[CLK_BUS].clk = clk;

	clk = devm_clk_get_optional(dev, "tbu");
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get tbu clock\n");
		return PTR_ERR(clk);
	}
	qcom_iommu->clks[CLK_TBU].clk = clk;

	if (of_property_read_u32(dev->of_node, "qcom,iommu-secure-id",
				 &qcom_iommu->sec_id)) {
		if (!qcom_iommu->cfg) {
			dev_err(dev, "missing qcom,iommu-secure-id property\n");
			return -ENODEV;
		}
		/* The secure world does not manage this instance at all */
		qcom_iommu->non_secure = true;
	}

	if (qcom_iommu_has_secure_context(qcom_iommu)) {
		ret = qcom_iommu_sec_ptbl_init(dev);
		if (ret) {
			dev_err(dev, "cannot init secure pg table(%d)\n", ret);
			return ret;
		}
	}

	platform_set_drvdata(pdev, qcom_iommu);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	/* register context bank devices, which are child nodes: */
	ret = devm_of_platform_populate(dev);
	if (ret) {
		dev_err(dev, "Failed to populate iommu contexts\n");
		return ret;
	}

	ret = iommu_device_sysfs_add(&qcom_iommu->iommu, dev, NULL,
				     dev_name(dev));
	if (ret) {
		dev_err(dev, "Failed to register iommu in sysfs\n");
		return ret;
	}

	ret = iommu_device_register(&qcom_iommu->iommu, &qcom_iommu_ops, dev);
	if (ret) {
		dev_err(dev, "Failed to register iommu\n");
		goto err_sysfs_remove;
	}

	if (qcom_iommu->local_base) {
		ret = pm_runtime_resume_and_get(dev);
		if (ret)
			goto err_iommu_unregister;
		writel_relaxed(0xffffffff, qcom_iommu->local_base + SMMU_INTR_SEL_NS);
		pm_runtime_put_sync(dev);
	}

	return 0;

err_iommu_unregister:
	iommu_device_unregister(&qcom_iommu->iommu);
err_sysfs_remove:
	iommu_device_sysfs_remove(&qcom_iommu->iommu);
	return ret;
}

static void qcom_iommu_device_remove(struct platform_device *pdev)
{
	struct qcom_iommu_dev *qcom_iommu = platform_get_drvdata(pdev);

	pm_runtime_force_suspend(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	iommu_device_sysfs_remove(&qcom_iommu->iommu);
	iommu_device_unregister(&qcom_iommu->iommu);
}

static int __maybe_unused qcom_iommu_resume(struct device *dev)
{
	struct qcom_iommu_dev *qcom_iommu = dev_get_drvdata(dev);
	unsigned int i;
	int ret;

	ret = clk_bulk_prepare_enable(CLK_NUM, qcom_iommu->clks);
	if (ret < 0)
		return ret;

	if (qcom_iommu->non_secure) {
		ret = qcom_iommu_reset_ns(qcom_iommu);
		if (ret)
			return ret;
	} else if (dev->pm_domain) {
		ret = qcom_scm_restore_sec_cfg(qcom_iommu->sec_id, 0);
		if (ret)
			return ret;
	}

	qcom_iommu_bfb_setup(qcom_iommu);

	if (qcom_iommu->cfg && qcom_iommu->cfg->ctx_restore) {
		/* Restore context banks lost over power collapse */
		for (i = 0; i <= qcom_iommu->max_asid; i++) {
			struct qcom_iommu_ctx *ctx = qcom_iommu->ctxs[i];

			if (ctx && ctx->domain && !ctx->secured_ctx)
				qcom_iommu_program_ctx(qcom_iommu, ctx);
		}
	}

	return ret;
}

static int __maybe_unused qcom_iommu_suspend(struct device *dev)
{
	struct qcom_iommu_dev *qcom_iommu = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(CLK_NUM, qcom_iommu->clks);

	return 0;
}

static const struct dev_pm_ops qcom_iommu_pm_ops = {
	SET_RUNTIME_PM_OPS(qcom_iommu_suspend, qcom_iommu_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static const struct qcom_iommu_bfb_reg msm8974_gpu_bfb[] = {
	{ 0x04c, 0x00000003 },
	{ 0x050, 0x00000000 },
	{ 0x514, 0x00000004 },
	{ 0x540, 0x00000010 },
	{ 0x56c, 0x00000000 },
	{ 0x0ac, 0x00000000 },
	{ 0x15c, 0x00000000 },
	{ 0x20c, 0x00000020 },
	{ 0x314, 0x00000000 },
	{ 0x394, 0x00000001 },
	{ 0x414, 0x00000081 },
	{ 0x008, 0x00000000 },
};

static const struct qcom_iommu_sid msm8974_gpu_sids[] = {
	{ .cbndx = 0, .sid = 0 },	/* GFX3D_USER */
	{ .cbndx = 1, .sid = 1 },	/* GFX3D_PRIV */
};

static const struct qcom_iommu_cfg msm8974_gpu_cfg = {
	.no_stall = true,
	.fmt = ARM_V7S,
	.no_afe = true,
	.ctx_restore = true,
	.bfb = msm8974_gpu_bfb,
	.num_bfb = ARRAY_SIZE(msm8974_gpu_bfb),
	.sids = msm8974_gpu_sids,
	.num_sids = ARRAY_SIZE(msm8974_gpu_sids),
};

static const struct qcom_iommu_bfb_reg msm8974_mdp_bfb[] = {
	{ 0x04c, 0xffffffff },
	{ 0x050, 0x00000000 },
	{ 0x514, 0x00000004 },
	{ 0x540, 0x00000010 },
	{ 0x56c, 0x00000000 },
	{ 0x0ac, 0x00006800 },
	{ 0x15c, 0x00006221 },
	{ 0x20c, 0x00016231 },
	{ 0x314, 0x00000000 },
	{ 0x394, 0x00000034 },
	{ 0x414, 0x00000074 },
	{ 0x008, 0x00000000 },
	{ 0x00c, 0x00000000 },
	{ 0x010, 0x00000000 },
	{ 0x014, 0x00000000 },
	{ 0x018, 0x00000000 },
	{ 0x01c, 0x00000000 },
	{ 0x020, 0x00000000 },
};

static const struct qcom_iommu_cfg msm8974_mdp_cfg = {
	.halt = true,
	.no_stall = true,
	.fmt = ARM_V7S,
	.no_afe = true,
	.ctx_restore = true,
	.bfb = msm8974_mdp_bfb,
	.num_bfb = ARRAY_SIZE(msm8974_mdp_bfb),
};

static const struct qcom_iommu_bfb_reg msm8974_venus_bfb[] = {
	{ 0x04c, 0xffffffff },
	{ 0x050, 0xffffffff },
	{ 0x514, 0x00000004 },
	{ 0x540, 0x00000008 },
	{ 0x56c, 0x00000000 },
	{ 0x0ac, 0x00013205 },
	{ 0x15c, 0x00004000 },
	{ 0x20c, 0x00014020 },
	{ 0x314, 0x00000000 },
	{ 0x394, 0x00000094 },
	{ 0x414, 0x00000114 },
	{ 0x008, 0x00000000 },
	{ 0x00c, 0x00000000 },
	{ 0x010, 0x00000000 },
	{ 0x014, 0x00000000 },
	{ 0x018, 0x00000000 },
	{ 0x01c, 0x00000000 },
	{ 0x020, 0x00000000 },
	{ 0x024, 0x00000000 },
	{ 0x028, 0x00000000 },
	{ 0x02c, 0x00000000 },
	{ 0x030, 0x00000000 },
	{ 0x034, 0x00000000 },
	{ 0x038, 0x00000000 },
};

static const struct qcom_iommu_cfg msm8974_venus_cfg = {
	.halt = true,
	.no_stall = true,
	.fmt = ARM_V7S,
	.no_afe = true,
	.ctx_restore = true,
	.bfb = msm8974_venus_bfb,
	.num_bfb = ARRAY_SIZE(msm8974_venus_bfb),
};


/*
 * MSM8992 MDP QSMMU v1. Same programming model as MSM8974. Talkman
 * 3.10 mmo_defconfig leaves CONFIG_IOMMU_LPAE off, so the pagetable
 * format is short-descriptor (ARM_V7S), not LPAE. BFB values are from
 * the 3.10 msm8992-iommu.dtsi tables with the downstream 0x2000
 * impl-def bias removed. MDP is TZ-managed (secure id 1). GPU stays off.
 */
static const struct qcom_iommu_bfb_reg msm8992_mdp_bfb[] = {
	{ 0x04c, 0x007fffff },
	{ 0x060, 0x00001777 },
	{ 0x514, 0x00000000 },
	{ 0x540, 0x00000004 },
	{ 0x56c, 0x00000010 },
	{ 0x0ac, 0x00005000 },
	{ 0x15c, 0x0000cc66 },
	{ 0x20c, 0x00002000 },
	{ 0x2bc, 0x0000cc10 },
	{ 0x314, 0x00000000 },
	{ 0x394, 0x00000000 },
	{ 0x414, 0x00000028 },
	{ 0x494, 0x00000068 },
	{ 0x008, 0x00000000 },
	{ 0x00c, 0x00000000 },
	{ 0x010, 0x00000000 },
	{ 0x014, 0x00000000 },
};

static const struct qcom_iommu_cfg msm8992_mdp_cfg = {
	.halt = true,
	.no_stall = true,
	.fmt = ARM_V7S,
	.no_afe = true,
	.ctx_restore = true,
	.bfb = msm8992_mdp_bfb,
	.num_bfb = ARRAY_SIZE(msm8992_mdp_bfb),
};

/*
 * MSM8994 MDP QSMMU v1. Same programming model as MSM8992.
 * 3.10 msm8994-iommu.dtsi BFB with the downstream 0x2000 impl-def
 * bias removed. Board DT keeps the 8994 MDP SMMU map; do not use
 * the 8992 MDP SMMU addresses. fmt is ARM_V7S (mmo_defconfig
 * leaves CONFIG_IOMMU_LPAE off).
 */
static const struct qcom_iommu_bfb_reg msm8994_mdp_bfb[] = {
	{ 0x04c, 0x007fffff },
	{ 0x060, 0x00001777 },
	{ 0x514, 0x00000000 },
	{ 0x540, 0x00000004 },
	{ 0x56c, 0x00000010 },
	{ 0x0ac, 0x00005000 },
	{ 0x15c, 0x000182c1 },
	{ 0x20c, 0x00005a1d },
	{ 0x2bc, 0x0001822d },
	{ 0x314, 0x00000000 },
	{ 0x394, 0x00000000 },
	{ 0x414, 0x00000028 },
	{ 0x494, 0x00000068 },
	{ 0x008, 0x00000000 },
	{ 0x00c, 0x00000000 },
	{ 0x010, 0x00000000 },
	{ 0x014, 0x00000000 },
	{ 0x018, 0x00000000 },
};

static const struct qcom_iommu_cfg msm8994_mdp_cfg = {
	.halt = true,
	.no_stall = true,
	.fmt = ARM_V7S,
	.no_afe = true,
	.ctx_restore = true,
	.bfb = msm8994_mdp_bfb,
	.num_bfb = ARRAY_SIZE(msm8994_mdp_bfb),
};

static const struct of_device_id qcom_iommu_of_match[] = {
	{ .compatible = "qcom,msm-iommu-v1" },
	{ .compatible = "qcom,msm-iommu-v2" },
	{ .compatible = "qcom,msm8974-gpu-iommu", .data = &msm8974_gpu_cfg },
	{ .compatible = "qcom,msm8974-mdp-iommu", .data = &msm8974_mdp_cfg },
	{ .compatible = "qcom,msm8974-venus-iommu", .data = &msm8974_venus_cfg },
	{ .compatible = "qcom,msm8992-mdp-iommu", .data = &msm8992_mdp_cfg },
	{ .compatible = "qcom,msm8994-mdp-iommu", .data = &msm8994_mdp_cfg },
	{ /* sentinel */ }
};

static struct platform_driver qcom_iommu_driver = {
	.driver	= {
		.name		= "qcom-iommu",
		.of_match_table	= qcom_iommu_of_match,
		.pm		= &qcom_iommu_pm_ops,
	},
	.probe	= qcom_iommu_device_probe,
	.remove = qcom_iommu_device_remove,
};

static int __init qcom_iommu_init(void)
{
	int ret;

	ret = platform_driver_register(&qcom_iommu_ctx_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&qcom_iommu_driver);
	if (ret)
		platform_driver_unregister(&qcom_iommu_ctx_driver);

	return ret;
}
device_initcall(qcom_iommu_init);