// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018, Linaro Limited

#include <dt-bindings/sound/qcom,q6afe.h>
#include <dt-bindings/sound/tas2552.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>
#include "common.h"
#include "qdsp6/q6afe.h"
#include "../codecs/wcd9330.h"

#define SLIM_MAX_TX_PORTS 16
#define SLIM_MAX_RX_PORTS 16
#define WCD9335_DEFAULT_MCLK_RATE	9600000
#define MI2S_BCLK_RATE			1536000
#define I2S_PCM_SEL_I2S		0
#define I2S_PCM_SEL_OFFSET	1

struct msm8994_data {
	void __iomem *quat_mux;
	struct snd_soc_jack jack;
	bool jack_setup;
};

static struct snd_soc_jack_pin msm8994_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static int msm8994_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				      struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);

	return 0;
}

static int msm_snd_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	u32 rx_ch[SLIM_MAX_RX_PORTS], tx_ch[SLIM_MAX_TX_PORTS];
	u32 rx_ch_cnt = 0, tx_ch_cnt = 0;
	int ret = 0;

	ret = snd_soc_dai_get_channel_map(codec_dai,
				&tx_ch_cnt, tx_ch, &rx_ch_cnt, rx_ch);
	if (ret != 0 && ret != -ENOTSUPP) {
		pr_err("failed to get codec chan map, err:%d\n", ret);
		goto end;
	} else if (ret == -ENOTSUPP) {
		return 0;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = snd_soc_dai_set_channel_map(cpu_dai, 0, NULL,
						  rx_ch_cnt, rx_ch);
	else
		ret = snd_soc_dai_set_channel_map(cpu_dai, tx_ch_cnt, tx_ch,
						  0, NULL);
	if (ret != 0 && ret != -ENOTSUPP)
		pr_err("Failed to set cpu chan map, err:%d\n", ret);
	else if (ret == -ENOTSUPP)
		ret = 0;
end:
	return ret;
}

/*
 * BE startup may only program clocks and DAI format. q6routing mixers
 * and codec DAPM muxes are userspace (ALSA UCM); kcontrol put from
 * here runs under the DPCM mutex and deadlocks in
 * snd_soc_dpcm_runtime_update (sm8250/sdm845/upstream msm8994).
 */
static int msm8994_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct msm8994_data *priv = snd_soc_card_get_drvdata(rtd->card);
	int ret;

	if (cpu_dai->id != QUATERNARY_MI2S_RX)
		return 0;

	if (!priv || !priv->quat_mux) {
		dev_err(rtd->card->dev, "missing lpaif_quat_mode_muxsel\n");
		return -EINVAL;
	}

	/* Select I2S on the QUAT LPAIF mux (PCM is the other mode). */
	iowrite32(I2S_PCM_SEL_I2S << I2S_PCM_SEL_OFFSET, priv->quat_mux);

	ret = snd_soc_dai_set_sysclk(cpu_dai, LPAIF_BIT_CLK,
				     MI2S_BCLK_RATE, 0);
	if (ret)
		return ret;

	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_BP_FP);
	if (!snd_soc_dai_is_dummy(codec_dai)) {
		snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_BC_FC |
					       SND_SOC_DAIFMT_NB_NF |
					       SND_SOC_DAIFMT_I2S);
		snd_soc_dai_set_sysclk(codec_dai, TAS2552_PLL_CLKIN_BCLK,
				       MI2S_BCLK_RATE, SND_SOC_CLOCK_IN);
	}

	return 0;
}

static void msm8994_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	if (cpu_dai->id == QUATERNARY_MI2S_RX)
		snd_soc_dai_set_sysclk(cpu_dai, LPAIF_BIT_CLK, 0, 0);
}

static const struct snd_soc_ops msm8994_ops = {
	.startup = msm8994_startup,
	.shutdown = msm8994_shutdown,
	.hw_params = msm_snd_hw_params,
};

static int msm8994_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct msm8994_data *priv = snd_soc_card_get_drvdata(card);

	/*
	 * Codec SLIMBUS configuration
	 * RX1, RX2, RX3, RX4, RX5, RX6, RX7, RX8, RX9, RX10, RX11, RX12, RX13
	 * TX1, TX2, TX3, TX4, TX5, TX6, TX7, TX8, TX9, TX10, TX11, TX12, TX13
	 * TX14, TX15, TX16
	 */
	unsigned int rx_ch[SLIM_MAX_RX_PORTS] = {144, 145, 146, 147, 148, 149,
					150, 151, 152, 153, 154, 155, 156};
	unsigned int tx_ch[SLIM_MAX_TX_PORTS] = {128, 129, 130, 131, 132, 133,
					    134, 135, 136, 137, 138, 139,
					    140, 141, 142, 143};

	snd_soc_dai_set_channel_map(codec_dai, ARRAY_SIZE(tx_ch),
					tx_ch, ARRAY_SIZE(rx_ch), rx_ch);

	snd_soc_dai_set_sysclk(codec_dai, 0, WCD9335_DEFAULT_MCLK_RATE,
				SNDRV_PCM_STREAM_PLAYBACK);

	/*
	 * sdm845 slim BE: machine jack + codec set_jack. qcom_snd_wcd_jack_setup
	 * only attaches LPI_MI2S / TX_CODEC_DMA, not SLIMBUS_0_RX.
	 */
	if (cpu_dai->id == SLIMBUS_0_RX) {
		int rval;

		if (!priv->jack_setup) {
			rval = snd_soc_card_jack_new_pins(card, "Headphone Jack",
							  SND_JACK_HEADPHONE,
							  &priv->jack,
							  msm8994_jack_pins,
							  ARRAY_SIZE(msm8994_jack_pins));
			if (rval < 0)
				return rval;
			priv->jack_setup = true;
		}

		rval = snd_soc_component_set_jack(codec_dai->component,
						  &priv->jack, NULL);
		if (rval != 0 && rval != -ENOTSUPP) {
			dev_err(card->dev, "Failed to set jack: %d\n", rval);
			return rval;
		}

		/*
		 * 3.10 msm8994.c msm_audrx_init → msm_afe_set_config.
		 * ADSP needs the tomtom CDC register map and PGD EA
		 * before SLIMBUS_0_RX start (IFD port enable lives there).
		 */
		rval = wcd9330_afe_set_config(codec_dai->component,
					      cpu_dai->dev);
		if (rval) {
			dev_err(card->dev, "Failed to set AFE config %d\n",
				rval);
			return rval;
		}
	}

	return 0;
}

static void msm8994_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	/* Same marker as apq8016 — UCM matches qdsp6 cards this way. */
	card->components = "qdsp6";

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm == 1) {
			link->be_hw_params_fixup = msm8994_be_hw_params_fixup;
			link->ops = &msm8994_ops;
			if (link->id != QUATERNARY_MI2S_RX)
				link->init = msm8994_init;
		}
	}
}

static int msm8994_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct msm8994_data *priv;
	struct resource *muxsel;
	struct device *dev = &pdev->dev;
	int ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	muxsel = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					      "lpaif_quat_mode_muxsel");
	if (muxsel) {
		priv->quat_mux = devm_ioremap_resource(dev, muxsel);
		if (IS_ERR(priv->quat_mux))
			return PTR_ERR(priv->quat_mux);
	}

	card->driver_name = "msm8994";
	card->dev = dev;
	card->owner = THIS_MODULE;
	snd_soc_card_set_drvdata(card, priv);
	dev_set_drvdata(dev, card);
	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	msm8994_add_be_ops(card);
	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id msm_snd_msm8994_dt_match[] = {
	{.compatible = "qcom,msm8994-sndcard"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_snd_msm8994_dt_match);

static struct platform_driver msm_snd_msm8994_driver = {
	.probe  = msm8994_platform_probe,
	.driver = {
		.name = "msm-snd-msm8994",
		.of_match_table = msm_snd_msm8994_dt_match,
	},
};
module_platform_driver(msm_snd_msm8994_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("msm8994 ASoC Machine Driver");
MODULE_LICENSE("GPL");
