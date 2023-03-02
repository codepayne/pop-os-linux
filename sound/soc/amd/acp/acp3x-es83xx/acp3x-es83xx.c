// SPDX-License-Identifier: GPL-2.0+
//
// Machine driver for AMD ACP Audio engine using ES8336 codec.
//
// Copyright 2023 Marian Postevca <posteuca@mutex.one>
#define DEBUG
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/circ_buf.h>

#include "../acp-mach.h"

#define RB_SIZE         32

#define circ_count(circ)	  \
	(CIRC_CNT((circ)->head, (circ)->tail, RB_SIZE))

#define circ_space(circ)	  \
	(CIRC_SPACE((circ)->head, (circ)->tail, RB_SIZE))

#define get_mach_priv(card) ((struct acp3x_es83xx_private *)((acp_get_drvdata(card))->mach_priv))

/* mclk-div-by-2 + terminating entry */
#define MAX_NO_PROPS 2

#define DUAL_CHANNEL	2

#define ES83XX_ENABLE_DMIC	BIT(4)
#define ES83XX_48_MHZ_MCLK	BIT(5)

enum {
	SPEAKER_ON = 0,
	SPEAKER_OFF,
	SPEAKER_SUSPEND,
	SPEAKER_RESUME,
	SPEAKER_MAX
};

const char *msg[SPEAKER_MAX] = {
	"SPEAKER_ON",
	"SPEAKER_OFF",
	"SPEAKER_SUSPEND",
	"SPEAKER_RESUME"
};

struct acp3x_es83xx_private {
	unsigned long quirk;
	bool speaker_on;
	bool stream_suspended;
	struct snd_soc_component *codec;
	struct device *codec_dev;
	struct gpio_desc *gpio_speakers, *gpio_headphone;
	struct acpi_gpio_params enable_spk_gpio, enable_hp_gpio;
	struct acpi_gpio_mapping gpio_mapping[3];
	struct snd_soc_dapm_route mic_map[2];
	struct delayed_work jack_work;
	struct mutex rb_lock;
	struct circ_buf gpio_rb;
	u8 gpio_events_buf[RB_SIZE];
};

static const unsigned int rates[] = {
	8000, 11025, 16000, 22050, 32000,
	44100, 48000, 64000, 88200, 96000,
};

static const unsigned int rates_48mhz_mclk[] = {
	48000, 96000,
};

static const unsigned int channels[] = {
	DUAL_CHANNEL,
};

static const struct snd_pcm_hw_constraint_list hw_constraint_rates_normal = {
	.count = ARRAY_SIZE(rates),
	.list = rates,
	.mask = 0,
};

static const struct snd_pcm_hw_constraint_list hw_constraint_rates_48mhz = {
	.count = ARRAY_SIZE(rates_48mhz_mclk),
	.list = rates_48mhz_mclk,
	.mask = 0,
};

static const struct snd_pcm_hw_constraint_list constraints_channels = {
	.count = ARRAY_SIZE(channels),
	.list = channels,
	.mask = 0,
};

#define ES83xx_12288_KHZ_MCLK_FREQ   (48000 * 256)
#define ES83xx_48_MHZ_MCLK_FREQ      (48000 * 1000)

static int acp3x_es83xx_speaker_power_event(struct snd_soc_dapm_widget *w,
					    struct snd_kcontrol *kcontrol, int event);

static void acp3x_es83xx_set_gpios_values(struct acp3x_es83xx_private *priv,
					  bool speaker, bool headphone)
{
	gpiod_set_value_cansleep(priv->gpio_speakers, speaker);
	gpiod_set_value_cansleep(priv->gpio_headphone, headphone);
}

static void acp3x_es83xx_rb_insert_evt(struct circ_buf *rb, u8 val)
{
	u8 *buf = rb->buf;

	if (circ_space(rb) == 0) {
		/* make some space by dropping the oldest entry, we are more
		 * interested in the last event
		 */
		rb->tail = (rb->tail + 1) & (RB_SIZE - 1);
	}
	buf[rb->head] = val;
	rb->head = (rb->head + 1) & (RB_SIZE - 1);
}

static int acp3x_es83xx_rb_remove_evt(struct circ_buf *rb)
{
	u8 *buf = rb->buf;
	int rc = -1;

	if (circ_count(rb)) {
		rc = buf[rb->tail];
		rb->tail = (rb->tail + 1) & (RB_SIZE - 1);
	}
	return rc;
}

static void acp3x_es83xx_jack_events(struct work_struct *work)
{
	struct acp3x_es83xx_private *priv =
		container_of(work, struct acp3x_es83xx_private, jack_work.work);
	int evt;

	mutex_lock(&priv->rb_lock);
	do {
		evt = acp3x_es83xx_rb_remove_evt(&priv->gpio_rb);
		switch (evt) {
		case SPEAKER_SUSPEND:
			dev_dbg(priv->codec_dev, "jack event, %s\n", msg[evt]);
			acp3x_es83xx_set_gpios_values(priv, 0, 0);
			break;
		case SPEAKER_RESUME:
			dev_dbg(priv->codec_dev, "jack event, %s\n", msg[evt]);
			acp3x_es83xx_set_gpios_values(priv, priv->speaker_on, !priv->speaker_on);
			break;
		case SPEAKER_ON:
			dev_dbg(priv->codec_dev, "jack event, %s\n", msg[evt]);
			priv->speaker_on = true;
			acp3x_es83xx_set_gpios_values(priv, 1, 0);
			break;
		case SPEAKER_OFF:
			dev_dbg(priv->codec_dev, "jack event, %s\n", msg[evt]);
			priv->speaker_on = false;
			acp3x_es83xx_set_gpios_values(priv, 0, 1);
			break;
		}
	} while (evt != -1);
	mutex_unlock(&priv->rb_lock);
}

static int acp3x_es83xx_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct acp3x_es83xx_private *priv = get_mach_priv(card);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (substream->stream == 0) {
			dev_dbg(priv->codec_dev, "trigger start/release/resume, activating GPIOs\n");
			mutex_lock(&priv->rb_lock);
			if (priv->stream_suspended) {
				priv->stream_suspended = false;
				acp3x_es83xx_rb_insert_evt(&priv->gpio_rb, SPEAKER_RESUME);
				mutex_unlock(&priv->rb_lock);
				queue_delayed_work(system_wq, &priv->jack_work,
						   msecs_to_jiffies(1));
			} else {
				mutex_unlock(&priv->rb_lock);
			}
		}

		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		if (substream->stream == 0) {
			dev_dbg(priv->codec_dev, "trigger pause/suspend/stop deactivating GPIOs\n");
			mutex_lock(&priv->rb_lock);
			priv->stream_suspended = true;
			acp3x_es83xx_rb_insert_evt(&priv->gpio_rb, SPEAKER_SUSPEND);
			mutex_unlock(&priv->rb_lock);
			queue_delayed_work(system_wq, &priv->jack_work, msecs_to_jiffies(1));
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int acp3x_es83xx_create_swnode(struct device *codec_dev)
{
	struct property_entry props[MAX_NO_PROPS] = {};
	struct fwnode_handle *fwnode;
	int ret;

	props[0] = PROPERTY_ENTRY_BOOL("everest,mclk-div-by-2");

	fwnode = fwnode_create_software_node(props, NULL);
	if (IS_ERR(fwnode))
		return PTR_ERR(fwnode);

	ret = device_add_software_node(codec_dev, to_software_node(fwnode));

	fwnode_handle_put(fwnode);
	return ret;
}

static int acp3x_es83xx_codec_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *codec_dai;
	struct acp3x_es83xx_private *priv;
	unsigned int freq;
	int ret;

	runtime = substream->runtime;
	rtd = asoc_substream_to_rtd(substream);
	codec_dai = asoc_rtd_to_codec(rtd, 0);
	priv = get_mach_priv(rtd->card);

	if (priv->quirk & ES83XX_48_MHZ_MCLK) {
		dev_dbg(priv->codec_dev, "using a 48Mhz MCLK\n");
		snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					   &hw_constraint_rates_48mhz);
		freq = ES83xx_48_MHZ_MCLK_FREQ;
	} else {
		dev_dbg(priv->codec_dev, "using a 12.288Mhz MCLK\n");
		snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					   &hw_constraint_rates_normal);
		freq = ES83xx_12288_KHZ_MCLK_FREQ;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec sysclk: %d\n", ret);
		return ret;
	}

	ret =  snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
				   | SND_SOC_DAIFMT_CBP_CFP);
	if (ret < 0) {
		dev_err(rtd->dev, "failed to set DAI fmt: %d\n", ret);
		return ret;
	}

	runtime->hw.channels_max = DUAL_CHANNEL;
	snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
				   &constraints_channels);
	runtime->hw.formats = SNDRV_PCM_FMTBIT_S32_LE;

	return 0;
}

static struct snd_soc_jack es83xx_jack;

static struct snd_soc_jack_pin es83xx_jack_pins[] = {
	{
		.pin	= "Headphone",
		.mask	= SND_JACK_HEADPHONE,
	},
	{
		.pin	= "Headset Mic",
		.mask	= SND_JACK_MICROPHONE,
	},
};

static const struct snd_soc_dapm_widget acp3x_es83xx_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),

	SND_SOC_DAPM_SUPPLY("Speaker Power", SND_SOC_NOPM, 0, 0,
			    acp3x_es83xx_speaker_power_event,
			    SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
};

static const struct snd_soc_dapm_route acp3x_es83xx_audio_map[] = {
	{"Headphone", NULL, "HPOL"},
	{"Headphone", NULL, "HPOR"},

	/*
	 * There is no separate speaker output instead the speakers are muxed to
	 * the HP outputs. The mux is controlled Speaker and/or headphone switch.
	 */
	{"Speaker", NULL, "HPOL"},
	{"Speaker", NULL, "HPOR"},
	{"Speaker", NULL, "Speaker Power"},
};


static const struct snd_kcontrol_new acp3x_es83xx_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Internal Mic"),
};

static int acp3x_es83xx_configure_widgets(struct snd_soc_card *card)
{
	card->dapm_widgets = acp3x_es83xx_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(acp3x_es83xx_widgets);
	card->controls = acp3x_es83xx_controls;
	card->num_controls = ARRAY_SIZE(acp3x_es83xx_controls);
	card->dapm_routes = acp3x_es83xx_audio_map;
	card->num_dapm_routes = ARRAY_SIZE(acp3x_es83xx_audio_map);

	return 0;
}

static int acp3x_es83xx_speaker_power_event(struct snd_soc_dapm_widget *w,
					    struct snd_kcontrol *kcontrol, int event)
{
	struct acp3x_es83xx_private *priv = get_mach_priv(w->dapm->card);
	u8 val;

	dev_dbg(priv->codec_dev, "speaker power event: %d\n", event);
	val = SND_SOC_DAPM_EVENT_ON(event) ? SPEAKER_ON : SPEAKER_OFF;

	dev_dbg(priv->codec_dev, "speaker power event = %s\n", msg[val]);

	mutex_lock(&priv->rb_lock);
	acp3x_es83xx_rb_insert_evt(&priv->gpio_rb, val);
	mutex_unlock(&priv->rb_lock);

	queue_delayed_work(system_wq, &priv->jack_work, msecs_to_jiffies(70));

	return 0;
}

static int acp3x_es83xx_suspend_pre(struct snd_soc_card *card)
{
	struct acp3x_es83xx_private *priv = get_mach_priv(card);

	dev_dbg(priv->codec_dev, "card suspend\n");
	snd_soc_component_set_jack(priv->codec, NULL, NULL);
	return 0;
}

static int acp3x_es83xx_resume_post(struct snd_soc_card *card)
{
	struct acp3x_es83xx_private *priv = get_mach_priv(card);

	dev_dbg(priv->codec_dev, "card resume\n");
	snd_soc_component_set_jack(priv->codec, &es83xx_jack, NULL);
	return 0;
}

static int acp3x_es83xx_configure_gpios(struct acp3x_es83xx_private *priv)
{
	int ret = 0;

	priv->enable_spk_gpio.crs_entry_index = 0;
	priv->enable_hp_gpio.crs_entry_index = 1;

	priv->enable_spk_gpio.active_low = false;
	priv->enable_hp_gpio.active_low = false;

	priv->gpio_mapping[0].name = "speakers-enable-gpios";
	priv->gpio_mapping[0].data = &priv->enable_spk_gpio;
	priv->gpio_mapping[0].size = 1;
	priv->gpio_mapping[0].quirks = ACPI_GPIO_QUIRK_ONLY_GPIOIO;

	priv->gpio_mapping[1].name = "headphone-enable-gpios";
	priv->gpio_mapping[1].data = &priv->enable_hp_gpio;
	priv->gpio_mapping[1].size = 1;
	priv->gpio_mapping[1].quirks = ACPI_GPIO_QUIRK_ONLY_GPIOIO;

	dev_info(priv->codec_dev, "speaker gpio %d active %s, headphone gpio %d active %s\n",
		 priv->enable_spk_gpio.crs_entry_index,
		 priv->enable_spk_gpio.active_low ? "low" : "high",
		 priv->enable_hp_gpio.crs_entry_index,
		 priv->enable_hp_gpio.active_low ? "low" : "high");
	return ret;
}

static int acp3x_es83xx_configure_mics(struct acp3x_es83xx_private *priv)
{
	int num_routes = 0;
	int i;

	if (!(priv->quirk & ES83XX_ENABLE_DMIC)) {
		priv->mic_map[num_routes].sink = "MIC1";
		priv->mic_map[num_routes].source = "Internal Mic";
		num_routes++;
	}

	priv->mic_map[num_routes].sink = "MIC2";
	priv->mic_map[num_routes].source = "Headset Mic";
	num_routes++;

	for (i = 0; i < num_routes; i++)
		dev_info(priv->codec_dev, "%s is %s\n",
			 priv->mic_map[i].source, priv->mic_map[i].sink);

	return num_routes;
}

static int acp3x_es83xx_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_component *codec = asoc_rtd_to_codec(runtime, 0)->component;
	struct snd_soc_card *card = runtime->card;
	struct acp3x_es83xx_private *priv = get_mach_priv(card);
	int ret = 0;
	int num_routes;

	ret = snd_soc_card_jack_new_pins(card, "Headset",
					 SND_JACK_HEADSET | SND_JACK_BTN_0,
					 &es83xx_jack, es83xx_jack_pins,
					 ARRAY_SIZE(es83xx_jack_pins));
	if (ret) {
		dev_err(card->dev, "jack creation failed %d\n", ret);
		return ret;
	}

	snd_jack_set_key(es83xx_jack.jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);

	snd_soc_component_set_jack(codec, &es83xx_jack, NULL);

	priv->codec = codec;
	priv->codec_dev = codec->dev;
	acp3x_es83xx_configure_gpios(priv);

	ret = devm_acpi_dev_add_driver_gpios(codec->dev, priv->gpio_mapping);
	if (ret)
		dev_warn(codec->dev, "failed to add speaker gpio\n");

	priv->gpio_speakers = gpiod_get_optional(codec->dev, "speakers-enable",
				priv->enable_spk_gpio.active_low ? GPIOD_OUT_LOW : GPIOD_OUT_HIGH);
	if (IS_ERR(priv->gpio_speakers)) {
		dev_err(codec->dev, "could not get speakers-enable GPIO\n");
		return PTR_ERR(priv->gpio_speakers);
	}

	priv->gpio_headphone = gpiod_get_optional(codec->dev, "headphone-enable",
				priv->enable_hp_gpio.active_low ? GPIOD_OUT_LOW : GPIOD_OUT_HIGH);
	if (IS_ERR(priv->gpio_headphone)) {
		dev_err(codec->dev, "could not get headphone-enable GPIO\n");
		return PTR_ERR(priv->gpio_headphone);
	}

	if (priv->quirk & ES83XX_48_MHZ_MCLK) {
		ret = acp3x_es83xx_create_swnode(codec->dev);
		if (ret != 0) {
			dev_err(codec->dev,
				"could not create software node inside codec: %d\n", ret);
			return ret;
		}
	}

	num_routes = acp3x_es83xx_configure_mics(priv);
	if (num_routes > 0) {
		ret = snd_soc_dapm_add_routes(&card->dapm, priv->mic_map, num_routes);
		if (ret != 0)
			device_remove_software_node(codec->dev);
	}

	return ret;
}

static const struct snd_soc_ops acp3x_es83xx_ops = {
	.startup = acp3x_es83xx_codec_startup,
	.trigger = acp3x_es83xx_trigger,
};


SND_SOC_DAILINK_DEF(codec,
		    DAILINK_COMP_ARRAY(COMP_CODEC("i2c-ESSX8336:00", "ES8316 HiFi")));

static const struct dmi_system_id acp3x_es83xx_dmi_table[] = {
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_VENDOR, "HUAWEI"),
			DMI_EXACT_MATCH(DMI_PRODUCT_NAME, "KLVL-WXXW"),
			DMI_EXACT_MATCH(DMI_PRODUCT_VERSION, "M1010"),
		},
		.driver_data = (void *)(ES83XX_ENABLE_DMIC),
	},
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_VENDOR, "HUAWEI"),
			DMI_EXACT_MATCH(DMI_PRODUCT_NAME, "KLVL-WXX9"),
			DMI_EXACT_MATCH(DMI_PRODUCT_VERSION, "M1010"),
		},
		.driver_data = (void *)(ES83XX_ENABLE_DMIC),
	},
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_VENDOR, "HUAWEI"),
			DMI_EXACT_MATCH(DMI_PRODUCT_NAME, "BOM-WXX9"),
			DMI_EXACT_MATCH(DMI_PRODUCT_VERSION, "M1010"),
		},
		.driver_data = (void *)(ES83XX_ENABLE_DMIC|ES83XX_48_MHZ_MCLK),
	},
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_VENDOR, "HUAWEI"),
			DMI_EXACT_MATCH(DMI_PRODUCT_NAME, "HVY-WXX9"),
			DMI_EXACT_MATCH(DMI_PRODUCT_VERSION, "M1040"),
		},
		.driver_data = (void *)(ES83XX_ENABLE_DMIC),
	},
	{}
};

static int acp3x_es83xx_configure_link(struct snd_soc_card *card, struct snd_soc_dai_link *link)
{
	link->codecs = codec;
	link->num_codecs = ARRAY_SIZE(codec);
	link->init = acp3x_es83xx_init;
	link->ops = &acp3x_es83xx_ops;

	return 0;
}

static int acp3x_es83xx_probe(struct snd_soc_card *card)
{
	int ret = 0;
	struct device *dev = card->dev;
	const struct dmi_system_id *dmi_id;

	dmi_id = dmi_first_match(acp3x_es83xx_dmi_table);
	if (dmi_id && dmi_id->driver_data) {
		struct acp3x_es83xx_private *priv;
		struct acp_card_drvdata *acp_drvdata;

		acp_drvdata = (struct acp_card_drvdata *)card->drvdata;

		dev_info(dev, "matched DMI table with this system, trying to register sound card\n");

		priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
		if (!priv)
			return -ENOMEM;

		priv->quirk = (unsigned long)dmi_id->driver_data;
		acp_drvdata->mach_priv = priv;

		priv->gpio_rb.buf = priv->gpio_events_buf;
		priv->gpio_rb.head = priv->gpio_rb.tail = 0;
		mutex_init(&priv->rb_lock);

		INIT_DELAYED_WORK(&priv->jack_work, acp3x_es83xx_jack_events);
		dev_info(dev, "successfully probed the sound card\n");
	} else {
		ret = -ENODEV;
		dev_warn(dev, "this system has a ES83xx codec defined in ACPI, but the driver doesn't have this system registered in DMI table\n");
	}
	return ret;
}


void acp3x_es83xx_init_ops(struct acp_mach_ops *ops)
{
	ops->probe = acp3x_es83xx_probe;
	ops->configure_widgets = acp3x_es83xx_configure_widgets;
	ops->configure_link = acp3x_es83xx_configure_link;
	ops->suspend_pre = acp3x_es83xx_suspend_pre;
	ops->resume_post = acp3x_es83xx_resume_post;
}
