/*
 * Copyright (c) 2020 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include <errno.h>
#include <device.h>
#include <drivers/gpio.h>
#include <zephyr/types.h>
#include <sys/util.h>
#include <string.h>
#include <logging/log.h>
#include "gpio_utils.h"

#define DT_DRV_COMPAT           ite_it8xxx2_gpio
#define GPIO_LOW                0
#define GPIO_HIGH               1

#define NUM_IO_MAX              8
#define NUM_GCR             6

#define KSOL_PIN 0x00f01D00
#define KSOLGCTRLR_OFFSET 0x0D
#define KSOLGOENR_OFFSET 0x0E
#define KSOLGDMRRR_OFFSET 0x0F
#define KSOLGPODR_OFFSET 0x28
#define KSOLPU_OFFSET 0x2B
#define KSOLPD_OFFSET 0x2C

#define KSOH_PIN 0x00f01D01
#define KSOHGCTRLR_OFFSET 0x09
#define KSOHGOENR_OFFSET 0x0A
#define KSOHGDMRRR_OFFSET 0x0B
#define KSOHGPODR_OFFSET 0x26
#define KSOHPU_OFFSET 0x2C
#define KSOHPD_OFFSET 0x2D

#define KSI_PIN 0x00f01D04
#define KSIGCTRLR_OFFSET 0x02
#define KSIGOENR_OFFSET 0x03
#define KSIGDATR_OFFSET 0x04
#define KSIGDMRRR_OFFSET 0x05
#define KSIGPODR_OFFSET 0x22
#define KSIPU_OFFSET 0x25
#define KSIPD_OFFSET 0x26

#define GPIO_DIR_INPUT  0x80
#define GPIO_DIR_OUTPUT 0x40
#define GPIO_INPUT_PULL_DOWN 0x02
#define GPIO_INPUT_PULL_UP 0x04
#define IVECT_OFFSET_WITH_IRQ           0x10
#define DATA_MIRROR_OFFSET      0x60


/*
 * this two function be used to enable/disable specific irq interrupt
 */
extern void ite_intc_irq_enable(unsigned int irq);
extern void ite_intc_irq_disable(unsigned int irq);


struct gpio_ite_gcr {
	volatile uint8_t *reg_addr;
};

static const struct gpio_ite_gcr GPIO_GCR[NUM_GCR] = {
	{ &GCR19 }, { &GCR20 }, { &GCR21 }, { &GCR22 }, { &GCR23 }, { &GCR24 },
};

/*
 * Strcture gpio_ite_cfg is about the setting of gpio
 * this config will be used at initial time
 */
struct gpio_ite_cfg {
	uint32_t reg_addr;      /* gpio register base address */
	uint8_t gpio_irq[8];    /* gpio's irq */
	uint8_t gpio_pin[8];    /* gpio's irq */
	uint8_t wui_bit[8];     /* wui_bit */
	uint32_t gpcr[8];       /* gpcr */
	uint32_t gpot[8];       /* gpot */
	uint32_t wuemr[8];      /* wuemr */
	uint32_t wuesr[8];      /* wuesr */
	uint32_t wubemr[8];     /* wubemr */
};

/*
 * Strcture gpio_ite_data is about callback function
 */
struct gpio_ite_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
	uint32_t pin_callback_enables;
};

/* dev macros for GPIO */
#define DEV_GPIO_DATA(dev) \
	((struct gpio_ite_data *)(dev)->data)

#define DEV_GPIO_CFG(dev) \
	((const struct gpio_ite_cfg *)(dev)->config)

/**
 * functions for bit / port access
 */
static inline void set_port(const struct gpio_ite_cfg *config, uint8_t value)
{
	if (config->reg_addr == KSI_PIN) {
		ite_write(config->reg_addr + KSIGDATR_OFFSET, 1, value);
	} else {
		ite_write(config->reg_addr, 1, value);
	}
}

static inline uint8_t get_port(const struct gpio_ite_cfg *config)
{
	uint8_t regv;

	if (config->reg_addr == KSI_PIN) {
		regv = ite_read(config->reg_addr + KSIGDATR_OFFSET, 1);
	} else {
		regv = ite_read(config->reg_addr, 1);
	}

	return regv;
}

/**
 * Driver functions
 */

static int gpio_ite_configure(const struct device *dev,
			      gpio_pin_t pin, gpio_flags_t flags)
{

	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	uint8_t input_control_value = 0;
	uint8_t gpotr_gpotr_value = 0;

	if ((flags & GPIO_OUTPUT) && (flags & GPIO_INPUT)) {
		/* Pin cannot be configured as input and output */
		return -ENOTSUP;
	} else if (!(flags & (GPIO_INPUT | GPIO_OUTPUT))) {
		/* Pin has to be configuread as input or output */
		return -ENOTSUP;
	}
	if (flags & GPIO_OUTPUT) {

		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			if (gpio_config->reg_addr == KSI_PIN) {
				SET_MASK(ECREG(gpio_config->reg_addr + KSIGDATR_OFFSET), BIT(pin));
			} else {
				SET_MASK(ECREG(gpio_config->reg_addr), BIT(pin));
			}
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			if (gpio_config->reg_addr == KSI_PIN) {
				CLEAR_MASK(ECREG(gpio_config->reg_addr
					+ KSIGDATR_OFFSET), BIT(pin));
			} else {
				CLEAR_MASK(ECREG(gpio_config->reg_addr), BIT(pin));
			}
		}
		if (gpio_config->reg_addr == KSI_PIN) {
			SET_MASK(ECREG(gpio_config->reg_addr + KSIGCTRLR_OFFSET), BIT(pin));
			SET_MASK(ECREG(gpio_config->reg_addr + KSIGOENR_OFFSET), BIT(pin));
			/* KS pin*/
		} else if (gpio_config->reg_addr == KSOL_PIN) {
			SET_MASK(ECREG(gpio_config->reg_addr + KSOLGCTRLR_OFFSET), BIT(pin));
			SET_MASK(ECREG(gpio_config->reg_addr + KSOLGOENR_OFFSET), BIT(pin));
		} else if (gpio_config->reg_addr == KSOH_PIN) {
			SET_MASK(ECREG(gpio_config->reg_addr + KSOHGCTRLR_OFFSET), BIT(pin));
			SET_MASK(ECREG(gpio_config->reg_addr + KSOHGOENR_OFFSET), BIT(pin));
		} else   {
			ite_write(gpio_config->gpcr[pin], 1, GPIO_DIR_OUTPUT);
		}

		if ((flags & GPIO_OPEN_DRAIN)) {
			/* Pin open-drain output*/
			if (gpio_config->reg_addr == KSI_PIN) {
				SET_MASK(ECREG(gpio_config->reg_addr + KSIGPODR_OFFSET), BIT(pin));

			} else if (gpio_config->reg_addr == KSOL_PIN) {
				SET_MASK(ECREG(gpio_config->reg_addr + KSOLGPODR_OFFSET), BIT(pin));
			} else if (gpio_config->reg_addr == KSOH_PIN) {
				SET_MASK(ECREG(gpio_config->reg_addr + KSOHGPODR_OFFSET), BIT(pin));
			} else   {
				gpotr_gpotr_value = ite_read(gpio_config->gpot[pin], 1);
				gpotr_gpotr_value |= BIT(pin);
				ite_write(gpio_config->gpot[pin], 1, gpotr_gpotr_value);
			}
		} else {
			/* Pin push-pull output*/
			if (gpio_config->reg_addr == KSI_PIN) {
				CLEAR_MASK(ECREG(gpio_config->reg_addr
					+ KSIGPODR_OFFSET), BIT(pin));
			} else if (gpio_config->reg_addr == KSOL_PIN) {
				CLEAR_MASK(ECREG(gpio_config->reg_addr
					+ KSOLGPODR_OFFSET), BIT(pin));
			} else if (gpio_config->reg_addr == KSOH_PIN) {
				CLEAR_MASK(ECREG(gpio_config->reg_addr
					+ KSOHGPODR_OFFSET), BIT(pin));
			} else  {
				gpotr_gpotr_value = ite_read(gpio_config->gpot[pin], 1);
				gpotr_gpotr_value &= ~BIT(pin);
				ite_write(gpio_config->gpot[pin], 1, gpotr_gpotr_value);
			}
		}
	} else if (flags & GPIO_INPUT) {
		if (flags & GPIO_PULL_DOWN && !(flags & GPIO_PULL_UP)) {
			if (gpio_config->reg_addr == KSI_PIN) {
				CLEAR_MASK(ECREG(gpio_config->reg_addr + KSIPU_OFFSET), BIT(pin));
				SET_MASK(ECREG(gpio_config->reg_addr + KSIPD_OFFSET), BIT(pin));

			} else if (gpio_config->reg_addr == KSOL_PIN) {
				CLEAR_MASK(ECREG(gpio_config->reg_addr + KSOLPU_OFFSET), BIT(pin));
				SET_MASK(ECREG(gpio_config->reg_addr + KSOLPD_OFFSET), BIT(pin));
			} else if (gpio_config->reg_addr == KSOH_PIN) {
				CLEAR_MASK(ECREG(gpio_config->reg_addr + KSOHPU_OFFSET), BIT(pin));
				SET_MASK(ECREG(gpio_config->reg_addr + KSOHPD_OFFSET), BIT(pin));
			} else   {
				input_control_value |= GPIO_INPUT_PULL_DOWN;
			}
		} else if (flags & GPIO_PULL_UP && !(flags & GPIO_PULL_DOWN)) {
			if (gpio_config->reg_addr == KSI_PIN) {
				CLEAR_MASK(ECREG(gpio_config->reg_addr + KSIPD_OFFSET), BIT(pin));
				SET_MASK(ECREG(gpio_config->reg_addr + KSIPU_OFFSET), BIT(pin));
			} else if (gpio_config->reg_addr == KSOL_PIN) {
				CLEAR_MASK(ECREG(gpio_config->reg_addr + KSOLPD_OFFSET), BIT(pin));
				SET_MASK(ECREG(gpio_config->reg_addr + KSOLPU_OFFSET), BIT(pin));
			} else if (gpio_config->reg_addr == KSOH_PIN) {
				CLEAR_MASK(ECREG(gpio_config->reg_addr + KSOHPD_OFFSET), BIT(pin));
				SET_MASK(ECREG(gpio_config->reg_addr + KSOHPU_OFFSET), BIT(pin));
			} else  {
				input_control_value |= GPIO_INPUT_PULL_UP;
			}
		} else if (flags & GPIO_PULL_DOWN && flags & GPIO_PULL_UP) {
			if (gpio_config->reg_addr != KSI_PIN && gpio_config->reg_addr != KSOH_PIN
			    && gpio_config->reg_addr != KSOL_PIN) {
				input_control_value |= GPIO_INPUT_PULL_DOWN | GPIO_INPUT_PULL_UP;
			}
		}
		if (gpio_config->reg_addr == KSI_PIN) {
			SET_MASK(ECREG(gpio_config->reg_addr + KSIGCTRLR_OFFSET), BIT(pin));
			CLEAR_MASK(ECREG(gpio_config->reg_addr + KSIGOENR_OFFSET), BIT(pin));

		} else if (gpio_config->reg_addr == KSOL_PIN) {
			SET_MASK(ECREG(gpio_config->reg_addr + KSOLGCTRLR_OFFSET), BIT(pin));
			CLEAR_MASK(ECREG(gpio_config->reg_addr + KSOLGOENR_OFFSET), BIT(pin));
		} else if (gpio_config->reg_addr == KSOH_PIN) {
			SET_MASK(ECREG(gpio_config->reg_addr + KSOHGCTRLR_OFFSET), BIT(pin));
			CLEAR_MASK(ECREG(gpio_config->reg_addr + KSOHGOENR_OFFSET), BIT(pin));
		} else   {
			input_control_value |= GPIO_DIR_INPUT;
			ite_write(gpio_config->gpcr[pin], 1, input_control_value);
		}
	}
	return 0;
}

static int gpio_ite_port_get_raw(const struct device *dev,
				 gpio_port_value_t *value)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);

	if (gpio_config->reg_addr == KSI_PIN) {
		*value = ite_read(gpio_config->reg_addr + KSIGDMRRR_OFFSET, 1);
	} else if (gpio_config->reg_addr == KSOL_PIN) {
		*value = ite_read(gpio_config->reg_addr + KSOLGDMRRR_OFFSET, 1);
	} else if (gpio_config->reg_addr == KSOH_PIN) {
		*value = ite_read(gpio_config->reg_addr + KSOHGDMRRR_OFFSET, 1);
	} else  {
		*value = ite_read(gpio_config->reg_addr + DATA_MIRROR_OFFSET, 1);
	}
	return 0;
}

static int gpio_ite_port_set_masked_raw(const struct device *dev,
					gpio_port_pins_t mask, gpio_port_value_t value)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	uint32_t port_val;

	port_val = get_port(gpio_config);
	port_val = (port_val & ~mask) | (value & mask);
	set_port(gpio_config, port_val);
	return 0;
}

static int gpio_ite_port_set_bits_raw(const struct device *dev,
				      gpio_port_pins_t pins_mask)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	uint32_t port_val;

	port_val = get_port(gpio_config);
	port_val |= pins_mask;
	set_port(gpio_config, port_val);
	return 0;
}

static int gpio_ite_port_clear_bits_raw(const struct device *dev,
					gpio_port_pins_t pins_mask)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	uint32_t port_val;

	port_val = get_port(gpio_config);
	port_val &= ~pins_mask;
	set_port(gpio_config, port_val);
	return 0;
}

static int gpio_ite_port_toggle_bits(const struct device *dev,
				     gpio_port_pins_t pins_mask)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	uint32_t port_val;

	port_val = get_port(gpio_config);
	port_val ^= pins_mask;
	set_port(gpio_config, port_val);
	return 0;
}

static int gpio_ite_manage_callback(const struct device *dev,
				    struct gpio_callback *callback,
				    bool set)
{
	struct gpio_ite_data *data = DEV_GPIO_DATA(dev);

	return gpio_manage_callback(&data->callbacks, callback, set);
}



static void gpio_ite_isr(const void *arg)
{

	int irq_index = 0;
	int irq_num = IVECT1 - IVECT_OFFSET_WITH_IRQ;
	const struct device *dev = arg;
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	struct gpio_ite_data *data = DEV_GPIO_DATA(dev);

	for (irq_index = 0; irq_index < 8; irq_index++) {
		if (gpio_config->gpio_irq[irq_index] == irq_num) {
			SET_MASK(ECREG(gpio_config->wuesr[irq_index]),
				gpio_config->wui_bit[irq_index]);
			gpio_fire_callbacks(&data->callbacks, dev,
				BIT(gpio_config->gpio_pin[irq_index]));
			break;
		}

	}
}

static int gpio_ite_pin_interrupt_configure(const struct device *dev,
	gpio_pin_t pin, enum gpio_int_mode mode, enum gpio_int_trig trig)
{

	int ret = 0;
	uint8_t both_tri_en = 0;
	uint8_t ch_rising_failing = 0;
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);

	if (gpio_config->reg_addr == KSI_PIN || gpio_config->reg_addr == KSOL_PIN
	    || gpio_config->reg_addr == KSOH_PIN) {
		return -ENOTSUP;
	}

	if (gpio_config->gpio_irq[pin] > CONFIG_NUM_IRQS) {
		return -ENOTSUP;
	}

	if (mode == GPIO_INT_MODE_DISABLED) {
		/* Disables interrupt for a pin. */
		ite_intc_irq_disable(gpio_config->gpio_irq[pin]);
		return ret;
	} else if (mode == GPIO_INT_MODE_EDGE) {
		/* edge trigger */
		if ((trig & GPIO_INT_TRIG_LOW) && (trig & GPIO_INT_TRIG_HIGH)) {
			/* both trigger*/
			both_tri_en = 1;
		} else {
			if (trig & GPIO_INT_TRIG_LOW) {
				ch_rising_failing = 1;
			} else if (trig & GPIO_INT_TRIG_HIGH) {
				ch_rising_failing = 0;
			}
		}
	} else {
		/* level trigger */
		return -ENOTSUP;
	}
	if (ch_rising_failing) {
		SET_MASK(ECREG(gpio_config->wuemr[pin]), BIT(gpio_config->wui_bit[pin]));
	} else {
		CLEAR_MASK(ECREG(gpio_config->wuemr[pin]), BIT(gpio_config->wui_bit[pin]));
	}
	SET_MASK(ECREG(gpio_config->wuesr[pin]), BIT(gpio_config->wui_bit[pin]));
	if (both_tri_en == 1) {
		SET_MASK(ECREG(gpio_config->wubemr[pin]), BIT(gpio_config->wui_bit[pin]));
	}
	/* Enable IRQ */
	irq_connect_dynamic(
		gpio_config->gpio_irq[pin], 0, gpio_ite_isr, dev, 0);
	ite_intc_irq_enable(gpio_config->gpio_irq[pin]);

	return ret;
}


static const struct gpio_driver_api gpio_ite_driver_api = {
	.pin_configure = gpio_ite_configure,
	.port_get_raw = gpio_ite_port_get_raw,
	.port_set_masked_raw = gpio_ite_port_set_masked_raw,
	.port_set_bits_raw = gpio_ite_port_set_bits_raw,
	.port_clear_bits_raw = gpio_ite_port_clear_bits_raw,
	.port_toggle_bits = gpio_ite_port_toggle_bits,
	.pin_interrupt_configure = gpio_ite_pin_interrupt_configure,
	.manage_callback = gpio_ite_manage_callback,
};
	#define dd(...) printf(__VA_ARGS__)
	#define SPIS_DEV_DBG_L(...)             printk(__VA_ARGS__)



static int gpio_ite_init(const struct device *dev)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	int i;

	for (i = 0; i < NUM_IO_MAX; i++) {
		/* init disable intc */
		ite_intc_irq_disable(gpio_config->gpio_irq[i]);
	}

	for (i = 0; i < NUM_GCR; i++) {
		/* change all gpio support 1.8 to 3.3v */
		*(GPIO_GCR[i].reg_addr) = 0x00;
	}

	return 0;
}


#define GPIO_ITE_DEV_CFG_KS_DATA(n, idx)			      \
	static struct gpio_ite_data gpio_ite_data_##n##_##idx;	      \
	static const struct gpio_ite_cfg gpio_ite_cfg_##n##_##idx = { \
		.reg_addr = DT_INST_REG_ADDR(idx),		      \
	};							      \
	DEVICE_DT_DEFINE(DT_NODELABEL(n),			      \
			 gpio_ite_init,				      \
			 device_pm_control_nop,			      \
			 &gpio_ite_data_##n##_##idx,		      \
			 &gpio_ite_cfg_##n##_##idx,		      \
			 POST_KERNEL,				      \
			 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	      \
			 &gpio_ite_driver_api)


#define GPIO_ITE_DEV_CFG_DATA(n, idx)				      \
	static struct gpio_ite_data gpio_ite_data_##n##_##idx;	      \
	static const struct gpio_ite_cfg gpio_ite_cfg_##n##_##idx = { \
		.reg_addr = DT_INST_REG_ADDR(idx),		      \
		.gpio_irq[0] = DT_INST_IRQ_BY_IDX(idx, 0, irq),	      \
		.gpio_irq[1] = DT_INST_IRQ_BY_IDX(idx, 1, irq),	      \
		.gpio_irq[2] = DT_INST_IRQ_BY_IDX(idx, 2, irq),	      \
		.gpio_irq[3] = DT_INST_IRQ_BY_IDX(idx, 3, irq),	      \
		.gpio_irq[4] = DT_INST_IRQ_BY_IDX(idx, 4, irq),	      \
		.gpio_irq[5] = DT_INST_IRQ_BY_IDX(idx, 5, irq),	      \
		.gpio_irq[6] = DT_INST_IRQ_BY_IDX(idx, 6, irq),	      \
		.gpio_irq[7] = DT_INST_IRQ_BY_IDX(idx, 7, irq),	      \
		.gpio_pin[0] = DT_INST_IRQ_BY_IDX(idx, 0, pin),	      \
		.gpio_pin[1] = DT_INST_IRQ_BY_IDX(idx, 1, pin),	      \
		.gpio_pin[2] = DT_INST_IRQ_BY_IDX(idx, 2, pin),	      \
		.gpio_pin[3] = DT_INST_IRQ_BY_IDX(idx, 3, pin),	      \
		.gpio_pin[4] = DT_INST_IRQ_BY_IDX(idx, 4, pin),	      \
		.gpio_pin[5] = DT_INST_IRQ_BY_IDX(idx, 5, pin),	      \
		.gpio_pin[6] = DT_INST_IRQ_BY_IDX(idx, 6, pin),	      \
		.gpio_pin[7] = DT_INST_IRQ_BY_IDX(idx, 7, pin),	      \
		.wui_bit[0] = DT_INST_IRQ_BY_IDX(idx, 0, wuibit),     \
		.wui_bit[1] = DT_INST_IRQ_BY_IDX(idx, 1, wuibit),     \
		.wui_bit[2] = DT_INST_IRQ_BY_IDX(idx, 2, wuibit),     \
		.wui_bit[3] = DT_INST_IRQ_BY_IDX(idx, 3, wuibit),     \
		.wui_bit[4] = DT_INST_IRQ_BY_IDX(idx, 4, wuibit),     \
		.wui_bit[5] = DT_INST_IRQ_BY_IDX(idx, 5, wuibit),     \
		.wui_bit[6] = DT_INST_IRQ_BY_IDX(idx, 6, wuibit),     \
		.wui_bit[7] = DT_INST_IRQ_BY_IDX(idx, 7, wuibit),     \
		.gpcr[0] = DT_INST_IRQ_BY_IDX(idx, 0, gpcr),	      \
		.gpcr[1] = DT_INST_IRQ_BY_IDX(idx, 1, gpcr),	      \
		.gpcr[2] = DT_INST_IRQ_BY_IDX(idx, 2, gpcr),	      \
		.gpcr[3] = DT_INST_IRQ_BY_IDX(idx, 3, gpcr),	      \
		.gpcr[4] = DT_INST_IRQ_BY_IDX(idx, 4, gpcr),	      \
		.gpcr[5] = DT_INST_IRQ_BY_IDX(idx, 5, gpcr),	      \
		.gpcr[6] = DT_INST_IRQ_BY_IDX(idx, 6, gpcr),	      \
		.gpcr[7] = DT_INST_IRQ_BY_IDX(idx, 7, gpcr),	      \
		.gpot[0] = DT_INST_IRQ_BY_IDX(idx, 0, gpot),	      \
		.gpot[1] = DT_INST_IRQ_BY_IDX(idx, 1, gpot),	      \
		.gpot[2] = DT_INST_IRQ_BY_IDX(idx, 2, gpot),	      \
		.gpot[3] = DT_INST_IRQ_BY_IDX(idx, 3, gpot),	      \
		.gpot[4] = DT_INST_IRQ_BY_IDX(idx, 4, gpot),	      \
		.gpot[5] = DT_INST_IRQ_BY_IDX(idx, 5, gpot),	      \
		.gpot[6] = DT_INST_IRQ_BY_IDX(idx, 6, gpot),	      \
		.gpot[7] = DT_INST_IRQ_BY_IDX(idx, 7, gpot),	      \
		.wuemr[0] = DT_INST_IRQ_BY_IDX(idx, 0, wuemr),	      \
		.wuemr[1] = DT_INST_IRQ_BY_IDX(idx, 0, wuemr),	      \
		.wuemr[2] = DT_INST_IRQ_BY_IDX(idx, 0, wuemr),	      \
		.wuemr[3] = DT_INST_IRQ_BY_IDX(idx, 0, wuemr),	      \
		.wuemr[4] = DT_INST_IRQ_BY_IDX(idx, 0, wuemr),	      \
		.wuemr[5] = DT_INST_IRQ_BY_IDX(idx, 0, wuemr),	      \
		.wuemr[6] = DT_INST_IRQ_BY_IDX(idx, 0, wuemr),	      \
		.wuemr[7] = DT_INST_IRQ_BY_IDX(idx, 0, wuemr),	      \
		.wuesr[0] = DT_INST_IRQ_BY_IDX(idx, 0, wuesr),	      \
		.wuesr[1] = DT_INST_IRQ_BY_IDX(idx, 1, wuesr),	      \
		.wuesr[2] = DT_INST_IRQ_BY_IDX(idx, 2, wuesr),	      \
		.wuesr[3] = DT_INST_IRQ_BY_IDX(idx, 3, wuesr),	      \
		.wuesr[4] = DT_INST_IRQ_BY_IDX(idx, 4, wuesr),	      \
		.wuesr[5] = DT_INST_IRQ_BY_IDX(idx, 5, wuesr),	      \
		.wuesr[6] = DT_INST_IRQ_BY_IDX(idx, 6, wuesr),	      \
		.wuesr[7] = DT_INST_IRQ_BY_IDX(idx, 7, wuesr),	      \
		.wubemr[0] = DT_INST_IRQ_BY_IDX(idx, 0, wubemr),      \
		.wubemr[1] = DT_INST_IRQ_BY_IDX(idx, 1, wubemr),      \
		.wubemr[2] = DT_INST_IRQ_BY_IDX(idx, 2, wubemr),      \
		.wubemr[3] = DT_INST_IRQ_BY_IDX(idx, 3, wubemr),      \
		.wubemr[4] = DT_INST_IRQ_BY_IDX(idx, 4, wubemr),      \
		.wubemr[5] = DT_INST_IRQ_BY_IDX(idx, 5, wubemr),      \
		.wubemr[6] = DT_INST_IRQ_BY_IDX(idx, 6, wubemr),      \
		.wubemr[7] = DT_INST_IRQ_BY_IDX(idx, 7, wubemr),      \
	};							      \
	DEVICE_DT_DEFINE(DT_NODELABEL(n),			      \
			 gpio_ite_init,				      \
			 device_pm_control_nop,			      \
			 &gpio_ite_data_##n##_##idx,		      \
			 &gpio_ite_cfg_##n##_##idx,		      \
			 POST_KERNEL,				      \
			 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	      \
			 &gpio_ite_driver_api)

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioa), okay)
GPIO_ITE_DEV_CFG_DATA(gpioa, 0);
#endif /* gpioa */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiob), okay)
GPIO_ITE_DEV_CFG_DATA(gpiob, 1);
#endif /* gpiob */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioc), okay)
GPIO_ITE_DEV_CFG_DATA(gpioc, 2);
#endif /* gpioc */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiod), okay)
GPIO_ITE_DEV_CFG_DATA(gpiod, 3);
#endif /* gpiod */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioe), okay)
GPIO_ITE_DEV_CFG_DATA(gpioe, 4);
#endif /* gpioe */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiof), okay)
GPIO_ITE_DEV_CFG_DATA(gpiof, 5);
#endif /* gpiof */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiog), okay)
GPIO_ITE_DEV_CFG_DATA(gpiog, 6);
#endif /* gpiog */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioh), okay)
GPIO_ITE_DEV_CFG_DATA(gpioh, 7);
#endif /* gpioh */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioi), okay)
GPIO_ITE_DEV_CFG_DATA(gpioi, 8);
#endif /* gpioi */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioj), okay)
GPIO_ITE_DEV_CFG_DATA(gpioj, 9);
#endif /* gpioj */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiok), okay)
GPIO_ITE_DEV_CFG_DATA(gpiok, 10);
#endif /* gpiok */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiol), okay)
GPIO_ITE_DEV_CFG_DATA(gpiol, 11);
#endif /* gpiol */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiom), okay)
GPIO_ITE_DEV_CFG_DATA(gpiom, 12);
#endif /* gpiom */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioksi), okay)
GPIO_ITE_DEV_CFG_KS_DATA(gpioksi, 13);
#endif /* gpioksi */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioksoh), okay)
GPIO_ITE_DEV_CFG_KS_DATA(gpioksoh, 14);
#endif /* gpioksoh */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioksol), okay)
GPIO_ITE_DEV_CFG_KS_DATA(gpioksol, 15);
#endif /* gpioksol */








