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

#define DT_DRV_COMPAT		ite_it8xxx2_gpio
#define GPIO_LOW		0
#define GPIO_HIGH		1
#define GPIO_PP			0
#define GPIO_OD			1
#define NUM_IO_MAX		8
#define NUM_GCR		    6

#define CURR_SUPP_GPIO_SET	10

/*
 * this two function be used to enable/disable specific irq interrupt
 */
extern void ite_intc_irq_enable(unsigned int irq);
extern void ite_intc_irq_disable(unsigned int irq);

#define GPIO_GPDRA		(DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpioa), 0))
#define GPIO_GPDRB		(DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpiob), 0))
#define GPIO_GPDRC		(DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpioc), 0))
#define GPIO_GPDRD		(DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpiod), 0))
#define GPIO_GPDRE		(DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpioe), 0))
#define GPIO_GPDRF		(DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpiof), 0))
#define GPIO_GPDRG		(DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpiog), 0))
#define GPIO_GPDRH		(DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpioh), 0))
#define GPIO_GPDRI		(DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpioi), 0))
#define GPIO_GPDRJ		(DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpioj), 0))
#define GPIO_GPDRM		(DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpiom), 0))

#define GPCR_OFFSET		0x10
#define GPOTR_OFFSET	0x70

#define GPIO_DIR_INPUT	0x80
#define GPIO_DIR_OUTPUT	0x40

#define GPIO_INPUT_PULL_DOWN 0x02
#define GPIO_INPUT_PULL_UP 0x04

#define IVECT_OFFSET_WITH_IRQ		0x10


struct gpio_ite_wui {
	volatile uint8_t *reg_addr;
	volatile uint8_t *clear_addr;
	volatile uint8_t *bothedge_addr;
	uint32_t pin;
	uint32_t irq;
};

struct gpio_ite_gcr {
	volatile uint8_t *reg_addr;
};


static const struct gpio_ite_gcr GPIO_GCR[NUM_GCR] = {
	{ &GCR19 },{ &GCR20 },{ &GCR21 },{ &GCR22 },{ &GCR23 },{ &GCR24 },          	 
};


static const struct gpio_ite_wui GPIO_GPDRA_wui[NUM_IO_MAX] = {
	{ &WUEMR9, &WUESR9, &WUBEMR9, BIT(3), 96 },           	  /* gpa0, */
	{ &WUEMR9, &WUESR9, &WUBEMR9, BIT(4), 97 },           	  /* gpa1, */
	{ &WUEMR9, &WUESR9, &WUBEMR9, BIT(5), 98 },           	  /* gpa2, */
	{ &WUEMR8, &WUESR8, &WUBEMR8, BIT(0), 88 },           	  /* gpa3, */
	{ &WUEMR8, &WUESR8, &WUBEMR8, BIT(1), 89 },          	  /* gpa4, */
	{ &WUEMR8, &WUESR8, &WUBEMR8, BIT(2), 90 },          	  /* gpa5, */
	{ &WUEMR8, &WUESR8, &WUBEMR8, BIT(3), 91 },          	  /* gpa6, */
	{ &WUEMR10, &WUESR10, &WUBEMR10, BIT(4), 105},       	  /* gpa7, */
};


static const struct gpio_ite_wui GPIO_GPDRB_wui[NUM_IO_MAX] = {
	{ &WUEMR10, &WUESR10, &WUBEMR10, BIT(5), 106 },           /* gpb0, */
	{ &WUEMR10, &WUESR10, &WUBEMR10, BIT(6), 107 },           /* gpb1, */
	{ &WUEMR8, &WUESR8, &WUBEMR8, BIT(4), 92 },               /* gpb2, */
	{ &WUEMR10, &WUESR10, &WUBEMR10, BIT(7), 108 },           /* gpb3, */
	{ &WUEMR9, &WUESR9, &WUBEMR9, BIT(6), 99 },               /* gpb4, */
	{ &WUEMR11, &WUESR11, &WUBEMR11, BIT(0), 109 },           /* gpb5, */
	{ &WUEMR11, &WUESR11, &WUBEMR11, BIT(1), 110 },           /* gpb6, */
	{ &WUEMR11, &WUESR11, &WUBEMR11, BIT(2), 111 },           /* gpb7, */
};

static const struct gpio_ite_wui GPIO_GPDRC_wui[NUM_IO_MAX] = {
	{ &WUEMR8, &WUESR8, &WUBEMR8, BIT(5), 93 },               /* gpc0, */
	{ &WUEMR11, &WUESR11, &WUBEMR11, BIT(3), 112 },           /* gpc1, */
	{ &WUEMR9, &WUESR9, &WUBEMR9, BIT(7), 100 },              /* gpc2, */
	{ &WUEMR11, &WUESR11, &WUBEMR11, BIT(4), 113 },           /* gpc3, */
	{ &WUEMR2, &WUESR2, &WUBEMR2, BIT(2), 21 },               /* gpc4, */
	{ &WUEMR11, &WUESR11, &WUBEMR11, BIT(5), 114 },           /* gpc5, */
	{ &WUEMR2, &WUESR2, &WUBEMR2, BIT(3), 6 },                /* gpc6, */
	{ &WUEMR8, &WUESR8, &WUBEMR8, BIT(6), 94 },               /* gpc7, */
};

static const struct gpio_ite_wui GPIO_GPDRD_wui[NUM_IO_MAX] = {
	{ &WUEMR2, &WUESR2, &WUBEMR2, BIT(0), 1 },                /* gpd0, */
	{ &WUEMR2, &WUESR2, &WUBEMR2, BIT(1), 31 },           	  /* gpd1, */
	{ &WUEMR2, &WUESR2, &WUBEMR2, BIT(4), 17 },               /* gpd2, */
	{ &WUEMR11, &WUESR11, &WUBEMR11, BIT(6), 115 },           /* gpd3, */
	{ &WUEMR11, &WUESR11, &WUBEMR11, BIT(7), 116 },           /* gpd4, */
	{ &WUEMR12, &WUESR12, &WUBEMR12, BIT(0), 117 },           /* gpd5, */
	{ &WUEMR12, &WUESR12, &WUBEMR12, BIT(1), 118 },           /* gpd6, */
	{ &WUEMR8, &WUESR8, &WUBEMR8, BIT(7), 95 },               /* gpd7, */
};

static const struct gpio_ite_wui GPIO_GPDRE_wui[NUM_IO_MAX] = {
	{ &WUEMR7, &WUESR7, &WUBEMR7, BIT(0), 72 },               /* gpe0, */
	{ &WUEMR7, &WUESR7, &WUBEMR7, BIT(1), 73 },           	  /* gpe1, */
	{ &WUEMR7, &WUESR7, &WUBEMR7, BIT(2), 74 },               /* gpe2, */
	{ &WUEMR7, &WUESR7, &WUBEMR7, BIT(3), 75 },               /* gpe3, */
	{ &WUEMR12, &WUESR12, &WUBEMR12, BIT(2), 119 },           /* gpe4, */
	{ &WUEMR4, &WUESR4, &WUBEMR4, BIT(0), 176 },              /* gpe5, */
	{ &WUEMR4, &WUESR4, &WUBEMR4, BIT(5), 177 },              /* gpe6, */
	{ &WUEMR4, &WUESR4, &WUBEMR4, BIT(6), 178 },              /* gpe7, */
};


static const struct gpio_ite_wui GPIO_GPDRF_wui[NUM_IO_MAX] = {
	{ &WUEMR10, &WUESR10, &WUBEMR10, BIT(0), 101 },           /* gpf0, */
	{ &WUEMR10, &WUESR10, &WUBEMR10, BIT(1), 102 },           /* gpf1, */
	{ &WUEMR10, &WUESR10, &WUBEMR10, BIT(2), 103 },           /* gpf2, */
	{ &WUEMR10, &WUESR10, &WUBEMR10, BIT(3), 104 },           /* gpf3, */
	{ &WUEMR6, &WUESR6, &WUBEMR6, BIT(4), 52 },               /* gpf4, */
	{ &WUEMR6, &WUESR6, &WUBEMR6, BIT(5), 53 },               /* gpf5, */
	{ &WUEMR6, &WUESR6, &WUBEMR6, BIT(6), 54 },               /* gpf6, */
	{ &WUEMR6, &WUESR6, &WUBEMR6, BIT(7), 55 },               /* gpf7, */
};

static const struct gpio_ite_wui GPIO_GPDRG_wui[NUM_IO_MAX] = {
	{ &WUEMR12, &WUESR12, &WUBEMR12, BIT(3), 120},            /* gpg0, */
	{ &WUEMR12, &WUESR12, &WUBEMR12, BIT(4), 121 },           /* gpg1, */
	{ &WUEMR12, &WUESR12, &WUBEMR12, BIT(5), 122 },           /* gpg2, */
	{ 0, 0, 0, BIT(0), 0 },                                   /* gpg3, */
	{ 0, 0, 0, BIT(0), 0 },               					  /* gpg4, */
	{ 0, 0, 0, BIT(0), 0 },               					  /* gpg5, */
	{ &WUEMR12, &WUESR12, &WUBEMR12, BIT(6), 123 },           /* gpg6, */
	{ 0, 0, 0, BIT(0), 0 },               					  /* gpg7, */
};

static const struct gpio_ite_wui GPIO_GPDRH_wui[NUM_IO_MAX] = {
	{ &WUEMR6, &WUESR6, &WUBEMR6, BIT(0), 48 },           	  /* gph0, */
	{ &WUEMR6, &WUESR6, &WUBEMR6, BIT(1), 49 },           	  /* gph1, */
	{ &WUEMR6, &WUESR6, &WUBEMR6, BIT(2), 50 },               /* gph2, */
	{ &WUEMR6, &WUESR6, &WUBEMR6, BIT(3), 51 },               /* gph3, */
	{ &WUEMR9, &WUESR9, &WUBEMR9, BIT(0), 85 },               /* gph4, */
	{ &WUEMR9, &WUESR9, &WUBEMR9, BIT(1), 86 },               /* gph5, */
	{ &WUEMR9, &WUESR9, &WUBEMR9, BIT(2), 87 },               /* gph6, */
	{ 0, 0, 0, BIT(0), 0 },                                   /* gph7, */
};

static const struct gpio_ite_wui GPIO_GPDRI_wui[NUM_IO_MAX] = {
	{ &WUEMR12, &WUESR12, &WUBEMR12, BIT(7), 124 },           /* gpi0, */
	{ &WUEMR13, &WUESR13, &WUBEMR13, BIT(0), 125 },           /* gpi1, */
	{ &WUEMR13, &WUESR13, &WUBEMR13, BIT(1), 126 },           /* gpi2, */
	{ &WUEMR13, &WUESR13, &WUBEMR13, BIT(2), 127 },           /* gpi3, */
	{ &WUEMR7, &WUESR7, &WUBEMR7, BIT(4), 76 },               /* gpi4, */
	{ &WUEMR7, &WUESR7, &WUBEMR7, BIT(5), 77 },               /* gpi5, */
	{ &WUEMR7, &WUESR7, &WUBEMR7, BIT(6), 78 },               /* gpi6, */
	{ &WUEMR7, &WUESR7, &WUBEMR7, BIT(7), 79 },               /* gpi7, */
};

static const struct gpio_ite_wui GPIO_GPDRJ_wui[NUM_IO_MAX] = {
	{ &WUEMR14, &WUESR14, &WUBEMR14, BIT(0), 128 },           /* gpj0, */
	{ &WUEMR14, &WUESR14, &WUBEMR14, BIT(1), 129 },           /* gpj1, */
	{ &WUEMR14, &WUESR14, &WUBEMR14, BIT(2), 130 },           /* gpj2, */
	{ &WUEMR14, &WUESR14, &WUBEMR14, BIT(3), 131 },           /* gpj3, */
	{ &WUEMR14, &WUESR14, &WUBEMR14, BIT(4), 132 },           /* gpj4, */
	{ &WUEMR14, &WUESR14, &WUBEMR14, BIT(5), 133 },           /* gpj5, */
	{ &WUEMR14, &WUESR14, &WUBEMR14, BIT(6), 134 },           /* gpj6, */
	{ &WUEMR14, &WUESR14, &WUBEMR14, BIT(7), 135 },           /* gpj7, */
};




/* struct gpio_ite_reg_table is record different gpio port's register set
 * base_addr: base address of gpio set
 * wui[]: wui register group
 */
struct gpio_ite_reg_table {
	uint32_t base_addr;
	const struct gpio_ite_wui *wui;
};

struct gpio_ite_reg_table gpiox_reg[CURR_SUPP_GPIO_SET] = {
	{GPIO_GPDRA, GPIO_GPDRA_wui},	/* GPIO GROPU A */
	{GPIO_GPDRB, GPIO_GPDRB_wui},	/* GPIO GROPU B */
	{GPIO_GPDRC, GPIO_GPDRC_wui},	/* GPIO GROPU C */
	{GPIO_GPDRD, GPIO_GPDRD_wui},	/* GPIO GROPU D */
	{GPIO_GPDRE, GPIO_GPDRE_wui},	/* GPIO GROPU E */
	{GPIO_GPDRF, GPIO_GPDRF_wui},	/* GPIO GROPU F */
	{GPIO_GPDRG, GPIO_GPDRG_wui},	/* GPIO GROPU G */
	{GPIO_GPDRH, GPIO_GPDRH_wui},	/* GPIO GROPU H */
	{GPIO_GPDRI, GPIO_GPDRI_wui},	/* GPIO GROPU I */
	{GPIO_GPDRJ, GPIO_GPDRJ_wui},	/* GPIO GROPU J */
};


/*
 * Strcture gpio_ite_cfg is about the setting of gpio
 * this config will be used at initial time
 */
struct gpio_ite_cfg {
	uint32_t reg_addr;	/* gpio register base address */
	uint8_t gpio_irq[8];	/* gpio's irq */
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
static inline void set_bit(const struct gpio_ite_cfg *config,
			   uint8_t bit, bool val)
{
	uint8_t regv, new_regv;

	regv = ite_read(config->reg_addr, 1);
	new_regv = (regv & ~BIT(bit)) | (val << bit);
	ite_write(config->reg_addr, 1, new_regv);
}

static inline uint8_t get_bit(const struct gpio_ite_cfg *config, uint8_t bit)
{
	uint8_t regv = ite_read(config->reg_addr, 1);

	return !!(regv & BIT(bit));
}

static inline void set_port(const struct gpio_ite_cfg *config, uint8_t value)
{
	ite_write(config->reg_addr, 1, value);
}

static inline uint8_t get_port(const struct gpio_ite_cfg *config)
{
	uint8_t regv = ite_read(config->reg_addr, 1);

	return regv;
}

/**
 * Driver functions
 */
static int gpio_ite_configure(const struct device *dev,
			      gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	unsigned int gpcr_offset = GPCR_OFFSET;
	unsigned int gpotr_offset = GPOTR_OFFSET;
	uint32_t gpcr_reg;
	uint32_t gpotr_reg;
	uint32_t gpcr_reg_addr;
	uint32_t gpotr_reg_addr;
	uint8_t input_control_value = 0;
	uint8_t gpotr_gpotr_value = 0;
	/* counting the gpio control register's base address */
	gpcr_reg = ((gpio_config->reg_addr & 0xff) - 1) * NUM_IO_MAX
			+ gpcr_offset;
	gpotr_reg = (gpio_config->reg_addr & 0xff) + gpotr_offset;	
	gpcr_reg_addr = gpcr_reg | (gpio_config->reg_addr & 0xffffff00);
	gpotr_reg_addr = gpotr_reg | (gpio_config->reg_addr & 0xffffff00);

	if ((flags & GPIO_OUTPUT) && (flags & GPIO_INPUT)) {
		/* Pin cannot be configured as input and output */
		return -ENOTSUP;
	} else if (!(flags & (GPIO_INPUT | GPIO_OUTPUT))) {
		/* Pin has to be configuread as input or output */
		return -ENOTSUP;
	}
	if (flags & GPIO_OUTPUT) {

		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			set_bit(gpio_config, pin, GPIO_HIGH);
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			set_bit(gpio_config, pin, GPIO_LOW);
		}
		ite_write(gpcr_reg_addr + pin, 1, GPIO_DIR_OUTPUT);
		if ((flags & GPIO_OPEN_DRAIN)) {
			/* Pin open-drain output*/
			gpotr_gpotr_value=ite_read(gpotr_reg_addr,1);
		    gpotr_gpotr_value |= BIT(pin);
			ite_write(gpotr_reg_addr,1,gpotr_gpotr_value);
		} else {
			/* Pin push-pull output*/
			gpotr_gpotr_value=ite_read(gpotr_reg_addr,1);
		    gpotr_gpotr_value &= ~BIT(pin);
			ite_write(gpotr_reg_addr,1,gpotr_gpotr_value);
		}
	} else if(flags & GPIO_INPUT){
		if(flags & GPIO_PULL_DOWN && !(flags & GPIO_PULL_UP))
		{
			input_control_value |= GPIO_INPUT_PULL_DOWN;
		} else if (flags & GPIO_PULL_UP && !(flags & GPIO_PULL_DOWN))
		{
			input_control_value |= GPIO_INPUT_PULL_UP;
		} else if (flags & GPIO_PULL_DOWN && flags & GPIO_PULL_UP)
		{
			input_control_value |= GPIO_INPUT_PULL_DOWN | GPIO_INPUT_PULL_UP;		
		}
		input_control_value |= GPIO_DIR_INPUT;
		ite_write(gpcr_reg_addr + pin, 1, input_control_value);
	}
	return 0;
}

static int gpio_ite_port_get_raw(const struct device *dev,
				gpio_port_value_t *value)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);

	*value = ite_read(gpio_config->reg_addr, 1);
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
				      gpio_port_pins_t pins)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	uint32_t port_val;

	port_val = get_port(gpio_config);
	port_val |= pins;
	set_port(gpio_config, port_val);
	return 0;
}

static int gpio_ite_port_clear_bits_raw(const struct device *dev,
					gpio_port_pins_t pins)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	uint32_t port_val;

	port_val = get_port(gpio_config);
	port_val &= ~pins;
	set_port(gpio_config, port_val);
	return 0;
}

static int gpio_ite_port_toggle_bits(const struct device *dev,
				     gpio_port_pins_t pins)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	uint32_t port_val;

	port_val = get_port(gpio_config);
	port_val ^= pins;
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
	int gpio_index = 0;
	int  irq_num = IVECT1 - IVECT_OFFSET_WITH_IRQ;
	const struct device *dev = arg;
	struct gpio_ite_wui *wui_local;
	struct gpio_ite_reg_table *gpio_tab_local = NULL;
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	struct gpio_ite_data *data = DEV_GPIO_DATA(dev);
	
	for (gpio_index = 0; gpio_index < CURR_SUPP_GPIO_SET; gpio_index++) {
		if (gpio_config->reg_addr == gpiox_reg[gpio_index].base_addr) {

			gpio_tab_local = &gpiox_reg[gpio_index];
		}
	}
	for (irq_index = 0; irq_index < NUM_IO_MAX; irq_index++) {
		wui_local = (struct gpio_ite_wui *)&
			gpio_tab_local->wui[irq_index];

		if ((wui_local->irq) == (irq_num)) {
			SET_MASK(*wui_local->clear_addr, wui_local->pin);
			gpio_fire_callbacks(&data->callbacks, dev,BIT(irq_index));
		}
	}

}

static int gpio_ite_pin_interrupt_configure(const struct device *dev,
	gpio_pin_t pin, enum gpio_int_mode mode, enum gpio_int_trig trig)
{

	int ret = 0;
	int gpio_index = 0;
	uint32_t g, i;
	uint8_t both_tri_en = 0;
	uint8_t ch_rising_failing =0;
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	struct gpio_ite_wui *wui_local;
	
	if(gpio_config->gpio_irq[pin] == 0)
	{
		printk("this pin %d not support WUI\r\n",pin);
		return -ENOTSUP;
	}
	g = gpio_config->gpio_irq[pin] / NUM_IO_MAX;
	i = gpio_config->gpio_irq[pin] % NUM_IO_MAX;
	if (mode == GPIO_INT_MODE_DISABLED) {	
		/* Disables interrupt for a pin. */
		ite_intc_irq_disable(gpio_config->gpio_irq[pin]);
		return ret;
	} else if (mode == GPIO_INT_MODE_EDGE) {
		/* edge trigger */
		/* both */
		if ((trig & GPIO_INT_TRIG_LOW) && (trig & GPIO_INT_TRIG_HIGH)) {
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
		printk("not support level trigger\r\n");
		return -ENOTSUP;
	}
	/* set wui , only gpiob gpiof, currently */
	for (gpio_index = 0; gpio_index < CURR_SUPP_GPIO_SET; gpio_index++) {
		if (gpio_config->reg_addr ==
			gpiox_reg[gpio_index].base_addr) {
			wui_local = (struct gpio_ite_wui *)
				&(gpiox_reg[gpio_index].wui[pin]);
			if(ch_rising_failing)
				SET_MASK(*wui_local->reg_addr, wui_local->pin);
			else
				CLEAR_MASK(*wui_local->reg_addr, wui_local->pin);
			SET_MASK(*wui_local->clear_addr, wui_local->pin);
			if (both_tri_en == 1) {
				SET_MASK(*wui_local->bothedge_addr, wui_local->pin);
			}
		}
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
	#define SPIS_DEV_DBG_L(...) 		printk(__VA_ARGS__)



static int gpio_ite_init(const struct device *dev)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	int i;
	
	/* enable wui group4*/
	WUENR4 |= 0x61;
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


#define GPIO_ITE_DEV_CFG_DATA(n, idx) \
static struct gpio_ite_data gpio_ite_data_##n##_##idx; \
static const struct gpio_ite_cfg gpio_ite_cfg_##n##_##idx = { \
	.reg_addr = DT_INST_REG_ADDR(idx),\
	.gpio_irq[0] = DT_INST_IRQ_BY_IDX(idx, 0, irq),	\
	.gpio_irq[1] = DT_INST_IRQ_BY_IDX(idx, 1, irq),	\
	.gpio_irq[2] = DT_INST_IRQ_BY_IDX(idx, 2, irq),	\
	.gpio_irq[3] = DT_INST_IRQ_BY_IDX(idx, 3, irq),	\
	.gpio_irq[4] = DT_INST_IRQ_BY_IDX(idx, 4, irq),	\
	.gpio_irq[5] = DT_INST_IRQ_BY_IDX(idx, 5, irq),	\
	.gpio_irq[6] = DT_INST_IRQ_BY_IDX(idx, 6, irq),	\
	.gpio_irq[7] = DT_INST_IRQ_BY_IDX(idx, 7, irq),	\
	}; \
DEVICE_DT_DEFINE(DT_NODELABEL(n), \
		gpio_ite_init, \
		device_pm_control_nop, \
		&gpio_ite_data_##n##_##idx, \
		&gpio_ite_cfg_##n##_##idx, \
		POST_KERNEL, \
		CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, \
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


