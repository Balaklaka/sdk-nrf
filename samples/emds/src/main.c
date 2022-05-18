/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic emergency data storage sample
 */
#include <sys/printk.h>
#include <irq.h>
#include <sys/reboot.h>
#include <dk_buttons_and_leds.h>
#include <emds/emds.h>

#include <bluetooth/bluetooth.h>
#include <sdc.h>
#include <mpsl.h>

#define PIN_DEBUG_ENABLE
#include "pin_debug_transport.h"

#include <drivers/gpio.h>

static const struct device *gpio_dev;
static struct gpio_callback gpio_cb;
#define POWER_LOSS_PIN (26)

#define EMDS_DEV_IRQ 24 /* device uses IRQ 24 */
#define EMDS_DEV_PRIO 0 /* device uses interrupt priority 2 */
#define EMDS_ISR_ARG 0
#define EMDS_IRQ_FLAGS 0 /* IRQ flags */

static int reboot_counter;
static int list_counter[256];

EMDS_STATIC_ENTRY_DEFINE(test, 1, list_counter, sizeof(list_counter));

static void isr_cb_func(void *arg)
{
	ARG_UNUSED(arg);
	
	/* Diable bluetooth and mpsl scheduler */
	(void) sdc_disable();
	mpsl_uninit();

	DBP0_ON;
	emds_store();
}

static void app_store_cb(void)
{
	DBP0_OFF;
}

static void button_handler_cb(uint32_t pressed, uint32_t changed)
{
	if (pressed & changed & BIT(0)) {
		printk("Button pressed\n");
		NVIC_SetPendingIRQ(EMDS_DEV_IRQ);
	}
	if (pressed & changed & BIT(3)) {
		printk("Store size: %d\n", emds_store_size_get());
		printk("Calculate time: %dus\n", emds_store_time_get());
		printk("Number of bootups %d\n", reboot_counter);
		for (int i = 0; i < ARRAY_SIZE(list_counter); i++) {
			printk("%d ", list_counter[i]);
		}
		printk("\n");
	}
}

static void gpio_cb_func(const struct device *dev, struct gpio_callback *gpio_cb, uint32_t pins)
{
	DBP2_ON;
	/* Diable bluetooth and mpsl scheduler */
	(void) sdc_disable();
	mpsl_uninit();

	DBP0_ON;
	emds_store();
	DBP2_OFF;
}

static int init_gpio(void)
{
	int err;

	gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	if (gpio_dev == NULL) {
		printk("GPIO_0 bind error\n");
		return -EAGAIN;
	}

	err = gpio_pin_configure(gpio_dev, POWER_LOSS_PIN, GPIO_INPUT | GPIO_PULL_DOWN);
	if (err) {
		printk("GPIO_0 config error: %d\n", err);
		return err;
	}

	gpio_init_callback(&gpio_cb, gpio_cb_func, BIT(POWER_LOSS_PIN));
	err = gpio_add_callback(gpio_dev, &gpio_cb);
	if (err) {
		printk("GPIO_0 add callback error: %d\n", err);
		return err;
	}

	err = gpio_pin_interrupt_configure(gpio_dev, POWER_LOSS_PIN, GPIO_INT_EDGE_FALLING);
	if (err) {
		printk("GPIO_0 configure interrupt error: %d\n", err);
		return err;
	}

	return 0;
}

static int init_emds(void)
{
	int err;
	DBP1_ON;

	err = emds_init(&app_store_cb);
	if (err) {
		printk("Initializing emds_init failed (err %d)\n", err);
		return err;
	}

	struct emds_entry entry;
	entry.id = 2;
	entry.data = (uint8_t*)&reboot_counter;
	entry.len = sizeof(reboot_counter);
	emds_entry_add(&entry);

	err = emds_load();
	if (err) {
		printk("Restore of data failed (err %d)\n", err);
		return err;
	}

	err = emds_prepare();
	if (err) {
		printk("Preparation failed (err %d)\n", err);
		return err;
	}

	IRQ_CONNECT(EMDS_DEV_IRQ, EMDS_DEV_PRIO, isr_cb_func,
		    EMDS_ISR_ARG, EMDS_IRQ_FLAGS);
	irq_enable(EMDS_DEV_IRQ);

	DBP1_OFF;

	printk("EMDS initialized\n");
	return 0;
}

static struct button_handler button_handler = {
	.cb = button_handler_cb,
};

void main(void)
{
	int err;

	DBP_PORTA_ENABLE;
	DBP_PORTB_ENABLE;

	printk("Initializing...\n");

	dk_leds_init();
	dk_buttons_init(NULL);
	dk_button_handler_add(&button_handler);

	bt_enable(NULL);

	err = init_emds();
	if (err) {
		printk("Init emds failed (err %d)\n", err);
	}

	err = init_gpio();
	if (err) {
		printk("Init gpio failed (err %d)\n", err);
	}

	printk("Store size: %d\n", emds_store_size_get());
	printk("Calculate time: %dus\n", emds_store_time_get());
	printk("Number of reboots %d\n", reboot_counter);
	reboot_counter++;
	for (int i = 0; i < ARRAY_SIZE(list_counter); i++) {
		printk("%d ", list_counter[i]);
		list_counter[i]++;
	}
	printk("\n");
}