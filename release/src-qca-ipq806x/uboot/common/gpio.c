/******************************************************************************
* Filename : gpio.c
* This part is used to control LED and detect button-press
******************************************************************************/

#include <common.h>
#include <command.h>
#include <gpio.h>
#include <asm/io.h>
#include <asm/arch-ipq806x/gpio.h>
#include <asm/arch-ipq806x/iomap.h>
#include "../board/qcom/ipq806x_cdp/ipq806x_cdp.h"

#if defined(BRTAC828) || defined(RTAD7200)
#include "../drivers/rtl8370mb/rtk_types.h"
#include "../drivers/rtl8370mb/led.h"
#endif

#define INVALID_GPIO_NR	0xFFFFFFFF
#define GPIO_OE_ENABLE		1
#define GPIO_OE_DISABLE		0
#define GPIO_OUT_BIT		1
#define GPIO_IN_BIT		0

#define LED_ON 1
#define LED_OFF 0

/* LED/BTN definitions */
static struct gpio_s {
	char		*name;
	unsigned int	gpio_nr;	/* GPIO# */
	unsigned int	dir;		/* direction. 0: output; 1: input */
	unsigned int	is_led;		/* 0: NOT LED; 1: is LED */
	unsigned int	def_onoff;	/* default value of LEDs */
	unsigned int	active_low;	/* low active if non-zero */
	gpio_func_data_t data;
} gpio_tbl[GPIO_IDX_MAX] = {
#if defined(BRTAC828) || defined(RTAC88S) || defined(RTAD7200)
#if defined(BRTAC828_SR1)
	/* BRT-AC828 SR1 */
	[RST_BTN] = {	/* GPIO54, Low  active, input  */
		.name = "Reset button",
		.gpio_nr = 54,
		.dir = 1,
		.is_led = 0,
		.active_low = 1,
	},
	[PWR_LED] = {	/* GPIO53, High active, output */
		.name = "Power LED",
		.gpio_nr = 53,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_ON,
		.active_low = 0,
	},
	[WIFI_2G_LED] = {	/* GPIO68,  High active, output */
		.name = "2G LED",
		.gpio_nr = 68,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[WIFI_5G_LED] = {	/* GPIO67, High active, output */
		.name = "5G LED",
		.gpio_nr = 67,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[USB_LED] = {	/* GPIO7, High active, output */
		.name = "USB LED",
		.gpio_nr = 7,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[USB3_LED] = {	/* GPIO8, High active, output */
		.name = "USB3 LED",
		.gpio_nr = 8,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
#else
	/* BRT-AC828 SR2 or above */
	[RST_BTN] = {	/* GPIO54, Low  active, input  */
		.name = "Reset button",
		.gpio_nr = 54,
		.dir = 1,
		.is_led = 0,
		.active_low = 1,
	},
	[WPS_BTN] = {	/* GPIO16, Low  active, input  */
		.name = "WPS button",
		.gpio_nr = 16,
		.dir = 1,
		.is_led = 0,
		.active_low = 1,
	},
	[USB_BTN] = {	/* GPIO17, Low  active, input  */
		.name = "USB button",
		.gpio_nr = 17,
		.dir = 1,
		.is_led = 0,
		.active_low = 1,
	},
	[USB3_BTN] = {	/* GPIO24, Low  active, input  */
		.name = "USB3 button",
		.gpio_nr = 24,
		.dir = 1,
		.is_led = 0,
		.active_low = 1,
	},
	[PWR_LED] = {	/* GPIO53, High active, output */
		.name = "Power LED",
		.gpio_nr = 53,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_ON,
		.active_low = 0,
	},
	[PWR_RED_LED] = {/* GPIO57, High active, output */
		.name = "Power Red LED",
		.gpio_nr = 57,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[WAN_LED] = {		/* GPIO9,  High active, output */
		.name = "WAN White LED",
		.gpio_nr = 9,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[WAN2_LED] = {		/* GPIO6,  High active, output */
		.name = "WAN2 White LED",
		.gpio_nr = 6,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[WAN_RED_LED] = {	/* GPIO56,  High active, output */
		.name = "WAN Red LED",
		.gpio_nr = 56,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[WAN2_RED_LED] = {	/* GPIO55,  High active, output */
		.name = "WAN2 Red LED",
		.gpio_nr = 55,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[WIFI_2G_LED] = {	/* GPIO68,  High active, output */
		.name = "2G LED",
		.gpio_nr = 68,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[WIFI_5G_LED] = {	/* GPIO67, High active, output */
		.name = "5G LED",
		.gpio_nr = 67,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[USB_LED] = {	/* GPIO7, High active, output */
		.name = "USB LED",
		.gpio_nr = 7,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[USB3_LED] = {	/* GPIO15, High active, output */
		.name = "USB3 LED",
		.gpio_nr = 15,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
#if !defined(BRTAC828_SR2)
	[SATA_LED] = {	/* GPIO25, High active, output */
		.name = "SATA LED",
		.gpio_nr = 25,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
#endif
	[FAIL_OVER_LED] = {	/* GPIO26, High active, output */
		.name = "Fail-over LED",
		.gpio_nr = 26,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
#endif	/* BRTAC828_SR1 */
#elif defined(RTAC88N)
#if defined(CONFIG_REF_AP148_030)
	[WPS_BTN] = {	/* GPIO65, Low  active, input  */
		.name = "WPS button",
		.gpio_nr = 65,
		.dir = 1,
		.is_led = 0,
		.active_low = 1,
	},
#elif defined(CONFIG_REF_AP161)
	[WPS_BTN] = {	/* GPIO65, Low  active, input (WPS_SWITCH) */
		.name = "WPS button",
		.gpio_nr = 68,
		.dir = 1,
		.is_led = 0,
		.active_low = 1,
	},
#endif
	[RST_BTN] = {	/* GPIO54, Low  active, input (SW_SWITCH) */
		.name = "Reset button",
		.gpio_nr = 54,
		.dir = 1,
		.is_led = 0,
		.active_low = 1,
	},
	[PWR_LED] = {	/* GPIO26, High active, output (SATA LED) */
		.name = "Power LED",
		.gpio_nr = 26,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_ON,
		.active_low = 0,
	},
	[WIFI_2G_LED] = {	/* GPIO9,  High active, output (STATUS_LED_FAIL) */
		.name = "2G LED",
		.gpio_nr = 9,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[WIFI_5G_LED] = {	/* GPIO53, High active, output (STATUS_LED_PASS) */
		.name = "5G LED",
		.gpio_nr = 53,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[USB_LED] = {		/* GPIO7,  High active, output (LED_USB1) */
		.name = "USB LED",
		.gpio_nr = 7,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
	[USB3_LED] = {	/* GPIO8,  High active, output (LED_USB3) */
		.name = "USB3 LED",
		.gpio_nr = 8,
		.dir = 0,
		.is_led = 1,
		.def_onoff = LED_OFF,
		.active_low = 0,
	},
#else
#error Define GPIO table!!!
#endif
};

/* Get real GPIO# of gpio_idx
 * @return:
 *  NULL:	GPIO# not found
 *  otherwise:	pointer to GPIO PIN's data
 */
static struct gpio_s *get_gpio_def(enum gpio_idx_e gpio_idx)
{
	struct gpio_s *g;

	if (gpio_idx < 0 || gpio_idx >= GPIO_IDX_MAX) {
		printf("%s: Invalid GPIO index %d/%d\n", __func__, gpio_idx, GPIO_IDX_MAX);
		return NULL;
	}

	g = &gpio_tbl[gpio_idx];
	if (!g->name)
		return NULL;
	return g;
}

/* Whether gpio_idx is active low or not
 * @return:	1:	active low
 * 		0:	active high
 */
static unsigned int get_gpio_active_low(enum gpio_idx_e gpio_idx)
{
	struct gpio_s *g = get_gpio_def(gpio_idx);

	if (!g)
		return INVALID_GPIO_NR;

	return !!(g->active_low);
}

/* Set GPIO# as GPIO PIN and direction.
 * @gpio_nr:	GPIO#
 * @dir:	GPIO direction
 * 		0: output
 * 		1: input.
 */
static void __ipq806x_set_gpio_dir(enum gpio_idx_e gpio_idx, int dir)
{
	struct gpio_s *g = get_gpio_def(gpio_idx);
	gpio_func_data_t data = {
		.func = 0,		/* GPIO_IN_OUT */
		.pull = GPIO_NO_PULL,
		.drvstr = GPIO_8MA,
	};

	if (!g)
		return;

	data.gpio = g->gpio_nr;
	if (!dir) {
		data.out = GPIO_OUTPUT;
		data.oe = GPIO_OE_ENABLE;
	} else {
		data.out = GPIO_INPUT;
		data.oe = GPIO_OE_DISABLE;
	}
	ipq_configure_gpio(&data, 1);
}

/* Set raw value to GPIO#
 * @gpio_nr:	GPIO#
 * @val:	GPIO direction
 * 		0: low-level voltage
 * 		1: high-level voltage
 */
static void __ipq806x_set_gpio_pin(enum gpio_idx_e gpio_idx, int val)
{
	struct gpio_s *g = get_gpio_def(gpio_idx);
	uint32_t addr, reg, mask;

	if (!g)
		return;

	addr = GPIO_IN_OUT_ADDR(g->gpio_nr);
	reg = readl(addr);

	mask = 1U << GPIO_OUT_BIT;
	if (!val) {
		/* output 0 */
		reg &= ~mask;
	} else {
		/* output 1 */
		reg |= mask;
	}

	writel(reg, addr);
}

/* Read raw value of GPIO#
 * @gpio_nr:	GPIO#
 * @return:
 * 		0: low-level voltage
 * 		1: high-level voltage
 */
static int __ipq806x_get_gpio_pin(enum gpio_idx_e gpio_idx)
{
	struct gpio_s *g = get_gpio_def(gpio_idx);
	uint32_t addr, reg;

	if (!g)
		return 0;

	addr = GPIO_IN_OUT_ADDR(g->gpio_nr);
	reg = readl(addr) && (1 << GPIO_IN_BIT);

	return !!reg;
}

/* Check button status. (high/low active is handled in this function)
 * @return:	1: key is pressed
 * 		0: key is not pressed
 */
static int check_button(enum gpio_idx_e gpio_idx)
{
	struct gpio_s *g = get_gpio_def(gpio_idx);

	if (!g)
		return 0;

	return !!(__ipq806x_get_gpio_pin(gpio_idx) ^ get_gpio_active_low(gpio_idx));
}

/* Check button status. (high/low active is handled in this function)
 * @onoff:	1: Turn on LED
 * 		0: Turn off LED
 */
void led_onoff(enum gpio_idx_e gpio_idx, int onoff)
{
	__ipq806x_set_gpio_pin(gpio_idx, onoff ^ get_gpio_active_low(gpio_idx));
}

void led_init(void)
{
	int i, on;

	for (i = 0; i < GPIO_IDX_MAX; i++) {
		if (!gpio_tbl[i].name || gpio_tbl[i].dir)
			continue;
		__ipq806x_set_gpio_dir(i, 0);
		on = 1;
		if (i == WAN_RED_LED || i == WAN2_RED_LED)
			on = 0;
		led_onoff(i, on);	/* turn on all LEDs, except WANx RED LED */
	}
	udelay(300 * 1000UL);
	for (i = 0; i < GPIO_IDX_MAX; i++) {
		if (!gpio_tbl[i].name || gpio_tbl[i].dir)
			continue;
		__ipq806x_set_gpio_dir(i, 0);
		led_onoff(i, gpio_tbl[i].def_onoff);
	}
}

void gpio_init(void)
{
	printf("ASUS %s gpio init : wps / reset pin\n", model);
	__ipq806x_set_gpio_dir(WPS_BTN, 1);
	__ipq806x_set_gpio_dir(RST_BTN, 1);
}

unsigned long DETECT(void)
{
	int key = 0;

	if (check_button(RST_BTN)) {
		key = 1;
		printf("reset buootn pressed!\n");
	}
	return key;
}

unsigned long DETECT_WPS(void)
{
	int key = 0;

	if (check_button(WPS_BTN)) {
		key = 1;
		printf("wps buootn pressed!\n");
	}
	return key;
}

void power_led_on(void)
{
	led_onoff(PWR_LED, 1);
}

void power_led_off(void)
{
	led_onoff(PWR_LED, 0);
}

/* Turn on model-specific LEDs */
void leds_on(void)
{
	led_onoff(PWR_LED, 1);
	led_onoff(WAN_LED, 1);
	led_onoff(WAN2_LED, 1);
	led_onoff(LAN_LED, 1);

	/* Don't turn on below LEDs in accordance with PM's request. */
	led_onoff(PWR_RED_LED, 0);
	wan_red_led_off();
	led_onoff(USB_LED, 0);
	led_onoff(USB3_LED, 0);
	led_onoff(WIFI_2G_LED, 0);
	led_onoff(WIFI_5G_LED, 0);
	led_onoff(WPS_LED, 0);
	led_onoff(FAIL_OVER_LED, 0);
}

/* Turn off model-specific LEDs */
void leds_off(void)
{
	led_onoff(PWR_LED, 0);
	led_onoff(WAN_LED, 0);
	led_onoff(WAN2_LED, 0);
	led_onoff(LAN_LED, 0);
	wan_red_led_off();

	led_onoff(PWR_RED_LED, 0);
	led_onoff(USB_LED, 0);
	led_onoff(USB3_LED, 0);
	led_onoff(WIFI_2G_LED, 0);
	led_onoff(WIFI_5G_LED, 0);
	led_onoff(WPS_LED, 0);
	led_onoff(FAIL_OVER_LED, 0);
}

/* Turn on all model-specific LEDs */
void all_leds_on(void)
{
	int i;

#if defined(BRTAC828) || defined(RTAD7200)
	for (i = UTP_PORT0; i <= UTP_PORT7; ++i) {
		rtk_led_modeForce_set(i , LED_GROUP_0, LED_FORCE_ON);
	}
#endif

	for (i = 0; i < GPIO_IDX_MAX; i++) {
		if (!gpio_tbl[i].name || !gpio_tbl[i].is_led)
			continue;
		led_onoff(i, 1);
	}

	/* WAN RED LED share same position with WAN BLUE LED. Turn on WAN BLUE LED only*/
	wan_red_led_off();
}

/* Turn off all model-specific LEDs */
void all_leds_off(void)
{
	int i;

#if defined(BRTAC828) || defined(RTAD7200)
	for (i = UTP_PORT0; i <= UTP_PORT7; ++i) {
		rtk_led_modeForce_set(i , LED_GROUP_0, LED_FORCE_OFF);
	}
#endif

	for (i = 0; i < GPIO_IDX_MAX; i++) {
		if (!gpio_tbl[i].name || !gpio_tbl[i].is_led)
			continue;
		led_onoff(i, 0);
	}

	wan_red_led_off();
}

#if defined(ALL_LED_OFF)
void enable_all_leds(void)
{
}

void disable_all_leds(void)
{
}
#endif

#if defined(DEBUG_LED_GPIO)
int do_test_gpio (cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
	struct gpio_s *g = NULL;
	int i, j, stop, old = 0, new = 0, status;
	unsigned int gpio_idx = GPIO_IDX_MAX;

	if (argc >= 2) {
		gpio_idx = simple_strtoul(argv[1], 0, 10);
		g = get_gpio_def(gpio_idx);
	}
	if (!g) {
		printf("%8s %20s %5s %9s %10s \n", "gpio_idx", "name", "gpio#", "direction", "active low");
		for (i = 0; i < GPIO_IDX_MAX; ++i) {
			if (!(g = get_gpio_def(i)))
				continue;
			printf("%8d %20s %5d %9s %10s \n", i, g->name, g->gpio_nr,
				(!g->dir)?"output":"input", (g->active_low)? "yes":"no");
		}
		return 1;
	}

	printf("%s: GPIO index %d GPIO#%d direction %s active_low %s\n",
		g->name, gpio_idx, g->gpio_nr,
		(!g->dir)?"output":"input", (g->active_low)? "yes":"no");
	printf("Press any key to stop testing ...\n");
	if (!g->dir) {
		/* output */
		for (i = 0, stop = 0; !stop; ++i) {
			printf("%s: %s\n", g->name, (i&1)? "ON":"OFF");
			led_onoff(gpio_idx, i & 1);
			for (j = 0, stop = 0; !stop && j < 40; ++j) {
				udelay(100000);
				if (tstc())
					stop = 1;
			}
		}
	} else {
		/* input */
		for (i = 0, stop = 0; !stop; ++i) {
			new = __ipq806x_get_gpio_pin(gpio_idx);
			status = check_button(gpio_idx);
			if (!i || old != new) {
				printf("%s: %d [%s]\n", g->name, new, status? "pressed":"not pressed");
				old = new;
			}
			for (j = 0, stop = 0; !stop && j < 10; ++j) {
				udelay(5000);
				if (tstc())
					stop = 1;
			}
		}
	}

	return 0;
}

U_BOOT_CMD(
    test_gpio, 2, 0, do_test_gpio,
    "test_gpio - Test GPIO.\n",
    "test_gpio [<gpio_idx>] - Test GPIO PIN.\n"
    "                <gpio_idx> is the index of GPIO table.\n"
    "                If gpio_idx is invalid or is not specified,\n"
    "                GPIO table is printed.\n"
);


int do_ledon(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	leds_on();

	return 0;
}

int do_ledoff(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	leds_off();

	return 0;
}

U_BOOT_CMD(
    ledon, 1, 1, do_ledon,
	"ledon\t -set led on\n",
	NULL
);

U_BOOT_CMD(
    ledoff, 1, 1, do_ledoff,
	"ledoff\t -set led off\n",
	NULL
);

#if defined(ALL_LED_OFF)
int do_all_ledon(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	enable_all_leds();

	return 0;
}

int do_all_ledoff(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	disable_all_leds();

	return 0;
}

U_BOOT_CMD(
    all_ledon, 1, 1, do_all_ledon,
	"all_ledon\t -set all_led on\n",
	NULL
);

U_BOOT_CMD(
    all_ledoff, 1, 1, do_all_ledoff,
	"all_ledoff\t -set all_led off\n",
	NULL
);
#endif

#endif	/* DEBUG_LED_GPIO */
