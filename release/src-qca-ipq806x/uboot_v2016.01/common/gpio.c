/******************************************************************************
* Filename : gpio.c
* This part is used to control LED and detect button-press
******************************************************************************/

#include <common.h>
#include <command.h>
#include <gpio.h>
#include <asm/io.h>
#include <asm/arch-qca-common/qca_common.h>
#include <asm/arch-qca-common/gpio.h>
#include <../dts/include/dt-bindings/qcom/gpio-ipq807x.h>	/* GPIO_NO_PULL, GPIO_8MA, etc. */

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
	struct qca_gpio_config data;
} gpio_tbl[GPIO_IDX_MAX] = {
#if defined(GTAXY16000) || defined(RTAX89U) || defined(GTAX6000S)
	[WPS_BTN] =		{	/* GPIO34, Low  active, input  */
		.name = "WPS button",
		.gpio_nr = 34,
		.dir = 1,
		.is_led = 0,
		.active_low = 1,
	},
#ifdef CONFIG_RTAX89U_OLD_ER1
	/* SR1 ~ ER1 */
	[LED_ON_OFF_BTN] =	{	/* GPIO25, Low  active, input  */
		.name = "LED ON/OFF button",
		.gpio_nr = 25, .dir = 1, .is_led = 0, .active_low = 1,
	},
#else
	/* ER2 or above. */
	[TURBO_BTN] =	{	/* GPIO25, Low  active, input  */
		.name = "Boost button",
		.gpio_nr = 25,
		.dir = 1,
		.is_led = 0,
		.active_low = 1,
	},
#endif
#ifdef CONFIG_RTAX89U_OLD_SR1
	/* Use Wi-Fi switch as RESET button temporarilly. */
	[RST_BTN] =		{	/* GPIO26, Low  active, input  */
		.name = "RESET button",
		.gpio_nr = 26, .dir = 1, .is_led = 0, .active_low = 1,
	},
#else
	[WIFI_SW_BTN] =		{	/* GPIO26, Low  active, input  */
		.name = "Wi-Fi switch button",
		.gpio_nr = 26,
		.dir = 1,
		.is_led = 0,
		.active_low = 1,
	},
#if defined(CONFIG_RTAX89U_OLD_PR1) || defined(CONFIG_RTAX89U_OLD_ER1)
	/* old PR1, PCB R3.50 */
	[RST_BTN] =		{	/* GPIO58, Low  active, input  */
		.name = "RESET button",
		.gpio_nr = 58,
		.dir = 1,
		.is_led = 0,
		.active_low = 1,
	},
#else
	/* new SR1, PCB R4.00 */
	[RST_BTN] =		{	/* GPIO61, Low  active, input  */
		.name = "RESET button",
		.gpio_nr = 61,
		.dir = 1,
		.is_led = 0,
		.active_low = 1,
	},
#endif
#endif
#ifdef CONFIG_RTAX89U_OLD_ER1
	/* old SR1 ~ old ER1 */
	[SWITCH_RESET] =	{	/* GPIO#63, Low active, output */
		.name = "SWITCH_RESET",	/* QCA8337 + AR8033 + AR8035 + AQR107 */
		.gpio_nr = 63, .dir = 0, .is_led = 0, .active_low = 1,
	},
#else
	/* old ER2 or above */
	[SWITCH_RESET] =	{	/* GPIO#63, Low active, output */
		.name = "SWITCH_RESET",
		.gpio_nr = 63,		/* QCA8337 + AR8035 + AQR107 */
		.dir = 0,
		.is_led = 0,
		.active_low = 1,
	},
	[PHY_RESET] = 		{	/* GPIO#55, Low active, output */
		.name = "PHY_RESET",
		.gpio_nr = 55,		/* AR8033 */
		.dir = 0,
		.is_led = 0,
		.active_low = 1,
	},
#endif
	[MALIBU_RESET] = 	{	/* GPIO#37, Low active, output */
		.name = "MALIBU_RESET",	/* QCA8075 */
		.gpio_nr = 37,
		.dir = 0,
		.is_led = 0,
		.active_low = 1,
	},
	[PWR_LED] = 		{	/* GPIO#21, High active, output */
		.name = "Power LED",
		.gpio_nr = 21,
		.dir = 0,
		.is_led = 1,
		.active_low = 0,
	},
	[WIFI_2G_LED] = 	{	/* GPIO#18, High active, output */
		.name = "2G LED",
		.gpio_nr = 18,
		.dir = 0,
		.is_led = 1,
		.active_low = 0,
	},
	[WIFI_5G_LED] = 	{	/* GPIO#19, High active, output */
		.name = "5G LED",
		.gpio_nr = 19,
		.dir = 0,
		.is_led = 1,
		.active_low = 0,
	},
#ifdef CONFIG_RTAX89U_OLD_SR1
	[WIFI_5GH_LED] =	{	/* GPIO#20, High active, output */
		.name = "5G_H LED",
		.gpio_nr = 20, .dir = 0, .is_led = 1, .active_low = 0,
	},
#else
	[RJ45_10G_LED] = 	{	/* GPIO#20, High active, output */
		.name = "10G RJ-45 LED",
		.gpio_nr = 20,
		.dir = 0,
		.is_led = 1,
		.active_low = 0,
	},
#ifdef CONFIG_RTAX89U_OLD_SR1
	[ALL_LED] = 	{		/* GPIO#55, High active, output */
		.name = "All LED",
		.gpio_nr = 55, .dir = 0, .is_led = 1, .active_low = 0,
	},
#endif
#if defined(CONFIG_RTAX89U_OLD_PR1) || defined(CONFIG_RTAX89U_OLD_ER1)
	/* old PR1, PCB R3.50 */
	[WAN_LED] = 	{		/* GPIO#33, High active, output */
		.name = "WAN WHITE LED",
		.gpio_nr = 33,
		.dir = 0,
		.is_led = 1,
		.active_low = 0,
	},
#else
	/* new SR1, PCB R4.00 */
	[WAN_LED] = 	{		/* GPIO#47, High active, output */
		.name = "WAN WHITE LED",
		.gpio_nr = 47,
		.dir = 0,
		.is_led = 1,
		.active_low = 0,
	},
#endif
#endif
	[WAN_RED_LED] = 	{	/* GPIO#44, Low active, output */
		.name = "WAN RED LED",
		.gpio_nr = 44,
		.dir = 0,
		.is_led = 1,
#ifdef CONFIG_RTAX89U_OLD_SR1
		.active_low = 1,
#else
		.active_low = 0,
#endif
	},
	[LAN_LED] = 		{	/* GPIO#35, High active, output */
		.name = "LAN LED",
		.gpio_nr = 35,
		.dir = 0,
		.is_led = 1,
		.active_low = 0,
	},
	[SFPP_10G_LED] = 	{	/* GPIO#36, High active, output */
		.name = "10G SFP+ LED",
		.gpio_nr = 36,
		.dir = 0,
		.is_led = 1,
		.active_low = 0,
	},
#ifndef CONFIG_RTAX89U_OLD_ER1
	/* old ER2 or above */
	[TURBO_LED] = 	{		/* GPIO#22, High active, output */
		.name = "Boost LED",
		.gpio_nr = 22,
		.dir = 0,
		.is_led = 1,
		.active_low = 0,
	},
#endif
#if !defined(CONFIG_RTAX89U_OLD_SR1) && !defined(CONFIG_RTAX89U_OLD_ER1)
	/* old PR1 or above */
	[FAN_CTRL_H] = 	{		/* GPIO#66, High active, output */
		.name = "FAN H.Ctrl",
		.gpio_nr = 66,
		.dir = 0,
		.is_led = 0,
		.active_low = 0,
	},
	[FAN_CTRL_L] = 	{		/* GPIO#64, High active, output */
		.name = "FAN L.Ctrl",
		.gpio_nr = 64,
		.dir = 0,
		.is_led = 0,
		.active_low = 0,
	},
	[FAN_SPEED] = 	{		/* GPIO#65, input */
		.name = "FAN RPM status",
		.gpio_nr = 65,
		.dir = 1,
		.is_led = 0,
		.active_low = 0,
	},
#endif
#elif defined(GTAX6000N)
	[PHY_RESET] = 		{	/* GPIO#44, Low active, output */
		.name = "PHY_RESET",
		.gpio_nr = 44,
		.dir = 0,
		.is_led = 0,
		.active_low = 1,
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

	if (!g) {
		debug("%s: gpio_idx %d not found.\n", __func__, gpio_idx);
		return INVALID_GPIO_NR;
	}

	return !!(g->active_low);
}

/* Set GPIO# as GPIO PIN and direction.
 * @gpio_nr:	GPIO#
 * @dir:	GPIO direction
 * 		0: output
 * 		1: input.
 */
static void __ipq807x_set_gpio_dir(enum gpio_idx_e gpio_idx, int dir)
{
	struct gpio_s *g = get_gpio_def(gpio_idx);
	struct qca_gpio_config data = {
		.func = 0,		/* GPIO_IN_OUT */
		.pull = GPIO_NO_PULL,
		.drvstr = GPIO_8MA,
	};

	if (!g) {
		debug("%s: gpio_idx %d not found.\n", __func__, gpio_idx);
		return;
	}

	data.gpio = g->gpio_nr;
	if (!dir) {
		data.out = GPIO_OUTPUT;
		data.oe = GPIO_OE_ENABLE;
	} else {
		data.out = GPIO_INPUT;
		data.oe = GPIO_OE_DISABLE;
	}
	gpio_tlmm_config(&data);
}

/* Set raw value to GPIO#
 * @gpio_nr:	GPIO#
 * @val:	GPIO direction
 * 		0: low-level voltage
 * 		1: high-level voltage
 */
static void __ipq807x_set_gpio_pin(enum gpio_idx_e gpio_idx, int val)
{
	struct gpio_s *g = get_gpio_def(gpio_idx);

	if (!g) {
		debug("%s: gpio_idx %d not found.\n", __func__, gpio_idx);
		return;
	}

	gpio_set_value(g->gpio_nr, !!val);
}

/* Read raw value of GPIO#
 * @gpio_nr:	GPIO#
 * @return:
 * 		0: low-level voltage
 * 		1: high-level voltage
 */
static int __ipq807x_get_gpio_pin(enum gpio_idx_e gpio_idx)
{
	struct gpio_s *g = get_gpio_def(gpio_idx);

	if (!g) {
		debug("%s: gpio_idx %d not found.\n", __func__, gpio_idx);
		return 0;
	}

	return !!gpio_get_value(g->gpio_nr);
}

/* Check button status. (high/low active is handled in this function)
 * @return:	1: key is pressed
 * 		0: key is not pressed
 */
static int check_button(enum gpio_idx_e gpio_idx)
{
	struct gpio_s *g = get_gpio_def(gpio_idx);

	if (!g) {
		debug("%s: gpio_idx %d not found.\n", __func__, gpio_idx);
		return 0;
	}

	return !!(__ipq807x_get_gpio_pin(gpio_idx) ^ get_gpio_active_low(gpio_idx));
}

/* Check button status. (high/low active is handled in this function)
 * @onoff:	1: Turn on LED
 * 		0: Turn off LED
 */
void led_onoff(enum gpio_idx_e gpio_idx, int onoff)
{
	__ipq807x_set_gpio_pin(gpio_idx, onoff ^ get_gpio_active_low(gpio_idx));
}

void led_init(void)
{
	int i, on;

	for (i = 0; i < GPIO_IDX_MAX; i++) {
		if (!gpio_tbl[i].name || gpio_tbl[i].dir)
			continue;
		__ipq807x_set_gpio_dir(i, 0);

		if (!gpio_tbl[i].is_led)
			continue;

		on = 1;
		if (i == WAN_RED_LED || i == WAN2_RED_LED)
			on = 0;
		led_onoff(i, on);	/* turn on all LEDs, except WANx RED LED */
	}
	udelay(300 * 1000UL);
	for (i = 0; i < GPIO_IDX_MAX; i++) {
		if (!gpio_tbl[i].name || gpio_tbl[i].dir)
			continue;
		__ipq807x_set_gpio_dir(i, 0);
		led_onoff(i, gpio_tbl[i].def_onoff);
	}
}

void gpio_init(void)
{
	int i;

	printf("ASUS %s gpio init : buttons\n", model);

	for (i = 0; i < GPIO_IDX_MAX; i++) {
		if (!gpio_tbl[i].name || !gpio_tbl[i].dir)
			continue;
		__ipq807x_set_gpio_dir(i, 1);
	}

#if defined(GTAXY16000) || defined(RTAX89U) || defined(GTAX6000N) || defined(GTAX6000S)
	/* Assert, de-assert RESET pin of PHY/switches. */
#if !defined(CONFIG_RTAX89U_OLD_ER1)
	__ipq807x_set_gpio_pin(PHY_RESET, 0);
#endif
	__ipq807x_set_gpio_pin(MALIBU_RESET, 0);
	__ipq807x_set_gpio_pin(SWITCH_RESET, 0);
	mdelay(100);
	__ipq807x_set_gpio_pin(SWITCH_RESET, 1);
	__ipq807x_set_gpio_pin(MALIBU_RESET, 1);
#if !defined(CONFIG_RTAX89U_OLD_ER1)
	__ipq807x_set_gpio_pin(PHY_RESET, 1);
#endif
	debug("Assert/de-assert PHY/Switch RESET pin.\n");

	/* FAN full-speed at start-up. */
	__ipq807x_set_gpio_pin(FAN_CTRL_L, 1);
	__ipq807x_set_gpio_pin(FAN_CTRL_H, 1);
#endif
}

unsigned long DETECT(void)
{
	int key = 0;

	if (check_button(RST_BTN)) {
		key = 1;
		printf("reset button pressed!\n");
	}
	return key;
}

unsigned long DETECT_WPS(void)
{
	int key = 0;

	if (check_button(WPS_BTN)) {
		key = 1;
		printf("wps button pressed!\n");
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
			if (!g->dir && !g->is_led)
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
			new = __ipq807x_get_gpio_pin(gpio_idx);
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
