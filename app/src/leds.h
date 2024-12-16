/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _APP_LEDS_H_
#define _APP_LEDS_H_

#define LED_GREEN_NODE DT_ALIAS(green_led)
#define LED_BLUE_NODE DT_ALIAS(blue_led)

#define LED_ON 0
#define LED_OFF 1

int init_leds(void);
void led_join(int value);
void led_msg(int value);
void led_error(int value);

#endif // _APP_LEDS_H_
