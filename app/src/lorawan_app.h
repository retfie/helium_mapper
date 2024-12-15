/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _APP_LORAWAN_H_
#define _APP_LORAWAN_H_

#define DEV_EUI_SIZE 8
#define APP_EUI_SIZE 8
#define APP_KEY_SIZE 16
#define PAYLOAD_KEY_SIZE 16

int init_lora(void);
void lora_send_msg(void);

#endif /* _APP_LORAWAN_H_ */
