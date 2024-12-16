/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _PAYLOAD_ENCRYPTION_H_
#define _PAYLOAD_ENCRYPTION_H_

#include <stdbool.h>

#include "config.h"

int init_encryption(void);
bool should_encrypt_payload(void);
int encrypt_payload(struct s_mapper_data *mapper_data, uint8_t *payload,
		    size_t *payload_size);

#endif // _PAYLOAD_ENCRYPTION_H_

