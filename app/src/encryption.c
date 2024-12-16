/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <tinycrypt/constants.h>
#include <tinycrypt/cbc_mode.h>
#include <tinycrypt/ctr_prng.h>
#include <zephyr/drivers/entropy.h>

#include "config.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(helium_mapper_enc);

#define TC_ALIGN_UP(N,PAGE) (((N) + (PAGE) - 1) & ~((PAGE) - 1))

#define TC_AES_IV_NBYTES 16
static uint8_t payload_clearbuf[TC_ALIGN_UP(sizeof(struct s_mapper_data), TC_AES_BLOCK_SIZE)];
static uint8_t payload_encbuf[sizeof(payload_clearbuf) + TC_AES_BLOCK_SIZE];
/* Temporary buffer for AES IV. */
static uint8_t enc_aes_iv[TC_AES_IV_NBYTES];

TCCtrPrng_t prng;
const struct device *entropy_dev;


bool should_encrypt_payload(void)
{
	uint8_t *payload_key = config_get_payload_key();

	return *payload_key != 0
		|| memcmp(payload_key,
			  (payload_key + 1),
			  PAYLOAD_KEY_SIZE - 1);
}

int encrypt_payload(struct s_mapper_data *mapper_data, uint8_t *payload,
		    size_t *payload_size)
{
	int err;
	struct tc_aes_key_sched_struct tc_sched;
	uint8_t *payload_key = config_get_payload_key();

	/* Sanity check. */
	assert (PAYLOAD_KEY_SIZE == TC_AES_KEY_SIZE);

	memset(&payload_clearbuf, 0, sizeof(payload_clearbuf));
	memset(&payload_encbuf, 0, sizeof(payload_encbuf));

	memcpy(&payload_clearbuf, mapper_data, sizeof(struct s_mapper_data));

	(void)tc_aes128_set_encrypt_key(&tc_sched, payload_key);

	/* Use HWRNG to seed the PRNG.  Reuse the payload_encbuf buffer. */
	err = entropy_get_entropy(entropy_dev,
				  &payload_encbuf[0],
				  sizeof(payload_encbuf));
	if (err) {
		LOG_ERR("ENTROPY: failed to obtain entropy.");
		return -EIO;
	}
	err = tc_ctr_prng_reseed(&prng, &payload_encbuf[0],
				 sizeof(payload_encbuf), 0, 0);
	if (err != TC_CRYPTO_SUCCESS) {
		LOG_ERR("PRNG failed");
		return -EIO;
	}

	/* fill-in IV from PRNG. */
	err = tc_ctr_prng_generate(&prng, NULL, 0,
				   &enc_aes_iv[0], sizeof(enc_aes_iv));
	if (err != TC_CRYPTO_SUCCESS) {
		LOG_ERR("PRNG failed: %d", err);
		return -EIO;
	}
	memset(&payload_encbuf, 0, sizeof(payload_encbuf));
	err = tc_cbc_mode_encrypt(payload_encbuf,
				  sizeof(payload_encbuf),
				  &payload_clearbuf[0],
				  sizeof(payload_clearbuf),
				  &enc_aes_iv[0],
				  &tc_sched);
	if (err != TC_CRYPTO_SUCCESS) {
		LOG_ERR("payload encrypt failed: %d", err);
		return -EIO;
	}

	*payload_size = sizeof(payload_encbuf);
	memcpy(payload, payload_encbuf, sizeof(payload_encbuf));

	return 0;
}

int init_encryption(void)
{
	int ret;
	uint8_t entropy[128];

	entropy_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));
	if (!device_is_ready(entropy_dev)) {
		LOG_ERR("ENTROPY: random device not ready!");
		return -ENODEV;
	}

	ret = entropy_get_entropy(entropy_dev, &entropy[0], sizeof(entropy));
	if (ret) {
		LOG_ERR("ENTROPY: failed to obtain entropy.");
		return ret;
	}

	ret = tc_ctr_prng_init(&prng, &entropy[0], sizeof(entropy), 0, 0U);
	if (ret != TC_CRYPTO_SUCCESS) {
		LOG_ERR("PRNG init error.");
		return ret;
	}

	return 0;
}
