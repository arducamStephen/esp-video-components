#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

#include "mbedtls/md5.h"

// #include "esp_cam_sensor.h"
// #include "esp_cam_sensor_detect.h"
// #include "sc2336_settings.h"
// #include "sc2336.h"

// #include <esp_types.h>
// #include <stdlib.h>
// #include <string.h>
// #include "sdkconfig.h"
// #include "esp_log.h"
// #include "esp_check.h"
// #include "esp_heap_caps.h"
// #include "freertos/FreeRTOS.h"
// #include "driver/i2c_master.h"



#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))


#define I2C_RETRIES 4U

#define ONE_KIB 1024U
#define MD5_SUFFIX_SIZE 9U

#define RP2040_GBDG_FLASH_BLOCK_SIZE (8U * ONE_KIB)
#define RP2040_GBDG_BLOCK_SIZE (RP2040_GBDG_FLASH_BLOCK_SIZE - MD5_SUFFIX_SIZE)

/*
 * 1MiB transfer size is an arbitrary limit
 * Max value is 4173330 (using a single manifest)
 */
#define MAX_TRANSFER_SIZE (1024U * ONE_KIB)

#define HALF_BUFFER (4U * ONE_KIB)

#define STATUS_SIZE 4
#define MD5_DIGEST_SIZE 16
#define VERSION_SIZE 4
#define ID_SIZE 8
#define TOTAL_RD_HDR_SIZE \
	(STATUS_SIZE + MD5_DIGEST_SIZE + VERSION_SIZE + ID_SIZE)

struct rp2040_gbdg_device_info {
	uint8_t md5[MD5_DIGEST_SIZE];
	uint64_t id;
	uint32_t version;
	uint32_t status;
};

#define MANIFEST_UNIT_SIZE 16
static_assert(MD5_DIGEST_SIZE == MANIFEST_UNIT_SIZE);
#define MANIFEST_HEADER_UNITS 1
#define MANIFEST_DATA_UNITS \
	DIV_ROUND_UP(MAX_TRANSFER_SIZE, RP2040_GBDG_BLOCK_SIZE)

#define STATUS_BUSY 0x01

#define DIRECT_PREFIX 0x00
#define DIRECT_CMD_CS 0x07
#define DIRECT_CMD_EMIT 0x08

#define WRITE_DATA_PREFIX 0x80
#define WRITE_DATA_PREFIX_SIZE 1

#define FIXED_SIZE_CMD_PREFIX 0x81

#define WRITE_DATA_UPPER_PREFIX 0x82
#define WRITE_DATA_UPPER_PREFIX_SIZE 1

#define NUM_GPIO 24


enum rp2040_gbdg_fixed_size_commands {
	/* 10-byte commands */
	CMD_SAVE_CACHE = 0x07,
	CMD_SEND_RB = 0x08,
	CMD_GPIO_ST_CL = 0x0b,
	CMD_GPIO_OE = 0x0c,
	CMD_DAT_RECV = 0x0d,
	CMD_DAT_EMIT = 0x0e,
	/* 18-byte commands */
	CMD_READ_CSUM = 0x11,
	CMD_SEND_MANI = 0x13,
};


struct rp2040_gbdg {
	// struct spi_controller *controller;

	// struct dentry *debugfs;
	size_t transfer_progress;

	// struct i2c_client *client;
	// struct crypto_shash *shash;
	// struct shash_desc *shash_desc;

	// struct regulator *regulator;

	// struct gpio_chip gc;
	// uint32_t gpio_requested;
	// uint32_t gpio_direction;

	bool fast_xfer_requires_i2c_lock;
	// struct gpio_descs *fast_xfer_gpios;
	uint8_t *fast_xfer_gpios;
	uint32_t fast_xfer_recv_gpio_base;
	uint8_t fast_xfer_data_index;
	uint8_t fast_xfer_clock_index;
	// void __iomem *gpio_base;
	void *gpio_base;
	// void __iomem *rio_base;
	void *rio_base;

	bool bypass_cache;

	uint8_t buffer[2 + HALF_BUFFER];
	uint8_t manifest_prep[(MANIFEST_HEADER_UNITS + MANIFEST_DATA_UNITS) *
			 MANIFEST_UNIT_SIZE];
};


#define sizeof_field(TYPE, MEMBER) sizeof(((TYPE *)0)->MEMBER)

#define __round_mask(x, y) ((__typeof__(x))((y)-1))
#define round_up(x, y) ((((x)-1) | __round_mask(x, y))+1)
#define round_down(x, y) ((x) & ~__round_mask(x, y))

const char hex_asc[] = "0123456789abcdef";

#define hex_asc_lo(x)	hex_asc[((x) & 0x0f)]
#define hex_asc_hi(x)	hex_asc[((x) & 0xf0) >> 4]

static inline char *hex_byte_pack(char *buf, uint8_t byte)
{
	*buf++ = hex_asc_hi(byte);
	*buf++ = hex_asc_lo(byte);
	return buf;
}

uint32_t gpio_direction;
static i2c_master_dev_handle_t client_handle = NULL;
struct rp2040_gbdg *rp2040_dev = NULL;


static const char *TAG = "rp2040";

char *bin2hex(char *dst, const void *src, size_t count)
{
	const unsigned char *_src = src;

	while (count--)
		dst = hex_byte_pack(dst, *_src++);
	return dst;
}

static int rp2040_gbdg_fast_xfer(struct rp2040_gbdg *priv_data, const uint8_t *data,
				 size_t len);


static int rp2040_gbdg_rp1_calc_offsets(uint8_t gpio, size_t *bank_offset,
					uint8_t *shift_offset)
{
	if (!bank_offset || !shift_offset || gpio >= 54)
		return -22;	//-EINVAL;
	if (gpio < 28) {
		*bank_offset = 0x0000;
		*shift_offset = gpio;
	} else if (gpio < 34) {
		*bank_offset = 0x4000;
		*shift_offset = gpio - 28;
	} else {
		*bank_offset = 0x8000;
		*shift_offset = gpio - 34;
	}

	return 0;
}

static int rp2040_gbdg_calc_mux_offset(uint8_t gpio, size_t *offset)
{
	size_t bank_offset;
	uint8_t shift_offset;
	int ret;

	ret = rp2040_gbdg_rp1_calc_offsets(gpio, &bank_offset, &shift_offset);
	if (ret)
		return ret;
	*offset = bank_offset + shift_offset * 8 + 0x4;

	return 0;
}

static int rp2040_gbdg_rp1_read_mux(struct rp2040_gbdg *priv_data, uint8_t gpio,
				    uint32_t *data)
{
	size_t offset;
	int ret;

	ret = rp2040_gbdg_calc_mux_offset(gpio, &offset);
	if (ret)
		return ret;

	// *data = readl(priv_data->gpio_base + offset);
	*data = *((volatile uint32_t *)(priv_data->gpio_base + offset));

	return 0;
}

static int rp2040_gbdg_rp1_write_mux(struct rp2040_gbdg *priv_data, uint8_t gpio,
				     uint32_t val)
{
	size_t offset;
	int ret;

	ret = rp2040_gbdg_calc_mux_offset(gpio, &offset);
	if (ret)
		return ret;

	// writel(val, priv_data->gpio_base + offset);
	*((volatile uint32_t *)(priv_data->gpio_base + offset)) = val;

	return 0;
}

static int rp2040_gbdg_get_device_info(i2c_master_dev_handle_t client,
				       struct rp2040_gbdg_device_info *info)
{
	uint8_t buf[TOTAL_RD_HDR_SIZE];
	// uint8_t retries = I2C_RETRIES;
	uint8_t *read_pos = buf;
	size_t field_size;
	int ret;

	// do {
	// 	ret = i2c_master_recv(client, buf, sizeof(buf));
	// 	if (!retries--)
	// 		break;
	// } while (ret == -ETIMEDOUT);

    ret = i2c_master_receive(client, buf, sizeof(buf), -1);

	// if (ret != sizeof(buf))
	// 	return ret < 0 ? ret : -EIO;
    if (ret) {
        ESP_LOGE(TAG, "i2c receive error, ret=%d\n", ret);
    }


	field_size = sizeof_field(struct rp2040_gbdg_device_info, status);
	memcpy(&info->status, read_pos, field_size);
	read_pos += field_size;

	field_size = sizeof_field(struct rp2040_gbdg_device_info, md5);
	memcpy(&info->md5, read_pos, field_size);
	read_pos += field_size;

	field_size = sizeof_field(struct rp2040_gbdg_device_info, version);
	memcpy(&info->version, read_pos, field_size);
	read_pos += field_size;

	field_size = sizeof_field(struct rp2040_gbdg_device_info, id);
	memcpy(&info->id, read_pos, field_size);

	return 0;
}


static int rp2040_gbdg_poll_device_info(struct rp2040_gbdg_device_info *info)
{
	struct rp2040_gbdg_device_info itnl;
	int ret;

	itnl.status = STATUS_BUSY;

	while (itnl.status & STATUS_BUSY) {
		ret = rp2040_gbdg_get_device_info(client_handle, &itnl);
		if (ret)
			return ret;
	}
	memcpy(info, &itnl, sizeof(itnl));

	return 0;
}

static int rp2040_gbdg_get_buffer_hash(uint8_t *md5)
{
	struct rp2040_gbdg_device_info info;
	int ret;

	ret = rp2040_gbdg_poll_device_info(&info);
	if (ret)
		return ret;

	memcpy(md5, info.md5, MD5_DIGEST_SIZE);

	return 0;
}


static int rp2040_gbdg_wait_until_free(uint8_t *status)
{
	struct rp2040_gbdg_device_info info;
	int ret;

	ret = rp2040_gbdg_poll_device_info(&info);
	if (ret)
		return ret;

	if (status)
		*status = info.status;

	return 0;
}


static int rp2040_gbdg_i2c_send(const uint8_t *buf, size_t len)
{
	// uint8_t retries = I2C_RETRIES;
	int ret;

	ret = rp2040_gbdg_wait_until_free(NULL);
	if (ret) {
		ESP_LOGE(TAG,"%s() rp2040_gbdg_wait_until_free failed\n", __func__);
		return ret;
	}

	// do {
	// 	ret = i2c_master_send(client, buf, len);
	// 	if (!retries--)
	// 		break;
	// } while (ret == -ETIMEDOUT);
    ret = i2c_master_transmit(client_handle, buf, len, -1);

	// if (ret != len) {
	// 	dev_err(&client->dev, "%s() i2c_master_send returned %d\n",
	// 		__func__, ret);
	// 	return ret < 0 ? ret : -EIO;
	// }
    if (ret) {
        ESP_LOGE(TAG, "i2c transmit error, ret=%d\n", ret);
        return ret;
    }


	return 0;
}

static int rp2040_gbdg_10byte_cmd(uint8_t cmd, uint32_t addr, uint32_t len)
{
	uint8_t buffer[10];

	buffer[0] = FIXED_SIZE_CMD_PREFIX;
	buffer[1] = cmd;
	memcpy(&buffer[2], &addr, sizeof(addr));
	memcpy(&buffer[6], &len, sizeof(len));

	return rp2040_gbdg_i2c_send(buffer, sizeof(buffer));
}


static int rp2040_gbdg_18byte_cmd(uint8_t cmd, const uint8_t *digest)
{
	uint8_t buffer[18];

	buffer[0] = FIXED_SIZE_CMD_PREFIX;
	buffer[1] = cmd;
	memcpy(&buffer[2], digest, MD5_DIGEST_SIZE);

	return rp2040_gbdg_i2c_send(buffer, sizeof(buffer));
}


static int rp2040_gbdg_block_hash(struct rp2040_gbdg *priv_data, const uint8_t *data,
				  size_t len, uint8_t *out)
{
	size_t remaining = RP2040_GBDG_BLOCK_SIZE;
	size_t pad;
	// int ret;

	static const uint8_t padding[64] = {
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF,
	};

	if (len > RP2040_GBDG_BLOCK_SIZE) {
		return -90;	//-EMSGSIZE;
	} else if (len == RP2040_GBDG_BLOCK_SIZE) {
		// return crypto_shash_digest(priv_data->shash_desc, data, len,
		// 			   out);
        mbedtls_md5(data, len, out);
        return 0;
		// return mbedtls_md5(data, len, out);
	} else {
		mbedtls_md5_context ctx;
		mbedtls_md5_init(&ctx);

		// ret = crypto_shash_init(priv_data->shash_desc);
		// ret = mbedtls_md5_starts(&ctx);
        mbedtls_md5_starts(&ctx);
		// if (ret)
		// 	return ret;

		// ret = crypto_shash_update(priv_data->shash_desc, data, len);
        mbedtls_md5_update(&ctx, data, len);
		// ret = mbedtls_md5_update(&ctx, data, len);
		// if (ret) {
		// 	mbedtls_md5_free(&ctx);
		// 	return ret;
		// }
		remaining -= len;

		/* Pad up-to a 64-byte boundary, unless that takes us over. */
		pad = round_up(len, 64);
		if (pad != len && pad < RP2040_GBDG_BLOCK_SIZE) {
			// ret = crypto_shash_update(priv_data->shash_desc,
			// 			  padding, pad - len);
            mbedtls_md5_update(&ctx, padding, pad - len);
			// ret = mbedtls_md5_update(&ctx, padding, pad - len);
			// if (ret) {
			// 	mbedtls_md5_free(&ctx);
			// 	return ret;
			// }
			remaining -= (pad - len);
		}

		/* Pad up-to RP2040_GBDG_BLOCK_SIZE in, preferably, 64-byte chunks */
		while (remaining) {
			pad = MIN(remaining, (size_t)64U);
			// ret = crypto_shash_update(priv_data->shash_desc,
			// 			  padding, pad);
            mbedtls_md5_update(&ctx, padding, pad);
			// ret = mbedtls_md5_update(&ctx, padding, pad);
			// if (ret) {
			// 	mbedtls_md5_free(&ctx);
			// 	return ret;
			// }
			remaining -= pad;
		}
		// return crypto_shash_final(priv_data->shash_desc, out);
        mbedtls_md5_finish(&ctx, out);
		// ret = mbedtls_md5_finish(&ctx, out);
		mbedtls_md5_free(&ctx);
		return 0;
	}
}

static int rp2040_gbdg_set_remote_buffer_fast(struct rp2040_gbdg *priv_data,
					      const uint8_t *data, unsigned int len)
{
	// struct i2c_client *client = priv_data->client;
	int ret;

	if (len > RP2040_GBDG_BLOCK_SIZE)
		return -90;	//-EMSGSIZE;
	if (!priv_data->fast_xfer_gpios)
		return -5;	//-EIO;

	ret = rp2040_gbdg_10byte_cmd(CMD_DAT_RECV, priv_data->fast_xfer_recv_gpio_base, len);
	if (ret) {
		ESP_LOGE(TAG, "%s() failed to enter fast data mode\n", __func__);
		return ret;
	}

	return rp2040_gbdg_fast_xfer(priv_data, data, len);
}

static int rp2040_gbdg_set_remote_buffer_i2c(struct rp2040_gbdg *priv_data,
					     const uint8_t *data, unsigned int len)
{
	// struct i2c_client *client = priv_data->client;
	unsigned int write_len;
	int ret;

	if (len > RP2040_GBDG_BLOCK_SIZE)
		return -90;	//-EMSGSIZE;

	priv_data->buffer[0] = WRITE_DATA_PREFIX;
	write_len = MIN(len, HALF_BUFFER);
	memcpy(&priv_data->buffer[1], data, write_len);

	ret = rp2040_gbdg_i2c_send(priv_data->buffer, write_len + 1);
	if (ret)
		return ret;

	len -= write_len;
	data += write_len;

	if (!len)
		return 0;

	priv_data->buffer[0] = WRITE_DATA_UPPER_PREFIX;
	memcpy(&priv_data->buffer[1], data, len);
	ret = rp2040_gbdg_i2c_send(priv_data->buffer, len + 1);

	return ret;
}

static int rp2040_gbdg_set_remote_buffer(struct rp2040_gbdg *priv_data,
					 const uint8_t *data, unsigned int len)
{
	if (priv_data->fast_xfer_gpios)
		return rp2040_gbdg_set_remote_buffer_fast(priv_data, data, len);
	else
		return rp2040_gbdg_set_remote_buffer_i2c(priv_data, data, len);
}

/* Loads data by checksum if available or resorts to sending byte-by-byte */
static int rp2040_gbdg_load_block_remote(struct rp2040_gbdg *priv_data,
					 const void *data, unsigned int len,
					 uint8_t *digest, bool persist)
{
	uint8_t ascii_digest[MD5_DIGEST_SIZE * 2 + 1] = { 0 };
	// struct i2c_client *client = priv_data->client;
	uint8_t remote_digest[MD5_DIGEST_SIZE];
	uint8_t local_digest[MD5_DIGEST_SIZE];
	int ret;

	if (len > RP2040_GBDG_BLOCK_SIZE)
		return -90;	//-EMSGSIZE;

	ret = rp2040_gbdg_block_hash(priv_data, data, len, local_digest);
	if (ret)
		return ret;

	if (digest)
		memcpy(digest, local_digest, MD5_DIGEST_SIZE);

	/* Check if the RP2040 has the data already */
	ret = rp2040_gbdg_18byte_cmd(CMD_READ_CSUM, local_digest);
	if (ret)
		return ret;

	ret = rp2040_gbdg_get_buffer_hash(remote_digest);
	if (ret)
		return ret;

	if (memcmp(local_digest, remote_digest, MD5_DIGEST_SIZE)) {
		bin2hex((char *)ascii_digest, local_digest, MD5_DIGEST_SIZE);
		ESP_LOGI(TAG, "%s() device missing data: %s\n", __func__, ascii_digest);
		/*
		 * N.B. We're fine to send (the potentially shorter) transfer->len
		 * number of bytes here as the RP2040 will pad with 0xFF up to buffer
		 * size once we stop sending.
		 */
		ret = rp2040_gbdg_set_remote_buffer(priv_data, data, len);
		if (ret)
			return ret;

		/* Make sure the data actually arrived. */
		ret = rp2040_gbdg_get_buffer_hash(remote_digest);
		if (memcmp(local_digest, remote_digest, MD5_DIGEST_SIZE)) {
			ESP_LOGE(TAG, "%s() unable to send data to device\n", __func__);
			return -121;	//-EREMOTEIO;
		}

		if (persist) {
			ESP_LOGE(TAG, "%s() sent missing data to device, saving\n", __func__);
			ret = rp2040_gbdg_10byte_cmd(CMD_SAVE_CACHE, 0,
						     0);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int rp2040_gbdg_transfer_block(struct rp2040_gbdg *priv_data,
				      const void *data, unsigned int len)
{
	// struct i2c_client *client = priv_data->client;
	int ret;

	if (len > RP2040_GBDG_BLOCK_SIZE)
		return -90;	//-EMSGSIZE;

	ret = rp2040_gbdg_load_block_remote(priv_data, data, len, NULL, true);
	if (ret)
		return ret;

	/* Remote rambuffer now has correct contents, send it */
	ret = rp2040_gbdg_10byte_cmd(CMD_SEND_RB, 0, len);
	if (ret)
		return ret;

	/*
	 * Wait for data to have actually completed sending as we may be de-asserting CS too quickly
	 * otherwise.
	 */
	ret = rp2040_gbdg_wait_until_free(NULL);
	if (ret)
		return ret;

	return 0;
}

static int rp2040_gbdg_transfer_manifest(struct rp2040_gbdg *priv_data,
					 const uint8_t *data, unsigned int len)
{
	// struct i2c_client *client = priv_data->client;
	static const char magic[] = "DATA_MANFST";
	unsigned int remaining = len;
	const uint32_t data_length = len;
	uint8_t digest[MD5_DIGEST_SIZE];
	uint8_t *digest_write_pos;
	uint8_t status;
	int ret;

	memcpy(priv_data->manifest_prep, magic, sizeof(magic));
	memcpy(priv_data->manifest_prep + sizeof(magic), &data_length,
	       sizeof(data_length));
	digest_write_pos =
		priv_data->manifest_prep + sizeof(magic) + sizeof(data_length);

	while (remaining) {
		unsigned int size = MIN(remaining, RP2040_GBDG_BLOCK_SIZE);

		ret = rp2040_gbdg_block_hash(priv_data, data, size,
					     digest_write_pos);
		if (ret)
			return ret;

		remaining -= size;
		data += size;
		digest_write_pos += MD5_DIGEST_SIZE;
	}

	ret = rp2040_gbdg_load_block_remote(
		priv_data, priv_data->manifest_prep,
		digest_write_pos - priv_data->manifest_prep, digest, true);
	if (ret)
		return ret;

	ESP_LOGI(TAG, "%s() issue CMD_SEND_MANI", __func__);
	ret = rp2040_gbdg_18byte_cmd(CMD_SEND_MANI, digest);
	if (ret)
		return ret;

	ret = rp2040_gbdg_wait_until_free(&status);
	if (ret)
		return ret;

	ESP_LOGI(TAG, "%s() SEND_MANI response: %02x", __func__, status);

	return status;
}

/* Precondition: correctly initialised fast_xfer_*, gpio_base, rio_base */
static int rp2040_gbdg_fast_xfer(struct rp2040_gbdg *priv_data, const uint8_t *data,
				 size_t len)
{
	// struct i2c_client *client = priv_data->client;
	void *clock_toggle;
	void *data_set;
	size_t clock_bank;
	size_t data_bank;
	uint8_t clock_offset;
	uint8_t data_offset;
	uint32_t clock_mux;
	uint32_t data_mux;

	// if (priv_data->fast_xfer_requires_i2c_lock)
	// 	i2c_lock_bus(client->adapter, I2C_LOCK_ROOT_ADAPTER);

	rp2040_gbdg_rp1_read_mux(priv_data, priv_data->fast_xfer_data_index,
				 &data_mux);
	rp2040_gbdg_rp1_read_mux(priv_data, priv_data->fast_xfer_clock_index,
				 &clock_mux);

	// gpiod_direction_output(priv_data->fast_xfer_gpios->desc[0], 1);

	rp2040_gbdg_rp1_calc_offsets(priv_data->fast_xfer_data_index,
				     &data_bank, &data_offset);
	rp2040_gbdg_rp1_calc_offsets(priv_data->fast_xfer_clock_index,
				     &clock_bank, &clock_offset);

	data_set = priv_data->rio_base + data_bank + 0x2000; /* SET offset */
	clock_toggle =
		priv_data->rio_base + clock_bank + 0x1000; /* XOR offset */

	while (len--) {
		/* MSB first ordering */
		uint32_t d = ~(*data++) << 4U;
		/*
		 * Clock out each bit of data, LSB first
		 * (DDR, achieves approx 5 Mbps)
		 */
		for (size_t i = 0; i < 8; i++) {
			/* Branchless set/clr data */
			// writel(1 << data_offset,
			//        data_set + ((d <<= 1) & 0x1000) /* CLR offset */
			// );
			*((volatile uint32_t *)(data_set + ((d <<= 1) & 0x1000))) = (1 << data_offset);

			/* Toggle the clock */
			// writel(1 << clock_offset, clock_toggle);
			*((volatile uint32_t *)(clock_toggle)) = (1 << clock_offset);
		}
	}

	rp2040_gbdg_rp1_write_mux(priv_data, priv_data->fast_xfer_data_index,
				  data_mux);
	rp2040_gbdg_rp1_write_mux(priv_data, priv_data->fast_xfer_clock_index,
				  clock_mux);

	// if (priv_data->fast_xfer_requires_i2c_lock)
	// 	i2c_unlock_bus(client->adapter, I2C_LOCK_ROOT_ADAPTER);

	return 0;
}

static int rp2040_gbdg_transfer_bypass(struct rp2040_gbdg *priv_data,
				       const uint8_t *data, unsigned int length)
{
	int ret;
	uint8_t *buf;

	if (priv_data->fast_xfer_gpios) {
		ret = rp2040_gbdg_10byte_cmd(CMD_DAT_EMIT,
			priv_data->fast_xfer_recv_gpio_base, length);
		return ret ? ret :
			     rp2040_gbdg_fast_xfer(priv_data, data, length);
	}

	buf = priv_data->buffer;

	while (length) {
		unsigned int xfer = MIN(length, HALF_BUFFER);

		buf[0] = DIRECT_PREFIX;
		buf[1] = DIRECT_CMD_EMIT;
		memcpy(&buf[2], data, xfer);
		ret = rp2040_gbdg_i2c_send(buf, xfer + 2);
		if (ret)
			return ret;
		length -= xfer;
		data += xfer;
	}

	return 0;
}

static int rp2040_gbdg_transfer_cached(struct rp2040_gbdg *priv_data,
				       const uint8_t *data, unsigned int length)
{
	int ret;

	/*
	 * Caching mechanism divides data into '8KiB - 9' (8183 byte)
	 * 'RP2040_GBDG_BLOCK_SIZE' blocks.
	 *
	 * If there's a large amount of data to send, instead, attempt to make use
	 * of a manifest.
	 */
	if (length > (2 * RP2040_GBDG_BLOCK_SIZE)) {
		if (!rp2040_gbdg_transfer_manifest(priv_data, data, length)) {
			// ESP_LOGI(TAG, "rp2040_gbdg_transfer_manifest data=%p, length=%d", data, length);
			return 0;
		}
	}

	priv_data->transfer_progress = 0;
	while (length) {
		unsigned int xfer = MIN(length, RP2040_GBDG_BLOCK_SIZE);

		ret = rp2040_gbdg_transfer_block(priv_data, data, xfer);
		if (ret)
			return ret;
		length -= xfer;
		data += xfer;
		priv_data->transfer_progress += xfer;
		// ESP_LOGI(TAG, "rp2040_gbdg_transfer_block xfer=%p, length=%d", xfer, length);
	}
	priv_data->transfer_progress = 0;

	return 0;
}

// int rp2040_gbdg_transfer_one(struct spi_controller *ctlr,
// 				    struct spi_device *spi,
// 				    struct spi_transfer *transfer)
int rp2040_gbdg_transfer_one(const uint8_t *tx_buf, size_t len)
{
	/* All transfers are performed in a synchronous manner. As such, return '0'
	 * on success or -ve on failure. (Returning +ve indicates async xfer)
	 */

	struct rp2040_gbdg *priv_data = rp2040_dev; // spi_controller_get_devdata(ctlr);

	if (priv_data->bypass_cache) {
		ESP_LOGI(TAG, "transfer bypass_cache len=%d", len);
		return rp2040_gbdg_transfer_bypass(priv_data, tx_buf, len);
	} else {
		ESP_LOGI(TAG, "transfer cached len=%d", len);
		return rp2040_gbdg_transfer_cached(priv_data, tx_buf, len);
	}
}


void rp2040_gbdg_set_cs(bool enable)
{
	static const uint8_t disable_cs[] = { DIRECT_PREFIX, DIRECT_CMD_CS, 0x00 };
	static const uint8_t enable_cs[] = { DIRECT_PREFIX, DIRECT_CMD_CS, 0x10 };
	// struct rp2040_gbdg *p_data;

	// p_data = spi_controller_get_devdata(spi->controller);

	/*
	 * 'enable' is inverted and instead describes the logic level of an
	 * active-low CS.
	 */
	// ESP_LOGI(TAG, "set cs enable=%d\n", enable);
	rp2040_gbdg_i2c_send(enable ? disable_cs : enable_cs, 3);
}


/********************************************************************* */
// Add near the top of your file, before usage
#define GPIO_LINE_DIRECTION_IN   1
#define GPIO_LINE_DIRECTION_OUT  0

static int rp2040_gbdg_gpio_get_direction(unsigned int offset)
{
	// struct rp2040_gbdg *priv_data = gpiochip_get_data(gc);

	if (offset >= NUM_GPIO)
		return -22;	//-EINVAL;

	return (gpio_direction & (1 << (offset + 8))) ?
		       GPIO_LINE_DIRECTION_IN :
		       GPIO_LINE_DIRECTION_OUT;
}

static int rp2040_gbdg_gpio_dir_in(unsigned int offset)
{
	// struct rp2040_gbdg *priv_data = gpiochip_get_data(gc);
	// struct i2c_client *client = priv_data->client;

	if (offset >= NUM_GPIO)
		return -22;	//-EINVAL;

	gpio_direction |= (1 << (offset + 8));

	return rp2040_gbdg_10byte_cmd(CMD_GPIO_OE, ~gpio_direction, gpio_direction);
}

int rp2040_gbdg_gpio_dir_out(unsigned int offset, int value)
{
	// struct rp2040_gbdg *priv_data = gpiochip_get_data(gc);
	// struct i2c_client *client = priv_data->client;
	uint32_t pattern;
	int ret;

	if (offset >= NUM_GPIO) {
		return -22;	//-EINVAL;
    }

	pattern = (1 << (offset + 8));

	ret = rp2040_gbdg_10byte_cmd(CMD_GPIO_ST_CL, value ? pattern : 0, !value ? pattern : 0);
	if (ret) {
		ESP_LOGE(TAG, "%s(%u, %d) could not ST_CL\n", __func__, offset, value);
		return ret;
	}

	gpio_direction &= ~pattern;
	ret = rp2040_gbdg_10byte_cmd(CMD_GPIO_OE, ~gpio_direction, gpio_direction);

	return ret;
}


static int rp2040_gbdg_gpio_get(unsigned int offset)
{
	// struct rp2040_gbdg *priv_data = gpiochip_get_data(gc);
	// struct i2c_client *client = priv_data->client;
	struct rp2040_gbdg_device_info info;
	int ret;

	if (offset >= NUM_GPIO)
		return -22;	//-EINVAL;

	ret = rp2040_gbdg_get_device_info(client_handle, &info);
	if (ret)
		return ret;

	return info.status & (1 << (offset + 8)) ? 1 : 0;
}

static void rp2040_gbdg_gpio_set(unsigned int offset, int value)
{
	// struct rp2040_gbdg *priv_data = gpiochip_get_data(gc);
	// struct i2c_client *client = priv_data->client;
	uint32_t pattern;

	if (offset >= NUM_GPIO) 
		return;

	pattern = (1 << (offset + 8));
	rp2040_gbdg_10byte_cmd(CMD_GPIO_ST_CL, value ? pattern : 0,
			       !value ? pattern : 0);
}

#if 1
static void rp2040_gbdg_parse_dt(struct rp2040_gbdg *rp2040_gbdg)
{
	// struct i2c_client *client = rp2040_gbdg->client;
	// struct of_phandle_args of_args[2] = { 0 };
	// struct device *dev = &client->dev;
	// struct device_node *dn;

	rp2040_gbdg->bypass_cache = false;
		// of_property_read_bool(client->dev.of_node, "bypass-cache");

	/* Optionally configure fast_xfer if RP1 is being used */
	// if (of_parse_phandle_with_args(client->dev.of_node, "fast_xfer-gpios",
	// 			       "#gpio-cells", 0, &of_args[0]) ||
	//     of_parse_phandle_with_args(client->dev.of_node, "fast_xfer-gpios",
	// 			       "#gpio-cells", 1, &of_args[1])) {
	// 	dev_info(dev, "Could not parse fast_xfer-gpios phandles\n");
	// 	goto node_put;
	// }

	// if (of_args[0].np != of_args[1].np) {
	// 	dev_info(
	// 		dev,
	// 		"fast_xfer-gpios are not provided by the same controller\n");
	// 	goto node_put;
	// }
	// dn = of_args[0].np;
	// if (!of_device_is_compatible(dn, "raspberrypi,rp1-gpio")) {
	// 	dev_info(dev, "fast_xfer-gpios controller is not an rp1\n");
	// 	goto node_put;
	// }
	// if (of_args[0].args_count != 2 || of_args[1].args_count != 2) {
	// 	dev_info(dev, "of_args count is %d\n", of_args[0].args_count);
	// 	goto node_put;
	// }

	// if (of_property_read_u32_index(
	// 	    client->dev.of_node, "fast_xfer_recv_gpio_base", 0,
	// 	    &rp2040_gbdg->fast_xfer_recv_gpio_base)) {
	// 	dev_info(dev, "Could not read fast_xfer_recv_gpio_base\n");
	// 	goto node_put;
	// }

	// rp2040_gbdg->fast_xfer_gpios =
	// 	devm_gpiod_get_array_optional(dev, "fast_xfer", GPIOD_ASIS);
	// if (IS_ERR_OR_NULL(rp2040_gbdg->fast_xfer_gpios)) {
	// 	rp2040_gbdg->fast_xfer_gpios = NULL;
	// 	dev_info(dev, "Could not acquire fast_xfer-gpios\n");
	// 	goto node_put;
	// }

	// rp2040_gbdg->fast_xfer_data_index = of_args[0].args[0];
	// rp2040_gbdg->fast_xfer_clock_index = of_args[1].args[0];
	// rp2040_gbdg->fast_xfer_requires_i2c_lock = of_property_read_bool(
	// 	client->dev.of_node, "fast_xfer_requires_i2c_lock");

	// rp2040_gbdg->gpio_base = of_iomap(dn, 0);
	// if (IS_ERR_OR_NULL(rp2040_gbdg->gpio_base)) {
	// 	dev_info(&client->dev, "%s() unable to map gpio_base\n",
	// 		 __func__);
	// 	rp2040_gbdg->gpio_base = NULL;
	// 	devm_gpiod_put_array(dev, rp2040_gbdg->fast_xfer_gpios);
	// 	rp2040_gbdg->fast_xfer_gpios = NULL;
	// 	goto node_put;
	// }

	// rp2040_gbdg->rio_base = of_iomap(dn, 1);
	// if (IS_ERR_OR_NULL(rp2040_gbdg->rio_base)) {
	// 	dev_info(&client->dev, "%s() unable to map rio_base\n",
	// 		 __func__);
	// 	rp2040_gbdg->rio_base = NULL;
	// 	iounmap(rp2040_gbdg->gpio_base);
	// 	rp2040_gbdg->gpio_base = NULL;
	// 	devm_gpiod_put_array(dev, rp2040_gbdg->fast_xfer_gpios);
	// 	rp2040_gbdg->fast_xfer_gpios = NULL;
	// 	goto node_put;
	// }

	/*
	 * fast_xfer mode requires first data bit to be clocked on a rising
	 * edge. Configure as output-low here before fast_xfer mode is entered.
	 */
	// gpiod_direction_output(rp2040_gbdg->fast_xfer_gpios->desc[1], 0);
// node_put:
	// if (of_args[0].np)
	// 	of_node_put(of_args[0].np);
	// if (of_args[1].np)
	// 	of_node_put(of_args[1].np);
}
#endif

/*
static int rp2040_gbdg_power_off(struct rp2040_gbdg *rp2040_gbdg)
{
	// struct device *dev = &rp2040_gbdg->client->dev;
	// int ret;

	// ret = regulator_disable(rp2040_gbdg->regulator);
	// if (ret) {
	// 	dev_err(dev, "%s: Could not disable regulator\n", __func__);
	// 	return ret;
	// }

	return 0;
}

static int rp2040_gbdg_power_on(struct rp2040_gbdg *rp2040_gbdg)
{
	// struct device *dev = &rp2040_gbdg->client->dev;
	// int ret;

	// ret = regulator_enable(rp2040_gbdg->regulator);
	// if (ret) {
	// 	dev_err(dev, "%s: Could not enable regulator\n", __func__);
	// 	return ret;
	// }

	return 0;
}
*/

int rp2040_gbdg_init(i2c_master_bus_handle_t bus_handle)
// int rp2040_gbdg_init(i2c_port_num_t i2c_port_num)
{
	struct rp2040_gbdg_device_info info;
	// i2c_master_bus_handle_t bus_handle;
	// struct spi_controller *controller;
	// struct device *dev = &client->dev;
	// struct rp2040_gbdg *rp2040_gbdg;
	// struct device_node *np;
	// char debugfs_name[128];
	int ret;

	// i2c_master_get_bus_handle(i2c_port_num, &bus_handle);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x0c,
        .scl_speed_hz = 100000,

    };
    // i2c_master_dev_handle_t client_handle = NULL;
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &client_handle);
    if (ret) {
        ESP_LOGE(TAG, "failed to add device");
    }

    // ret = rp2040_gbdg_power_on(rp2040_gbdg);
	// if (ret)
	// 	return dev_err_probe(dev, ret, "Could not power on device\n");

	rp2040_dev = heap_caps_calloc(1, sizeof(struct rp2040_gbdg), MALLOC_CAP_DEFAULT); 
	if (!rp2040_dev) {
		ESP_LOGE(TAG, "Failed to allocate memory for rp2040_devdata");
		return -1;
	}

  	ret = rp2040_gbdg_get_device_info(client_handle, &info);
	if (ret) {
        ESP_LOGE(TAG, "Could not get device info\n");
		// goto err_pm;
	}

    ESP_LOGI(TAG, "%s() found dev ID: %llx, fw ver. %u\n", __func__,
		 info.id, info.version);

    gpio_direction = ~0;

	rp2040_gbdg_set_cs(1);

	rp2040_gbdg_parse_dt(rp2040_dev);

    return 0;
}


/*
static int rp2040_gbdg_probe(struct i2c_client *client)
{
	struct rp2040_gbdg_device_info info;
	struct spi_controller *controller;
	struct device *dev = &client->dev;
	struct rp2040_gbdg *rp2040_gbdg;
	struct device_node *np;
	char debugfs_name[128];
	int ret;

	np = dev->of_node;

	controller = devm_spi_alloc_master(dev, sizeof(struct rp2040_gbdg));
	if (!controller)
		return dev_err_probe(dev, ENOMEM,
				     "could not alloc spi controller\n");

	rp2040_gbdg = spi_controller_get_devdata(controller);
	i2c_set_clientdata(client, rp2040_gbdg);
	rp2040_gbdg->controller = controller;
	rp2040_gbdg->client = client;

	ret = rp2040_gbdg_get_regulator(dev, rp2040_gbdg);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Cannot get regulator\n");

	ret = rp2040_gbdg_power_on(rp2040_gbdg);
	if (ret)
		return dev_err_probe(dev, ret, "Could not power on device\n");

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = rp2040_gbdg_get_device_info(client, &info);
	if (ret) {
		dev_err(dev, "Could not get device info\n");
		goto err_pm;
	}

	dev_info(dev, "%s() found dev ID: %llx, fw ver. %u\n", __func__,
		 info.id, info.version);

	rp2040_gbdg->shash = crypto_alloc_shash("md5", 0, 0);
	if (IS_ERR(rp2040_gbdg->shash)) {
		ret = PTR_ERR(rp2040_gbdg->shash);
		dev_err(dev, "Could not allocate shash\n");
		goto err_pm;
	}

	if (crypto_shash_digestsize(rp2040_gbdg->shash) != MD5_DIGEST_SIZE) {
		ret = -EINVAL;
		dev_err(dev, "error: Unexpected hash digest size\n");
		goto err_shash;
	}

	rp2040_gbdg->shash_desc =
		devm_kmalloc(dev,
			     sizeof(struct shash_desc) +
				     crypto_shash_descsize(rp2040_gbdg->shash),
			     0);

	if (!rp2040_gbdg->shash_desc) {
		ret = -ENOMEM;
		dev_err(dev,
			"error: Could not allocate memory for shash_desc\n");
		goto err_shash;
	}
	rp2040_gbdg->shash_desc->tfm = rp2040_gbdg->shash;

	controller->bus_num = -1;
	controller->num_chipselect = 1;
	controller->mode_bits = SPI_CPOL | SPI_CPHA;
	controller->bits_per_word_mask = SPI_BPW_MASK(8);
	controller->min_speed_hz = 35000000;
	controller->max_speed_hz = 35000000;
	controller->max_transfer_size = rp2040_gbdg_max_transfer_size;
	controller->max_message_size = rp2040_gbdg_max_transfer_size;
	controller->transfer_one = rp2040_gbdg_transfer_one;
	controller->set_cs = rp2040_gbdg_set_cs;

	controller->dev.of_node = np;
	controller->auto_runtime_pm = true;

	ret = devm_spi_register_controller(dev, controller);
	if (ret) {
		dev_err(dev, "error: Could not register SPI controller\n");
		goto err_shash;
	}

	memset(&rp2040_gbdg->gc, 0, sizeof(struct gpio_chip));
	rp2040_gbdg->gc.parent = dev;
	rp2040_gbdg->gc.label = MODULE_NAME;
	rp2040_gbdg->gc.owner = THIS_MODULE;
	rp2040_gbdg->gc.base = -1;
	rp2040_gbdg->gc.ngpio = NUM_GPIO;

	rp2040_gbdg->gc.request = rp2040_gbdg_gpio_request;
	rp2040_gbdg->gc.free = rp2040_gbdg_gpio_free;
	rp2040_gbdg->gc.get_direction = rp2040_gbdg_gpio_get_direction;
	rp2040_gbdg->gc.direction_input = rp2040_gbdg_gpio_dir_in;
	rp2040_gbdg->gc.direction_output = rp2040_gbdg_gpio_dir_out;
	rp2040_gbdg->gc.get = rp2040_gbdg_gpio_get;
	rp2040_gbdg->gc.set = rp2040_gbdg_gpio_set;
	rp2040_gbdg->gc.can_sleep = true;

	rp2040_gbdg->gpio_requested = 0;

	/ * Coming out of reset, all GPIOs are inputs * /
	rp2040_gbdg->gpio_direction = ~0;

	ret = devm_gpiochip_add_data(dev, &rp2040_gbdg->gc, rp2040_gbdg);
	if (ret) {
		dev_err(dev, "error: Could not add data to gpiochip\n");
		goto err_shash;
	}

	rp2040_gbdg_parse_dt(rp2040_gbdg);

	snprintf(debugfs_name, sizeof(debugfs_name), "rp2040-spi:%s",
		 dev_name(dev));
	rp2040_gbdg->debugfs = debugfs_create_dir(debugfs_name, NULL);
	debugfs_create_file("transfer_progress", 0444, rp2040_gbdg->debugfs,
			    rp2040_gbdg, &transfer_progress_fops);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

err_shash:
	crypto_free_shash(rp2040_gbdg->shash);
err_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	rp2040_gbdg_power_off(rp2040_gbdg);

	return ret;
}
*/