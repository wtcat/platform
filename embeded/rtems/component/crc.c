/*
 * Copyright (c) 2018 Google LLC.
 * Copyright (c) 2020 Intel Corporation
 * Copyright (c) 2017 Nordic Semiconductor ASA
 * Copyright (c) 2015 Runtime Inc
 * Copyright (c) 2017 Intel Corporation.
 * Copyright (c) 2018 Workaround GmbH.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "component/crc.h"

static const uint8_t crc8_ccitt_small_table[16] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
	0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d
};

uint8_t crc7_be(uint8_t seed, const uint8_t *src, size_t len) {
	while (len--) {
		uint8_t e = seed ^ *src++;
		uint8_t f = e ^ (e >> 4) ^ (e >> 7);

		seed = (f << 1) ^ (f << 4);
	}
	return seed;
}

uint8_t crc8_ccitt(uint8_t val, const void *buf, size_t cnt) {
	size_t i;
	const uint8_t *p = buf;

	for (i = 0; i < cnt; i++) {
		val ^= p[i];
		val = (val << 4) ^ crc8_ccitt_small_table[val >> 4];
		val = (val << 4) ^ crc8_ccitt_small_table[val >> 4];
	}
	return val;
}

uint8_t crc8(const uint8_t *src, size_t len, uint8_t polynomial, uint8_t initial_value,
	  bool reversed) {
	uint8_t crc = initial_value;
	size_t i, j;

	for (i = 0; i < len; i++) {
		crc ^= src[i];

		for (j = 0; j < 8; j++) {
			if (reversed) {
				if (crc & 0x01) {
					crc = (crc >> 1) ^ polynomial;
				} else {
					crc >>= 1;
				}
			} else {
				if (crc & 0x80) {
					crc = (crc << 1) ^ polynomial;
				} else {
					crc <<= 1;
				}
			}
		}
	}
	return crc;
}

uint16_t crc16(uint16_t poly, uint16_t seed, const uint8_t *src, size_t len) {
	uint16_t crc = seed;
	size_t i, j;

	for (i = 0; i < len; i++) {
		crc ^= (uint16_t)(src[i] << 8U);

		for (j = 0; j < 8; j++) {
			if (crc & 0x8000UL) {
				crc = (crc << 1U) ^ poly;
			} else {
				crc = crc << 1U;
			}
		}
	}
	return crc;
}

uint16_t crc16_reflect(uint16_t poly, uint16_t seed, const uint8_t *src, size_t len) {
	uint16_t crc = seed;
	size_t i, j;

	for (i = 0; i < len; i++) {
		crc ^= (uint16_t)src[i];

		for (j = 0; j < 8; j++) {
			if (crc & 0x0001UL) {
				crc = (crc >> 1U) ^ poly;
			} else {
				crc = crc >> 1U;
			}
		}
	}
	return crc;
}

uint16_t crc16_ccitt(uint16_t seed, const uint8_t *src, size_t len) {
	for (; len > 0; len--) {
		uint8_t e, f;

		e = seed ^ *src++;
		f = e ^ (e << 4);
		seed = (seed >> 8) ^ ((uint16_t)f << 8) ^ ((uint16_t)f << 3) ^ ((uint16_t)f >> 4);
	}
	return seed;
}

uint16_t crc16_itu_t(uint16_t seed, const uint8_t *src, size_t len) {
	for (; len > 0; len--) {
		seed = (seed >> 8U) | (seed << 8U);
		seed ^= *src++;
		seed ^= (seed & 0xffU) >> 4U;
		seed ^= seed << 12U;
		seed ^= (seed & 0xffU) << 5U;
	}
	return seed;
}

uint32_t crc32_ieee(const uint8_t *data, size_t len) {
	return crc32_ieee_update(0x0, data, len);
}

uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, size_t len) {
	/* crc table generated from polynomial 0xedb88320 */
	static const uint32_t table[16] = {
		0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
		0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
		0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
		0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
	};

	crc = ~crc;

	for (size_t i = 0; i < len; i++) {
		uint8_t byte = data[i];

		crc = (crc >> 4) ^ table[(crc ^ byte) & 0x0f];
		crc = (crc >> 4) ^ table[(crc ^ ((uint32_t)byte >> 4)) & 0x0f];
	}
	return (~crc);
}
