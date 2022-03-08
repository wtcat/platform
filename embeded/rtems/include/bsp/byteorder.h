/*
 * Copyright (c) 2015-2016, Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ASM_BYTEORDER_H_
#define ASM_BYTEORDER_H_

#include <stddef.h>
#include <stdint.h>

#include <rtems/score/assert.h>

#ifdef __cplusplus
extern "C"{
#endif

/* Internal helpers only used by the * APIs further below */
#define __bswap_16(x) __builtin_bswap16((uint16_t)(x))
#define __bswap_32(x) __builtin_bswap16((uint32_t)(x))
#define __bswap_64(x) __builtin_bswap16((uint64_t)(x))

#if (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
#define le16_to_cpu(val) (val)
#define cpu_to_le16(val) (val)
#define le32_to_cpu(val) (val)
#define cpu_to_le32(val) (val)
#define le64_to_cpu(val) (val)
#define cpu_to_le64(val) (val)
#define be16_to_cpu(val) __bswap_16(val)
#define cpu_to_be16(val) __bswap_16(val)
#define be32_to_cpu(val) __bswap_32(val)
#define cpu_to_be32(val) __bswap_32(val)
#define be64_to_cpu(val) __bswap_64(val)
#define cpu_to_be64(val) __bswap_64(val)

#elif (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#define le16_to_cpu(val) __bswap_16(val)
#define cpu_to_le16(val) __bswap_16(val)
#define le32_to_cpu(val) __bswap_32(val)
#define cpu_to_le32(val) __bswap_32(val)
#define le64_to_cpu(val) __bswap_64(val)
#define cpu_to_le64(val) __bswap_64(val)
#define be16_to_cpu(val) (val)
#define cpu_to_be16(val) (val)
#define be32_to_cpu(val) (val)
#define cpu_to_be32(val) (val)
#define be64_to_cpu(val) (val)
#define cpu_to_be64(val) (val)
#else
#error "Unknown byte order"
#endif

/**
 *  @brief Put a 16-bit integer as big-endian to arbitrary location.
 *
 *  Put a 16-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in big-endian format.
 *
 *  @param val 16-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void put_be16(uint16_t val, uint8_t dst[2])
{
	dst[0] = val >> 8;
	dst[1] = val;
}

/**
 *  @brief Put a 24-bit integer as big-endian to arbitrary location.
 *
 *  Put a 24-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in big-endian format.
 *
 *  @param val 24-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void put_be24(uint32_t val, uint8_t dst[3])
{
	dst[0] = val >> 16;
	put_be16(val, &dst[1]);
}

/**
 *  @brief Put a 32-bit integer as big-endian to arbitrary location.
 *
 *  Put a 32-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in big-endian format.
 *
 *  @param val 32-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void put_be32(uint32_t val, uint8_t dst[4])
{
	put_be16(val >> 16, dst);
	put_be16(val, &dst[2]);
}

/**
 *  @brief Put a 48-bit integer as big-endian to arbitrary location.
 *
 *  Put a 48-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in big-endian format.
 *
 *  @param val 48-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void put_be48(uint64_t val, uint8_t dst[6])
{
	put_be16(val >> 32, dst);
	put_be32(val, &dst[2]);
}

/**
 *  @brief Put a 64-bit integer as big-endian to arbitrary location.
 *
 *  Put a 64-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in big-endian format.
 *
 *  @param val 64-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void put_be64(uint64_t val, uint8_t dst[8])
{
	put_be32(val >> 32, dst);
	put_be32(val, &dst[4]);
}

/**
 *  @brief Put a 16-bit integer as little-endian to arbitrary location.
 *
 *  Put a 16-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in little-endian format.
 *
 *  @param val 16-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void put_le16(uint16_t val, uint8_t dst[2])
{
	dst[0] = val;
	dst[1] = val >> 8;
}

/**
 *  @brief Put a 24-bit integer as little-endian to arbitrary location.
 *
 *  Put a 24-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in littel-endian format.
 *
 *  @param val 24-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void put_le24(uint32_t val, uint8_t dst[3])
{
	put_le16(val, dst);
	dst[2] = val >> 16;
}

/**
 *  @brief Put a 32-bit integer as little-endian to arbitrary location.
 *
 *  Put a 32-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in little-endian format.
 *
 *  @param val 32-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void put_le32(uint32_t val, uint8_t dst[4])
{
	put_le16(val, dst);
	put_le16(val >> 16, &dst[2]);
}

/**
 *  @brief Put a 48-bit integer as little-endian to arbitrary location.
 *
 *  Put a 48-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in little-endian format.
 *
 *  @param val 48-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void put_le48(uint64_t val, uint8_t dst[6])
{
	put_le32(val, dst);
	put_le16(val >> 32, &dst[4]);
}

/**
 *  @brief Put a 64-bit integer as little-endian to arbitrary location.
 *
 *  Put a 64-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in little-endian format.
 *
 *  @param val 64-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void put_le64(uint64_t val, uint8_t dst[8])
{
	put_le32(val, dst);
	put_le32(val >> 32, &dst[4]);
}

/**
 *  @brief Get a 16-bit integer stored in big-endian format.
 *
 *  Get a 16-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 16-bit integer to get.
 *
 *  @return 16-bit integer in host endianness.
 */
static inline uint16_t get_be16(const uint8_t src[2])
{
	return ((uint16_t)src[0] << 8) | src[1];
}

/**
 *  @brief Get a 24-bit integer stored in big-endian format.
 *
 *  Get a 24-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 24-bit integer to get.
 *
 *  @return 24-bit integer in host endianness.
 */
static inline uint32_t get_be24(const uint8_t src[3])
{
	return ((uint32_t)src[0] << 16) | get_be16(&src[1]);
}

/**
 *  @brief Get a 32-bit integer stored in big-endian format.
 *
 *  Get a 32-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 32-bit integer to get.
 *
 *  @return 32-bit integer in host endianness.
 */
static inline uint32_t get_be32(const uint8_t src[4])
{
	return ((uint32_t)get_be16(&src[0]) << 16) | get_be16(&src[2]);
}

/**
 *  @brief Get a 48-bit integer stored in big-endian format.
 *
 *  Get a 48-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 48-bit integer to get.
 *
 *  @return 48-bit integer in host endianness.
 */
static inline uint64_t get_be48(const uint8_t src[6])
{
	return ((uint64_t)get_be32(&src[0]) << 16) | get_be16(&src[4]);
}

/**
 *  @brief Get a 64-bit integer stored in big-endian format.
 *
 *  Get a 64-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 64-bit integer to get.
 *
 *  @return 64-bit integer in host endianness.
 */
static inline uint64_t get_be64(const uint8_t src[8])
{
	return ((uint64_t)get_be32(&src[0]) << 32) | get_be32(&src[4]);
}

/**
 *  @brief Get a 16-bit integer stored in little-endian format.
 *
 *  Get a 16-bit integer, stored in little-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the little-endian 16-bit integer to get.
 *
 *  @return 16-bit integer in host endianness.
 */
static inline uint16_t get_le16(const uint8_t src[2])
{
	return ((uint16_t)src[1] << 8) | src[0];
}

/**
 *  @brief Get a 24-bit integer stored in big-endian format.
 *
 *  Get a 24-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 24-bit integer to get.
 *
 *  @return 24-bit integer in host endianness.
 */
static inline uint32_t get_le24(const uint8_t src[3])
{
	return ((uint32_t)src[2] << 16) | get_le16(&src[0]);
}

/**
 *  @brief Get a 32-bit integer stored in little-endian format.
 *
 *  Get a 32-bit integer, stored in little-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the little-endian 32-bit integer to get.
 *
 *  @return 32-bit integer in host endianness.
 */
static inline uint32_t get_le32(const uint8_t src[4])
{
	return ((uint32_t)get_le16(&src[2]) << 16) | get_le16(&src[0]);
}

/**
 *  @brief Get a 48-bit integer stored in little-endian format.
 *
 *  Get a 48-bit integer, stored in little-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the little-endian 48-bit integer to get.
 *
 *  @return 48-bit integer in host endianness.
 */
static inline uint64_t get_le48(const uint8_t src[6])
{
	return ((uint64_t)get_le32(&src[2]) << 16) | get_le16(&src[0]);
}

/**
 *  @brief Get a 64-bit integer stored in little-endian format.
 *
 *  Get a 64-bit integer, stored in little-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the little-endian 64-bit integer to get.
 *
 *  @return 64-bit integer in host endianness.
 */
static inline uint64_t get_le64(const uint8_t src[8])
{
	return ((uint64_t)get_le32(&src[4]) << 32) | get_le32(&src[0]);
}

/**
 * @brief Swap one buffer content into another
 *
 * Copy the content of src buffer into dst buffer in reversed order,
 * i.e.: src[n] will be put in dst[end-n]
 * Where n is an index and 'end' the last index in both arrays.
 * The 2 memory pointers must be pointing to different areas, and have
 * a minimum size of given length.
 *
 * @param dst A valid pointer on a memory area where to copy the data in
 * @param src A valid pointer on a memory area where to copy the data from
 * @param length Size of both dst and src memory areas
 */
static inline void memcpy_swap(void *dst, const void *src, size_t length)
{
	uint8_t *pdst = (uint8_t *)dst;
	const uint8_t *psrc = (const uint8_t *)src;

	_Assert(((psrc < pdst && (psrc + length) <= pdst) ||
		  (psrc > pdst && (pdst + length) <= psrc)))
	psrc += length - 1;
	for (; length > 0; length--) 
		*pdst++ = *psrc--;
}

/**
 * @brief Swap buffer content
 *
 * In-place memory swap, where final content will be reversed.
 * I.e.: buf[n] will be put in buf[end-n]
 * Where n is an index and 'end' the last index of buf.
 *
 * @param buf A valid pointer on a memory area to swap
 * @param length Size of buf memory area
 */
static inline void mem_swap(void *buf, size_t length)
{
	size_t i;

	for (i = 0; i < (length/2); i++) {
		uint8_t tmp = ((uint8_t *)buf)[i];

		((uint8_t *)buf)[i] = ((uint8_t *)buf)[length - 1 - i];
		((uint8_t *)buf)[length - 1 - i] = tmp;
	}
}

#ifdef __cplusplus
}
#endif
#endif /* ASM_BYTEORDER_H_ */
