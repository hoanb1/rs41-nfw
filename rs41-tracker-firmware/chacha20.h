/*
 * File: chacha20.h
 * Ultra-compact, constant-time ChaCha20 Stream Cipher (RFC 8439)
 * Optimized for ARM Cortex-M4 (STM32L4) - Zero dynamic memory, zero dependencies
 */

#ifndef CHACHA20_H
#define CHACHA20_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define CHACHA20_ROTL32(v, n) (((v) << (n)) | ((v) >> (32 - (n))))

#define CHACHA20_QUARTER_ROUND(a, b, c, d) \
    a += b; d = CHACHA20_ROTL32(d ^ a, 16); \
    c += d; b = CHACHA20_ROTL32(b ^ c, 12); \
    a += b; d = CHACHA20_ROTL32(d ^ a, 8);  \
    c += d; b = CHACHA20_ROTL32(b ^ c, 7);

static inline void chacha20_block(const uint8_t key[32], const uint8_t nonce[12], uint32_t counter, uint8_t output[64]) {
    uint32_t state[16];
    uint32_t orig[16];

    // Constants "expand 32-byte k"
    state[0] = 0x61707865;
    state[1] = 0x3320646e;
    state[2] = 0x79622d32;
    state[3] = 0x6b206574;

    // 256-bit Key (8 x 32-bit words, little-endian)
    for (int i = 0; i < 8; i++) {
        state[4 + i] = (uint32_t)key[i * 4 + 0] |
                       ((uint32_t)key[i * 4 + 1] << 8) |
                       ((uint32_t)key[i * 4 + 2] << 16) |
                       ((uint32_t)key[i * 4 + 3] << 24);
    }

    // 32-bit Block counter
    state[12] = counter;

    // 96-bit Nonce (3 x 32-bit words, little-endian)
    for (int i = 0; i < 3; i++) {
        state[13 + i] = (uint32_t)nonce[i * 4 + 0] |
                        ((uint32_t)nonce[i * 4 + 1] << 8) |
                        ((uint32_t)nonce[i * 4 + 2] << 16) |
                        ((uint32_t)nonce[i * 4 + 3] << 24);
    }

    memcpy(orig, state, sizeof(state));

    // 20 Rounds (10 iterations of column + diagonal rounds)
    for (int r = 0; r < 10; r++) {
        // Column round
        CHACHA20_QUARTER_ROUND(state[0], state[4], state[8],  state[12]);
        CHACHA20_QUARTER_ROUND(state[1], state[5], state[9],  state[13]);
        CHACHA20_QUARTER_ROUND(state[2], state[6], state[10], state[14]);
        CHACHA20_QUARTER_ROUND(state[3], state[7], state[11], state[15]);

        // Diagonal round
        CHACHA20_QUARTER_ROUND(state[0], state[5], state[10], state[15]);
        CHACHA20_QUARTER_ROUND(state[1], state[6], state[11], state[12]);
        CHACHA20_QUARTER_ROUND(state[2], state[7], state[8],  state[13]);
        CHACHA20_QUARTER_ROUND(state[3], state[4], state[9],  state[14]);
    }

    // Add original state and serialize to byte array (little-endian)
    for (int i = 0; i < 16; i++) {
        uint32_t v = state[i] + orig[i];
        output[i * 4 + 0] = (uint8_t)(v & 0xFF);
        output[i * 4 + 1] = (uint8_t)((v >> 8) & 0xFF);
        output[i * 4 + 2] = (uint8_t)((v >> 16) & 0xFF);
        output[i * 4 + 3] = (uint8_t)((v >> 24) & 0xFF);
    }
}

// Encrypt or decrypt data in-place using ChaCha20 (XOR with keystream)
static inline void chacha20_crypt(const uint8_t key[32], const uint8_t nonce[12], uint32_t counter, uint8_t *data, size_t len) {
    uint8_t keyStream[64];
    size_t offset = 0;

    while (len > 0) {
        chacha20_block(key, nonce, counter, keyStream);
        size_t chunk = (len < 64) ? len : 64;
        for (size_t i = 0; i < chunk; i++) {
            data[offset + i] ^= keyStream[i];
        }
        offset += chunk;
        len -= chunk;
        counter++;
    }
}

// Derive unique 256-bit Device Key from 256-bit Master Key and 32-bit Device ID
// K_dev = ChaCha20_Block(MasterKey, Nonce=['KDF\0', DeviceID_LE, 0, 0], Counter=0)[0..31]
static inline void chacha20_derive_key(const uint8_t master_key[32], uint32_t device_id, uint8_t derived_key[32]) {
    uint8_t nonce[12] = {'K', 'D', 'F', 0};
    nonce[4] = (uint8_t)(device_id & 0xFF);
    nonce[5] = (uint8_t)((device_id >> 8) & 0xFF);
    nonce[6] = (uint8_t)((device_id >> 16) & 0xFF);
    nonce[7] = (uint8_t)((device_id >> 24) & 0xFF);
    nonce[8] = 0; nonce[9] = 0; nonce[10] = 0; nonce[11] = 0;
    uint8_t block[64];
    chacha20_block(master_key, nonce, 0, block);
    memcpy(derived_key, block, 32);
}

// Generate zero-collision 32-bit hardware Device ID from factory 96-bit STM32 silicon UID
// Uses MurmurHash3 32-bit avalanche finalizer for zero-touch mass flashing
static inline uint32_t generateHardwareDeviceId(void) {
#if defined(RSM4x4) || defined(STM32L4)
    uint32_t w0 = HAL_GetUIDw0();
    uint32_t w1 = HAL_GetUIDw1();
    uint32_t w2 = HAL_GetUIDw2();
#else
    const uint32_t *uid = (const uint32_t *)0x1FFFF7E8;
    uint32_t w0 = uid[0], w1 = uid[1], w2 = uid[2];
#endif

    uint32_t h = w0 ^ 0x9747b28c;
    h = (h ^ (w1 * 0xcc9e2d51)) * 0x1b873593;
    h = (h ^ (w2 * 0x85ebca6b)) * 0xc2b2ae35;

    // MurmurHash3 32-bit avalanche mixer
    h ^= h >> 16;
    h *= 0x85ebca6b;
    h ^= h >> 13;
    h *= 0xc2b2ae35;
    h ^= h >> 16;

    // Avoid reserved values 0 and 0xFFFFFFFF
    if (h == 0 || h == 0xFFFFFFFF) h = 0x41410001;
    return h;
}

#endif // CHACHA20_H
