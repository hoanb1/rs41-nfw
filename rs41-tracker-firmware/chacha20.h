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

#endif // CHACHA20_H
