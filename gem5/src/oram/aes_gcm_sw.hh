#ifndef AES_GCM_SW_HH
#define AES_GCM_SW_HH
// =============================================================================
// aes_gcm_sw.hh — Software AES-GCM-128 matching the RTL's exact algorithm.
//
// Used by DDR_FILL to encrypt slot init data via sendFunctional so the
// ORAM RTL can decrypt without tag mismatch. Matches:
//   - aes_gcm_pipelined.v: counter starts at 2, EY0 at counter=1
//   - ghash_single_cycle_fpga.v: GF(2^128) with x^128+x^7+x^2+x+1
//   - Bit order: MSB-first (standard NIST GCM / SP 800-38D)
//
// Usage:
//   AesGcmSw gcm;
//   gcm.setKey(key_bytes);  // 16 bytes, big-endian
//   gcm.encrypt(iv, plaintext, 4096, ciphertext, tag);
// =============================================================================
#include <cstdint>
#include <cstring>

class AesGcmSw {
public:
    // Set 128-bit key (16 bytes, big-endian). Call once before encrypt.
    void setKey(const uint8_t key[16]) {
        keyExpansion(key);
        // Precompute H = AES(K, 0^128) for GHASH
        uint8_t zero[16] = {};
        aesEncryptBlock(zero, H);
    }

    // Set key from 4 × 32-bit words (Verilator layout: w[0]=key[31:0] LSW)
    void setKeyFromWords(uint32_t w0, uint32_t w1, uint32_t w2, uint32_t w3) {
        uint8_t k[16];
        // Verilator: w3=key[127:96] (MSW), w0=key[31:0] (LSW)
        // AES big-endian: byte[0]=key[127:120]
        storeBE32(k + 0,  w3);
        storeBE32(k + 4,  w2);
        storeBE32(k + 8,  w1);
        storeBE32(k + 12, w0);
        setKey(k);
    }

    // Encrypt plaintext (len bytes, must be multiple of 16) with 96-bit IV.
    // Produces ciphertext (same length) and 128-bit tag (16 bytes).
    void encrypt(const uint8_t iv[12], const uint8_t *plain, int len,
                 uint8_t *cipher, uint8_t tag[16]) {
        // J0 = IV || 0x00000001
        uint8_t j0[16];
        memcpy(j0, iv, 12);
        storeBE32(j0 + 12, 1);

        // EY0 = AES(K, J0) — for final tag XOR
        uint8_t ey0[16];
        aesEncryptBlock(j0, ey0);

        // GCTR: encrypt each 16-byte block with counter starting at 2
        uint32_t ctr = 2;
        int nblocks = len / 16;
        for (int i = 0; i < nblocks; i++) {
            uint8_t ctr_block[16];
            memcpy(ctr_block, iv, 12);
            storeBE32(ctr_block + 12, ctr++);

            uint8_t keystream[16];
            aesEncryptBlock(ctr_block, keystream);

            for (int b = 0; b < 16; b++)
                cipher[i * 16 + b] = plain[i * 16 + b] ^ keystream[b];
        }

        // GHASH(H, {}, C, len) — no AAD
        uint8_t ghash[16] = {};
        for (int i = 0; i < nblocks; i++) {
            xorBlock(ghash, cipher + i * 16);
            gfMul(ghash, H, ghash);
        }
        // Length block: [0 (AAD len 64-bit)] [ciphertext len in bits 64-bit]
        uint8_t lenblock[16] = {};
        uint64_t cbitlen = (uint64_t)len * 8;
        storeBE64(lenblock + 8, cbitlen);
        xorBlock(ghash, lenblock);
        gfMul(ghash, H, ghash);

        // Tag = GHASH XOR EY0
        for (int b = 0; b < 16; b++)
            tag[b] = ghash[b] ^ ey0[b];
    }

    // Set IV from 3 × 32-bit words (Verilator layout: w[0]=iv[31:0] LSW)
    static void ivFromWords(uint32_t w0, uint32_t w1, uint32_t w2, uint8_t iv[12]) {
        // Verilator: w2=iv[95:64] (MSW), w0=iv[31:0] (LSW)
        storeBE32(iv + 0, w2);
        storeBE32(iv + 4, w1);
        storeBE32(iv + 8, w0);
    }

private:
    uint8_t roundKeys[176]; // 11 × 16 bytes
    uint8_t H[16];          // GHASH subkey

    // ---- AES-128 ----
    static const uint8_t sbox[256];
    static const uint8_t rcon[11];

    static void storeBE32(uint8_t *p, uint32_t v) {
        p[0] = (v >> 24) & 0xFF;
        p[1] = (v >> 16) & 0xFF;
        p[2] = (v >>  8) & 0xFF;
        p[3] =  v        & 0xFF;
    }
    static void storeBE64(uint8_t *p, uint64_t v) {
        storeBE32(p, (uint32_t)(v >> 32));
        storeBE32(p + 4, (uint32_t)v);
    }

    void keyExpansion(const uint8_t key[16]) {
        memcpy(roundKeys, key, 16);
        for (int i = 4; i < 44; i++) {
            uint8_t tmp[4];
            memcpy(tmp, roundKeys + (i - 1) * 4, 4);
            if (i % 4 == 0) {
                // RotWord + SubWord + Rcon
                uint8_t t = tmp[0];
                tmp[0] = sbox[tmp[1]] ^ rcon[i / 4];
                tmp[1] = sbox[tmp[2]];
                tmp[2] = sbox[tmp[3]];
                tmp[3] = sbox[t];
            }
            for (int b = 0; b < 4; b++)
                roundKeys[i * 4 + b] = roundKeys[(i - 4) * 4 + b] ^ tmp[b];
        }
    }

    void aesEncryptBlock(const uint8_t in[16], uint8_t out[16]) {
        uint8_t state[16];
        memcpy(state, in, 16);

        // Initial round key
        addRoundKey(state, 0);

        // Rounds 1-9
        for (int r = 1; r <= 9; r++) {
            subBytes(state);
            shiftRows(state);
            mixColumns(state);
            addRoundKey(state, r);
        }

        // Round 10 (no MixColumns)
        subBytes(state);
        shiftRows(state);
        addRoundKey(state, 10);

        memcpy(out, state, 16);
    }

    void addRoundKey(uint8_t state[16], int round) {
        for (int i = 0; i < 16; i++)
            state[i] ^= roundKeys[round * 16 + i];
    }

    static void subBytes(uint8_t state[16]) {
        for (int i = 0; i < 16; i++)
            state[i] = sbox[state[i]];
    }

    static void shiftRows(uint8_t state[16]) {
        // AES state is column-major: state[row + 4*col]
        uint8_t t;
        // Row 1: shift left 1
        t = state[1]; state[1] = state[5]; state[5] = state[9];
        state[9] = state[13]; state[13] = t;
        // Row 2: shift left 2
        t = state[2]; state[2] = state[10]; state[10] = t;
        t = state[6]; state[6] = state[14]; state[14] = t;
        // Row 3: shift left 3
        t = state[15]; state[15] = state[11]; state[11] = state[7];
        state[7] = state[3]; state[3] = t;
    }

    static uint8_t xtime(uint8_t a) {
        return (a << 1) ^ ((a & 0x80) ? 0x1B : 0x00);
    }

    static void mixColumns(uint8_t state[16]) {
        for (int c = 0; c < 4; c++) {
            uint8_t *s = state + 4 * c;
            uint8_t a0 = s[0], a1 = s[1], a2 = s[2], a3 = s[3];
            uint8_t t = a0 ^ a1 ^ a2 ^ a3;
            s[0] = a0 ^ xtime(a0 ^ a1) ^ t;
            s[1] = a1 ^ xtime(a1 ^ a2) ^ t;
            s[2] = a2 ^ xtime(a2 ^ a3) ^ t;
            s[3] = a3 ^ xtime(a3 ^ a0) ^ t;
        }
    }

    // ---- GF(2^128) multiplication (NIST GCM, reflected bit order) ----
    static void xorBlock(uint8_t dst[16], const uint8_t src[16]) {
        for (int i = 0; i < 16; i++) dst[i] ^= src[i];
    }

    static void gfMul(const uint8_t X[16], const uint8_t Y[16], uint8_t out[16]) {
        // NIST SP 800-38D Algorithm 1: X • Y in GF(2^128)
        // R = 0xE1000000...0 (bit-reflected reduction polynomial)
        uint8_t Z[16] = {};
        uint8_t V[16];
        memcpy(V, Y, 16);

        for (int i = 0; i < 128; i++) {
            // If bit i of X is set (MSB first: byte i/8, bit 7-(i%8))
            if (X[i / 8] & (1 << (7 - (i % 8)))) {
                for (int b = 0; b < 16; b++) Z[b] ^= V[b];
            }
            // V = V >> 1 (right shift in reflected convention)
            int lsb = V[15] & 1;
            for (int b = 15; b > 0; b--)
                V[b] = (V[b] >> 1) | ((V[b - 1] & 1) << 7);
            V[0] >>= 1;
            if (lsb) V[0] ^= 0xE1; // reduction
        }
        memcpy(out, Z, 16);
    }
};

// ---- AES S-box (FIPS 197) ----
const uint8_t AesGcmSw::sbox[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16,
};

const uint8_t AesGcmSw::rcon[11] = {
    0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36,
};

#endif // AES_GCM_SW_HH
