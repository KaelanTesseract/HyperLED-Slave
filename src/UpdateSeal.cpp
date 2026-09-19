/*
 * HyperLED - Open Source LED Controller
 *
 * Copyright (c) 2026 Dennis Guse
 *
 * Licensed under the EUPL, Version 1.2 or – as soon they will be approved by
 * the European Commission - subsequent versions of the EUPL (the "Licence");
 * You may not use this work except in compliance with the Licence.
 * You may obtain a copy of the Licence at:
 *
 * https://joinup.ec.europa.eu/software/page/eupl
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the Licence is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the Licence for the specific language governing permissions and
 * limitations under the Licence.
 */
#include "UpdateSeal.h"

#include <esp_random.h>
#include <mbedtls/ecdh.h>
#include <mbedtls/ecp.h>
#include <mbedtls/gcm.h>
#include <mbedtls/sha256.h>
#include <string.h>

namespace UpdateSeal {

namespace {

int hardwareRandom(void*, unsigned char* out, size_t len) {
    esp_fill_random(out, len);
    return 0;
}

const char LABEL[] = "HyperLED update key v1";

}  // namespace

void wipe(void* data, size_t len) {
    volatile uint8_t* p = static_cast<volatile uint8_t*>(data);
    while (len--) *p++ = 0;
}

bool generate(KeyPair& kp) {
    mbedtls_ecp_group grp;
    mbedtls_mpi d;
    mbedtls_ecp_point q;
    mbedtls_ecp_group_init(&grp);
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&q);

    size_t olen = 0;
    bool ok = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_CURVE25519) == 0 &&
              mbedtls_ecp_gen_keypair(&grp, &d, &q, hardwareRandom, nullptr) == 0 &&
              mbedtls_mpi_write_binary_le(&d, kp.priv, PRIVATE_LEN) == 0 &&
              mbedtls_ecp_point_write_binary(&grp, &q, MBEDTLS_ECP_PF_UNCOMPRESSED, &olen, kp.pub,
                                             PUBLIC_LEN) == 0 &&
              olen == PUBLIC_LEN;

    mbedtls_ecp_point_free(&q);
    mbedtls_mpi_free(&d);
    mbedtls_ecp_group_free(&grp);
    if (!ok) wipe(&kp, sizeof(kp));
    return ok;
}

bool deriveKey(const KeyPair& own, const uint8_t peerPub[PUBLIC_LEN], const uint8_t masterPub[PUBLIC_LEN],
               const uint8_t slavePub[PUBLIC_LEN], uint8_t key[KEY_LEN]) {
    mbedtls_ecp_group grp;
    mbedtls_mpi d, z;
    mbedtls_ecp_point peer;
    mbedtls_ecp_group_init(&grp);
    mbedtls_mpi_init(&d);
    mbedtls_mpi_init(&z);
    mbedtls_ecp_point_init(&peer);

    uint8_t shared[32];
    bool ok = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_CURVE25519) == 0 &&
              mbedtls_mpi_read_binary_le(&d, own.priv, PRIVATE_LEN) == 0 &&
              mbedtls_ecp_point_read_binary(&grp, &peer, peerPub, PUBLIC_LEN) == 0 &&
              mbedtls_ecp_check_pubkey(&grp, &peer) == 0 &&
              mbedtls_ecdh_compute_shared(&grp, &z, &peer, &d, hardwareRandom, nullptr) == 0 &&
              mbedtls_mpi_write_binary_le(&z, shared, sizeof(shared)) == 0;

    if (ok) {
        // An all-zero secret means the peer sent a low-order point; refuse it.
        uint8_t acc = 0;
        for (size_t i = 0; i < sizeof(shared); i++) acc |= shared[i];
        ok = acc != 0;
    }
    if (ok) {
        mbedtls_sha256_context sha;
        mbedtls_sha256_init(&sha);
        ok = mbedtls_sha256_starts(&sha, 0) == 0 &&
             mbedtls_sha256_update(&sha, reinterpret_cast<const uint8_t*>(LABEL), sizeof(LABEL) - 1) == 0 &&
             mbedtls_sha256_update(&sha, shared, sizeof(shared)) == 0 &&
             mbedtls_sha256_update(&sha, masterPub, PUBLIC_LEN) == 0 &&
             mbedtls_sha256_update(&sha, slavePub, PUBLIC_LEN) == 0 &&
             mbedtls_sha256_finish(&sha, key) == 0;
        mbedtls_sha256_free(&sha);
    }

    wipe(shared, sizeof(shared));
    mbedtls_ecp_point_free(&peer);
    mbedtls_mpi_free(&z);
    mbedtls_mpi_free(&d);
    mbedtls_ecp_group_free(&grp);
    if (!ok) wipe(key, KEY_LEN);
    return ok;
}

bool seal(const uint8_t key[KEY_LEN], const uint8_t* aad, size_t aadLen, const uint8_t* plain, size_t len,
          uint8_t* out) {
    uint8_t* nonce = out;
    uint8_t* cipher = out + NONCE_LEN;
    uint8_t* tag = cipher + len;
    esp_fill_random(nonce, NONCE_LEN);

    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    bool ok = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, key, KEY_LEN * 8) == 0 &&
              mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT, len, nonce, NONCE_LEN, aad, aadLen, plain,
                                        cipher, TAG_LEN, tag) == 0;
    mbedtls_gcm_free(&gcm);
    return ok;
}

bool open(const uint8_t key[KEY_LEN], const uint8_t* aad, size_t aadLen, const uint8_t* in, size_t inLen,
          uint8_t* plain) {
    if (inLen < OVERHEAD) return false;
    size_t len = inLen - OVERHEAD;
    const uint8_t* nonce = in;
    const uint8_t* cipher = in + NONCE_LEN;
    const uint8_t* tag = cipher + len;

    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    bool ok = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, key, KEY_LEN * 8) == 0 &&
              mbedtls_gcm_auth_decrypt(&gcm, len, nonce, NONCE_LEN, aad, aadLen, tag, TAG_LEN, cipher, plain) == 0;
    mbedtls_gcm_free(&gcm);
    if (!ok) wipe(plain, len);
    return ok;
}

}  // namespace UpdateSeal
