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
#pragma once

#include <stddef.h>
#include <stdint.h>

// Protects the Wi-Fi credentials the Master hands a Slave for an online update.
//
// ESP-NOW is unencrypted in HyperLED, and the credentials used to travel in plain JSON - anyone
// with an ESP32 in radio range could read the Wi-Fi password the moment an update was started.
// Now each update first agrees on a fresh key (X25519, a new key pair on both sides every time),
// and the credentials go out sealed with AES-256-GCM. The update URL is authenticated with them,
// so it cannot be swapped on the way either.
//
// This defeats listening in. Neither side proves who it is, so a device that actively answers in
// a Slave's place during the few seconds of an update could still get the credentials; the Master
// refuses to send them when two different answers arrive for one Slave.
//
// Keep this file identical in the Master and the Slave project, like HyperBus.h.
namespace UpdateSeal {

constexpr size_t PUBLIC_LEN = 32;
constexpr size_t PRIVATE_LEN = 32;
constexpr size_t KEY_LEN = 32;
constexpr size_t NONCE_LEN = 12;
constexpr size_t TAG_LEN = 16;
constexpr size_t OVERHEAD = NONCE_LEN + TAG_LEN;

// How long either side keeps its half of a key exchange before giving up on it.
constexpr unsigned long EXCHANGE_TIMEOUT_MS = 20000;

struct KeyPair {
    uint8_t priv[PRIVATE_LEN];
    uint8_t pub[PUBLIC_LEN];
};

// A fresh X25519 key pair from the hardware random number generator.
bool generate(KeyPair& kp);

// The shared AES key: SHA-256 over a label, the X25519 secret and both public keys (the Master's
// first), so both sides arrive at the same key and it is bound to this exchange.
bool deriveKey(const KeyPair& own, const uint8_t peerPub[PUBLIC_LEN], const uint8_t masterPub[PUBLIC_LEN],
               const uint8_t slavePub[PUBLIC_LEN], uint8_t key[KEY_LEN]);

// out receives nonce | ciphertext | tag, i.e. len + OVERHEAD bytes. aad is authenticated, not hidden.
bool seal(const uint8_t key[KEY_LEN], const uint8_t* aad, size_t aadLen, const uint8_t* plain, size_t len,
          uint8_t* out);

// The reverse of seal(); in is nonce | ciphertext | tag. False if anything was altered.
bool open(const uint8_t key[KEY_LEN], const uint8_t* aad, size_t aadLen, const uint8_t* in, size_t inLen,
          uint8_t* plain);

// Overwrites secrets in a way the compiler cannot optimise away.
void wipe(void* data, size_t len);

}  // namespace UpdateSeal

// CMD_TRIGGER_UPDATE_SEALED payload:
//   [0]        format (UPDATE_SEAL_FORMAT)
//   [1]        url length n
//   [2..2+n)   url
//   [..]       nonce | ciphertext | tag, where the plaintext is [ssid len][ssid][pass len][pass]
// Bytes 0..2+n are the additional authenticated data. An ESP-NOW packet carries 244 payload bytes;
// with the longest SSID (32) and password (64) that leaves 116 bytes for the URL.
#define UPDATE_SEAL_FORMAT 1
#define UPDATE_SEAL_MAX_URL 116
