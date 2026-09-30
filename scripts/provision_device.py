#!/usr/bin/env python3
"""
RS41 Enterprise IoT Provisioning & Claiming Utility
Sử dụng Root Master Key để tự động phái sinh khóa thiết bị (ChaCha20-KDF),
cấp phát Device ID duy nhất, tạo mã bảo mật Claim Token và nạp vào thiết bị.
"""

import sys
import os
import argparse
import struct
import random
import string
import json
import urllib.request
import urllib.error

# Root Master Key (256-bit)
ROOT_MASTER_KEY = bytes([
    0x7a, 0x7a, 0xd8, 0x4d, 0xe5, 0x74, 0xbb, 0xa3,
    0xac, 0xaa, 0x13, 0xd0, 0x57, 0xcd, 0xd0, 0x00,
    0x82, 0x4e, 0x54, 0xcb, 0x95, 0x97, 0x9a, 0x22,
    0x0f, 0xe1, 0x69, 0x35, 0x06, 0x67, 0x89, 0xf5
])

def chacha20_rotl32(v, c):
    return ((v << c) & 0xffffffff) | (v >> (32 - c))

def chacha20_quarter_round(state, a, b, c, d):
    state[a] = (state[a] + state[b]) & 0xffffffff
    state[d] = chacha20_rotl32(state[d] ^ state[a], 16)
    state[c] = (state[c] + state[d]) & 0xffffffff
    state[b] = chacha20_rotl32(state[b] ^ state[c], 12)
    state[a] = (state[a] + state[b]) & 0xffffffff
    state[d] = chacha20_rotl32(state[d] ^ state[a], 8)
    state[c] = (state[c] + state[d]) & 0xffffffff
    state[b] = chacha20_rotl32(state[b] ^ state[c], 7)

def chacha20_block(key, nonce, counter):
    constants = [0x61707865, 0x3320646e, 0x79622d32, 0x6b206574]
    key_words = [int.from_bytes(key[i*4:(i+1)*4], 'little') for i in range(8)]
    nonce_words = [int.from_bytes(nonce[i*4:(i+1)*4], 'little') for i in range(3)]
    state = constants + key_words + [counter] + nonce_words
    orig = list(state)
    for _ in range(10):
        chacha20_quarter_round(state, 0, 4, 8, 12)
        chacha20_quarter_round(state, 1, 5, 9, 13)
        chacha20_quarter_round(state, 2, 6, 10, 14)
        chacha20_quarter_round(state, 3, 7, 11, 15)
        chacha20_quarter_round(state, 0, 5, 10, 15)
        chacha20_quarter_round(state, 1, 6, 11, 12)
        chacha20_quarter_round(state, 2, 7, 8, 13)
        chacha20_quarter_round(state, 3, 4, 9, 14)
    out = bytearray()
    for i in range(16):
        out.extend(((state[i] + orig[i]) & 0xffffffff).to_bytes(4, 'little'))
    return bytes(out)

def derive_device_key(master_key: bytes, device_id: int) -> bytes:
    nonce = bytearray(12)
    nonce[0:4] = b'KDF\x00'
    nonce[4:8] = device_id.to_bytes(4, 'little')
    block = chacha20_block(master_key, bytes(nonce), 0)
    return block[:32]

def generate_claim_token() -> str:
    chars = string.ascii_uppercase + string.digits
    # Remove ambiguous characters (0, O, 1, I)
    chars = chars.replace('0', '').replace('O', '').replace('1', '').replace('I', '')
    p1 = ''.join(random.choices(chars, k=4))
    p2 = ''.join(random.choices(chars, k=4))
    return f"{p1}-{p2}"

def register_to_hoan_uk(server_url: str, device_id_str: str, num_id: int, name: str, placement: str):
    url = f"{server_url.rstrip('/')}/api/v1/devices/provision"
    payload = {
        "numericId": num_id,
        "name": name,
        "placement": placement,
        "type": "mobile_weather_station",
        "protocol": "chacha20_4fsk"
    }
    data = json.dumps(payload).encode('utf-8')
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            if resp.status in (200, 201):
                return json.loads(resp.read().decode('utf-8'))
    except Exception as e:
        return {"error": str(e)}
    return None

def configure_device_serial(port: str, device_id: int, device_key: bytes):
    try:
        import serial, time
        s = serial.Serial(port, 115200, timeout=1.5)
        time.sleep(0.5)
        s.write(b'\r\nSTATUS\r\n')
        time.sleep(0.3)

        # Set Device ID
        s.write(f"SET:ID={device_id}\r\n".encode())
        time.sleep(0.3)

        # Set Derived Key
        s.write(f"SET:KEY={device_key.hex()}\r\n".encode())
        time.sleep(0.3)

        # Enable ChaCha20 encryption
        s.write(b"SET:ENC=1\r\n")
        time.sleep(0.3)

        s.write(b"STATUS\r\n")
        time.sleep(0.5)
        out = s.read(2048).decode('utf-8', errors='replace')
        s.close()
        return True, out
    except Exception as e:
        return False, str(e)

def main():
    parser = argparse.ArgumentParser(description="RS41 Enterprise IoT Provisioning Utility")
    parser.add_argument("--id", type=int, required=True, help="32-bit Numeric Device ID (e.g. 1, 2, 100)")
    parser.add_argument("--name", type=str, default="", help="Device Friendly Name (e.g. VinFast VF8)")
    parser.add_argument("--placement", type=str, default="outdoor", choices=["outdoor", "in_vehicle", "indoor", "on_vehicle_exterior"], help="Sensor Placement Environment")
    parser.add_argument("--serial", type=str, default="", help="Serial port to provision directly (e.g. /dev/ttyACM0)")
    parser.add_argument("--server", type=str, default="http://192.168.3.24:3000", help="hoan.uk API Server URL")
    args = parser.parse_args()

    num_id = args.id
    device_id_str = f"IOT-{num_id:08X}"
    name = args.name or f"RS41 Tracker #{num_id}"
    derived_key = derive_device_key(ROOT_MASTER_KEY, num_id)

    print("=" * 70)
    print("      CHỨNG THƯ XUẤT XƯỞNG & KÍCH HOẠT THIẾT BỊ IOT (PROVISIONING)")
    print("=" * 70)
    print(f" Mã định danh thiết bị (Device ID) : {device_id_str} (Số ID: {num_id})")
    print(f" Tên thiết bị                      : {name}")
    print(f" Môi trường đặt cảm biến           : {args.placement}")
    print(f" Khóa riêng thiết bị (256-bit Key) : {derived_key.hex()}")
    print(f" Thuật toán mã hóa                 : ChaCha20 (RFC 8439) + Master KDF")
    print("-" * 70)

    # 1. Đăng ký lên hệ thống hoan.uk
    server_resp = register_to_hoan_uk(args.server, device_id_str, num_id, name, args.placement)
    claim_token = None
    if server_resp and "claimToken" in server_resp:
        claim_token = server_resp["claimToken"]
        print(f" Mã kích hoạt bí mật (Claim Token) : {claim_token}  <--- IN LÊN TEM DÁN / THẺ CÀO")
        print(f" Trạng thái trên hoan.uk           : SẴN SÀNG CHỜ NGƯỜI DÙNG KÍCH HOẠT")
    else:
        claim_token = generate_claim_token()
        print(f" Mã kích hoạt offline (Claim Token): {claim_token}")
        if server_resp and "error" in server_resp:
            print(f" [Lưu ý đồng bộ Server]: {server_resp['error']}")

    claim_url = f"{args.server}/devices?claim={device_id_str}&token={claim_token}"
    print(f" Link nhận thiết bị trực tiếp      : {claim_url}")
    print("-" * 70)

    # 2. Cấu hình trực tiếp vào phần cứng qua cổng Serial nếu có
    if args.serial and os.path.exists(args.serial):
        print(f"-> Đang nạp cấu hình trực tiếp vào phần cứng qua cổng {args.serial}...")
        success, res = configure_device_serial(args.serial, num_id, derived_key)
        if success:
            print("-> NẠP CẤU HÌNH THÀNH CÔNG VÀO PHẦN CỨNG RS41!")
        else:
            print(f"-> Nạp cổng serial thất bại: {res}")
    else:
        print("-> Cấu hình thủ công qua CLI (nếu không cắm serial trực tiếp):")
        print(f"   SET:ID={num_id}")
        print(f"   SET:KEY={derived_key.hex()}")
        print(f"   SET:ENC=1")

    print("=" * 70)

if __name__ == "__main__":
    main()
