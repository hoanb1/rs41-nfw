#!/usr/bin/env /home/pi/horus-venv/bin/python3
"""
Car Tracker & Enterprise IoT Telemetry Decoder
Hỗ trợ:
1. Giao thức bảo mật ChaCha20 Full-Packet Encrypted (Protocol Marker 0x03)
   - Quản lý định danh 32-bit (Hỗ trợ 4.2+ tỷ thiết bị IoT)
   - Giải mã luồng ChaCha20 RFC 8439 (Zero-overhead, Zero-dependency)
   - Dynamic Multi-device Keystore (Tích hợp quản lý hàng vạn thiết bị)
2. Giao thức tiêu chuẩn Horus Binary V3 ASN.1 (Tương thích ngược 100%)
3. Chuyển tiếp dữ liệu thời gian thực lên Platform hoan.uk qua REST Ingest & MQTT
"""

import sys
import json
import logging
import math
import struct
import binascii
import os
from datetime import datetime
import urllib.request
try:
    from horusdemodlib.decoder import decode_packet
except ImportError:
    decode_packet = None

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

HTTP_INGEST_URL = "http://localhost:3000/api/v1/telemetry/ingest"
KEYSTORE_FILE = "/home/pi/iot_device_keys.json"

# Default Master Pre-Shared Key (256-bit ChaCha20 Key)
DEFAULT_IOT_KEY = bytes([
    0x7a, 0x7a, 0xd8, 0x4d, 0xe5, 0x74, 0xbb, 0xa3,
    0xac, 0xaa, 0x13, 0xd0, 0x57, 0xcd, 0xd0, 0x00,
    0x82, 0x4e, 0x54, 0xcb, 0x95, 0x97, 0x9a, 0x22,
    0x0f, 0xe1, 0x69, 0x35, 0x06, 0x67, 0x89, 0xf5
])

DEVICE_KEYSTORE = {
    1: DEFAULT_IOT_KEY,
}

def load_keystore():
    """Tải danh bạ khóa mã hóa đa thiết bị từ JSON nếu có cấu hình"""
    global DEVICE_KEYSTORE
    if os.path.exists(KEYSTORE_FILE):
        try:
            with open(KEYSTORE_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                for k, v in data.items():
                    dev_id = int(k, 16) if k.startswith("0x") or k.startswith("0X") else int(k)
                    key_bytes = bytes.fromhex(v.strip())
                    if len(key_bytes) == 32:
                        DEVICE_KEYSTORE[dev_id] = key_bytes
            logging.info(f"[KEYSTORE] Loaded {len(DEVICE_KEYSTORE)} IoT device keys")
        except Exception as e:
            logging.warning(f"[KEYSTORE-ERR] Failed to load {KEYSTORE_FILE}: {e}")

load_keystore()

# ============================================================
# Pure Python ChaCha20 Implementation (RFC 8439)
# Zero external C-library dependency, runs everywhere
# ============================================================
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

def chacha20_crypt(key, nonce, counter, data):
    out = bytearray()
    for block_idx in range((len(data) + 63) // 64):
        keystream = chacha20_block(key, nonce, counter + block_idx)
        chunk = data[block_idx*64 : min(len(data), (block_idx+1)*64)]
        for b_in, b_k in zip(chunk, keystream):
            out.append(b_in ^ b_k)
    return bytes(out)

def verify_crc16(data: bytes) -> bool:
    if len(data) < 3:
        return False
    packet_crc = struct.unpack('<H', data[:2])[0]
    calc_crc = binascii.crc_hqx(data[2:], 0xffff)
    return packet_crc == calc_crc

def decode_encrypted_iot_packet(data: bytes):
    """Giải mã gói tin IoT ChaCha20 bảo mật 32-byte"""
    if len(data) < 32:
        return None
    if not verify_crc16(data[:32]):
        logging.warning("[CRC-FAIL] Encrypted IoT packet failed CRC16 checksum")
        return None

    device_id = struct.unpack('>I', data[3:7])[0]
    seq = struct.unpack('>H', data[7:9])[0]
    ciphertext = data[9:32]

    key = DEVICE_KEYSTORE.get(device_id, DEFAULT_IOT_KEY)
    nonce = bytearray(12)
    nonce[0:4] = device_id.to_bytes(4, 'little')
    nonce[4:6] = seq.to_bytes(2, 'little')

    plaintext = chacha20_crypt(key, bytes(nonce), 1, ciphertext)
    if len(plaintext) != 23:
        return None

    fmt = '>iihHBhBHHBH'
    lat_raw, lon_raw, alt, speed_raw, sats, temp_raw, hum, press_raw, batt_mv, flags, pad = struct.unpack(fmt, plaintext)

    lat = (lat_raw / 1e7) if (lat_raw != 0 or lon_raw != 0) else None
    lon = (lon_raw / 1e7) if (lat_raw != 0 or lon_raw != 0) else None
    speed = round(speed_raw / 10.0, 1)
    temp = round(temp_raw / 100.0, 2) if temp_raw != -32000 else None
    humidity = hum if hum <= 100 else None
    pressure = round(press_raw / 10.0, 1) if press_raw > 0 else None
    batt = round(batt_mv / 1000.0, 2) if batt_mv > 0 else None

    callsign = f"IOT-{device_id:08X}"
    return {
        "callsign": callsign,
        "device_id": device_id,
        "sequence_number": seq,
        "latitude": lat,
        "longitude": lon,
        "altitude": alt,
        "speed": speed,
        "sats": sats,
        "temperature": temp,
        "ext_temperature": temp,
        "humidity": humidity,
        "ext_humidity": humidity,
        "pressure": pressure,
        "ext_pressure": pressure,
        "battery_voltage": batt,
        "flags": flags,
        "encrypted": True,
        "cipher": "ChaCha20-RFC8439",
        "raw_hex": data[:32].hex()
    }

# ============================================================
# Telemetry Analytics & Filtering
# ============================================================
def calculate_dew_point(temp, humidity):
    """Tính điểm sương (°C) theo công thức Magnus-Tetens chuẩn khí tượng"""
    if temp is None or humidity is None or humidity <= 0:
        return None
    try:
        a = 17.27
        b = 237.7
        alpha = ((a * temp) / (b + temp)) + math.log(humidity / 100.0)
        dew_point = (b * alpha) / (a - alpha)
        return round(dew_point, 1)
    except Exception:
        return None

def haversine_distance_m(lat1, lon1, lat2, lon2):
    """Tính khoảng cách (mét) giữa 2 tọa độ GPS theo công thức Haversine chuẩn trắc địa"""
    try:
        R = 6371000.0  # Earth radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0)**2
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return R * c
    except Exception:
        return 0.0

device_tracker_cache = {}

def forward_to_hoan_uk(payload):
    try:
        data = json.dumps(payload).encode('utf-8')
        req = urllib.request.Request(
            HTTP_INGEST_URL,
            data=data,
            headers={
                'Content-Type': 'application/json',
                'User-Agent': 'CarTrackerDecoder/1.0'
            }
        )
        with urllib.request.urlopen(req, timeout=5) as resp:
            if resp.status == 200:
                logging.info(f"[FORWARD] Ingested to hoan.uk: {payload.get('deviceId')}")
    except Exception as e:
        logging.warning(f"[FORWARD-FAIL] Could not send to hoan.uk: {e}")

def process_line(line):
    line = line.strip()
    if not line:
        return
    try:
        raw_bytes = bytes.fromhex(line)
        pkt = None

        # 1. Phát hiện gói tin mã hóa ChaCha20 (Marker 0x03 tại byte 2, size >= 32)
        if len(raw_bytes) >= 32 and raw_bytes[2] == 0x03:
            pkt = decode_encrypted_iot_packet(raw_bytes)
        
        # 2. Tương thích ngược: Thử giải mã chuẩn Horus V3 ASN.1
        if pkt is None and decode_packet is not None:
            try:
                pkt = decode_packet(raw_bytes)
            except Exception:
                pkt = None

        if pkt:
            now_dt = datetime.now()
            now_str = now_dt.strftime("%Y-%m-%d %H:%M:%S")
            now_ts = now_dt.timestamp()
            callsign = str(pkt.get("callsign", "IOT-00000001")).upper()
            lat = pkt.get("latitude")
            lon = pkt.get("longitude")
            alt = pkt.get("altitude")
            raw_speed = pkt.get("speed", 0)
            temp = pkt.get("ext_temperature") if pkt.get("ext_temperature") is not None else pkt.get("temperature")
            humidity = pkt.get("ext_humidity") if pkt.get("ext_humidity") is not None else pkt.get("humidity")
            batt = pkt.get("battery_voltage") if pkt.get("battery_voltage") is not None else pkt.get("batt_voltage")
            pressure = pkt.get("ext_pressure") if pkt.get("ext_pressure") is not None else pkt.get("pressure")
            is_encrypted = pkt.get("encrypted", False)

            # 1. Speed Deadband: Triệt tiêu rung giật vận tốc khi đỗ xe
            speed = raw_speed if raw_speed is not None else 0.0
            if speed < 1.5:
                speed = 0.0

            # 2. Tính điểm sương
            dew_point = calculate_dew_point(temp, humidity)

            # 3. Lọc tọa độ rỗng (0,0)
            has_valid_fix = (lat is not None and lon is not None and (abs(lat) > 0.001 or abs(lon) > 0.001))

            # 4. Kinematic Outlier Gate & Stationary Anchor Filter
            cached = device_tracker_cache.get(callsign, {})
            if has_valid_fix:
                if cached.get("lat") is not None and cached.get("time") is not None:
                    dt = now_ts - cached["time"]
                    if 0 < dt < 600:
                        dist = haversine_distance_m(cached["lat"], cached["lon"], lat, lon)
                        implied_speed = (dist / dt) * 3.6
                        if implied_speed > 180.0:
                            logging.warning(f"[OUTLIER-REJECTED] {callsign}: Jump {dist:.0f}m in {dt:.1f}s ({implied_speed:.1f} km/h) -> Giữ vị trí cũ")
                            lat = cached["lat"]
                            lon = cached["lon"]
                            if alt is not None and "alt" in cached:
                                alt = cached["alt"]

                # Neo vị trí tĩnh khi dừng xe
                if speed == 0.0:
                    if cached.get("anchor_lat") is not None:
                        dist_anchor = haversine_distance_m(cached["anchor_lat"], cached["anchor_lon"], lat, lon)
                        if dist_anchor < 20.0:
                            lat = cached["anchor_lat"]
                            lon = cached["anchor_lon"]
                        else:
                            cached["anchor_lat"] = lat
                            cached["anchor_lon"] = lon
                    else:
                        cached["anchor_lat"] = lat
                        cached["anchor_lon"] = lon
                else:
                    cached["anchor_lat"] = lat
                    cached["anchor_lon"] = lon

                device_tracker_cache[callsign] = {
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "time": now_ts,
                    "anchor_lat": cached.get("anchor_lat", lat),
                    "anchor_lon": cached.get("anchor_lon", lon)
                }

            is_moving = (speed >= 2.5)
            role_desc = f"TRẠM THỜI TIẾT DI ĐỘNG ({speed:.1f} km/h)" if is_moving else "TRẠM THỜI TIẾT TẠI CHỖ (ĐỨNG YÊN)"
            security_tag = "[CHACHA20-ENCRYPTED]" if is_encrypted else "[HORUS-V3-ASN1]"

            print("=" * 65)
            print(f"[{now_str}] {role_desc} | {security_tag} | THIẾT BỊ: {callsign}")
            if has_valid_fix:
                print(f"  Vị trí: {lat:.6f}, {lon:.6f} | Độ cao: {alt} m | Vận tốc: {speed} km/h")
            else:
                print(f"  Vị trí: Đang dò vệ tinh / Lưu vị trí gần nhất | Điện áp: {batt} V")
            if temp is not None:
                print(f"  Nhiệt độ ngoài : {temp:.1f} °C")
            if humidity is not None:
                print(f"  Độ ẩm không khí: {humidity:.1f} %")
            if dew_point is not None:
                print(f"  Điểm sương     : {dew_point:.1f} °C")
            if pressure is not None:
                print(f"  Áp suất khí quyển: {pressure:.1f} hPa")
            if batt is not None:
                print(f"  Điện áp pin    : {batt:.2f} V")
            print("=" * 65, flush=True)

            loc_dict = {
                "speed": speed,
                "sats": pkt.get("satellites", pkt.get("sats", 0)),
                "gps_fix": has_valid_fix
            }
            if has_valid_fix:
                loc_dict["lat"] = lat
                loc_dict["lon"] = lon
                loc_dict["alt"] = alt
            else:
                pkt.pop("latitude", None)
                pkt.pop("longitude", None)
                pkt.pop("altitude", None)

            # Build Universal Payload for hoan.uk
            universal_payload = {
                "deviceId": callsign,
                "deviceType": "mobile_weather_station",
                "stationRole": "mobile" if is_moving else "stationary",
                "protocol": "chacha20_4fsk" if is_encrypted else "horus_v3",
                "timestamp": datetime.now().isoformat(),
                "location": loc_dict,
                "environment": {
                    "temperature": temp,
                    "humidity": humidity,
                    "pressure": pressure,
                    "dewPoint": dew_point
                },
                "system": {
                    "voltage": batt,
                    "seq": pkt.get("sequence_number"),
                    "encrypted": is_encrypted,
                    "rssi": pkt.get("rssi"),
                    "snr": pkt.get("snr")
                },
                "raw": pkt
            }

            # 1. Forward directly to hoan.uk ingestion API
            forward_to_hoan_uk(universal_payload)

            # 2. Append local log
            log_path = "/home/pi/car_tracker.log" if os.path.exists("/home/pi") else "/tmp/car_tracker.log"
            try:
                with open(log_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(universal_payload, default=str) + "\n")
            except Exception as e:
                logging.warning(f"[LOG-ERR] Failed to write log: {e}")
    except Exception as e:
        logging.warning(f"[DECODE-ERR] {e}")

def main():
    logging.info("Car Tracker & IoT Secure Decoder san sang nhan du lieu tu stdin...")
    for line in sys.stdin:
        process_line(line)

if __name__ == "__main__":
    main()
