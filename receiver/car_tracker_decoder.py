#!/usr/bin/env python3
"""
Car Tracker & Enterprise IoT Telemetry Decoder & Zero-Knowledge Relay
Hỗ trợ:
1. Giao thức chuyển tiếp an toàn Zero-Knowledge Encrypted Relay (Marker 0x03)
   - Trạm thu KHÔNG lưu khóa, KHÔNG giải mã tại biên để bảo mật tuyệt đối
   - Xác thực toàn vẹn CRC16 và chuyển tiếp gói tin mã hóa nguyên vẹn lên api.hoan.uk
2. Giao thức tiêu chuẩn Horus Binary V3 ASN.1 & V2 (Tương thích ngược 100%)
3. Trích xuất chính xác SNR (dB) và ước lượng RSSI (dBm) từ bộ giải mã RTL-SDR
4. Chuyển tiếp dữ liệu thời gian thực lên Platform api.hoan.uk qua REST Ingest & MQTT Broker
"""

import sys
import json
import logging
import math
import struct
import binascii
import os
import socket
import argparse
from datetime import datetime
import urllib.request
import urllib.error
import ssl

try:
    from horusdemodlib.decoder import decode_packet
except ImportError:
    decode_packet = None

try:
    from horusdemodlib.demod import HorusLib, Mode
except ImportError:
    HorusLib = None
    Mode = None

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

HTTP_INGEST_URL = os.environ.get("HTTP_INGEST_URL", "https://api.hoan.uk/api/v1/telemetry/ingest")
MQTT_BROKER = os.environ.get("MQTT_BROKER", "mqtt.hoan.uk")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_TOPIC = os.environ.get("MQTT_TOPIC", "hoanuk/telemetry")
MQTT_USERNAME = os.environ.get("MQTT_USERNAME", None)
MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD", None)
MQTT_ENABLED = os.environ.get("MQTT_ENABLED", "1").lower() in ("1", "true", "yes")

# SondeHub Amateur Ingest (https://amateur.sondehub.org)
SONDEHUB_AMATEUR_URL = os.environ.get("SONDEHUB_AMATEUR_URL", "https://api.v2.sondehub.org/amateur/telemetry")
SONDEHUB_AMATEUR_ENABLED = os.environ.get("SONDEHUB_AMATEUR_ENABLED", "1").lower() in ("1", "true", "yes")
UPLOADER_CALLSIGN = os.environ.get("UPLOADER_CALLSIGN", "XV9HNT-SDR")
last_sondehub_upload_time = {}

# ============================================================
# CRC16 Checksum Verification
# ============================================================
def verify_crc16(data: bytes) -> bool:
    if len(data) < 3:
        return False
    packet_crc = struct.unpack('<H', data[:2])[0]
    calc_crc = binascii.crc_hqx(data[2:], 0xffff)
    return packet_crc == calc_crc

def calculate_dew_point(temp, humidity):
    if temp is None or humidity is None or humidity <= 0:
        return None
    a = 17.27
    b = 237.7
    try:
        alpha = ((a * temp) / (b + temp)) + math.log(humidity / 100.0)
        return round((b * alpha) / (a - alpha), 1)
    except Exception:
        return None

def haversine_distance_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2.0)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2.0)**2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c

device_tracker_cache = {}

# ============================================================
# Pure Python MQTT 3.1.1 Publisher (Zero external dependency)
# ============================================================
def mqtt_publish(host: str, port: int, topic: str, payload_str: str, client_id="rs41_sdr_gateway", username=None, password=None, timeout=2.5) -> bool:
    try:
        s = socket.create_connection((host, port), timeout=timeout)
        s.settimeout(timeout)

        # 1. CONNECT packet
        proto_name = b"MQTT"
        connect_flags = 0x02  # Clean session
        if username:
            connect_flags |= 0x80
        if password:
            connect_flags |= 0x40

        var_header = bytearray()
        var_header.extend(len(proto_name).to_bytes(2, 'big'))
        var_header.extend(proto_name)
        var_header.append(0x04)  # Level 4 (MQTT 3.1.1)
        var_header.append(connect_flags)
        var_header.extend((60).to_bytes(2, 'big'))  # Keepalive

        payload = bytearray()
        payload.extend(len(client_id).to_bytes(2, 'big'))
        payload.extend(client_id.encode('utf-8'))
        if username:
            payload.extend(len(username).to_bytes(2, 'big'))
            payload.extend(username.encode('utf-8'))
        if password:
            payload.extend(len(password).to_bytes(2, 'big'))
            payload.extend(password.encode('utf-8'))

        rem_len = len(var_header) + len(payload)
        rem_bytes = bytearray()
        x = rem_len
        while True:
            b = x % 128
            x = x // 128
            if x > 0:
                b |= 128
            rem_bytes.append(b)
            if x <= 0:
                break

        connect_pkt = bytearray([0x10]) + rem_bytes + var_header + payload
        s.sendall(connect_pkt)

        # Read CONNACK (4 bytes)
        ack = s.recv(4)
        if len(ack) < 4 or ack[0] != 0x20 or ack[3] != 0x00:
            s.close()
            return False

        # 2. PUBLISH packet (QoS 0)
        topic_bytes = topic.encode('utf-8')
        msg_bytes = payload_str.encode('utf-8')
        pub_var_header = len(topic_bytes).to_bytes(2, 'big') + topic_bytes
        pub_rem_len = len(pub_var_header) + len(msg_bytes)

        pub_rem_bytes = bytearray()
        x = pub_rem_len
        while True:
            b = x % 128
            x = x // 128
            if x > 0:
                b |= 128
            pub_rem_bytes.append(b)
            if x <= 0:
                break

        pub_pkt = bytearray([0x30]) + pub_rem_bytes + pub_var_header + msg_bytes
        s.sendall(pub_pkt)

        # 3. DISCONNECT packet
        s.sendall(bytes([0xE0, 0x00]))
        s.close()
        return True
    except Exception as e:
        logging.debug(f"[MQTT-FAIL] {e}")
        return False

# ============================================================
# Telemetry Forwarding: REST Ingest + MQTT
# ============================================================
def forward_to_hoan_uk(payload: dict):
    payload_json = json.dumps(payload, default=str)
    dev_id = payload.get("deviceId", "UNKNOWN")

    # 1. Forward via HTTP REST API
    if HTTP_INGEST_URL:
        try:
            req = urllib.request.Request(
                HTTP_INGEST_URL,
                data=payload_json.encode('utf-8'),
                headers={
                    'Content-Type': 'application/json',
                    'User-Agent': 'RS41-SDR-Relay/3.0'
                }
            )
            # Create standard or non-verifying SSL context if needed
            ctx = ssl.create_default_context()
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE

            with urllib.request.urlopen(req, context=ctx, timeout=3.5) as resp:
                if resp.status in (200, 201, 204):
                    logging.info(f"[FORWARD:HTTP] Ingested to {HTTP_INGEST_URL} -> {dev_id}")
        except Exception as e:
            logging.warning(f"[FORWARD:HTTP-FAIL] Could not send to {HTTP_INGEST_URL} ({dev_id}): {e}")

    # 2. Forward via MQTT Broker
    if MQTT_ENABLED and MQTT_BROKER:
        dev_topic = f"{MQTT_TOPIC}/{dev_id}"
        success = mqtt_publish(
            host=MQTT_BROKER,
            port=MQTT_PORT,
            topic=dev_topic,
            payload_str=payload_json,
            client_id=f"rs41_gateway_{dev_id}",
            username=MQTT_USERNAME,
            password=MQTT_PASSWORD
        )
        if success:
            logging.info(f"[FORWARD:MQTT] Published to {MQTT_BROKER}:{MQTT_PORT} -> {dev_topic}")
        else:
            logging.debug(f"[FORWARD:MQTT-WARN] MQTT publish skipped or timed out ({dev_topic})")

def forward_to_sondehub_amateur(pkt: dict, now_dt: datetime, snr=None, rssi=None):
    """
    Chuyển tiếp dữ liệu bóng / xe thám trắc không mã hóa lên SondeHub Amateur (https://amateur.sondehub.org)
    Sử dụng chuẩn REST API v2: PUT https://api.v2.sondehub.org/amateur/telemetry
    """
    if not SONDEHUB_AMATEUR_ENABLED or not SONDEHUB_AMATEUR_URL:
        return

    callsign = str(pkt.get("callsign", "HORUS-SONDE")).upper()
    lat = pkt.get("latitude")
    lon = pkt.get("longitude")
    alt = pkt.get("altitude")

    # Yêu cầu tọa độ GPS hợp lệ
    if lat is None or lon is None or alt is None or (abs(lat) < 0.001 and abs(lon) < 0.001):
        return

    # Giới hạn tốc độ gửi (tối đa 1 gói / 10s cho mỗi callsign)
    now_ts = now_dt.timestamp()
    last_sent = last_sondehub_upload_time.get(callsign, 0)
    if (now_ts - last_sent) < 10.0:
        return
    last_sondehub_upload_time[callsign] = now_ts

    iso_time = now_dt.strftime("%Y-%m-%dT%H:%M:%S.000000Z")
    temp = pkt.get("ext_temperature") if pkt.get("ext_temperature") is not None else pkt.get("temperature")
    humidity = pkt.get("ext_humidity") if pkt.get("ext_humidity") is not None else pkt.get("humidity")
    pressure = pkt.get("ext_pressure") if pkt.get("ext_pressure") is not None else pkt.get("pressure")
    batt = pkt.get("battery_voltage") if pkt.get("battery_voltage") is not None else pkt.get("batt_voltage")
    speed_kmh = pkt.get("speed", 0.0)

    sondehub_record = {
        "software_name": "RS41-NFW-Decoder",
        "software_version": "3.0",
        "uploader_callsign": UPLOADER_CALLSIGN,
        "time_received": iso_time,
        "payload_callsign": callsign,
        "datetime": iso_time,
        "lat": round(float(lat), 6),
        "lon": round(float(lon), 6),
        "alt": round(float(alt), 1),
        "frequency": 437.600
    }

    if temp is not None:
        sondehub_record["temp"] = round(float(temp), 1)
    if humidity is not None:
        sondehub_record["humidity"] = round(float(humidity), 1)
    if pressure is not None:
        sondehub_record["pressure"] = round(float(pressure), 1)
    if batt is not None:
        sondehub_record["batt"] = round(float(batt), 2)
    if speed_kmh is not None:
        sondehub_record["speed"] = round(float(speed_kmh), 1)
    if snr is not None:
        sondehub_record["snr"] = round(float(snr), 1)
    if rssi is not None:
        sondehub_record["rssi"] = round(float(rssi), 1)

    payload_json = json.dumps([sondehub_record])

    try:
        req = urllib.request.Request(
            SONDEHUB_AMATEUR_URL,
            data=payload_json.encode('utf-8'),
            headers={
                'Content-Type': 'application/json',
                'User-Agent': f'RS41-NFW-Decoder/3.0 ({UPLOADER_CALLSIGN})'
            },
            method='PUT'
        )
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE

        with urllib.request.urlopen(req, context=ctx, timeout=4.0) as resp:
            if resp.status in (200, 201, 204):
                logging.info(f"[SONDEHUB-AMATEUR] Uploaded telemetry for {callsign} (uploader: {UPLOADER_CALLSIGN})")
    except Exception as e:
        logging.warning(f"[SONDEHUB-AMATEUR-WARN] Upload failed for {callsign}: {e}")

# ============================================================
# Process Decoded Unencrypted Packet (Horus V3 ASN.1 / V2)
# ============================================================
def process_decoded_packet(pkt: dict, snr=None, rssi=None):
    if not pkt:
        return
    try:
        now_dt = datetime.now()
        now_str = now_dt.strftime("%Y-%m-%d %H:%M:%S")
        now_ts = now_dt.timestamp()
        callsign = str(pkt.get("callsign", "HORUS-SONDE")).upper()
        lat = pkt.get("latitude")
        lon = pkt.get("longitude")
        alt = pkt.get("altitude")
        raw_speed = pkt.get("speed", 0)
        temp = pkt.get("ext_temperature") if pkt.get("ext_temperature") is not None else pkt.get("temperature")
        humidity = pkt.get("ext_humidity") if pkt.get("ext_humidity") is not None else pkt.get("humidity")
        batt = pkt.get("battery_voltage") if pkt.get("battery_voltage") is not None else pkt.get("batt_voltage")
        pressure = pkt.get("ext_pressure") if pkt.get("ext_pressure") is not None else pkt.get("pressure")

        if snr is not None:
            pkt["snr"] = round(float(snr), 1)
        if rssi is not None:
            pkt["rssi"] = round(float(rssi), 1)
        elif snr is not None:
            pkt["rssi"] = round(max(-125.0, min(-35.0, -118.0 + max(0.0, float(snr)) * 1.15)), 1)

        speed = raw_speed if raw_speed is not None else 0.0
        if speed < 1.5:
            speed = 0.0

        dew_point = calculate_dew_point(temp, humidity)
        has_valid_fix = (lat is not None and lon is not None and (abs(lat) > 0.001 or abs(lon) > 0.001))

        cached = device_tracker_cache.get(callsign, {})
        if has_valid_fix:
            if cached.get("lat") is not None and cached.get("time") is not None:
                dt = now_ts - cached["time"]
                if 0 < dt < 600:
                    dist = haversine_distance_m(cached["lat"], cached["lon"], lat, lon)
                    implied_speed = (dist / dt) * 3.6
                    if implied_speed > 180.0:
                        logging.warning(f"[OUTLIER-REJECTED] {callsign}: Jump {dist:.0f}m -> Giữ vị trí cũ")
                        lat = cached["lat"]
                        lon = cached["lon"]
                        if alt is not None and "alt" in cached:
                            alt = cached["alt"]

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

        print("=" * 65)
        print(f"[{now_str}] {role_desc} | [HORUS-UNENCRYPTED] | THIẾT BỊ: {callsign}")
        if has_valid_fix:
            print(f"  Vị trí: {lat:.6f}, {lon:.6f} | Độ cao: {alt} m | Vận tốc: {speed} km/h")
        else:
            print(f"  Vị trí: Đang dò vệ tinh | Điện áp: {batt} V")
        if temp is not None:
            print(f"  Nhiệt độ ngoài : {temp:.1f} °C")
        if humidity is not None:
            print(f"  Độ ẩm không khí: {humidity:.1f} %")
        if dew_point is not None:
            print(f"  Điểm sương     : {dew_point:.1f} °C")
        if pressure is not None:
            print(f"  Áp suất khí quyển: {pressure:.1f} hPa")
        if pkt.get("rssi") is not None:
            print(f"  Tín hiệu (RSSI): {pkt['rssi']} dBm | SNR: {pkt.get('snr', 'N/A')} dB")
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

        universal_payload = {
            "deviceId": callsign,
            "deviceType": "radiosonde",
            "stationRole": "mobile" if is_moving else "stationary",
            "protocol": "horus_v3",
            "timestamp": datetime.now().isoformat(),
            "environment_temperature": temp,
            "environment_humidity": humidity,
            "environment_pressure": pressure,
            "system_voltage": batt,
            "speed": speed,
            "sats": loc_dict.get("sats", 0),
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
                "encrypted": False,
                "rssi": pkt.get("rssi"),
                "snr": pkt.get("snr")
            },
            "raw": pkt
        }
        if has_valid_fix:
            universal_payload["lat"] = lat
            universal_payload["lon"] = lon
            universal_payload["alt"] = alt

        forward_to_hoan_uk(universal_payload)
        forward_to_sondehub_amateur(pkt, now_dt, snr=pkt.get("snr"), rssi=pkt.get("rssi"))

    except Exception as e:
        logging.warning(f"[DECODE-ERR] {e}")

# ============================================================
# Process Encrypted IoT Packet (Zero-Knowledge Pass-Through)
# ============================================================
def process_encrypted_raw_packet(data: bytes, snr=None, rssi=None):
    if len(data) < 32:
        return
    if not verify_crc16(data[:32]):
        logging.warning("[CRC-FAIL] Encrypted IoT packet failed CRC16 checksum")
        return

    device_id = struct.unpack('>I', data[3:7])[0]
    seq = struct.unpack('>H', data[7:9])[0]
    callsign = f"IOT-{device_id:08X}"
    raw_hex_str = data[:32].hex()

    rssi_val = None
    if rssi is not None:
        rssi_val = round(float(rssi), 1)
    elif snr is not None:
        rssi_val = round(max(-125.0, min(-35.0, -118.0 + max(0.0, float(snr)) * 1.15)), 1)

    snr_val = round(float(snr), 1) if snr is not None else None

    now_dt = datetime.now()
    now_str = now_dt.strftime("%Y-%m-%d %H:%M:%S")

    print("=" * 65)
    print(f"[{now_str}] [ZERO-KNOWLEDGE RELAY] | [CHACHA20-ENCRYPTED] | THIẾT BỊ: {callsign}")
    print(f"  Frame Seq: {seq} | Kích thước: 32 bytes | CRC16: PASS")
    if rssi_val is not None:
        print(f"  Tín hiệu (RSSI): {rssi_val} dBm | SNR: {snr_val if snr_val is not None else 'N/A'} dB")
    print(f"  Raw Hex  : {raw_hex_str}")
    print(f"  -> Chuyển tiếp an toàn đến {HTTP_INGEST_URL} (Không lưu khóa tại Gateway)")
    print("=" * 65, flush=True)

    relay_payload = {
        "deviceId": callsign,
        "deviceType": "encrypted_tracker",
        "stationRole": "mobile",
        "protocol": "chacha20_4fsk",
        "timestamp": now_dt.isoformat(),
        "encrypted": True,
        "raw_hex": raw_hex_str,
        "system": {
            "seq": seq,
            "device_id": device_id,
            "encrypted": True,
            "rssi": rssi_val,
            "snr": snr_val
        },
        "raw": {
            "device_id": device_id,
            "seq": seq,
            "protocol_marker": 0x03,
            "hex": raw_hex_str
        }
    }

    forward_to_hoan_uk(relay_payload)

# ============================================================
# Main Packet Dispatcher
# ============================================================
def process_raw_bytes(raw_bytes: bytes, snr=None, rssi=None):
    # 1. Nếu là gói tin mã hóa Marker 0x03 -> Chuyển tiếp Zero-Knowledge ngay lập tức
    if len(raw_bytes) >= 32 and raw_bytes[2] == 0x03:
        process_encrypted_raw_packet(raw_bytes, snr=snr, rssi=rssi)
        return

    # 2. Nếu là gói tin chuẩn không mã hóa -> Giải mã và gửi
    pkt = None
    if decode_packet is not None:
        try:
            pkt = decode_packet(raw_bytes)
        except Exception:
            pkt = None

    if pkt:
        process_decoded_packet(pkt, snr=snr, rssi=rssi)

def run_audio_mode(baud_rate=100, tone_spacing=803, sample_rate=48000):
    if HorusLib is None:
        logging.error("HorusLib khong kha dung trong moi truong hien tai!")
        sys.exit(1)

    logging.info(f"Khoi dong HorusLib Audio Receiver ({baud_rate} baud, {tone_spacing}Hz spacing, {sample_rate}Hz)...")

    def frame_callback(frame):
        if frame.crc_pass and frame.data:
            frame_snr = float(frame.snr)
            process_raw_bytes(frame.data, snr=frame_snr)

    with HorusLib(mode=Mode.BINARY, rate=baud_rate, tone_spacing=tone_spacing,
                  sample_rate=sample_rate, callback=frame_callback) as horus:
        while True:
            chunk = sys.stdin.buffer.read(horus.nin * 2)
            if not chunk:
                break
            horus.add_samples(chunk)

def run_stdin_hex_mode():
    logging.info("Car Tracker & IoT Secure Relay san sang nhan du lieu HEX tu stdin...")
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            if line.startswith("{") and "data" in line:
                js = json.loads(line)
                raw_bytes = bytes.fromhex(js["data"])
                snr = js.get("snr") or js.get("EbNodB")
                rssi = js.get("rssi")
                process_raw_bytes(raw_bytes, snr=snr, rssi=rssi)
            else:
                parts = line.split()
                hex_str = parts[0]
                snr = None
                if len(parts) >= 3 and parts[1] == "SNR:":
                    snr = float(parts[2])
                raw_bytes = bytes.fromhex(hex_str)
                process_raw_bytes(raw_bytes, snr=snr)
        except Exception as e:
            logging.warning(f"[HEX-ERR] {e}")

def main():
    global HTTP_INGEST_URL, MQTT_BROKER, MQTT_PORT, MQTT_TOPIC, MQTT_ENABLED
    parser = argparse.ArgumentParser(description="Car Tracker & IoT Telemetry Decoder & Zero-Knowledge Relay")
    parser.add_argument("--audio", action="store_true", help="Doc truc tiep audio PCM tu rtl_fm qua HorusLib")
    parser.add_argument("--rate", type=int, default=100, help="Baud rate (default: 100)")
    parser.add_argument("--spacing", type=int, default=803, help="Tone spacing (default: 803)")
    parser.add_argument("--sample-rate", type=int, default=48000, help="Audio sample rate (default: 48000)")
    parser.add_argument("--url", type=str, default=None, help="HTTP Ingest API URL (default: https://api.hoan.uk/api/v1/telemetry/ingest)")
    parser.add_argument("--mqtt-broker", type=str, default=None, help="MQTT Broker hostname (default: api.hoan.uk)")
    parser.add_argument("--mqtt-port", type=int, default=None, help="MQTT Broker port (default: 1883)")
    parser.add_argument("--mqtt-topic", type=str, default=None, help="MQTT Topic prefix (default: hoanuk/telemetry)")
    parser.add_argument("--no-mqtt", action="store_true", help="Disable MQTT publishing")
    args = parser.parse_args()

    if args.url:
        HTTP_INGEST_URL = args.url
    if args.mqtt_broker:
        MQTT_BROKER = args.mqtt_broker
    if args.mqtt_port:
        MQTT_PORT = args.mqtt_port
    if args.mqtt_topic:
        MQTT_TOPIC = args.mqtt_topic
    if args.no_mqtt:
        MQTT_ENABLED = False

    logging.info(f"Hoan.uk HTTP Ingest Target: {HTTP_INGEST_URL}")
    if MQTT_ENABLED:
        logging.info(f"Hoan.uk MQTT Target       : {MQTT_BROKER}:{MQTT_PORT} (Topic: {MQTT_TOPIC}/<deviceId>)")
    else:
        logging.info("MQTT Forwarding           : Disabled")

    if args.audio:
        run_audio_mode(baud_rate=args.rate, tone_spacing=args.spacing, sample_rate=args.sample_rate)
    else:
        run_stdin_hex_mode()

if __name__ == "__main__":
    main()
