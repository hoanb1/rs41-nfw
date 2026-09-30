#!/usr/bin/env /home/pi/horus-venv/bin/python3
"""
Car Tracker Telemetry Decoder (Horus Binary V3 4FSK)
Nhận dữ liệu từ horus_demod và tự động chuyển tiếp lên hoan.uk qua:
1. MQTT: telemetry/v1/<callsign>/data
2. HTTP Ingest: http://localhost:3000/api/v1/telemetry/ingest
"""

import sys
import json
import logging
from datetime import datetime
import urllib.request
from horusdemodlib.decoder import decode_packet

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

HTTP_INGEST_URL = "http://localhost:3000/api/v1/telemetry/ingest"

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
                logging.info(f"[FORWARD] Successfully ingested to hoan.uk: {payload.get('deviceId')}")
    except Exception as e:
        logging.warning(f"[FORWARD-FAIL] Could not send to hoan.uk: {e}")

def process_line(line):
    line = line.strip()
    if not line:
        return
    try:
        raw_bytes = bytes.fromhex(line)
        pkt = decode_packet(raw_bytes)
        if pkt:
            now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            callsign = str(pkt.get("callsign", "CAR01")).upper()
            lat = pkt.get("latitude")
            lon = pkt.get("longitude")
            alt = pkt.get("altitude")
            speed = pkt.get("speed", 0)
            temp = pkt.get("ext_temperature") if pkt.get("ext_temperature") is not None else pkt.get("temperature")
            humidity = pkt.get("ext_humidity") if pkt.get("ext_humidity") is not None else pkt.get("humidity")
            batt = pkt.get("battery_voltage") if pkt.get("battery_voltage") is not None else pkt.get("batt_voltage")
            pressure = pkt.get("ext_pressure") if pkt.get("ext_pressure") is not None else pkt.get("pressure")

            # Filter out (0,0) unfixed GPS coordinates (searching for satellites)
            has_valid_fix = (lat is not None and lon is not None and (abs(lat) > 0.001 or abs(lon) > 0.001))

            print("=" * 60)
            print(f"[{now_str}] NHAN GOI TELEMETRY TU XE: {callsign}")
            if has_valid_fix:
                print(f"  Toa do: {lat:.6f}, {lon:.6f} | Do cao: {alt} m | Toc do: {speed} km/h")
            else:
                print(f"  Toa do: Dang cho khoa GPS fix ve tinh (indoor)... | Dien ap: {batt} V")
            if temp is not None:
                print(f"  Nhiet do: {temp:.1f} °C")
            if humidity is not None:
                print(f"  Do am: {humidity:.1f} %")
            if pressure is not None:
                print(f"  Khi ap: {pressure:.1f} hPa")
            if batt is not None:
                print(f"  Dien ap nguon/pin: {batt:.2f} V")
            print("=" * 60, flush=True)

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
                # Strip 0,0 from raw pkt to prevent accidental null-island injection
                pkt.pop("latitude", None)
                pkt.pop("longitude", None)
                pkt.pop("altitude", None)

            # Build Universal Payload for hoan.uk
            universal_payload = {
                "deviceId": callsign,
                "deviceType": "vehicle_tracker",
                "protocol": "horus_v3",
                "timestamp": datetime.now().isoformat(),
                "location": loc_dict,
                "environment": {
                    "temperature": temp,
                    "humidity": humidity,
                    "pressure": pressure
                },
                "system": {
                    "voltage": batt,
                    "rssi": pkt.get("rssi"),
                    "snr": pkt.get("snr")
                },
                "raw": pkt
            }

            # 1. Forward directly to hoan.uk ingestion API
            forward_to_hoan_uk(universal_payload)

            # 2. Append local log
            with open("/home/pi/car_tracker.log", "a", encoding="utf-8") as f:
                f.write(json.dumps(universal_payload, default=str) + "\n")
    except Exception as e:
        logging.warning(f"[DECODE-ERR] {e}")

def main():
    logging.info("Car Tracker Horus V3 Decoder san sang nhan du lieu tu stdin...")
    for line in sys.stdin:
        process_line(line)

if __name__ == "__main__":
    main()
