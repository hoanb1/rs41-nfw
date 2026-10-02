#!/usr/bin/env python3
"""
SDR Multi-Service Dynamic Priority Arbiter (5-Tier Hierarchy)
Ma tran dieu phoi phan cap uu tien 1 thiet bi RTL-SDR:
  Muc 1: SatNOGS Client (Ve tinh quoc te - Uu tien toi cao)
  Muc 2: Ground Station Native (Tram mat dat rieng - Port 7000 / Tracking)
  Muc 3: Nguoi dung tuong tac truc tiep tren OpenWebRX (Port 8073)
  Muc 4: Lich tha bong tham khong khi tuong (06:45-09:30 & 18:45-21:30) -> SondeHub Bridge
  Muc 5: Xe o to RS41-NFW (437.600 MHz) -> api.hoan.uk & mqtt.hoan.uk (Che do ranh)
"""

import time
import os
import subprocess
import logging
from datetime import datetime, time as dtime

logging.basicConfig(level=logging.INFO, format="%(asctime)s [SDR-ARBITER] %(message)s")

LOCK_SATNOGS = "/tmp/satnogs_active.lock"
LOCK_GROUND_STATION = "/tmp/ground_station_active.lock"
CHECK_INTERVAL_SEC = 5

STATE_SATNOGS = "MUC_1_SATNOGS_OBSERVATION"
STATE_GROUND_STATION = "MUC_2_GROUND_STATION_TRACKING"
STATE_USER_OPENWEBRX = "MUC_3_OPENWEBRX_USER_ACTIVE"
STATE_METEO_BALLOON = "MUC_4_METEO_BALLOON_WINDOW"
STATE_CAR_TRACKER = "MUC_5_CAR_TRACKER_IDLE"

current_state = None

def run_cmd(cmd_list):
    try:
        subprocess.run(cmd_list, capture_output=True, timeout=5, check=False)
    except Exception as e:
        logging.warning(f"Command failed {cmd_list}: {e}")

def is_satnogs_active():
    """Kiem tra xem SatNOGS co dang thuc hien quan sat ve tinh khong"""
    if os.path.exists(LOCK_SATNOGS):
        return True
    try:
        res = subprocess.run(["pgrep", "-f", "satnogs_flowgraph|gr-satnogs"], capture_output=True, text=True, timeout=2)
        return res.returncode == 0 and len(res.stdout.strip()) > 0
    except Exception:
        return False

def is_ground_station_active():
    """Kiem tra xem Ground Station rieng (Port 7000) co dang bám bat ve tinh hoac co nguoi dung thao tac khong"""
    if os.path.exists(LOCK_GROUND_STATION):
        return True
    try:
        # Kiem tra ket noi TCP tu nguoi dung den Ground Station UI (Port 7000)
        cmd = "ss -tn 'sport = :7000 and not ( dport = :7000 or dst 127.0.0.1 or dst ::1 )' | grep -E 'ESTAB|SYN-RECV'"
        res = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=2)
        if len(res.stdout.strip()) > 0:
            return True
    except Exception:
        pass
    return False

def has_active_openwebrx_users():
    """Kiem tra xem co nguoi dung nao dang truy cap web OpenWebRX (Port 8073) khong"""
    try:
        cmd = "ss -tn 'sport = :8073 and not ( dport = :8073 or dst 127.0.0.1 or dst ::1 )' | grep -E 'ESTAB|SYN-RECV'"
        res = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=2)
        return len(res.stdout.strip()) > 0
    except Exception:
        return False

def is_in_balloon_window():
    """Kiem tra xem co dang trong khung gio tha bong khi tuong khong (06:45-09:30 & 18:45-21:30)"""
    now_t = datetime.now().time()
    morning_start = dtime(6, 45)
    morning_end = dtime(9, 30)
    evening_start = dtime(18, 45)
    evening_end = dtime(21, 30)

    if morning_start <= now_t <= morning_end:
        return True
    if evening_start <= now_t <= evening_end:
        return True
    return False

def apply_state(new_state):
    global current_state
    if new_state == current_state:
        return

    logging.info(f"===> CHUYEN TRANG THAI SDR: [{current_state}] -> [{new_state}] <===")

    if new_state == STATE_SATNOGS:
        logging.info("[MUC 1: SatNOGS] Tam dung tat ca cac app khac de nhuong doc quyen RTL-SDR cho SatNOGS...")
        run_cmd(["sudo", "systemctl", "stop", "openwebrx.service", "car-tracker-rx.service"])

    elif new_state == STATE_GROUND_STATION:
        logging.info("[MUC 2: Ground Station] Kich hoat che do tram mat dat Ground Station (Port 7000)...")
        run_cmd(["sudo", "systemctl", "stop", "car-tracker-rx.service"])
        run_cmd(["sudo", "systemctl", "start", "ground-station.service"])

    elif new_state == STATE_USER_OPENWEBRX:
        logging.info("[MUC 3: OpenWebRX User] Phat hien nguoi dung truy cap web OpenWebRX -> Nguoi dung chiem quyen dieu khien...")
        run_cmd(["sudo", "systemctl", "stop", "car-tracker-rx.service"])
        run_cmd(["sudo", "systemctl", "start", "openwebrx.service"])

    elif new_state == STATE_METEO_BALLOON:
        logging.info("[MUC 4: Bong Tham Khong] Trong KHUNG GIO THA BONG KHI TUONG (400-406 MHz) -> Bat OpenWebRX 403 MHz & SondeHub Bridge...")
        run_cmd(["sudo", "systemctl", "stop", "car-tracker-rx.service"])
        run_cmd(["sudo", "systemctl", "start", "openwebrx.service", "sondehub-bridge.service"])

    elif new_state == STATE_CAR_TRACKER:
        logging.info("[MUC 5: Xe O To RS41-NFW] Thoi gian ranh -> Kich hoat bo thu Xe O To (437.600 MHz) day ve api.hoan.uk...")
        run_cmd(["sudo", "systemctl", "stop", "openwebrx.service"])
        run_cmd(["sudo", "systemctl", "start", "car-tracker-rx.service"])

    current_state = new_state

def evaluate_and_arbitrate():
    # 1. Muc 1: SatNOGS Client (Uu tien toi cao)
    if is_satnogs_active():
        apply_state(STATE_SATNOGS)
        return

    # 2. Muc 2: Ground Station Native (Tram ve tinh mat dat rieng)
    if is_ground_station_active():
        apply_state(STATE_GROUND_STATION)
        return

    # 3. Muc 3: Nguoi dung tuong tac truc tiep tren OpenWebRX
    if has_active_openwebrx_users():
        apply_state(STATE_USER_OPENWEBRX)
        return

    # 4. Muc 4: Khung gio tha bong tham khong khi tuong
    if is_in_balloon_window():
        apply_state(STATE_METEO_BALLOON)
        return

    # 5. Muc 5: Che do ranh thu xe o to RS41-NFW
    apply_state(STATE_CAR_TRACKER)

def main():
    logging.info("Khoi dong SDR 5-Tier Priority Arbiter...")
    while True:
        try:
            evaluate_and_arbitrate()
        except Exception as e:
            logging.error(f"Loi trong vong lap dieu phoi: {e}")
        time.sleep(CHECK_INTERVAL_SEC)

if __name__ == "__main__":
    main()
