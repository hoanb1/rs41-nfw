#!/bin/bash
# Script khoi dong bo thu RTL-SDR giai ma tin hieu xe hoi (Horus Binary V3 4FSK)
# Tan so mac dinh: 437.600 MHz

set -o pipefail

FREQ=${1:-"437.600M"}
DEVICE_INDEX=${2:-"0"}

echo "=========================================================="
echo "Khoi dong bo thu RTL-SDR Horus V3 tai tan so: $FREQ (Device $DEVICE_INDEX)"
echo "=========================================================="

exec rtl_fm -d "$DEVICE_INDEX" -M usb -f "$FREQ" -s 48k -g 40 -p 0 2>/dev/null \
  | /home/pi/horus-venv/bin/horus_demod -m binary --sample-rate 48000 --rate 100 --tonespacing 803 -t 5 - \
  | /home/pi/horus-venv/bin/python3 -u /home/pi/car_tracker_decoder.py
