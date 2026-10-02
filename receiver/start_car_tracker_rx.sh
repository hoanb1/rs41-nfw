#!/bin/bash
# Script khoi dong bo thu RTL-SDR giai ma tin hieu xe hoi & trạm khi tuong (Horus Binary V3 4FSK)
# Tan so mac dinh: 437.600 MHz

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FREQ=${1:-"437.600M"}
DEVICE_INDEX=${2:-"0"}

# Python and Demodulator binary detection
PYTHON_BIN="python3"
if [ -f "/home/pi/horus-venv/bin/python3" ]; then
    PYTHON_BIN="/home/pi/horus-venv/bin/python3"
fi

HORUS_DEMOD="horus_demod"
if [ -f "/home/pi/horus-venv/bin/horus_demod" ]; then
    HORUS_DEMOD="/home/pi/horus-venv/bin/horus_demod"
fi

DECODER_SCRIPT="$SCRIPT_DIR/car_tracker_decoder.py"
if [ ! -f "$DECODER_SCRIPT" ] && [ -f "/home/pi/car_tracker_decoder.py" ]; then
    DECODER_SCRIPT="/home/pi/car_tracker_decoder.py"
fi

echo "=========================================================="
echo "Khoi dong bo thu RTL-SDR tai tan so: $FREQ (Device $DEVICE_INDEX)"
echo "Ingest API : https://api.hoan.uk/api/v1/telemetry/ingest"
echo "MQTT Broker: mqtt.hoan.uk:1883"
echo "=========================================================="

exec rtl_fm -d "$DEVICE_INDEX" -M usb -f "$FREQ" -s 48k -g 40 -p 0 2>/dev/null \
  | $HORUS_DEMOD -m binary --sample-rate 48000 --rate 100 --tonespacing 803 -t 5 - \
  | $PYTHON_BIN -u "$DECODER_SCRIPT" --url "https://api.hoan.uk/api/v1/telemetry/ingest" --mqtt-broker "mqtt.hoan.uk"
