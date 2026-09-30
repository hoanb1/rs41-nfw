# 🚗 RS41 Car Tracker — Hệ Thống Định Vị & Giám Sát Hành Trình Ô Tô

> **Chuyển đổi bóng thám không khí tượng Vaisala RS41 thành thiết bị định vị & cảm biến môi trường tầm xa, siêu tiết kiệm năng lượng cho xe ô tô.**

Tài liệu thiết kế kiến trúc chuẩn IEEE 1016: xem tại [`docs/SDD.md`](./docs/SDD.md).

---

## 🌟 Điểm nổi bật (Key Features)

- **🔋 Thời lượng Pin vượt trội (15 - 45+ ngày chỉ với 2 viên pin AA)**:
  - Tự động nhận diện trạng thái di chuyển / đỗ xe dựa trên GPS u-blox M10.
  - Chu kỳ phát sóng thông minh: **180s (3 phút) khi xe chạy / 900s (15 phút) khi xe đỗ**.
  - Dòng tiêu thụ trung bình khi đỗ xe chỉ **~2.0 - 2.3 mA**.
- **🌡️ Bảo toàn que đo cảm biến Nhiệt độ & Độ ẩm (Sensor Boom)**:
  - Không lãng phí cụm cảm biến Pt1000 và màng đo ẩm điện dung cao cấp của Vaisala.
  - Áp dụng cơ chế **Duty-cycling chủ động**: Cấp điện đo trong **~80 ms**, sau đó ngắt nguồn hoàn toàn bằng lệnh phần cứng `selectSensorBoom(0, 0)` đưa dòng rò về **0 mA** giữa các chu kỳ.
- **🛡️ Triệt tiêu hoàn toàn tọa độ lỗi (0,0 - Null Island)**:
  - Bảo vệ 3 tầng (Trạm thu $\rightarrow$ Ingestion Engine $\rightarrow$ InfluxDB / Client) ngăn chặn vệt tọa độ 0,0 nhảy sang vị trí biển Tây Phi khi xe đỗ trong hầm hoặc nhà xe có mái che.
  - Tự động duy trì vị trí hợp lệ gần nhất (`isLastKnown = true`) trên bản đồ.
- **🔘 Điều khiển đa năng chỉ với 1 nút bấm (Multi-Function Button)**:
  - *Nhấn 1 lần*: Kiểm tra trạng thái máy qua đèn LED (2 nháy Xanh = Tốt, 2 nháy Đỏ = Lỗi/Yếu).
  - *Nhấn đúp (2 lần)*: Phát vị trí khẩn cấp ngay lập tức (Force TX không cần chờ chu kỳ).
  - *Nhấn 3 lần*: Đổi linh hoạt giữa 3 Profile năng lượng (Active $\rightarrow$ Eco $\rightarrow$ Ultra).
  - *Giữ > 2.5 giây*: **TẮT NGUỒN HOÀN TOÀN** qua mạch MOSFET ngắt pin triệt để.
- **💻 Cổng Serial CLI (XDATA 9600 bps)**:
  - Cho phép cấu hình chuyên sâu: cài đặt chu kỳ phát, công suất RF, tần số, chu kỳ đo que nhiệt ẩm, xem trạng thái chi tiết qua lệnh `STATUS`.
- **📡 Tầm xa vô tuyến Sub-1GHz (Horus Binary V3 4FSK)**:
  - Điều chế 4FSK tốc độ 100 baud tại tần số 437.600 MHz với công suất 100 mW (20 dBm).
  - Giải mã cực nhạy tại trạm thu SDR với ngưỡng SNR âm ($<-10\text{ dB}$).

---

## ⚡ Bảng Profile Năng Lượng

| Profile | Khi xe di chuyển | Khi xe đỗ / Dừng | Chu kỳ đo Nhiệt / Ẩm | Tuổi thọ pin (2x AA Alkaline) | Tuổi thọ pin (2x AA Lithium) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Profile 1 (Active)** | 60s (1 phút) | 300s (5 phút) | 5 phút / lần | ~7 - 10 ngày | ~12 - 15 ngày |
| **Profile 2 (Eco - Mặc định)** | **180s (3 phút)** | **900s (15 phút)** | **15 phút / lần** | **~18 - 25 ngày** | **~30 - 35 ngày** |
| **Profile 3 (Ultra Deep-Save)** | **300s (5 phút)** | **1800s (30 phút)** | **30 phút / lần** | **~35 - 45 ngày** | **~50 - 65 ngày** |

---

## 📂 Cấu Trúc Thư Mục Dự Án

```text
rs41-nfw/
├── docs/
│   └── SDD.md                     <-- Tài liệu thiết kế hệ thống chi tiết chuẩn IEEE 1016
├── rs41-tracker-firmware/         <-- Firmware chính thức cho STM32L412
│   ├── rs41-tracker-firmware.ino  <-- Mã nguồn chính (Arduino Sketch)
│   ├── CONFIG.h                   <-- Tệp cấu hình toàn bộ thông số hoạt động
│   ├── HorusBinaryV3.c / .h       <-- Bộ đóng gói giao thức ASN.1
│   └── build/                     <-- Thư mục chứa file binary sau khi biên dịch
├── receiver/                      <-- Phân hệ máy thu RTL-SDR & Chuyển tiếp
│   ├── car_tracker_decoder.py     <-- Script bóc tách telemetry, lọc 0,0 & đẩy lên hoan.uk
│   ├── start_car_tracker_rx.sh    <-- Khởi động đường ống rtl_fm + horus_demod
│   └── car-tracker-rx.service     <-- Cấu hình dịch vụ nền Linux Systemd
├── tools/                         <-- Tiện ích mở rộng & Sounding Software
├── .gitignore                     <-- Khai báo loại trừ build artifacts & cache
└── README.md                      <-- Hướng dẫn nhanh cho người vận hành
```

---

## 🛠️ Hướng Dẫn Biên Dịch & Nạp Firmware

### 1. Biên dịch Firmware bằng `arduino-cli`
```bash
/home/hoan/bin/arduino-cli compile \
  --fqbn STMicroelectronics:stm32:GenL4:pnum=GENERIC_L412RBTXP \
  --output-dir rs41-tracker-firmware/build \
  rs41-tracker-firmware
```

### 2. Nạp vi điều khiển STM32L412 bằng OpenOCD (CMSIS-DAP / ST-Link)
```bash
openocd -f interface/cmsis-dap.cfg -c "adapter speed 500" -f target/stm32l4x.cfg \
  -c "init; reset halt; program rs41-tracker-firmware/build/rs41-tracker-firmware.ino.bin 0x08000000 verify reset; exit"
```

---

## 📡 Cấu Hình Trạm Thu & Giám Sát Web

### 1. Khởi động dịch vụ trạm thu trên Raspberry Pi
```bash
sudo cp receiver/car-tracker-rx.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now car-tracker-rx.service
systemctl status car-tracker-rx.service
```

### 2. Xem trực tuyến trên Web Portal
Mọi dữ liệu từ xe (vị trí GPS, vận tốc, lộ trình, điện áp pin, nhiệt độ que đo, độ ẩm không khí) được cập nhật theo thời gian thực tại:
👉 **`https://hoan.uk/devices`**

---

## 📜 Giấy Phép & Tác Quyền
Dự án được kế thừa và phát triển từ mã nguồn mở `rs41-nfw` theo giấy phép **GPL-3.0**. Vui lòng tham khảo tệp [`LICENSING.md`](./LICENSING.md) để biết thêm chi tiết.
