# nRF52 BLE PPG Monitor

BLE 光體積變化描記（PPG）監測器，使用 nRF52840 + MAX30102，透過 BLE 以 1000 Hz 傳輸 RED/IR 原始資料。

## 硬體

| 元件 | 型號 |
|------|------|
| MCU | ProMicro nRF52840 |
| PPG 感測器 | MH-ET LIVE MAX30102 |
| SoftDevice | S132 v3.1.0 |

### 接線

| MAX30102 | nRF52840 |
|----------|----------|
| VCC | VIN |
| GND | GND |
| SDA | P1.13 |
| SCL | P1.11 |

P0.13 控制 MAX30102 電源開關，P0.06 為 LED 狀態指示燈。

## 功能

- VDD 設為 3.3V（UICR REGOUT0）
- MAX30102 以 1000 Hz 取樣（無平均），SpO2 模式，RED + IR 雙通道
- BLE 封包：1 byte 序號 + 10 組 (RED 3B + IR 3B) = 61 bytes
- 廣播中：LED 每秒閃一次；已連線：LED 每秒閃兩次

## 專案結構

```
261119_ppg/BLE_PPG_2/
├── main.c                     # 韌體主程式
├── ppg_3sec.py                # PC 端 3 秒擷取 + 波形回放腳本
└── pca10056/s132/
    ├── config/sdk_config.h    # SDK 模組設定
    └── arm5_no_packs/
        ├── ble_ppg_pca10056_s132.uvprojx  # Keil 專案檔
        └── RTE/               # 啟動檔

case_stl/                      # 3D 列印外殼 STL（本體、上蓋、墊片板）
s132_nrf52_3.1.0_softdevice.hex  # SoftDevice 燒錄檔
```

## 建置

### 需求

- Keil MDK-ARM v5
- [nRF5 SDK 12.3.0](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download)（解壓至 `SDK12/`）

### 編譯與燒錄

1. 用 Keil 開啟 `ble_ppg_pca10056_s132.uvprojx`
2. 確認 SDK 路徑指向 `SDK12/`
3. 先燒錄 SoftDevice：`s132_nrf52_3.1.0_softdevice.hex`
4. 再燒錄應用程式

## PC 端擷取

```bash
pip install bleak matplotlib numpy scipy
python ppg_3sec.py
```

連線後自動擷取 3 秒 PPG 資料，回放波形並計算 SpO2、心率、PI，結果存為 CSV。

## 外殼

`case_stl/` 內含可直接列印的 STL 檔：

| 檔案 | 說明 |
|------|------|
| `ppg_case_body_v2.stl` | 外殼本體 |
| `ppg_case_lid_v2.stl` | 上蓋 |
| `ppg_spacer_plate_v3.stl` | 墊片板 |
| `ppg_print_all_final.stl` | 全部零件合併版 |

外觀尺寸約 33 x 37 x 35 mm，含 USB-C 開孔與感測器窗口。

## License

MIT
