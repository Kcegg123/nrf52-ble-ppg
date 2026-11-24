# 改動記錄 2026-11-19 ~ 11-20

## 1. PPG 外殼 v2（`ppg_case/`）

新增 `make_case_v2.py`（voxel CSG，純 numpy，解析度 0.2mm），產出 `out/PPG_case_v2.stl` / `out/PPG_case_v2.3mf`。

最終設計參數：
| 項目 | 數值 |
|------|------|
| 外觀尺寸 | 33 × 37 × 35 mm（Y 軸後來 +4mm） |
| 壁厚 / 底板 | 1.5 / 1.2 mm |
| 感測器開窗（右壁 X=33 面） | 20 × 20 mm（原 15×15，後加大） |
| USB-C 開孔（左壁 X=0 面） | 10 × 4 mm，中心離底 **12 mm** |
| 內部支撐橫條（前後壁） | 離底 **27 mm**，厚 1.2mm、深 3mm |
| 線槽缺口（前後壁頂部） | 8 × 5 mm |
| 頂部 | 開放 |

Mesh 通過 watertight 驗證，預估列印 ~20 分鐘。

## 2. XIAO nRF52840 Plus 燒錄（`261119_ppg/`）

- `BLE_PPG_Arduino/`：1000Hz PPG 韌體 Arduino 版（Bluefruit / S140），封包與舊板相同（[seq]+10×[RED3+IR3]=61B），編譯產出 `build/BLE_PPG_Arduino.uf2`。
- Windows 11 無法掛載 UF2 bootloader 虛擬磁碟（已知相容性問題）；serial DFU 的 COM 埠也打不開。裝了 Zadig（libusb driver）後磁碟出現在 Get-Disk 但檔案系統仍不可讀。
- **最終成功方案**：`flash_xiao_safe.py` — 以磁碟名稱「Adafruit nRF UF2」+ 廠商字串 + 容量 <128MB 三重驗證鎖定目標磁碟後，用 write-through 逐 512B 區塊寫入 UF2（等同 Linux `dd` 的 UF2 燒錄方式）。燒錄成功，板子快閃 = 韌體執行中、等待 MAX30102。

## 3. ⚠ 系統碟 GPT 事故與修復

- `flash_uf2_v3.py`（已停用）含「逐一嘗試裝置路徑」的錯誤 fallback，在 UF2 磁碟未列舉時打開了 **PhysicalDrive0（系統 SSD）**，把 467 個 UF2 區塊寫到 LBA 0–466，毀掉保護性 MBR + 主 GPT（分割區資料未受損，首個分割區在 LBA 2048）。
- `repair_gpt.py`：從磁碟尾端的備份 GPT 驗證（簽章+CRC）後重建主 GPT 表、主標頭與保護性 MBR，寫前先把現場 dump 到 `261119_ppg/gpt_rescue/`。**修復完成並驗證**（Primary GPT header is VALID；分割區 EFI/MSR/C:/Recovery 全部正常）。
- `flash_uf2.py`、`flash_uf2_raw.py`、`flash_uf2_v3.py` 皆已改為停用 stub。教訓已寫入長期記憶：**絕不對候選路徑清單做破壞性寫入；raw 寫入前必須正向驗證裝置身分**。

## 4. 目前狀態 / 待辦

- [x] 外殼 v2 檔案可直接進 Bambu Studio 切片
- [x] 新板燒錄完成（1000Hz 韌體）
- [ ] 新板接上 MAX30102（VIN→3V3、GND→GND、SDA→D4、SCL→D5）後 LED 應轉慢閃並廣播 `BLE_PPG`
  - 最後一次接線後板子完全不亮 → 疑似短路，**待檢查接線**
- [ ] 舊板（D7:B5:89:C8:5E:70）持續可用，`BLE_PPG_2/ppg_3sec.py` 量測正常
