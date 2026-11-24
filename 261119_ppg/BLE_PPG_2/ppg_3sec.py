"""
PPG 3-Second Capture — Collect 3s of 1000 Hz data, then play back waveform
in real-time. Press [Re-measure] to capture again.

Requirements: pip install bleak matplotlib numpy scipy
"""

import asyncio
import threading
import time
import csv
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import numpy as np
from scipy.signal import butter, sosfilt
from bleak import BleakClient, BleakScanner

DEVICE_NAME  = "BLE_PPG"
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"

SAMPLE_RATE     = 1000
CAPTURE_SECONDS = 3
TARGET_SAMPLES  = SAMPLE_RATE * CAPTURE_SECONDS

SAMPLES_PER_PKT = 10
PKT_LEN = 1 + SAMPLES_PER_PKT * 6

sos_bp = butter(4, [0.5, 5.0], btype='band', fs=SAMPLE_RATE, output='sos')
DECIMATE = 4


class PPGCapture:
    def __init__(self):
        self.reset()
        self.client = None
        self.ppg_char = None
        self.loop = None
        self.connected = False

    def reset(self):
        self.ir_samples = []
        self.red_samples = []
        self.last_seq = -1
        self.lost_packets = 0
        self.total_packets = 0
        self.capturing = False
        self.capture_done = False
        self.pleth_dec = None
        self.ir_dec = None
        self.t_dec = None
        self.spo2 = None
        self.pr = None
        self.pi_val = None
        self.playback_start = None

    def parse_notification(self, sender, data: bytearray):
        if not self.capturing:
            return
        if len(data) < PKT_LEN:
            return

        seq = data[0]
        self.total_packets += 1
        if self.last_seq >= 0:
            expected = (self.last_seq + 1) & 0xFF
            if seq != expected:
                self.lost_packets += (seq - expected) & 0xFF
        self.last_seq = seq

        for i in range(SAMPLES_PER_PKT):
            if len(self.ir_samples) >= TARGET_SAMPLES:
                self.capturing = False
                self._finish()
                return
            offset = 1 + i * 6
            red = (data[offset] << 16) | (data[offset+1] << 8) | data[offset+2]
            ir  = (data[offset+3] << 16) | (data[offset+4] << 8) | data[offset+5]
            self.red_samples.append(red)
            self.ir_samples.append(ir)

    def _finish(self):
        ir_arr = np.array(self.ir_samples, dtype=float)
        red_arr = np.array(self.red_samples, dtype=float)
        filt_ir = sosfilt(sos_bp, ir_arr - np.mean(ir_arr))
        filt_red = sosfilt(sos_bp, red_arr - np.mean(red_arr))

        idx = np.arange(0, len(ir_arr), DECIMATE)
        self.t_dec = idx / SAMPLE_RATE
        self.pleth_dec = -filt_ir[idx]
        self.ir_dec = ir_arr[idx]

        ir_dc = np.mean(ir_arr)
        red_dc = np.mean(red_arr)
        ir_ac = np.max(filt_ir) - np.min(filt_ir)
        red_ac = np.max(filt_red) - np.min(filt_red)

        self.pi_val = (ir_ac / ir_dc * 100) if ir_dc > 1000 else 0
        self.spo2 = None
        self.pr = None

        if ir_dc > 1000 and red_dc > 1000 and ir_ac > 0:
            R = (red_ac / red_dc) / (ir_ac / ir_dc)
            sp = 110 - 25 * R
            self.spo2 = max(0, min(100, int(round(sp))))

            arr = -filt_ir
            threshold = np.std(arr) * 0.6
            peaks = []
            min_gap = int(SAMPLE_RATE * 0.4)
            for i in range(2, len(arr) - 2):
                if arr[i] > arr[i-1] and arr[i] >= arr[i+1] and arr[i] > threshold:
                    if peaks and (i - peaks[-1]) < min_gap:
                        if arr[i] > arr[peaks[-1]]:
                            peaks[-1] = i
                    else:
                        peaks.append(i)
            if len(peaks) >= 3:
                intervals = np.diff(peaks) / SAMPLE_RATE
                intervals = intervals[(intervals > 0.4) & (intervals < 2.0)]
                if len(intervals) >= 2:
                    self.pr = int(round(60.0 / np.median(intervals)))

        self.capture_done = True
        self.playback_start = None
        print(f"Capture done! {len(self.ir_samples)} samples, lost={self.lost_packets}")

    def save_csv(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"ppg_capture_{timestamp}.csv"
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['sample_index', 'time_s', 'red', 'ir'])
            for i in range(len(self.ir_samples)):
                writer.writerow([i, f'{i/SAMPLE_RATE:.4f}', self.red_samples[i], self.ir_samples[i]])
        print(f"Saved: {filename}")

    async def connect(self):
        print(f"Scanning for {DEVICE_NAME}...")
        device = None
        while device is None:
            found = await BleakScanner.discover(timeout=5.0, return_adv=True)
            for d, adv in found.values():
                name = adv.local_name or d.name or ""
                if name == DEVICE_NAME:
                    device = d
                    break
            if device is None:
                print("Not found, retrying...")
                await asyncio.sleep(2)

        print(f"Found: {device.name} [{device.address}]")
        self.client = BleakClient(device.address, timeout=20.0, mtu_size=65)
        await self.client.connect()
        self.connected = True
        print("Connected!")

        base_suffix = SERVICE_UUID.lower()[8:]
        for svc in self.client.services:
            for ch in svc.characteristics:
                if "notify" in ch.properties and ch.uuid.lower().endswith(base_suffix):
                    self.ppg_char = ch
        if self.ppg_char is None:
            raise RuntimeError("PPG characteristic not found")
        await self.client.start_notify(self.ppg_char, self.parse_notification)

    def start_capture(self):
        self.reset()
        self.capturing = True
        print(f"Capturing {TARGET_SAMPLES} samples...")

    async def run_loop(self):
        await self.connect()
        self.start_capture()
        while self.connected and self.client.is_connected:
            await asyncio.sleep(0.5)


cap = PPGCapture()


def main():
    cap.loop = asyncio.new_event_loop()
    threading.Thread(target=lambda: (asyncio.set_event_loop(cap.loop),
                     cap.loop.run_until_complete(cap.run_loop())), daemon=True).start()

    while not cap.connected:
        time.sleep(0.2)

    # --- Plot ---
    fig = plt.figure(figsize=(13, 7.5), facecolor='black')
    fig.canvas.manager.set_window_title(f'PPG 3-Second Capture ({SAMPLE_RATE} Hz)')

    ax1 = fig.add_axes([0.07, 0.52, 0.88, 0.38])
    ax2 = fig.add_axes([0.07, 0.12, 0.88, 0.35])
    for ax in [ax1, ax2]:
        ax.set_facecolor('black')
        ax.tick_params(colors='#555555', labelsize=9)
        for s in ['bottom', 'left']:
            ax.spines[s].set_color('#333333')
        for s in ['top', 'right']:
            ax.spines[s].set_visible(False)
        ax.grid(True, color='#1a1a1a', linewidth=0.5)
        ax.set_xlim(0, CAPTURE_SECONDS)

    ax1.set_ylabel('Pleth', color='#00ff66', fontsize=12, fontweight='bold')
    ax2.set_ylabel('IR raw', color='#00ccff', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Time (s)', color='#555555', fontsize=10)

    line_pleth, = ax1.plot([], [], color='#00ff66', linewidth=1.2)
    line_ir, = ax2.plot([], [], color='#00ccff', linewidth=0.8)

    vitals_text = fig.text(0.5, 0.96, '', color='#00ff66', fontsize=14,
                           ha='center', fontweight='bold')
    info_text = fig.text(0.5, 0.92, 'Collecting data...', color='#aaaaaa',
                         fontsize=11, ha='center')

    ax_prog = fig.add_axes([0.07, 0.49, 0.88, 0.008])
    ax_prog.set_xlim(0, 1); ax_prog.set_ylim(0, 1); ax_prog.axis('off')
    prog_bar = ax_prog.barh(0.5, 0, height=1.0, color='#00ff66')

    ax_btn = fig.add_axes([0.4, 0.02, 0.2, 0.045])
    btn = Button(ax_btn, 'Re-measure', color='#333333', hovercolor='#555555')
    btn.label.set_color('white'); btn.label.set_fontsize(12)

    saved = [False]

    def update(frame):
        # --- Phase 1: collecting ---
        if cap.capturing:
            n = len(cap.ir_samples)
            prog_bar[0].set_width(n / TARGET_SAMPLES)
            info_text.set_text(f"Collecting... {n}/{TARGET_SAMPLES}")
            vitals_text.set_text('')
            return line_pleth, line_ir

        # --- Phase 2: playback animation ---
        if cap.capture_done and cap.t_dec is not None:
            if cap.playback_start is None:
                cap.playback_start = time.time()
                # Set Y limits once
                p_min, p_max = np.min(cap.pleth_dec), np.max(cap.pleth_dec)
                p_rng = p_max - p_min if p_max > p_min else 1
                ax1.set_ylim(p_min - p_rng * 0.1, p_max + p_rng * 0.1)
                ir_min, ir_max = np.min(cap.ir_dec), np.max(cap.ir_dec)
                ir_rng = ir_max - ir_min if ir_max > ir_min else 1
                ax2.set_ylim(ir_min - ir_rng * 0.05, ir_max + ir_rng * 0.05)
                prog_bar[0].set_width(1.0)
                info_text.set_text('Playing back...')

            elapsed = time.time() - cap.playback_start
            # Show up to elapsed time (play at real speed)
            show_t = min(elapsed, CAPTURE_SECONDS)
            mask = cap.t_dec <= show_t
            line_pleth.set_data(cap.t_dec[mask], cap.pleth_dec[mask])
            line_ir.set_data(cap.t_dec[mask], cap.ir_dec[mask])

            if elapsed >= CAPTURE_SECONDS:
                # Playback done — show results
                spo2_str = f"{cap.spo2}" if cap.spo2 else "--"
                pr_str = f"{cap.pr}" if cap.pr else "--"
                pi_str = f"{cap.pi_val:.2f}" if cap.pi_val else "--"
                vitals_text.set_text(
                    f"SpO₂: {spo2_str}%    PR: {pr_str} bpm    PI: {pi_str}%")
                info_text.set_text(
                    f"Samples: {len(cap.ir_samples)}    "
                    f"Rate: {SAMPLE_RATE} Hz    "
                    f"Duration: {CAPTURE_SECONDS}s    "
                    f"Lost: {cap.lost_packets}")
                if not saved[0]:
                    cap.save_csv()
                    saved[0] = True

        return line_pleth, line_ir

    def on_remeasure(event):
        saved[0] = False
        line_pleth.set_data([], [])
        line_ir.set_data([], [])
        vitals_text.set_text('')
        info_text.set_text('Collecting data...')
        prog_bar[0].set_width(0)
        cap.start_capture()

    btn.on_clicked(on_remeasure)
    ani = animation.FuncAnimation(fig, update, interval=33, blit=False,
                                  cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
