#!/usr/bin/env python3
"""
松灵 Ranger Delta 舵轮状态与转向到位实时监视及记录工具
(Ranger Delta Wheel State Monitor & Diagnostic Logger)

功能:
1. 实时监听 CAN (0x271 转向角, 0x281 轮速, 0x291 模式, 0x211 系统, 0x265~0x268 舵机驱动器)
2. 实时判定每个轮子的朝向状态:
   - 竖向 (纵向/直行): |角度| <= 15° 或 ||角度|-180°| <= 15°
   - 横向 (横移): ||角度|-90°| <= 15° (即 +75°~+105° 或 -105°~-75°)
   - 斜向 (过渡/倾斜): 其他角度
3. 判定各轮是否达到目标角度 (横移时是否到位)，检测是否有舵轮卡阻或未转到位
4. 自动写入 CSV 诊断日志，并在退出时生成统计诊断报告
"""

import argparse
import csv
import datetime
import math
import os
import signal
import socket
import struct
import sys
import threading
import time
from pathlib import Path


# CAN IDs
SYSTEM_STATUS = 0x211
CHASSIS_MOTION = 0x221
STEERING_ANGLES = 0x271
WHEEL_SPEEDS = 0x281
MOTION_MODE = 0x291
SET_MOTION = 0x111
SET_MOTION_MODE = 0x141

# Motor 5~8 Low speed feedback: 0x265 (右前), 0x266 (右后), 0x267 (左后), 0x268 (左前)
MOTOR_STEER_BASE = 0x265

MODE_NAMES = {
    0: "双阿克曼 (Dual Ackermann)",
    1: "平行横移 (Parallel Crab)",
    2: "原地自旋 (Spin Turn)",
    3: "驻车模式 (Parking)",
    4: "侧滑模式 (Side Slip)",
}

WHEEL_NAMES = {
    1: "右前轮 (W1 Front-Right)",
    2: "右后轮 (W2 Rear-Right)",
    3: "左后轮 (W3 Rear-Left)",
    4: "左前轮 (W4 Front-Left)",
}


def classify_direction(angle_deg: float) -> str:
    """判定轮子朝向: 竖向 / 横向 / 斜向"""
    a = abs(angle_deg)
    # 竖向: 接近 0 度或 180 度 (允许 ±15° 偏差)
    if a <= 15.0 or abs(a - 180.0) <= 15.0:
        return "竖向"
    # 横向: 接近 90 度 (允许 ±15° 偏差, 即 75°~105°)
    if abs(a - 90.0) <= 15.0:
        return "横向"
    # 其余为斜向
    return "斜向"


class WheelMonitor:
    def __init__(self, interface="can0", log_dir=None):
        self.interface = interface
        self.running = False
        self.can_socket = None
        self.rx_thread = None

        # Data state
        self.lock = threading.Lock()
        self.can_connected = False
        self.frame_count = 0
        self.last_rx_time = 0.0

        # Chassis general state
        self.battery_voltage = 0.0
        self.control_mode = "未知"
        self.motion_mode_id = 0
        self.switch_status = 0
        self.linear_speed_fb = 0.0
        self.angular_speed_fb = 0.0

        # Commanded state
        self.cmd_linear_x = 0.0
        self.cmd_linear_y = 0.0
        self.cmd_angular_z = 0.0
        self.target_steer_deg = 0.0

        # 4 Wheels state: 1 (右前), 2 (右后), 3 (左后), 4 (左前)
        self.steering_rad = [0.0, 0.0, 0.0, 0.0]
        self.steering_deg = [0.0, 0.0, 0.0, 0.0]
        self.wheel_speed = [0.0, 0.0, 0.0, 0.0]

        # History tracking for summary stats
        self.history_min_deg = [999.0, 999.0, 999.0, 999.0]
        self.history_max_deg = [-999.0, -999.0, -999.0, -999.0]
        self.reached_transverse = [False, False, False, False]
        self.reached_longitudinal = [False, False, False, False]

        # Steering Motors (5, 6, 7, 8) status
        self.motor_enabled = [False, False, False, False]
        self.motor_error = [False, False, False, False]
        self.motor_temp = [0, 0, 0, 0]
        self.motor_voltage = [0.0, 0.0, 0.0, 0.0]

        # CSV Logging
        if log_dir is None:
            log_dir = Path("/home/leona/WorkingSpace/navigation2/project_slam/logs/wheel_monitor")
        else:
            log_dir = Path(log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = log_dir / f"wheel_record_{timestamp_str}.csv"
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.write_csv_header()

        self.start_time = time.time()
        self.logged_rows = 0

    def write_csv_header(self):
        self.csv_writer.writerow([
            "timestamp", "time_iso", "motion_mode_id", "motion_mode_name", "switch_status",
            "cmd_vx", "cmd_vy", "cmd_wz", "target_steer_deg",
            "w1_deg", "w1_dir", "w1_speed_mps", "w1_aligned",
            "w2_deg", "w2_dir", "w2_speed_mps", "w2_aligned",
            "w3_deg", "w3_dir", "w3_speed_mps", "w3_aligned",
            "w4_deg", "w4_dir", "w4_speed_mps", "w4_aligned",
            "battery_vol",
            "m5_rf_enabled", "m5_rf_err", "m5_rf_temp",
            "m6_rr_enabled", "m6_rr_err", "m6_rr_temp",
            "m7_lr_enabled", "m7_lr_err", "m7_lr_temp",
            "m8_lf_enabled", "m8_lf_err", "m8_lf_temp",
        ])
        self.csv_file.flush()

    def start(self):
        self.running = True
        try:
            self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_socket.bind((self.interface,))
            self.can_socket.settimeout(1.0)
            self.can_connected = True
        except Exception as e:
            self.can_connected = False
            print(f"\033[31m[错误] 无法绑定 CAN 接口 {self.interface}: {e}\033[0m")
            print("若车辆下电或接口未启用，可在机器人上电后使用: sudo ip link set can0 up type can bitrate 500000")

        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def _rx_loop(self):
        while self.running:
            if not self.can_connected:
                # Retry connection periodically
                try:
                    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                    s.bind((self.interface,))
                    s.settimeout(1.0)
                    self.can_socket = s
                    self.can_connected = True
                except Exception:
                    time.sleep(1.0)
                    continue

            try:
                frame = self.can_socket.recv(16)
                if len(frame) == 16:
                    can_id, length, data = struct.unpack("=IB3x8s", frame)
                    self.handle_can_frame(can_id, data[:length])
            except socket.timeout:
                continue
            except Exception:
                if self.running:
                    self.can_connected = False
                    time.sleep(0.5)

    def handle_can_frame(self, can_id, data):
        now = time.time()
        with self.lock:
            self.last_rx_time = now
            self.frame_count += 1

            if can_id == SYSTEM_STATUS and len(data) >= 8:
                # struct '>BBHI'
                status, ctrl_mode, vol_tenths, _ = struct.unpack(">BBHI", data[:8])
                ctrl_names = {0: "待机", 1: "CAN控制", 2: "串口", 3: "遥控"}
                self.control_mode = ctrl_names.get(ctrl_mode, f"模式{ctrl_mode}")
                self.battery_voltage = vol_tenths / 10.0

            elif can_id == CHASSIS_MOTION and len(data) >= 8:
                # struct '>hhxxh'
                vx_mm, wz_mrad, steer_mrad = struct.unpack(">hhxxh", data[:8])
                self.linear_speed_fb = vx_mm / 1000.0
                self.angular_speed_fb = wz_mrad / 1000.0

            elif can_id == MOTION_MODE and len(data) >= 3:
                # struct '>BBB'
                mode, sw, _ = struct.unpack(">BBB", data[:3])
                self.motion_mode_id = mode
                self.switch_status = sw

            elif can_id == STEERING_ANGLES and len(data) >= 8:
                # struct '>hhhh' (mrad)
                s1, s2, s3, s4 = struct.unpack(">hhhh", data[:8])
                rads = [s1 / 1000.0, s2 / 1000.0, s3 / 1000.0, s4 / 1000.0]
                degs = [math.degrees(r) for r in rads]
                self.steering_rad = rads
                self.steering_deg = degs

                for i in range(4):
                    d = degs[i]
                    if d < self.history_min_deg[i]:
                        self.history_min_deg[i] = d
                    if d > self.history_max_deg[i]:
                        self.history_max_deg[i] = d
                    dir_name = classify_direction(d)
                    if dir_name == "横向":
                        self.reached_transverse[i] = True
                    elif dir_name == "竖向":
                        self.reached_longitudinal[i] = True

                # Write row to CSV
                self.log_csv_row(now)

            elif can_id == WHEEL_SPEEDS and len(data) >= 8:
                # struct '>hhhh' (mm/s)
                w1, w2, w3, w4 = struct.unpack(">hhhh", data[:8])
                self.wheel_speed = [w1 / 1000.0, w2 / 1000.0, w3 / 1000.0, w4 / 1000.0]

            elif can_id == SET_MOTION and len(data) >= 8:
                # Controller output: speed, yaw_rate, reserved, steering
                spd_mm, _, steer_mrad = struct.unpack(">hhxxh", data[:8])
                self.target_steer_deg = math.degrees(steer_mrad / 1000.0)

            elif 0x265 <= can_id <= 0x268 and len(data) >= 5:
                # Motor 5~8 low speed feedback
                idx = can_id - 0x265  # 0: M5(RF), 1: M6(RR), 2: M7(LR), 3: M8(LF)
                vol_tenths = (data[0] | (data[1] << 8)) if len(data) >= 2 else 0
                drv_temp = (data[2] | (data[3] << 8)) if len(data) >= 4 else 0
                status_byte = data[4]
                self.motor_voltage[idx] = vol_tenths / 10.0
                self.motor_temp[idx] = drv_temp
                self.motor_enabled[idx] = bool((status_byte >> 6) & 1)
                self.motor_error[idx] = bool((status_byte >> 5) & 1)

    def log_csv_row(self, timestamp):
        dt_iso = datetime.datetime.fromtimestamp(timestamp).isoformat(timespec="milliseconds")
        mode_str = MODE_NAMES.get(self.motion_mode_id, f"Mode{self.motion_mode_id}")
        row = [
            f"{timestamp:.3f}", dt_iso, self.motion_mode_id, mode_str, self.switch_status,
            f"{self.cmd_linear_x:.3f}", f"{self.cmd_linear_y:.3f}", f"{self.cmd_angular_z:.3f}",
            f"{self.target_steer_deg:.1f}",
        ]
        for i in range(4):
            deg = self.steering_deg[i]
            d_type = classify_direction(deg)
            spd = self.wheel_speed[i]
            # Is aligned with target?
            err = abs(deg - self.target_steer_deg)
            aligned = "YES" if err <= 8.0 else f"NO(err={err:.1f}°)"
            row.extend([f"{deg:.2f}", d_type, f"{spd:.3f}", aligned])

        row.append(f"{self.battery_voltage:.1f}")
        for idx in range(4):
            row.extend([
                "1" if self.motor_enabled[idx] else "0",
                "1" if self.motor_error[idx] else "0",
                str(self.motor_temp[idx]),
            ])

        self.csv_writer.writerow(row)
        self.logged_rows += 1
        if self.logged_rows % 10 == 0:
            self.csv_file.flush()

    def get_snapshot(self):
        with self.lock:
            now = time.time()
            alive = (now - self.last_rx_time < 0.5) if self.last_rx_time > 0 else False
            return {
                "alive": alive,
                "voltage": self.battery_voltage,
                "control_mode": self.control_mode,
                "mode_id": self.motion_mode_id,
                "mode_text": MODE_NAMES.get(self.motion_mode_id, "未知模式"),
                "switch_status": "切换中" if self.switch_status == 1 else "就绪/完成",
                "target_steer_deg": self.target_steer_deg,
                "steering_deg": list(self.steering_deg),
                "wheel_speed": list(self.wheel_speed),
                "history_min": list(self.history_min_deg),
                "history_max": list(self.history_max_deg),
                "reached_transverse": list(self.reached_transverse),
                "reached_longitudinal": list(self.reached_longitudinal),
                "motor_enabled": list(self.motor_enabled),
                "motor_error": list(self.motor_error),
                "motor_temp": list(self.motor_temp),
                "logged_rows": self.logged_rows,
                "csv_path": str(self.csv_path),
            }

    def close(self):
        self.running = False
        if self.can_socket:
            try:
                self.can_socket.close()
            except Exception:
                pass
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()

    def print_diagnostic_summary(self):
        snap = self.get_snapshot()
        print("\n" + "=" * 80)
        print("          🔍 松灵 Ranger Delta 舵轮转向到位诊断分析总结报告")
        print("=" * 80)
        print(f"日志记录文件: \033[36m{snap['csv_path']}\033[0m")
        print(f"有效数据帧数: {snap['logged_rows']} 帧 | 运行时间: {time.time() - self.start_time:.1f} 秒\n")

        print("【4个车轮舵向极限与状态统计】:")
        print(f"{'轮位':<8} {'轮号':<6} {'最小角度':<12} {'最大角度':<12} {'当前角度':<12} {'当前朝向':<10} {'曾到横向(±90°)?':<18} {'曾到竖向(0°)?'}")
        print("-" * 88)

        wheel_codes = ["W1(右前)", "W2(右后)", "W3(左后)", "W4(左前)"]
        all_transverse = True
        failed_wheels = []

        for i in range(4):
            min_d = snap['history_min'][i]
            max_d = snap['history_max'][i]
            cur_d = snap['steering_deg'][i]
            cur_dir = classify_direction(cur_d)
            did_trans = snap['reached_transverse'][i]
            did_long = snap['reached_longitudinal'][i]

            min_str = f"{min_d:+.1f}°" if min_d < 900 else "--"
            max_str = f"{max_d:+.1f}°" if max_d > -900 else "--"
            cur_str = f"{cur_d:+.1f}°"
            trans_str = "\033[32m✔ 是 (已达到)\033[0m" if did_trans else "\033[31m✘ 否 (未转到位!)\033[0m"
            long_str = "\033[32m✔ 是\033[0m" if did_long else "✘ 否"

            if not did_trans:
                all_transverse = False
                failed_wheels.append((wheel_codes[i], min_d, max_d, cur_d))

            dir_color = "\033[34m" if cur_dir == "横向" else ("\033[32m" if cur_dir == "竖向" else "\033[33m")
            dir_str = f"{dir_color}[{cur_dir}]\033[0m"

            print(f"{wheel_codes[i]:<10} W{i+1:<5} {min_str:<12} {max_str:<12} {cur_str:<12} {dir_str:<16} {trans_str:<26} {long_str}")

        print("-" * 88)
        print("\n【核心诊断结论与建议】:")
        if all_transverse:
            print("  \033[32m🟢 4个舵轮均曾成功旋转至横向 (约±90°)，硬件舵轮旋转机械行程完全正常。\033[0m")
        else:
            print(f"  \033[31m🔴 发现 {len(failed_wheels)} 个轮子在整个记录期间【未能转到横向 (±90°±15°)】:\033[0m")
            for w_name, mn, mx, cur in failed_wheels:
                print(f"     👉 \033[33m{w_name}\033[0m: 当前角度={cur:+.1f}°, 行程范围=[{mn:+.1f}° ~ {mx:+.1f}°]")
            print("\n  💡 排查建议:")
            print("     1. 请检查上述受阻轮子的舵机减速箱、机械限位或对地转向阻力；")
            print("     2. 检查下方的舵机驱动器报警状态 (0x265~0x268 是否有过流、过温或传感器报警)；")
            print("     3. 完整时序数据已保存在上方 CSV 文件中，可用 Excel 或 Python 绘制各轮角度曲线。")

        print("=" * 80 + "\n")


def run_live_tui(monitor: WheelMonitor):
    """终端实时看板刷新 (ANSI 清屏平滑更新)"""
    sys.stdout.write("\033[2J")  # Clear screen once
    wheel_labels = [
        ("W1", "右前轮 (Front-Right)"),
        ("W2", "右后轮 (Rear-Right) "),
        ("W3", "左后轮 (Rear-Left)  "),
        ("W4", "左前轮 (Front-Left) "),
    ]

    while monitor.running:
        snap = monitor.get_snapshot()

        lines = []
        lines.append("\033[H")  # Move cursor to home (top-left)
        lines.append("\033[1;37;44m" + " 松灵 Ranger Delta 舵轮状态与转向到位实时监视器 (Wheel State Monitor) ".center(80) + "\033[0m")

        status_color = "\033[32m🟢 正常在线 (50Hz)\033[0m" if snap["alive"] else "\033[31m🔴 离线/等待数据 (请确认底盘已上电并配置can0)\033[0m"
        lines.append(f"通信状态: {status_color} | 电池电压: {snap['voltage']:.1f} V | 控制模式: {snap['control_mode']}")
        lines.append(f"运动模式: \033[36m{snap['mode_text']}\033[0m (状态: {snap['switch_status']}) | 期望转角: {snap['target_steer_deg']:+.1f}°")
        lines.append(f"记录文件: \033[90m{snap['csv_path']}\033[0m (已记录: \033[33m{snap['logged_rows']}\033[0m 帧)")
        lines.append("=" * 80)

        lines.append(f"{'轮号':<4} {'位置':<22} {'实时角度':<14} {'指向状态':<14} {'目标横向对齐':<18} {'驱动轮速':<10} {'舵电机'}")
        lines.append("-" * 80)

        for i in range(4):
            code, name = wheel_labels[i]
            deg = snap["steering_deg"][i]
            dir_type = classify_direction(deg)

            if dir_type == "横向":
                dir_badge = "\033[1;37;44m 🔵 横向 \033[0m"
            elif dir_type == "竖向":
                dir_badge = "\033[1;37;42m 🟢 竖向 \033[0m"
            else:
                dir_badge = "\033[1;30;43m 🟡 斜向 \033[0m"

            # Check alignment with target (specifically transverse target in Crab mode)
            if snap["mode_id"] in (1, 4):  # Crab or side slip
                if dir_type == "横向":
                    align_badge = "\033[32m[ 🟢 已到位 ]\033[0m"
                else:
                    align_badge = f"\033[31m[ 🔴 偏离 {abs(abs(deg)-90):.1f}° ]\033[0m"
            elif snap["mode_id"] == 0:  # Ackermann
                if dir_type == "竖向":
                    align_badge = "\033[32m[ 🟢 直行对齐 ]\033[0m"
                else:
                    align_badge = f"\033[33m[ 偏角 {deg:+.1f}° ]\033[0m"
            else:
                align_badge = f"[ 当前 {deg:+.1f}° ]"

            spd = snap["wheel_speed"][i]
            spd_str = f"{spd:+.2f} m/s"

            # Motor status
            m_err = snap["motor_error"][i]
            m_en = snap["motor_enabled"][i]
            m_temp = snap["motor_temp"][i]
            if m_err:
                m_str = f"\033[31m故障报警!\033[0m({m_temp}°C)"
            elif not m_en:
                m_str = f"\033[90m未使能\033[0m({m_temp}°C)"
            else:
                m_str = f"\033[32m正常\033[0m({m_temp}°C)"

            deg_str = f"\033[1m{deg:+6.1f}°\033[0m"
            lines.append(f"{code:<4} {name:<20} {deg_str:<21} {dir_badge:<20} {align_badge:<24} {spd_str:<10} {m_str}")

        lines.append("-" * 80)
        lines.append("\033[90m提示: 请按 Ctrl+C 退出监控，退出时将自动在上方生成统计分析报告并保存CSV。\033[0m")

        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()
        time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description="松灵 Ranger Delta 舵轮状态与到位实时诊断记录工具")
    parser.add_argument("--interface", "-i", default="can0", help="SocketCAN 接口名 (默认: can0)")
    parser.add_argument("--log-dir", "-l", default=None, help="CSV日志保存目录 (默认: project_slam/logs/wheel_monitor)")
    args = parser.parse_args()

    monitor = WheelMonitor(interface=args.interface, log_dir=args.log_dir)

    def sig_handler(sig, frame):
        monitor.running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    monitor.start()
    try:
        run_live_tui(monitor)
    finally:
        monitor.close()
        # Clean terminal screen and output diagnostic summary
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()
        monitor.print_diagnostic_summary()


if __name__ == "__main__":
    main()
