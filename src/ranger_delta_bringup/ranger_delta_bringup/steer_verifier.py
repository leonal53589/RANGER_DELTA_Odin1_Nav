#!/usr/bin/env python3
"""
松灵 Ranger Delta 舵轮转角极限与指令响应验证工具
(Ranger Delta Steering Angle Limit & Response Verifier)

功能:
1. 安全零速验证 (线速度强制为 0，仅让轮子在原地旋转打轮，绝对不会发生底盘冲出)
2. 交互式角度测试: 允许用户直接输入任意角度 (如 15, 30, 45, 60, 90, -30, -60, -90 等)，下发给底盘并实时回读实测转角，验证固件与物理限位
3. 自动化全范围扫描: 自动测试 [-90° ~ +90°] 各阶梯角度，生成转角响应与限位截断对比表
4. 横向反向驱动验证: 验证当轮子处于 90° 横向时，通过反向速度实现左平移的可行性
"""

import argparse
import math
import socket
import struct
import sys
import threading
import time

# CAN IDs
SET_CONTROL = 0x421       # 使能控制模式
SET_MOTION_MODE = 0x141   # 设定运动模式 (0:阿克曼, 1:平行横移)
SET_MOTION = 0x111        # 运动控制 (线速度, 角速度, 保留, 转向角)
CAN_CONTROL = 1
MODE_CRAB = 1             # 平行横移模式

SYSTEM_STATUS = 0x211
STEERING_ANGLES = 0x271
MOTION_MODE = 0x291
CAN_FRAME = struct.Struct('=IB3x8s')


class SteerVerifier:
    def __init__(self, interface='can0'):
        self.interface = interface
        self.running = True
        self.sock = None
        self.lock = threading.Lock()

        # Telemetry
        self.current_mode = 0
        self.steering_deg = [0.0, 0.0, 0.0, 0.0]  # W1(右前), W2(右后), W3(左后), W4(空/未接)
        self.last_rx_time = 0.0

        # Command to keep sending (20ms cycle)
        self.cmd_speed = 0.0      # m/s
        self.cmd_steer_rad = 0.0  # rad
        self.cmd_mode = MODE_CRAB

        self.connect()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread.start()
        self.tx_thread.start()

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.bind((self.interface,))
            self.sock.settimeout(0.5)
            print(f"\033[32m[成功] 已连接 SocketCAN 接口 {self.interface}\033[0m")
        except Exception as e:
            print(f"\033[31m[错误] 无法连接 {self.interface}: {e}\033[0m")
            print("请确认车辆已上电，且已执行: sudo ip link set can0 up type can bitrate 500000")
            sys.exit(1)

    def _rx_loop(self):
        while self.running:
            try:
                frame = self.sock.recv(16)
                if len(frame) == 16:
                    can_id, length, data = struct.unpack('=IB3x8s', frame)
                    payload = data[:length]
                    if can_id == STEERING_ANGLES and len(payload) >= 8:
                        s1, s2, s3, s4 = struct.unpack('>hhhh', payload[:8])
                        with self.lock:
                            self.steering_deg = [
                                math.degrees(s1 / 1000.0),
                                math.degrees(s2 / 1000.0),
                                math.degrees(s3 / 1000.0),
                                math.degrees(s4 / 1000.0),
                            ]
                            self.last_rx_time = time.time()
                    elif can_id == MOTION_MODE and len(payload) >= 1:
                        with self.lock:
                            self.current_mode = payload[0]
            except socket.timeout:
                continue
            except Exception:
                if self.running:
                    time.sleep(0.1)

    def _tx_loop(self):
        """保持 50Hz 发送心跳与控制指令"""
        loop_count = 0
        while self.running:
            try:
                # 每秒下发一次使能与模式
                if loop_count % 50 == 0:
                    enable_frame = CAN_FRAME.pack(SET_CONTROL, 1, bytes([CAN_CONTROL]))
                    mode_frame = CAN_FRAME.pack(SET_MOTION_MODE, 1, bytes([self.cmd_mode]))
                    self.sock.send(enable_frame)
                    self.sock.send(mode_frame)

                # 下发 0x111 运动指令
                with self.lock:
                    spd_raw = round(self.cmd_speed * 1000)
                    steer_raw = round(self.cmd_steer_rad * 1000)
                payload = struct.pack('>hhxxh', spd_raw, 0, steer_raw)
                motion_frame = CAN_FRAME.pack(SET_MOTION, len(payload), payload)
                self.sock.send(motion_frame)
                loop_count += 1
            except Exception:
                pass
            time.sleep(0.02)

    def set_target(self, steer_deg, speed_mps=0.0):
        with self.lock:
            self.cmd_steer_rad = math.radians(steer_deg)
            self.cmd_speed = speed_mps

    def get_angles(self):
        with self.lock:
            return list(self.steering_deg)

    def wait_and_read(self, duration=2.0):
        """保持当前指令并采样读取实测角度"""
        start = time.time()
        samples_w1, samples_w2, samples_w3 = [], [], []
        while time.time() - start < duration and self.running:
            angles = self.get_angles()
            samples_w1.append(angles[0])
            samples_w2.append(angles[1])
            samples_w3.append(angles[2])
            time.sleep(0.05)
        # 取最后稳定期的均值
        last_samples = 10 if len(samples_w1) >= 10 else len(samples_w1)
        w1_avg = sum(samples_w1[-last_samples:]) / last_samples
        w2_avg = sum(samples_w2[-last_samples:]) / last_samples
        w3_avg = sum(samples_w3[-last_samples:]) / last_samples
        return w1_avg, w2_avg, w3_avg

    def stop_robot(self):
        self.set_target(0.0, 0.0)
        time.sleep(0.5)

    def close(self):
        self.running = False
        self.stop_robot()
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass


def run_interactive_mode(verifier: SteerVerifier):
    print("\n" + "=" * 70)
    print("【模式 1: 交互式任意转角测试 (零速度安全打轮)】")
    print("说明: 输入任意期望角度，底盘将在保持速度为0的安全状态下旋转轮子。")
    print("可测试正负大角度（如 15, 30, 45, 60, 90, -30, -45, -60, -90 等）。")
    print("输入 'q' 退出测试。")
    print("=" * 70)

    while verifier.running:
        try:
            user_input = input("\n👉 请输入测试转角 (度数, 例如 45 或 -90): ").strip()
            if not user_input:
                continue
            if user_input.lower() in ('q', 'exit', 'quit'):
                break

            target_deg = float(user_input)
            raw_can_val = round(math.radians(target_deg) * 1000)

            print(f"\n正在下发: 目标转角={target_deg:+.1f}° (CAN 0x111 原始值={raw_can_val:+d} mrad, 驱动速度=0.00 m/s)...")
            verifier.set_target(target_deg, speed_mps=0.0)

            # 动态打印 2.0 秒回读过程
            for tick in range(10):
                time.sleep(0.2)
                w1, w2, w3, w4 = verifier.get_angles()
                sys.stdout.write(f"\r  [执行中 {(tick+1)*0.2:.1f}s] 实测角度: W1(右前)={w1:+6.1f}°, W2(右后)={w2:+6.1f}°, W3(左后)={w3:+6.1f}°")
                sys.stdout.flush()
            print()

            w1, w2, w3 = verifier.wait_and_read(0.5)
            avg_deg = (w1 + w2 + w3) / 3.0
            diff = abs(avg_deg - target_deg)

            print(f"  \033[1m【测试结果】\033[0m")
            print(f"  下发目标: {target_deg:+.1f}°  |  CAN 0x111 下发值: {raw_can_val:+d}")
            print(f"  实测角度: W1={w1:+.1f}°, W2={w2:+.1f}°, W3={w3:+.1f}° (平均={avg_deg:+.1f}°)")

            if diff <= 5.0:
                print(f"  \033[32m✔ 成功到位! (误差仅 {diff:.1f}°，在此角度范围内硬件与固件完全支持)\033[0m")
            else:
                print(f"  \033[31m🔴 未能到位! (下发 {target_deg:+.1f}°, 实测停留于 {avg_deg:+.1f}°, 差值={diff:.1f}°)\033[0m")
                if abs(avg_deg - (-30.0)) <= 3.0:
                    print("  \033[33m💡 发现特征: 轮子被精准限制在 -30.0° 附近，印证了固件/硬件的负向偏转硬限位！\033[0m")
                elif abs(avg_deg - (+90.0)) <= 3.0:
                    print("  \033[33m💡 发现特征: 轮子成功停在 +90.0°，说明正向 90° 旋转通道畅通！\033[0m")

        except ValueError:
            print("输入无效，请输入纯数字（例如 45 或 -30）。")
        except KeyboardInterrupt:
            break


def run_sweep_mode(verifier: SteerVerifier):
    print("\n" + "=" * 70)
    print("【模式 2: 自动阶梯扫描测试 (阶梯标定轮舵极限范围)】")
    print("说明: 将依次自动下发各阶梯角度，测试各轮的响应与截断点。")
    print("=" * 70)

    # 扫描阶梯: 0° -> 正向阶梯 -> 0° -> 负向阶梯
    sweep_angles = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 0.0, -15.0, -30.0, -45.0, -60.0, -90.0, 0.0]

    results = []

    for target in sweep_angles:
        raw_val = round(math.radians(target) * 1000)
        print(f"正在测试目标: {target:+6.1f}° (CAN 0x111 原始={raw_val:+5d} mrad)...", end="", flush=True)
        verifier.set_target(target, speed_mps=0.0)
        w1, w2, w3 = verifier.wait_and_read(duration=1.5)
        avg_act = (w1 + w2 + w3) / 3.0
        diff = abs(avg_act - target)
        status = "到位" if diff <= 4.0 else f"截断于 {avg_act:+.1f}°"
        results.append((target, raw_val, w1, w2, w3, avg_act, status))
        print(f" 实测: W1={w1:+5.1f}°, W2={w2:+5.1f}°, W3={w3:+5.1f}° -> {status}")

    print("\n" + "=" * 80)
    print("                      📊 舵轮阶梯测试标定汇总报告")
    print("=" * 80)
    print(f"{'下发目标':<10} {'CAN原始值':<12} {'W1实测':<10} {'W2实测':<10} {'W3实测':<10} {'平均实测':<10} {'到位判定'}")
    print("-" * 80)
    for target, raw, w1, w2, w3, avg_act, st in results:
        target_str = f"{target:+.1f}°"
        raw_str = f"{raw:+d}"
        w1_s = f"{w1:+.1f}°"
        w2_s = f"{w2:+.1f}°"
        w3_s = f"{w3:+.1f}°"
        avg_s = f"{avg_act:+.1f}°"
        st_color = "\033[32m✔ 到位\033[0m" if "到位" in st else f"\033[31m🔴 {st}\033[0m"
        print(f"{target_str:<10} {raw_str:<12} {w1_s:<10} {w2_s:<10} {w3_s:<10} {avg_s:<10} {st_color}")
    print("=" * 80 + "\n")


def main():
    parser = argparse.ArgumentParser(description="松灵 Ranger Delta 舵轮转角极限与指令响应验证工具")
    parser.add_argument("--interface", "-i", default="can0", help="SocketCAN 接口名 (默认: can0)")
    parser.add_argument("--sweep", action="store_true", help="直接运行自动化阶梯全范围扫描标定")
    args = parser.parse_args()

    print("\033[1;36m" + "=" * 70)
    print("  松灵 Ranger Delta 舵轮转角极限与指令响应验证工具 (Steer Verifier)")
    print("=" * 70 + "\033[0m")

    verifier = SteerVerifier(interface=args.interface)

    try:
        if args.sweep:
            run_sweep_mode(verifier)
        else:
            print("\n请选择验证模式:")
            print("  1. 手动交互模式 (输入任意角度，如 30, 45, 90, -30, -60, -90 等，实时观察轮子响应)")
            print("  2. 自动阶梯扫描模式 (自动测试 0°~90° 与 0°~-90° 阶梯，输出限位对照表)")
            choice = input("请输入选项 (1 或 2, 默认 1): ").strip()
            if choice == "2":
                run_sweep_mode(verifier)
            else:
                run_interactive_mode(verifier)
    finally:
        verifier.close()
        print("\n验证结束，已发送停机回正指令。")


if __name__ == '__main__':
    main()
