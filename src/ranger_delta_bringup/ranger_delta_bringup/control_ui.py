#!/usr/bin/env python3
import csv
import datetime
import math
import os
from pathlib import Path
import sys
import time

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QKeySequence
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QRadioButton,
    QShortcut,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

import rclpy
from agx_bringup.msg import MotionModeFeedback, SteeringAngles, SystemStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def classify_direction(angle_deg: float) -> str:
    a = abs(angle_deg)
    if a <= 15.0 or abs(a - 180.0) <= 15.0:
        return "竖向"
    if abs(a - 90.0) <= 15.0:
        return "横向"
    return "斜向"


class RangerDeltaControlUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("松灵 Ranger Delta 安全运控与舵轮到位监测面板")
        self.resize(760, 840)

        # Publisher & Subscriptions
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.node.create_subscription(SystemStatus, "/system_status", self.on_system_status, 1)
        self.node.create_subscription(MotionModeFeedback, "/motion_mode_feedback", self.on_motion_mode, 1)
        self.node.create_subscription(SteeringAngles, "/steering_angles", self.on_steering_angles, 1)
        self.node.create_subscription(Odometry, "/agx/wheel_odom_raw", self.on_odom_feedback, 1)

        # State tracking
        self.last_status_time = 0.0
        self.last_mode_time = 0.0
        self.current_vol = 0.0
        self.current_ctrl_mode = "未知"
        self.current_motion_mode = "未知"
        self.current_faults = "正常"
        self.fb_linear_speed = 0.0
        self.fb_angular_speed = 0.0
        self.steering_deg = [0.0, 0.0, 0.0, 0.0]

        # Recorder state
        self.is_recording = False
        self.record_file = None
        self.record_writer = None
        self.record_count = 0
        self.record_start_time = 0.0
        self.rec_min_deg = [999.0, 999.0, 999.0, 999.0]
        self.rec_max_deg = [-999.0, -999.0, -999.0, -999.0]
        self.rec_reached_trans = [False, False, False, False]

        # Motion command state
        self.linear_speed = 0.08  # Default slow speed: 8 cm/s
        self.angular_speed = 0.15  # Default slow turn: 0.15 rad/s
        self.pulse_duration = 1.0  # Seconds for pulse jog
        self.pulse_timer = QTimer(self)
        self.pulse_timer.setSingleShot(True)
        self.pulse_timer.timeout.connect(self.stop_robot)

        self.continuous_cmd = None
        self.continuous_timer = QTimer(self)
        self.continuous_timer.setInterval(50)  # 20Hz publishing
        self.continuous_timer.timeout.connect(self.send_continuous_cmd)

        self.setup_ui()
        self.setup_shortcuts()

        # ROS2 spin timer (50Hz)
        self.ros_timer = QTimer(self)
        self.ros_timer.setInterval(20)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start()

    def setup_ui(self):
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # ================= 1. 状态与遥测看板 =================
        status_box = QGroupBox("1. 底盘状态与实时遥测 (Telemetry)")
        status_box.setStyleSheet("QGroupBox { font-weight: bold; font-size: 13px; }")
        status_layout = QGridLayout(status_box)

        self.lbl_can_heartbeat = QLabel("🔴 CAN通信: 等待数据...")
        self.lbl_can_heartbeat.setStyleSheet("color: red; font-weight: bold;")
        self.lbl_voltage = QLabel("电池电压: -- V")
        self.lbl_ctrl_mode = QLabel("控制模式: --")
        self.lbl_motion_mode = QLabel("运动模式: --")
        self.lbl_fault = QLabel("系统健康: --")
        self.lbl_feedback_vel = QLabel("底盘实测速度: vx=0.00 m/s, wz=0.00 rad/s")
        status_layout.addWidget(self.lbl_can_heartbeat, 0, 0)
        status_layout.addWidget(self.lbl_voltage, 0, 1)
        status_layout.addWidget(self.lbl_ctrl_mode, 1, 0)
        status_layout.addWidget(self.lbl_motion_mode, 1, 1)
        status_layout.addWidget(self.lbl_fault, 2, 0)
        status_layout.addWidget(self.lbl_feedback_vel, 2, 1)

        main_layout.addWidget(status_box)

        # ================= 4舵轮朝向看板 =================
        wheel_box = QGroupBox("4舵轮实时转角与指向状态 (竖向 / 横向 / 斜向)")
        wheel_box.setStyleSheet("QGroupBox { font-size: 13px; font-weight: bold; }")
        wheel_layout = QGridLayout(wheel_box)

        self.lbl_w1 = QLabel("W1(右前): 0.0° [ 竖向 ]")
        self.lbl_w2 = QLabel("W2(右后): 0.0° [ 竖向 ]")
        self.lbl_w3 = QLabel("W3(左后): 0.0° [ 竖向 ]")
        self.lbl_w4 = QLabel("W4(左前): 0.0° [ 竖向 ]")
        for lbl in (self.lbl_w1, self.lbl_w2, self.lbl_w3, self.lbl_w4):
            lbl.setFont(QFont("Arial", 11, QFont.Bold))
            lbl.setStyleSheet("padding: 6px; border: 1px solid #ddd; border-radius: 4px; background: #fafafa;")

        wheel_layout.addWidget(self.lbl_w1, 0, 0)
        wheel_layout.addWidget(self.lbl_w2, 0, 1)
        wheel_layout.addWidget(self.lbl_w3, 1, 0)
        wheel_layout.addWidget(self.lbl_w4, 1, 1)
        main_layout.addWidget(wheel_box)

        # ================= 轮子转角到位记录与诊断分析 =================
        rec_box = QGroupBox("舵轮转角到位记录与诊断工具 (Diagnostic Logger)")
        rec_box.setStyleSheet("QGroupBox { font-size: 13px; font-weight: bold; }")
        rec_layout = QVBoxLayout(rec_box)
        rec_btn_layout = QHBoxLayout()

        self.btn_rec_toggle = QPushButton("⏺ 开始录制轮子数据 (Start Record)")
        self.btn_rec_toggle.setStyleSheet("padding: 6px 12px; font-weight: bold; background-color: #5cb85c; color: white; border-radius: 4px;")
        self.btn_rec_toggle.clicked.connect(self.toggle_recording)

        self.lbl_rec_info = QLabel("未录制 | 日志自动保存至: logs/wheel_monitor/")
        self.lbl_rec_info.setStyleSheet("color: #666; font-size: 11px;")

        rec_btn_layout.addWidget(self.btn_rec_toggle)
        rec_btn_layout.addWidget(self.lbl_rec_info)
        rec_layout.addLayout(rec_btn_layout)

        self.lbl_diag_result = QLabel("")
        self.lbl_diag_result.setStyleSheet("color: #333; font-size: 11px; background: #f5f5f5; padding: 6px; border-radius: 4px; border: 1px solid #e0e0e0;")
        self.lbl_diag_result.setVisible(False)
        rec_layout.addWidget(self.lbl_diag_result)
        main_layout.addWidget(rec_box)

        # ================= 2. 速度限制与安全调优 =================
        speed_box = QGroupBox("2. 安全速度调节 (默认极低速平稳档)")
        speed_box.setStyleSheet("QGroupBox { font-weight: bold; font-size: 13px; }")
        speed_layout = QGridLayout(speed_box)

        # Linear speed slider (0.02 to 0.20 m/s, step 0.01)
        self.lbl_linear_val = QLabel("线速度上限: 0.08 m/s (8 cm/s)")
        self.slider_linear = QSlider(Qt.Horizontal)
        self.slider_linear.setRange(2, 20)  # represents 0.02 to 0.20
        self.slider_linear.setValue(8)
        self.slider_linear.valueChanged.connect(self.on_linear_slider_changed)

        # Angular speed slider (0.05 to 0.35 rad/s)
        self.lbl_angular_val = QLabel("角速度上限: 0.15 rad/s (~8.6°/s)")
        self.slider_angular = QSlider(Qt.Horizontal)
        self.slider_angular.setRange(5, 35)  # represents 0.05 to 0.35
        self.slider_angular.setValue(15)
        self.slider_angular.valueChanged.connect(self.on_angular_slider_changed)

        # Control style selection: Pulse jog vs Hold-to-move
        self.radio_pulse = QRadioButton("点动模式 (单次点击运行指定时长后自动平稳停车)")
        self.radio_pulse.setChecked(True)
        self.radio_hold = QRadioButton("长按模式 (鼠标按住或按键保持移动，松开即停车)")
        mode_btn_group = QButtonGroup(self)
        mode_btn_group.addButton(self.radio_pulse)
        mode_btn_group.addButton(self.radio_hold)

        # Pulse duration selector
        pulse_widget = QWidget()
        pulse_layout = QHBoxLayout(pulse_widget)
        pulse_layout.setContentsMargins(0, 0, 0, 0)
        lbl_pulse = QLabel("点动时长:")
        self.rb_pulse_05 = QRadioButton("0.5秒 (微调)")
        self.rb_pulse_10 = QRadioButton("1.0秒")
        self.rb_pulse_15 = QRadioButton("1.5秒 (推荐横移/自旋)")
        self.rb_pulse_20 = QRadioButton("2.0秒")
        self.rb_pulse_15.setChecked(True)

        pulse_time_group = QButtonGroup(self)
        pulse_time_group.addButton(self.rb_pulse_05)
        pulse_time_group.addButton(self.rb_pulse_10)
        pulse_time_group.addButton(self.rb_pulse_15)
        pulse_time_group.addButton(self.rb_pulse_20)
        self.rb_pulse_05.toggled.connect(lambda: self.set_pulse_duration(0.5))
        self.rb_pulse_10.toggled.connect(lambda: self.set_pulse_duration(1.0))
        self.rb_pulse_15.toggled.connect(lambda: self.set_pulse_duration(1.5))
        self.rb_pulse_20.toggled.connect(lambda: self.set_pulse_duration(2.0))

        pulse_layout.addWidget(lbl_pulse)
        pulse_layout.addWidget(self.rb_pulse_05)
        pulse_layout.addWidget(self.rb_pulse_10)
        pulse_layout.addWidget(self.rb_pulse_15)
        pulse_layout.addWidget(self.rb_pulse_20)

        self.lbl_hint = QLabel("💡 提示: 横移需4轮旋转90°，系统已内置零速预对齐保护；建议长按或选1.5s点动以留足转向与移动时间。")
        self.lbl_hint.setStyleSheet("color: #555; font-size: 11px;")

        speed_layout.addWidget(self.lbl_linear_val, 0, 0)
        speed_layout.addWidget(self.slider_linear, 0, 1)
        speed_layout.addWidget(self.lbl_angular_val, 1, 0)
        speed_layout.addWidget(self.slider_angular, 1, 1)
        speed_layout.addWidget(self.radio_pulse, 2, 0, 1, 2)
        speed_layout.addWidget(pulse_widget, 3, 0, 1, 2)
        speed_layout.addWidget(self.radio_hold, 4, 0, 1, 2)
        speed_layout.addWidget(self.lbl_hint, 5, 0, 1, 2)

        main_layout.addWidget(speed_box)

        # ================= 3. 运控执行区 (阿克曼 / 自旋 / 斜移) =================
        ctrl_box = QGroupBox("3. 机器人运动执行区 (支持鼠标点击或快捷键)")
        ctrl_box.setStyleSheet("QGroupBox { font-weight: bold; font-size: 13px; }")
        ctrl_layout = QGridLayout(ctrl_box)

        # Buttons
        self.btn_fwd = self.create_motion_button("⬆ 前进 (W)", 1.0, 0.0, 0.0)
        self.btn_back = self.create_motion_button("⬇ 后退 (S)", -1.0, 0.0, 0.0)
        self.btn_turn_left = self.create_motion_button("↖ 左前转向 (A)", 1.0, 0.0, 1.0)
        self.btn_turn_right = self.create_motion_button("↗ 右前转向 (D)", 1.0, 0.0, -1.0)

        self.btn_spin_left = self.create_motion_button("↺ 原地左自旋 (Q)", 0.0, 0.0, 1.0)
        self.btn_spin_right = self.create_motion_button("↻ 原地右自旋 (E)", 0.0, 0.0, -1.0)

        self.btn_crab_left = self.create_motion_button("⬅ 向左横移 (J)", 0.0, 1.0, 0.0)
        self.btn_crab_right = self.create_motion_button("➡ 向右横移 (L)", 0.0, -1.0, 0.0)

        # Layout
        ctrl_layout.addWidget(self.btn_turn_left, 0, 0)
        ctrl_layout.addWidget(self.btn_fwd, 0, 1)
        ctrl_layout.addWidget(self.btn_turn_right, 0, 2)

        ctrl_layout.addWidget(self.btn_spin_left, 1, 0)
        ctrl_layout.addWidget(self.btn_back, 1, 1)
        ctrl_layout.addWidget(self.btn_spin_right, 1, 2)

        ctrl_layout.addWidget(self.btn_crab_left, 2, 0)
        ctrl_layout.addWidget(self.btn_crab_right, 2, 2)

        main_layout.addWidget(ctrl_box)

        # ================= 4. 急停大按钮 =================
        self.btn_stop = QPushButton("🛑 紧急制动 / 停车 (SPACE)")
        self.btn_stop.setFont(QFont("Arial", 16, QFont.Bold))
        self.btn_stop.setStyleSheet(
            "QPushButton { background-color: #d9534f; color: white; padding: 14px; border-radius: 8px; }"
            "QPushButton:hover { background-color: #c9302c; }"
            "QPushButton:pressed { background-color: #ac2925; }"
        )
        self.btn_stop.clicked.connect(self.stop_robot)
        main_layout.addWidget(self.btn_stop)

        # ================= 5. 指令监视条 =================
        self.lbl_cmd_sent = QLabel("最后下发: [静止停止] vx=0.00, vy=0.00, wz=0.00")
        self.lbl_cmd_sent.setStyleSheet("font-family: monospace; font-size: 12px; color: #555;")
        main_layout.addWidget(self.lbl_cmd_sent)

    def create_motion_button(self, label, dir_x, dir_y, dir_z):
        btn = QPushButton(label)
        btn.setFont(QFont("Arial", 12))
        btn.setStyleSheet("padding: 10px; border-radius: 6px; background-color: #f0f0f0;")

        # Pulse jog mode: clicked event
        btn.clicked.connect(lambda: self.on_button_clicked(dir_x, dir_y, dir_z))
        # Hold to move mode: press / release events
        btn.pressed.connect(lambda: self.on_button_pressed(dir_x, dir_y, dir_z))
        btn.released.connect(self.on_button_released)
        return btn

    def setup_shortcuts(self):
        # Keyboard shortcuts
        QShortcut(QKeySequence(Qt.Key_W), self, activated=lambda: self.trigger_shortcut(1.0, 0.0, 0.0))
        QShortcut(QKeySequence(Qt.Key_S), self, activated=lambda: self.trigger_shortcut(-1.0, 0.0, 0.0))
        QShortcut(QKeySequence(Qt.Key_A), self, activated=lambda: self.trigger_shortcut(1.0, 0.0, 1.0))
        QShortcut(QKeySequence(Qt.Key_D), self, activated=lambda: self.trigger_shortcut(1.0, 0.0, -1.0))
        QShortcut(QKeySequence(Qt.Key_Q), self, activated=lambda: self.trigger_shortcut(0.0, 0.0, 1.0))
        QShortcut(QKeySequence(Qt.Key_E), self, activated=lambda: self.trigger_shortcut(0.0, 0.0, -1.0))
        QShortcut(QKeySequence(Qt.Key_J), self, activated=lambda: self.trigger_shortcut(0.0, 1.0, 0.0))
        QShortcut(QKeySequence(Qt.Key_L), self, activated=lambda: self.trigger_shortcut(0.0, -1.0, 0.0))
        QShortcut(QKeySequence(Qt.Key_Space), self, activated=self.stop_robot)

    def trigger_shortcut(self, dir_x, dir_y, dir_z):
        if self.radio_pulse.isChecked():
            self.on_button_clicked(dir_x, dir_y, dir_z)
        else:
            self.start_motion(dir_x, dir_y, dir_z)

    def on_linear_slider_changed(self, value):
        self.linear_speed = value / 100.0
        self.lbl_linear_val.setText(f"线速度上限: {self.linear_speed:.2f} m/s ({int(self.linear_speed*100)} cm/s)")

    def on_angular_slider_changed(self, value):
        self.angular_speed = value / 100.0
        deg_per_sec = math.degrees(self.angular_speed)
        self.lbl_angular_val.setText(f"角速度上限: {self.angular_speed:.2f} rad/s (~{deg_per_sec:.1f}°/s)")

    def set_pulse_duration(self, duration):
        self.pulse_duration = duration

    def calculate_twist(self, dir_x, dir_y, dir_z):
        cmd = Twist()
        cmd.linear.x = dir_x * self.linear_speed
        cmd.linear.y = dir_y * self.linear_speed
        cmd.angular.z = dir_z * self.angular_speed
        return cmd

    def on_button_clicked(self, dir_x, dir_y, dir_z):
        if not self.radio_pulse.isChecked():
            return
        # Pulse Jog: publish for pulse_duration then stop
        cmd = self.calculate_twist(dir_x, dir_y, dir_z)
        self.continuous_cmd = cmd
        self.continuous_timer.start()
        self.pulse_timer.start(int(self.pulse_duration * 1000))
        self.update_cmd_display(cmd, f"点动中 ({self.pulse_duration}s)")

    def on_button_pressed(self, dir_x, dir_y, dir_z):
        if self.radio_pulse.isChecked():
            return
        self.start_motion(dir_x, dir_y, dir_z)

    def on_button_released(self):
        if self.radio_pulse.isChecked():
            return
        self.stop_robot()

    def start_motion(self, dir_x, dir_y, dir_z):
        cmd = self.calculate_twist(dir_x, dir_y, dir_z)
        self.continuous_cmd = cmd
        self.continuous_timer.start()
        self.update_cmd_display(cmd, "长按持续下发")

    def send_continuous_cmd(self):
        if self.continuous_cmd is not None:
            self.cmd_pub.publish(self.continuous_cmd)

    def stop_robot(self):
        self.pulse_timer.stop()
        self.continuous_timer.stop()
        self.continuous_cmd = None
        stop_cmd = Twist()
        # Publish multiple stop frames to guarantee reception
        for _ in range(3):
            self.cmd_pub.publish(stop_cmd)
        self.update_cmd_display(stop_cmd, "已停止")

    def update_cmd_display(self, cmd, tag):
        self.lbl_cmd_sent.setText(
            f"最后下发 [{tag}]: vx={cmd.linear.x:.2f} m/s, vy={cmd.linear.y:.2f} m/s, wz={cmd.angular.z:.2f} rad/s"
        )

    # ---------------- ROS2 Callbacks & Telemetry ----------------
    def on_system_status(self, msg: SystemStatus):
        self.last_status_time = time.monotonic()
        self.current_vol = msg.vol
        self.current_ctrl_mode = msg.ctrl_mode_text
        has_fault = any(bit.value for bit in msg.faults)
        self.current_faults = "异常报警!" if has_fault else "正常"
        self.update_telemetry_ui()

    def on_motion_mode(self, msg: MotionModeFeedback):
        self.last_mode_time = time.monotonic()
        self.current_motion_mode = f"{msg.mode_text} ({msg.switch_text})"
        self.update_telemetry_ui()

    def on_steering_angles(self, msg: SteeringAngles):
        d1 = math.degrees(msg.steering_01)
        d2 = math.degrees(msg.steering_02)
        d3 = math.degrees(msg.steering_03)
        d4 = math.degrees(msg.steering_04)
        degs = [d1, d2, d3, d4]
        self.steering_deg = degs

        wheel_widgets = [
            (self.lbl_w1, "W1(右前)", d1),
            (self.lbl_w2, "W2(右后)", d2),
            (self.lbl_w3, "W3(左后)", d3),
            (self.lbl_w4, "W4(左前)", d4),
        ]
        for lbl, name, deg in wheel_widgets:
            dir_type = classify_direction(deg)
            if dir_type == "横向":
                badge_style = "color: #0056b3; background: #e7f1ff; border: 1px solid #b8daff;"
            elif dir_type == "竖向":
                badge_style = "color: #28a745; background: #eafaf1; border: 1px solid #c3e6cb;"
            else:
                badge_style = "color: #d97706; background: #fffbe6; border: 1px solid #ffe58f;"
            lbl.setText(f"{name}: {deg:+6.1f}°  [ {dir_type} ]")
            lbl.setStyleSheet(f"padding: 6px; border-radius: 4px; font-weight: bold; {badge_style}")

        # If recording, write to CSV
        if self.is_recording and self.record_writer:
            now_ts = time.time()
            now_iso = datetime.datetime.fromtimestamp(now_ts).isoformat(timespec="milliseconds")
            for i in range(4):
                if degs[i] < self.rec_min_deg[i]:
                    self.rec_min_deg[i] = degs[i]
                if degs[i] > self.rec_max_deg[i]:
                    self.rec_max_deg[i] = degs[i]
                if classify_direction(degs[i]) == "横向":
                    self.rec_reached_trans[i] = True

            cmd_x = self.continuous_cmd.linear.x if self.continuous_cmd else 0.0
            cmd_y = self.continuous_cmd.linear.y if self.continuous_cmd else 0.0
            cmd_z = self.continuous_cmd.angular.z if self.continuous_cmd else 0.0

            self.record_writer.writerow([
                f"{now_ts:.3f}", now_iso, self.current_motion_mode,
                f"{cmd_x:.3f}", f"{cmd_y:.3f}", f"{cmd_z:.3f}",
                f"{d1:.2f}", classify_direction(d1),
                f"{d2:.2f}", classify_direction(d2),
                f"{d3:.2f}", classify_direction(d3),
                f"{d4:.2f}", classify_direction(d4),
                f"{self.current_vol:.1f}",
            ])
            self.record_count += 1
            if self.record_count % 10 == 0:
                self.record_file.flush()
            self.lbl_rec_info.setText(f"🔴 正在录制中... (已记录: {self.record_count} 帧, 耗时: {now_ts - self.record_start_time:.1f}s)")

    def toggle_recording(self):
        if not self.is_recording:
            # Start recording
            log_dir = Path("/home/leona/WorkingSpace/navigation2/project_slam/logs/wheel_monitor")
            log_dir.mkdir(parents=True, exist_ok=True)
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.rec_path = log_dir / f"wheel_gui_record_{ts}.csv"
            self.record_file = open(self.rec_path, "w", newline="", encoding="utf-8")
            self.record_writer = csv.writer(self.record_file)
            self.record_writer.writerow([
                "timestamp", "time_iso", "motion_mode",
                "cmd_vx", "cmd_vy", "cmd_wz",
                "w1_deg", "w1_dir", "w2_deg", "w2_dir",
                "w3_deg", "w3_dir", "w4_deg", "w4_dir", "voltage",
            ])
            self.record_file.flush()

            self.is_recording = True
            self.record_count = 0
            self.record_start_time = time.time()
            self.rec_min_deg = [999.0, 999.0, 999.0, 999.0]
            self.rec_max_deg = [-999.0, -999.0, -999.0, -999.0]
            self.rec_reached_trans = [False, False, False, False]

            self.btn_rec_toggle.setText("⏹ 停止录制并生成诊断 (Stop & Analyze)")
            self.btn_rec_toggle.setStyleSheet("padding: 6px 12px; font-weight: bold; background-color: #d9534f; color: white; border-radius: 4px;")
            self.lbl_diag_result.setVisible(False)
        else:
            # Stop recording
            self.is_recording = False
            if self.record_file:
                self.record_file.flush()
                self.record_file.close()
                self.record_file = None
                self.record_writer = None

            self.btn_rec_toggle.setText("⏺ 开始录制轮子数据 (Start Record)")
            self.btn_rec_toggle.setStyleSheet("padding: 6px 12px; font-weight: bold; background-color: #5cb85c; color: white; border-radius: 4px;")
            self.lbl_rec_info.setText(f"录制已保存至: {self.rec_path.name} (共 {self.record_count} 帧)")

            # Generate diagnostic summary
            w_names = ["W1(右前)", "W2(右后)", "W3(左后)", "W4(左前)"]
            summary_lines = [f"<b>🔍 录制诊断报告 (共 {self.record_count} 帧):</b>"]
            for i in range(4):
                mn = f"{self.rec_min_deg[i]:+.1f}°" if self.rec_min_deg[i] < 900 else "--"
                mx = f"{self.rec_max_deg[i]:+.1f}°" if self.rec_max_deg[i] > -900 else "--"
                cur = f"{self.steering_deg[i]:+.1f}°"
                did = "<font color='green'>✔ 达到横向(±90°)</font>" if self.rec_reached_trans[i] else "<font color='red'>✘ 未达到横向!</font>"
                summary_lines.append(f"• <b>{w_names[i]}</b>: 范围 [{mn} ~ {mx}], 当前 {cur} -> {did}")

            self.lbl_diag_result.setText("<br>".join(summary_lines))
            self.lbl_diag_result.setVisible(True)

    def on_odom_feedback(self, msg: Odometry):
        self.fb_linear_speed = msg.twist.twist.linear.x
        self.fb_angular_speed = msg.twist.twist.angular.z
        self.lbl_feedback_vel.setText(
            f"底盘实测速度: vx={self.fb_linear_speed:.2f} m/s, wz={self.fb_angular_speed:.2f} rad/s"
        )

    def update_telemetry_ui(self):
        now = time.monotonic()
        alive = (now - self.last_status_time < 0.5)
        if alive:
            self.lbl_can_heartbeat.setText("🟢 CAN通信: 正常在线 (50Hz)")
            self.lbl_can_heartbeat.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.lbl_can_heartbeat.setText("🔴 CAN通信: 超时未接收 (离线)")
            self.lbl_can_heartbeat.setStyleSheet("color: red; font-weight: bold;")

        self.lbl_voltage.setText(f"电池电压: {self.current_vol:.1f} V")
        self.lbl_ctrl_mode.setText(f"控制模式: {self.current_ctrl_mode}")
        self.lbl_motion_mode.setText(f"运动模式: {self.current_motion_mode}")
        if self.current_faults == "正常":
            self.lbl_fault.setText("系统健康: 正常")
            self.lbl_fault.setStyleSheet("color: green;")
        else:
            self.lbl_fault.setText("系统健康: 异常报警!")
            self.lbl_fault.setStyleSheet("color: red; font-weight: bold;")

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.update_telemetry_ui()

    def closeEvent(self, event):
        self.stop_robot()
        event.accept()


def main():
    rclpy.init()
    node = rclpy.create_node("ranger_delta_control_ui")
    app = QApplication(sys.argv)
    window = RangerDeltaControlUI(node)
    window.show()
    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
