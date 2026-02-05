import sys
import threading
import time
from collections import deque

from PyQt5 import QtCore, QtGui, QtWidgets
import serial
import serial.tools.list_ports

import pyqtgraph as pg
from pyqtgraph import PlotWidget, PlotItem


# ---------------------------
# PyQtGraph Canvas Widget
# ---------------------------
class PyQtGraphCanvas(pg.GraphicsLayoutWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Set background
        self.setBackground('w')
        
        # Create subplots
        self.pos_plot = self.addPlot(title="Position (mm)", row=0, col=0)
        self.nextRow()
        self.vel_plot = self.addPlot(title="Velocity (mm/s)", row=1, col=0)
        self.nextRow()
        self.acc_plot = self.addPlot(title="Acceleration (mm/s²)", row=2, col=0)
        
        # Link X axes
        self.vel_plot.setXLink(self.pos_plot)
        self.acc_plot.setXLink(self.pos_plot)
        
        # Configure plots
        for plot in [self.pos_plot, self.vel_plot, self.acc_plot]:
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.addLegend()
        
        # Data curves
        self.curves = {}
        self._setup_curves()
    
    def _setup_curves(self):
        # Position curves
        self.curves['dpos'] = self.pos_plot.plot(pen=pg.mkPen('r', width=2), name="Quỹ đạo đặt")
        self.curves['cpos'] = self.pos_plot.plot(pen=pg.mkPen('b', width=2), name="Quỹ đạo thực tế")
        
        # Velocity curves  
        self.curves['vdes'] = self.vel_plot.plot(pen=pg.mkPen('g', width=2), name="Vận tốc theo quỹ đạo")
        self.curves['vactual'] = self.vel_plot.plot(pen=pg.mkPen('m', width=2), name="Vận tốc thực tế")
        self.curves['vmotor'] = self.vel_plot.plot(pen=pg.mkPen('c', width=2), name="Vận tốc động cơ")
        
        # Acceleration curves
        self.curves['ades'] = self.acc_plot.plot(pen=pg.mkPen('y', width=2), name="Gia tốc theo quỹ đạo")
        self.curves['amotor'] = self.acc_plot.plot(pen=pg.mkPen('k', width=2, style=QtCore.Qt.DotLine), name="Gia tốc động cơ (thô)")
        self.curves['amotor_filtered'] = self.acc_plot.plot(pen=pg.mkPen('r', width=2), name="Gia tốc động cơ (lọc)")
        
        # Set X-axis label to time
        self.pos_plot.setLabel('bottom', 'Time', units='s')
        self.vel_plot.setLabel('bottom', 'Time', units='s')
        self.acc_plot.setLabel('bottom', 'Time', units='s')
    
    def update_plot(self, x, pos_data, vel_data, acc_data):
        # Update all curves with new data
        self.curves['dpos'].setData(x, pos_data[0])
        self.curves['cpos'].setData(x, pos_data[1])
        
        self.curves['vdes'].setData(x, vel_data[0])
        self.curves['vactual'].setData(x, vel_data[1])
        self.curves['vmotor'].setData(x, vel_data[2])
        
        self.curves['ades'].setData(x, acc_data[0])
        self.curves['amotor'].setData(x, acc_data[1])
        self.curves['amotor_filtered'].setData(x, acc_data[2])
    
    def clear_plot(self):
        # Clear all curve data
        for curve in self.curves.values():
            curve.clear()
        
        # Reset plot ranges
        for plot in [self.pos_plot, self.vel_plot, self.acc_plot]:
            plot.enableAutoRange()


# ---------------------------
# Serial Reader Thread
# ---------------------------
class SerialReader(QtCore.QObject):
    line_received = QtCore.pyqtSignal(str)
    connected = QtCore.pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.ser = None
        self._running = False
        self._thread = None

    def list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def open(self, port, baud):
        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.01)
            self._running = True
            self._thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._thread.start()
            self.connected.emit(True)
            return True
        except Exception as e:
            self.connected.emit(False)
            QtWidgets.QMessageBox.critical(None, "Serial Error", str(e))
            return False

    def _reader_loop(self):
        buf = bytearray()
        while self._running and self.ser is not None:
            try:
                data = self.ser.read(512)
                if data:
                    buf.extend(data)
                    while b'\n' in buf:
                        line, _, buf = buf.partition(b'\n')
                        try:
                            s = line.decode('utf-8', errors='ignore').strip()
                            if s:
                                self.line_received.emit(s)
                        except Exception:
                            pass
                else:
                    time.sleep(0.001)
            except Exception:
                break
        self.connected.emit(False)

    def write_text(self, text: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(text.encode('utf-8'))
            except Exception as e:
                QtWidgets.QMessageBox.warning(None, "Write Error", str(e))

    def close(self):
        self._running = False
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.connected.emit(False)


# ---------------------------
# Main Window
# ---------------------------
class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Control - Position/Velocity & Plot (PyQtGraph)")
        self.setMinimumSize(1200, 800)

        # Serial
        self.serial = SerialReader()
        self.serial.line_received.connect(self.on_line_received)
        self.serial.connected.connect(self.on_serial_connected)

        # Buffers
        self.max_points = 50000
        self.dt = 0.005  # Default time step in seconds (5ms)
        self.x_idx = 0
        self.x_data = deque(maxlen=self.max_points)
        self.time_data = deque(maxlen=self.max_points)

        # Position (q_desired, q_actual)
        self.q_desired_data = deque(maxlen=self.max_points)
        self.q_actual_data = deque(maxlen=self.max_points)
        
        # Velocity (v_desired, v_actual, v_motor)
        self.v_desired_data = deque(maxlen=self.max_points)
        self.v_actual_data = deque(maxlen=self.max_points)
        self.v_motor_data = deque(maxlen=self.max_points)
        
        # Acceleration (a_desired, a_motor, a_motor_filtered)
        self.a_desired_data = deque(maxlen=self.max_points)
        self.a_motor_data = deque(maxlen=self.max_points)
        self.a_motor_filtered_data = deque(maxlen=self.max_points)
        
        # Additional data (PWM, Time from MCU)
        self.pwm_data = deque(maxlen=self.max_points)
        self.mcu_time_data = deque(maxlen=self.max_points)
        
        # Filter settings
        self.filter_window = 10  # Moving average window size

        self.rx_counter = 0
        self.last_rx_counter = 0

        # UI
        self._build_ui()

        # Timer to refresh plot (faster - 30ms for better real-time)
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.setInterval(30)
        self.plot_timer.timeout.connect(self.refresh_plot)
        self.plot_timer.start()

        # Timer to update RX rate
        self.rx_timer = QtCore.QTimer(self)
        self.rx_timer.setInterval(500)
        self.rx_timer.timeout.connect(self.update_rx_rate)
        self.rx_timer.start()

        # Populate ports
        self.refresh_ports()

    def _build_ui(self):
        main_layout = QtWidgets.QHBoxLayout(self)
        
        # ===== LEFT PANEL: CONTROLS =====
        left_panel = QtWidgets.QWidget()
        left_panel.setMaximumWidth(420)
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        
        # --- Serial Connection Group ---
        serial_group = QtWidgets.QGroupBox("ĐIỀU KHIỂN")
        serial_layout = QtWidgets.QVBoxLayout()
        
        # Port selection
        port_layout = QtWidgets.QHBoxLayout()
        port_layout.addWidget(QtWidgets.QLabel("COM Port:"))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(160)
        port_layout.addWidget(self.port_combo, 1)
        self.port_refresh_btn = QtWidgets.QPushButton("Làm mới")
        self.port_refresh_btn.clicked.connect(self.refresh_ports)
        port_layout.addWidget(self.port_refresh_btn)
        serial_layout.addLayout(port_layout)
        
        # Baudrate selection
        baud_layout = QtWidgets.QHBoxLayout()
        baud_layout.addWidget(QtWidgets.QLabel("Baudrate:"))
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"])
        self.baud_combo.setCurrentText("115200")
        baud_layout.addWidget(self.baud_combo, 1)
        serial_layout.addLayout(baud_layout)
        
        # Connect/Disconnect buttons
        conn_layout = QtWidgets.QHBoxLayout()
        self.connect_btn = QtWidgets.QPushButton("Kết nối")
        self.connect_btn.clicked.connect(self.on_connect_clicked)
        self.disconnect_btn = QtWidgets.QPushButton("Ngắt kết nối")
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.clicked.connect(self.on_disconnect_clicked)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.disconnect_btn)
        serial_layout.addLayout(conn_layout)
        
        serial_group.setLayout(serial_layout)
        left_layout.addWidget(serial_group)
        
        # --- Data Configuration Group ---
        config_group = QtWidgets.QGroupBox("CẤU HÌNH DỮ LIỆU")
        config_layout = QtWidgets.QVBoxLayout()
        
        # Max points input
        points_layout = QtWidgets.QHBoxLayout()
        points_layout.addWidget(QtWidgets.QLabel("Số điểm tối đa:"))
        self.max_points_edit = QtWidgets.QLineEdit()
        self.max_points_edit.setText("50000")
        self.max_points_edit.setValidator(QtGui.QIntValidator(100, 100000))
        points_layout.addWidget(self.max_points_edit, 1)
        self.apply_points_btn = QtWidgets.QPushButton("Áp dụng")
        self.apply_points_btn.clicked.connect(self.on_apply_max_points)
        points_layout.addWidget(self.apply_points_btn)
        config_layout.addLayout(points_layout)
        
        # Time step (dt) input
        dt_layout = QtWidgets.QHBoxLayout()
        dt_layout.addWidget(QtWidgets.QLabel("Bước thời gian dt (s):"))
        self.dt_edit = QtWidgets.QLineEdit()
        self.dt_edit.setText("0.005")
        self.dt_edit.setValidator(QtGui.QDoubleValidator(0.0001, 1.0, 6))
        dt_layout.addWidget(self.dt_edit, 1)
        self.apply_dt_btn = QtWidgets.QPushButton("Áp dụng")
        self.apply_dt_btn.clicked.connect(self.on_apply_dt)
        dt_layout.addWidget(self.apply_dt_btn)
        config_layout.addLayout(dt_layout)
        
        # Filter window input
        filter_layout = QtWidgets.QHBoxLayout()
        filter_layout.addWidget(QtWidgets.QLabel("Bộ lọc (window):"))
        self.filter_edit = QtWidgets.QLineEdit()
        self.filter_edit.setText("10")
        self.filter_edit.setValidator(QtGui.QIntValidator(1, 100))
        filter_layout.addWidget(self.filter_edit, 1)
        self.apply_filter_btn = QtWidgets.QPushButton("Áp dụng")
        self.apply_filter_btn.clicked.connect(self.on_apply_filter)
        filter_layout.addWidget(self.apply_filter_btn)
        config_layout.addLayout(filter_layout)
        
        config_group.setLayout(config_layout)
        left_layout.addWidget(config_group)
        # --- Mode Selection Group ---
        mode_group = QtWidgets.QGroupBox("CHỌN CHẾ ĐỘ ĐỘNG CƠ")
        mode_layout = QtWidgets.QVBoxLayout()

        mode_btn_layout = QtWidgets.QHBoxLayout()
        self.mode0_btn = QtWidgets.QPushButton("P")
        self.mode0_btn.clicked.connect(lambda: self.send_command("mode 0"))
        mode_btn_layout.addWidget(self.mode0_btn)

        self.mode1_btn = QtWidgets.QPushButton("P-PI")
        self.mode1_btn.clicked.connect(lambda: self.send_command("mode 1"))
        mode_btn_layout.addWidget(self.mode1_btn)
        mode_layout.addLayout(mode_btn_layout)

        self.mode2_btn = QtWidgets.QPushButton("P-PI v1")
        self.mode2_btn.clicked.connect(lambda: self.send_command("mode 2"))
        mode_btn_layout.addWidget(self.mode2_btn)
        mode_layout.addLayout(mode_btn_layout)

        self.mode3_btn = QtWidgets.QPushButton("Manual (SetPos)")
        self.mode3_btn.clicked.connect(lambda: self.send_command("mode 3"))
        mode_btn_layout.addWidget(self.mode3_btn)
        mode_layout.addLayout(mode_btn_layout)

        reset_btn_layout = QtWidgets.QHBoxLayout()
        self.reset_btn = QtWidgets.QPushButton("RESET")
        self.reset_btn.clicked.connect(lambda: self.send_command("reset"))
        reset_btn_layout.addWidget(self.reset_btn)
        mode_layout.addLayout(reset_btn_layout)

        # Direction control for SetPos mode
        dir_label = QtWidgets.QLabel("Chiều (SetPos mode):")
        mode_layout.addWidget(dir_label)

        dir_btn_layout = QtWidgets.QHBoxLayout()
        self.dir_neg_btn = QtWidgets.QPushButton("Lùi")
        self.dir_neg_btn.clicked.connect(lambda: self.send_command("c-1"))
        dir_btn_layout.addWidget(self.dir_neg_btn)

        self.dir_zero_btn = QtWidgets.QPushButton("Dừng")
        self.dir_zero_btn.clicked.connect(lambda: self.send_command("c0"))
        dir_btn_layout.addWidget(self.dir_zero_btn)

        self.dir_pos_btn = QtWidgets.QPushButton("Tiến")
        self.dir_pos_btn.clicked.connect(lambda: self.send_command("c1"))
        dir_btn_layout.addWidget(self.dir_pos_btn)
        mode_layout.addLayout(dir_btn_layout)

        mode_group.setLayout(mode_layout)
        left_layout.addWidget(mode_group)
        # --- PID & Control Group ---
        pid_group = QtWidgets.QGroupBox("ĐIỀU CHỈNH PID")
        pid_layout = QtWidgets.QVBoxLayout()

        # Kp input
        kp_layout = QtWidgets.QHBoxLayout()
        kp_layout.addWidget(QtWidgets.QLabel("Kp:"))
        self.kp_edit = QtWidgets.QLineEdit()
        self.kp_edit.setText("1.0")
        self.kp_edit.setValidator(QtGui.QDoubleValidator(0.0, 1000.0, 6))
        kp_layout.addWidget(self.kp_edit, 1)
        self.send_kp_btn = QtWidgets.QPushButton("Gửi Kp")
        self.send_kp_btn.clicked.connect(self.on_send_kp_clicked)
        kp_layout.addWidget(self.send_kp_btn)
        pid_layout.addLayout(kp_layout)

        # Enable/Disable PID
        enable_layout = QtWidgets.QHBoxLayout()
        self.enable_pid_btn = QtWidgets.QPushButton("Enable Improved PID")
        self.enable_pid_btn.clicked.connect(lambda: self.send_command("1e"))
        self.enable_pid_btn.setStyleSheet("background-color: #90EE90;")
        enable_layout.addWidget(self.enable_pid_btn)

        self.disable_pid_btn = QtWidgets.QPushButton("Disable Improved PID")
        self.disable_pid_btn.clicked.connect(lambda: self.send_command("0e"))
        self.disable_pid_btn.setStyleSheet("background-color: #FFB6C6;")
        enable_layout.addWidget(self.disable_pid_btn)
        pid_layout.addLayout(enable_layout)

        pid_group.setLayout(pid_layout)
        left_layout.addWidget(pid_group)
        # --- Position/Velocity Control Group ---
        control_group = QtWidgets.QGroupBox("ĐIỀU KHIỂN VỊ TRÍ / VẬN TỐC")
        control_layout = QtWidgets.QVBoxLayout()
        
        # Value input
        control_layout.addWidget(QtWidgets.QLabel("Giá trị (mm hoặc mm/s):"))
        self.value_edit = QtWidgets.QLineEdit()
        self.value_edit.setText("100.0")
        self.value_edit.setValidator(QtGui.QDoubleValidator(bottom=-1e9, top=1e9, decimals=6))
        font = self.value_edit.font()
        font.setPointSize(12)
        self.value_edit.setFont(font)
        control_layout.addWidget(self.value_edit)

        # Auto Cycle Count input
        cycle_layout = QtWidgets.QHBoxLayout()
        cycle_layout.addWidget(QtWidgets.QLabel("Số lần chạy auto:"))
        self.cycle_edit = QtWidgets.QLineEdit()
        self.cycle_edit.setText("5")
        self.cycle_edit.setValidator(QtGui.QIntValidator(1, 1000))
        font_cycle = self.cycle_edit.font()
        font_cycle.setPointSize(10)
        self.cycle_edit.setFont(font_cycle)
        cycle_layout.addWidget(self.cycle_edit, 1)
        control_layout.addLayout(cycle_layout)

        # Auto Run button
        self.send_auto_btn = QtWidgets.QPushButton("CHẠY TỰ ĐỘNG")
        self.send_auto_btn.clicked.connect(self.on_send_auto_clicked)
        self.send_auto_btn.setMinimumHeight(40)
        # self.send_auto_btn.setStyleSheet("background-color: #87CEEB; font-weight: bold;")
        control_layout.addWidget(self.send_auto_btn)
        
        # Send buttons
        send_layout = QtWidgets.QHBoxLayout()
        self.send_pos_btn = QtWidgets.QPushButton("GỬI VỊ TRÍ")
        self.send_pos_btn.clicked.connect(self.on_send_position_clicked)
        self.send_pos_btn.setMinimumHeight(35)
        send_layout.addWidget(self.send_pos_btn)
        
        self.send_vel_btn = QtWidgets.QPushButton("GỬI VẬN TỐC")
        self.send_vel_btn.clicked.connect(self.on_send_velocity_clicked)
        self.send_vel_btn.setMinimumHeight(35)
        send_layout.addWidget(self.send_vel_btn)
        control_layout.addLayout(send_layout)
        
        # Quick position buttons
        quick_label = QtWidgets.QLabel("Vị trí nhanh:")
        control_layout.addWidget(quick_label)
        
        quick_layout = QtWidgets.QGridLayout()
        quick_positions = [0, 100, 200, 300, 400]
        for i, pos in enumerate(quick_positions):
            btn = QtWidgets.QPushButton(f"{pos} mm")
            btn.clicked.connect(lambda checked, p=pos: self.quick_send_position(p))
            quick_layout.addWidget(btn, i // 3, i % 3)
        control_layout.addLayout(quick_layout)
        
        # Quick velocity buttons
        quick_vel_label = QtWidgets.QLabel("Vận tốc nhanh:")
        control_layout.addWidget(quick_vel_label)
        
        quick_vel_layout = QtWidgets.QGridLayout()
        quick_velocities = [0, 20, 30, 40, 45]
        for i, vel in enumerate(quick_velocities):
            btn = QtWidgets.QPushButton(f"{vel} mm/s")
            btn.clicked.connect(lambda checked, v=vel: self.quick_send_velocity(v))
            quick_vel_layout.addWidget(btn, i // 3, i % 3)
        control_layout.addLayout(quick_vel_layout)
        
        control_group.setLayout(control_layout)
        left_layout.addWidget(control_group)
        
        # --- Clear / Export Buttons ---
        ce_layout = QtWidgets.QHBoxLayout()
        self.clear_btn = QtWidgets.QPushButton("CLEAR ALL")
        self.clear_btn.clicked.connect(self.on_clear_clicked)
        self.clear_btn.setMinimumHeight(35)
        self.clearlog_btn = QtWidgets.QPushButton("Clear Log")
        self.clearlog_btn.clicked.connect(self.on_clear_log_clicked)
        self.clearlog_btn.setMinimumHeight(35)
        self.export_btn = QtWidgets.QPushButton("Export TXT")
        self.export_btn.clicked.connect(self.on_export_clicked)
        self.export_btn.setMinimumHeight(35)
        
        ce_layout.addWidget(self.clear_btn)
        ce_layout.addWidget(self.clearlog_btn)
        ce_layout.addWidget(self.export_btn)
        left_layout.addLayout(ce_layout)
        
        # --- Status Group ---
        status_group = QtWidgets.QGroupBox("TRẠNG THÁI")
        status_layout = QtWidgets.QVBoxLayout()
        
        self.status_label = QtWidgets.QLabel("Chưa kết nối")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        status_layout.addWidget(self.status_label)
        
        self.rx_label = QtWidgets.QLabel("RX: 0/s")
        status_layout.addWidget(self.rx_label)
        
        self.data_count_label = QtWidgets.QLabel("Số điểm dữ liệu: 0")
        status_layout.addWidget(self.data_count_label)
        
        status_group.setLayout(status_layout)
        left_layout.addWidget(status_group)
        
        # --- Log Text Box ---
        log_label = QtWidgets.QLabel("Log:")
        left_layout.addWidget(log_label)
        
        self.log_edit = QtWidgets.QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setMaximumBlockCount(2000)
        self.log_edit.setPlaceholderText("UART log sẽ hiện ở đây...")
        left_layout.addWidget(self.log_edit, 1)
        
        # ===== RIGHT PANEL: PLOT =====
        right_panel = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_panel)
        
        plot_label = QtWidgets.QLabel("BIỂU ĐỒ DỮ LIỆU - PYQTGRAPH (REAL-TIME)")
        plot_label.setStyleSheet("font-weight: bold; font-size: 12pt; color: blue;")
        plot_label.setAlignment(QtCore.Qt.AlignCenter)
        right_layout.addWidget(plot_label)
        
        # Use PyQtGraph instead of Matplotlib
        self.canvas = PyQtGraphCanvas(self)
        right_layout.addWidget(self.canvas, 1)
        
        # Add panels to main layout
        main_layout.addWidget(left_panel)
        main_layout.addWidget(right_panel, 1)

    # --- Serial handlers ---
    def refresh_ports(self):
        self.port_combo.clear()
        ports = self.serial.list_ports()
        self.port_combo.addItems(ports)

    def on_connect_clicked(self):
        port = self.port_combo.currentText().strip()
        if not port:
            QtWidgets.QMessageBox.information(self, "Thông báo", "Không có cổng COM nào được chọn.")
            return
        try:
            baud = int(self.baud_combo.currentText())
        except ValueError:
            baud = 115200
        ok = self.serial.open(port, baud)
        if ok:
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.status_label.setText(f"Đã kết nối: {port} - {baud} baud")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")

    def on_disconnect_clicked(self):
        self.serial.close()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.status_label.setText("Đã ngắt kết nối")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")

    def on_serial_connected(self, ok: bool):
        if not ok:
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.status_label.setText("Đã ngắt kết nối")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")

    def on_line_received(self, line: str):
        self.rx_counter += 1
        self.log_edit.appendPlainText(line)

        parts = line.strip().split()
        vals = []
        for p in parts:
            try:
                vals.append(float(p))
            except ValueError:
                pass

        # Expected 9 values: q_desired, q_actual, v_desired, v_actual, v_motor, a_desired, a_motor, pwm, time
        if len(vals) == 9:
            self.x_idx += 1
            current_time = self.x_idx * self.dt
            self.x_data.append(self.x_idx)
            self.time_data.append(current_time)
            
            # Store all data
            self.q_desired_data.append(vals[0])
            self.q_actual_data.append(vals[1])
            self.v_desired_data.append(vals[2])
            self.v_actual_data.append(vals[3])
            self.v_motor_data.append(vals[4])
            self.a_desired_data.append(vals[5])
            self.a_motor_data.append(vals[6])
            self.pwm_data.append(vals[7])
            self.mcu_time_data.append(vals[8])
            
            # Apply moving average filter to acceleration
            filtered_acc = self._apply_moving_average_filter(self.a_motor_data)
            self.a_motor_filtered_data.append(filtered_acc)
            
            self.data_count_label.setText(f"Số điểm dữ liệu: {len(self.x_data)}")

    # --- UI actions ---
    def on_send_position_clicked(self):
        if not (self.serial.ser and self.serial.ser.is_open):
            QtWidgets.QMessageBox.information(self, "Thông báo", "Chưa kết nối COM.")
            return
        text = self.value_edit.text().strip()
        if not text:
            QtWidgets.QMessageBox.information(self, "Thông báo", "Nhập giá trị trước khi gửi.")
            return
        payload = f"{text}s"
        self.serial.write_text(payload)
        print(f"Đã gửi vị trí: {text} (mm)")
        self.log_edit.appendPlainText(f"[TX] Position: {text}s")

    def on_send_velocity_clicked(self):
        if not (self.serial.ser and self.serial.ser.is_open):
            QtWidgets.QMessageBox.information(self, "Thông báo", "Chưa kết nối COM.")
            return
        text = self.value_edit.text().strip()
        if not text:
            QtWidgets.QMessageBox.information(self, "Thông báo", "Nhập giá trị trước khi gửi.")
            return
        payload = f"{text}v"
        self.serial.write_text(payload)
        print(f"Đã gửi vận tốc: {text} (mm/s)")
    def send_command(self, cmd: str):
        """Generic function to send any command"""
        if not (self.serial.ser and self.serial.ser.is_open):
            QtWidgets.QMessageBox.information(self, "Thông báo", "Chưa kết nối COM.")
            return
        self.serial.write_text(cmd)
        print(f"Đã gửi lệnh: {cmd}")

    def on_send_kp_clicked(self):
        """Send Kp parameter"""
        if not (self.serial.ser and self.serial.ser.is_open):
            QtWidgets.QMessageBox.information(self, "Thông báo", "Chưa kết nối COM.")
            return
        text = self.kp_edit.text().strip()
        if not text:
            QtWidgets.QMessageBox.information(self, "Thông báo", "Nhập giá trị Kp trước khi gửi.")
            return
        payload = f"{text}p"
        self.serial.write_text(payload)
        print(f"Đã gửi Kp: {text}")

    def on_send_auto_clicked(self):
        """Send auto run command with position and cycle count"""
        if not (self.serial.ser and self.serial.ser.is_open):
            QtWidgets.QMessageBox.information(self, "Thông báo", "Chưa kết nối COM.")
            return
        
        position = self.value_edit.text().strip()
        cycles = self.cycle_edit.text().strip()
        
        if not position or not cycles:
            QtWidgets.QMessageBox.information(self, "Thông báo", "Nhập đầy đủ vị trí và số lần chạy.")
            return
        
        # Format: x{position}x{cycles}
        payload = f"x{position}x{cycles}"
        self.serial.write_text(payload)
        print(f"Đã gửi chạy tự động: {payload}")
        self.log_edit.appendPlainText(f"[TX] Auto Run: {payload} (Pos={position}mm, Cycles={cycles})")

    def quick_send_position(self, value_mm: float):
        self.value_edit.setText(str(value_mm))
        self.on_send_position_clicked()

    def quick_send_velocity(self, value_mms: float):
        self.value_edit.setText(str(value_mms))
        self.on_send_velocity_clicked()

    def on_clear_clicked(self):
        self.x_data.clear()
        self.time_data.clear()
        self.q_desired_data.clear()
        self.q_actual_data.clear()
        self.v_desired_data.clear()
        self.v_actual_data.clear()
        self.v_motor_data.clear()
        self.a_desired_data.clear()
        self.a_motor_data.clear()
        self.a_motor_filtered_data.clear()
        self.pwm_data.clear()
        self.mcu_time_data.clear()
        self.x_idx = 0

        self.canvas.clear_plot()
        self.data_count_label.setText("Số điểm dữ liệu: 0")

    def on_clear_log_clicked(self):
        self.log_edit.clear()

    def on_export_clicked(self):
        """Xuất toàn bộ dữ liệu ra TXT (có header)."""
        if len(self.x_data) == 0:
            QtWidgets.QMessageBox.information(self, "Thông báo", "Không có dữ liệu để xuất.")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Lưu dữ liệu TXT", "trajectory_log.txt", "Text Files (*.txt);;All Files (*)"
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write("# idx\tTime_PC(s)\tq_desired\tq_actual\tv_desired\tv_actual\tv_motor\ta_desired\ta_motor\ta_motor_filtered\tPWM\tTime_MCU\n")
                for i, (t, qd, qa, vd, va, vm, ad, am, amf, pwm, tmcu) in enumerate(
                    zip(self.time_data, self.q_desired_data, self.q_actual_data, 
                        self.v_desired_data, self.v_actual_data, self.v_motor_data,
                        self.a_desired_data, self.a_motor_data, self.a_motor_filtered_data,
                        self.pwm_data, self.mcu_time_data), start=1
                ):
                    f.write(f"{i}\t{t:.6f}\t{qd:.6f}\t{qa:.6f}\t{vd:.6f}\t{va:.6f}\t{vm:.6f}\t{ad:.6f}\t{am:.6f}\t{amf:.6f}\t{pwm:.3f}\t{tmcu:.6f}\n")
            QtWidgets.QMessageBox.information(self, "Thành công", f"Đã lưu: {path}\n(Filter window: {self.filter_window})")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Lỗi", f"Không thể lưu file:\n{e}")

    def refresh_plot(self):
        """Refresh plot với PyQtGraph - nhanh hơn nhiều so với Matplotlib"""
        if len(self.time_data) == 0:
            return

        # Use time data instead of index
        x = list(self.time_data)
        pos_data = (list(self.q_desired_data), list(self.q_actual_data))
        vel_data = (list(self.v_desired_data), list(self.v_actual_data), list(self.v_motor_data))
        acc_data = (list(self.a_desired_data), list(self.a_motor_data), list(self.a_motor_filtered_data))

        # Update plot - PyQtGraph is much faster than Matplotlib
        self.canvas.update_plot(x, pos_data, vel_data, acc_data)

    def on_apply_max_points(self):
        """Apply new maximum points setting"""
        try:
            new_max = int(self.max_points_edit.text())
            if new_max < 100:
                QtWidgets.QMessageBox.warning(self, "Cảnh báo", "Số điểm tối thiểu là 100")
                return
            if new_max > 100000:
                QtWidgets.QMessageBox.warning(self, "Cảnh báo", "Số điểm tối đa là 100000")
                return
            
            # Update max_points and recreate deques
            self.max_points = new_max
            
            # Recreate deques with new maxlen while preserving data
            old_x = list(self.x_data)
            old_time = list(self.time_data)
            old_qd = list(self.q_desired_data)
            old_qa = list(self.q_actual_data)
            old_vd = list(self.v_desired_data)
            old_va = list(self.v_actual_data)
            old_vm = list(self.v_motor_data)
            old_ad = list(self.a_desired_data)
            old_am = list(self.a_motor_data)
            old_amf = list(self.a_motor_filtered_data)
            old_pwm = list(self.pwm_data)
            old_tmcu = list(self.mcu_time_data)
            
            self.x_data = deque(old_x[-new_max:], maxlen=new_max)
            self.time_data = deque(old_time[-new_max:], maxlen=new_max)
            self.q_desired_data = deque(old_qd[-new_max:], maxlen=new_max)
            self.q_actual_data = deque(old_qa[-new_max:], maxlen=new_max)
            self.v_desired_data = deque(old_vd[-new_max:], maxlen=new_max)
            self.v_actual_data = deque(old_va[-new_max:], maxlen=new_max)
            self.v_motor_data = deque(old_vm[-new_max:], maxlen=new_max)
            self.a_desired_data = deque(old_ad[-new_max:], maxlen=new_max)
            self.a_motor_data = deque(old_am[-new_max:], maxlen=new_max)
            self.a_motor_filtered_data = deque(old_amf[-new_max:], maxlen=new_max)
            self.pwm_data = deque(old_pwm[-new_max:], maxlen=new_max)
            self.mcu_time_data = deque(old_tmcu[-new_max:], maxlen=new_max)
            
            QtWidgets.QMessageBox.information(self, "Thành công", f"Đã đặt số điểm tối đa: {new_max}")
            self.log_edit.appendPlainText(f"[CONFIG] Max points set to: {new_max}")
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "Lỗi", "Vui lòng nhập số nguyên hợp lệ")

    def update_rx_rate(self):
        rate = (self.rx_counter - self.last_rx_counter) * 2  # per second (0.5s window)
        self.last_rx_counter = self.rx_counter
        self.rx_label.setText(f"RX: {rate}/s")
    
    def _apply_moving_average_filter(self, data_deque):
        """Apply moving average filter to the latest data point"""
        if len(data_deque) < self.filter_window:
            # Not enough data, return the last value
            return data_deque[-1] if data_deque else 0.0
        
        # Get the last N points for moving average
        window_data = list(data_deque)[-self.filter_window:]
        return sum(window_data) / len(window_data)
    
    def on_apply_filter(self):
        """Apply new filter window size"""
        try:
            new_window = int(self.filter_edit.text())
            if new_window < 1:
                QtWidgets.QMessageBox.warning(self, "Cảnh báo", "Window size phải >= 1")
                return
            if new_window > 100:
                QtWidgets.QMessageBox.warning(self, "Cảnh báo", "Window size không nên > 100")
                return
            
            old_window = self.filter_window
            self.filter_window = new_window
            
            # Recompute filtered acceleration for all existing data
            self.a_motor_filtered_data.clear()
            for i in range(len(self.a_motor_data)):
                # Get window of data up to current point
                start_idx = max(0, i + 1 - self.filter_window)
                window_data = list(self.a_motor_data)[start_idx:i+1]
                filtered_val = sum(window_data) / len(window_data)
                self.a_motor_filtered_data.append(filtered_val)
            
            QtWidgets.QMessageBox.information(self, "Thành công", 
                f"Đã đặt window = {new_window} (trước: {old_window})\nDữ liệu đã được lọc lại")
            self.log_edit.appendPlainText(f"[CONFIG] Filter window set to: {new_window}")
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "Lỗi", "Vui lòng nhập số nguyên hợp lệ")
    
    def on_apply_dt(self):
        """Apply new time step (dt) setting"""
        try:
            new_dt = float(self.dt_edit.text())
            if new_dt <= 0:
                QtWidgets.QMessageBox.warning(self, "Cảnh báo", "dt phải lớn hơn 0")
                return
            if new_dt > 1.0:
                QtWidgets.QMessageBox.warning(self, "Cảnh báo", "dt không nên lớn hơn 1 giây")
                return
            
            old_dt = self.dt
            self.dt = new_dt
            
            # Recalculate all time values
            self.time_data.clear()
            for i in range(len(self.x_data)):
                self.time_data.append((i + 1) * self.dt)
            
            QtWidgets.QMessageBox.information(self, "Thành công", 
                f"Đã đặt dt = {new_dt}s (trước: {old_dt}s)\nDữ liệu cũ đã được cập nhật thời gian")
            self.log_edit.appendPlainText(f"[CONFIG] Time step (dt) set to: {new_dt}s")
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "Lỗi", "Vui lòng nhập số thực hợp lệ")

    # def on_reset_clicked(self):
    #     """Send reset command to MCU"""
    #     if not (self.serial.ser and self.serial.ser.is_open):
    #         QtWidgets.QMessageBox.information(self, "Thông báo", "Chưa kết nối COM.")
    #         return
        
    #     # Send "reset" command
    #     self.serial.write_text("reset")
    #     print("Đã gửi lệnh: reset")
    #     self.log_edit.appendPlainText("[TX] Reset command")
        
    #     # Optional: Clear all data on reset
    #     self.on_clear_clicked()
def main():
    # Configure PyQtGraph global options
    pg.setConfigOptions(antialias=True, background='w', foreground='k')
    
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()