import os
import time
from PyQt5.QtCore import Qt, QUrl, QTimer
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QDockWidget, QComboBox, QPushButton,
    QLineEdit, QGroupBox, QGridLayout, QSlider,
    QTabWidget, QPlainTextEdit, QTextEdit
)
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage
from PyQt5.QtSerialPort import QSerialPortInfo
from PyQt5.QtGui import QPixmap

from src.workers.video_worker import VideoWorker, HudData
from src.workers.mavlink_worker import MavlinkWorker


class MapWebPage(QWebEnginePage):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window

    def javaScriptConsoleMessage(self, level, message, lineNumber, sourceID):
        if message.startswith("WP_CLICK:"):
            try:
                coords = message.replace("WP_CLICK:", "").strip().split(",")
                lat = float(coords[0].strip())
                lon = float(coords[1].strip())
                self.main_window.add_waypoint_from_map(lat, lon)
            except Exception as e:
                print(f"Waypoint parse hatası: {e}")
        elif message.startswith("WP_REMOVE:"):
            try:
                coords = message.replace("WP_REMOVE:", "").strip().split(",")
                lat = float(coords[0].strip())
                lon = float(coords[1].strip())
                self.main_window.remove_waypoint_from_map(lat, lon)
            except Exception as e:
                print(f"Waypoint remove parse hatası: {e}")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DATUM — UUV Ground Control")
        self.resize(1400, 820)

        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #1a1a2e;
                color: #e0e0e0;
                font-family: 'Segoe UI', 'Arial', sans-serif;
                font-size: 12px;
            }
            QGroupBox {
                border: 1px solid #2d4a6e;
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 4px;
                font-weight: bold;
                font-size: 11px;
                color: #7eb8f7;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QPushButton {
                border-radius: 4px;
                padding: 4px 10px;
                font-size: 11px;
                font-weight: bold;
                border: 1px solid #3a5a8a;
                background-color: #1e3a5f;
                color: #cce0ff;
                min-height: 24px;
            }
            QPushButton:hover  { background-color: #2a4e7f; }
            QPushButton:pressed { background-color: #0d2a4a; }
            QComboBox, QLineEdit {
                background-color: #162032;
                border: 1px solid #2d4a6e;
                border-radius: 4px;
                padding: 2px 6px;
                color: #d0e8ff;
                min-height: 22px;
                font-size: 11px;
            }
            QComboBox::drop-down { border: none; }
            QLabel {
                color: #c0d8f0;
                font-size: 11px;
            }
            QSlider::groove:horizontal {
                height: 4px;
                background: #2d4a6e;
                border-radius: 2px;
            }
            QSlider::handle:horizontal {
                background: #4a90d9;
                border: 1px solid #2d6aaa;
                width: 14px;
                height: 14px;
                margin: -5px 0;
                border-radius: 7px;
            }
            QTabWidget::pane {
                border: 1px solid #2d4a6e;
                border-radius: 4px;
                background: #111827;
            }
            QTabBar::tab {
                background: #162032;
                color: #8ab4d8;
                padding: 4px 12px;
                border: 1px solid #2d4a6e;
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                font-size: 10px;
            }
            QTabBar::tab:selected {
                background: #1e3a5f;
                color: #e0f0ff;
            }
            QDockWidget {
                color: #7eb8f7;
                font-weight: bold;
                font-size: 11px;
                titlebar-close-icon: none;
            }
            QDockWidget::title {
                background: #162032;
                padding: 3px 8px;
                border-bottom: 1px solid #2d4a6e;
            }
            QPlainTextEdit, QTextEdit {
                background-color: #0b1520;
                color: #39d353;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 10px;
                border: none;
                padding: 4px 6px;
            }
        """)

        self.mavlink_worker = None
        self.waypoints = []
        self.telemetry_data = {
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "depth": 0.0, "gps": "Bekleniyor...", "speed": 0.0
        }

        # GPS harita throttle — 2Hz (500ms)
        self._last_gps_map_update = 0.0
        # RC gönderim throttle — 50ms (20Hz maks)
        self._last_rc_send = 0.0
        # Mod değişimi bekleniyorken veya kullanıcı secim yapıyorken combo'nun geri sıfırlanmaması için
        self._pending_mode_id = None
        self._user_selecting_mode = False

        self.hud_data = HudData()
        self.init_ui()

        # Telemetri etiketi 5Hz'de yenilenir (200ms) — UI donmasını önler
        self._telemetry_timer = QTimer(self)
        self._telemetry_timer.setInterval(200)
        self._telemetry_timer.timeout.connect(self.refresh_telemetry_label)
        self._telemetry_timer.start()

        self.video_worker = VideoWorker(camera_source=0, hud_data=self.hud_data, parent=self)
        self.video_worker.frame_ready.connect(self.update_video_frame)
        self.video_worker.start()

    def closeEvent(self, event):
        self._telemetry_timer.stop()
        if hasattr(self, 'video_worker') and self.video_worker.isRunning():
            self.video_worker.stop()
        if self.mavlink_worker is not None and self.mavlink_worker.isRunning():
            self.mavlink_worker.stop()
        event.accept()

    # ------------------------------------------------------------------
    # UI kurulumu
    # ------------------------------------------------------------------
    def init_ui(self):
        central_widget = QWidget(self)
        main_layout = QHBoxLayout(central_widget)

        # ==================== SOL TARAF ====================
        left_layout = QVBoxLayout()

        # Bağlantı Paneli
        conn_group = QGroupBox("Bağlantı Ayarları")
        conn_layout = QHBoxLayout(conn_group)

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.refresh_ports()
        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.port_combo)

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "57600", "115200", "921600"])
        self.baud_combo.setCurrentText("115200")
        conn_layout.addWidget(QLabel("Baud:"))
        conn_layout.addWidget(self.baud_combo)

        self.refresh_btn = QPushButton("Yenile")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        conn_layout.addWidget(self.refresh_btn)

        self.connect_btn = QPushButton("Bağlan")
        self.connect_btn.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold;")
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.connect_btn)
        left_layout.addWidget(conn_group)

        # UUV Operasyonel Kontrolleri
        command_group = QGroupBox("UUV Operasyonel Kontrolleri")
        cmd_grid = QGridLayout(command_group)

        self.arm_btn = QPushButton("ARM")
        self.arm_btn.setStyleSheet("background-color: #b71c1c; color: white; font-weight: bold;")
        self.arm_btn.clicked.connect(lambda: self.send_arm_command(True))
        cmd_grid.addWidget(self.arm_btn, 0, 0)

        self.disarm_btn = QPushButton("DISARM")
        self.disarm_btn.setStyleSheet("background-color: #424242; color: white; font-weight: bold;")
        self.disarm_btn.clicked.connect(lambda: self.send_arm_command(False))
        cmd_grid.addWidget(self.disarm_btn, 0, 1)

        cmd_grid.addWidget(QLabel("Mod:"), 1, 0)
        self.mode_combo = QComboBox()

        # (metin, mode_id) şeklinde UserData ile saklıyoruz — parse'a gerek kalmıyor
        modes = [
            ("MANUAL",    0,  "Kullanıcının doğrudan aracı sürdüğü manuel mod."),
            ("ACRO",      1,  "Dönüş hızının kontrol edildiği akrobatik mod."),
            ("STEERING",  3,  "Yön korumalı, sürüşü kolaylaştıran mod."),
            ("HOLD",      4,  "Aracın anlık konumunda beklemesini sağlar."),
            ("LOITER",    5,  "Mevcut konumu GPS ile korur."),
            ("FOLLOW",    6,  "Lider aracı veya belirli bir hedefi takip eder."),
            ("SIMPLE",    7,  "Kullanıcı yönelimine göre basit hareket modu."),
            ("AUTO",      10, "Yüklenen görev rotasını otomatik uygular."),
            ("RTL",       11, "Başlangıç konumuna geri döner."),
            ("SMART_RTL", 12, "İzlediği yolu hatırlayarak eve dönen akıllı mod."),
        ]
        for idx, (name, mode_id, desc) in enumerate(modes):
            self.mode_combo.addItem(f"{name} ({mode_id})", userData=mode_id)
            self.mode_combo.setItemData(idx, desc, Qt.ToolTipRole)
        self.mode_combo.activated.connect(self._on_mode_combo_user_activated)
        cmd_grid.addWidget(self.mode_combo, 1, 1)

        self.set_mode_btn = QPushButton("Modu Değiştir")
        self.set_mode_btn.clicked.connect(self.change_flight_mode)
        cmd_grid.addWidget(self.set_mode_btn, 1, 2)

        cmd_grid.addWidget(QLabel("Hedef Derinlik (m):"), 2, 0)
        self.depth_input = QLineEdit()
        self.depth_input.setPlaceholderText("Örn: 2.5")
        cmd_grid.addWidget(self.depth_input, 2, 1)

        self.set_depth_btn = QPushButton("Derinliği Ayarla")
        self.set_depth_btn.clicked.connect(self.apply_target_depth)
        cmd_grid.addWidget(self.set_depth_btn, 2, 2)

        cmd_grid.addWidget(QLabel("Yoğunluk:"), 3, 0)
        self.density_val_input = QLineEdit()
        self.density_val_input.setPlaceholderText("Örn: 1000")
        cmd_grid.addWidget(self.density_val_input, 3, 1)

        self.set_density_btn = QPushButton("Yoğunluğu Ayarla")
        self.set_density_btn.clicked.connect(self.apply_density)
        cmd_grid.addWidget(self.set_density_btn, 3, 2)

        left_layout.addWidget(command_group)

        # Video Alanı
        self.video_label = QLabel("Kamera bekleniyor...", self)
        self.video_label.setStyleSheet(
            "background-color: #0a0f1a;"
            "color: #4a6fa5;"
            "border: 1px solid #2d4a6e;"
            "border-radius: 6px;"
        )
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 360)
        left_layout.addWidget(self.video_label, 3)

        # Telemetri Etiketi
        self.telemetry_label = QLabel("Telemetri: Bağlantı bekleniyor...", self)
        self.telemetry_label.setStyleSheet(
            "background-color: #0f1d30;"
            "color: #4fc3f7;"
            "font-family: 'Consolas', 'Courier New', monospace;"
            "font-size: 11px;"
            "padding: 5px 10px;"
            "border: 1px solid #1e4060;"
            "border-radius: 4px;"
        )
        left_layout.addWidget(self.telemetry_label, 0)

        main_layout.addLayout(left_layout, 2)

        # ==================== SAĞ TARAF ====================
        right_layout = QVBoxLayout()

        map_group = QGroupBox("Harita ve Waypoint Görev Planlama")
        map_layout = QVBoxLayout(map_group)

        self.map_view = QWebEngineView()
        self.map_view.setContextMenuPolicy(Qt.NoContextMenu)
        self.map_page = MapWebPage(self)
        self.map_view.setPage(self.map_page)
        map_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "map.html"))
        self.map_view.setUrl(QUrl.fromLocalFile(map_path))
        map_layout.addWidget(self.map_view, 4)

        wp_action_layout = QHBoxLayout()
        self.upload_mission_btn = QPushButton("Görevi Yükle (AUTO)")
        self.upload_mission_btn.setStyleSheet("background-color: #0277bd; color: white; font-weight: bold;")
        self.upload_mission_btn.clicked.connect(self.upload_mission_action)
        wp_action_layout.addWidget(self.upload_mission_btn)

        self.clear_wp_btn = QPushButton("Temizle")
        self.clear_wp_btn.clicked.connect(self.clear_waypoints)
        wp_action_layout.addWidget(self.clear_wp_btn)

        map_layout.addLayout(wp_action_layout)
        right_layout.addWidget(map_group)

        # Manuel Kontrol
        manual_group = QGroupBox("Manuel Kontrol (MANUAL Mod)")
        manual_layout = QGridLayout(manual_group)

        self.throttle_slider = QSlider(Qt.Horizontal)
        self.throttle_slider.setRange(1000, 2000)
        self.throttle_slider.setValue(1500)

        self.rudder_slider = QSlider(Qt.Horizontal)
        self.rudder_slider.setRange(1000, 2000)
        self.rudder_slider.setValue(1500)

        self.throttle_label = QLabel("Gaz (1500)")
        self.rudder_label = QLabel("Yön (1500)")

        # Etiket güncelleme: valueChanged (hafif, sadece label)
        self.throttle_slider.valueChanged.connect(
            lambda v: self.throttle_label.setText(f"Gaz ({v})")
        )
        self.rudder_slider.valueChanged.connect(
            lambda v: self.rudder_label.setText(f"Yön ({v})")
        )

        # RC gönderim: sliderMoved (sadece kullanıcı sürüklediğinde) + throttle
        self.throttle_slider.sliderMoved.connect(self._on_slider_moved)
        self.rudder_slider.sliderMoved.connect(self._on_slider_moved)

        self.reset_rc_btn = QPushButton("Sıfırla")
        self.reset_rc_btn.clicked.connect(self.reset_rc_override)

        manual_layout.addWidget(self.throttle_label, 0, 0)
        manual_layout.addWidget(self.throttle_slider, 0, 1)
        manual_layout.addWidget(self.rudder_label, 1, 0)
        manual_layout.addWidget(self.rudder_slider, 1, 1)
        manual_layout.addWidget(self.reset_rc_btn, 0, 2, 2, 1)

        right_layout.addWidget(manual_group)

        main_layout.addLayout(right_layout, 1)
        self.setCentralWidget(central_widget)

        # Sistem Terminali
        self.dev_console_dock = QDockWidget("  ⬛ Sistem Terminali", self)
        self.dev_console_dock.setFeatures(
            QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable
        )

        dev_widget = QWidget(self)
        dev_layout = QVBoxLayout(dev_widget)
        dev_layout.setContentsMargins(4, 4, 4, 4)
        dev_layout.setSpacing(3)

        self.terminal_tabs = QTabWidget(self)
        self.terminal_tabs.setMaximumHeight(130)

        # Tab 1: Sistem Logları — QPlainTextEdit (çok daha hızlı)
        self.console_output = QPlainTextEdit(self)
        self.console_output.setReadOnly(True)
        self.console_output.setMaximumBlockCount(200)   # Maksimum 200 satır — bellek güvenli
        self.console_output.setPlaceholderText("Konsol hazır.")

        # Tab 2: Otopilot Mesajları — QTextEdit (renk desteği için)
        self.statustext_output = QTextEdit(self)
        self.statustext_output.setReadOnly(True)
        self.statustext_output.setStyleSheet(
            "background-color: #0b1520; color: #58a6ff;"
            "font-family: 'Consolas', 'Courier New', monospace;"
            "font-size: 10px; border: none; padding: 4px 6px;"
        )

        self.terminal_tabs.addTab(self.console_output, "Loglar")
        self.terminal_tabs.addTab(self.statustext_output, "Otopilot")

        dev_layout.addWidget(self.terminal_tabs)

        cmd_layout = QHBoxLayout()
        cmd_layout.setContentsMargins(0, 0, 0, 0)
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("Komut: arm | disarm | param <ad> <değer>")
        self.cmd_input.setMaximumHeight(26)
        self.cmd_input.returnPressed.connect(self.send_dev_command)
        cmd_layout.addWidget(self.cmd_input)
        dev_layout.addLayout(cmd_layout)

        self.dev_console_dock.setWidget(dev_widget)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.dev_console_dock)
        self.dev_console_dock.setMaximumHeight(180)

    # ------------------------------------------------------------------
    # Port yenileme
    # ------------------------------------------------------------------
    def refresh_ports(self):
        self.port_combo.clear()
        self.port_combo.addItem("udpin:0.0.0.0:14550")
        self.port_combo.addItem("tcp:localhost:14550")
        for port in QSerialPortInfo.availablePorts():
            self.port_combo.addItem(port.portName())

    # ------------------------------------------------------------------
    # Bağlantı yönetimi
    # ------------------------------------------------------------------
    def toggle_connection(self):
        if self.mavlink_worker is None or not self.mavlink_worker.isRunning():
            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            if not port:
                return

            # Eski worker sinyallerini temizle (reconnect durumu)
            if self.mavlink_worker is not None:
                self._disconnect_worker()
                self.mavlink_worker = None

            self.connect_btn.setText("Bağlanıyor...")
            self.connect_btn.setStyleSheet("background-color: #f57c00; color: white; font-weight: bold;")

            self.mavlink_worker = MavlinkWorker(port=port, baud=baud, parent=self)
            self._connect_worker()
            self.mavlink_worker.start()
        else:
            self.mavlink_worker.stop()
            self.mavlink_worker = None
            self.connect_btn.setText("Bağlan")
            self.connect_btn.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold;")

    def _connect_worker(self):
        w = self.mavlink_worker
        w.heartbeat_received.connect(self.handle_heartbeat)
        w.attitude_received.connect(self.update_attitude)
        w.depth_received.connect(self.update_depth)
        w.gps_received.connect(self.update_vehicle_gps)
        w.speed_received.connect(self.update_speed)
        w.battery_received.connect(self.update_battery)
        w.gps_fix_received.connect(self.update_gps_fix)
        w.connection_status.connect(self.handle_connection_status)
        w.wp_log_msg.connect(self.append_log)
        w.command_ack_received.connect(self.handle_command_ack)
        w.param_value_received.connect(self.handle_param_value)
        w.mission_ack_received.connect(self.handle_mission_ack)
        w.statustext_received.connect(self.append_statustext_log)
        w.servo_output_received.connect(self.update_servo_sliders)

    def _disconnect_worker(self):
        w = self.mavlink_worker
        try:
            w.heartbeat_received.disconnect()
            w.attitude_received.disconnect()
            w.depth_received.disconnect()
            w.gps_received.disconnect()
            w.speed_received.disconnect()
            w.battery_received.disconnect()
            w.gps_fix_received.disconnect()
            w.connection_status.disconnect()
            w.wp_log_msg.disconnect()
            w.command_ack_received.disconnect()
            w.param_value_received.disconnect()
            w.mission_ack_received.disconnect()
            w.statustext_received.disconnect()
            w.servo_output_received.disconnect()
        except Exception:
            pass

    def handle_connection_status(self, success, message):
        if success:
            self.connect_btn.setText("Bağlantıyı Kes")
            self.connect_btn.setStyleSheet("background-color: #c62828; color: white; font-weight: bold;")
        else:
            self.connect_btn.setText("Bağlan")
            self.connect_btn.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold;")
            # Thread içeriden öldüyse nesneyi serbest bırak
            if self.mavlink_worker and not self.mavlink_worker.isRunning():
                self.mavlink_worker = None

    # ------------------------------------------------------------------
    # Komutlar
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # Araç durumu — heartbeat'ten gerçek zamanlı senkronizasyon
    # ------------------------------------------------------------------

    # ArduRover custom_mode → combo index eşlemesi
    _MODE_ID_TO_COMBO_IDX = {
        0:  0,   # MANUAL
        1:  1,   # ACRO
        3:  2,   # STEERING
        4:  3,   # HOLD
        5:  4,   # LOITER
        6:  5,   # FOLLOW
        7:  6,   # SIMPLE
        10: 7,   # AUTO
        11: 8,   # RTL
        12: 9,   # SMART_RTL
    }

    # ARM butonu renkleri
    _STYLE_ARM_ACTIVE   = "background-color: #1b5e20; color: #69f0ae; font-weight: bold; border: 2px solid #69f0ae;"
    _STYLE_ARM_INACTIVE = "background-color: #b71c1c; color: white; font-weight: bold;"
    _STYLE_DISARM_ACTIVE   = "background-color: #1b5e20; color: #69f0ae; font-weight: bold; border: 2px solid #69f0ae;"
    _STYLE_DISARM_INACTIVE = "background-color: #424242; color: white; font-weight: bold;"

    def _on_mode_combo_user_activated(self, _index):
        # Kullanıcı combobox'tan manuel mod seçti, "Modu Değiştir" butonuna basana kadar heartbeat combobox'ı değiştirmesin
        self._user_selecting_mode = True

    def handle_heartbeat(self, custom_mode: int, base_mode: int):
        # ---- 1. Armed durumu ----
        MAV_MODE_FLAG_SAFETY_ARMED = 128
        is_armed = bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED)

        if is_armed:
            self.arm_btn.setStyleSheet(self._STYLE_ARM_ACTIVE)
            self.disarm_btn.setStyleSheet(self._STYLE_DISARM_INACTIVE)
        else:
            self.arm_btn.setStyleSheet(self._STYLE_ARM_INACTIVE)
            self.disarm_btn.setStyleSheet(self._STYLE_DISARM_ACTIVE)

        # ---- 2. Mod combo senkronizasyonu ----
        # Kullanıcı mod komutu gönderdiyse ve araç henüz o moda geçmediyse
        # combo'ya dokunma — aksi hâlde heartbeat combo'yu eski moda geri sıfırlar.
        if self._pending_mode_id is not None:
            if custom_mode == self._pending_mode_id:
                # Araç istenen moda geçti → kilidi kaldır
                self._pending_mode_id = None
                self._user_selecting_mode = False
            else:
                # Henüz geçmedi → combo'ya dokunma
                return

        # Kullanıcı combobox'ı kendi eliyle değiştirdiyse ancak henüz butonla komut göndermediyse combo'yu koru
        if self._user_selecting_mode:
            return

        idx = self._MODE_ID_TO_COMBO_IDX.get(custom_mode)
        if idx is not None and self.mode_combo.currentIndex() != idx:
            self.mode_combo.blockSignals(True)
            self.mode_combo.setCurrentIndex(idx)
            self.mode_combo.blockSignals(False)

    # ------------------------------------------------------------------
    # Komutlar
    # ------------------------------------------------------------------

    def send_arm_command(self, armed: bool):
        self._last_arm_command = armed
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            self.mavlink_worker.set_arm(armed)

    def change_flight_mode(self):
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            mode_id   = self.mode_combo.currentData()
            mode_name = self.mode_combo.currentText()
            # Komut gönderildi — araç onaylayana kadar combo'yu dondur
            self._pending_mode_id = mode_id
            self._user_selecting_mode = False
            self.mavlink_worker.set_flight_mode(mode_id)
            self.append_log(f"Mod değişimi istendi: {mode_name} → ID={mode_id}")

    def apply_target_depth(self):
        try:
            depth = float(self.depth_input.text())
            if self.mavlink_worker and self.mavlink_worker.isRunning():
                self._last_param_requested = "ATCTARG_DEP"
                self.mavlink_worker.set_parameter("ATCTARG_DEP", depth)
        except ValueError:
            self.append_log("Hata: Derinlik değeri geçerli değil.")

    def apply_density(self):
        try:
            val = float(self.density_val_input.text())
            if self.mavlink_worker and self.mavlink_worker.isRunning():
                self._last_param_requested = "GND_SPEC_GRAV"
                self.mavlink_worker.set_parameter("GND_SPEC_GRAV", val)
                self.append_log(f"Yoğunluk ayarlandı: {val}")
        except ValueError:
            self.append_log("Hata: Yoğunluk değeri geçerli değil.")

    # ------------------------------------------------------------------
    # RC Override (Slider)
    # ------------------------------------------------------------------
    def _on_slider_moved(self, _value):
        """sliderMoved → sadece kullanıcı sürüklerken tetiklenir. 20Hz throttle."""
        now = time.monotonic()
        if now - self._last_rc_send < 0.05:   # 50ms = 20Hz maks
            return
        self._send_rc_now()
        self._last_rc_send = now

    def _send_rc_now(self):
        """Anlık slider değerlerini RC olarak gönder."""
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            throttle = self.throttle_slider.value()
            rudder   = self.rudder_slider.value()
            self.mavlink_worker.send_rc(rudder, throttle)

    def reset_rc_override(self):
        self.throttle_slider.setValue(1500)
        self.rudder_slider.setValue(1500)
        self._send_rc_now()   # Reset'te hemen gönder

    # ------------------------------------------------------------------
    # Waypoint
    # ------------------------------------------------------------------
    def add_waypoint_from_map(self, lat, lon):
        wp = {"lat": lat, "lon": lon, "alt": 0.0}
        self.waypoints.append(wp)
        self.append_log(f"Waypoint Eklendi → Toplam: {len(self.waypoints)}")

    def remove_waypoint_from_map(self, lat, lon):
        for wp in self.waypoints:
            if abs(wp["lat"] - lat) < 0.0001 and abs(wp["lon"] - lon) < 0.0001:
                self.waypoints.remove(wp)
                self.append_log(f"Waypoint Silindi → Kalan: {len(self.waypoints)}")
                return
        self.append_log(f"Hata: Silinecek waypoint bulunamadı ({lat:.5f}, {lon:.5f})")

    def upload_mission_action(self):
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            self.mavlink_worker.upload_mission(self.waypoints)
            if not self.waypoints:
                self.append_log("Görev listesi temizlendi ve araca gönderiliyor.")
            else:
                self.append_log(f"{len(self.waypoints)} adet waypoint yüklemesi başlatıldı.")
        else:
            self.append_log("Bağlantı Hatası: Araç bağlı değil!")

    def clear_waypoints(self):
        self.waypoints = []
        self.map_view.page().runJavaScript(
            "if (typeof clearWaypoints === 'function') { clearWaypoints(); }"
        )
        self.append_log("Haritadaki waypoint'ler silindi.")
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            self.mavlink_worker.upload_mission([])

    # ------------------------------------------------------------------
    # Log (QPlainTextEdit — hızlı)
    # ------------------------------------------------------------------
    def append_log(self, message: str):
        self.console_output.appendPlainText(f"> {message}")

    def append_statustext_log(self, severity: int, text):
        color_map = {
            0: "#ff4444",  # EMERGENCY
            1: "#ff6666",  # ALERT
            2: "#ff8800",  # CRITICAL
            3: "#ffaa00",  # ERROR
            4: "#ffff44",  # WARNING
            5: "#44ccff",  # NOTICE
            6: "#44ff88",  # INFO
            7: "#888888",  # DEBUG
        }
        color = color_map.get(severity, "#ffffff")
        text_str = text.decode("utf-8", "ignore") if isinstance(text, bytes) else str(text)
        self.statustext_output.append(
            f"<span style='color:{color};'>&gt; [{severity}] {text_str}</span>"
        )

    # ------------------------------------------------------------------
    # Telemetri güncelleme (sadece veri güncellenir — label timer'da yenilenir)
    # ------------------------------------------------------------------
    def refresh_telemetry_label(self):
        t = self.telemetry_data
        self.telemetry_label.setText(
            f"Roll: {t['roll']:.1f}° | Pitch: {t['pitch']:.1f}° | Yaw: {t['yaw']:.1f}° | "
            f"Hız: {t['speed']:.2f} m/s | Derinlik: {t['depth']:.2f}m | GPS: {t['gps']}"
        )

    def update_attitude(self, roll, pitch, yaw):
        self.telemetry_data["roll"]  = roll
        self.telemetry_data["pitch"] = pitch
        self.telemetry_data["yaw"]   = yaw
        self.hud_data.set_attitude(roll, pitch, yaw)
        # Label QTimer ile yenileniyor — burada setText çağrılmıyor

    def update_depth(self, depth):
        self.telemetry_data["depth"] = depth
        self.hud_data.set_depth(depth)

    def update_speed(self, speed):
        self.telemetry_data["speed"] = speed

    def update_battery(self, voltage: float):
        self.hud_data.set_voltage(voltage)

    def update_gps_fix(self, fix_type: int):
        self.hud_data.set_gps_fix(fix_type)

    def update_vehicle_gps(self, lat, lon):
        self.telemetry_data["gps"] = f"{lat:.5f}, {lon:.5f}"

        # Harita güncelleme 2Hz'e kısıtlandı (500ms) — runJavaScript pahalı
        now = time.monotonic()
        if now - self._last_gps_map_update >= 0.5:
            js_code = f"updateVehiclePosition({lat}, {lon});"
            self.map_view.page().runJavaScript(js_code)
            self._last_gps_map_update = now

    def update_servo_sliders(self, steering_pwm: int, throttle_pwm: int):
        # Kullanıcı slider'ı sürüklüyorken (mouse basılıysa) değerin değişmesini engelliyoruz
        # Böylece titreme veya mouse elinden kayması olmaz
        if not self.rudder_slider.isSliderDown():
            self.rudder_slider.setValue(steering_pwm)
            
        if not self.throttle_slider.isSliderDown():
            self.throttle_slider.setValue(throttle_pwm)

    def update_video_frame(self, image):
        pixmap = QPixmap.fromImage(image)
        scaled = pixmap.scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.video_label.setPixmap(scaled)

    # ------------------------------------------------------------------
    # ACK / Parametre geri bildirim
    # ------------------------------------------------------------------
    def flash_button(self, btn, success, duration=2000):
        color = "#2e7d32" if success else "#c62828"
        original = btn.styleSheet()
        btn.setStyleSheet(f"background-color: {color}; color: white; font-weight: bold;")
        QTimer.singleShot(duration, lambda: btn.setStyleSheet(original))

    def handle_command_ack(self, command, result, result_str):
        success = (result == 0)
        if command == 400:   # MAV_CMD_COMPONENT_ARM_DISARM
            btn = self.arm_btn if getattr(self, '_last_arm_command', True) else self.disarm_btn
            self.flash_button(btn, success)
            status = "başarılı" if success else f"başarısız → {result_str}"
            self.append_log(f"[ACK] ARM/DISARM {status}")
        elif command == 176:  # MAV_CMD_DO_SET_MODE
            self.flash_button(self.set_mode_btn, success)
            status = "başarılı" if success else f"başarısız → {result_str}"
            self.append_log(f"[ACK] Mod değişimi {status}")
            if not success:
                # Araç komutu reddetti — combo kilidini aç (eski moda heartbeat geri getirecek)
                self._pending_mode_id = None

    def handle_param_value(self, param_id, param_val):
        param_str = (
            param_id.decode("ascii").strip("\x00")
            if isinstance(param_id, bytes)
            else str(param_id)
        )
        last_param = getattr(self, "_last_param_requested", "")
        if param_str == last_param:
            btn_map = {
                "ATCTARG_DEP": self.set_depth_btn,
                "GND_SPEC_GRAV": self.set_density_btn,
            }
            btn = btn_map.get(param_str)
            if btn:
                self.flash_button(btn, True)
            self.append_log(f"Parametre ayarlandı: {param_str} = {param_val:.4f}")
            self._last_param_requested = ""

    def handle_mission_ack(self, result, result_str):
        success = (result == 0)
        if not self.waypoints:
            self.flash_button(self.clear_wp_btn, success)
            msg = "✓ Görev listesi temizlendi" if success else f"✗ Temizleme başarısız → {result_str}"
            self.append_log(f"[WP] {msg}")
        else:
            self.flash_button(self.upload_mission_btn, success)
            if success:
                self.append_log(f"[WP] ✓ {len(self.waypoints)} noktalı görev yüklendi")
            else:
                self.append_log(f"[WP] ✗ Yükleme başarısız → {result_str}")

    # ------------------------------------------------------------------
    # Terminal komut satırı
    # ------------------------------------------------------------------
    def send_dev_command(self):
        cmd = self.cmd_input.text().strip()
        if not cmd:
            return
        self.append_log(f"CMD: {cmd}")
        parts = cmd.lower().split()
        if parts[0] == "arm":
            self.send_arm_command(True)
        elif parts[0] == "disarm":
            self.send_arm_command(False)
        elif parts[0] == "param" and len(parts) >= 3:
            p_name = parts[1].upper()
            try:
                p_val = float(parts[2])
                if self.mavlink_worker and self.mavlink_worker.isRunning():
                    self.mavlink_worker.set_parameter(p_name, p_val)
            except ValueError:
                self.append_log("Hata: Parametre değeri geçerli bir sayı değil.")
        else:
            self.append_log("Bilinmeyen komut. Desteklenenler: arm, disarm, param <ad> <değer>")
        self.cmd_input.clear()