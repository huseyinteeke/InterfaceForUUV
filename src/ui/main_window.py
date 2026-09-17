import os
from PyQt5.QtCore import Qt, QUrl
from PyQt5.QtWidgets import (QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, 
                             QLabel, QDockWidget, QShortcut, QInputDialog, 
                             QMessageBox, QComboBox, QPushButton, QLineEdit, QGroupBox, QGridLayout, QSlider)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtSerialPort import QSerialPortInfo
from PyQt5.QtGui import QKeySequence, QPixmap

from src.workers.video_worker import VideoWorker
from src.workers.mavlink_worker import MavlinkWorker
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage

class MapWebPage(QWebEnginePage):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window

    def javaScriptConsoleMessage(self, level, message, lineNumber, sourceID):
        if message.startswith("WP_CLICK:"):
            try:
                coords = message.replace("WP_CLICK:", "").split(",")
                lat = float(coords[0])
                lon = float(coords[1])
                self.main_window.add_waypoint_from_map(lat, lon)
            except Exception as e:
                print(f"Waypoint parse hatası: {e}")
        elif message.startswith("WP_REMOVE:"):
            try:
                coords = message.replace("WP_REMOVE:", "").split(",")
                lat = float(coords[0])
                lon = float(coords[1])
                self.main_window.remove_waypoint_from_map(lat, lon)
            except Exception as e:
                print(f"Waypoint remove parse hatası: {e}")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DATUM")
        self.resize(1400, 800)
        
        self.mavlink_worker = None
        self.waypoints = []
        self.telemetry_data = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "depth": 0.0, "gps": "Bekleniyor...", "speed": 0.0}
        
        self.init_ui()
        
        self.video_worker = VideoWorker(camera_source=0, parent=self)
        self.video_worker.frame_ready.connect(self.update_video_frame)
        self.video_worker.start()

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
        self.port_combo.addItem("udpin:localhost:14550")
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
        
        modes = [
            ("MANUAL (0)", "Kullanıcının doğrudan aracı sürdüğü manuel mod."),
            ("ACRO (1)", "Dönüş hızının kontrol edildiği akrobatik/yarı otonom mod."),
            ("STEERING (3)", "Yön korumalı, sürüşü kolaylaştıran mod."),
            ("HOLD (4)", "Aracın anlık bulunduğu konumda beklemesini sağlar."),
            ("LOITER (5)", "Mevcut konumu (GPS ile) rüzgar/akıntıya rağmen korur."),
            ("FOLLOW (6)", "Lider aracı veya belirli bir hedefi takip eder."),
            ("SIMPLE (7)", "Kullanıcı yönelimine göre basit hareket modu."),
            ("AUTO (10)", "Yüklenen görev rotasını (waypoint) otomatik uygular."),
            ("RTL (11)", "Başlangıç/Ev konumuna (Return to Launch) geri döner."),
            ("SMART_RTL (12)", "İzlediği yolu hatırlayarak eve dönen akıllı mod."),
            ("GUIDED (15)", "Haritadan anlık seçilen tek bir hedefe gitmesini sağlar.")
        ]
        
        for idx, (mode_name, desc) in enumerate(modes):
            self.mode_combo.addItem(mode_name)
            self.mode_combo.setItemData(idx, desc, Qt.ToolTipRole)
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

        # Manuel Kontrol (Sliderlar)

        # Video Alanı
        self.video_label = QLabel("EasyCap Video Bekleniyor...", self)
        self.video_label.setStyleSheet("background-color: black; color: white;")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 360)
        left_layout.addWidget(self.video_label, 3)

        # Telemetri Alanı
        self.telemetry_label = QLabel("Telemetri: Bağlantı Bekleniyor...", self)
        self.telemetry_label.setStyleSheet(
            "background-color: #2b2b2b; "
            "color: #4caf50; "
            "font-family: 'Consolas', 'Courier New', monospace; "
            "font-size: 14px; "
            "font-weight: bold; "
            "padding: 10px; "
            "border: 1px solid #4caf50; "
            "border-radius: 5px;"
        )
        left_layout.addWidget(self.telemetry_label, 1)

        main_layout.addLayout(left_layout, 2)

        # ==================== SAĞ TARAF (Harita Bileşeni) ====================
        right_layout = QVBoxLayout()
        
        map_group = QGroupBox("Harita ve Waypoint Görev Planlama")
        map_layout = QVBoxLayout(map_group)

        # QWebEngineView ile haritayı yüklüyoruz
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

        self.guided_goto_btn = QPushButton("Noktaya Git (GUIDED)")
        self.guided_goto_btn.setStyleSheet("background-color: #e65100; color: white; font-weight: bold;")
        self.guided_goto_btn.clicked.connect(self.guided_goto_action)
        wp_action_layout.addWidget(self.guided_goto_btn)

        self.clear_wp_btn = QPushButton("Temizle")
        self.clear_wp_btn.clicked.connect(self.clear_waypoints)
        wp_action_layout.addWidget(self.clear_wp_btn)

        map_layout.addLayout(wp_action_layout)
        right_layout.addWidget(map_group)

        # Manuel Kontrol (Sliderlar)
        manual_group = QGroupBox("Manuel Kontrol (MANUAL Mod)")
        manual_layout = QGridLayout(manual_group)
        
        self.throttle_slider = QSlider(Qt.Horizontal)
        self.throttle_slider.setRange(1000, 2000)
        self.throttle_slider.setValue(1500)
        self.throttle_slider.valueChanged.connect(self.send_rc_override)
        
        self.rudder_slider = QSlider(Qt.Horizontal)
        self.rudder_slider.setRange(1000, 2000)
        self.rudder_slider.setValue(1500)
        self.rudder_slider.valueChanged.connect(self.send_rc_override)

        self.throttle_label = QLabel("Gaz (1500)")
        self.rudder_label = QLabel("Yön (1500)")

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

        # Geliştirici Konsolu
        self.dev_console_dock = QDockWidget("Geliştirici Konsolu", self)
        dev_widget = QWidget(self)
        dev_layout = QVBoxLayout(dev_widget)
        self.console_output = QLabel("Konsol hazır.", self)
        self.console_output.setStyleSheet(
            "background-color: #121212; "
            "color: #00ff00; "
            "font-family: 'Consolas', 'Courier New', monospace; "
            "font-size: 13px; "
            "padding: 8px; "
            "border: 1px solid #333; "
            "border-radius: 4px;"
        )
        self.console_output.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.console_output.setWordWrap(True)
        dev_layout.addWidget(self.console_output, 4)
        
        cmd_layout = QHBoxLayout()
        self.cmd_input = QLineEdit()
        self.cmd_input.returnPressed.connect(self.send_dev_command)
        cmd_layout.addWidget(self.cmd_input)
        dev_layout.addLayout(cmd_layout)
        
        self.dev_console_dock.setWidget(dev_widget)
        self.addDockWidget(Qt.RightDockWidgetArea, self.dev_console_dock)
        self.dev_console_dock.hide()

        self.dev_shortcut = QShortcut(QKeySequence("Ctrl+Shift+D"), self)
        self.dev_shortcut.activated.connect(self.toggle_dev_console)

    def refresh_ports(self):
        self.port_combo.clear()
        self.port_combo.addItem("udpin:localhost:14550")
        self.port_combo.addItem("tcp:localhost:14550")
        ports = QSerialPortInfo.availablePorts()
        for port in ports: self.port_combo.addItem(port.portName())

    def toggle_connection(self):
        if self.mavlink_worker is None or not self.mavlink_worker.isRunning():
            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            if not port or port == "Port Bulunamadı": return
            
            self.connect_btn.setText("Bağlanıyor...")
            self.connect_btn.setStyleSheet("background-color: #f57c00; color: white; font-weight: bold;")
            
            self.mavlink_worker = MavlinkWorker(port=port, baud=baud, parent=self)
            self.mavlink_worker.attitude_received.connect(self.update_attitude)
            self.mavlink_worker.depth_received.connect(self.update_depth)
            self.mavlink_worker.gps_received.connect(self.update_vehicle_gps)
            self.mavlink_worker.speed_received.connect(self.update_speed)
            self.mavlink_worker.connection_status.connect(self.handle_connection_status)
            self.mavlink_worker.log_msg.connect(self.append_log)
            self.mavlink_worker.start()
        else:
            self.mavlink_worker.stop()
            self.mavlink_worker = None
            self.connect_btn.setText("Bağlan")
            self.connect_btn.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold;")

    def handle_connection_status(self, success, message):
        if success:
            self.connect_btn.setText("Bağlantıyı Kes")
            self.connect_btn.setStyleSheet("background-color: #c62828; color: white; font-weight: bold;")
        else:
            self.connect_btn.setText("Bağlan")
            self.connect_btn.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold;")

    def send_arm_command(self, armed: bool):
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            self.mavlink_worker.set_arm(armed)

    def send_rc_override(self):
        throttle = self.throttle_slider.value()
        rudder = self.rudder_slider.value()
        self.throttle_label.setText(f"Gaz ({throttle})")
        self.rudder_label.setText(f"Yön ({rudder})")
        
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            self.mavlink_worker.send_rc(rudder, throttle)

    def reset_rc_override(self):
        self.throttle_slider.setValue(1500)
        self.rudder_slider.setValue(1500)

    def change_flight_mode(self):
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            mode_map = {
                "MANUAL (0)": 0, "ACRO (1)": 1, "STEERING (3)": 3, "HOLD (4)": 4, 
                "LOITER (5)": 5, "FOLLOW (6)": 6, "SIMPLE (7)": 7, "AUTO (10)": 10, 
                "RTL (11)": 11, "SMART_RTL (12)": 12, "GUIDED (15)": 15
            }
            selected = self.mode_combo.currentText()
            mode_id = mode_map.get(selected, 0)
            self.mavlink_worker.set_flight_mode(mode_id)
            self.append_log(f"Mod değişimi istendi: {selected}")

    def apply_target_depth(self):
        try:
            depth = float(self.depth_input.text())
            if self.mavlink_worker and self.mavlink_worker.isRunning():
                self.mavlink_worker.set_target_depth(depth)
        except ValueError: pass


    def apply_density(self):
        try:
            val = float(self.density_val_input.text())
            if self.mavlink_worker and self.mavlink_worker.isRunning():
                self.mavlink_worker.set_parameter("DENSITY", val)
                self.append_log(f"Yoğunluk ayarlandı: {val}")
        except ValueError:
            self.append_log("Hata: Yoğunluk değeri geçerli değil.")

    def add_waypoint_from_map(self, lat, lon):
        wp = {"lat": lat, "lon": lon, "alt": 0.0}
        self.waypoints.append(wp)
        self.append_log(f"Waypoint Eklendi -> Toplam: {len(self.waypoints)}")

    def remove_waypoint_from_map(self, lat, lon):
        # Eşleşen waypointi listeden çıkar (yaklaşık koordinat eşleştirmesi)
        for wp in self.waypoints:
            if abs(wp["lat"] - lat) < 0.00001 and abs(wp["lon"] - lon) < 0.00001:
                self.waypoints.remove(wp)
                self.append_log(f"Waypoint Silindi -> Kalan: {len(self.waypoints)}")
                break

    def upload_mission_action(self):
        if not self.waypoints:
            self.append_log("Uyarı: Henüz haritadan waypoint seçilmedi!")
            return

        if self.mavlink_worker and self.mavlink_worker.isRunning():
            self.mavlink_worker.upload_mission(self.waypoints)
            self.append_log(f"{len(self.waypoints)} adet görev noktası için yükleme başlatıldı.")
        else:
            self.append_log("Bağlantı Hatası: Araç bağlı değil!")

    def guided_goto_action(self):
        if not self.waypoints:
            self.append_log("Uyarı: Hedef nokta yok. Haritadan bir nokta seçin.")
            return
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            wp = self.waypoints[-1] # Son seçilen noktaya git
            self.mavlink_worker.send_guided_waypoint(wp["lat"], wp["lon"], wp["alt"])
            self.append_log(f"GUIDED modu hedefi gönderildi: {wp['lat']}, {wp['lon']}")
        else:
            self.append_log("Bağlantı Hatası: Araç bağlı değil!")

    def clear_waypoints(self):
        self.waypoints = []
        self.map_view.page().runJavaScript("if (typeof clearWaypoints === 'function') { clearWaypoints(); }")
        self.append_log("Waypoint listesi temizlendi.")

    def append_log(self, message: str):
        current_text = self.console_output.text()
        lines = current_text.split('\n')
        if len(lines) > 20:
            lines = lines[-20:]
        lines.append(f"> {message}")
        self.console_output.setText('\n'.join(lines))
    def update_video_frame(self, image):
        self.video_label.setPixmap(QPixmap.fromImage(image).scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def refresh_telemetry_label(self):
        t = self.telemetry_data
        self.telemetry_label.setText(
            f"Roll: {t['roll']:.2f}° | Pitch: {t['pitch']:.2f}° | Yaw: {t['yaw']:.2f}° | "
            f"Hız: {t['speed']:.2f} m/s | Derinlik: {t['depth']:.2f}m | GPS: {t['gps']}"
        )

    def update_attitude(self, roll, pitch, yaw):
        self.telemetry_data.update({"roll": roll, "pitch": pitch, "yaw": yaw})
        self.refresh_telemetry_label()

    def update_depth(self, depth):
        self.telemetry_data["depth"] = depth
        self.refresh_telemetry_label()

    def update_speed(self, speed):
        self.telemetry_data["speed"] = speed
        self.refresh_telemetry_label()

    def send_dev_command(self):
        cmd = self.cmd_input.text().strip()
        if not cmd: return
        
        self.append_log(f"KULLANICI: {cmd}")
        
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

    def toggle_dev_console(self):
        password, ok = QInputDialog.getText(self, "Geliştirici Girişi", "Yetkili Şifresi:", QLineEdit.Password, "")
        if ok and password == "sara2026":
            if self.dev_console_dock.isHidden(): self.dev_console_dock.show()
            else: self.dev_console_dock.hide()

    def closeEvent(self, event):
        self.video_worker.stop()
        if self.mavlink_worker and self.mavlink_worker.isRunning(): self.mavlink_worker.stop()
        event.accept()

    def update_vehicle_gps(self, lat, lon):
        js_code = f"updateVehiclePosition({lat}, {lon});"
        self.map_view.page().runJavaScript(js_code)
        
        self.telemetry_data["gps"] = f"{lat:.5f}, {lon:.5f}"
        self.refresh_telemetry_label()