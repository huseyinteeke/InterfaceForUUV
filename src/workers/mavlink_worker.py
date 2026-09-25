import time
import math
from PyQt5.QtCore import QThread, pyqtSignal
from pymavlink import mavutil

class MavlinkWorker(QThread):
    attitude_received    = pyqtSignal(float, float, float)
    heartbeat_received   = pyqtSignal(int, int)   # (custom_mode, base_mode)
    depth_received       = pyqtSignal(float)
    speed_received       = pyqtSignal(float)
    gps_received         = pyqtSignal(float, float)
    battery_received     = pyqtSignal(float)   # Volts
    gps_fix_received     = pyqtSignal(int)     # fix_type (0-6)
    connection_status    = pyqtSignal(bool, str)
    log_msg              = pyqtSignal(str)     # Genel sistem mesajları
    wp_log_msg           = pyqtSignal(str)     # Sadece waypoint / görev mesajları
    command_ack_received = pyqtSignal(int, int, str)
    param_value_received = pyqtSignal(str, float)
    mission_ack_received = pyqtSignal(int, str)
    statustext_received  = pyqtSignal(int, str)
    servo_output_received = pyqtSignal(int, int) # (steering_pwm, throttle_pwm)

    def __init__(self, port, baud=115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._is_running = True
        self.master = None

        self.waypoints_to_upload = []
        self.uploading_mission   = False
        self._pending_upload     = None

        # ArduPilot HOME lokasyonu (seq=0 için kullanılır)
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0
        self.home_known = False

        # Araç anlık konumu
        self.current_lat = 0.0
        self.current_lon = 0.0

    # ---------------------------------------------------------------
    # Ana döngü
    # ---------------------------------------------------------------
    def run(self):
        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
            self.log_msg.emit(f"MAVLink bağlantısı bekleniyor: {self.port} ...")

            connected = False
            last_heartbeat_time      = 0
            last_heartbeat_sent_time = 0

            while self._is_running:
                # GCS Heartbeat — 1 Hz zorunlu.
                # Kesilirse araç GCS Failsafe → RTL'e geçer!
                if time.time() - last_heartbeat_sent_time > 0.7:
                    if self.master:
                        self.master.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            0, 0, 0
                        )
                    last_heartbeat_sent_time = time.time()

                # Heartbeat timeout — 3 saniye sessizlik = bağlantı koptu
                if connected and (time.time() - last_heartbeat_time > 3.0):
                    connected = False
                    self.connection_status.emit(False, "Bağlantı Koptu (Heartbeat Timeout)")
                    self.log_msg.emit("MAVLink bağlantısı koptu (Cihaz söküldü veya Heartbeat alınamıyor)")
                    self._is_running = False
                    break

                try:
                    msg = self.master.recv_match(blocking=False)
                    if not msg:
                        time.sleep(0.005)   # CPU'yu boşa meşgul etme
                        continue

                    msg_type = msg.get_type()

                    # ---- HEARTBEAT ----------------------------------------
                    if msg_type == 'HEARTBEAT':
                        last_heartbeat_time = time.time()
                        mode      = msg.custom_mode
                        base_mode = msg.base_mode
                        self.heartbeat_received.emit(mode, base_mode)

                        if not connected:
                            connected = True
                            print(f"[CONNECT] Sistem {self.master.target_system} bağlandı")
                            self.connection_status.emit(True, f"Bağlandı: Sistem {self.master.target_system}")
                            self.log_msg.emit(f"MAVLink bağlantısı başarılı: Sistem ID {self.master.target_system}")

                            # HOME konumunu iste — upload_mission()'da seq=0 için kullanılacak
                            self.master.mav.command_long_send(
                                self.master.target_system, self.master.target_component,
                                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                                0, 0, 0, 0, 0, 0, 0, 0
                            )

                            # Yüksek frekanslı telemetri stream istekleri
                            self.master.mav.request_data_stream_send(
                                self.master.target_system, self.master.target_component,
                                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 25, 1   # ATTITUDE @ 25 Hz
                            )
                            self.master.mav.request_data_stream_send(
                                self.master.target_system, self.master.target_component,
                                mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1  # GPS/Derinlik @ 10 Hz
                            )
                            self.master.mav.request_data_stream_send(
                                self.master.target_system, self.master.target_component,
                                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5, 1  # Batarya/GPS fix @ 5 Hz
                            )
                            self.master.mav.request_data_stream_send(
                                self.master.target_system, self.master.target_component,
                                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10, 1   # VFR_HUD @ 10 Hz
                            )
                            self.master.mav.request_data_stream_send(
                                self.master.target_system, self.master.target_component,
                                mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 10, 1  # Baro2 (SCALED_PRESSURE2) @ 10 Hz
                            )
                            self.master.mav.request_data_stream_send(
                                self.master.target_system, self.master.target_component,
                                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1  # SERVO_OUTPUT_RAW @ 10 Hz
                            )
                            self.log_msg.emit("Telemetri: ATTITUDE@25Hz · POSITION@10Hz · BATT/GPS@5Hz · VFR@10Hz · BARO2@10Hz · RC@10Hz")

                    # ---- ATTITUDE -----------------------------------------
                    elif msg_type == 'ATTITUDE':
                        roll  = msg.roll  * 57.2958
                        pitch = msg.pitch * 57.2958
                        yaw   = msg.yaw   * 57.2958
                        self.attitude_received.emit(roll, pitch, yaw)

                    # ---- GLOBAL_POSITION_INT (GPS Konum ve Hız) ------------
                    elif msg_type == 'GLOBAL_POSITION_INT':
                        lat   = msg.lat / 1e7
                        lon   = msg.lon / 1e7
                        self.current_lat = lat
                        self.current_lon = lon
                        speed = math.sqrt(msg.vx**2 + msg.vy**2) / 100.0
                        self.gps_received.emit(lat, lon)
                        self.speed_received.emit(speed)

                    # ---- SERVO_OUTPUT_RAW (Gerçek Motor/Servo Çıkışları) ----
                    elif msg_type == 'SERVO_OUTPUT_RAW':
                        steering = msg.servo1_raw
                        throttle = msg.servo8_raw
                        self.servo_output_received.emit(steering, throttle)

                    # ---- VFR_HUD (Yer Hızı) --------------------------------
                    elif msg_type == 'VFR_HUD':
                        self.speed_received.emit(msg.groundspeed)

                    # ---- SCALED_PRESSURE2 
                    elif msg_type == 'SCALED_PRESSURE2':
                        press_diff = getattr(msg, 'press_abs', 0.0)
                        depth = press_diff
                        print(depth)
                        self.depth_received.emit(depth)

      

                    # ---- HOME_POSITION -------------------------------------
                    elif msg_type == 'HOME_POSITION':
                        self.home_lat   = msg.latitude  / 1e7
                        self.home_lon   = msg.longitude / 1e7
                        self.home_alt   = msg.altitude  / 1000.0
                        self.home_known = True
                        print(f"[HOME] lat={self.home_lat:.6f} lon={self.home_lon:.6f} alt={self.home_alt}m")
                        self.log_msg.emit(
                            f"HOME: {self.home_lat:.6f}, {self.home_lon:.6f} (Alt: {self.home_alt:.1f}m)"
                        )

                    # ---- MISSION_REQUEST (eski protokol, MAVLink v1) --------
                    elif msg_type == 'MISSION_REQUEST':
                        # Araç eski protokol kullanıyor: float koordinatlı MISSION_ITEM ile cevap ver.
                        if self.uploading_mission and self.waypoints_to_upload:
                            seq = msg.seq
                            if seq < len(self.waypoints_to_upload):
                                wp = self.waypoints_to_upload[seq]
                                is_home = (seq == 0)
                                print(f"[MISSION_SEND v1] seq={seq} lat={wp['lat']:.6f} lon={wp['lon']:.6f}")
                                self.master.mav.mission_item_send(
                                    self.master.target_system,
                                    self.master.target_component,
                                    seq,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                    0,                      # current
                                    0 if is_home else 1,    # autocontinue
                                    0, 0, 0, 0,             # param1-4
                                    float(wp["lat"]),        # lat (float, derece)
                                    float(wp["lon"]),        # lon (float, derece)
                                    float(wp["alt"]),        # alt
                                )
                                label = "HOME" if is_home else f"WP{seq}"
                                self.wp_log_msg.emit(f"[WP] Nokta gönderildi (v1) → seq={seq} ({label})")

                    # ---- MISSION_REQUEST_INT (yeni protokol, MAVLink v2) ----
                    elif msg_type == 'MISSION_REQUEST_INT':
                        # Araç yeni protokol kullanıyor: int koordinatlı MISSION_ITEM_INT ile cevap ver.
                        if self.uploading_mission and self.waypoints_to_upload:
                            seq = msg.seq
                            if seq < len(self.waypoints_to_upload):
                                wp = self.waypoints_to_upload[seq]
                                is_home = (seq == 0)
                                print(f"[MISSION_SEND v2] seq={seq} lat={wp['lat']:.6f} lon={wp['lon']:.6f}")
                                self.master.mav.mission_item_int_send(
                                    self.master.target_system,
                                    self.master.target_component,
                                    seq,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                    0,                        # current
                                    0 if is_home else 1,      # autocontinue
                                    0, 0, 0, 0,               # param1-4
                                    int(wp["lat"] * 1e7),     # lat (int, 1e7 ölçeği)
                                    int(wp["lon"] * 1e7),     # lon (int, 1e7 ölçeği)
                                    float(wp["alt"]),
                                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                                )
                                label = "HOME" if is_home else f"WP{seq}"
                                self.wp_log_msg.emit(f"[WP] Nokta gönderildi (v2) → seq={seq} ({label})")

                    # ---- MISSION_ACK ---------------------------------------
                    elif msg_type == 'MISSION_ACK':
                        res = msg.type
                        try:
                            res_str = mavutil.mavlink.enums['MAV_MISSION_RESULT'][res].name
                        except KeyError:
                            res_str = f"UNKNOWN({res})"
                        print(f"[MISSION_ACK] result={res} ({res_str})")

                        if self._pending_upload is not None:
                            pending = self._pending_upload
                            self._pending_upload     = None
                            self.waypoints_to_upload = pending
                            self.uploading_mission   = True
                            self.wp_log_msg.emit(
                                f"[WP] Eski görev temizlendi → {len(pending) - 1} waypoint + HOME yükleniyor..."
                            )
                            self.master.mav.mission_count_send(
                                self.master.target_system,
                                self.master.target_component,
                                len(pending),
                                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                            )
                        elif self.uploading_mission:
                            # Yükleme ACK'i → tamamlandı
                            self.uploading_mission = False
                            self.mission_ack_received.emit(res, res_str)

                    # ---- MISSION_ITEM_REACHED ------------------------------
                    elif msg_type == 'MISSION_ITEM_REACHED':
                        seq = msg.seq
                        if seq > 0:
                            self.wp_log_msg.emit(f"[WP] ✓ {seq}. hedefe ulaşıldı!")

                    # ---- MISSION_CURRENT -----------------------------------
                    elif msg_type == 'MISSION_CURRENT':
                        seq = msg.seq
                        # seq=0 = araç henüz görevde değil / HOME'da — loglama
                        # seq>0 = gerçek waypoint'e ilerliyor — logla
                        if seq > 0:
                            if not hasattr(self, '_last_logged_seq') or self._last_logged_seq != seq:
                                self._last_logged_seq = seq
                                self.wp_log_msg.emit(f"[WP] → {seq}. hedefe ilerliyor")

                    # ---- COMMAND_ACK ---------------------------------------
                    elif msg_type == 'COMMAND_ACK':
                        cmd = msg.command
                        res = msg.result
                        try:
                            res_str = mavutil.mavlink.enums['MAV_RESULT'][res].name
                        except KeyError:
                            res_str = f"UNKNOWN({res})"
                        print(f"[CMD_ACK] cmd={cmd} result={res} ({res_str})")
                        self.command_ack_received.emit(cmd, res, res_str)

                    # ---- STATUSTEXT ----------------------------------------
                    elif msg_type == 'STATUSTEXT':
                        severity = msg.severity
                        text     = msg.text
                        print(f"[STATUSTEXT] sev={severity} text={text}")
                        self.statustext_received.emit(severity, text)

                    # ---- PARAM_VALUE ----------------------------------------
                    elif msg_type == 'PARAM_VALUE':
                        param_id  = msg.param_id
                        param_val = msg.param_value
                        print(f"[PARAM] {param_id} = {param_val}")
                        self.param_value_received.emit(param_id, param_val)

                    # ---- BATTERY_STATUS ------------------------------------
                    elif msg_type == 'BATTERY_STATUS':
                        if msg.voltages and msg.voltages[0] != 65535:
                            volts = msg.voltages[0] / 1000.0
                            self.battery_received.emit(volts)

                    # ---- GPS_RAW_INT ----------------------------------------
                    elif msg_type == 'GPS_RAW_INT':
                        self.gps_fix_received.emit(msg.fix_type)

                except Exception:
                    pass

        except Exception as e:
            self.connection_status.emit(False, f"Hata: {str(e)}")
            self.log_msg.emit(f"Bağlantı Hatası: {str(e)}")
        finally:
            if self.master:
                try:
                    self.master.close()
                except Exception:
                    pass
                self.master = None
                self.log_msg.emit("MAVLink bağlantısı kapatıldı.")



    def set_arm(self, armed: bool):
        if not self.master:
            return
        arm_val = 1 if armed else 0
        force   = 21196 if not armed else 0
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, arm_val, force, 0, 0, 0, 0, 0
        )
        self.log_msg.emit(f"Komut gönderildi: {'ARM' if armed else 'DISARM (FORCE)'}")

    def set_flight_mode(self, mode_id: int):
        if not self.master:
            return
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0
        )
        self.log_msg.emit(f"Mod değiştirme komutu gönderildi: {mode_id}")

    def set_parameter(self, param_name: str, value: float):
        if not self.master:
            return
        param_id = param_name.encode('ascii')[:16]
        self.master.mav.param_set_send(
            self.master.target_system, self.master.target_component,
            param_id, value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        self.log_msg.emit(f"Parametre gönderildi: {param_name} = {value}")

    def upload_mission(self, waypoints):
        """
        Waypoint görev listesini araca yükler.

        ArduPilot MAVLink Mission Protocol (DOĞRU sıra):
        ─────────────────────────────────────────────────
        seq = 0  → HOME pozisyonu (MAVLink zorunluluğu).
                   ArduPilot bu noktayı navigasyona dahil ETMEZ,
                   referans/RTL noktası olarak saklar.
        seq = 1…N → Gerçek gezinme waypoint'leri.

        ÖNEMLİ HATALAR (düzeltildi):
          ✗ YANLIŞ: full_list = [ilk_wp_kopyası] + waypoints
            → seq=0 ve seq=1 aynı koordinat → araç seq=0'da takılı kalır.
          ✓ DOĞRU: seq=0 olarak araç HOME koordinatını (veya anlık konumu)
            kullan, seq=1'den itibaren gerçek waypoint'leri sırala.

        Protokol adımları:
          1. MISSION_CLEAR_ALL        → eski görevi sil
          2. MISSION_ACK bekle        → temizleme onayı
          3. MISSION_COUNT            → toplam sayıyı bildir (N_wp + 1 HOME)
          4. Her MISSION_REQUEST_INT  → ilgili seq'i MISSION_ITEM_INT ile yanıtla
          5. MISSION_ACK bekle        → yükleme onayı
        """
        if not self.master:
            self.wp_log_msg.emit("[WP] HATA: Araç bağlı değil!")
            return

        if not waypoints:
            # Temizleme işlemi
            self._pending_upload   = None
            self.uploading_mission = True
            self.master.mav.mission_clear_all_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )
            self.wp_log_msg.emit("[WP] Görev listesi temizleniyor (MISSION_CLEAR_ALL)...")
            return

        # seq=0 → HOME
        # HOME biliniyorsa ArduPilot'un kayıtlı HOME'unu kullan.
        # Bilinmiyorsa aracın anlık konumunu fallback olarak kullan.
        if self.home_known:
            home_item = {"lat": self.home_lat, "lon": self.home_lon, "alt": self.home_alt}
        else:
            home_item = {"lat": self.current_lat, "lon": self.current_lon, "alt": 0.0}
            self.wp_log_msg.emit("[WP] UYARI: HOME henüz bilinmiyor, araç konumu kullanılıyor.")

        # full_list: [HOME, WP1, WP2, ..., WPN]
        full_list = [home_item] + list(waypoints)

        self._pending_upload   = full_list
        self.uploading_mission = True
        self.wp_log_msg.emit(
            f"[WP] Yükleme başlatıldı → {len(waypoints)} waypoint (+ HOME seq=0)"
        )

        self.master.mav.mission_clear_all_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

    def send_rc(self, rudder, throttle):
        if not self.master:
            return
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            rudder,          # CH1 → Steering (Yön)
            0,               # CH2 → kullanılmıyor
            throttle,        # CH3 → Throttle (Gaz)
            0, 0, 0, 0, 0   # CH4-8
        )

    def stop(self):
        self._is_running = False
        self.wait()