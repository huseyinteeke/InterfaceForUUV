import time
import math
from PyQt5.QtCore import QThread, pyqtSignal
from pymavlink import mavutil

class MavlinkWorker(QThread):
    attitude_received = pyqtSignal(float, float, float)
    heartbeat_received = pyqtSignal(int)
    depth_received = pyqtSignal(float)
    speed_received = pyqtSignal(float)
    gps_received = pyqtSignal(float, float)
    connection_status = pyqtSignal(bool, str)
    log_msg = pyqtSignal(str) 
    command_ack_received = pyqtSignal(int, int, str)
    param_value_received = pyqtSignal(str, float)
    mission_ack_received = pyqtSignal(int, str)
    statustext_received = pyqtSignal(int, str)

    def __init__(self, port, baud=115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._is_running = True
        self.master = None
        
        self.waypoints_to_upload = []
        self.uploading_mission = False

    def run(self):
        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
            self.log_msg.emit(f"MAVLink bağlantısı bekleniyor: {self.port} ...")
            
            connected = False
            last_heartbeat_time = 0
            last_heartbeat_sent_time = 0
            
            while self._is_running:
                # GCS olarak araca 1 Hz hızında Heartbeat göndermeliyiz.
                # Aksi takdirde araç GCS Failsafe tetikler ve sürekli RTL (Eve Dönüş) moduna girer!
                if time.time() - last_heartbeat_sent_time > 1.0:
                    if self.master:
                        self.master.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            0, 0, 0
                        )
                    last_heartbeat_sent_time = time.time()

                if connected and (time.time() - last_heartbeat_time > 3.0):
                    connected = False
                    self.connection_status.emit(False, "Bağlantı Koptu (Heartbeat Timeout)")
                    self.log_msg.emit("MAVLink bağlantısı koptu (Cihaz söküldü veya Heartbeat alınamıyor)")
                    self._is_running = False
                    break

                try:
                    msg = self.master.recv_match(blocking=True, timeout=0.1)
                    if not msg:
                        continue
                    
                    msg_type = msg.get_type()
                    
                    # Heartbeat işleme
                    if msg_type == 'HEARTBEAT':
                        last_heartbeat_time = time.time()
                        mode = msg.custom_mode
                        base_mode = msg.base_mode
                        armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        print(f"[HB] mode={mode} armed={armed} base_mode={base_mode}")
                        self.heartbeat_received.emit(mode)
                        
                        if not connected:
                            connected = True
                            print(f"[CONNECT] Sistem {self.master.target_system} bağlandı")
                            self.connection_status.emit(True, f"Bağlandı: Sistem {self.master.target_system}")
                            self.log_msg.emit(f"MAVLink bağlantısı başarılı: Sistem ID {self.master.target_system}")
                            
                            # Bağlantı kurulduğunda eski kalıntı görevi temizle
                            self.master.mav.mission_clear_all_send(
                                self.master.target_system,
                                self.master.target_component,
                                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                            )
                            print("[CONNECT] Eski görev temizlendi (MISSION_CLEAR_ALL)")
                            self.log_msg.emit("Bağlantı: Önceki oturumdan kalan görev temizlendi.")
                            
                            # HOME konumunu iste
                            self.master.mav.command_long_send(
                                self.master.target_system, self.master.target_component,
                                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                                0, 0, 0, 0, 0, 0, 0, 0
                            )

                    elif msg_type == 'ATTITUDE':
                        roll = msg.roll * 57.2958
                        pitch = msg.pitch * 57.2958
                        yaw = msg.yaw * 57.2958
                        self.attitude_received.emit(roll, pitch, yaw)

                    elif msg_type == 'GLOBAL_POSITION_INT':
                        lat = msg.lat / 1e7
                        lon = msg.lon / 1e7
                        self.current_lat = lat
                        self.current_lon = lon
                        
                        depth = -msg.relative_alt / 1000.0
                        speed = math.sqrt(msg.vx**2 + msg.vy**2) / 100.0
                        print(f"[GPS] lat={lat:.6f} lon={lon:.6f} speed={speed:.2f} m/s depth={depth:.2f}m")
                        
                        self.depth_received.emit(depth)
                        self.gps_received.emit(lat, lon)
                        self.speed_received.emit(speed)
                    
                    elif msg_type in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
                        print(f"[MISSION_REQ] seq={msg.seq} uploading={self.uploading_mission} wp_count={len(self.waypoints_to_upload)}")
                        if self.uploading_mission and self.waypoints_to_upload:
                            seq = msg.seq
                            if seq < len(self.waypoints_to_upload):
                                wp = self.waypoints_to_upload[seq]
                                print(f"[MISSION_SEND] seq={seq} lat={wp['lat']:.6f} lon={wp['lon']:.6f} alt={wp['alt']}")
                                self.master.mav.mission_item_int_send(
                                    self.master.target_system,
                                    self.master.target_component,
                                    seq,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                    0, # current
                                    1, # autocontinue
                                    0, 0, 0, 0, # parametreler
                                    int(wp["lat"] * 1e7),
                                    int(wp["lon"] * 1e7),
                                    float(wp["alt"]),
                                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                                )
                                self.log_msg.emit(f"Görev noktası gönderildi: {seq}")
                    
                    elif msg_type == 'MISSION_ACK':
                        res = msg.type
                        try:
                            res_str = mavutil.mavlink.enums['MAV_MISSION_RESULT'][res].name
                        except KeyError:
                            res_str = f"UNKNOWN({res})"
                        print(f"[MISSION_ACK] result={res} ({res_str}) pending={getattr(self, '_pending_upload', None) is not None}")
                        
                        # Temizleme sonrası bekleyen yükleme varsa başlat
                        if hasattr(self, '_pending_upload') and self._pending_upload is not None:
                            pending = self._pending_upload
                            self._pending_upload = None
                            self.waypoints_to_upload = pending
                            self.uploading_mission = True
                            print(f"[MISSION_UPLOAD] Temizleme tamam, {len(pending)} nokta yükleniyor...")
                            self.log_msg.emit(f"Eski görev temizlendi. Yeni görev yükleniyor ({len(pending)} nokta)...")
                            self.master.mav.mission_count_send(
                                self.master.target_system,
                                self.master.target_component,
                                len(pending),
                                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                            )
                        elif self.uploading_mission:
                            self.uploading_mission = False
                            
                            if res == 0 and len(self.waypoints_to_upload) > 1:
                                print(f"[MISSION_SET_CURRENT] seq=1 ayarlanıyor...")
                                self.master.mav.command_long_send(
                                    self.master.target_system,
                                    self.master.target_component,
                                    mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                                    0,
                                    1, 0, 0, 0, 0, 0, 0
                                )
                                self.log_msg.emit("BİLGİ: Görev başlangıcı 1. hedefe ayarlandı.")
                                
                            self.mission_ack_received.emit(res, res_str)

                    elif msg_type == 'MISSION_ITEM_REACHED':
                        seq = msg.seq
                        print(f"[REACHED] seq={seq}")
                        self.log_msg.emit(f"BİLGİ: Araç {seq}. hedefine başarıyla ulaştı!")
                        
                    elif msg_type == 'MISSION_CURRENT':
                        seq = msg.seq
                        print(f"[MISSION_CURRENT] seq={seq}")
                        if not hasattr(self, 'current_mission_seq') or self.current_mission_seq != seq:
                            self.current_mission_seq = seq
                            self.log_msg.emit(f"BİLGİ: Araç şu anda {seq}. hedefe ilerliyor...")

                    elif msg_type == 'HOME_POSITION':
                        h_lat = msg.latitude / 1e7
                        h_lon = msg.longitude / 1e7
                        h_alt = msg.altitude / 1000.0
                        print(f"[HOME] lat={h_lat:.6f} lon={h_lon:.6f} alt={h_alt}m")
                        self.log_msg.emit(f"BİLGİ: Drone HOME lokasyonu: {h_lat:.6f}, {h_lon:.6f} (Alt: {h_alt}m)")

                    elif msg_type == 'COMMAND_ACK':
                        cmd = msg.command
                        res = msg.result
                        try:
                            res_str = mavutil.mavlink.enums['MAV_RESULT'][res].name
                        except KeyError:
                            res_str = f"UNKNOWN({res})"
                        print(f"[CMD_ACK] cmd={cmd} result={res} ({res_str})")
                        self.command_ack_received.emit(cmd, res, res_str)

                    elif msg_type == 'STATUSTEXT':
                        severity = msg.severity
                        text = msg.text
                        print(f"[STATUSTEXT] sev={severity} text={text}")
                        self.statustext_received.emit(severity, text)

                    elif msg_type == 'PARAM_VALUE':
                        param_id = msg.param_id
                        param_val = msg.param_value
                        print(f"[PARAM] {param_id} = {param_val}")
                        self.param_value_received.emit(param_id, param_val)

                except Exception as e:
                    time.sleep(0.1)
                    pass

        except Exception as e:
            self.connection_status.emit(False, f"Hata: {str(e)}")
            self.log_msg.emit(f"Bağlantı Hatası: {str(e)}")
        finally:
            if self.master:
                try:
                    self.master.close()
                except:
                    pass
                self.master = None
                self.log_msg.emit("MAVLink bağlantısı kapatıldı.")

    def set_arm(self, armed: bool):
        if not self.master: return
        
        if not armed:
            # Önce HOLD moduna çek ki araç dursun
            print("[CMD] HOLD moduna geçiriliyor (DISARM öncesi)")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4, 0, 0, 0, 0, 0  # 4 = HOLD
            )
        
        arm_val = 1 if armed else 0
        # param2 = 21196 → ArduPilot "force" arm/disarm (hareket halindeyken bile çalışır)
        force = 21196 if not armed else 0
        print(f"[CMD] {'ARM' if armed else 'DISARM'} gönderiliyor (force={force})")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, arm_val, force, 0, 0, 0, 0, 0
        )
        durum = "ARM" if armed else "DISARM (FORCE)"
        self.log_msg.emit(f"Komut gönderildi: {durum}")

    def set_flight_mode(self, mode_id: int):
        if not self.master: return
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0
        )
        self.log_msg.emit(f"Uçuş modu değiştirme komutu gönderildi: {mode_id}")

    def set_parameter(self, param_name: str, value: float):
        if not self.master: return
        param_id = param_name.encode('ascii')[:16]
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            param_id,
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        self.log_msg.emit(f"Parametre gönderildi: {param_name} = {value}")


    def upload_mission(self, waypoints):
        """
        Waypoint görev listesini araca yükler.
        
        MAVLink Mission Protocol doğru sırası:
        1. MISSION_CLEAR_ALL → eski görevi sil
        2. MISSION_ACK bekle (temizleme onayı)
        3. MISSION_COUNT → yeni görev sayısını bildir
        4. MISSION_REQUEST_INT'lere cevap ver
        5. MISSION_ACK bekle (yükleme onayı)
        6. MAV_CMD_DO_SET_MISSION_CURRENT → başlangıç noktasını ayarla
        
        ArduPilot'ta seq=0 HOME lokasyonudur, araç seq=1'den başlar.
        """
        if not self.master:
            self.log_msg.emit("Araç bağlı değil!")
            return

        if not waypoints:
            # Sadece temizleme
            self._pending_upload = None
            self.uploading_mission = True
            self.master.mav.mission_clear_all_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )
            self.log_msg.emit("Görev listesi temizleme komutu gönderildi (MISSION_CLEAR_ALL).")
            return

        # HOME (seq=0) = görevin ilk noktası
        home_wp = {
            "lat": waypoints[0]["lat"],
            "lon": waypoints[0]["lon"],
            "alt": waypoints[0]["alt"]
        }
        
        full_list = [home_wp] + waypoints
        
        # Önce eski görevi temizle, temizleme ACK'i gelince _pending_upload'tan yükle
        self._pending_upload = full_list
        self.uploading_mission = True
        self.log_msg.emit(f"Önce eski görev temizleniyor, sonra {len(full_list)} nokta yüklenecek...")
        
        self.master.mav.mission_clear_all_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

    def send_guided_waypoint(self, lat, lon, alt):
        if not self.master: return
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b110111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        self.log_msg.emit(f"GUIDED Hedef Gönderildi: {lat}, {lon}")

    def send_rc(self, rudder, throttle):
        if not self.master: return
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            rudder,       # Channel 1 (Steering)
            0,            # Channel 2 (unused for basic rover)
            throttle,     # Channel 3 (Throttle)
            0, 0, 0, 0, 0 # Channels 4-8
        )

    def stop(self):
        self._is_running = False
        self.wait()