from PyQt5.QtCore import QThread, pyqtSignal
from pymavlink import mavutil

class MavlinkWorker(QThread):
    attitude_received = pyqtSignal(float, float, float)
    heartbeat_received = pyqtSignal(int)
    depth_received = pyqtSignal(float)
    speed_received = pyqtSignal(float)
    gps_received = pyqtSignal(float, float)
    connection_status = pyqtSignal(bool, str)
    log_msg = pyqtSignal(str) # Log mesajları için yeni sinyal

    def __init__(self, port, baud=115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._is_running = True
        self.master = None
        
        # Görev (Mission) yükleme durumu için değişkenler
        self.waypoints_to_upload = []
        self.uploading_mission = False

    def run(self):
        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
            self.log_msg.emit(f"MAVLink bağlantısı bekleniyor: {self.port} ...")
            
            connected = False
            
            while self._is_running:
                try:
                    msg = self.master.recv_match(blocking=True, timeout=0.1)
                    if not msg:
                        continue
                    
                    msg_type = msg.get_type()
                    
                    # İlk heartbeat alındığında bağlantıyı "Başarılı" olarak işaretle
                    if not connected and msg_type == 'HEARTBEAT':
                        connected = True
                        self.connection_status.emit(True, f"Bağlandı: Sistem {self.master.target_system}")
                        self.log_msg.emit(f"MAVLink bağlantısı başarılı: Sistem ID {self.master.target_system}")

                    if msg_type == 'ATTITUDE':
                        roll = msg.roll * 57.2958
                        pitch = msg.pitch * 57.2958
                        yaw = msg.yaw * 57.2958
                        self.attitude_received.emit(roll, pitch, yaw)
                        
                    elif msg_type == 'HEARTBEAT':
                        mode = msg.custom_mode
                        self.heartbeat_received.emit(mode)

                    elif msg_type == 'GLOBAL_POSITION_INT':
                        lat = msg.lat / 1e7
                        lon = msg.lon / 1e7
                        depth = -msg.relative_alt / 1000.0
                        import math
                        speed = math.sqrt(msg.vx**2 + msg.vy**2) / 100.0
                        
                        self.depth_received.emit(depth)
                        self.gps_received.emit(lat, lon)
                        self.speed_received.emit(speed)
                    
                    elif msg_type in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
                        if hasattr(self, 'uploading_mission') and self.uploading_mission:
                            seq = msg.seq
                            if seq < len(self.waypoints_to_upload):
                                wp = self.waypoints_to_upload[seq]
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
                        if hasattr(self, 'uploading_mission') and self.uploading_mission:
                            self.uploading_mission = False
                            self.log_msg.emit(f"Görev yükleme tamamlandı. Durum: {msg.type}")

                except Exception as e:
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
        arm_val = 1 if armed else 0
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, arm_val, 0, 0, 0, 0, 0, 0
        )
        durum = "ARM" if armed else "DISARM"
        self.log_msg.emit(f"Komut gönderildi: {durum}")

    def set_flight_mode(self, mode_id: int):
        if not self.master: return
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
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

    def set_target_depth(self, depth_meters: float):
        """
        Kullanıcının özel olarak ArduPilot'a eklediği bir parametre üzerinden
        derinlik hedefi gönderir. 'TGT_DEPTH' kısmını kendi eklediğiniz
        parametre ismi ile değiştirebilirsiniz.
        Genel parametre gönderme metodunu (set_parameter) kullanarak ekstra
        metot oluşumundan kaçınıyoruz.
        """
        if not self.master: return
        
        # Kullanıcının eklediği parametre ismi. Gerektiğinde buradan değiştirebilirsiniz.
        param_adi = "TGT_DEPTH"
        
        self.set_parameter(param_adi, depth_meters)

    def upload_mission(self, waypoints):
        """
        GCS olarak sadece görev sayısını araca bildiriyoruz.
        Araç bize MISSION_REQUEST dönecek, run() döngüsünde yakalayıp 
        noktaları tek tek göndereceğiz. (MAVLink Protokolü)
        """
        if not self.master or not waypoints:
            self.log_msg.emit("Araç bağlı değil veya waypoint listesi boş!")
            return

        self.waypoints_to_upload = waypoints
        self.uploading_mission = True
        
        self.log_msg.emit(f"Toplam {len(waypoints)} adet waypoint için görev başlangıcı bildiriliyor...")
        
        # Görev sayısını bildir
        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            len(waypoints),
            0
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