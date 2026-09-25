import cv2
import math
import numpy as np
import threading
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage


class HudData:
    """Thread-safe container for MAVLink telemetry data used by the HUD overlay."""

    def __init__(self):
        self._lock = threading.Lock()
        self._roll = 0.0       # degrees
        self._pitch = 0.0      # degrees
        self._yaw = 0.0        # degrees (heading)
        self._depth = 0.0      # metres (positive = below surface)
        self._voltage = 0.0    # Volts  (0 = unknown)
        self._gps_fix = 0      # 0=NoFix 2=2D 3=3D 4=3D+DGPS 5=RTK float 6=RTK fixed

    # ------------------------------------------------------------------ setters
    def set_attitude(self, roll: float, pitch: float, yaw: float):
        with self._lock:
            self._roll = roll
            self._pitch = pitch
            self._yaw = yaw

    def set_depth(self, depth: float):
        with self._lock:
            self._depth = depth

    def set_voltage(self, voltage: float):
        with self._lock:
            self._voltage = voltage

    def set_gps_fix(self, fix_type: int):
        with self._lock:
            self._gps_fix = fix_type

    # ------------------------------------------------------------------ getters
    def snapshot(self):
        """Return a consistent snapshot of all values (no tearing)."""
        with self._lock:
            return (
                self._roll,
                self._pitch,
                self._yaw,
                self._depth,
                self._voltage,
                self._gps_fix,
            )


# ---------------------------------------------------------------------------
# HUD drawing helpers
# ---------------------------------------------------------------------------

def _alpha_rect(frame, x1, y1, x2, y2, color, alpha=0.45):
    """Draw a semi-transparent filled rectangle."""
    overlay = frame.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)


def _put_text(frame, text, pos, scale=0.55, color=(255, 255, 255),
              thickness=1, shadow=True):
    if shadow:
        cv2.putText(frame, text, (pos[0] + 1, pos[1] + 1),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 1,
                    cv2.LINE_AA)
    cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color,
                thickness, cv2.LINE_AA)


def _rotate_point(cx, cy, px, py, angle_deg):
    """Rotate point (px,py) around (cx,cy) by angle_deg degrees."""
    rad = math.radians(angle_deg)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    dx, dy = px - cx, py - cy
    return (
        int(cx + dx * cos_a - dy * sin_a),
        int(cy + dx * sin_a + dy * cos_a),
    )


# ---------------------------------------------------------------------------
# Artificial Horizon (AHRS)
# ---------------------------------------------------------------------------

def _draw_artificial_horizon(frame, cx, cy, radius, roll_deg, pitch_deg):
    HORIZON_ALPHA = 0.10   # 0 = tamamen şeffaf, 1 = tamamen opak

    px_per_deg = radius / 25.0
    pitch_offset = int(pitch_deg * px_per_deg)

    size = radius * 2 + 4
    lx, ly = size // 2, size // 2

    rad = math.radians(-roll_deg)
    cos_r, sin_r = math.cos(rad), math.sin(rad)

    # --- Vektörize sky/ground hesabı (döngüsüz, hızlı) ---
    rows_idx, cols_idx = np.mgrid[0:size, 0:size]
    dx = cols_idx - lx
    dy = rows_idx - ly

    # Ufuk ekseni koordinatlarına döndür
    ry = -dx * sin_r + dy * cos_r
    is_ground = (ry > pitch_offset)

    sky_color    = np.array([80, 40, 10],  dtype=np.uint8)   # BGR — koyu mavi tonu
    ground_color = np.array([20, 70, 30],  dtype=np.uint8)   # BGR — koyu yeşil-kahve

    canvas = np.where(is_ground[:, :, None], ground_color, sky_color).astype(np.uint8)

    # --- Dairesel maske ---
    mask = np.zeros((size, size), dtype=np.uint8)
    cv2.circle(mask, (lx, ly), radius, 255, -1)

    # --- Çerçeve üzerine alpha-blend (sadece daire içi) ---
    x0, y0 = cx - lx, cy - ly
    fh, fw = frame.shape[:2]

    src_x1 = max(0, -x0);  src_y1 = max(0, -y0)
    src_x2 = min(size, fw - x0);  src_y2 = min(size, fh - y0)
    dst_x1 = max(0, x0);   dst_y1 = max(0, y0)
    dst_x2 = dst_x1 + (src_x2 - src_x1)
    dst_y2 = dst_y1 + (src_y2 - src_y1)

    if src_x2 > src_x1 and src_y2 > src_y1:
        roi        = frame[dst_y1:dst_y2, dst_x1:dst_x2]          # orijinal kamera
        src_canvas = canvas[src_y1:src_y2, src_x1:src_x2]
        src_mask   = mask[src_y1:src_y2, src_x1:src_x2].astype(bool)

        # Sadece daire içindeki pikselleri blend et
        blended = roi.copy()
        blended[src_mask] = (
            HORIZON_ALPHA       * src_canvas[src_mask].astype(np.float32) +
            (1 - HORIZON_ALPHA) * roi[src_mask].astype(np.float32)
        ).astype(np.uint8)

        frame[dst_y1:dst_y2, dst_x1:dst_x2] = blended

    # Ufuk çizgisi (katı, görünür)
    p1 = _rotate_point(cx, cy, cx - radius, cy + pitch_offset, -roll_deg)
    p2 = _rotate_point(cx, cy, cx + radius, cy + pitch_offset, -roll_deg)
    cv2.line(frame, p1, p2, (255, 255, 0), 2, cv2.LINE_AA)



# ---------------------------------------------------------------------------
# Pitch Ladder
# ---------------------------------------------------------------------------

def _draw_pitch_ladder(frame, cx, cy, radius, roll_deg, pitch_deg):
    """Draw pitch reference lines every 10 degrees."""
    px_per_deg = radius / 25.0
    bar_half_w = int(radius * 0.35)

    for deg in range(-30, 31, 10):
        if deg == 0:
            continue
        offset = int((deg - pitch_deg) * px_per_deg)
        if abs(offset) > radius * 0.9:
            continue

        # Bar centre in horizon-relative coords, then rotate by roll
        bx, by = cx, cy - offset  # before roll
        p1 = _rotate_point(cx, cy, bx - bar_half_w, by, -roll_deg)
        p2 = _rotate_point(cx, cy, bx + bar_half_w, by, -roll_deg)

        color = (200, 200, 200)
        cv2.line(frame, p1, p2, color, 1, cv2.LINE_AA)

        # Label at right end
        lbl = f"{abs(deg)}"
        label_pos = _rotate_point(cx, cy, bx + bar_half_w + 4, by + 4, -roll_deg)
        # Only draw label if inside the circle
        if math.hypot(label_pos[0] - cx, label_pos[1] - cy) < radius * 0.92:
            _put_text(frame, lbl, label_pos, scale=0.38, color=(220, 220, 220),
                      thickness=1, shadow=True)


# ---------------------------------------------------------------------------
# Roll Arc Indicator
# ---------------------------------------------------------------------------

def _draw_roll_arc(frame, cx, cy, radius, roll_deg):
    """Draw a fixed roll-tick arc and a moving roll pointer."""
    arc_r = radius + 10

    # Fixed tick marks at 0, ±10, ±20, ±30, ±45, ±60 degrees
    ticks = [(-60, 8), (-45, 8), (-30, 10), (-20, 7), (-10, 7),
             (0, 14),
             (10, 7), (20, 7), (30, 10), (45, 8), (60, 8)]

    for angle, tick_len in ticks:
        # angle=0 is straight up → convert: arc angle = -90 + angle
        a_rad = math.radians(-90 + angle)
        ox = int(cx + arc_r * math.cos(a_rad))
        oy = int(cy + arc_r * math.sin(a_rad))
        ix = int(cx + (arc_r - tick_len) * math.cos(a_rad))
        iy = int(cy + (arc_r - tick_len) * math.sin(a_rad))
        color = (255, 255, 0) if angle == 0 else (180, 180, 180)
        cv2.line(frame, (ox, oy), (ix, iy), color, 1, cv2.LINE_AA)

    # Roll pointer triangle (moves with roll)
    ptr_a = math.radians(-90 - roll_deg)
    tip_r = arc_r - 2
    tx = int(cx + tip_r * math.cos(ptr_a))
    ty = int(cy + tip_r * math.sin(ptr_a))
    # Small triangle pointing inward
    side = math.radians(-90 - roll_deg + 90)
    hw = 6
    lx = int(tx + hw * math.cos(side))
    ly = int(ty + hw * math.sin(side))
    rx = int(tx - hw * math.cos(side))
    ry = int(ty - hw * math.sin(side))
    inner_r = arc_r - 18
    bx = int(cx + inner_r * math.cos(ptr_a))
    by = int(cy + inner_r * math.sin(ptr_a))
    pts = np.array([[tx, ty], [lx, ly], [bx, by], [rx, ry]], np.int32)
    cv2.fillPoly(frame, [pts], (0, 220, 255))
    cv2.polylines(frame, [pts], True, (255, 255, 255), 1, cv2.LINE_AA)


# ---------------------------------------------------------------------------
# Compass Tape
# ---------------------------------------------------------------------------

def _draw_compass_tape(frame, cx, bottom_y, heading_deg, width=320):
    """Draw a horizontal compass tape centred on heading."""
    tape_h = 30
    y1, y2 = bottom_y - tape_h, bottom_y
    x1, x2 = cx - width // 2, cx + width // 2

    _alpha_rect(frame, x1, y1, x2, y2, (0, 0, 0), alpha=0.55)

    deg_per_px = 0.25  # degrees per pixel — tune for zoom level
    px_per_deg = 1.0 / deg_per_px

    # Draw labels and ticks
    for delta in range(-60, 61, 5):
        deg = (heading_deg + delta) % 360
        px_off = int(delta * px_per_deg)
        tx = cx + px_off
        if tx < x1 or tx > x2:
            continue

        tick_len = 10 if (int(deg) % 10 == 0) else 5
        cv2.line(frame, (tx, y1), (tx, y1 + tick_len), (200, 200, 200), 1)

        if int(deg) % 30 == 0:
            # Cardinal / label
            cardinals = {0: 'N', 45: 'NE', 90: 'E', 135: 'SE',
                         180: 'S', 225: 'SW', 270: 'W', 315: 'NW'}
            lbl = cardinals.get(int(deg), str(int(deg)))
            _put_text(frame, lbl, (tx - 8, y1 + 24), scale=0.42,
                      color=(255, 255, 0) if lbl in ('N', 'S', 'E', 'W') else (220, 220, 220),
                      thickness=1, shadow=True)

    # Centre marker (current heading arrow)
    cv2.line(frame, (cx, y1), (cx, y2), (0, 255, 255), 2)
    # Heading readout above tape
    hdg_txt = f"{int(heading_deg):03d}"
    _put_text(frame, hdg_txt, (cx - 18, y1 - 4), scale=0.55,
              color=(0, 255, 255), thickness=1, shadow=True)


# ---------------------------------------------------------------------------
# Telemetry Panels
# ---------------------------------------------------------------------------

def _draw_telemetry_panels(frame, depth, voltage, gps_fix):
    """
    Minimal HUD panels — sadece metinsel veri.
    Grafiksel göstergeler (ufuk, roll yayı, pusula) ayrıca çiziliyor.
    Tüm paneller hafif şeffaf arka plana sahip, görüntüyü kapatmaz.
    """
    fh, fw = frame.shape[:2]

    # ---- Sol sütun: Batarya / GPS / Derinlik ----
    px, py = 8, 8          # sol üst köşe başlangıcı
    line_h = 22            # satır yüksekliği
    panel_alpha = 0.30     # çok hafif arka plan
    panel_w = 145          # panel genişliği

    # Panel arka planı — tek bir hafif dikdörtgen (3 satır)
    _alpha_rect(frame, px - 2, py - 2,
                px + panel_w, py + line_h * 3 + 4,
                (0, 0, 0), panel_alpha)

    # --- Satır 1: Batarya ---
    if voltage <= 0:
        bat_color = (160, 160, 160)
        bat_txt = "BAT  -- V"
    elif voltage >= 14.0:
        bat_color = (60, 220, 60)
        bat_txt = f"BAT  {voltage:.1f} V"
    elif voltage >= 10.0:
        bat_color = (0, 190, 255)
        bat_txt = f"BAT  {voltage:.1f} V"
    else:
        bat_color = (60, 80, 255)
        bat_txt = f"BAT  {voltage:.1f} V"

    _put_text(frame, bat_txt, (px, py + 14), scale=0.50,
              color=bat_color, thickness=1, shadow=True)

    # Batarya doluluk çubuğu — küçük, satır içi
    if voltage > 0:
        max_v, min_v = 16.8, 12.0
        pct = max(0.0, min(1.0, (voltage - min_v) / (max_v - min_v)))
        bar_x = px + 95
        bar_y1, bar_y2 = py + 4, py + 12
        bar_total_w = 44
        bar_fill_w = int(bar_total_w * pct)
        cv2.rectangle(frame, (bar_x, bar_y1), (bar_x + bar_total_w, bar_y2),
                      (60, 60, 60), -1)
        cv2.rectangle(frame, (bar_x, bar_y1), (bar_x + bar_fill_w, bar_y2),
                      bat_color, -1)
        cv2.rectangle(frame, (bar_x, bar_y1), (bar_x + bar_total_w, bar_y2),
                      (140, 140, 140), 1)

    # --- Satır 2: GPS Fix ---
    fix_labels = {0: "NO FIX", 1: "NO FIX", 2: "2D FIX",
                  3: "3D FIX", 4: "3D+DGPS", 5: "RTK FLOAT", 6: "RTK FIXED"}
    fix_colors = {0: (60, 60, 220), 1: (60, 60, 220), 2: (0, 150, 255),
                  3: (60, 210, 60), 4: (60, 210, 60),
                  5: (0, 220, 180),  6: (0, 230, 230)}
    fix_label = fix_labels.get(gps_fix, "UNKN")
    gps_color  = fix_colors.get(gps_fix, (160, 160, 160))

    # Küçük durum noktası
    dot_y = py + line_h + 10
    cv2.circle(frame, (px + 5, dot_y - 4), 5, gps_color, -1, cv2.LINE_AA)
    _put_text(frame, f"GPS  {fix_label}", (px + 14, dot_y), scale=0.50,
              color=gps_color, thickness=1, shadow=True)

    # --- Satır 3: Derinlik ---
    dep_y = py + line_h * 2 + 8
    _put_text(frame, f"DEP  {depth:.2f} m", (px, dep_y), scale=0.50,
              color=(160, 230, 255), thickness=1, shadow=True)

    # ---- Sağ taraf: Dikey derinlik gauge ----
    # Sade, ince, şeffaf — sadece çerçeve + doluluk çubuğu
    gx = fw - 22
    gy1, gy2 = fh // 4, 3 * fh // 4
    g_h = gy2 - gy1
    max_depth = 10.0
    dep_pct = max(0.0, min(1.0, depth / max_depth))
    fill_h = int(g_h * dep_pct)

    # Arka plan (çok hafif)
    _alpha_rect(frame, gx - 4, gy1 - 2, gx + 14, gy2 + 2, (0, 0, 0), 0.25)
    # Doluluk (yüzeyden aşağıya doğru)
    if fill_h > 0:
        cv2.rectangle(frame, (gx - 2, gy1), (gx + 12, gy1 + fill_h),
                      (160, 220, 255), -1)
    # Çerçeve
    cv2.rectangle(frame, (gx - 4, gy1 - 2), (gx + 14, gy2 + 2),
                  (140, 140, 140), 1)
    # Etiket — sadece sayı
    _put_text(frame, f"{depth:.1f}", (gx - 8, gy2 + 14), scale=0.38,
              color=(160, 230, 255), thickness=1, shadow=True)
    # "0m" üstte
    _put_text(frame, "0m", (gx - 4, gy1 - 6), scale=0.35,
              color=(140, 140, 140), thickness=1, shadow=False)



# ---------------------------------------------------------------------------
# Main HUD entry point
# ---------------------------------------------------------------------------

def draw_hud(frame: np.ndarray, hud_data: HudData) -> np.ndarray:
    """
    Composite a Mission Planner-style HUD onto `frame` (BGR, in-place).
    Returns the modified frame.
    """
    roll, pitch, yaw, depth, voltage, gps_fix = hud_data.snapshot()

    fh, fw = frame.shape[:2]
    cx, cy = fw // 2, fh // 2

    # Horizon circle radius: ~35% of the shorter dimension
    radius = int(min(fw, fh) * 0.33)

    # 1. Artificial horizon (sky/ground fill inside circle)
    _draw_artificial_horizon(frame, cx, cy, radius, roll, pitch)

    # 2. Pitch ladder lines
    _draw_pitch_ladder(frame, cx, cy, radius, roll, pitch)

    # 3. Horizon circle border
    cv2.circle(frame, (cx, cy), radius, (200, 200, 200), 1, cv2.LINE_AA)

    # 4. Centre crosshair
    ch = 12
    cv2.line(frame, (cx - ch, cy), (cx - 4, cy), (0, 255, 255), 2, cv2.LINE_AA)
    cv2.line(frame, (cx + 4, cy), (cx + ch, cy), (0, 255, 255), 2, cv2.LINE_AA)
    cv2.line(frame, (cx, cy - ch), (cx, cy - 4), (0, 255, 255), 2, cv2.LINE_AA)
    cv2.line(frame, (cx, cy + 4), (cx, cy + ch), (0, 255, 255), 2, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), 3, (0, 255, 255), -1, cv2.LINE_AA)

    # 5. Roll arc
    _draw_roll_arc(frame, cx, cy, radius, roll)

    # 6. Compass tape
    compass_y = fh - 10
    _draw_compass_tape(frame, cx, compass_y, yaw)

    # 7. Telemetry panels (only data without graphical representation)
    _draw_telemetry_panels(frame, depth, voltage, gps_fix)

    return frame


# ---------------------------------------------------------------------------
# VideoWorker
# ---------------------------------------------------------------------------

class VideoWorker(QThread):
    frame_ready = pyqtSignal(QImage)

    def __init__(self, camera_source=0, hud_data: HudData = None, parent=None):
        super().__init__(parent)
        self.camera_source = camera_source
        self._is_running = True
        self.cap = None
        self.hud_data = hud_data  # None means no HUD

    def run(self):
        self.cap = cv2.VideoCapture(self.camera_source, cv2.CAP_DSHOW)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            print("Kamera açılmadı.")
            self.cap.release()  
            return

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        while self._is_running:
            ret, frame = self.cap.read()
            if not ret:
                # Frame gelmedi — CPU spin önleme
                import time as _time
                _time.sleep(0.01)
                continue

            h, w, _ = frame.shape
            if h > 480:
                frame = frame[0:480, 0:w]

            if self.hud_data is not None:
                frame = draw_hud(frame, self.hud_data)

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            fh2, fw2, ch = rgb_frame.shape
            bytes_per_line = ch * fw2

            qt_image = QImage(
                rgb_frame.data, fw2, fh2, bytes_per_line, QImage.Format_RGB888
            ).copy()
            self.frame_ready.emit(qt_image)

        self.cap.release()

    def stop(self):
        self._is_running = False
        self.wait(3000)   # En fazla 3 saniye bekle — deadlock önleme