import cv2
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage

class VideoWorker(QThread):
    frame_ready = pyqtSignal(QImage)

    def __init__(self, camera_source=0, parent=None):
        super().__init__(parent)
        self.camera_source = camera_source
        self._is_running = True

    def run(self):
        cap = cv2.VideoCapture(self.camera_source)
        while self._is_running:
            ret, frame = cap.read()
            if ret:
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_frame.shape
                bytes_per_line = ch * w
                qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.frame_ready.emit(qt_image)
            self.msleep(30) # ~30 FPS sınırlaması
        cap.release()

    def stop(self):
        self._is_running = False
        self.wait()