import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

# ROS2 노드: 차량 통제 명령을 구독해서 UI에 반영
class VehicleControlDisplay(Node):
    def __init__(self, qt_callback):
        super().__init__('vehicle_control_display')
        self.qt_callback = qt_callback
        # /vehicle/control/cmd 토픽 구독
        self.create_subscription(String, '/vehicle/control/cmd', self.cmd_callback, 10)

    def cmd_callback(self, msg):
        # 토픽 메시지 수신 시 UI 업데이트 콜백 호출
        self.get_logger().info(f'차량통제 명령 수신: {msg.data}')
        self.qt_callback(msg.data)

# PyQt5 윈도우: 차량 통제 메시지를 화면에 크게 표시
class DisplayWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('TurtleBot4 차량통제 알림')
        self.setWindowState(Qt.WindowFullScreen)  # 전체화면

        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)

        self.label = QLabel('')
        self.label.setAlignment(Qt.AlignCenter)
        # 1080p 해상도에서 가로 절반 잘림 방지: 폰트 크기 180~200이 적당
        font = QFont('Arial', 180)
        self.label.setFont(font)
        self.label.setStyleSheet("color: red; font-weight: bold;")
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)

    def show_alert(self, cmd):
        # 사이렌 이모티콘 없이, 실제 화살표만 표시
        self.label.setText('차량 통제 중\n⬅️ ⬅️ ⬅️   ➡️ ➡️ ➡️')

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    window = DisplayWindow()
    window.show()

    def update_ui(cmd):
        window.show_alert(cmd)

    node = VehicleControlDisplay(update_ui)

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))
    timer.start(50)

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
