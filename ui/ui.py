import sys
import signal
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem
from PyQt5.QtCore import QTimer
from PyQt5 import uic
from PyQt5.QtGui import QImage, QPixmap
from functools import partial

app = None

from libs.widgets import *
from ros_manager import RosManager

form_class = uic.loadUiType("./forms/mainwindow.ui")[0]


def signal_handler(sig, frame):
    QApplication.quit()


class MyApp(QMainWindow, form_class):
    def __init__(self, type):
        super().__init__()
        self.setupUi(self)
        self.RM = RosManager(type)
        self.type = type

        self.set_values()
        self.set_widgets()
        self.set_timers()

    def set_values(self):
        self.sig_in = False
        if self.type == 'target':
            self.ego_state_string = ['정상', '비상 상황 인지', '차선 변경 수락', '차선 변경 거절']
            self.target_state_string = ['정상', '비상 상황 감지', '우측 합류 요청', '좌측 합류 요청']
        else:
            self.ego_state_string = ['정상', '비상 상황 감지', '좌측 차선 변경', '우측 차선 변경']
            self.target_state_string = ['정상', '비상 상황 인지', '차선 변경 수락', '차선 변경 거절']
            
        self.state_buttons = [self.normalButton, self.emergencyButton, self.leftButton, self.rightButton]
        self.prev_state = 0

    def set_widgets(self):
        self.rviz_widget = RvizWidget(self)
        self.initUI()
    
    def set_timers(self):
        self.timer = QTimer(self)   
        self.timer.timeout.connect(self.updateUI)
        self.timer.start(100)

        self.user_input_timer = QTimer(self)
        self.user_input_timer.timeout.connect(self.state_triggered)
        self.user_input_timer.start(500)


    def updateUI(self):
        self.table_update(self.RM.communication_performance)
        self.state_update(self.RM.states)
        self.image_update(self.RM.compressed_image)
        
    def table_update(self, communication_performance):
        self.tableWidget.setItem(-1, 1, QTableWidgetItem(communication_performance['comulative_time']))
        self.tableWidget.setItem(0, 1, QTableWidgetItem(communication_performance['distance']))
        self.tableWidget.setItem(1, 1, QTableWidgetItem(communication_performance['rtt']))
        self.tableWidget.setItem(2, 1, QTableWidgetItem(communication_performance['speed']))
        self.tableWidget.setItem(3, 1, QTableWidgetItem(communication_performance['packet_size']))
        self.tableWidget.setItem(4, 1, QTableWidgetItem(communication_performance['packet_rate']))

    def state_update(self, states):
        ego_state = int(states['ego'])
        target_state = int(states['target'])
        self.egoLabel.setText(self.ego_state_string[ego_state])
        
        self.targetLabel.setText(self.target_state_string[target_state])
        for i, state_button in enumerate(self.state_buttons):
            if i == ego_state:
                state_button.setStyleSheet("QPushButton {background-color: #0066ff;color: white;}")
            else:
                state_button.setStyleSheet("QPushButton {background-color: #eeeeec; color: black;}")
        

    def image_update(self, compressed_image):
        if compressed_image is not None:
            height, width, channel = compressed_image.shape
            bytes_per_line = 3 * width
            q_img = QImage(compressed_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img)
            scaled_pixmap = pixmap.scaled(self.cameraLabel.size(), aspectRatioMode=True)            
            self.cameraLabel.setPixmap(scaled_pixmap)

    def click_state(self, value):
        self.RM.user_value = value
        self.check_timer()

    def check_timer(self):
        if not self.sig_in:
            self.sig_in = True
            self.user_input_timer.start(100)
            QTimer.singleShot(5000, self.stop_user_input_timer)
        else:
            self.stop_user_input_timer
    
    def stop_user_input_timer(self):
        self.sig_in = False
        self.RM.user_value = 0
        self.cameraLabel.setStyleSheet("QLabel {background-color: white;}")
        self.user_input_timer.stop()
        self.RM.publish()

    def state_triggered(self):
        self.RM.publish()
        if self.RM.user_value == 1:
            self.cameraLabel.setStyleSheet("QLabel {background-color: red;}")
            
        
    def initUI(self):
        self.set_conntection()
        self.rvizLayout.addWidget(self.rviz_widget)

    def set_conntection(self):
        for i in range(0,4):
            self.state_buttons[i].clicked.connect(partial(self.click_state, int(i)))

def main():
    app = QApplication(sys.argv)
    type = sys.argv[1]
    ex = MyApp(type)
    ex.show()
    signal.signal(signal.SIGINT, signal_handler)
    app.exec_()

if __name__ == '__main__':
    main()
