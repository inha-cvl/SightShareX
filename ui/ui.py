#!/usr/bin/env python3

import sys
import signal
import setproctitle
setproctitle.setproctitle("ui")
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem
from PyQt5.QtCore import QTimer
from PyQt5 import uic
from PyQt5.QtGui import QImage, QPixmap
from functools import partial

app = None

from libs.widgets import *
from ros_manager import RosManager

# type에 따라 .ui 파일 선택
def load_ui_by_type(type):
    if type == 'target':
        return uic.loadUiType("./forms/mainwindow_target.ui")[0]
    else:  # 기본적으로 'ego'로 처리
        return uic.loadUiType("./forms/mainwindow_ego.ui")[0]



def signal_handler(sig, frame):
    QApplication.quit()


class MyApp(QMainWindow):
    def __init__(self, type):
        form_class = load_ui_by_type(type)
        super(MyApp, self).__init__()
        self.ui = form_class()
        self.ui.setupUi(self)  # UI 설정
        self.RM = RosManager(type)
        self.type = type

        self.set_values()
        self.set_widgets()
        self.set_timers()

    def set_values(self):
        self.sig_in = False
        if self.type == 'target':
            self.ego_state_string = ['정상', '차선 변경 수락', '차선 변경 거절', '', '차량 사고 인지','보행자 사고 인지', '고장 차량 인지', '낙하물 인지' ]
            self.target_state_string = ['정상', '우측 합류 요청', '좌측 합류 요청', '직진 초기화','차량 사고 감지','보행자 사고 감지', '고장 차량 감지', '낙하물 감지']
        else:
            self.ego_state_string = ['정상', '좌측 차선 변경', '우측 차선 변경', '직진 초기화', '차량 사고 감지','보행자 사고 감지', '고장 차량 감지', '낙하물 감지' ]
            self.target_state_string = ['정상', '차선 변경 수락', '차선 변경 거절',  '', '차량 사고 인지','보행자 사고 인지', '고장 차량 인지', '낙하물 인지' ]
            
        self.state_buttons = [self.ui.normalButton, self.ui.leftButton, self.ui.rightButton, self.ui.straightButton, self.ui.carAccidentButton, self.ui.pedestrianAccidentButton, self.ui.carBrokenButton, self.ui.fallenObjectButton]
        self.simulator_buttons = [self.ui.stopButton, self.ui.startButton,  self.ui.initializeButton]
        self.prev_state = 0

    def set_widgets(self):
        self.rviz_widget = RvizWidget(self, self.type)
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
        self.ui.tableWidget.setItem(-1, 1, QTableWidgetItem(communication_performance['comulative_time']))
        self.ui.tableWidget.setItem(0, 1, QTableWidgetItem(communication_performance['distance']))
        self.ui.tableWidget.setItem(1, 1, QTableWidgetItem(communication_performance['rtt']))
        self.ui.tableWidget.setItem(2, 1, QTableWidgetItem(communication_performance['speed']))
        self.ui.tableWidget.setItem(3, 1, QTableWidgetItem(communication_performance['packet_size']))
        self.ui.tableWidget.setItem(4, 1, QTableWidgetItem(communication_performance['packet_rate']))

    def state_update(self, states):
        ego_state = int(states['ego'])
        target_state = int(states['target'])
        self.ui.egoLabel.setText(self.ego_state_string[ego_state])
        
        self.ui.targetLabel.setText(self.target_state_string[target_state])
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
            scaled_pixmap = pixmap.scaled(self.ui.cameraLabel.size(), aspectRatioMode=True)            
            self.ui.cameraLabel.setPixmap(scaled_pixmap)

    def click_state(self, value):
        self.RM.user_value = value
        self.check_timer()
    
    def click_simulator(self, value):
        self.RM.simulator_value = value
        self.check_timer()

    def check_timer(self):
        if not self.sig_in:
            self.sig_in = True
            self.user_input_timer.start(100)
            QTimer.singleShot(1000, self.stop_user_input_timer)
        else:
            self.stop_user_input_timer
    
    def stop_user_input_timer(self):
        self.sig_in = False
        self.RM.user_value = 0
        self.ui.cameraLabel.setStyleSheet("QLabel {background-color: white;}")
        self.user_input_timer.stop()
        self.RM.publish()

    def state_triggered(self):
        self.RM.publish()
        if self.RM.user_value in [4,5,6,7]:
            self.ui.cameraLabel.setStyleSheet("QLabel {background-color: red;}")
            
        
    def initUI(self):
        self.set_conntection()
        self.ui.rvizLayout.addWidget(self.rviz_widget)

    def set_conntection(self):
        for i in range(0,8):
            self.state_buttons[i].clicked.connect(partial(self.click_state, int(i)))
        for i in range(0,3):
            self.simulator_buttons[i].clicked.connect(partial(self.click_simulator, int(i)))

def main():
    app = QApplication(sys.argv)
    type = sys.argv[1]
    ex = MyApp(type)
    ex.show()
    signal.signal(signal.SIGINT, signal_handler)
    app.exec_()

if __name__ == '__main__':
    main()
