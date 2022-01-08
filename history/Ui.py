import PyQt5
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import cv2

class ui(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setObjectName("MainWindow")
        self.resize(800, 600)
        self.setStyleSheet("")

        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.capture = QtWidgets.QPushbutton(self.centralwidget)
        self.capture.setGeometry(QtCore.QRect(160, 430, 111, 61))
        self.capture.setStyleSheet("background: #555556;color: #FFFFFF;border-radius:10px")
        self.capture.setObjectName("capture")
        self.grasp = QtWidgets.QPushbutton(self.centralwidget)
        self.grasp.setGeometry(QtCore.QRect(360, 430, 111, 61))
        self.grasp.setStyleSheet("background: #555556;color: #FFFFFF;border-radius:10px")
        self.grasp.setObjectName("grasp")
        self.put = QtWidgets.QPushbutton(self.centralwidget)
        self.put.setGeometry(QtCore.QRect(560, 430, 101, 61))
        self.put.setStyleSheet("background: #555556;color: #FFFFFF;border-radius:10px")
        self.put.setObjectName("put")
        self.change_pic_pos = QtWidgets.QPushbutton(self.centralwidget)
        self.change_pic_pos.setGeometry(QtCore.QRect(210, 30, 141, 71))
        self.change_pic_pos.setStyleSheet("background: #555556;color: #FFFFFF;border-radius:10px")
        self.change_pic_pos.setObjectName("change_pic_pos")
        self.set_put_pos = QtWidgets.QPushbutton(self.centralwidget)
        self.set_put_pos.setGeometry(QtCore.QRect(430, 30, 131, 71))
        self.set_put_pos.setStyleSheet("background: #555556;color: #FFFFFF;border-radius:10px")
        self.set_put_pos.setObjectName("set_put_pos")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(240, 180, 291, 171))
        self.label.setObjectName("label")
        self.label.setStyleSheet("background:white")

        self.retranslateUi()
        self.connect_event()

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.capture.setText(_translate("MainWindow", "拍照"))
        self.grasp.setText(_translate("MainWindow", "抓取"))
        self.put.setText(_translate("MainWindow", "放置"))
        self.change_pic_pos.setText(_translate("MainWindow", "切换拍照姿态"))
        self.set_put_pos.setText(_translate("MainWindow", "设置放置姿态"))

    def connect_event(self):
        self.capture.clicked.connect(self.printself)
        self.grasp.clicked.connect(self.printself)
        self.put.clicked.connect(self.printself)
        self.change_pic_pos.clicked.connect(self.printself)
        self.set_put_pos.clicked.connect(self.printself)

    def printself(self):
        print("pushed!")
    def keyPressEvent(self, event):#重新实现了keyPressEvent()事件处理器。
        #按住键盘事件
        #这个事件是PyQt自带的自动运行的，当我修改后，其内容也会自动调用
        if event.key() == QtCore.Qt.Key_Escape:#当我们按住键盘是esc按键时
            print("esc")
            self.close()#关闭程序

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ui()
    ex.show()
    sys.exit(app.exec_())
