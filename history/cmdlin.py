# region import basic library
from argparse import ArgumentDefaultsHelpFormatter
import math
import os
import queue
import sys
import cmd
import time
import numpy as np
# endregion

#sys.path.append("/home/yons/grasp_ws/JAKA_SDK/python3-refresh/")
#os.system("export LD_LIBRARY_PATH=/home/yons/grasp_ws/JAKA_SDK/python3-refresh/")
# region import JAKA
from GraspInit import *
from jaka import JAKA
from poseTransForm.poseTransform import *
# endregion

import  threading
from multiprocessing import Queue
from time import sleep


IPaddress = "192.168.15.106"

class Query(threading.Thread):
  # region 成员参数
    intro = '机器人智能抓取系统，输入 help 或者?查看帮助。\n'  # 命令行欢迎
    prompt = 'cmd>'      # 命令行提示符
    depth_image = None      # 深度图像
    color_image = None       # 彩色图像
    toolCoordinate = None # 根据坐标系 可以用APP设置
    gg = None  # grasp group 用于存储算法输出结果
    homePose = None      # home 位姿 用于存储home
    basketPose = None    # basket 位姿 basket
    def  __init__(self ,qget):
        threading.Thread.__init__(self)
  # endregion
  # region 成员方法
      # def do_change_pos(self,arg):
      #   tcp = JAKA(IPaddress)
      #   self.homePose = tcp.getpos6DoF()
      #   tcp.robot.logout()
        self.ai_get = qget

    def run(self):

      self.do_set_basket("")
      self.do_set_home('')
      self.gripper_init()
      while True:
        sleep(0.5)
        if self.ai_get.full():
          cmd = self.ai_get.get()
          if cmd == 'photo':
            self.do_go_home("")
            print('go home')
            self.do_capture('')
            self.do_show_image('')

            # self.do_go_home("80")
          elif cmd == "put":
            self.do_gripper_open("")
            self.do_grasp("20")
            time.sleep(1)
            self.do_gripper_close("10")
            time.sleep(3)
            self.do_go_basket("")


            self.do_gripper_open("")
            sleep(2)
            tcp=JAKA(IPaddress)
            tcp.joint_move(self.basketPose,120)
            tcp.robot.logout()

            self.do_go_home('')
          elif cmd == 'init':
            self.robot_init()

    def gripper_init(self):
      tcp = JAKA(IPaddress)
      tcp.robot.set_analog_output(iotype = 2,index = 0,value = 1)

    def robot_init(self):
      tcp = JAKA(IPaddress)
      tcp.robot.set_analog_output(iotype = 2,index = 1,value = 1000)
      self.do_go_home('')

    def do_gripper_open(self,arg):
      if(not arg):
        arg = 1000
      tcp = JAKA(IPaddress)
      tcp.robot.set_analog_output(iotype = 2,index = 0,value = 0)#
      tcp.robot.set_analog_output(iotype = 2,index = 1,value = 20)#
      tcp.robot.set_analog_output(iotype = 2,index = 3,value = int(arg))#
      tcp.robot.logout()

    def do_gripper_close(self,arg):
      if(not arg):
        arg = 400
      tcp = JAKA(IPaddress)
      tcp.robot.set_analog_output(iotype = 2,index = 0,value = 0)#
      tcp.robot.set_analog_output(iotype = 2,index = 1,value = 50)#
      tcp.robot.set_analog_output(iotype = 2,index = 3,value = int(arg))#
      tcp.robot.logout()

    def do_set_home(self,arg):
      tcp = JAKA(IPaddress)
      self.homePose = [1.7850742322049737, 1.6209502754266347, -1.4273571101986477, 1.323693566851979, 1.575793716891346, -0.6591922776628348]
      print(tcp.getjoints())
      # [math.radians(118.651),math.radians(103.708),math.radians(-107.928),math.radians(94.547),math.radians(89.891),math.radians(33.421)]
      tcp.robot.logout()

    def do_go_home(self,arg):
      tcp = JAKA(IPaddress)
      if not arg:
        arg = 40
      tcp.joint_move(self.homePose,int(arg))
      tcp.robot.logout()

    def do_set_basket(self,arg):
      tcp = JAKA(IPaddress)

      self.basketPose = [math.radians(193.1),math.radians(116.764),math.radians(-105.233),math.radians(123.3),math.radians(85.058),math.radians(46.278)] #tcp.getpos6DoF()
      tcp.robot.logout()

    def do_go_basket(self,arg):
      tcp = JAKA(IPaddress)
      if not arg:
        arg = 60

      now_tcp = tcp.getposXYZ()
      now_rpy = tcp.getposRPY()
      for i in now_rpy:
          now_tcp.append(i)
      now_tcp[2] += 150
      tcp.liner_move(now_tcp, 120)
      tcp.joint_move(self.basketPose,120)
      # print(tcp.robot.linear_/move(end_pos = [0,0,-100,0,0,0], move_mode = True, is_block = True, speed = 120))
      put_down  = [math.radians(194.267),math.radians(85.313),math.radians(-97.208),math.radians(100.929),math.radians(91.487),math.radians(52.251)]
      tcp.joint_move(put_down,100)
      # self.do_gripper_open("")
      # sleep(2)
      # tcp.joint_move(self.basketPose,120)
      tcp.robot.logout()

    def do_grasp(self,arg):
      tcp = JAKA(IPaddress)
      curPos = tcp.getpos6DoF()
      print("cur pos:",curPos)
      tcp.robot.logout()
      dkr = GG_2_Liner_move(self.gg,curPos)
      print("gg: ",self.gg)
      print("tar pos:",dkr)
      tcp = JAKA(IPaddress)
      if not arg:
        arg = 120
      # before_pos = dkr
      # before_pos[2] = dkr[2] + 50
      # tcp.liner_move(before_pos,120)
      # print('point 1')
      # time.sleep(2)
      # dkr[2] = dkr[2] - 50
      print(dkr)
      tcp.liner_move(dkr,arg)
      print('point 1')
      time.sleep(1)
      tcp.moveInFlangeCoordinate([0,0, self.gg.depths*1000,0,0,0])

      tcp.robot.logout()

    def do_capture(self,arg):
      time.sleep(1)
      while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 1280 x 720 depth image
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()
        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue
        else:
          self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
          self.color_image = np.asanyarray(color_frame.get_data())
          break

    def do_show_image(self,arg):
      calcucate_cmd = False
      while True:
        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((self.depth_image,self.depth_image,self.depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, self.color_image)

        # Render images:
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))

        # cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        # cv2.imshow('Align Example', images)
        # key = cv2.waitKey(0)

        # # Press esc or 'q' to close the image window
        # if key & 0xFF == ord('q') or key == 27:
        #     cv2.destroyAllWindows()
        #     break
        # elif key & 0xFF == ord('p'):

        if calcucate_cmd == False:
            print("AI Calculating ...")
            if(arg == ''):
              arg = '1'
            self.gg = demo_inner(self.color_image,self.depth_image , './',int(arg))
            print("SAVE: ",self.gg)
            calcucate_cmd = True
            time.sleep(5)
            break

    def do_exit(self, _):
        '退出'
        exit(0)
  # endregion

from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
import sys

class Window(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.ui_put = Queue(1)
        self.ai = Query(self.ui_put)
        self.ai.daemon = True
        self.ai.start()

    def robot_init(self):
      cmd = 'init'
      self.ui_put.put(cmd)

    def robot_photo(self):
      cmd = 'photo'
      self.ui_put.put(cmd)
      self.put_pushbutton.setStyleSheet("QPushButton{color:#D80C1E}"
                                       "QPushButton:hover{background-color:#EEEEEE}"
                                       "QPushButton{text-align:right}"
                                    #    "QPushButton:hover{color:#EEEEEE}"
                                       "QPushButton{background-color:#FFFFFF}"
                                       "QPushButton{border:2px solid #D80C1E}"
                                       "QPushButton{border-radius:10px}"
                                       "QPushButton{padding:2px 4px}")
      self.put_pushbutton.setEnabled(True)
      QApplication.processEvents()

    def rotbot_put(self):
      self.put_pushbutton.setStyleSheet("QPushButton{color:#EEEEEE}"
                                        "QPushButton:hover{background-color:#EEEEEE}"
                                        "QPushButton{text-align:right}"
                                      #    "QPushButton:hover{color:#EEEEEE}"
                                        "QPushButton{background-color:#FFFFFF}"
                                        "QPushButton{border:2px solid #D80C1E}"
                                        "QPushButton{border-radius:10px}"
                                        "QPushButton{padding:2px 4px}")
      self.put_pushbutton.setEnabled(False)
      QApplication.processEvents()
      cmd = 'put'
      self.ui_put.put(cmd)

    def initUI(self):
        self.resize(599, 500)
        self.setWindowTitle('任意抓取应用')
        # self.setStyleSheet('QWidget{background-color:#D80C1E}')
        self.setWindowFlags(QtCore.Qt.WindowCloseButtonHint)
        # self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        # self.setStyleSheet("backgroud-color:#FFFFFF")
        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.photo_pushbutton = QtWidgets.QPushButton(self.centralwidget)
        self.photo_pushbutton.setGeometry(QtCore.QRect(122, 170, 356, 80))
        font = QtGui.QFont()
        font.setPointSize(19)
        font.setBold(True)
        font.setWeight(75)
        self.photo_pushbutton.setFont(font)
        self.photo_pushbutton.setStyleSheet("QPushButton{color:#D80C1E}"
                                       "QPushButton{text-align:right}"
                                       "QPushButton:hover{background-color:#EEEEEE}"
                                    #    "QPushButton:hover{color:#EEEEEE}"
                                       "QPushButton{background-color:#FFFFFF}"
                                       "QPushButton{border:2px solid #D80C1E}"
                                       "QPushButton{border-radius:10px}"
                                       "QPushButton{padding:2px 4px}")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("./img/图层 1 拷贝.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.photo_pushbutton.setIcon(icon)
        self.photo_pushbutton.setIconSize(QtCore.QSize(40, 40))
        self.photo_pushbutton.setObjectName("photo_pushbutton")

        self.put_pushbutton = QtWidgets.QPushButton(self.centralwidget)
        self.put_pushbutton.setGeometry(QtCore.QRect(122, 270, 356, 80))
        font = QtGui.QFont()
        font.setPointSize(19)
        font.setBold(True)
        font.setWeight(75)
        self.put_pushbutton.setFont(font)
        self.put_pushbutton.setContextMenuPolicy(QtCore.Qt.PreventContextMenu)
        self.put_pushbutton.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.put_pushbutton.setStyleSheet("QPushButton{color:#EEEEEE}"
                                       "QPushButton{text-align:right}"
                                       "QPushButton:hover{background-color:#EEEEEE}"
                                    #    "QPushButton:hover{color:#EEEEEE}"
                                       "QPushButton{background-color:#FFFFFF}"
                                       "QPushButton{border:2px solid #D80C1E}"
                                       "QPushButton{border-radius:10px}"
                                       "QPushButton{padding:2px 4px}")
        icon = QtGui.QIcon()
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("./img/形状 5.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.put_pushbutton.setIcon(icon1)
        self.put_pushbutton.setIconSize(QtCore.QSize(40, 40))
        self.put_pushbutton.setCheckable(True)
        self.put_pushbutton.setAutoRepeat(False)
        self.put_pushbutton.setAutoExclusive(False)
        self.put_pushbutton.setObjectName("put_pushbutton")
        self.SetPutPos_pushbutton = QtWidgets.QPushButton(self.centralwidget)
        self.SetPutPos_pushbutton.setGeometry(QtCore.QRect(122, 70, 356, 80))
        font = QtGui.QFont()
        font.setPointSize(19)
        font.setBold(True)
        font.setWeight(75)
        self.SetPutPos_pushbutton.setFont(font)
        self.SetPutPos_pushbutton.setToolTipDuration(0)
        self.SetPutPos_pushbutton.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.SetPutPos_pushbutton.setStyleSheet("QPushButton{color:#D80C1E}"
                                       "QPushButton{text-align:right}"
                                       "QPushButton:hover{background-color:#EEEEEE}"
                                    #    "QPushButton:hover{color:#EEEEEE}"
                                       "QPushButton{background-color:#FFFFFF}"
                                       "QPushButton{border:2px solid #D80C1E}"
                                       "QPushButton{border-radius:10px}"
                                       "QPushButton{padding:2px 4px}")
        icon = QtGui.QIcon()
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("./img/形状 1.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.SetPutPos_pushbutton.setIcon(icon2)
        self.SetPutPos_pushbutton.setIconSize(QtCore.QSize(40, 40))
        self.SetPutPos_pushbutton.setCheckable(True)
        self.SetPutPos_pushbutton.setAutoRepeat(False)
        self.SetPutPos_pushbutton.setObjectName("SetPutPos_pushbutton")
        self.put_pushbutton.setEnabled(False)

        self.fun_connect()
        # MainWindow.setCentralWidget(self.centralwidget)
        # self.menubar = QtWidgets.QMenuBar(MainWindow)
        # self.menubar.setGeometry(QtCore.QRect(0, 0, 599, 26))
        # self.menubar.setObjectName("menubar")
        # MainWindow.setMenuBar(self.menubar)
        # self.statusbar = QtWidgets.QStatusBar(MainWindow)
        # self.statusbar.setObjectName("statusbar")
        # MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi()

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        # MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.photo_pushbutton.setText(_translate("MainWindow", "  拍照                             "))
        self.put_pushbutton.setText(_translate("MainWindow", "  放置                             "))
        self.SetPutPos_pushbutton.setText(_translate("MainWindow", "  初始                             "))

    def fun_connect(self):
        self.photo_pushbutton.clicked.connect(self.robot_photo)
        self.put_pushbutton.clicked.connect(self.rotbot_put)
        self.SetPutPos_pushbutton.clicked.connect(self.robot_init)

    def set_put_pos(self):
      self.ai.do_set_basket("")

    def take_photo(self):
        self.set_home()
        self.ai.do_capture("")
        time.sleep(0.5)
        self.ai.do_show_image('')
        self.put_pushbutton.setStyleSheet("QPushButton{color:#D80C1E}"
                                       "QPushButton:hover{background-color:#EEEEEE}"
                                       "QPushButton{text-align:right}"
                                    #    "QPushButton:hover{color:#EEEEEE}"
                                       "QPushButton{background-color:#FFFFFF}"
                                       "QPushButton{border:2px solid #D80C1E}"
                                       "QPushButton{border-radius:10px}"
                                       "QPushButton{padding:2px 4px}")
        self.put_pushbutton.setEnabled(True)
        QApplication.processEvents()

    def set_home(self):
        self.ai.do_set_home("")

    def  put_move(self):
        self.put_pushbutton.setStyleSheet("QPushButton{color:#EEEEEE}"
                                        "QPushButton:hover{background-color:#EEEEEE}"
                                        "QPushButton{text-align:right}"
                                      #    "QPushButton:hover{color:#EEEEEE}"
                                        "QPushButton{background-color:#FFFFFF}"
                                        "QPushButton{border:2px solid #D80C1E}"
                                        "QPushButton{border-radius:10px}"
                                        "QPushButton{padding:2px 4px}")
        self.put_pushbutton.setEnabled(False)
        QApplication.processEvents()
        self.ai.do_gripper_open("")
        self.ai.do_grasp("20")
        self.ai.do_gripper_close("10")
        sleep(3)

        self.ai.do_go_basket("")
        self.ai.do_gripper_open("")
        sleep(2)

        self.ai.do_go_home("80")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Window()
    ex.show()
    sys.exit(app.exec_())
