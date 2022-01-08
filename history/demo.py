from jaka import JAKA
import time
# _ABS = 0
# _INRC = 1
# _homePose = None
# # 算法预测结果输出
# RR =[[ 0.19847853,  0.6778425 , -0.70790946],
#        [-0.5518209 ,  0.6742066 ,  0.49085554],
#        [ 0.81      ,  0.29321492,  0.5078631 ]]
# TT =[ 0.00828513, -0.01113187,  0.319     ]

# # TT 单位 cm
# # curFlangePose 单位 mm
# # Bais 单位 mm
# home =[250.29994417884936, 495.03823780097326, 148.59145700527188, 3.11032295740378, 0.0007676469403735762, -0.6444378437842379]
# tcp  = JAKA("192.168.1.8")
# curToolPose = tcp.getpos6DoF()

# tcp.robot.linear_move(end_pos = home, move_mode = _ABS, is_block = False, speed = 15)
# time.sleep(2)



IO_TOOL = 1
IO_CABINET =0
#控制柜面板 IO
#工具 IO
IO_EXTEND = 2
#扩展 IO
tcp  = JAKA("192.168.15.106")

# tcp.robot.set_digital_output(2,index = 0,value = 1)#设置 DO3 的引脚输出

# tcp.robot.set_digital_output(2,index = 1,value = 2)#设置 DO3 的引脚输出
# # iotype = IO_EXTEND
# tcp.robot.set_digital_output(2,index = 3,value = 500)#设置 DO3 的引脚输出
# tcp.robot.logout()
#登出

robot = tcp.robot


IO_TOOL = 1
IO_CABINET=0
#控制柜面板 IO
#工具 IO
IO_EXTEND = 2
#扩展 IO

tcp.moveInFlangeCoordinate([0,0,-30,0,0,0])
# robot.set_analog_output(iotype = 2,index = 0,value = 0)#
# time.sleep(1)
# robot.set_analog_output(iotype = 2,index = 1,value = 20)#
# robot.set_analog_output(iotype = 2,index = 3,value = 500)#
# time.sleep(1)
# robot.set_analog_output(iotype = 2,index = 1,value = 20)#
time.sleep(1)
robot.logout()
