# region import basic library
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
from time import sleep

SPEED = 120

class Query(cmd.Cmd):
  # region 成员参数
    intro = '[JAKA] please in put help or ? for help\n'  # 命令行欢迎
    prompt = 'JAKA>>>'      # 命令行提示符
    depth_image = None      # 深度图像
    color_image = None       # 彩色图像
    toolCoordinate = None # 根据坐标系 可以用APP设置
    gg = None  # grasp group 用于存储算法输出结果
    homePose = None      # home 位姿 用于存储home
    basketPose = None    # basket 位姿 basket
    putDownPose = None
    tcp = None
  # endregion
  # region 成员方法
    def __init__(self):
      super().__init__()
      with open("./config.json",'r') as load_f:
        load_dict = json.load(load_f)

      self.tcp = JAKA(load_dict['IPaddress'])
      self.homePose = load_dict['homePose']
      self.basketPose = load_dict['basketPose']
      self.putDownPose = load_dict['putDownPose']
      self.do_gi("")
      #tcp.getpos6DoF()

    def wait(self,time):
        sleep(time*0.1)

    def do_go(self,arg):
      if(not arg):
        arg = 1000
      self.tcp.robot.set_analog_output(iotype = 2,index = 0,value = 0)#
      self.tcp.robot.set_analog_output(iotype = 2,index = 1,value = 100)#
      self.tcp.robot.set_analog_output(iotype = 2,index = 3,value = int(arg))#

    def do_gc(self,arg):
      if(not arg):
        arg = 20
      self.tcp.robot.set_analog_output(iotype = 2,index = 0,value = 0)#
      self.tcp.robot.set_analog_output(iotype = 2,index = 1,value = 50)#
      self.tcp.robot.set_analog_output(iotype = 2,index = 3,value = int(arg))#


    def do_gp(self,arg):
      print(self.tcp.getjoints())

    def do_sh(self,arg):
      self.homePose = self.tcp.getjoints()
      print(self.homePose)

    def do_gh(self,arg):
      if not arg:
        arg = 40
      self.tcp.joint_move(self.homePose,int(SPEED))
      self.wait(20)

    def do_sb(self,arg):
      self.basketPose = self.tcp.getjoints()


    def do_gb(self,arg):
      self.tcp.joint_move(self.basketPose,SPEED)
      self.tcp.joint_move(self.putDownPose,SPEED)
      self.do_go("")
      self.wait(20)
      self.tcp.joint_move(self.basketPose,SPEED)



    def do_gsp(self,arg):
      self.do_go("")
      curPos = self.tcp.getpos6DoF()
      dkr = GG_2_Liner_move_V2(self.gg,curPos)
      with open("./config.json",'r') as load_f: # height restriction
        load_dict = json.load(load_f)
      if dkr[2]>load_dict['posRes']:
        print("valied: ",dkr)
        if not arg:
          arg = 30
        self.tcp.robot.linear_move(end_pos = dkr,move_mode = 0,is_block=True,speed=SPEED)
        self.tcp.jj([0,0, self.gg.depths[0]*1000,0,0,0],dkr,SPEED)
        self.do_gc("")
        self.wait(15)
        self.tcp.jj([0,0, -100,0,0,0],dkr,SPEED)
        return True
      else:
        print("invalied: ",dkr)
        return False


    def do_cap(self,arg):
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
      if(arg == ''):
        arg = '1'
      self.gg = demo_inner(self.color_image,self.depth_image , './',int(arg))

    def cap(self,arg):
      time.sleep(0.5)
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
      if(arg == ''):
        arg = '1'
      self.gg = demo_inner(self.color_image,self.depth_image , './',int(arg))
      print("I see an obj!" if len(self.gg)>0 else "Nothing at all.")
      return len(self.gg)

    def do_orbit_manu(self,arg): # 循环抓取
      num = 0
      if not arg:
        num = 1
      else:
        num = int(arg)
      while num: # 抓取 num次
        if self.cap(""):  # 如果有在圆锥里的grasp 就进入下一步
          if self.do_gsp(""): # 如果该grasp 没有低于桌面临界值，就进行抓取 并返回True
            self.do_gb("") # 回basket
            self.do_gh("") # 回家
        num -= 1

    def do_orbit(self,arg):
      self.do_gh("")
      print(arg)
      while True:
        if self.cap(""):
          self.do_gsp("")
          self.do_gb("")
          self.do_gh("")
        else:
          break

    def do_gi(self, arg): # 先下电，再初始化夹爪
      self.tcp.robot.disable_robot()
      if(not arg):
        arg = 1000
      self.tcp.robot.set_analog_output(iotype = 2,index = 0,value = 1) # 夹爪enable
      sleep(5)
      self.tcp.robot.enable_robot() # 机器人上电

    def do_exit(self, _):
        '退出'
        if self.tcp:
          self.tcp.robot_disconnect()
        if pipeline:
          pipeline.stop()
        exit(0)
  # endregion


if __name__ == '__main__':
    a = Query()
    # a.cmdloop()
    a.do_gh("")
    while True:
      # IO_input = a.tcp.robot.get_digital_input(0,8) # DI9 : 9-1 = 8
      # sleep(1)
      # if (not IO_input[0]) and IO_input[1]:
      a.do_orbit_manu("")
