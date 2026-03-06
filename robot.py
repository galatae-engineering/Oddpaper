import serial
import time
from threading import Thread
import json
import math
import serial.tools.list_ports

class Robot:
  def __init__(self,debug=False):
    self.absolute_distance_mode=True
    self.debug=debug

    port_name=None
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
      if(p.description=="Giga"):
        port_name=p.device
    
    if(port_name==None):
      print("Error, no arduino found")

    self.arduino=serial.Serial(port=port_name, baudrate=115200, timeout=0.1)
    time.sleep(2)
    #self.reset_and_home_joints()

  def _send_message(self,message):
    self.arduino.write((message+"\n").encode('utf-8'))
    if(self.debug):
      print(message+" sent")

  def _read_message(self):
    a=self.arduino.readline()
    data = a.decode('utf-8', errors='ignore')
    return data

  def _print_message_from_serial(self,message):
    print("arduino: "+message)

  def _read_serial_buffer(self):
    message=self._read_message()
    return message.strip()

  def send_message_and_wait_conf(self,message):
    self._send_message(message)
    return self._wait_for_conf()

  def _wait_for_message(self):
    message=self._read_serial_buffer()
    
    while(message==""):
      message=self._read_serial_buffer()
      time.sleep(0.001)

    if(self.debug):
      self._print_message_from_serial(message)
    
    return message

  def _wait_for_conf(self):
    message=self._wait_for_message()
    while(message!="ok" and message!="error"):
      message=self._wait_for_message()

    return message=="ok"

  def reset_and_home_joints(self):
    self.send_message_and_wait_conf("$RST=*")
    self.send_message_and_wait_conf("$H")

  def _get_gcode_arguments_string(self,args,args_represent_pose=True):
    if(args_represent_pose):
      gcode_words=["X","Y","Z","A","B","C"]
    else:
      gcode_words=["I","J","K","L","M","N"]
    s=""
    for i in range(len(args)):
      s=s+gcode_words[i]+str(round(args[i],2))

    return s

  def _move(self,pose):
    message="G1"+self._get_gcode_arguments_string(pose)
    return self.send_message_and_wait_conf(message)
  
  def probe(self,pose):
    message="G38.2"+self._get_gcode_arguments_string(pose)
    return self.send_message_and_wait_conf(message)
  
  def update_absolute_distance_mode(self,expected_value):
    if(self.absolute_distance_mode != expected_value):
      self.send_message_and_wait_conf(["G91","G90"][expected_value])
      self.absolute_distance_mode=expected_value

  #pose=[x,y,z,pitch (relative to z axis), roll (relative to arm),gripper angle]
  def go_to_pose(self,pose):
    self.update_absolute_distance_mode(True)
    self._move(pose)
      
  def jog(self,pose):
    self.update_absolute_distance_mode(False)
    self._move(pose)

  def _ask_for_pos_json_and_return_property_value(self,property_name):
    self._send_message("?")
    
    property_value=""
    while(property_value==""):
      message=self._wait_for_message()
      try:
        property_value=json.loads(message)[property_name]
      except:
        pass

    return property_value

  def get_tool_pose(self):
    return self._ask_for_pos_json_and_return_property_value("tool_pose")
  
  def get_angles(self):
    return self._ask_for_pos_json_and_return_property_value("angles")
  
  def reset_angles(self,angles):
    message="G92"+self._get_gcode_arguments_string(angles,False)
    self.send_message_and_wait_conf(message)

  #speed in deg/s , 100 deg/s is like a good value
  def set_joint_speed(self,speed):
    message="F"+str(speed)
    self.send_message_and_wait_conf(message)

  def _get_point_in_line_segment(self,p1,p2,rel_pos):
    p=[]
    number_of_indexes=min(len(p1),len(p2))

    for i in range(number_of_indexes):
      p.append(p1[i]+(p2[i]-p1[i])*rel_pos)

    return p

  def _get_dist_between_vectors(self,p1,p2):
    square_dist=0
    for i in range(len(p2)):
      square_dist+=math.pow(p2[i]-p1[i],2)

    return math.sqrt(square_dist)

  def set_joint_speed_and_get_number_of_iterations(self,p1,p2):
    self.set_joint_speed(1000)
    distance=self._get_dist_between_vectors(p1,p2)
    N=math.ceil(distance)
    return N
  
  def linear_move_to_pose(self,p2):
    p1=self.get_tool_pose()
    N=self.set_joint_speed_and_get_number_of_iterations(p1,p2)

    for i in range(N+1):
      intermediate_pose=self.get_pose_in_line_segment(p1,p2,i/N)
      self.go_to_pose(intermediate_pose)

  def linear_probe(self,p2):
    p1=self.get_tool_pose()
    N=self.set_joint_speed_and_get_number_of_iterations(p1,p2)
    
    success=False
    i=0
    while(i<=N and success==False):
      intermediate_pose=self._get_point_in_line_segment(p1,p2,i/N)
      success=self.probe(intermediate_pose)
      i+=1

    return success

  def open_gripper(self,extra_degrees=0):
    gcode="M101"
    if(extra_degrees!=0):
      gcode+="P"+str(extra_degrees)
    self.send_message_and_wait_conf(gcode)

  def close_gripper(self,extra_degrees=0):
    self.send_message_and_wait_conf("M100P"+str(extra_degrees))

  def calibrate_gripper(self):
    self.send_message_and_wait_conf("$H")

  def go_to_foetus_pos(self):
    self.send_message_and_wait_conf("G28")

  def set_tool(self,args): #[x,y,z]
    self.send_message_and_wait_conf("G10L2P1"+self._get_gcode_arguments_string(args))
