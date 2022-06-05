#!/usr/bin/env python
from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
# thread module of python
import threading

from std_msgs.msg import String
# TFMessage module of tf2 messages, tf means transformation
# It used to get information of end-effector pose 
from tf2_msgs.msg import TFMessage
# Rotation module of scipy package
# It used for convert quaternion to euler or euler to quaternion
from scipy.spatial.transform import Rotation

# keyboard module of pynput python package for keyboard input
from pynput import keyboard

import socket, time

class Communication:
  def __init__(self):
    #Member
    self.running_main = False
    self.running_thread = True
    self.running_send_thread = True
    self.terminal_thread = threading.Thread(target = self.manual_code_input)
    self.terminal_thread.start()
    self.send_thread = threading.Thread(target = self.pos_send)
    self.send_thread.start()
    self.robot_now = 'R'
                    #01234567890123456789012345678901234567890123456789
    self.str_send = 'D0.000,0.000,0.000,0.180,0.0,000.0,0.0,F,R  '
    self.spec_length = len(self.str_send)

    str_pub_code = rospy.Publisher("str_code", String, queue_size = 10)
    #rospy.Subscriber("/tf", TFMessage, self.callback_endpos)
    rospy.Subscriber("str_now", String, self.callback_now)
    rospy.Subscriber("str_pos", String, self.callback_pos)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('192.168.0.7', 1470))
    sock.listen(15)
    print("Listening...")
    self.client_socket, addr = sock.accept()
    print("Connected ! ", addr)
    self.running_main = True  #now start
    while self.running_main:
      rospy.sleep(0.03)
      try:
        data = self.client_socket.recv(1024)
      except socket.timeout:
        print('timeout')
        continue
      msg = data.decode()
      if msg.find('S') != -1:
        print('recv msg:', msg)
        self.running_main = False
        self.running_thread = False
        self.running_send_thread = False
        break
      elif msg.find('T') != -1:
        print('recv msg:', msg)
      elif msg.find('R') != -1:
        print('recv msg:', 'R')
        str_pub_code.publish(msg)
      elif msg == 'E':
        print('recv msg:', msg)
      elif msg.find('P') != -1:
        print('recv msg:', msg)
        str_pub_code.publish('P')
      elif msg.find('G') != -1:
        print('recv msg:', msg)
        str_pub_code.publish('G')
      elif len(msg.split(',')) == 8:
        str_pub_code.publish(msg)
    #end
    print("Socket close")
    rospy.sleep(1)
    self.client_socket.close()
    sock.close()

  def manual_code_input(self):
    while self.running_thread:
      code = raw_input('input : ')
      print(code)
      if code == 'end':
        self.running_main = False
        self.running_thread = False
        self.running_send_thread = False
    print('manual thread end')

  def pos_send(self):
    while self.running_send_thread:
      rospy.sleep(1)
      if (self.running_main):
        try :
          self.client_socket.send(self.str_send.encode(encoding='utf=8'))
        except socket.error as msg:
          print(msg)

  def callback_now(self, now):
    if now.data == 'G':
      self.update_now('G')
    elif now.data == 'P':
      self.update_now('P')
    elif now.data == 'R':
      self.update_now('R')
    elif now.data == 'F':
      self.update_now('F')
    
  def update_now(self, now):
    print('call ' + now)
    self.robot_now = now
    s = list(self.str_send)
    s[41] = self.robot_now
    self.str_send = "".join(s)
    print(self.str_send)

  def callback_pos(self, str_pos):
    if (self.running_main):
      self.str_send = 'D' + str_pos.data + ',' + self.robot_now
      #add blank string
      while len(self.str_send) < self.spec_length:
        self.str_send += ' '
      print(self.str_send)

def main(args):
  rospy.init_node('com', anonymous=True)
  try:
    com = Communication()
  except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    main(sys.argv)