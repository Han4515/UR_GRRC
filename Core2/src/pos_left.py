#!/usr/bin/env python
from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
# thread module of python
import threading
# standard messages for various purpose (e.g. String)
from std_msgs.msg import String

class Position:
    def __init__(self):
        #Member initialize
        self.seq_num = 0
        self.is_senario = False
        self.pause = False

        self.str_pub_pos = rospy.Publisher("str_pos", String, queue_size = 10)
        self.str_pub_now = rospy.Publisher("str_now", String, queue_size = 10)
        rospy.Subscriber("str_code", String, self.callback_code)

    def callback_code(self, str_code):
        #Code
        print(str_code.data)
        if str_code.data is 'G' and self.is_senario is False:
            self.is_senario = True
            self.seq_num = 1
            self.str_pub_now.publish('G')
            self.senario_thread = threading.Thread(target = self.GRRC_video)
            self.senario_thread.start()
            print('recv start')
        elif str_code.data is 'R':
            self.seq_num = 0
            self.is_senario = False
            self.str_pub_now.publish('R')
            print('recv ready')
        elif str_code.data is 'P':
            if self.pause is True :
                self.pause = False
                self.str_pub_now.publish('G')
            else:
                self.pause = True
                self.str_pub_now.publish('P')
            print('recv pause')
        elif len(str_code.data.split(',')) == 8:
            self.str_pub_pos.publish(str_code.data)

    def GRRC_video(self):
        while self.seq_num is not 0:
            rospy.sleep(0.03)
            if self.pause is True:
                continue

            if self.seq_num is 1:
                self.str_pub_pos.publish("0.500,0.000,0.500,180.0,0.0,000.0,3.0,F")#1
                rospy.sleep(3)
            elif self.seq_num is 2:
                self.str_pub_pos.publish("0.450,0.450,0.200,180.0,0.0,000.0,3.0,O")#2
                rospy.sleep(3)
            elif self.seq_num is 3:
                self.str_pub_pos.publish("0.450,0.450,0.090,180.0,0.0,000.0,3.0,T")#3
                rospy.sleep(3)
                rospy.sleep(2)
            elif self.seq_num is 4:
                self.str_pub_pos.publish("0.450,0.450,0.200,180.0,0.0,000.0,3.0,O")#4
                rospy.sleep(3)
                rospy.sleep(6)#5
            elif self.seq_num is 5:
                self.str_pub_pos.publish("0.538,-0.635,0.300,180.0,0.0,000.0,6.0,O")#6
                rospy.sleep(6)
            elif self.seq_num is 6:
                self.str_pub_pos.publish("0.538,-0.635,0.140,180.0,0.0,000.0,3.0,F")#7
                rospy.sleep(3)
            elif self.seq_num is 7:
                self.str_pub_pos.publish("0.538,-0.635,0.300,180.0,0.0,000.0,3.0,O")#8
                rospy.sleep(3)
                rospy.sleep(2)
            elif self.seq_num is 8:
                self.str_pub_pos.publish("0.450,0.350,0.200,180.0,0.0,000.0,6.0,O")#9
                rospy.sleep(6)
                rospy.sleep(11)#10 11 12
                #rospy.sleep(30)#13
                self.pause = True   #wait!
                self.str_pub_now.publish('P')
            elif self.seq_num is 9:
                self.str_pub_pos.publish("0.450,0.350,0.090,180.0,0.0,000.0,3.0,T")#14
                rospy.sleep(3)
                rospy.sleep(2)
            elif self.seq_num is 10:
                self.str_pub_pos.publish("0.450,0.350,0.200,180.0,0.0,000.0,3.0,O")#15
                rospy.sleep(3)
                rospy.sleep(6)#16
            elif self.seq_num is 11:
                self.str_pub_pos.publish("0.538,-0.635,0.300,180.0,0.0,000.0,6.0,O")#17
                rospy.sleep(6)
            elif self.seq_num is 12:
                self.str_pub_pos.publish("0.538,-0.635,0.135,180.0,0.0,000.0,3.0,F")#18
                rospy.sleep(3)
            elif self.seq_num is 13:
                self.str_pub_pos.publish("0.538,-0.635,0.300,180.0,0.0,000.0,3.0,O")#19
                rospy.sleep(3)
                rospy.sleep(2)
            elif self.seq_num is 14:
                self.str_pub_pos.publish("0.538,0.000,0.300,180.0,0.0,000.0,3.0,O")#20
                rospy.sleep(3)
            elif self.seq_num is 15:
                self.str_pub_pos.publish("0.450,0.450,0.200,180.0,0.0,000.0,3.0,O")#21
                rospy.sleep(3)
                #rospy.sleep(3)#22~30
            elif self.seq_num is 16:
                self.str_pub_pos.publish("0.500,0.000,0.500,180.0,0.0,000.0,3.0,F")#1
                rospy.sleep(3)
                #end
                self.seq_num = -1
                self.is_senario = False
                self.str_pub_now.publish('F')
                print('ok')
            self.seq_num += 1
        print('out')

    

def main(args):
  rospy.init_node('pos', anonymous=True)
  try:
    pos = Position()
    rospy.spin()
  except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    main(sys.argv)