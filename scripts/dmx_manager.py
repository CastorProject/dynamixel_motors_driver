#!/usr/bin/env python
import rospy
import time
import dmx_firmware
from dynamixel_motors_driver.msg import GoalPosition
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_motors_driver.srv import TorqueEnable

class DmxManager(object):
    def __init__(self):
        #set node name
        rospy.init_node("dmx_manager")
        #set rates
        self.rate = rospy.Rate(10) # 10hz

        self.get_params()
        self.initPublishers()
        self.initSubscribers()

    def initial_protocol(self):
        time.sleep(5)
        rospy.loginfo("launching initial protocol")
        self.set_safety_position()


    def get_params(self):
        #fecth motor id parameter dictionary
        self.motor_id = rospy.get_param("dmx_id")
        self.head_range = rospy.get_param("head_limit")
        self.right_shoulder_range = rospy.get_param("right_shoulder_limit")
        self.right_elbow_range = rospy.get_param("right_elbow_limit")
        self.left_shoulder_range = rospy.get_param("left_shoulder_limit")
        self.left_elbow_range = rospy.get_param("left_elbow_limit")

        print self.head_range
        #create DmxStiffness
        self.stiffness = dmx_firmware.DmxStiffnessHandler()

    def initPublishers(self):
        #create motor publisher
        self.dmx_publisher = dmx_firmware.DmxMotorPublisher()

    def initSubscribers(self):
        #create dmx motor state Subscriber
        self.dmx_state = dmx_firmware.DmxMotorStateSubscriber()

    #set safety motor position for the robot
    def set_safety_position(self):
        rospy.loginfo("setting safety position")
        safe_position = {
                            'head':           self.head_range["origin"],
                            'right_shoulder': self.right_shoulder_range["origin"],
                            'right_elbow':    self.right_elbow_range["origin"],
                            'left_shoulder':  self.left_shoulder_range["origin"],
                            'left_elbow':     self.left_elbow_range["origin"]
                        }

        self.dmx_publisher.publish(id = self.motor_id, val = safe_position)


    def main_loop(self):


        self.rate.sleep()



    def shutdown(self):
        #go to safe position
        self.set_safety_position()
        #relase motors
        #self.stiffness.set_stiffness(False)

        print("Turn off")

if __name__ == '__main__':
    man = DmxManager()

    rospy.on_shutdown(man.shutdown)

    man.initial_protocol()

    while not (rospy.is_shutdown()):
        man.main_loop()

    rospy.loginfo("DMX manager finished")
