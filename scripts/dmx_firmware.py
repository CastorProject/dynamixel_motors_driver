#!/usr/bin/env python
import rospy
import time
import dmx_motor_publisher as dmx_pub
from dynamixel_motors_driver.msg import GoalPosition
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_motors_driver.srv import TorqueEnable


#object to create motor position publisher
class DmxMotorPublisher(object):
    def __init__(self):
        #TODO: fetch from params
        topic_name = "/goal_dynamixel_position"
        self.head_range = rospy.get_param("head_limit")
        self.right_shoulder_range = rospy.get_param("right_shoulder_limit")
        self.right_elbow_range = rospy.get_param("right_elbow_limit")
        self.left_shoulder_range = rospy.get_param("left_shoulder_limit")
        self.left_elbow_range = rospy.get_param("left_elbow_limit")
        #create publisher
        self.position_publisher = rospy.Publisher(topic_name, GoalPosition, queue_size = 1, latch = False)

    def degree_to_step(self, val):
        return val

    def publish(self, id= {}, val= {}):
        #validate ranges
        if val["head"] > self.head_range["max"]:
            val["head"] =  self.head_range["max"]
        elif val["head"] < self.head_range["min"]:
            val["head"] = self.head_range["min"]
        #set msg lists
        id_list    = [ id["head"], id["right_shoulder"], id["right_elbow"], id["left_shoulder"], id["left_elbow"] ]
        value_list = [ val["head"], val["right_shoulder"], val["right_elbow"], val["left_shoulder"], val["left_elbow"]]
        #publish command
        self.position_publisher.publish(id = id_list, goal_position = value_list)

#subscriber object to get motor states
class DmxMotorStateSubscriber(object):
    def __init__(self):
        #TODO: fetch from params
        topic_name = "/dynamixel_state"
        #create subscriber
        self.state_subscriber = rospy.Subscriber(topic_name, DynamixelStateList, self.callback)
        #data variable
        self.data = []

    def callback(self, state):
        self.data = state.dynamixel_state
        self.data = [self.data[0].present_position, self.data[1].present_position, self.data[2].present_position]

#stiffnesss handler object
class DmxStiffnessHandler(object):
    def __init__(self):
        #create service
        #TODO: get name as param
        self.service = '/joint_torque_enable'
        #get motor id_list
        self.motor_id = rospy.get_param("dmx_id")

        self.stiffness_srv = rospy.Service("enable_stiffness", TorqueEnable, self.set_stiffness)

    def torque_enable(self, id =1, val = False):
        rospy.wait_for_service(self.service)
        try:
             enable_torque = rospy.ServiceProxy(self.service, TorqueEnable)
             #send service
             res = enable_torque(id = id, torque_enable= val)
             time.sleep(0.001)
             return (res.result)

        except rospy.ServiceException, e:
             print "Service call failed: %s"%e


    def set_stiffness(self, req):
        val = req.torque_enable
        #call service for all motors
        res = self.torque_enable(self.motor_id["head"], val = val)
        print res
        res = self.torque_enable(self.motor_id["right_elbow"], val = val)
        print res
        res = self.torque_enable(self.motor_id["left_elbow"], val = val)
        print res
        res = self.torque_enable(self.motor_id["right_shoulder"], val = val)
        print res
        res = self.torque_enable(self.motor_id["left_shoulder"], val = val)
        print res

        return res
