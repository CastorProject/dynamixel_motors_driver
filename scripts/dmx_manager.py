#!/usr/bin/env python
import rospy
import time
import dmx_firmware
#from dynamixel_.msg import GoalPosition
#from dynamixel_workbench_msgs_custom.msg import DynamixelStateList
#from dynamixel_motors_msgs_custom.srv import TorqueEnable

#TODO: CREATE DISABLE SERVICE


class DmxManager(object):
    def __init__(self):
        #set node name
        rospy.init_node("dmx_manager")
        #set rates
        self.rate = rospy.Rate(10) # 10hz
        #fetch params from yaml files
        self.get_params()
        self.initPublishers()
        self.initSubscribers()
        #OFFs
        self.OFF = False

    def initial_protocol(self):
        time.sleep(5)
        rospy.loginfo("launching initial protocol")
        self.set_safety_position()

    #function to read yaml config parameters
    def get_params(self):
        #READ DMX_CONFIG FILE#
        #read motor's info
        self.head_info           = rospy.get_param("head")
        self.right_shoulder_info = rospy.get_param("right_shoulder")
        self.right_elbow_info    = rospy.get_param("right_elbow")
        self.left_shoulder_info  = rospy.get_param("left_shoulder")
        self.left_elbow_info     = rospy.get_param("left_elbow")

        #read origin position
        self.origin              = rospy.get_param("origin")
        #group ids
        self.ids                 = {"head"          : self.head_info["ID"],
                                    "right_shoulder": self.right_shoulder_info["ID"],
                                    "right_elbow"   : self.right_elbow_info["ID"],
                                    "left_shoulder" : self.left_shoulder_info["ID"],
                                    "left_elbow"    : self.left_elbow_info["ID"]
                                    }

        #merge origin with general articulation info
        self.head_info["origin"] = self.origin["head"]
        self.left_shoulder_info["origin"] = self.origin["left_shoulder"]
        self.left_elbow_info["origin"] = self.origin["left_elbow"]
        self.right_shoulder_info["origin"] = self.origin["right_shoulder"]
        self.right_elbow_info["origin"] = self.origin["right_elbow"]

        #fetch service names
        self.goal_position_srv   = "/dmx_controller/goal_position"
        self.goal_speed_srv      = "/dmx_controller/goal_speed"
        self.torque_enable_srv   = "/dmx_controller/torque_enable"
        #fetch topics names
        self.motor_status_topic  = "/dmx_controller/dynamixel_status"
        self.joint_state_topic   = "/dmx_controller/joint_states"


    #function to launch publishers
    def initPublishers(self):
        print("Init manager publisher")

    #function to launch subscribers
    def initSubscribers(self):
        #create motor status subscriber
        self.dmx_status_sub = dmx_firmware.DmxMotorStatusSubscriber(topic_name = self.motor_status_topic)

        #create joint state subscriber
        self.joint_state_sub = dmx_firmware.DmxJointStatesSubscriber(topic_name = self.joint_state_topic )


    #set position function [pos = dict {"joint_name": value}]
    def set_position(self, pos):
        #enable torque
        service = dmx_firmware.DmxCommandClientService(service_name = self.goal_position_srv)
        for joint in pos:
            service.service_request_threaded(id = self.ids[joint], val = pos[joint])

    def set_speed(self, speed):
        service = dmx_firmware.DmxCommandClientService(service_name = self.goal_speed_srv)
        for joint in speed:
            service.service_request_threaded(id = self.ids[joint], val = speed[joint])



    #TODO: move to next layer
    def open_arms(self):
        safe_position = {
                            'head':           self.origin["head"],
                            'right_shoulder': 120,
                            'right_elbow':    self.origin["right_elbow"],
                            'left_shoulder':  800,
                            #'left_elbow':     self.origin["left_elbow"]
                        }
        self.set_position(safe_position)



    def rotate_head_right(self):
        #print self.head_info

        pos = {'head':self.head_info['CW_Angle_Limit']}
        self.set_position(pos)

    def rotate_head_left(self):
        #print self.head_info

        pos = {'head':self.head_info['CCW_Angle_Limit']}
        self.set_position(pos)

    def rotate_head(self, angle):
        pos = {'head':angle}
        self.set_position(pos)

    def point_routine(self):
        self.set_safety_position()
        time.sleep(3)
        self.rotate_head(angle = self.head_info['CCW_Angle_Limit']*3/4)
        self.set_position({'left_elbow':1000})

    def routine1(self):
        for i in range(5):
            self.open_arms()
            self.rotate_head_right()
            time.sleep(3)
            self.set_safety_position()
            self.rotate_head_left()
            time.sleep(3)

    def routine2(self):
        self.set_speed({
                            'head':           200,
                            'right_shoulder': 200,
                            'right_elbow':    200,
                            'left_shoulder':  200,
                            'left_elbow':     200
                        })
        self.set_safety_position()
        time.sleep(3)
        for i in range(5):
            print i
            self.set_position({'left_elbow':1000, 'right_elbow':70})
            time.sleep(3)
            self.set_safety_position()

            time.sleep(3)



    #set safety motor position for the robot
    def set_safety_position(self):
        rospy.loginfo("setting safety position")
        safe_position = {
                            'head':           self.origin["head"],
                            'right_shoulder': self.origin["right_shoulder"],
                            'right_elbow':    self.origin["right_elbow"],
                            'left_shoulder':  self.origin["left_shoulder"],
                            'left_elbow':     self.origin["left_elbow"]
                        }
        self.set_position(safe_position)


    #set stiffness to all motors
    def set_stiffness(self, val= False):
        service = dmx_firmware.DmxStiffnessHandler(service_name = self.torque_enable_srv)
        for i in self.ids:
            service.service_request(id = self.ids[i], val = val)


    #def main assessment loop
    def main_loop(self):


        self.rate.sleep()



    def shutdown(self):
        #go to safe position
        print("shutdown")
        self.set_stiffness(False)
        self.OFF = True
        #relase motors
        #self.stiffness.set_stiffness(False)

        print("Turn off")

if __name__ == '__main__':
    man = DmxManager()

    #rospy.on_shutdown(man.shutdown)
    #man.routine1()

    man.point_routine()
    time.sleep(3)
    man.set_safety_position()

    man.routine1()
    time.sleep(3)
    man.set_safety_position()

    time.sleep(2)
    
    man.routine2()
    time.sleep(3)
    man.set_safety_position()



    while not (rospy.is_shutdown()):
        man.main_loop()


    time.sleep(5)
    rospy.loginfo("DMX manager finished")
