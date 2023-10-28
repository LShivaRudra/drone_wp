#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import numpy as np

sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 3)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: ,%s")%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s")%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Stabilized Mode could not be set.")%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set.")%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Altitude Mode could not be set.")%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Position Mode could not be set.")%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Autoland Mode could not be set.")%e


class Controller:
    def __init__(self):
        self.drone_pose = PoseStamped()
        self.state = State()
        self.sp = PoseStamped()

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        To convert euler angles to quaternion.
        
        Input params(in radians)
             roll: The rotation around x-axis.
             pitch: The rotation around y-axis.
             yaw: The rotation around z-axis.
        
        Output list
             [qx, qy, qz, qw]: The orientation in quaternion 
        """
        self.qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        self.qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        self.qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        self.qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [self.qx, self.qy, self.qz, self.qw]

    def pose_cb(self,msg):
        self.drone_pose = msg

    def stateCb(self,msg):
        self.state = msg

def main():
    modes = fcuModes()
    cnt = Controller()
    rospy.init_node('position_wp_node',anonymous=True)
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    sub_pose = rospy.Subscriber('/mavros/local_position/pose',PoseStamped,callback=cnt.pose_cb)
    # sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    tolerance_ = 0.1
    rate = rospy.Rate(20)

    while (cnt.state.armed == 0):
        modes.setArm()
        rate.sleep()
        print("ARMING")

    modes.setOffboardMode()
    print("---------")
    print("OFFBOARD")
    print("---------")
    
    waypoints_={1:[0.0,0.0,1.0],2:[5.0,0.0,1.0],3:[5.0,5.0,1.0],4:[0.0,5.0,1.0],5:[0.0,0.0,1.0]}

    for i in range(1,6):
      cnt.sp.pose.position.x = waypoints_[i][0]
      cnt.sp.pose.position.y = waypoints_[i][1]
      cnt.sp.pose.position.z = waypoints_[i][2]
      drone_pose = [cnt.drone_pose.pose.position.x, cnt.drone_pose.pose.position.y, cnt.drone_pose.pose.position.z]
      sp = [cnt.sp.pose.position.x, cnt.sp.pose.position.y, cnt.sp.pose.position.z]
      while (np.linalg.norm(np.array(drone_pose) - np.array(sp))>tolerance_):
        print(waypoints_[i])
        drone_pose = [cnt.drone_pose.pose.position.x, cnt.drone_pose.pose.position.y, cnt.drone_pose.pose.position.z]
        sp_pub.publish(cnt.sp)
        rate.sleep()
    
    modes.setAutoLandMode()
    print("---------")
    print("LANDING")
    print("---------")

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass