#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import ActuatorControl

def talker():
	rospy.init_node('actuator_controller', anonymous=True)

	pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)

	rate = rospy.Rate(1)

	msg_out = ActuatorControl()
	msg_out.group_mix = 2 # Use group 2 (auxilary controls)
	msg_out.controls = [0.0, 0.0, 0.0, 0.0, -1.0, -1.0, -1.0, -1.0]
	is_low = True

	while not rospy.is_shutdown():
		is_low = not is_low

		if is_low:
			msg_out.controls = [0.0, 0.0, 0.0, 0.0, -1.0, -1.0, -1.0, -1.0]
			rospy.loginfo("Set servos low")
		else:
			msg_out.controls = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]
			rospy.loginfo("Set servos high")

		msg_out.header.stamp = rospy.Time.now()
		pub.publish(msg_out)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass