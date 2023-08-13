

#! /usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time

e = """
Communications Failed
"""

x = 0.0
y = 0.0 
theta = 0.0

def callback(msg):
    # print(msg.pose.pose)
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def test(pathpoints):
	msg=Twist()

	pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	rospy.init_node('robot_talker',anonymous=True)

	# rospy.init_node('check_odometry')
	goal_reached = False

	try :
		while not rospy.is_shutdown():
			msg.angular.z=0
			msg.linear.x=-0.1
			#buff='my current time is %s" %rospy.get_time()
			
			odom_sub = rospy.Subscriber('/odom', Odometry, callback)
			# rospy.spin()

			# pathpoints = [[-4,-4],[-2,-3],[-2,-2],[0,-3]]
			
			skipper = 0
			
			while goal_reached is False:
				for points in pathpoints:
					point_reached = False
					skipper += 1
					if skipper % 4 is 0 or  skipper is len(pathpoints):

						while point_reached is False:
							goal_x = points[0]
							goal_y = points[1]

							inc_x = goal_x - x 
							inc_y = goal_y - y

							dist = math.sqrt(inc_x**2 + inc_y**2)

							angle_to_goal = math.atan2(inc_y, inc_x)

							angle_diff = angle_to_goal-theta
							if abs(angle_to_goal-theta) > 0.5 :
								msg.angular.z=np.sign(angle_to_goal-theta)*0.5
								msg.linear.x=-0.0
							elif abs(angle_to_goal-theta) > 0.1 :
								msg.angular.z=np.sign(angle_to_goal-theta)*0.1
								msg.linear.x=-0.0
							else :
								msg.angular.z=0
								if dist > 0.35 :
									msg.linear.x= 0.22
								elif dist > 0.1:
									msg.linear.x= 0.05
								else :
									msg.linear.x= 0.0
									point_reached = True

							print("dist: ", dist )
							print("angle diff :",angle_diff )
							print("lin vel", msg.linear.x)
							print("ang vel", msg.angular.z)
							pub.publish(msg)
				goal_reached = True
					
			

			time.sleep(0.1)


	except :
		print(e)
	finally :	
		msg.angular.z=0
		msg.linear.x=-0
		#buff='my current time is %s" %rospy.get_time()
		pub.publish(msg)



if __name__=='__main__':

	with open('path_points.txt', 'r') as f:
		pathpoints = []
		for line in f:
			words = line.split()
			coordinates =[]
			for i in words:
				i = i.replace(',', '')
				coordinates.append(float(i))
			pathpoints.append(coordinates)
						
	test(pathpoints)
	
	
