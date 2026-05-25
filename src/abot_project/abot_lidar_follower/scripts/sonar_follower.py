#!/usr/bin/env python
# test mail: chutter@uos.de

import rospy
from geometry_msgs.msg import Twist, Vector3
from riki_msgs.msg import Sonar

class SonarFollower:
	def __init__(self):
		self.mini_distance= rospy.get_param('~mini_distance', 30) # if this is set to False the O button has to be kept pressed in order for it to move
		self.max_distance= rospy.get_param('~max_distance', 50) # if this is set to False the O button has to be kept pressed in order for it to move
		self.lost_distance= rospy.get_param('~lost_distance', 100) # if this is set to False the O button has to be kept pressed in order for it to move
		self.speed = rospy.get_param('~speed', 0.2) 


		self.cmdVelPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size =3)
		# the topic for the messages from the ps3 controller (game pad)

		# the topic for the tracker that gives us the current position of the object we are following
		self.SonarSubscriber = rospy.Subscriber('/sonar', Sonar, self.SonarCallback)

		rospy.on_shutdown(self.controllerLoss)

	
	def SonarCallback(self, sonar):
		distance = sonar.distance

		velocity = Twist()	
		rospy.loginfo('Distance: {}, '.format(distance))
                if distance > self.max_distance and distance < self.lost_distance:	
			velocity.linear = Vector3(self.speed,0,0.)
			velocity.angular= Vector3(0., 0., 0.)
                elif distance < self.mini_distance :
			velocity.linear = Vector3(-self.speed,0,0.)
			velocity.angular= Vector3(0., 0., 0.)
		self.cmdVelPublisher.publish(velocity)
		


	def stopMoving(self):
		velocity = Twist()
		velocity.linear = Vector3(0.,0.,0.)
		velocity.angular= Vector3(0.,0.,0.)
		self.cmdVelPublisher.publish(velocity)

	def controllerLoss(self):
		# we lost connection so we will stop moving and become inactive
		self.stopMoving()
		rospy.loginfo('lost connection')
		


if __name__ == '__main__':
	print('starting')
	rospy.init_node('sonar_follower')
	follower = SonarFollower()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')


