#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
import audio_gen_oop

audio_gen = audio_gen_oop.Audio_gen(600,'/home/turtlebot/catkin_ws/src/my_turtle_turn/scripts/White-noise-sound-20sec-mono-44100Hz.wav',0)
hz = 100

angular_vel = 0.1
def turn():
    pub = rospy.Publisher('mobile_base/commands/velocity', Twist,queue_size=1)
    turn_left = Twist()
    turn_left.angular.z = angular_vel
    turn_left.linear.x = 0#-linear_vel/10.0
    
    turn_right = Twist()
    turn_right.angular.z = -angular_vel
    turn_right.linear.x = 0#-linear_vel/10.0
    
    stop = Twist()
    stop.angular.z = 0
    stop.linear.x = 0
    audio_gen.send_to_speaker()

    time.sleep(15)
    t = rospy.Time.now().to_sec()
    
    
    while not rospy.is_shutdown():
    
        pub.publish(turn_left)
               
        rate.sleep()
    print("Quitting")

if __name__=="__main__":
    node = rospy.init_node('my_turtle',anonymous=True)
    try:
        turn()
    except rospy.ROSInterruptException:
        pass