#!/usr/bin/env python
import rospy
import angles
from geometry_msgs.msg import Twist,Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64,String
from sensor_msgs.msg import Imu
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
import random
import math
import SWHear
import time
import numpy as np
hz = 10
ear = SWHear.SWHear(rate=44100,updatesPerSecond=hz)
ear.stream_start()
freq = 500


linear_vel = 0.1
angular_vel = 0.1

yaw = 0
turn_prob = 1.0/(hz*10)
mu = math.pi
sigma = math.pi/2.0
tolerance = 10 * math.pi / 180

new_heading = 'NaN'
turn_amt = 0
turn = False
bumper = BumperEvent()
avoid_obstacle = False

drd_heading = 0
start = True
hdg_ctrl_effort = 0
hdg_scale = angular_vel/100.0 #max_rot_vel/max_ctrl_effort

goal = Point()
pose = Point()
# pose = Odometry()
log_filename = '/home/turtlebot/experiments/catkin_ws/my_turtle/'+time.strftime('%Y%m%d%H%M%S') + 'data.txt'

def callback_imu(data):
    global yaw,drd_heading,start

    quaternion = (data.orientation.x,
                data.orientation.y,
                data.orientation.z,
                data.orientation.w)
    (r, p, y) = euler_from_quaternion(quaternion)
    yaw = y
    if start:
        start = False
        drd_heading = yaw
def callback_bumper(data):
    global bumper, avoid_obstacle
    bumper = BumperEvent()
    bumper = data
    if bumper.state == 1 and not turn:
        avoid_obstacle = True

def callback_hdg_pid(data):
    global hdg_ctrl_effort
    hdg_ctrl_effort = data.data

def callback_odom(data):
    global pose
    pose = Point()
    pose = data.pose.pose.position

def goal_direction(g,p):
    """ function to compute the orientation of robot wrt to gaol.
        g is goal x,y location and p is Twist of robot current pose"""
    # return math.atan2(g.y - p.pose.pose.position.y, g.x - p.pose.pose.position.x)
    return math.atan2(g.y - p.y, g.x - p.x)

def goal_distance(g,p):
    """computes distance of robot from desired goal location"""
    # return math.sqrt(pow(g.x-p.pose.pose.position.x,2) + pow(g.y-p.pose.pose.position.y,2))
    return math.sqrt(pow(g.x-p.x,2) + pow(g.y-p.y,2))
def callback_log(data):
    with open(log_filename,'a') as f:
        f.write(data.data+'\n')
square = [[0,0],[1,0],[1,1],[0,1]]
def update_goal(p):
    new_goal = Point()
    new_goal.x = square[p%4][0]
    new_goal.y = square[p%4][1]
    return new_goal

def explore():
    global turn,new_heading,turn_amt,avoid_obstacle,drd_heading,goal
    global ear

    pub = rospy.Publisher('mobile_base/commands/velocity', Twist,queue_size=1)
    pub_hdg_setpoint = rospy.Publisher('/hdg/setpoint',Float64,queue_size=1)
    pub_hdg_state = rospy.Publisher('/hdg/state',Float64,queue_size=1)

    sub_imu = rospy.Subscriber('/mobile_base/sensors/imu_data',Imu,callback_imu,queue_size=1)
    sub_bumper = rospy.Subscriber('/mobile_base/events/bumper',BumperEvent,callback_bumper,queue_size=1)
    sub_hdg_pid = rospy.Subscriber('/hdg/control_effort',Float64,callback_hdg_pid,queue_size=1)

    sub_odom = rospy.Subscriber('/robot_pose_ekf/odom_combined',PoseWithCovarianceStamped,callback_odom,queue_size=1)
    
    pub_log = rospy.Publisher('/my_turtle/log',String,queue_size=1)
    sub_log = rospy.Subscriber('/my_turtle/log',String,callback_log,queue_size=1)
    rate = rospy.Rate(hz)
    straight = Twist()
    straight.linear.x = linear_vel
    
    turn_left = Twist()
    turn_left.angular.z = angular_vel
    turn_left.linear.x = 0#-linear_vel/10.0
    
    turn_right = Twist()
    turn_right.angular.z = -angular_vel
    turn_right.linear.x = 0#-linear_vel/10.0
    
    stop = Twist()
    stop.angular.z = 0
    stop.linear.x = 0
    x = 0

    time.sleep(15)
    t = rospy.Time.now().to_sec()
    
    
    revs = 0
    g = 1
    goal = update_goal(g)
    drd_heading = goal_direction(goal,pose)#when going to a goal location

    while not rospy.is_shutdown():
        if revs >= 10:
            pub.publish(stop)
            return
        drd_heading = goal_direction(goal,pose)#when going to a goal location
        t_amt = angles.shortest_angular_distance(yaw,drd_heading)
        if abs(t_amt) > tolerance:
            pub.publish(turn_left)
        else:
            goal_d = goal_distance(goal,pose)
            if goal_d < 0.1:
                g = g + 1
                g = g % 4
                
                if g == 0:
                    revs = revs + 1
                goal = update_goal(g)
                
            else:
                rot_vel = hdg_ctrl_effort * hdg_scale
                if rot_vel > angular_vel: rot_vel = angular_vel
                if rot_vel < -angular_vel: rot_vel = -angular_vel

                straight.angular.z = rot_vel
            if avoid_obstacle:
                straight = stop

            log = '{},{},{},{},{}'.format(rospy.Time.now().to_sec()-t,goal_d,pose.x,pose.y,yaw)
            # print log
            if goal_d > 0.1:
                pub_log.publish(log)
            pub.publish(straight)
            # if rospy.Time.now().to_sec() - t < 90:
            #     turn_left.linear.x = 0
            #     turn_left.angular.z = 0.2
            #     pub.publish(turn_left)
            # else:
            #     pub.publish(stop)
            #     ear.close()
            # print 'straight',yaw,drd_heading,rot_vel
        set_p = drd_heading
        state_p = yaw
        if set_p < 0:
            set_p = set_p + math.pi * 2.0
        if state_p < 0:
            state_p = state_p + math.pi * 2.0
        # print 'straight',state_p,set_p,rot_vel
        pub_hdg_setpoint.publish(set_p)
        pub_hdg_state.publish(state_p)
        # print(yaw)        
        rate.sleep()
    print("Quitting")

if __name__=="__main__":
    node = rospy.init_node('my_turtle',anonymous=True)
    try:
        explore()
    except rospy.ROSInterruptException:
        pass