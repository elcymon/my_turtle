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
from virtual_wall import virtual_wall

wall = virtual_wall(2,2,True)

hz = 40
ear = SWHear.SWHear(rate=44100,updatesPerSecond=hz)
freq = 500

linear_vel = 0.1
angular_vel = 0.1

yaw = 0
base_prob = 1.0/(hz*10)
mu = math.pi
sigma = math.pi/2.0
tolerance = 3 * math.pi / 180

new_heading = 'NaN'
turn_amt = 0
turn = False
reverseBool = False
bumper = BumperEvent()
avoid_obstacle = False
bumper_event = False # update bound_check to initiate obstacle avoidance

drd_heading = 0
start = True
hdg_ctrl_effort = 0
hdg_scale = angular_vel/100.0 #max_rot_vel/max_ctrl_effort

goal = Point()
pose = Point()
home = Point()
home.x = 0
home.y = 0
home.z = 0

# pose = Odometry()
log_filename = '/home/turtlebot/catkin_ws/src/my_turtle/results/'+time.strftime('%Y%m%d%H%M%S') + 'data.txt'

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
    global bumper, bumper_event
    bumper = BumperEvent()
    bumper = data
    if bumper.state == 1 and not turn:
        bumper_event = True

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
    # a = 5#do not write anything
    with open(log_filename,'a') as f:
        f.write(data.data+'\n')
def computeGrad(prev_sound):
    ''' reads the sound signal from microphone and update the robot's turning probability
    returns the new turning probability and the sound intensity measured'''
    sound_intensity = prev_sound #assume there is not change in intensity
    turn_prob = base_prob

    if not ear.data is None and not ear.fft is None:
            aa = ear.fft
            
            sound_intensity = np.mean(aa) # update intensity of sound increased from last time step
            
    if prev_sound > sound_intensity:
        #increase turn probability to change orientation
        turn_prob *= 10
    elif prev_sound < sound_intensity:
        #reduce turn probability to maintain orientation
        turn_prob /= 10
    else:
        #leave turn probability as is
        turn_prob *= 1
    
    return turn_prob,sound_intensity
    
def explore(hear=True):
    global turn,new_heading,turn_amt,avoid_obstacle,drd_heading,goal,bumper_event
    global ear
    global reverseBool,revStart

    if hear:
        ear.stream_start()

    pub = rospy.Publisher('mobile_base/commands/velocity', Twist,queue_size=1)
    pub_hdg_setpoint = rospy.Publisher('/hdg/setpoint',Float64,queue_size=1)
    pub_hdg_state = rospy.Publisher('/hdg/state',Float64,queue_size=1)

    sub_imu = rospy.Subscriber('/mobile_base/sensors/imu_data',Imu,callback_imu,queue_size=1)
    sub_bumper = rospy.Subscriber('/mobile_base/events/bumper',BumperEvent,callback_bumper,queue_size=1)
    sub_hdg_pid = rospy.Subscriber('/hdg/control_effort',Float64,callback_hdg_pid,queue_size=1)

    sub_odom = rospy.Subscriber('/robot_pose_ekf/odom_combined',PoseWithCovarianceStamped,callback_odom,queue_size=1)
    
    pub_log = rospy.Publisher('/my_turtle/log',String,queue_size=1)
    pub_bump = rospy.Publisher('/my_turtle/bump_info',String,queue_size = 1)
    sub_log = rospy.Subscriber('/my_turtle/log',String,callback_log,queue_size=1)
    
    rate = rospy.Rate(hz) #looping rate
    
    straight = Twist()
    straight.angular.z = 0
    straight.linear.x = linear_vel

    reverse = Twist()
    reverse.angular.z = 0
    reverse.linear.x = -linear_vel
    
    turn_left = Twist()
    turn_left.angular.z = angular_vel
    turn_left.linear.x = 0
    
    turn_right = Twist()
    turn_right.angular.z = -angular_vel
    turn_right.linear.x = 0
    
    stop = Twist()
    stop.angular.z = 0
    stop.linear.x = 0
    x = 0

    # goal.x = 3
    # goal.y = 0
    sound_intensity = 0
    prev_sound = 0
    time.sleep(15)
    t = rospy.Time.now().to_sec()
    first_sound = True
    revStart = None # for holding location reverse motion started
    revD = 0.1 #  reversing distance
    expDuration = 100
    
    while not rospy.is_shutdown():# and x < 10 * 60 * 4
        goal_d = np.Inf # initially set goal distance to be infinite to prevent stopping

        if hear:
            turn_prob,prev_sound = computeGrad(prev_sound)
        else:
            turn_prob = base_prob
        
        x +=1
        bound_check = 3
        if not turn:
            (bound_check,avoid_obstacle,bumpside,bumpyaw) = wall.bumper_event(pose,yaw)
            if bumper_event:
                avoid_obstacle = bumper_event
                bound_check = bumper.bumper
                bumper_event = False

            pub_bump.publish('bumper={},avoid={},{},({},{},{})'.format(bound_check,avoid_obstacle,bumpside,pose.x,pose.y,bumpyaw))
        # if endExperiment:
        #     print 'end experiment' 
        if rospy.Time.now().to_sec()-t >= expDuration and not avoid_obstacle and not turn:
            # Experiment duration reached. Go home.
            new_heading = goal_direction(home,pose)#when going to a goal location
            goal_d = goal_distance(home,pose)
            t_amt = angles.shortest_angular_distance(yaw,new_heading)

            if abs(t_amt) > tolerance:
                turn = True
        elif random.random() < turn_prob and not turn and not avoid_obstacle:
            # perform random walk
            turn = True
            temp_heading = yaw + random.gauss(mu,sigma)
            new_heading = angles.normalize_angle(temp_heading) #% 360
            # turn_amt = angles.shortest_angular_distance(yaw,new_heading)# (new_heading - yaw) % 360
        # if avoid_obstacle or True:
        #     y = yaw / angles.pi * 180
        #     if y < 0:
        #         y = y + 360

            # print bound_check,avoid_obstacle,pose.x,pose.y,y
        if bound_check != 3 and avoid_obstacle and not turn:
            #boundary obstacle avoidance behaviour
            if bound_check == 0: #left obstacle
                temp_heading = yaw - math.pi/4.0
            elif bound_check == 2: #right obstacle
                temp_heading = yaw + math.pi/4.0
            else: #front obstacle
                temp_heading = yaw + random.random() * math.pi + math.pi/2.0
            #print bumper.state,'bumper',bumper.bumper,temp_heading
            new_heading = angles.normalize_angle(temp_heading) #% 360
            turn_amt = angles.shortest_angular_distance(yaw,new_heading)# (new_heading - yaw) % 360
            turn = True
            reverseBool = True
            revStart = pose
            avoid_obstacle = False
        
        # drd_heading = goal_direction(goal,pose)#when going to a goal location

        if reverseBool and revStart != None:
            #reverse for at lease revD distance before turning
            pub.publish(reverse)
            if goal_distance(revStart,pose) >= revD:
                revStart = None
                reverseBool = False
        elif turn:
            drd_heading = new_heading
            t_amt = angles.shortest_angular_distance(yaw,new_heading)
            if t_amt < 0:
                pub.publish(turn_right)
                # print 'tr',yaw,new_heading,turn_amt
            else:
                pub.publish(turn_left)
                # print 'tl',yaw,new_heading,turn_amt
            if abs(t_amt) < tolerance:
                turn = False
                avoid_obstacle = False
        else:
            # goal_d = goal_distance(goal,pose)
            # rospy.loginfo("goal_d = %.2f, x = %.2f, y = %.2f, intensity = %.2f",goal_d,pose.x,pose.y,sound_intensity)
            # print rospy.Time.now().to_sec()
            if goal_d < 0.1:
                #stop if goal (i.e. home in go home behaviour) is reached
                straight.linear.x = 0
                rot_vel = 0
                break
            else:
                rot_vel = hdg_ctrl_effort * hdg_scale
            
            if rot_vel > angular_vel: rot_vel = angular_vel
            if rot_vel < -angular_vel: rot_vel = -angular_vel

            straight.angular.z = rot_vel
            # if avoid_obstacle:
            #     straight = stop
            pub.publish(straight)

        log = '{},{},{},{},{},{}'.format(rospy.Time.now().to_sec()-t,pose.x,pose.y,yaw,sound_intensity,turn_prob)
        pub_log.publish(log)
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
        explore(hear=False)
    except rospy.ROSInterruptException:
        pass