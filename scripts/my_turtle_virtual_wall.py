#!/usr/bin/env python
import rospy
import angles
from geometry_msgs.msg import Twist,Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64,String, Bool
from sensor_msgs.msg import Imu
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
import random

import math
import SWHear
import time
import numpy as np
from virtual_wall import virtual_wall
import sys

class MyTurtle:
    def __init__(self,robotID,poseOffset=(0,0,0),base_prob=0,prob_multiplier=10,
                    prob_divisor=10,qSize = 1,velocity=0.1,expDuration=600,
                    goalPose=None,hear=False,theta_A=np.Inf,experimentWaitDuration=0,
                    worldWidth=5,worldLength=5,centredOrigin=True):
        print(robotID,poseOffset,base_prob,prob_divisor,prob_multiplier,qSize,velocity,expDuration,
                goalPose,hear,theta_A,experimentWaitDuration,worldLength,worldWidth,centredOrigin)
        self.robotID = robotID
        self.experimentWaitDuration = experimentWaitDuration
        self.experimentStart = False

        self.wall = virtual_wall(worldWidth,worldLength,centredOrigin)

        self.hz = 40
        if hear:
            self.ear = SWHear.SWHear(rate=44100,updatesPerSecond=self.hz)
        else:
            self.ear = None
        
        self.theta_A = theta_A
        
        random.seed(int(time.time()/(1 + poseOffset[0])))#set random seed to current time and based on self.pose
        self.linear_vel = velocity
        self.angular_vel = velocity / 1.77 # half of 0.354 diameter

        self.poseOffset = poseOffset

        self.base_prob = base_prob
        self.prob_multiplier = prob_multiplier # probability multiplier
        self.prob_divisor = prob_divisor
        self.sound_intensity_list = [] # list of sound intensity readings
        self.qSize = qSize # size of queue before updating sound intensity values (size of queue for average filter)
        self.expDuration = expDuration
        self.new_comm_signal = False

        self.mu = math.pi
        self.sigma = math.pi/2.0
        self.tolerance = 5 * math.pi / 180

        self.new_heading = 'NaN'
        self.turn_amt = 0
        self.turn = False
        self.reverseBool = False
        self.bumper = BumperEvent()
        self.avoid_obstacle = False
        self.bumper_event = False # update bound_check to initiate obstacle avoidance

        self.start = True
        self.hdg_ctrl_effort = 0
        self.hdg_scale = self.angular_vel/1000.0 #max_rot_vel/max_ctrl_effort

        if goalPose != None:
            self.goal = Point(x=goalPose[0],y=goalPose[1],z=0)
            self.goalYaw = goalPose[2]
        else:
            self.goal = None
            self.goalYaw = None
        
        self.pose = Point(x=self.poseOffset[0],y=self.poseOffset[1],z=0)
        self.yaw = 0 + self.poseOffset[2]
        self.drd_heading = self.yaw
        
        self.home = Point(x=0,y=0,z=0)
        

        # self.pose = Odometry()
        self.pkg_path = '/home/turtlebot/catkin_ws/src/my_turtle'
    
    def callback_experimentStart(self,data):
        self.experimentStart = data.data
    
    def callback_imu(self,data):

        quaternion = (data.orientation.x,
                    data.orientation.y,
                    data.orientation.z,
                    data.orientation.w)
        _, _, y = euler_from_quaternion(quaternion)
        self.yaw = y + self.poseOffset[2]
        if self.start:
            self.start = False
            self.drd_heading = self.yaw

    def callback_bumper(self,data):
        self.bumper = BumperEvent()
        self.bumper = data
        if self.bumper.state == 1 and not self.turn:
            self.bumper_event = True

    def callback_hdg_pid(self,data):
        self.hdg_ctrl_effort = data.data

    def callback_odom(self,data):
        self.pose = Point()
        self.pose.x = self.poseOffset[0] + data.pose.pose.position.x
        self.pose.y = self.poseOffset[1] + data.pose.pose.position.y

    def goal_direction(self,g,p):
        """ function to compute the orientation of robot wrt to gaol.
            g is self.goal x,y location and p is Twist of robot current self.pose"""
        # reself.turn math.atan2(g.y - p.self.pose.self.pose.position.y, g.x - p.self.pose.self.pose.position.x)
        return math.atan2(g.y - p.y, g.x - p.x)

    def goal_distance(self,g,p):
        """computes distance of robot from desired self.goal location"""
        # reself.turn math.sqrt(pow(g.x-p.self.pose.self.pose.position.x,2) + pow(g.y-p.self.pose.self.pose.position.y,2))
        return math.sqrt(pow(g.x-p.x,2) + pow(g.y-p.y,2))
    # def callback_log(data):
    #     # a = 5#do not write anything
    #     with open(self.pkg_path,'a') as f:
    #         f.write(data.data+'\n')
    def computeGrad(self,turn_prob,prev_sound,curr_sound,turn):
        ''' reads the sound signal from microphone and update the robot's self.turning probability
        reself.turns the new self.turning probability and the sound intensity measured'''

        if not self.ear.data is None and not self.ear.fft is None:
            aa = self.ear.fft

            sound_intensity = np.mean(aa) # update intensity of sound increased from last time step
            #if sound_intensity < 300:
            self.sound_intensity_list.append(sound_intensity)
            # print(sound_intensity)

        # self.sound_intensity_list.append(get_quality())
        if self.turn and False:
            self.sound_intensity_list = []
            self.turn_prob = self.base_prob
            self.new_comm_signal = False
        
        if len(self.sound_intensity_list) > 2*self.qSize and False:
            self.sound_intensity_list.pop(0)
        if len(self.sound_intensity_list) >= self.qSize:
            #sSize = int(len(self.sound_intensity_list)/2) # get half of sample length
            #curr_sound = np.mean(self.sound_intensity_list[sSize:])
            #prev_sound = np.mean(self.sound_intensity_list[:sSize])
            self.new_comm_signal = True
            curr_sound = np.mean(self.sound_intensity_list)
            # l1 = len(self.sound_intensity_list)
            self.sound_intensity_list = []
            # print(l1,len(self.sound_intensity_list))
        # print(prev_sound,curr_sound,self.turn_prob)


            
        if not self.turn and self.new_comm_signal:
            self.new_comm_signal = False
            self.turn_prob = self.base_prob
            if prev_sound > curr_sound:
                #increase self.turn probability to change orientation
                self.turn_prob = self.base_prob * self.prob_multiplier
            elif prev_sound < curr_sound:
                #reduce self.turn probability to maintain orientation
                self.turn_prob = self.base_prob / self.prob_divisor
            else:
                #leave self.turn probability as is
                self.turn_prob = self.turn_prob
            prev_sound = curr_sound # update sound intensity value

        #curr_sound = sound_intensity

        return self.turn_prob,prev_sound,curr_sound
    def goToGoal(self):
        self.goal_d = self.goal_distance(self.goal,self.pose)
        if self.goal_d < 0.1:
            goal_direction = self.goalYaw
        else:
            goal_direction = self.goal_direction(self.goal,self.pose)
        
        self.new_heading = goal_direction if goal_direction > 0 else 2 * math.pi + goal_direction#when going to a self.goal location
        self.new_heading = angles.normalize_angle(self.new_heading)
        t_amt = angles.shortest_angular_distance(self.yaw,self.new_heading)
        
        if abs(t_amt) > self.tolerance:
            self.turn = True
        return t_amt
    def turnToGoalYaw(self):
        t_amt = angles.shortest_angular_distance(self.yaw,self.goalYaw)
        if abs(t_amt) > self.tolerance:
            self.turn = True
        return t_amt

    def explore(self):

        
        pub = rospy.Publisher('/{}mobile_base/commands/velocity'.format(self.robotID), Twist,queue_size=1)
        pub_hdg_setpoint = rospy.Publisher('/{}/hdg/setpoint'.format(self.robotID),Float64,queue_size=1)
        pub_hdg_state = rospy.Publisher('/{}/hdg/state'.format(self.robotID),Float64,queue_size=1)

        sub_imu = rospy.Subscriber('/{}/mobile_base/sensors/imu_data'.format(self.robotID),Imu,self.callback_imu,queue_size=1)
        sub_bumper = rospy.Subscriber('/{}/mobile_base/events/bumper'.format(self.robotID),BumperEvent,self.callback_bumper,queue_size=1)
        sub_hdg_pid = rospy.Subscriber('/{}/hdg/control_effort'.format(self.robotID),Float64,self.callback_hdg_pid,queue_size=1)

        sub_odom = rospy.Subscriber('/{}/robot_pose_ekf/odom_combined'.format(self.robotID),PoseWithCovarianceStamped,self.callback_odom,queue_size=1)
        
        pub_bump = rospy.Publisher('/{}/my_turtle/bump_info'.format(self.robotID),String,queue_size = 1)
        sub_experimentStart = rospy.Subscriber('/experimentStart',Bool,self.callback_experimentStart,queue_size=1)
        pub_log = rospy.Publisher('/log',String,queue_size=1)
        # sub_log = rospy.Subscriber('/my_turtle/log',String,self.callback_log,queue_size=1)
        
        rate = rospy.Rate(self.hz) #looping rate
        
        straight = Twist()
        straight.angular.z = 0
        straight.linear.x = self.linear_vel

        reverse = Twist()
        reverse.angular.z = 0
        reverse.linear.x = -self.linear_vel
        
        turn_left = Twist()
        turn_left.angular.z = self.angular_vel
        turn_left.linear.x = 0
        
        turn_right = Twist()
        turn_right.angular.z = -self.angular_vel
        turn_right.linear.x = 0
        
        stop = Twist()
        stop.angular.z = 0
        stop.linear.x = 0
        x = 0

        sound_intensity = 0
        prev_sound = 0
        curr_sound = 0

        first_sound = True
        revStart = None # for holding location reverse motion started
        revD = 0.05 #  reversing distance
        self.turn_prob = self.base_prob # initial self.turn_prob is same as self.base_prob
        logheader = self.robotID + ':t,x,y,yaw,prev_sound,curr_sound,turn_prob,acTion'
        #pause for some  seconds before starting motion
        time.sleep(self.experimentWaitDuration)
        #log = logheader
        while not self.experimentStart: #busy wait till experiment start is true
            pub_log.publish(logheader)
        
        
        # mlist = []
        # mAvg = 0
        # m = 0
        if self.ear != None:
            self.ear.stream_start()
        t = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():# and x < 10 * 60 * 4
            self.goal_d = np.Inf # initially set self.goal distance to be infinite to prevent stopping

            if self.ear != None:
                self.turn_prob,prev_sound,curr_sound = computeGrad(self.turn_prob,prev_sound,curr_sound,self.turn)#last values of soundIntensity and self.turn_prob
                #chemotaxis should only be activated when sound intensity is below a threshold
                if curr_sound > self.theta_A:
                    self.turn_prob = self.base_prob
            else:
                self.turn_prob = self.base_prob
            
            
            x +=1
            bound_check = 3
            if not self.turn:
                (bound_check,self.avoid_obstacle,bumpside,bumpyaw) = self.wall.bumper_event(self.pose,self.yaw)
                if self.bumper_event:
                    self.avoid_obstacle = self.bumper_event
                    bound_check = self.bumper
                    self.bumper_event = False

                pub_bump.publish('bumper={},avoid={},{},({},{},{})'.format(bound_check,self.avoid_obstacle,bumpside,self.pose.x,self.pose.y,bumpyaw))
            
            if self.goal != None and self.base_prob == 0:#if goal directed behaviour, go toward goal
                    t_amt = self.goToGoal()
            
            elif rospy.Time.now().to_sec()-t >= self.expDuration:# if time limited experiment
                # Experiment duration reached. Go self.home.
                if self.ear != None:
                    self.ear.close()
                if self.goal == None:#if goal is not to return to a specific goal pose
                    break
                else:
                    t_amt = self.goToGoal()

            elif random.random() < self.turn_prob and not self.turn and not self.avoid_obstacle:
                # perform random walk
                self.turn = True
                temp_heading = self.yaw + angles.normalize_angle(random.gauss(self.mu,self.sigma))
                self.new_heading = angles.normalize_angle(temp_heading) #% 360
                # self.self.turn_amt = angles.shortest_angular_distance(self.yaw,self.new_heading)# (self.new_heading - self.yaw) % 360
            # if self.avoid_obstacle or True:
            #     y = self.yaw / angles.pi * 180
            #     if y < 0:
            #         y = y + 360

                # print bound_check,self.avoid_obstacle,self.pose.x,self.pose.y,y
            if bound_check != 3 and self.avoid_obstacle and not self.turn:
                #boundary obstacle avoidance behaviour
                if bound_check == 0: #left obstacle
                    temp_heading = self.yaw - math.pi/4.0
                elif bound_check == 2: #right obstacle
                    temp_heading = self.yaw + math.pi/4.0
                else: #front obstacle
                    temp_heading = self.yaw + random.random() * math.pi + math.pi/2.0
                #print self.bumper.state,'self.bumper',self.bumper.self.bumper,temp_heading
                self.new_heading = angles.normalize_angle(temp_heading) #% 360
                self.turn_amt = angles.shortest_angular_distance(self.yaw,self.new_heading)# (self.new_heading - self.yaw) % 360
                self.turn = True
                self.reverseBool = True
                revStart = self.pose
                self.avoid_obstacle = False
            
            # self.drd_heading = self.goal_direction(self.goal,self.pose)#when going to a self.goal location
            acTion = 'straight'
            if self.reverseBool and revStart != None:
                acTion = 'reverse'
                #reverse for at lease revD distance before self.turning
                pub.publish(reverse)
                if self.goal_distance(revStart,self.pose) >= revD:
                    revStart = None
                    self.reverseBool = False
            elif self.turn:
                self.drd_heading = self.new_heading
                t_amt = angles.shortest_angular_distance(self.yaw,self.new_heading)
                if t_amt < 0:
                    acTion = 'turnRight'
                    pub.publish(turn_right)
                    # print 'tr',self.yaw,self.new_heading,self.self.turn_amt
                else:
                    acTion = 'turnLeft'
                    pub.publish(turn_left)
                    # print 'tl',self.yaw,self.new_heading,self.self.turn_amt
                if abs(t_amt) < self.tolerance:
                    self.turn = False
                    self.avoid_obstacle = False
            elif self.goal_d > 0.1:
                # self.goal_d = self.goal_distance(self.goal,self.pose)
                # rospy.loginfo("self.goal_d = %.2f, x = %.2f, y = %.2f, intensity = %.2f",self.goal_d,self.pose.x,self.pose.y,sound_intensity)
                # print rospy.Time.now().to_sec()
                
                rot_vel = self.hdg_ctrl_effort * self.hdg_scale
                
                if rot_vel > self.angular_vel: rot_vel = self.angular_vel
                if rot_vel < -self.angular_vel: rot_vel = -self.angular_vel

                straight.angular.z = rot_vel
                # if self.avoid_obstacle:
                #     straight = stop
                pub.publish(straight)
            else:
                #t_amt = self.turnToGoalYaw()
                if self.goal_d <= 0.1 and abs(t_amt) < self.tolerance:
                    acTion = 'stop'
                    #stop if self.goal (i.e. self.home in go self.home behaviour) is reached
                    pub.publish(stop)
                    break
                

                # print 'straight',self.yaw,self.drd_heading,rot_vel
            set_p = self.drd_heading
            state_p = self.yaw
            if set_p < 0:
                set_p = set_p + math.pi * 2.0
            if state_p < 0:
                state_p = state_p + math.pi * 2.0
            # print 'straight',state_p,set_p,rot_vel
            pub_hdg_setpoint.publish(set_p)
            pub_hdg_state.publish(state_p)
            # print(self.yaw)        
            log = '{}:{},{},{},{},{},{},{}'.format(self.robotID,self.pose.x,self.pose.y,self.yaw,prev_sound,curr_sound,self.turn_prob,acTion)#,m,mAvg)
            pub_log.publish(log)
            rospy.loginfo(log)
            
            rate.sleep()
        print("Quitting")

if __name__=="__main__":
    robotID = sys.argv[1]
    poseOffset=eval(sys.argv[2])
    base_prob=eval(sys.argv[3])
    prob_multiplier=eval(sys.argv[4])
    prob_divisor=eval(sys.argv[5])
    qSize=eval(sys.argv[6])
    velocity=eval(sys.argv[7])
    expDuration=eval(sys.argv[8])
    goalPose=eval(sys.argv[9])
    hear=eval(sys.argv[10])
    theta_A=eval(sys.argv[11])
    experimentWaitDuration=eval(sys.argv[12])
    worldWidth=eval(sys.argv[13])
    worldLength=eval(sys.argv[14])
    centredOrigin=eval(sys.argv[15])
    
    turtle = MyTurtle(robotID=robotID,
                    poseOffset=poseOffset,
                    base_prob=base_prob,
                    prob_multiplier=prob_multiplier,
                    prob_divisor=prob_divisor,
                    qSize=qSize,
                    velocity=velocity,
                    expDuration=expDuration,
                    goalPose=goalPose,
                    hear=hear,
                    theta_A=theta_A,
                    experimentWaitDuration=experimentWaitDuration,
                    worldWidth=worldWidth,
                    worldLength=worldLength,
                    centredOrigin=centredOrigin)
    # go to goal with 0 turn probability
    # turtle = MyTurtle(poseOffset=(0,0,0),goalPose=(15,0,0),hear=True,qSize=1)

    # do random walk and go to goal location
    # turtle = MyTurtle(goalPose=(2,0,0),base_prob=0.0025,expDuration=100)

    # do random walk for limited time
    # turtle = MyTurtle(goalPose=None,base_prob=0.0025,expDuration=100)

    # nest attraction for limited time
    # turtle = MyTurtle(poseOffset=(0,0,0),base_prob=0.0025,qSize=40,hear=True)

    node = rospy.init_node('my_turtle',anonymous=True)
    try:
        turtle.explore()

    except rospy.ROSInterruptException:
        pass
