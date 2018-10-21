#!/usr/bin/env python
'''This file/class should create a virtual wall which a robot
will continually check to see if it is within the boundary. If the robot
exits the boundary, the class should return the new direction the robot should 
turn to with respect to the obstacle avoidance behaviour'''
'''
Wall is square boundary of points ABCD shown below
            -x
            ^
            |
            |
            |
-y<----------

D             C



B             A

D         -x        C
           |
180 - 270  | 90-180
           |
           |
-y----------------------+y
           |
           |
270 - 360  | 0 - 90
           |
B         +x        A
'''
import rospy
import angles
from geometry_msgs.msg import Point
class virtual_wall(object):
    def __init__(self,l,w,c):
        self.centred = c
        self.a = Point()
        self.b = Point()
        self.c = Point()
        self.d = Point()
        if c:
            [self.a.x,self.a.y] = [l/2.0,w/2.0]
            [self.b.x,self.b.y] = [l/2.0,-w/2.0]
            [self.c.x,self.c.y] = [-l/2.0,w/2.0]
            [self.d.x,self.d.y] = [-l/2.0,-w/2.0]
        else:
            [self.a.x,self.a.y] = [l,w]
            [self.b.x,self.b.y] = [l,0]
            [self.c.x,self.c.y] = [0,w]
            [self.d.x,self.d.y] = [0,0]

    def bumper_event(self,pose,yaw):
        '''
        pose is a point object of the x,y,z location of the robot
        yaw is the angle/orientation of the robot
        returns a value that signifies whether front, left or right collision occured
        '''
        yaw = 180 * yaw / angles.pi
        if yaw < 0:
            yaw = yaw + 360
        bump = (3,False)
        loc = ('within boundary',yaw)
        if pose.x >= self.a.x and pose.y >= self.a.y:
            loc = ('edge A',yaw)
            if yaw < 180 or yaw > 270:
                bump = (1,True)
        elif pose.x >= self.b.x and pose.y <= self.b.y:
            loc = ('edge B',yaw)
            if yaw > 180 or yaw < 90:
                bump = (1,True)
        elif pose.x <= self.c.x and pose.y >= self.c.y:
            loc = ('edge C',yaw)
            if yaw > 0 and yaw < 270:
                bump = (1,True)
        elif pose.x <= self.d.x and pose.y <= self.d.y:
            loc = ('edge D',yaw)
            if yaw > 90 and yaw < 360:
                bump = (1,True)    
        elif pose.x < self.a.x and pose.y > self.a.y and pose.x > self.c.x:
            #crossed left border i.e AC
            loc = ('side AC',yaw)
            if yaw > 60 and yaw < 120:
                bump = (1,True)
            elif yaw <= 60 and yaw > 0:
                bump = (0,True)
            elif yaw >= 120 and yaw < 180:
                bump = (2,True)
        elif pose.x < self.b.x and pose.y < self.b.y and pose.x > self.d.x:
            #crossed right border i.e BD
            loc = ('side BD',yaw)
            if yaw > 240 and yaw < 300:
                bump = (1,True)
            elif yaw >= 300 and yaw < 360:
                bump = (2,True)
            elif yaw <= 240 and yaw > 180:
                bump = (0,True)
        elif pose.x > self.a.x and pose.y < self.a.y and pose.y > self.b.y:
            #crossed BOTTOM border i.e AB
            loc = ('side AB',yaw)
            if yaw > 330 or yaw < 30:
                bump = (1,True)
            elif yaw < 90 and yaw >= 30:
                bump = (2,True)
            elif yaw <= 330 and yaw > 270:
                bump = (0,True)
        elif pose.x < self.c.x and pose.y < self.c.y and pose.y > self.d.y:
            #crossed TOP border i.e DC
            loc = ('side DC',yaw)
            if yaw < 210 and yaw > 150:
                bump = (1,True)
            elif yaw >= 150 and yaw < 270:
                bump = (2,True)
            elif yaw <= 150 and yaw > 90:
                bump = (0,True)
        #  print loc,bump
        return bump + loc
        
             