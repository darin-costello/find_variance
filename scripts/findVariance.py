#!/usr/bin/python

import sys, rospy
from sphero_swarm_node.msg import SpheroTwist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

import math

Accurcy = 45
MAXVEL = 60
RADIUS = 20
ALPHA = 2

SQUARESIZE = 200

class Finder():
    def __init__(self):
        print "Intitiated"

        rospy.init_node("Finder", anonymouse=True)

        self.sum = (0,0)
        self.max = 0
        self.initialize = False
        self.velPub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.index = 0
        self.path = []
        self.prev = 0
        self.posSub = rospy.Subscriber("robot_pos", Pose2D, self.control)
        self.masks = [(0,1),(1,0),(0,1),(1,0)]
        self.count = 1


    def control(self, pos):
        robPos = (pos.x, pos.y)
        if self.initialize ==True:
            distance = dist(robPos, self.path[self.index])
            if distance > RADIUS:
                dest = self.path[self.index]
                theta = math.atan2((dest[1] - robPos[1]), (dest[0] - robPos[0]))

                rob_move = Twist()
                vel = min(MAXVEL, distance * ALPHA)
                rob_move.linear.x = vel * math.cos(theta)
                rob_move.linear.y = -vel * math.sin(theta)
                self.velPub.publish(rob_move)
                self.calcDistFromLine(robPos)
            else:
                if self.index % 4 == 0:
                    print("max: %f, average, %f" % (self.max, self.sum[0]/self.sum[1]))

                if self.count % 5 == 0: #Pause to still the sphero
                    self.index = (self.index + 1) % 4
                    self.prev = (self.prev + 1) % 4

                else:
                    self.count += 1
                    rob_move = Twist()
                    self.velPub.publish(rob_move)

        else:
            self.path = [(pos.x, pos.y),
             (pos.x + SQUARESIZE, pos.y),
             (pos.x + SQUARESIZE, pos.y + SQUARESIZE),
             (pos.x, pos.y + SQUARESIZE)]
            self.index = 1
            self.prev = 0
            self.initialized ==True


    def calcDistFromLine(self, point):
        mask = self.mask(self.prev)
        distance = math.abs(dotProd(point, mask), dotProd(self.path(self.prev), mask))
        self.max = max(self.max, distance)
        self.sum = (self.sum[0] + distance, self.sum[1] + 1)

def dotProd(p1, p2):
    return (p1[0] * p2[0]) + (p1[1] * p2[1])

def dist(p1, p2):
    return math.sqrt( (p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 )
