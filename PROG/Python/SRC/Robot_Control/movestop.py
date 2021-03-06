#!/usr/bin/env python

#import roslib; roslib.load_manifest("dynamixel_hr_ros")
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from dynamixel_hr_ros.msg import ChainState, CommandPosition
import sys
sys.path.insert(1,'/home/pi/catkin_ws/src/dynamixel_hr_ros/')
from dxl import *
import numpy as np
import re
import RPi.GPIO as RG
        
def parsser(output):
	data_file = open(output,'r')
	ang_off = np.zeros([6,3],float)
	direction = np.zeros([6,3],float)
	ang_check = np.zeros([6,6],float)
	for line_raw in data_file:
            line = line_raw[:-1]
            if "AO" in line.split('&'):
                ang_off = np.array([float(value) for value in line.split('&')[1:]]).reshape((6,3))
            elif "D" in line.split('&'):
                direction = np.array([float(value) for value in line.split('&')[1:]]).reshape((6,3))
            elif "AC" in line.split('&'):
                ang_check = np.array([float(value) for value in line.split('&')[1:]]).reshape((6,6))
            elif "IDS" in line.split('&'):
                ids = np.array([float(value) for value in line.split('&')[1:]]).reshape((6,3)).tolist()
            else:
                print "Unknown entry: {0}".format(line)
	data_file.close()
	return ang_off , direction , ang_check, ids


class Robot_control:
    
    def __init__(self):

        self.status = 'STOPPED'
        RG.setmode(RG.BCM)
        RG.setup(14, RG.OUT)
        self.AnglesDict = {}

        self.Delta = 1.
	
        self.angles_off , self.direction , self.angles_check, self.legs_motors_ids = parsser('/home/pi/Robot_Control/angles.txt')
        
        rospy.init_node('angle_ctrl')

        self.publisher_status = rospy.Publisher('status', String ,queue_size =1)
        self.publisher_legs = rospy.Publisher("/dxl/command_position", CommandPosition,queue_size =1)
        self.enabler=rospy.Publisher("/dxl/enable",Bool,queue_size =1)
        self.enabler.publish(True)
        
        self.subscriber_status = rospy.Subscriber('status', String, self.update_status)
        self.subscriber_status = rospy.Subscriber('led', Int8, self.SwitchLight)
        self.subscriber_delta = rospy.Subscriber('delta', Float32, self.UpdateDelta)
        rospy.Subscriber("/dxl/chain_state",ChainState, self.UpdateAngles)
        [rospy.Subscriber("angles_raw_leg_{0}".format(i), numpy_msg(Floats), self.check_angles, i) for i in range(6)]
		

        print "Initialization done. Running..."
        rospy.spin()

    def SwitchLight(self, message):
        if message.data == 1:
            RG.output(14, RG.HIGH)
        else:
            RG.output(14, RG.LOW)

    def UpdateDelta(self, message):
        self.Delta = message.data

    def UpdateAngles(self, message):
        for n_id in range(len(message.id)):
            self.AnglesDict[message.id[n_id]] = message.angle[n_id]

    def legation(self, angles, angles_off, direction, angles_check):
        '''etape 1'''
        '''angles = vecteur contenant les angles calcules pour le deplacement'''		
        angles = np.copy(angles)
        
        angles[2] = angles[2] - angles[1]
        
        '''etape 2&3'''
        '''angles_off = vecteur contenant les offsets des angles des moteurs'''
        '''direction = vecteur de 1 et -1 indiquant le sens'''
        angles[0] = angles[0]*direction[0] + angles_off[0]
        angles[1] = angles[1]*direction[1] + angles_off[1]
        angles[2] = angles[2]*direction[2] + angles_off[2]
        
        '''etape 4'''
        '''angles_check = vecteur contenant les angles min et max theoriquement possible'''
        '''             = (alphamin,betamin,gammamin,alphamax,betamax,gammamax)'''
        angles_check = np.copy(angles_check)
        if  ( (angles_check[:3]>angles).any() ) or ( (angles_check[3:]<angles).any() ):
            '''stop'''
            self.status = 'status'
            self.publisher_status.publish('ERROR')
            ''' de base vaut 0'''        
        return angles
		
    def update_status(self, status):
        print "Status updated to {0}".format(status)
        self.status = status.data

    def check_angles(self, angles_msg, numleg):
        '''Extraction donnees fonction de la patte commandee'''
        print "Received new data from leg {0}".format(numleg)
        ang_off = self.angles_off[numleg,:]
        direction = self.direction[numleg,:]
        ang_check = self.angles_check[numleg,:]
        
        angles = self.legation(angles_msg.data, ang_off, direction, ang_check)
		
        if self.status != 'ERROR' or self.status != 'STOP':
            command=CommandPosition()
            command.id=self.legs_motors_ids[numleg]
            command.angle=angles
            command.speed=[min(1.5,max(0.3,0.95*abs(angles[n_id] - self.AnglesDict[self.legs_motors_ids[numleg][n_id]])/self.Delta)) for n_id in range(3)]
            self.publisher_legs.publish(command)

R = Robot_control()
