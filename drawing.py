import rospy
import std_msgs
import roslib
#roslib.load_manifest("baxter_pick_and_place")
import readline
import math
from ein.msg import EinState
import numpy as np
from sensor_msgs.msg import Range

readline.parse_and_bind('tab: complete')
readline.parse_and_bind('set editing-mode emacs')

class Draw:
	'''
	alphabet is a dictionary, where the keys are letters and the values are the schematics for drawing. Each schematic consists of a list of strokes, where a stroke is represented by
	a tuple of it's starting and ending point. Each point is represented by a x and y position.
	'''
	alphabet = {'a':[[[.008,-.008],[-.008,0]],[[-.008,0],[.008,.008]],[[0,-.004],[0,.004]]],
		    'b':[[[.008,-.008],[.004,.008]],[[.004,.008],[0,-.008]],[[0,-.008],[-.004,.008]],[[-.004,.008],[-.008,-.008]],[[.008,-.008],[-.008,-.008]]],
		    'c':[[[-.008,.008],[-.008,-.008]],[[-.008,-.008],[.008,-.008]],[[.008,-.008],[.008,.008]]],
		    'd':[[[-.008,-.008],[0,.008]],[[0,.008],[.008,-.008]],[[.008,-.008],[-.008,-.008]]],
		    'e':[[[.008,-.008],[.008,.008]],[[0,-.008],[0,.008]],[[-.008,-.008],[-.008,.008]],[[.008,-.008],[-.008,-.008]]],
		    'f':[[[0,-.008],[0,.008]],[[-.008,-.008],[-.008,.008]],[[.008,-.008],[-.008,-.008]]],
		    'g':[[[-.008,0],[0,-.008]],[[0,-.008],[.008,0]],[[.008,0],[0,.006]],[[0,.008],[0,.004]]],
		    'h':[[[0,-.008],[0,.008]],[[.008,-.008],[-.008,-.008]],[[.008,.008],[-.008,.008]]],
		    'i':[[[.008,-.008],[.008,.008]],[[-.008,0],[.008,0]],[[-.008,-.008],[-.008,.008]]],
		    'j':[[[-.008,-.008],[-.008,.008]],[[-.008,0],[.008,0]],[[.008,0],[0,-.008]]],
		    'k':[[[0,-.008],[.008,.008]],[[0,-.008],[-.008,.008]],[[.008,-.008],[-.008,-.008]]],
		    'l':[[[.008,-.008],[.008,.008]],[[.008,-.008],[-.008,-.008]]],
		    'm':[[[.008,-.008],[-.008,-.008]],[[-.008,-.008],[.008,0]],[[.008,0],[-.008,.008]],[[-.008,.008],[.008,.008]]],
		    'n':[[[.008,-.008],[-.008,-.008]],[[-.008,-.008],[.008,.008]],[[.008,.008],[-.008,.008]]],
		    'o':[[[-.008,0],[0,-.008]],[[0,-.008],[.008,0]],[[.008,0],[0,.008]],[[0,.008],[-.008,0]]],
		    'p':[[[-.008,-.008],[-.004,.008]],[[-.004,.008],[0,-.008]],[[-.008,-.008],[.008,-.008]]],
		    'q':[[[-.008,0],[0,-.008]],[[0,-.008],[.008,0]],[[.008,0],[0,.008]],[[0,.008],[-.008,0]],[[.002,.002],[.008,.008]]],
		    'r':[[[-.008,-.008],[-.004,.008]],[[-.004,.008],[0,-.008]],[[-.008,-.008],[.008,-.008]],[[0,-.008],[.008,.008]]],
		    's':[[[-.004,.008],[-.008,0]],[[-.008,0],[-.004,-.008]],[[-.004,-.008],[.004,.008]],[[.004,.008],[.008,0]],[[.008,0],[.004,-.008]]],
		    't':[[[-.008,-.008],[-.008,.008]],[[-.008,0],[.008,0]]],
		    'u':[[[-.008,-.008],[.008,-.008]],[[.008,-.008],[.008,.008]],[[.008,.008],[-.008,.008]]],
		    'v':[[[-.008,-.008],[.008,0]],[[.008,0],[-.008,.008]]],
		    'w':[[[-.008,-.008],[.008,-.008]],[[.008,-.008],[-.008,0]],[[-.008,0],[.008,.008]],[[.008,.008],[-.008,.008]]],
		    'x':[[[-.008,-.008],[.008,.008]],[[.008,-.008],[-.008,.008]]],
		    'y':[[[-.008,-.008],[0,0]],[[-.008,.008],[0,0]],[[0,0],[.008,0]]],
		    'z':[[[-.008,-.008],[-.008,.008]],[[-.008,.008],[.008,-.008]],[[.008,-.008],[.008,.008]]]}
	def __init__(self,publish_topic,r,gridSize):
		'''
		Initializes all the basic neccesities for drawing.
		@public_topic: The topic the commands will be pushed to. In this case, either the left or right arm of Baxter.
		@r: The radius from the focal point that the utensil is at (With the current utensil, this is 0)
		@gridSize: The gridsize units of the grid that Baxter is basing it's drawing off of.
		'''
		self.forth_command_publisher = rospy.Publisher(publish_topic,std_msgs.msg.String, queue_size=10) #Creates a publisher to the topic (One of the arms)
		self.r = r #Sets the distance of utensil from focal point
		self.gridSize = gridSize #Sets the grid size

		self.restingHeight = .31 #**The height, in meters, from the canvas
		self.drawGap = 15 #How much Baxter moves it's end effector down to the canvas

		self.transferSpeed = .5 #The speed at which Baxter moves it's end effector when transfering the tool above the canvas
		self.carefulSpeed = .2 #The speed at which Baxter raises it's end effector after drawing
		self.drawingSpeed = .2 #The speed at which Baxter's end effector moves while drawing
		
		self.forth_command_publisher.publish(str(self.gridSize) + " setGridSize") #Sets the gridSize
		rospy.sleep(.1)
		self.forth_command_publisher.publish(str(self.transferSpeed) + " setSpeed") #Sets the current speed of Baxter's end effector
		rospy.sleep(.1)
		self.forth_command_publisher.publish("ikModeIkFast") #Sets the IK mode to Fast, for best control over end effector.
		rospy.sleep(.1)
		#self.forth_command_publisher.publish(".03 setW1GoThresh")
		#rospy.sleep(.1)
	def relativeAngle(self,x1,y1,x2,y2):
		'''
		Calculates the relative angle between the two points (x1,y1) and (x2,y2) relative to the grid.
		'''
		ang = math.atan2((y2 -y1),(x2-x1))
		if ang < 0:
			ang = (2*math.pi) + ang
		return ang*-1
	def linearEEPose(self,x,y,radians):
		'''
		Calculates the correct pose for the pen to be at given a position (x,y) and desired orientation
		@x: The x position of the desired end effector position
		@y: the y position of the desired enf effector position
		@radians: the angle at which the utensil should be held
		
		@return: The exact command to to move the end effector to the desired position and orientation
		'''
		radians = radians/2
		x0 = math.sin(radians)
		y0 = math.cos(radians)

		return str(x) + " " + str(y) + " " + str(self.restingHeight) + " " + str(x0) + " " + str(y0) + " 0 0 moveToEEPose "
	def putPen(self,x1,y1,x2,y2):
		'''
		Puts the pen down at (x1,y1) oriented at (x2,y2)
		'''
		command_block = ""
		
		#Change speed to transfer speed
		command_block += str(self.transferSpeed) + " setSpeed "
		#Move EE to above (x1,y1), oriented in the direction of (x2,y2)
		command_block += self.linearEEPose(x1,y1,self.relativeAngle(x1,y1,x2,y2)) + "waitUntilAtCurrentPosition "
		#Angle pen
		#command_block += "( oXUp ) 40 replicateWord waitUntilAtCurrentPosition "
		#Change speed so pen can touch paper
		command_block += str(self.carefulSpeed) + " setSpeed "
		#Move down to paper
		command_block += "( zDown waitUntilAtCurrentPosition ) " + str(self.drawGap) + " replicateWord "
		command_block += "1 waitForSeconds "
		#Change speed to drawing speed
		command_block += str(self.drawingSpeed) + " setSpeed "

		return command_block
	def raisePen(self):
		'''
		Raises the utensil from the table
		'''
		command_block = ""
		#change to carefulspeed
		command_block += str(self.transferSpeed) + " setSpeed "
		#move pen back up
		command_block += "( zUp ) " + str(self.drawGap) + " replicateWord waitUntilAtCurrentPosition "
		return command_block
	def drawLine(self,x1,y1,x2,y2):
		'''
		Draws the line from (x1,y1) to (x2,y2)
		'''
		command_block = ""
		
		#Change speed to drawing speed
		command_block += str(self.drawingSpeed) + " setSpeed "

		#Calculate the distance to move and then move in that direction by that much
		dist = math.hypot(x2-x1,y2-y1)/(self.gridSize)
		
	 	command_block += "( localXDown waitUntilAtCurrentPosition ) " + str(int(dist)) + " replicateWord "

		command_block += "1 waitForSeconds "
		#command_block += "( oYDown ) 20 replicateWord ( zUp ) 5 replicateWord waitUntilAtCurrentPosition "

		return command_block
	def calibrate(self):
		'''
		A function to test basic calibration. Baxter will draw a plus sign wth 4 strokes, where each stroke starts at one of the ends and ends at the middle.
		'''
		cm = ""
		cm += self.putPen(.6,.6,.6,.5)
		cm += self.raisePen()
		cm += self.putPen(.6,.6,.6,.7)
		cm += self.raisePen()
		cm += self.putPen(.6,.6,.5,.6)
		cm += self.raisePen()
		cm += self.putPen(.6,.6,.7,.6)
		cm += self.raisePen()
		return cm

	
	def drawSentence(self,x,y,scaling,theta,sentence):
		'''
		Writes a sentence
		@x: The x position at which the sentence starts
		@y: The y position at which the sentence starts
		@scaling: The scalar multiplier for how large the text is
		@theta: The angle at which the text gets written
		@sentence: The string of characters to write
		'''
		cm = ""
		rotationM = np.matrix(str(math.cos(theta)) + " " + str(-1*math.sin(theta)) + "; " + str(math.sin(theta)) + " " + str(math.cos(theta))) #Affine transformation matrix with scaling/orientation
		for letter in sentence:
			for line in self.alphabet[letter]:
				
				p1 = np.matrix(str(line[0][0]) + "; " + str(line[0][1])) * scaling #Grabs the starting point of the stroke
				p2 = np.matrix(str(line[1][0]) + "; " + str(line[1][1])) * scaling #Grabs the ending point of the stroke
				p1 = np.dot(rotationM,p1) #applies the affine transformation
				p2 = np.dot(rotationM,p2) #applies the affine transformation

				cm += self.putPen(p1[0,0]+x,p1[1,0]+y,p2[0,0]+x,p2[1,0]+y)
				cm += self.drawLine(p1[0,0]+x,p1[1,0]+y,p2[0,0]+x,p2[1,0]+y)
				cm += self.raisePen()
			y = y+(.02*scaling)*math.cos(theta) #Offsets for next letter
			x = x+(.02*scaling)*math.sin(theta) #Offsets for next letter
			
			
			"""p1 = np.dot(rotationM,ord(letter)[0][0]*scaling)
			p2 = np.dot(rotationM,ord(letter)[0][1]*scaling)
			cm += self.putPen(self.alphabet[ord(letter)][0][0][0,0],self.alphabet[ord(letter)][0][0][1,0],self.alphabet[ord(letter)][0][1][0,0],self.alphabet[ord(letter)][0][1][1,0])
			for pointPairs in self.alphabet[ord(letter)]:
				p1 = np.dot(rotationM,pointPairs[0]*scaling)
				p2 = np.dot(rotationM,pointPairs[1]*scaling)
				cm += self.drawLine(p1[0,0]+x,p1[1,0]+y,p2[0,0]+x,p2[1,0]+y)"""
		return cm
		
		
def main():
	cm = "" #The command that will be sent to Baxter
	import sys
	arm = "left" #The arm that will be drawing
	rospy.init_node("ein_client_%s" % arm, anonymous=True) #Initialize the arm
        radius = 0 #The radius from how far the utensil is from the focal point
	gridSize = .01 #**The units, in meters(?), at which the grid is built	
	client = Draw("/ein/%s/forth_commands" % arm,radius,gridSize) #Initializes all the necessary properties to behin drawing

	#cm += client.calibrate()
	#cm = client.putPen(.5,.7,.65,.8)
	#cm += client.drawLine(.5,.7,.65,.8)
	#cm += client.raisePen()
	cm += client.drawSentence(.7,.5,5,0,'z') #Draws the letter "a" starting at (.7,.5) with a scaled size of 5 at an angle of 0.
	client.forth_command_publisher.publish(cm) #Publish the command
	#print cm
	
if __name__=='__main__':
	main()	
