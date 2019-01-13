import wpilib
import math
import time
from magicbot.state_machine import AutonomousStateMachine, state

from automations.motion import ChassisMotion
from components.vision import Vision
from pyswervedrive.chassis import Chassis
# these imports won't work yet


class LeftStartAuto(AutonomousStateMachine):
    MODE_NAME = "LEFT_START_AUTO"
    chassis: Chassis
    motion: ChassisMotion

    #ship_deposit = (5.5, 0.2) 

    @state(first=True)
    def drive_to_cargoship(self, initial_call):
        if initial_call:	
        	self.chassis.set_inputs(3, 0, 0)
        	if self.chassis.odometry_x > 3:
        		self.chassis.set_inputs(0, 0, 0)
        		self.next_state_now("align_for_deployment")

@state
def drive_to_loading_bay(self):
	x = 1
	#first pass
	if x == 1:
		self.chassis.set_inputs(0, -2, math.pi)
		if self.chassis.odometry_y< -2:
			self.chassis.set_inputs(0,0,math.pi)
			if: #is facing towards the loading bay
				self.chassis.set_inputs(-4,-0.5,0)
				if self.chassis.odometry_x < -5.6:
					self.chassis.set_inputs(0,0,0)
					x+=1
					self.next_state_now("align_for_intake")
		elif #is facing towards the loading bay
			self.chassis.set_inputs(0,-2,0)
			if self.chassis.odometry_y < -2:
				self.set_inputs(-4,-0.5,0)
				if self.chassis.odometry_x < -5.6:
					self.chassis.set_inputs(0,0,0)
					x+=1
					self.next_state_now("align_for_intake")
	#second pickup
	elif x == 2:
	self.chassis.set_inputs(-3.25, -1.2, (math.pi/2)*-1)
		if #facing loading bay
			self.chassis.set_inputs(-3.25, -1.2, 0)
				if self.chassis.odometry_x < -6.5:
					self.chassis.set_inputs(0, -1.2,0)
					if self.chassis.odometry_y < -2.4:
						self.chassis.set_inputs(0,0,0)
						x+=1
						self.next_state_now("align_for_intake")
				elif self.chassis.odometry_y < -2.4:
					self.chassis.set_inputs(-3.25,0,0)
					if self.chassis.odometry_x < -6.5:
						self.chassis.set_inputs(0,0,0)
						x+=1
						self.next_state_now("align_for_intake")
			elif self.chassis.odometry_y < -2.4:
				self.chassis.set_inputs(-3.25,0,(math.pi/2)*-1)
				if self.chassis.odometry_x < -6.5:
					self.chassis.set_inputs(0,0,(math.pi/2)*-1)
					if #facing loading bay
						self.chassis.set_inputs(0,0,0)
						x+=1
						self.next_state_now("align_for_intake")
				elif #facing loading bay
					self.chassis.set_inputs(-3.25,0,0)
					if self.chassis.odometry_x:
						self.chassis.set_inputs(0,0,0)
						x+=1
						self.next_state_now("align_for_intake")

			elif self.chassis.odometry_x < -6.5:
				self.chassis.set_inputs(0,-1.2,(math.pi/2)*-1)
				if #facing loading bay
					self.set_inputs(0,-1.2,0)
					if self.chassis.odometry_y > 2.4:
						self.chassis.set_inputs(0,0,0)
						x+=1
						self.next_state_now("align_for_intake")
				elif self.chassis.odometry_y > 2.4:
					self.chassis.set_inputs(0,0,(math.pi/2)*-1)
					if #facing loading bay
						self.chassis.set_inputs(0,0,0)
						x+=1
						self.next_state_now("align_for_intake")


		#getting back from side bays 
	#if near loading bay
		#self.next_state_now("align_for_intake")
pass

@state
def drive_to_side_cargo_bay(self):
y = 1
#first pass
if y == 1:
	self.chassis.set_inputs(3.25, 1.2, math.pi/2)
	if #facing side cargo bay
		self.chassis.set_inputs(3.25, 1.2, 0)
		if self.chassis.odometry_x > 6.5:
			self.chassis.set_inputs(0, 1.2,0)
			if self.chassis.odometry_y > 2.4:
				self.chassis.set_inputs(0,0,0)
				y+=1
				self.next_state_now("align_for_deployment")
		elif self.chassis.odometry_y > 2.4:
			self.chassis.set_inputs(3.25,0,0)
			if self.chassis.odometry_x > 6.5:
				self.chassis.set_inputs(0,0,0)
				y+=1
				self.next_state_now("align_for_deployment")
	elif self.chassis.odometry_y > 2.4:
		self.chassis.set_inputs(3.25,0,math.pi/2)
		if self.chassis.odometry_x > 6.5:
			self.chassis.set_inputs(0,0,math.pi/2)
			if #facing side cargo bay
				self.chassis.set_inputs(0,0,0)
				y+=1
				self.next_state_now("align_for_deployment")
		elif #facing towards cargo bay
			self.chassis.set_inputs(3.25,0,0)
			if self.chassis.odometry_x:
				self.chassis.set_inputs(0,0,0)
				y+=1
				self.next_state_now("align_for_deployment")

	elif self.chassis.odometry_x > 6.5:
		self.chassis.set_inputs(0,1.2,math.pi/2)
		if #facing towards cargo bay
			self.set_inputs(0,1.2,0)
			if self.chassis.odometry_y > 2.4:
				self.chassis.set_inputs(0,0,0)
				y+=1
				self.next_state_now("align_for_deployment")
		elif self.chassis.odometry_y > 2.4:
			self.chassis.set_inputs(0,0,math.pi/2)
			if #facing towards cargo bay
				self.chassis.set_inputs(0,0,0)
				y+=1
				self.next_state_now("align_for_deployment")
#second pass
elif y == 2:
	self.chassis.set_inputs(3.5, 1.2, math.pi/2)
	if #facing side cargo bay
		self.chassis.set_inputs(3.5, 1.2, 0)
		if self.chassis.odometry_x > 7:
			self.chassis.set_inputs(0, 1.2,0)
			if self.chassis.odometry_y > 2.4:
				self.chassis.set_inputs(0,0,0)
				y+=1
				self.next_state_now("align_for_deployment")
		elif self.chassis.odometry_y > 2.4:
			self.chassis.set_inputs(3.5,0,0)
			if self.chassis.odometry_x > 7:
				self.chassis.set_inputs(0,0,0)
				y+=1
				self.next_state_now("align_for_deployment")
	elif self.chassis.odometry_y > 2.4:
		self.chassis.set_inputs(3.5,0,math.pi/2)
		if self.chassis.odometry_x > 7:
			self.chassis.set_inputs(0,0,math.pi/2)
			if #facing side cargo bay
				self.chassis.set_inputs(0,0,0)
				y+=1
				self.next_state_now("align_for_deployment")
		elif #facing towards cargo bay
			self.chassis.set_inputs(3.5,0,0)
			if self.chassis.odometry_x > 7:
				self.chassis.set_inputs(0,0,0)
				y+=1
				self.next_state_now("align_for_deployment")

	elif self.chassis.odometry_x > 7:
		self.chassis.set_inputs(0,1.2,math.pi/2)
		if #facing towards cargo bay
			self.set_inputs(0,1.2,0)
			if self.chassis.odometry_y > 2.4:
				self.chassis.set_inputs(0,0,0)
				y+=1
				self.next_state_now("align_for_deployment")
		elif self.chassis.odometry_y > 2.4
			self.chassis.set_inputs(0,0,math.pi/2)
			if #facing towards cargo bay
				self.chassis.set_inputs(0,0,0)
				y+=1
				self.next_state_now("align_for_deployment")


	#move towards side cargo bay with a hatch
	#needs to go 2.4m to the right and 6.5 m forward 
	#needs to go 0.5 m forward each time for a new hatch

	#if near side cargo bay without hatch
		#self.next_state_now("align_for_deployment")
	pass

@state
def align_for_deployment(self):
	#if aligned
		#self.next_state_now("deposit_hatch")
	pass

@state
def align_for_intake(self):
	#if aligned
		#self.next_state_now("intake hatch")
	pass

@state
def deposit_hatch(self):
	#if hatch deposited 
		#self.next_state_now("drive_to_loading_bay")
    pass


@state
def intake_hatch(self):
	#if hatch in robot
		#self.next_state_now("drive_to_side_cargo_bay")
    pass

@state
def stop(self):
    self.chassis.set_inputs(0, 0, 0)
    self.done()

@property
def current_odometry(self):
    return (self.chassis.odometry_x, self.chassis.odometry_y)
