import wpilib
import math
import time
from magicbot.state_machine import AutonomousStateMachine, state

from automations.motion import ChassisMotion
from components.vision import Vision
from pyswervedrive.chassis import Chassis
from utilities.navx import NavX
# these imports won't work yet


class LeftStartAuto(AutonomousStateMachine):
	MODE_NAME = "LEFT_START_AUTO"
	chassis: Chassis
	motion: ChassisMotion

	
	def __init__(self):
		self.lb_stage = 1
		self.lb_x_1 = 0
		self.lb_y_1 = -2
		self.lb_spin_1 = math.pi 
		self.lb_x_2 = -3.25
		self.lb_y_2 = -1.2
		self.lb_spin_2 = (math.pi/2)*-1
		self.lb_x_3 = -3.5
		self.lb_y_3 = -1.2
		self.lb_spin_3 = (math.pi/2)*-1
		self.lb_x_4 = -3.75
		self.lb_y_4 = -1.2
		self.lb_spin_4 = (math.pi/2)*-1

		self.scb_stage = 1
		self.scb_x_1 = 3.25
		self.scb_y_1 = 1.2
		self.scb_spin_1 = math.pi/2
		self.scb_x_2 = 3.5
		self.scb_y_2 = 1.2
		self.scb_spin_2 = math.pi/2
		self.scb_x_3 = 3.75
		self.scb_y_3 = 1.2
		self.scb_spin_3 = math.pi/2



	@state(first=True)
	def drive_to_front_of_cargoship(self, initial_call):
		if initial_call:	
			self.chassis.set_inputs(3, 0, 0)
			if self.chassis.odometry_x > 2.9:
				self.chassis.set_inputs(0, 0, 0)
				self.next_state_now("align_for_deployment")
	
	@state
	def drive_to_loading_bay(self):
		#first pickup
		if self.lb_stage == 1:
			self.chassis.set_inputs(0,self.lb_y_1,self.lb_spin_1)
			if #facing loading bay
				self.chassis.set_inputs(self.lb_y,self.lb_y_1,0)
				self.lb_spin_1 = 0
			if self.chassis.odometry_y < -2:
				self.chassis.set_inputs(self.lb_x_1,0,self.lb_spin_1)
				self.lb_y_1 = 0
			if self.lb_y_1 == 0 and self.lb_spin_1 == 0:
				self.chassis.set_inputs(-4,-0.6,0)
				if self.chassis.odometry_x < 5.6:
					self.chassis.set_inputs(0,0,0)
					self.lb_stage +=1
					self.next_state_now("align_for_intake")

		elif self.lb_stage == 2:
			self.chassis.set_inputs(self.lb_x_2,self.lb_y_2,self.lb_spin_2)
			if #facing loading bay
				self.chassis.set_input(self.lb_x_2,self.lb_y_2,0)
				self.lb_spin_2 = 0
			if self.chassis.odometry_x < - 6.5:
				self.chassis.set_inputs(0,self.lb_y_2,self.lb_spin_2)
				self.lb_x_2 = 0
			if self.chassis.odometry_y < -2.4:
				self.chass.set_inputs(self.lb_x_2,0,self.lb_spin_2)
				self.lb_y_2 = 0
			if self.lb_x_2 == 0 and self.lb_y_2 == 0 and lb_spin_2 == 0
				self.lb_stage+=1
				self.next_state_now("align_for_intake")


		elif self.lb_stage == 3:
			self.chassis.set_inputs(self.lb_x_3,self.lb_y_3,self.lb_spin_3)
			if #facing loading bay
				self.chassis.set_inputs(self._x_3,self.lb_y_3,0)
				self.lb_spin_3 = 0
			if self.chassis.odometry_x < - 6.5:
				self.chassis.set_inputs(0,self.lb_y_3,self.lb_spin_3)
				self.lb_x_3 = 0
			if self.chassis.odometry_y < -2.4:
				self.chass.set_inputs(self.lb_x_3,0,self.lb_spin_3)
				self.lb_y_3 = 0
			if self.lb_x_3 == 0 and self.lb_y_3 == 0 and self.lb_spin_3 == 0
				self.lb_stage+=1
				self.next_state_now("align_for_intake")


		elif self.lb_stage == 4:
			self.chassis.set_inputs(self.lb_x_4,self.lb_y_4,self.lb_spin_4)
			if #facing loading bay
				self.chassis.set_inputs(self.lb_x_4,self.lb_y_4,0)
				self.lb_spin_4 = 0
			if self.chassis.odometry_x < - 6.5:
				self.chassis.set_inputs(0,self.lb_y_4,self.lb_spin_4)
				self.lb_x_4 = 0
			if self.chassis.odometry_y < -2.4:
				self.chass.set_inputs(self.lb_x_4,0,spin)
				self.lb_y_4 = 0
			if self.lb_x_4 == 0 and self.lb_y_4 == 0 and self.lb_spin_4 == 0
				self.lb_stage+=1
				self.next_state_now("align_for_intake")







			#getting back from side bays 
		#if near loading bay
			#self.next_state_now("align_for_intake")
	pass

	@state
	def drive_to_side_cargo_bay(self):
		#first pass
		if self.scb_stage == 1:
			self.chassis.set_inputs(self.scb_x_1,self.scb_y_1,self.scb_spin_1)
			if #facing cargo bay
				self.chassis.set_inputs(self.scb_x_1,self.scb_y_1,0)
				self.scb_spin_1 = 0
			if self.chassis.odometry_x > 6.5:
				self.chassis.set_inputs(0,self.scb_y_1,self.scb_spin_1)
				self.scb_x_1 = 0
			if self.chassis.odometry_y > 2.4:
				self.chassis.set_inputs(self.scb_x_1,0,self.scb_spin_1)
				self.scb_y_1 = 0
			if self.scb_x_1 == 0 and self.scb_y_1 == 0 and self.scb_spin_1 == 0:
				self.scb_stage+=1
				self.next_state_now("align_for_deployment")
		#second pass
		if self.scb_stage == 2:
			self.chassis.set_inputs(self.scb_x_2,self.scb_y_2,self.scb_spin_2)
			if #facing cargo bay
				self.chassis.set_inputs(self.scb_x_2,self.scb_y_2,0)
				self.scb_spin_2 = 0
			if self.chassis.odometry_x > 6.5:
				self.chassis.set_inputs(0,self.scb_y_2,self.scb_spin_2)
				self.scb_x_2 = 0
			if self.chassis.odometry_y > 2.4:
				self.chassis.set_inputs(self.scb_x_2,0,self.scb_spin_2)
				self.scb_y_2 = 0
			if self.scb_x_2 == 0 and self.scb_y_2 == 0 and self.scb_spin_2 == 0:
				self.scb_stage+=1
				self.next_state_now("align_for_deployment")
		#third pass
		if self.scb_stage == 3:
			self.chassis.set_inputs(self.scb_x_3,self.scb_y_3,self.scb_spin_3)
			if #facing cargo bay
				self.chassis.set_inputs(self.scb_x_3,self.scb_y_3,0)
				self.scb_spin_3 = 0
			if self.chassis.odometry_x > 6.5:
				self.chassis.set_inputs(0,self.scb_y_3,self.scb_spin_3)
				self.scb_x_3 = 0
			if self.chassis.odometry_y > 2.4:
				self.chassis.set_inputs(self.scb_x_3,0,self.scb_spin_3)
				self.scb_y_3 = 0
			if self.scb_x_3 == 0 and self.scb_y_3 == 0 and self.scb_spin_3 == 0:
				self.scb_stage+=1
				self.next_state_now("align_for_deployment")
			

		#move towards side cargo bay with a hatch
		#needs to go 2.4m to the right and 6.5 m forward 
		#needs to go 0.5 m forward each time for a new hatch

		#if near side cargo bay without hatch
			#self.next_state_now("align_for_deployment")
		

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
