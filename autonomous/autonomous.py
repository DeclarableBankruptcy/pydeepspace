import wpilib
import math
import time
from magicbot.state_machine import AutonomousStateMachine, state

from automations.motion import ChassisMotion
from components.vision import Vision
from pyswervedrive.chassis import Chassis
from utilities.navx import NavX # will work
# these imports won't work yet


class LeftStartAuto(AutonomousStateMachine):
	MODE_NAME = "LEFT_START_AUTO"
	chassis: Chassis
	motion: ChassisMotion

	
	def __init__(self):
		self.loading_bay_stage = 1
		self.loading_bay_x_1 = 0
		self.loading_bay_y_1 = -2
		self.loading_bay_spin_1 = math.pi 
		self.loading_bay_x_2 = -3.25
		self.loading_bay_y_2 = -1.2
		self.loading_bay_spin_2 = (math.pi/2)*-1
		self.loading_bay_x_3 = -3.5
		self.loading_bay_y_3 = -1.2
		self.loading_bay_spin_3 = (math.pi/2)*-1
		self.loading_bay_x_4 = -3.75
		self.loading_bay_y_4 = -1.2
		self.loading_bay_spin_4 = (math.pi/2)*-1

		self.side_cargoship_stage = 1
		self.side_cargoship_x_1 = 3.25
		self.side_cargoship_y_1 = 1.2
		self.side_cargoship_spin_1 = math.pi/2
		self.side_cargoship_x_2 = 3.5
		self.side_cargoship_y_2 = 1.2
		self.side_cargoship_spin_2 = math.pi/2
		self.side_cargoship_x_3 = 3.75
		self.side_cargoship_y_3 = 1.2
		self.side_cargoship_spin_3 = math.pi/2



	@state(first=True)
	def drive_to_front_of_cargoship(self, initial_call):
		if initial_call:	
			self.chassis.set_inputs(3, 0, 0)
			if self.chassis.odometry_x > 2.9:
				self.chassis.set_inputs(0, 0, 0)
				self.next_state_now("align_for_deployment")

	@state
	def drive_to_side_cargo_bay(self):
		#first pass
		if self.side_cargoship_stage == 1:
			self.chassis.set_inputs(self.side_cargoship_x_1,self.side_cargoship_y_1,self.side_cargoship_spin_1)
			if self.navx.getAngle() >= math.pi*1.5: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_inputs(self.side_cargoship_x_1,self.side_cargoship_y_1,0)
				self.side_cargoship_spin_1 = 0
			if self.chassis.odometry_x > 6.5:
				self.chassis.set_inputs(0,self.side_cargoship_y_1,self.side_cargoship_spin_1)
				self.side_cargoship_x_1 = 0
			if self.chassis.odometry_y > 2.4:
				self.chassis.set_inputs(self.side_cargoship_x_1,0,self.side_cargoship_spin_1)
				self.side_cargoship_y_1 = 0
			if self.side_cargoship_x_1 == 0 and self.side_cargoship_y_1 == 0 and self.side_cargoship_spin_1 == 0:
				self.side_cargoship_stage+=1
				self.next_state_now("align_for_deployment")
		#second pass
		if self.side_cargoship_stage == 2:
			self.chassis.set_inputs(self.side_cargoship_x_2,self.side_cargoship_y_2,self.side_cargoship_spin_2)
			if self.navx.getAngle() >= math.pi*1.5: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_inputs(self.side_cargoship_x_2,self.side_cargoship_y_2,0)
				self.side_cargoship_spin_2 = 0
			if self.chassis.odometry_x > 6.5:
				self.chassis.set_inputs(0,self.side_cargoship_y_2,self.side_cargoship_spin_2)
				self.side_cargoship_x_2 = 0
			if self.chassis.odometry_y > 2.4:
				self.chassis.set_inputs(self.side_cargoship_x_2,0,self.side_cargoship_spin_2)
				self.side_cargoship_y_2 = 0
			if self.side_cargoship_x_2 == 0 and self.side_cargoship_y_2 == 0 and self.side_cargoship_spin_2 == 0:
				self.side_cargoship_stage+=1
				self.next_state_now("align_for_deployment")
		#third pass
		if self.side_cargoship_stage == 3:
			self.chassis.set_inputs(self.side_cargoship_x_3,self.side_cargoship_y_3,self.side_cargoship_spin_3)
			if self.navx.getAngle() >= math.pi*1.5: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_inputs(self.side_cargoship_x_3,self.side_cargoship_y_3,0)
				self.side_cargoship_spin_3 = 0
			if self.chassis.odometry_x > 6.5:
				self.chassis.set_inputs(0,self.side_cargoship_y_3,self.side_cargoship_spin_3)
				self.side_cargoship_x_3 = 0
			if self.chassis.odometry_y > 2.4:
				self.chassis.set_inputs(self.side_cargoship_x_3,0,self.side_cargoship_spin_3)
				self.side_cargoship_y_3 = 0
			if self.side_cargoship_x_3 == 0 and self.side_cargoship_y_3 == 0 and self.side_cargoship_spin_3 == 0:
				self.side_cargoship_stage+=1
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
	def deposit_hatch(self):
		#if hatch deposited 
			#self.next_state_now("drive_to_loading_bay")
		pass

	@state
	def drive_to_loading_bay(self):
		#first pickup
		if self.loading_bay_stage == 1:
			self.chassis.set_inputs(0,self.loading_bay_y_1,self.loading_bay_spin_1)
			if self.navx.getAngle() >= math.pi: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_inputs(self.loading_bay_y,self.loading_bay_y_1,0)
				self.loading_bay_spin_1 = 0
			if self.chassis.odometry_y < -2:
				self.chassis.set_inputs(self.loading_bay_x_1,0,self.loading_bay_spin_1)
				self.loading_bay_y_1 = 0
			if self.loading_bay_y_1 == 0 and self.loading_bay_spin_1 == 0 and self.loading_bay_spin_1 == 0:
				self.chassis.set_inputs(-4,-0.6,0)
			if self.chassis.odometry_x < 5.6:
				self.chassis.set_inputs(0,0,0)
				self.loading_bay_stage +=1
				self.next_state_now("align_for_intake")
		#sencond pickup
		elif self.loading_bay_stage == 2:
			self.chassis.set_inputs(self.loading_bay_x_2,self.loading_bay_y_2,self.loading_bay_spin_2)
			if self.navx.getAngle() >= math.pi: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_input(self.loading_bay_x_2,self.loading_bay_y_2,0)
				self.loading_bay_spin_2 = 0
			if self.chassis.odometry_x < - 6.5:
				self.chassis.set_inputs(0,self.loading_bay_y_2,self.loading_bay_spin_2)
				self.loading_bay_x_2 = 0
			if self.chassis.odometry_y < -2.4:
				self.chass.set_inputs(self.loading_bay_x_2,0,self.loading_bay_spin_2)
				self.loading_bay_y_2 = 0
			if self.loading_bay_x_2 == 0 and self.loading_bay_y_2 == 0 and loading_bay_spin_2 == 0:
				self.loading_bay_stage+=1
				self.next_state_now("align_for_intake")

		#third pickup
		elif self.loading_bay_stage == 3:
			self.chassis.set_inputs(self.loading_bay_x_3,self.loading_bay_y_3,self.loading_bay_spin_3)
			if self.navx.getAngle() >= math.pi: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_inputs(self._x_3,self.loading_bay_y_3,0)
				self.loading_bay_spin_3 = 0
			if self.chassis.odometry_x < - 6.5:
				self.chassis.set_inputs(0,self.loading_bay_y_3,self.loading_bay_spin_3)
				self.loading_bay_x_3 = 0
			if self.chassis.odometry_y < -2.4:
				self.chass.set_inputs(self.loading_bay_x_3,0,self.loading_bay_spin_3)
				self.loading_bay_y_3 = 0
			if self.loading_bay_x_3 == 0 and self.loading_bay_y_3 == 0 and self.loading_bay_spin_3 == 0:
				self.loading_bay_stage+=1
				self.next_state_now("align_for_intake")

		#fourth pickup
		elif self.loading_bay_stage == 4:
			self.chassis.set_inputs(self.loading_bay_x_4,self.loading_bay_y_4,self.loading_bay_spin_4)
			if self.navx.getAngle() >= math.pi: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_inputs(self.loading_bay_x_4,self.loading_bay_y_4,0)
				self.loading_bay_spin_4 = 0
			if self.chassis.odometry_x < - 6.5:
				self.chassis.set_inputs(0,self.loading_bay_y_4,self.loading_bay_spin_4)
				self.loading_bay_x_4 = 0
			if self.chassis.odometry_y < -2.4:
				self.chass.set_inputs(self.loading_bay_x_4,0,spin)
				self.loading_bay_y_4 = 0
			if self.loading_bay_x_4 == 0 and self.loading_bay_y_4 == 0 and self.loading_bay_spin_4 == 0:
				self.loading_bay_stage+=1
				self.next_state_now("align_for_intake")
		

	@state
	def align_for_intake(self):
		#if aligned
			#self.next_state_now("intake hatch")
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














class RightStartAuto(AutonomousStateMachine):
	MODE_NAME = "LEFT_START_AUTO"
	chassis: Chassis
	motion: ChassisMotion

	
	def __init__(self):
		self.loading_bay_stage = 1
		self.loading_bay_x_1 = 0
		self.loading_bay_y_1 = 2
		self.loading_bay_spin_1 = math.pi 
		self.loading_bay_x_2 = -3.25
		self.loading_bay_y_2 = 1.2
		self.loading_bay_spin_2 = (math.pi/2)
		self.loading_bay_x_3 = -3.5
		self.loading_bay_y_3 = 1.2
		self.loading_bay_spin_3 = (math.pi/2)
		self.loading_bay_x_4 = -3.75
		self.loading_bay_y_4 = 1.2
		self.loading_bay_spin_4 = (math.pi/2)

		self.side_cargoship_stage = 1
		self.side_cargoship_x_1 = 3.25
		self.side_cargoship_y_1 = -1.2
		self.side_cargoship_spin_1 = (math.pi/2)*-1
		self.side_cargoship_x_2 = 3.5
		self.side_cargoship_y_2 = -1.2
		self.side_cargoship_spin_2 = (math.pi/2)*-1
		self.side_cargoship_x_3 = 3.75
		self.side_cargoship_y_3 = -1.2
		self.side_cargoship_spin_3 = (math.pi/2)*-1



	@state(first=True)
	def drive_to_front_of_cargoship(self, initial_call):
		if initial_call:	
			self.chassis.set_inputs(3, 0, 0)
			if self.chassis.odometry_x > 2.9:
				self.chassis.set_inputs(0, 0, 0)
				self.next_state_now("align_for_deployment")

	@state
	def drive_to_side_cargo_bay(self):
		#first pass
		if self.side_cargoship_stage == 1:
			self.chassis.set_inputs(self.side_cargoship_x_1,self.side_cargoship_y_1,self.side_cargoship_spin_1)
			if self.navx.getAngle() >= math.pi*0.5: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_inputs(self.side_cargoship_x_1,self.side_cargoship_y_1,0)
				self.side_cargoship_spin_1 = 0
			if self.chassis.odometry_x > 6.5:
				self.chassis.set_inputs(0,self.side_cargoship_y_1,self.side_cargoship_spin_1)
				self.side_cargoship_x_1 = 0
			if self.chassis.odometry_y < -2.4:
				self.chassis.set_inputs(self.side_cargoship_x_1,0,self.side_cargoship_spin_1)
				self.side_cargoship_y_1 = 0
			if self.side_cargoship_x_1 == 0 and self.side_cargoship_y_1 == 0 and self.side_cargoship_spin_1 == 0:
				self.side_cargoship_stage+=1
				self.next_state_now("align_for_deployment")
		#second pass
		if self.side_cargoship_stage == 2:
			self.chassis.set_inputs(self.side_cargoship_x_2,self.side_cargoship_y_2,self.side_cargoship_spin_2)
			if self.navx.getAngle() >= math.pi*0.5: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_inputs(self.side_cargoship_x_2,self.side_cargoship_y_2,0)
				self.side_cargoship_spin_2 = 0
			if self.chassis.odometry_x > 6.5:
				self.chassis.set_inputs(0,self.side_cargoship_y_2,self.side_cargoship_spin_2)
				self.side_cargoship_x_2 = 0
			if self.chassis.odometry_y < -2.4:
				self.chassis.set_inputs(self.side_cargoship_x_2,0,self.side_cargoship_spin_2)
				self.side_cargoship_y_2 = 0
			if self.side_cargoship_x_2 == 0 and self.side_cargoship_y_2 == 0 and self.side_cargoship_spin_2 == 0:
				self.side_cargoship_stage+=1
				self.next_state_now("align_for_deployment")
		#third pass
		if self.side_cargoship_stage == 3:
			self.chassis.set_inputs(self.side_cargoship_x_3,self.side_cargoship_y_3,self.side_cargoship_spin_3)
			if self.navx.getAngle() >= math.pi*0.5: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_inputs(self.side_cargoship_x_3,self.side_cargoship_y_3,0)
				self.side_cargoship_spin_3 = 0
			if self.chassis.odometry_x > 6.5:
				self.chassis.set_inputs(0,self.side_cargoship_y_3,self.side_cargoship_spin_3)
				self.side_cargoship_x_3 = 0
			if self.chassis.odometry_y < -2.4:
				self.chassis.set_inputs(self.side_cargoship_x_3,0,self.side_cargoship_spin_3)
				self.side_cargoship_y_3 = 0
			if self.side_cargoship_x_3 == 0 and self.side_cargoship_y_3 == 0 and self.side_cargoship_spin_3 == 0:
				self.side_cargoship_stage+=1
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
	def deposit_hatch(self):
		#if hatch deposited 
			#self.next_state_now("drive_to_loading_bay")
		pass

	@state
	def drive_to_loading_bay(self):
		#first pickup
		if self.loading_bay_stage == 1:
			self.chassis.set_inputs(0,self.loading_bay_y_1,self.loading_bay_spin_1)
			if self.navx.getAngle() >= math.pi: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_inputs(self.loading_bay_y,self.loading_bay_y_1,0)
				self.loading_bay_spin_1 = 0
			if self.chassis.odometry_y > 2:
				self.chassis.set_inputs(self.loading_bay_x_1,0,self.loading_bay_spin_1)
				self.loading_bay_y_1 = 0
			if self.loading_bay_y_1 == 0 and self.loading_bay_spin_1 == 0:
				self.chassis.set_inputs(-4,0.6,0)
				if self.chassis.odometry_x < 5.6:
					self.chassis.set_inputs(0,0,0)
					self.loading_bay_stage +=1
					self.next_state_now("align_for_intake")
		#sencond pickup
		elif self.loading_bay_stage == 2:
			self.chassis.set_inputs(self.loading_bay_x_2,self.loading_bay_y_2,self.loading_bay_spin_2)
			if self.navx.getAngle() >= math.pi: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_input(self.loading_bay_x_2,self.loading_bay_y_2,0)
				self.loading_bay_spin_2 = 0
			if self.chassis.odometry_x < - 6.5:
				self.chassis.set_inputs(0,self.loading_bay_y_2,self.loading_bay_spin_2)
				self.loading_bay_x_2 = 0
			if self.chassis.odometry_y < 2.4:
				self.chass.set_inputs(self.loading_bay_x_2,0,self.loading_bay_spin_2)
				self.loading_bay_y_2 = 0
			if self.loading_bay_x_2 == 0 and self.loading_bay_y_2 == 0 and loading_bay_spin_2 == 0:
				self.loading_bay_stage+=1
				self.next_state_now("align_for_intake")

		#third pickup
		elif self.loading_bay_stage == 3:
			self.chassis.set_inputs(self.loading_bay_x_3,self.loading_bay_y_3,self.loading_bay_spin_3)
			if self.navx.getAngle() >= math.pi: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_inputs(self._x_3,self.loading_bay_y_3,0)
				self.loading_bay_spin_3 = 0
			if self.chassis.odometry_x < - 6.5:
				self.chassis.set_inputs(0,self.loading_bay_y_3,self.loading_bay_spin_3)
				self.loading_bay_x_3 = 0
			if self.chassis.odometry_y < 2.4:
				self.chass.set_inputs(self.loading_bay_x_3,0,self.loading_bay_spin_3)
				self.loading_bay_y_3 = 0
			if self.loading_bay_x_3 == 0 and self.loading_bay_y_3 == 0 and self.loading_bay_spin_3 == 0:
				self.loading_bay_stage+=1
				self.next_state_now("align_for_intake")

		#fourth pickup
		elif self.loading_bay_stage == 4:
			self.chassis.set_inputs(self.loading_bay_x_4,self.loading_bay_y_4,self.loading_bay_spin_4)
			if self.navx.getAngle() >= math.pi: #might be wrong depending on wat side the hatch is on 
				self.chassis.set_inputs(self.loading_bay_x_4,self.loading_bay_y_4,0)
				self.loading_bay_spin_4 = 0
			if self.chassis.odometry_x < - 6.5:
				self.chassis.set_inputs(0,self.loading_bay_y_4,self.loading_bay_spin_4)
				self.loading_bay_x_4 = 0
			if self.chassis.odometry_y < 2.4:
				self.chass.set_inputs(self.loading_bay_x_4,0,spin)
				self.loading_bay_y_4 = 0
			if self.loading_bay_x_4 == 0 and self.loading_bay_y_4 == 0 and self.loading_bay_spin_4 == 0:
				self.loading_bay_stage+=1
				self.next_state_now("align_for_intake")
		

	@state
	def align_for_intake(self):
		#if aligned
			#self.next_state_now("intake hatch")
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
