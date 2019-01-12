import wpilib
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
    def drive_to_ship(self, initial_call):
        if initial_call:	
        	self.chassis.set_inputs(3, 0, 0)
        	if self.chassis.odometry_y > 3:
        		self.chassis.set_inputs(0, 0, 0)
        		self.next_state_now("deposit_hatch")
        if not self.chassis.set_inputs:
            self.next_state_now("deposit_hatch")

            #TODO and fix later 

            

    @state
    def deposit_hatch(self):
        pass	   

    @state
    def intake_hatch(self):
        pass

    @state
    def stop(self):
        self.chassis.set_inputs(0, 0, 0)
        self.done()

    @property
    def current_odometry(self):
        return (self.chassis.odometry_x, self.chassis.odometry_y)
