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

    ship_deposit = ()  # TODO

    @state
    def drive_to_ship(self):
        self.motion.set_trajectory([self.current_waypoint, self.ship_deposit],
                                   end_heading=0, end_speed=0.4)
        if not self.motion.trajectory_executing:
            self.next_state_now("deposit_hatch")

    @state
    def intake_hatch(self):
        pass

    @state
    def deposit_hatch(self):
        pass

    @state
    def stop(self):
        self.chassis.set_inputs(0, 0, 0)
        self.done()

    @property
    def current_waypoint(self):
        return (self.chassis.odometry_x, self.chassis.odometry_y)
