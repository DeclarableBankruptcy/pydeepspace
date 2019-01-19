import wpilib
import math
from magicbot.state_machine import AutonomousStateMachine, state

from pyswervedrive.chassis import Chassis
from utilities.navx import NavX


def reflect_2d_y(v: tuple) -> tuple:
    return (v[0], -v[1])


class LeftStartAuto(AutonomousStateMachine):
    MODE_NAME = "LEFT_START_AUTO"

    imu: NavX
    chassis: Chassis
    motion: ChassisMotion
    hatchcontroller: HatchController

    def __init__(self):
        self.point1 = ((5.8),(0.2),())    # \
        self.point2 = ((4.8),(2.2),())    # |-- turn 180 degres somwhere in here 
        self.point3 = ((0.2),(3.38),())   # /
        self.point4 = ((6.6),(0.8),())   #} turn 90 degres here    
        self.point5 = ((4.7),(0.8),())    # \
        self.point6 = ((3),(-1.94),())    # |-- turn back 90 degres somhere here
        self.point7 = ((0.2),(-3.38),()) #  /
        self.point8_side = ((6.6),(-0.8),())  #} turn back another 90 degres here 
        #or                                             
        self.point8_front = ((5.8),(-0.2),())  #} turn 180         this is for if we go to front 

        self.loading_bay_pos = ((), (), ())
        self.cargoship_pos = ((), (), ())
        self.start_pos = ()
        self.completed_runs = 0
        # TODO find the correct co-ordinates

    @state(first=True)
    def drive_to_cargoship(self, initial_call):
        if initial_call:
            self.motion.set_waypoints(
                (self.current_pos, self.cargoship_pos[self.completed_runs])
            )
        if not self.motion.is_executing:
            self.chassis.set_inputs(0, 0, 0)
            self.completed_runs += 1
            self.next_state_now("deposit_hatch")

    @state
    def deposit_hatch(self, initial_call):
        if initial_call:
            self.hatchcontroller.engage()
        if not self.hatchcontroller.is_executing:
            self.next_state_now("drive_to_loading_bay")

    @state
    def drive_to_loading_bay(self, initial_call):
        if initial_call:
            self.motion.set_waypoints(
                (self.current_pos, self.loading_bay_pos[self.completed_runs - 1])
            )
            # -1 because we dont have to pickup a hatch on the first run
        if not self.motion.is_executing:
            self.next_state_now("intake_hatch")

    @state
    def intake_hatch(self, initial_call):
        if initial_call:
            self.hatchcontroller.engage()
        if not self.hatchcontroller.is_executing:
            self.next_state_now("drive_to_cargoship")

    @state
    def stop(self):
        self.chassis.set_inputs(0, 0, 0)
        self.done()

    @property
    def current_pos(self):
        return (self.chassis.odometry_x, self.chassis.odometry_y, self.imu.getAngle())


class RightStartAuto(LeftStartAuto):
    MODE_NAME = "RIGHT_START_AUTO"
    chassis: Chassis
    motion: ChassisMotion

    def __init__(self):
        super(self.__init__)
        self.loading_bay_pos = reflect_2d_y(self.loading_bay_pos)
        self.cargoship_pos = reflect_2d_y(self.cargoship_pos)
        self.start_pos = reflect_2d_y(self.start_pos)