import wpilib
import math
from magicbot.state_machine import AutonomousStateMachine, state

from pyswervedrive.swervechassis import SwerveChassis
from utilities.navx import NavX
from utilities.vector_pursuit import VectorPursuit
from automations.motion import ChassisMotion

def reflect_2d_y(v: tuple) -> tuple:
    return (v[0], -v[1])


class LeftStartAuto(AutonomousStateMachine):
    MODE_NAME = "LEFT_START_AUTO"

    imu: NavX
    chassis: SwerveChassis
    motion: ChassisMotion
    #hatchcontroller: HatchController


    def __init__(self):
        self.front_left_cargo_bay = (5.8,0.2)
        self.setup_for_left_loading_bay = (4.8,2.2)
        self.left_loading_bay = (0.2,3.38)
        self.left_side_cargo_bay = (6.6,0.8)
        self.setup_one_for_right_loading_bay = (4.7,0.8)
        self.setup_two_for_right_loading_bay = (3,-1.94)
        self.right_loading_bay = (0.2,-3.38)
        self.right_side_cargo_bay = (6.6,-0.8)

        self.setup_right_front_cargo_bay = (4.8,-2.2)
        self.right_front_cargo_bay = (5.8,-0.2)

        #self.loading_bay_pos = ((), (), ())
        #self.cargoship_pos = ((), (), ())
        self.start_pos = (1.57,0.38)
        self.completed_runs = 0
        
    @state(first=True)
    def drive_to_cargoship(self, initial_call):
        if self.completed_runs == 0:
            self.motion.set_trajectory(
                (self.current_pos, self.front_left_cargo_bay)
            )
            if not self.motion.is_executing:
                self.chassis.set_inputs(0, 0, 0)
                self.completed_runs += 1
                self.next_state_now("deposit_hatch")
        if self.completed_runs == 1:
            self.motion.set_trajectory(
                (self.current_pos, self.left_side_cargo_bay)
            )
            if not self.motion.is_executing:
                self.chassis.set_inputs(0,0,0)
                self.next_state_now("deposit hatch")
        if self.completed_runs == 2:
            self.motion.set_trajectory(
                (self.current_pos, self.right_side_cargo_bay)
            )
            if not self.motion.is_executing:
                self.chassis.set_inputs(0,0,0)
                self.completed_runs += 1
                self.next_state_now("deposit hatch")
        else:
            self.next_state_now("done")

    @state
    def deposit_hatch(self, initial_call):

        pass

        if initial_call:
            self.hatchcontroller.engage()
        if not self.hatchcontroller.is_executing:
            self.next_state_now("drive_to_loading_bay")

    @state
    def drive_to_loading_bay(self, initial_call):
        if self.completed_runs == 1:
            self.motion.set_trajectory(
                (self.current_pos, self.setup_for_left_loading_bay)
            )
            if not self.motion.is_executing:
                self.motion.set_trajectory(
                    (self.current_pos, self.left_loading_bay)
                )
                if not self.motion.is_executing:
                    self.chassis.set_inputs(0,0,0)
                    self.next_state_now("intake_hatch")
        if self.completed_runs == 2:
            self.motion.set_trajectory(
                (self.current_pos, self.setup_one_for_right_loading_bay)
            )
            if not self.motion.is_executing:
                self.motion.set_trajectory(
                    (self.current_pos, self.setup_two_for_right_loading_bay)
                )
                if not self.motion.is_executing:
                    self.motion.set_trajectory(
                        (self.current_pos, self.right_loading_bay)
                    )
                    if not self.motion.is_executing:
                        self.chassis.set_inputs(0,0,0)
                        self.next_state_now("intake hatch")
        else:
            self.next_state_now("done")

    @state
    def intake_hatch(self, initial_call):

        pass

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


'''class RightStartAuto(LeftStartAuto):
    MODE_NAME = "RIGHT_START_AUTO"
    chassis: Chassis
    motion: ChassisMotion

    def __init__(self):
        super(self.__init__)
        self.loading_bay_pos = reflect_2d_y(self.loading_bay_pos)
        self.cargoship_pos = reflect_2d_y(self.cargoship_pos)
        self.start_pos = reflect_2d_y(self.start_pos)'''