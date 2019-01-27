import wpilib
import math
import numpy as np
from magicbot.state_machine import AutonomousStateMachine, state

from pyswervedrive.swervechassis import SwerveChassis
from utilities.navx import NavX
from utilities.pure_pursuit import PurePursuit
# from automations.alignment import Aligner


def reflect_2d_y(v: tuple) -> tuple:
    return (v[0], -v[1])


class LeftStartAuto(AutonomousStateMachine):
    MODE_NAME = "LEFT_START_AUTO"
    # DEFAULT = True

    imu: NavX
    # align: Aligner
    chassis: SwerveChassis
    pursuit: PurePursuit
    # hatchcontroller: HatchController

    # hatch: Hatch

    def __init__(self):
        super().__init__()
        self.front_cargo_bay = (5.6 - SwerveChassis.LENGTH / 2 - 0.5, 0.2)
        self.setup_loading_bay = (3.3, 2.2)
        self.loading_bay = (0.2 + SwerveChassis.LENGTH / 2 + 0.5, 3.38)
        self.side_cargo_bay = (6.6, 0.9 + SwerveChassis.WIDTH / 2 + 0.5)
        # The waypoints we use to move to the other side of the field
        self.cross_point = (4.7, 1)
        # points on opposite side of field
        # self.opp_front_cargo_bay = reflect_2d_y(self.front_cargo_bay)
        # self.opp_setup_loading_bay = reflect_2d_y(self.setup_loading_bay)
        self.opp_loading_bay = reflect_2d_y(self.loading_bay)
        self.opp_side_cargo_bay = reflect_2d_y(self.side_cargo_bay)
        self.opp_cross_point = (4.7, -1.8) 

        self.start_pos = (1.2 + SwerveChassis.LENGTH / 2, 0 + SwerveChassis.WIDTH / 2)
        self.completed_runs = 0
        self.desired_angle = 0
        self.desired_angle_navx = 0

    def on_enable(self):
        super().on_enable()
        self.chassis.odometry_x = self.start_pos[0]
        self.chassis.odometry_y = self.start_pos[1]
        # print(f"odometry = {self.current_pos}")

    @state(first=True)
    def drive_to_cargo_bay(self, initial_call):
        if initial_call:
            # print(f"odometry = {self.current_pos}")
            if self.completed_runs == 0:
                self.desired_angle = 0
                self.pursuit.build_path((self.start_pos, (2.5, 0.1), self.front_cargo_bay))
            elif self.completed_runs == 1:
                self.desired_angle = math.pi / 2
                self.chassis.set_heading_sp(-math.pi / 2)
                self.pursuit.build_path((self.loading_bay, self.side_cargo_bay))
            elif self.completed_runs == 2:
                self.desired_angle = math.pi / 2
                self.chassis.set_heading_sp(math.pi / 2)
                self.pursuit.build_path((self.opp_loading_bay, self.opp_side_cargo_bay))
        if self.pursuit.completed_path and self.completed_runs > 3:
            self.next_state("stop")
        if self.pursuit.completed_path:
            # if abs(self.imu.getAngle()) >= abs(self.desired_angle) or self.completed_runs == 0:
            self.next_state("deposit_hatch")
            self.completed_runs += 1
        self.follow_path()

    @state
    def deposit_hatch(self, initial_call):
        self.next_state_now("drive_to_loading_bay")
        # if initial_call:
        #     self.hatchcontroller.start_punch()
        #     if not self.hatchcontroller.is_executing and :
        #         self.next_state_now("drive_to_loading_bay")
        

    @state
    def drive_to_loading_bay(self, initial_call):
        if initial_call:
            if self.completed_runs == 1:
                self.chassis.set_heading_sp(math.pi)
                self.pursuit.build_path(
                    (
                        self.front_cargo_bay,
                        self.setup_loading_bay,
                        self.loading_bay)
                )
            elif self.completed_runs == 2:
                # we only have a quater field, stop here
                self.next_state_now("stop")
                return
                self.chassis.set_heading_sp(math.pi)
                self.pursuit.build_path(
                    (
                        self.side_cargo_bay,
                        self.cross_point,
                        self.opp_cross_point,
                        self.opp_loading_bay,
                    )
                )
            elif self.completed_runs == 3:
                # return to the loading bay for start of teleop
                self.chassis.set_heading_sp(math.pi)
                self.pursuit.build_path((self.opp_side_cargo_bay, self.opp_loading_bay))
        if self.pursuit.completed_path and self.completed_runs > 3:
            self.next_state("stop")
        if self.pursuit.completed_path: #and abs(self.imu.getAngle()) >= (math.pi - math.pi/90):
            self.next_state("intake_hatch")
        self.follow_path()

    @state
    def intake_hatch(self, initial_call):
        self.next_state_now("drive_to_cargo_bay")

        # self.align.align(deposit=False, intake=True)
        # if self.align.ground_tape_align_intake == True:
        #     self.hatchcontroller.engage()
        #     if not self.hatchcontroller.is_executing:
        #         self.next_state_now("drive_to_loading_bay")
        



    @state
    def stop(self):
        self.chassis.set_inputs(0, 0, 0)
        self.done()

    @property
    def current_pos(self):
        return (self.chassis.odometry_x, self.chassis.odometry_y, self.imu.getAngle())
    
    def follow_path(self):
        changed_segment, direction = self.pursuit.compute_direction(self.current_pos)
        if self.pursuit.completed_path or changed_segment:
            self.chassis.set_inputs(0, 0, 0, field_oriented=False)
            return
        vx, vy = direction / np.linalg.norm(direction)
        # TODO investigate using trapezodial trajectory or other motion profiling
        vz = 0  # TODO implement a system to allow for rotation in waypoints
        self.chassis.set_inputs(vx * 1.5, vy * 1.5, vz)


class RightStartAuto(LeftStartAuto):
    MODE_NAME = "RIGHT_START_AUTO"
    DEFAULT = False

    def __init__(self):
        super().__init__()
        # self.opp_front_cargo_bay = self.front_cargo_bay
        # self.opp_setup_loading_bay = self.setup_loading_bay
        self.opp_loading_bay = self.loading_bay
        self.opp_side_cargo_bay = self.side_cargo_bay
        self.opp_cross_point = self.cross_point

        self.front_cargo_bay = reflect_2d_y(self.front_cargo_bay)
        self.setup_loading_bay = reflect_2d_y(self.setup_loading_bay)
        self.loading_bay = reflect_2d_y(self.loading_bay)
        self.side_cargo_bay = reflect_2d_y(self.side_cargo_bay)
        self.cross_point = reflect_2d_y(self.cross_point)
        self.start_pos = reflect_2d_y(self.start_pos)
