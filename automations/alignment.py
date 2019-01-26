from components.vision import Vision
from magicbot.state_machine import StateMachine, state
from pyswervedrive.swervechassis import SwerveChassis
from magicbot import tunable
import math


class Aligner(StateMachine):
    """
    A state machine for alignment using vision systems.
    The robot will use two methods of alignment, targets above
    objectives from longer range and fine adjustment using the ground
    tape once we are able to see it.
    """

    chassis: SwerveChassis
    vision: Vision

    def setup(self):
        self.successful = False
        self.target_tape_loop_counter = 0
        self.ground_tape_loop_counter = 0

    target_tape_kP_x = tunable(0.5)  # forwards
    target_tape_kP_y = tunable(1)  # m/s
    target_tape_tolerance = tunable(0.05)  # % of camera view

    ground_tape_kP_x = tunable(0.5)  # forward
    ground_tape_kP_y = tunable(1)  # m/s
    ground_tape_kP_angle = tunable(1)  # 45 degres
    ground_tape_distance_tolerance_y = tunable(0.02)
    # ground_tape_distance_tolerance_x = tunable(0.4) 
    ground_tape_angle_tolerance = tunable(math.pi / 360)  # this equals 0.5 degres

    def align(self, deposit, intake):
        self.align_for_deposit = deposit
        self.align_for_intake = intake



    @state(first=True)
    def target_tape_align(self, initial_call):
        """
        Align with the objective using the vision tape above the objective.
        The robot will try to correct errors untill they are within target_tape_tolerance
        by strafing and moving in a hyberbolic curve towards the target.
        """
        if initial_call:
            self.loop_counter = 0
        error = self.vision.get_target_tape_error()
        if error is None:
            self.loop_counter += 1
            if self.loop_counter > 3:
                self.chassis.set_inputs(0, 0, 0)
                self.done()
        else:
            if abs(error) > self.target_tape_tolerance:
                self.chassis.set_inputs(
                    (1 - abs(error)) * self.target_tape_kP_x,
                    error * self.target_tape_kP_y,
                    0,
                    field_oriented=False,
                )
            elif (
                self.vision.get_ground_tape_error_x()
                or self.vision.get_ground_tape_error_y is not None
            ):
                self.next_state_now("ground_tape_align")

    @state
    def ground_tape_align(self, initial_call):
        if initial_call:
            self.ground_tape_loop_counter = 0
        error_x = self.vision.get_ground_tape_error_x()
        error_y = self.vision.get_ground_tape_error_y()
        error_angle = self.vision.get_ground_tape_error_angle()
        positioned_to_outake = self.vision.get_positioned_to_outake()
        if error_x or error_y is None:
            self.loop_counter += 1
            if self.loop_counter > 3:
                self.chassis.set_inputs(0, 0, 0)
                self.done()
        else:
            if abs(error_x) <= self.ground_tape_distance_tolerance_x:
                error_x = 0
            elif abs(error_y) <= self.ground_tape_distance_tolerance_y:
                error_y = 0
            elif abs(error_angle) <= self.ground_tape_angle_tolerance:
                error_angle = 0
            elif error_x and error_y and error_angle == 0:
                return True
                self.done()
            else:
                self.chassis.set_inputs(
                    error_x * self.ground_tape_kP_x,
                    error_y * self.ground_tape_kP_y,
                    error_angle * self.ground_tape_kP_angle,
                    field_oriented=False,
                )
        if positioned_to_outake:
            self.successful = True
            self.done()
