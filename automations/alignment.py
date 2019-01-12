import math
import wpilib

from vision import Vision
from magicbot.state_machine import StateMachine, state
from pyswervedrive.chassis import Chassis
from utilities.imu import IMU


class AlignmentStateMachine(StateMachine):
    """A state machine designed to help the robot align using the vision
    system"""
    vision: Vision
    chassis: Chassis
    imu: IMU

    def __init__(self, angle_tolerance, range_tolerance):
        self.angle_tolerance = angle_tolerance
        self.range_tolerance = range_tolerance

    @state(first=True)
    def check_ground(self):
        """check if we can see vision tape on the ground before """
        if self.vision.ground_tape_angle == []:  # TODO vision sees ground tape
            self.next_state_now("ground_align")
        else:
            self.next_state_now("target_tape_align")

    @state
    def ground_tape_align(self):
        """Attempt to align with the objective by way of the vision tape on the ground,
        correct errors untill they aare within tolerance"""
        heading = self.imu.getAngle()
        if ((abs(self.vision.ground_tape_angle) > self.angle_tolerance)
           or (abs(self.vision.ground_tape_distance) > self.range_tolerance)):
            self.chassis.set_inputs(self.vision.ground_tape_distance *
                                    math.cos(self.vision.ground_tape_angle),
                                    self.vision.ground_tape_distance *
                                    math.sin(self.vision.ground_tape_angle),
                                    heading + self.vision.ground_tape_angle)
        else:
            self.done()

    @state
    def target_tape_align(self):
        heading = self.imu.getAngle()
        if ((abs(self.vision.target_tape_angle) > self.angle_tolerance)
           or (abs(self.vision.target_tape_distance) > self.range_tolerance)):
            self.chassis.set_inputs(self.vision.target_tape_distance *
                                    math.cos(self.vision.target_tape_angle),
                                    self.vision.target_tape_distance *
                                    math.sin(self.vision.target_tape_angle),
                                    heading + self.vision.ground_tape_angle)
        else:
            self.next_state_now("ground_tape_align")
