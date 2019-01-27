import math
import numpy as np
from magicbot.state_machine import AutonomousStateMachine, state
from utilities.pure_pursuit import PurePursuit
from pyswervedrive.swervechassis import SwerveChassis
from utilities.navx import NavX


class TestPursuitAuto(AutonomousStateMachine):

    MODE_NAME = "Test Pursuit Auto"
    DEFAULT = True

    chassis: SwerveChassis
    pursuit: PurePursuit
    imu: NavX

    # Kp = 4
    # Ki = 0.1
    # Kd = 0

    def __init__(self):
        super().__init__()
        self.points = [(0.0, 0), (0, 0)]
        # self.loops = 1
        # self.error_i = 0
        # self.last_error = 0

    def on_enable(self):
        super().on_enable()
        self.chassis.odometry_x = 0
        self.chassis.odometry_y = 0

    @state(first=True)
    def move_forwards(self, initial_call):
        if initial_call:
            self.chassis.set_inputs(1, 0, 0)
            # self.chassis.heading_hold_on()
            # self.chassis.set_heading_sp(0)
        if self.chassis.odometry_x > 2:
            self.chassis.set_inputs(0, 0, 0)
            self.done()
        # if initial_call:
        #     self.pursuit.build_path(self.points)
        #     self.chassis.heading_hold_on()
        #     self.chassis.set_heading_sp(0)
        #     # self.loops = 1
        # heading = self.imu.getAngle()
        # x, y = self.chassis.position
        # position = np.array((x, y, heading))
        # changed_segment, vector = self.pursuit.compute_direction(position)
        # if self.pursuit.completed_path or changed_segment:
        #     self.chassis.set_inputs(0, 0, 0, field_oriented=False)
        #     self.done()
        #     return
        # normalised = vector / np.linalg.norm(vector)
        # self.chassis.set_inputs(normalised[0]*2, normalised[1]*2, 0)

        # # vx, vy = vector[0], vector[1]
        # # if changed_waypoint:
        # #     self.error_i = 0
        # #     self.last_error = 0
        # # vector = np.array((vx, vy))

        # if self.pursuit.distance_along_path(self.chassis.position) >= 1:
        #     self.chassis.set_inputs(0, 0, 0)
        #     self.done()
        # error = np.linalg.norm(
        #     position[:2]
        #     - self.pursuit.waypoints[self.pursuit.current_waypoint_number + 1]
        # )
        # self.error_i += error
        # error_d = error - self.last_error
        # output = error * self.Kp + self.Ki / self.error_i + self.Kd * (error_d / 20)
        # print(output)
        # self.chassis.set_inputs(output * vx, output * vy, 0)
        # self.loops += 1
        # last_error = error
