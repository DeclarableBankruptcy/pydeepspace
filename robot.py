#!/usr/bin/env python3
import math
import numpy as np

import ctre
import magicbot
import wpilib
from pyswervedrive.swervechassis import SwerveChassis
from pyswervedrive.swervemodule import SwerveModule
from utilities.functions import rescale_js, constrain_angle
from utilities.navx import NavX
from utilities.pure_pursuit import PurePursuit
from networktables import NetworkTables

# ctre.TalonSRX.Notifier = None

from automations.alignment import Aligner
from automations.hatch import HatchController
from components.hatch import Hatch
from utilities.functions import constrain_angle, rescale_js


class Robot(magicbot.MagicRobot):

    chassis: SwerveChassis
    align: Aligner
    hatch: Hatch
    hatchman: HatchController

    module_drive_free_speed: float = 94000.0


    def createObjects(self):
        """Create motors and stuff here."""
        x_dist = 0.2165
        y_dist = 0.2625
        self.module_a = SwerveModule(  # top right module now back left
            "a",
            steer_talon=ctre.TalonSRX(1),
            drive_talon=ctre.TalonSRX(2),
            x_pos=x_dist,
            y_pos=y_dist,
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
            reverse_drive_direction=True,
        )
        self.module_b = SwerveModule(  # bottom left module now front right
            "b",
            steer_talon=ctre.TalonSRX(3),
            drive_talon=ctre.TalonSRX(4),
            x_pos=-x_dist,
            y_pos=y_dist,
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_drive_encoder=True,
            reverse_steer_encoder=True,
        )
        self.module_c = SwerveModule(  # top right module now back left
            "c",
            steer_talon=ctre.TalonSRX(5),
            drive_talon=ctre.TalonSRX(6),
            x_pos=-x_dist,
            y_pos=-y_dist,
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
            reverse_drive_direction=True,
        )
        self.module_d = SwerveModule(  # bottom left module now front right
            "d",
            steer_talon=ctre.TalonSRX(7),
            drive_talon=ctre.TalonSRX(8),
            x_pos=x_dist,
            y_pos=-y_dist,
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
            reverse_drive_direction=True,
        )
        self.imu = NavX()

        # Controlers
        self.joystick = wpilib.Joystick(1)
        self.gamepad = wpilib.XboxController(0)

        # Controller related variables
        self.spin_rate = 1.5
        self.snaps = [math.radians(a * 45) for a in range(-3, 5)]

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.chassis.set_inputs(0, 0, 0)

    def teleopPeriodic(self):
        """Allow the drivers to control the robot."""
        # Handle co-driver input
        # Intaking
        if self.gamepad.getAButtonPressed():
            self.hatchman.start_punch(force=True)

        # Outaking
        if (
            self.gamepad.getTriggerAxis(self.gamepad.Hand.kLeft) > 0
            or self.gamepad.getTriggerAxis(self.gamepad.Hand.kRight) > 0
        ):
            if not self.hatch.is_executing:
                self.hatch.override = self.gamepad.getBumper(
                    self.gamepad.Hand.kLeft
                )
                self.hatch.outtake(force=True)


        # Snap to angle
        # x is set to -y and y is set x to rotate the coordinate system counter
        # clockwise to follow our system for giving directions
        x = -rescale_js(self.gamepad.getY(self.gamepad.Hand.kLeft), deadzone=0.5)
        y = rescale_js(self.gamepad.getX(self.gamepad.Hand.kLeft), deadzone=0.5)

        if x != 0.0 or y != 0.0:
            direction = math.atan2(y, x)
            snapped_angle = min(self.snaps, key=lambda x: abs(x - direction))
            self.chassis.set_heading_sp(snapped_angle)

        # Handle driver input
        # Driving
        throttle = (
            1 - self.joystick.getThrottle()
        ) / 2  # TODO: don't set to 0 when not turned on
        joystick_vx = -rescale_js(
            self.joystick.getY(), deadzone=0.1, exponential=1.5, rate=4 * throttle
        )
        joystick_vy = -rescale_js(
            self.joystick.getX(), deadzone=0.1, exponential=1.5, rate=4 * throttle
        )
        joystick_vz = -rescale_js(
            self.joystick.getZ(), deadzone=0.2, exponential=20.0, rate=self.spin_rate
        )

        if joystick_vx or joystick_vy or joystick_vz:
            self.chassis.set_inputs(
                joystick_vx,
                joystick_vy,
                joystick_vz,
                field_oriented=not self.joystick.getRawButton(6),
            )
        else:
            self.chassis.set_inputs(0, 0, 0)

        # Snap to angle
        joystick_hat = self.joystick.getPOV()

        if joystick_hat != -1:
            constrained_angle = -constrain_angle(math.radians(joystick_hat))
            self.chassis.set_heading_sp(math.radians(constrained_angle))


if __name__ == "__main__":
    wpilib.run(Robot)
