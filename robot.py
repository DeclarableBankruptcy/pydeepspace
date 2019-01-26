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


class Robot(magicbot.MagicRobot):
    module_drive_free_speed: float = 84000.0  # encoder ticks / 100 ms
    chassis: SwerveChassis

    def createObjects(self):
        """Create motors and stuff here."""

        # a + + b + - c - - d - +
        x_dist = 0.25
        y_dist = 0.25
        self.module_a = SwerveModule(  # top right module now back left
            # "a", steer_talon=ctre.TalonSRX(48), drive_talon=ctre.TalonSRX(49),
            "a",
            steer_talon=ctre.TalonSRX(1),
            drive_talon=ctre.TalonSRX(2),
            x_pos=x_dist,
            y_pos=y_dist,  # x = 0.33 y = 0.28
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
            reverse_drive_direction=True,
        )
        self.module_b = SwerveModule(  # bottom left module now front right
            "b",
            steer_talon=ctre.TalonSRX(3),
            drive_talon=ctre.TalonSRX(4),
            x_pos=-x_dist,
            y_pos=y_dist,  # x = 0.31, y = 0.28
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
            reverse_drive_direction=True,
        )
        self.module_c = SwerveModule(  # top right module now back left
            # "a", steer_talon=ctre.TalonSRX(48), drive_talon=ctre.TalonSRX(49),
            "c",
            steer_talon=ctre.TalonSRX(5),
            drive_talon=ctre.TalonSRX(6),
            x_pos=-x_dist,
            y_pos=-y_dist,  # x = 0.33 y = 0.28
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
            reverse_drive_direction=True,
        )
        self.module_d = SwerveModule(  # bottom left module now front right
            "d",
            steer_talon=ctre.TalonSRX(7),
            drive_talon=ctre.TalonSRX(8),
            x_pos=x_dist,
            y_pos=-y_dist,  # x = 0.31, y = 0.28
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
            reverse_drive_direction=True,
        )
        self.imu = NavX()
        self.pursuit = PurePursuit(look_ahead=0.2, ending_tolerance=0.1)

        self.sd = NetworkTables.getTable("SmartDashboard")

        # boilerplate setup for the joystick
        self.joystick = wpilib.Joystick(0)

        self.spin_rate = 1.5

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.chassis.set_inputs(0, 0, 0)

    def teleopPeriodic(self):
        """
        Process inputs from the driver station here.
        This is run each iteration of the control loop before magicbot components are executed.
        """
        if self.joystick.getRawButtonPressed(7):
            pass

        if self.joystick.getRawButtonPressed(10):
            self.imu.resetHeading()
            self.chassis.set_heading_sp(0)

        throttle = (1 - self.joystick.getThrottle()) / 2
        self.sd.putNumber("joy_throttle", throttle)

        # this is where the joystick inputs get converted to numbers that are sent
        # to the chassis component. we rescale them using the rescale_js function,
        # in order to make their response exponential, and to set a dead zone -
        # which just means if it is under a certain value a 0 will be sent
        # TODO: Tune these constants for whatever robot they are on
        joystick_vx = -rescale_js(
            self.joystick.getY(), deadzone=0.1, exponential=1.5, rate=4 * throttle
        )
        joystick_vy = -rescale_js(
            self.joystick.getX(), deadzone=0.1, exponential=1.5, rate=4 * throttle
        )
        joystick_vz = -rescale_js(
            self.joystick.getZ(), deadzone=0.2, exponential=20.0, rate=self.spin_rate
        )
        joystick_hat = self.joystick.getPOV()

        self.sd.putNumber("joy_vx", joystick_vx)
        self.sd.putNumber("joy_vy", joystick_vy)
        self.sd.putNumber("joy_vz", joystick_vz)

        if joystick_vx or joystick_vy or joystick_vz:
            self.chassis.set_inputs(
                joystick_vx,
                joystick_vy,
                joystick_vz,
                field_oriented=not self.joystick.getRawButton(6),
            )
        else:
            self.chassis.set_inputs(0, 0, 0)
            # self.module_a.steer_motor.stop()
            # self.module_b.steer_motor.stop()
            # self.module_a.drive_motor.stop()
            # self.module_b.drive_motor.stop()

        if joystick_hat != -1:
            constrained_angle = -constrain_angle(math.radians(joystick_hat))
            self.chassis.set_heading_sp(constrained_angle)

    def testPeriodic(self):
        joystick_vx = -rescale_js(
            self.joystick.getY(), deadzone=0.1, exponential=1.5, rate=0.5
        )
        self.sd.putNumber("joy_vx", joystick_vx)

        # 7 8
        # 9 10
        if self.joystick.getRawButton(7):
            self.module_a.store_steer_offsets()
            self.module_a.steer_motor.set(ctre.ControlMode.PercentOutput, joystick_vx)
        if self.joystick.getRawButton(9):
            self.module_b.store_steer_offsets()
            self.module_b.steer_motor.set(ctre.ControlMode.PercentOutput, joystick_vx)

        if self.joystick.getRawButton(10):
            self.module_c.store_steer_offsets()
            self.module_c.steer_motor.set(ctre.ControlMode.PercentOutput, joystick_vx)
        if self.joystick.getRawButton(8):
            self.module_d.store_steer_offsets()
            self.module_d.steer_motor.set(ctre.ControlMode.PercentOutput, joystick_vx)

    def robotPeriodic(self):
        # super().robotPeriodic()
        self.sd.putNumber("imu_heading", self.imu.getAngle())
        for module in self.chassis.modules:
            self.sd.putNumber(
                module.name + "_pos_steer",
                module.steer_motor.getSelectedSensorPosition(0),
            )
            try:
                self.sd.putNumber(module.name + "_setpoint", module.setpoint)
            except:
                pass


if __name__ == "__main__":
    wpilib.run(Robot)