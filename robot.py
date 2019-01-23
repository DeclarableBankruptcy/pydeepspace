#!/usr/bin/env python3

import magicbot
import wpilib
import ctre
from pyswervedrive.swervechassis import SwerveChassis
from pyswervedrive.swervemodule import SwerveModule
from utilities.navx import NavX
from utilities.pure_pursuit import PurePursuit


class Robot(magicbot.MagicRobot):
    module_drive_free_speed: float = 84000.0  # encoder ticks / 100 ms
    chassis: SwerveChassis
    def createObjects(self):
        """Create motors and stuff here."""
        self.module_a = SwerveModule(  # top right module now back left
            # "a", steer_talon=ctre.TalonSRX(48), drive_talon=ctre.TalonSRX(49),
            "a",
            steer_talon=ctre.TalonSRX(42),
            drive_talon=ctre.TalonSRX(48),
            x_pos=-(0.375-0.160),
            y_pos=0.375-0.100,  # x = 0.33 y = 0.28
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
        )
        self.module_b = SwerveModule(  # bottom left module now front right
            "b",
            steer_talon=ctre.TalonSRX(58),
            drive_talon=ctre.TalonSRX(2),
            x_pos=0.375-0.160,
            y_pos=-(0.375-0.100),  # x = 0.31, y = 0.28
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
        )
        self.module_c = SwerveModule(  # top right module now back left
            # "a", steer_talon=ctre.TalonSRX(48), drive_talon=ctre.TalonSRX(49),
            "c",
            steer_talon=ctre.TalonSRX(51),
            drive_talon=ctre.TalonSRX(52),
            x_pos=-(0.375-0.160),
            y_pos=-(0.375-0.100),  # x = 0.33 y = 0.28
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
        )
        self.module_d = SwerveModule(  # bottom left module now front right
            "d",
            steer_talon=ctre.TalonSRX(53),
            drive_talon=ctre.TalonSRX(54),
            x_pos=0.375-0.160,
            y_pos=(0.375-0.100),  # x = 0.31, y = 0.28
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
        )
        self.imu = NavX()
        self.pursuit = PurePursuit(look_ahead=0.2, ending_tolerance=0.1)

    def teleopInit(self):
        """Initialise driver control."""
        pass

    def teleopPeriodic(self):
        """Allow the drivers to control the robot."""
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
