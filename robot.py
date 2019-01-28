#!/usr/bin/env python3

import math

import magicbot
import wpilib

from automations.align import Aligner
from automations.cargo import CargoManager
from automations.climb import Climber
from automations.hatch import HatchController
from components.arm import Arm
from components.hatch import Hatch
from components.intake import Intake
from components.lift import Lift
from utilities.functions import constrain_angle, rescale_js


class Robot(magicbot.MagicRobot):

    align: Aligner
    arm: Arm
    cargoman: CargoManager
    climb: Climber
    hatch: Hatch
    hatchman: HatchController
    intake: Intake
    back_lift: Lift
    front_lift: Lift

    def createObjects(self):
        """Create motors and stuff here."""
        # Arm
        self.arm_main_piston = wpilib.Solenoid(0)
        self.arm_helper_piston = wpilib.Solenoid(1)

        # Intake
        self.intake_motor = ctre.TalonSRX(0)
        self.intake_switch = wpilib.DigitalInput(0)

        # Lift
        self.back_lift_motor = rev.CANSparkMax(1, rev.MotorType.kBrushless)
        self.back_lift_drive_motor = ctre.TalonSRX(2)
        self.back_lift_switch = wpilib.DigitalInput(1)
        self.front_lift_motor = rev.CANSparkMax(3, rev.MotorType.kBrushless)
        self.front_lift_drive_motor = ctre.TalonSRX(4)
        self.front_lift_switch = wpilib.DigitalInput(2)

        # Controlers
        self.joystick = wpilib.Joystick(1)
        self.gamepad = wpilib.XboxController(0)

        # Controller related variables
        self.spin_rate = 1.5
        self.snaps = [math.radians(a * 45) for a in range(-3, 5)]

    def teleopInit(self):
        """Initialise driver control."""
        pass

    def teleopPeriodic(self):
        """Allow the drivers to control the robot."""
        # Handle co-driver input
        # Intaking
        if self.gamepad.getAButtonPressed():
            self.hatch_controller.start_punch(force=True)
        if self.gamepad.getBButtonPressed():
            self.cargoman.intake_depot(force=True)
        if self.gamepad.getXButtonPressed():
            self.hatch.intake(force=True)  # might remove
        if self.gamepad.getYButtonPressed():
            self.cargoman.intake_loading(force=True)

        # Cargo
        pov = self.gamepad.getPOV()
        if pov == 0:
            self.arm.elevate()
        elif pov == 180:
            self.arm.lower()

        # Outaking
        if (
            self.gamepad.getTriggerAxis(self.gamepad.Hand.kLeft) > 0
            or self.gamepad.getTriggerAxis(self.gamepad.Hand.kRight) > 0
        ):
            if self.intake.contained():
                if not self.cargoman.is_executing:
                    self.cargoman.override = self.gamepad.getBumper(
                        self.gamepad.Hand.kLeft
                    )
                    self.cargoman.start_outtake(force=True)
            elif self.intake.contained():
                if not self.hatch.is_executing:
                    self.hatch.override = self.gamepad.getBumper(
                        self.gamepad.Hand.kLeft
                    )
                    self.hatch.outtake(force=True)

        # Climbing
        if self.gamepad.getStartButtonPressed():
            self.climb.climb(force=True)
        if self.gamepad.getBackButtonPressed():
            self.climb.reset(force=True)

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
