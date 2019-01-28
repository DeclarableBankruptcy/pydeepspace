import wpilib


class Hatch:
    top_puncher: wpilib.Solenoid
    left_puncher: wpilib.Solenoid
    right_puncher: wpilib.Solenoid

    top_limit_switch: wpilib.DigitalInput
    left_limit_switch: wpilib.DigitalInput
    right_limit_switch: wpilib.DigitalInput

    def __init__(self):
        self.punch_on_top = False
        self.punch_on_bottom = False

    def execute(self):
        """Run at the end of every control loop iteration."""
        self.top_puncher.set(self.punch_on_bottom)
        self.left_puncher.set(self.punch_on_top)
        self.right_puncher.set(self.punch_on_top)

    def punch_top(self):
        self.punch_on_top = True

    def punch_bottom(self):
        self.punch_on_bottom = True

    def retract(self):
        self.punch_on_top = self.punch_on_bottom = False

    def contained(self):
        return any(
            [
                not self.top_limit_switch.get(),
                not self.left_limit_switch.get(),
                not self.right_limit_switch.get(),
            ]
        )
