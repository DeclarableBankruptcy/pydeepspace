import hal

from networktables import NetworkTables


class Vision:
    def __init__(self):
        self.nt = NetworkTables.getTable("/vision")
        self.target_tape_error = self.nt.getEntry("target_tape_error")
        self.target_tape_error.addListener(
            self.new_target_value,
            NetworkTables.NotifyFlags.NEW | NetworkTables.NotifyFlags.UPDATE,
        )

        self.target_tape_deposting_range = self.nt.getEntry(
            "target_tape_deposting_range"
        )
        self.target_tape_deposting_range.addListener(
            self.new_target_value_deposting_range,
            NetworkTables.NotifyFlags.NEW | NetworkTables.NotifyFlags.UPDATE,
        )

        self.ground_tape_error_x = self.nt.getEntry("ground_tape_error_x")
        self.ground_tape_error_x.addListener(
            self.new_ground_value_y,
            NetworkTables.NotifyFlags.NEW | NetworkTables.NotifyFlags.UPDATE,
        )

        self.ground_tape_y = self.nt.getEntry("ground_tape_y")
        self.ground_tape_y.addListener(
            self.new_ground_value_y,
            NetworkTables.NotifyFlags.NEW | NetworkTables.NotifyFlags.UPDATE,
        )

        self.ground_tape_angle = self.nt.getEntry("ground_tape_angle")
        self.ground_tape_angle.addListener(
            self.new_ground_value_angle,
            NetworkTables.NotifyFlags.NEW | NetworkTables.NotifyFlags.UPDATE,
        )

        self.positioned_to_outake = self.nt.getEntry("positioned_to_outake")
        self.positioned_to_outake.addListener(
            self.new_positioned_to_outake_value,
            NetworkTables.NotifyFlags.NEW | NetworkTables.NotifyFlags.UPDATE,
        )

        # network tables doesn't like none, so we use out of bound values
        self.target_tape_error_value = 999
        self.ground_x_value = 999
        self.ground_y_value = 999
        self.ground_angle_value = 999
        self.positioned_to_outake_value = False

    def new_target_value(self, entry, key, value, param):
        # self.time = time.monotonic()
        self.target_tape_error_value = value

    def new_ground_x_value(self, entry, key, value, param):
        # self.time = time.monotonic()
        self.ground_x_value = value

    def new_ground_y_value(self, entry, key, value, param):
        # self.time = time.monotonic()
        self.ground_y_value = value

    def new_ground_value_angle(self, entry, key, value, param):
        # self.time = time.monotonic()
        self.ground_angle_value = value

    def new_positioned_to_outake_value(self, entry, key, value, param):
        # self.time = time.monotonic()
        self.positioned_to_outake_value = value

    def get_target_tape_error(self):
        if not -1 <= self.target_tape_error_value <= 1:
            return None
        elif hal.isSimulation():
            return 0
        else:
            return self.target_tape_error_value

    def get_target_tape_deposting_ragne(self):
        if self.target_tape_deposting_range == True:
            return True
        else:
            return False

    def get_ground_tape_error_x(self):
        if not -1 <= self.ground_x_value <= 1:
            return None
        elif hal.isSimulation():
            return 0
        else:
            return self.ground_x_value

    def get_ground_tape_error_y(self):
        if not -1 <= self.ground_y_value <= 1:
            return None
        elif hal.isSimulation():
            return 0
        else:
            return self.ground_y_value

    def get_ground_tape_error_angle(self):
        if not -1 <= self.ground_angle_value <= 1:
            return None
        elif hal.isSimulation():
            return 0
        else:
            return self.ground_angle_value

    def get_positioned_to_outake(self):
        return self.positioned_to_outake_value
