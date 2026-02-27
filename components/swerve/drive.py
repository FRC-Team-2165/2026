from .module import SwerveModule, SwerveModuleConfig
from .vector import Polar, Cartesian

from wpimath import applyDeadband
from wpimath.geometry import Translation2d

from wpilib.drive import RobotDriveBase


def clamp(value, min, max):
    if value < min:
        return min
    elif value > max:
        return max
    else:
        return value


class SwerveDrive(RobotDriveBase):
    modules: list[SwerveModule]
    _position: Polar
    deadband: dict[str, float]

    def __init__(self, *configs: SwerveModuleConfig, deadband: float = 0, starting_position: Polar = Polar(0,0)):
        super().__init__()
        self.modules = list(SwerveModule(cfg) for cfg in configs)
        self.deadband = {
            "x": deadband,
            "y": deadband,
            "z": deadband,
        }

        self._position = starting_position.to_polar()

    def set_all_deadband(self, deadband: float) -> None:
        self.deadband["x"] = deadband
        self.deadband["y"] = deadband
        self.deadband["z"] = deadband

    def set_x_deadband(self, deadband: float) -> None:
        self.deadband["x"] = deadband

    def set_y_deadband(self, deadband: float) -> None:
        self.deadband["y"] = deadband

    def set_rotation_deadband(self, deadband: float) -> None:
        self.deadband["z"] = deadband

    def getDescription(self) -> str:
        return "SwerveDrive"

    def drive(self, x_speed: float, y_speed: float, rot: float, current_angle: float = 0, square_inputs: bool = False) -> None:
        """
        Moves the drivetrain according to the inputs from the cartesian inputs. This method is
        designed to easily take inputs from a controller. As such, inputs are clamped to the range [-1, 1].

        The expected movement is as follows:
        - +x: right, -x: left
        - +y: forward, -y: reverse
        - +rot: CW, -rot: CCW


        You can enable field-relative drive by supplying the `current_angle` argument. Most often
        this will be a value supplied by a gyro. However, to use traditional "robot-relative" drive,
        either don't supply the argument, or constantly set it to 0. This is because a robot-relative
        drive is the same as a field-relative drive, but where the "gyro" input is always 0 ("forward").

        Inputs are trimmed by the supplied deadband parameters after squaring, if used.
        """

        x_speed = clamp(x_speed, -1, 1)
        y_speed = clamp(y_speed, -1, 1)
        rot = clamp(rot, -1, 1)

        if square_inputs:
            x_speed *= abs(x_speed)
            y_speed *= abs(y_speed)
            rot *= abs(rot)

        x_speed = applyDeadband(x_speed, self.deadband["x"])
        y_speed = applyDeadband(y_speed, self.deadband["y"])
        rot = applyDeadband(rot, self.deadband["z"])


        # Convert cartesian vector input to polar vector. Makes all of the math *much* simpler.
        target_vector = Cartesian(x_speed, -y_speed).to_polar()
        target_vector.theta -= current_angle

        max_module_distance = max(m.offset_from_center() for m in self.modules)

        module_states: list[Polar] = []
        # Combine target vector with rotation state for each module.
        # Final vector is a sum of weighted rotation vector and target vector.
        for m in self.modules:
            rotation_angle = m.rotation_angle()
            if rot > 0:
                rotation_angle = (rotation_angle + 180) % 360
            rotation_speed = abs(rot) * (m.offset_from_center() / max_module_distance)
            rotation = Polar(rotation_speed, rotation_angle)

            module_states.append(target_vector + rotation)

        # Desaturate (normalize) module speeds. Very necessary.
        top_speed = max(m.magnitude for m in module_states)
        if top_speed > 1:
            for state in module_states:
                state.magnitude /= top_speed


        for i, s in enumerate(module_states):
            if s.magnitude == 0:
                # only stop the drive motors, but don't reset the angle
                self.modules[i].speed = 0
            else:
                self.modules[i].set_state(s)
        self.feed()

        net_vector = sum((m.get_state_mps() for m in self.modules), start=Polar.zero())
        net_vector.theta += current_angle
        net_vector.magnitude *= 0.02 / len(self.modules) # 0.02 from robot running at 50Hz.
        self._position += net_vector

    def initialize(self) -> None:
        """
        Puts the swerve module wheels facing parallel to the orientation of the robot.

        Optimizes the module angles to minimize wheel movement. Cancelled by any other drive call.
        """
        for m in self.modules:
            m.set_state(Polar(0, 90))
        self.feed()

    def brace(self) -> None:
        """
        Put the swerve module wheels into an "X" position at a full stop to maximize resistance
        to being moved involuntarily.

        Optimizes angle of the "X" relative to the module's previous angle. Cancelled by any other drive call.
        """
        for m in self.modules:
            m.angle = m.rotation_angle() - 90
        self.feed()

    def stopMotor(self) -> None:
        """
        Disables the drive until `drive` is called again.
        """
        for m in self.modules:
            m.stopMotor()
        self.feed()

    def location(self) -> Translation2d:
        # May not account for rotation, but also may be a non-issue

        trans = self._position.to_translation2d()
        return Translation2d(-trans.X(), trans.Y())

    def reset_position(self) -> None:
        """
        Resets the tracked position of the drive. Does not affect the state of the drive, only the odometry.
        """
        self._position = Polar.zero()
