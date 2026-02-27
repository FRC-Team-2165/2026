from commands2 import Subsystem
from enum import Flag
from dataclasses import dataclass
from wpimath.geometry import Pose3d, Translation3d, Rotation3d
import math

from wpilib import AnalogEncoder
from wpilib import Servo
from wpimath.controller import PIDController
from rev import SparkMax
import rev
from phoenix5 import WPI_TalonSRX

class TargetingStatus(Flag):
    Set = 0,
    Adjusting = 1,
    TooLow = 2,
    TooHigh = 3,

    def is_ok(self) -> bool:
        return TargetingStatus.TooHigh not in self and TargetingStatus.TooLow not in self

    def is_active(self) -> bool:
        return TargetingStatus.Adjusting not in self


@dataclass
class Range:
    min: float
    max: float

    def __contains__(self, item: float) -> bool:
        return self.min <= item <= self.max

    def clamp(self, value: float) -> float:
        return max(self.min, min(value, self.max))

NEO_MAX_RPM = 5676
# ft/s - assuming a 2-inch diameter wheel
NEO_MAX_SPEED = NEO_MAX_RPM * 2 * math.pi / 7200


class ShooterSubsystem(Subsystem):
    _target_elevation: float
    requested_elevation: float
    elevation_tolerance: float
    elevation_range: Range

    _target_angle: float
    requested_angle: float
    angle_tolerance: float
    angle_range: Range
    angle_controller: PIDController

    angle_motor: WPI_TalonSRX
    angle_sensor: AnalogEncoder

    elevation_servo: Servo

    shooters: list[SparkMax]
    shooter_encoders: list[rev.SparkAbsoluteEncoder]

    def __init__(self):
        super().__init__()

        self._target_elevation = 0.0
        self.requested_elevation = 0.0
        self.elevation_tolerance = 0.1
        self.elevation_range = Range(0.0, 360.0)

        self._target_angle = 0.0
        self.requested_angle = 0.0
        self.angle_tolerance = 0.1
        self.angle_range = Range(0.0, 360.0)

        #TODO Figure out coefficients
        self.angle_controller = PIDController(1/45, 0, 0)
        self.angle_controller.setTolerance(1) # 1 degree

        # TODO put 3d coordinates relative to center of robot
        self.shooter_mouth = Translation3d()

        self.enabled = False

        turntable_ratio = 96/12
        full_angle_range = (360 / turntable_ratio) * 10
        self.angle_slop = (10 - turntable_ratio) * full_angle_range / 2

        self.angle_sensor = AnalogEncoder(0, full_angle_range, full_angle_range / 2)
        self.angle_motor = WPI_TalonSRX(19)

        # CHECK this may be more usable as a direct PWM output
        # TODO BIG TODO This servo has extremely poor documentation, and everything will need to be checked
        # It will likely have a practical range of ~90 degrees, but that means nothing to the controller
        self.elevation_servo = Servo(0)
        config = rev.SparkBaseConfig().encoder.velocityConversionFactor(2 * math.pi / 7200) #ft/s assuming 2in wheels
        shooter_left = SparkMax(20, SparkMax.MotorType.kBrushless)
        shooter_left.configure(config, rev.ResetMode.kNoResetSafeParameters, rev.PersistMode.kNoPersistParameters)
        shooter_right = SparkMax(21, SparkMax.MotorType.kBrushless)
        shooter_right.configure(config, rev.ResetMode.kNoResetSafeParameters, rev.PersistMode.kNoPersistParameters)
        self.shooters = [shooter_left, shooter_right]
        self.shooter_encoders = [s.getAbsoluteEncoder() for s in self.shooters]

        target_shooter_speed = 35 #ft/s
        self.shooter_speed = target_shooter_speed / NEO_MAX_SPEED

    def enable_launcher(self) -> None:
        self.enabled = True

    def disable_launcher(self) -> None:
        self.enabled = False

    def toggle_launcher(self) -> None:
        self.enabled = not self.enabled

    def launcher_enabled(self) -> bool:
        return self.enabled

    def elevation_status(self) -> TargetingStatus:
        if abs(self.elevation - self.target_elevation) < self.elevation_tolerance:
            res = TargetingStatus.Set
        else:
            res = TargetingStatus.Adjusting

        if self.requested_elevation > self.elevation_range.max:
            res |= TargetingStatus.TooHigh
        elif self.requested_elevation < self.elevation_range.min:
            res |= TargetingStatus.TooLow

        return res

    def angle_status(self) -> TargetingStatus:
        if abs(self.angle - self.target_angle) < self.angle_tolerance:
            res = TargetingStatus.Set
        else:
            res = TargetingStatus.Adjusting

        if self.requested_angle > self.angle_range.max:
            res |= TargetingStatus.TooHigh
        elif self.requested_angle < self.angle_range.min:
            res |= TargetingStatus.TooLow

        return res

    def request_elevation(self, target: float) -> float:
        """requests the elevation change to the specific value. Returns the actual target elevation, which may be
        different from the requested value."""
        self.requested_elevation = target
        self._target_elevation = self.elevation_range.clamp(target)
        return self._target_elevation

    def request_angle(self, target: float) -> float:
        """requests the angle change to the specific value. Returns the actual target angle, which may be
        different from the requested value."""
        target %= 360
        self.requested_angle = target
        self._target_angle = self.angle_range.clamp(target)
        return self._target_angle

    @property
    def elevation(self) -> float:
        # CHECK return shooter elevation detected by encoder
        #     At present, this isn't possible, since it's a servo, and there isn't an encoder
        #  This will always return the target elevation, not the actual, so the elevation_status
        #  will always register as set
        return self.elevation_servo.getAngle()


    @elevation.setter
    def elevation(self, target: float) -> None:
        self.request_elevation(target)

    @property
    def target_elevation(self) -> float:
        return self._target_elevation

    @property
    def angle(self) -> float:
        # The encoder has a larger range than 360 in practice, so this corrects for that
        return self.angle_sensor.get() - self.angle_slop

    @angle.setter
    def angle(self, target: float) -> None:
        self.request_angle(target)

    @property
    def target_angle(self) -> float:
        return self._target_angle

    @property
    def position(self) -> Pose3d:
        return Pose3d(self.shooter_mouth, Rotation3d.fromDegrees(0, 0, self.angle))

    def periodic(self) -> None:
        # enable or disable shooters
        for shooter in self.shooters:
            # These have internal controllers, so this should be sufficient
            shooter.set(self.shooter_speed if self.enabled else 0)

        speed = self.angle_controller.calculate(self.angle)
        self.angle_motor.set(speed)

        # TODO Set servo angle based on target_elevation

