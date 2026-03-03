import wpilib
from commands2 import Subsystem
from enum import Flag
from dataclasses import dataclass
from wpimath.geometry import Pose3d, Translation3d, Rotation3d
import math
import pyerf

from wpilib import AnalogEncoder
from wpilib import Servo
from wpimath.controller import PIDController
from rev import SparkMax
import rev
from phoenix5 import WPI_TalonSRX

from wpilib import SmartDashboard as sd

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
        out =  max(self.min, min(value, self.max))
        return out

    def size(self) -> float:
        return self.max - self.min

    def coerce(self, n: float) -> float:
        value = (n - self.min) % self.size() + self.min
        return value

NEO_MAX_RPM = 5676
# ft/s - assuming a 2-inch diameter wheel
NEO_MAX_SPEED = NEO_MAX_RPM * 2 * math.pi / 720


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

        self.angle_sensor = AnalogEncoder(0, 450, 281.6)
        self.angle_motor = WPI_TalonSRX(19)
        self.angle_motor.setInverted(True)

        self._target_elevation = 0.0
        self.requested_elevation = 0.0
        self.elevation_tolerance = 0.1
        self.elevation_range = Range(5.0, 21.0)

        self.angle_range = Range(-255, 105)
        self._target_angle = self.angle
        self.requested_angle = 0.0
        self.angle_tolerance = 1

        #TODO Figure out coefficients
        self.large_kp = 0.015
        self.large_ki = 0.002
        self.large_kd = 0.00000
        self.angle_controller = PIDController(self.large_kp, self.large_ki, self.large_kd)
        self.angle_controller.setTolerance(1) # 1 degree

        # TODO put 3d coordinates relative to center of robot
        self.shooter_mouth = Translation3d()

        self.enabled = False

        turntable_ratio = 96/12
        full_angle_range = (360 / turntable_ratio) * 10
        self.angle_slop = (10 - turntable_ratio) * full_angle_range / 2


        # The servo is 110 degrees, and has a useful input range of 0.19 to 0.84, decreasing as it raises
        self.elevation_servo = Servo(0)

        config = rev.SparkBaseConfig()
        config.encoder.velocityConversionFactor(2 * math.pi / 720) #ft/s assuming 2in wheels
        config.inverted(True)
        shooter_left = SparkMax(20, SparkMax.MotorType.kBrushless)
        shooter_left.configure(config, rev.ResetMode.kNoResetSafeParameters, rev.PersistMode.kNoPersistParameters)
        config.inverted(False)
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
        coerced_target = self.angle_range.coerce(target) # (-255, 105]
        self.requested_angle = coerced_target
        self._target_angle = self.angle_range.clamp(coerced_target)
        return self._target_angle

    @property
    def elevation(self) -> float:
        # CHECK return shooter elevation detected by encoder
        #     At present, this isn't possible, since it's a servo, and there isn't an encoder
        #  This will always return the target elevation, not the actual, so the elevation_status
        #  will always register as set
        # servo_value = self.elevation_servo.get()
        # tilt = 11.08 * (1 + pyerf.erf(0.0159 * servo_value - 0.54))
        # return tilt
        return self._target_elevation


    @elevation.setter
    def elevation(self, target: float) -> None:
        self.request_elevation(target)

    @property
    def target_elevation(self) -> float:
        return self._target_elevation

    @property
    def angle(self) -> float:
        ang = self.angle_sensor.get()
        if ang > self.angle_range.max + 50: # This is just to make negative happen
            ang -= 450
        # ang = (ang - self.angle_range.min) % 360 + self.angle_range.min
        return ang

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

        self.angle_controller.setSetpoint(self._target_angle)
        speed = 0
        error = self.angle - self._target_angle
        if wpilib.DriverStation.isEnabled() and abs(error) > self.angle_tolerance:
            speed = self.angle_controller.calculate(self.angle)
            if abs(error) < 10:
                if speed < 0:
                    speed -= 0.08
                else:
                    speed += 0.08
                # speed += 0.1 * (speed / speed)

        self.angle_motor.set(speed)
        sd.putNumber("turret output", speed)
        sd.putNumber("turret target", self._target_angle)
        sd.putNumber("turret error", self.angle - self._target_angle)
        sd.putNumber("turret angle", self.angle)
        sd.putNumber("turret elevation", self.elevation)


        servo_target_angle = (pyerf.erfinv(self._target_elevation/11.08 - 1) + 0.54) / 0.0159
        # This convolution translates the target angle into servo-useful coordinates.
        # The servo is 110 degrees, and has a useful input range of 0.19 to 0.84, decreasing as it raises
        servo_target = 1 - (servo_target_angle / 110)
        servo_target = servo_target * 0.65 + 0.19
        self.elevation_servo.set(servo_target)

