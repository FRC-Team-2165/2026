import math
from wpimath.geometry import Translation2d

from wpilib import MotorSafety

from wpilib import SmartDashboard as sd


from phoenix6 import controls
import phoenix6.hardware as phoenix6
import rev

FalconMotor = phoenix6.TalonFX
CANCoder = phoenix6.CANcoder

from .vector import Polar

MAX_NEO_SPEED = 3.55 #mps

from dataclasses import dataclass

@dataclass
class SwerveModuleConfig:
    drive_motor_id: int
    "The CAN id of the drive motor"
    turn_motor_id: int
    "The CAN id of the turn motor"
    turn_encoder_id: int
    "The CAN id of the CANCoder"
    relative_position: Translation2d # offset from the center of the robot
    "The position of the module, relative to the center of the robot."
    inverted: bool = False # Should the drive motor be inverted
    "The inversion of the drive motor. Turn motor can't be inverted."
    gear_ratio: float = 1 # The gear ratio of the drive wheel, in falcon rotations per wheel rotation
    """
    The gear ratio of the drive wheel, in motor rotations per wheel rotation.

    If you only use the raw speed methods, and not the unit-based ones (names containing "mps"),
    this value doesn't need to be set.
    """
    max_drive_motor_speed: float = 1
    """The max speed of the motor, in RPM. 

    If you only use the raw speed methods, and not the unit-based ones (names containing "mps"),
    this value doesn't need to be set.
    """
    wheel_radius = 0.0508
    """Radius of the module's wheel, in meters. Defaults to the equivalent of a 2-inch radius.
    
    If you only use the raw speed methods, and not the unit-based ones (names containing "mps"),
    this value doesn't need to be set.
    """


    @property
    def wheel_circumference(self) -> float:
        """The circumference of the module's wheel, in meters.

        This value is based of wheel_radius, which defaults to a 2-inch radius wheel. If that is not set to the
        correct size, this value will be meaningless.
        """
        return math.tau * self.wheel_radius

    @property
    def max_drive_speed(self) -> float:
        """Calculates the maximum drive speed of the module's wheel, in meters per second.

        This calculation is based on gear_ratio, max_drive_motor_speed, and wheel_radius.
        If any one of those values is not set, the returned value will be meaningless.
        """
        #              m / rot          *         rot / min          / s/min = m/s
        return self.wheel_circumference * self.max_drive_motor_speed / self.gear_ratio / 60

class SwerveModule(MotorSafety):
    """
    Represents a swerve drive module based on Falcon 500 motors and the CTRE CANCoder magnetic encoder.
    Specifically designed for the MK4i swerve module kits from Swerve Drive Specialties.

    All angle control in these modules assumes that you have calibrated your CANCoders to be at -90 when
    the wheels are well-aligned to go forward. 

    Assumptions:
    - CANCoder
        - measures the angle range from [-180, 180)
        - has been offset so that the 0 position refers to when the wheel facing a quarter rotation clockwise
        - clockwise rotations increase angle, counterclockwise decrease
        - relative value set to absolute position on boot
    - Turning Motor
        - PID is controlled by a CANCoder using the "remote sensor" setup
        - calibrated to produce a clockwise motion given a positive value, and a counterclockwise given a negative
        - set to "Brake" for neutral input (this is optional for drive motor, but recommended).
    - PIDF values have been set in the motors themselves
    """
    config: SwerveModuleConfig
    drive_motor: rev.SparkMax
    drive_encoder: rev.SparkRelativeEncoder
    turn_motor: FalconMotor
    turn_encoder: CANCoder
    

    relative_position: Translation2d

    # gear_ratio: float

    def __init__(self, config: SwerveModuleConfig):
        """
        Creates the swerve module according the given configuration. See SwerveModuleConfig for 
        more information on configuration options.
        """
        super().__init__()

        self.config = config

        motor_config = rev.SparkBaseConfig()
        # QUESTION Are these values correct?
        motor_config.encoder.positionConversionFactor(config.wheel_circumference / config.gear_ratio )
        motor_config.encoder.velocityConversionFactor(config.wheel_circumference / config.gear_ratio / 60)
        motor_config.inverted(config.inverted)
        self.drive_motor = rev.SparkMax(config.drive_motor_id, rev.SparkMax.MotorType.kBrushless)
        self.drive_motor.configure(motor_config, rev.ResetMode.kNoResetSafeParameters, rev.PersistMode.kNoPersistParameters)
        self.drive_encoder = self.drive_motor.getEncoder()

        # self.drive_encoder.setVelocityConversionFactor(SWERVE_WHEEL_CIRCUMFERENCE / (config.gear_ratio * 60.0))
        # self.drive_encoder.setPositionConversionFactor(SWERVE_WHEEL_CIRCUMFERENCE / config.gear_ratio)

        self.turn_motor = FalconMotor(config.turn_motor_id)
        self.turn_encoder = CANCoder(config.turn_encoder_id)


        self.relative_position = config.relative_position
        self.turn_request = controls.PositionDutyCycle(0).with_slot(0)

        # self.gear_ratio = config.gear_ratio

        # self.reset_position()
        
    
    @property
    def inverted(self) -> bool:
        """
        Returns whether the drive motor is inverted. 
        """
        return self.drive_motor.getInverted()

    @inverted.setter
    def inverted(self, inverted: bool) -> None:
        """
        Sets whether the drive motor is inverted.

        This is initially set during configuration, and shouldn't need to be changed beyond that. 
        However, this is here if you have the use case.
        """
        self.drive_motor.setInverted(inverted) 

    def set_state(self, state: Polar) -> None:
        """
        Sets the state of the module, with speed being from [-1, 1]. Runs optimization to minimize heading change.
        """
        state = self._optimize(state)
        self.angle = state.theta
        self.speed = state.magnitude

    def set_state_mps(self, state: Polar) -> None:
        """
        Set the state of the module, with a speed from negative to positive MAX_NEO_SPEED. Runs optimization to minimize heading change.
        """

        normal = Polar(state.magnitude / self.config.max_drive_speed, state.theta)
        self.set_state(normal)

    def get_state(self) -> Polar:
        """
        Returns a representation of the current state of the module.
        """
        return Polar(self.speed, self.angle)

    def get_state_mps(self) -> Polar:
        """
        Returns a representation of the current state of the module with the speed in m/s.
        """
        return Polar(self.speed_mps, self.angle)

    @property
    def angle(self) -> float:
        """Returns the current module angle in degrees"""
        return self.turn_encoder.get_absolute_position().value_as_double * 360

    @angle.setter
    def angle(self, angle: float) -> None:
        """Sets the current angle in degrees. Values outside the range [0, 360) will be normalized."""
        sd.putNumber("Module angle", angle)
        angle = self._optimize_angle(angle)
        sd.putNumber("Module angle (opt)", angle)
        # cancoder_value = 4096 / 360.0 * angle
        # self.turn_motor.set_position(cancoder_value)
        self.turn_motor.set_control(self.turn_request.with_position(angle / 360))
        self.feed()
        # self.turn_motor.set(ctre.ControlMode.Position, cancoder_value)

    def _angle(self) -> float:
        return self.turn_encoder.get_position().value_as_double * 360

    @property
    def speed(self) -> float:
        """Returns a value in the range [-1, 1] representing the drive motor's approximate speed."""
        return self.drive_motor.get()
    
    @speed.setter
    def speed(self, speed: float) -> None:
        """Sets the speed of the drive motor in the range [-1, 1]. Values outside the range will be clamped."""
        if abs(speed) > 1:
            speed /= abs(speed) # cap, while retaining sign
        self.drive_motor.set(speed)
        self.turn_motor.feed()
        self.feed()

    @property
    def speed_mps(self) -> float:
        """Returns the drive motor's approximate speed, in meters per second."""
        return self.drive_encoder.getVelocity()

    @speed_mps.setter
    def speed_mps(self, speed: float) -> None:
        """Sets the drive motor's speed, in meters per second"""
        self.speed = speed / self.config.max_drive_speed

    

    def stopMotor(self) -> None:
        self.drive_motor.stopMotor()
        self.turn_motor.stopMotor()
        self.feed()


    def rotation_angle(self) -> float:
        """The base angle for this module when set to turn.

        This angle is perpendicular to the line defined by the translation from the center of the robot to the module.
        """
        base = self.relative_position.angle().degrees() + 90
        return base

    def offset_from_center(self) -> float:
        """The distance of the module's center of rotation to the center of the robot"""
        return self.relative_position.norm()


    def _optimize_angle(self, angle: float) -> float:
        sd.putNumber("Module angle (actual)", self._angle())
        offset = self._angle() // 360
        angle = angle % 360 + offset * 360 # set angle in the same degree as the current rotation
        target_angles = (angle + n * 180 for n in range(-2, 3)) # test every reasonable offset from current position
        return min(target_angles, key = lambda a: abs(self._angle() - a))
        

    def _optimize(self, state: Polar) -> Polar:
        a = self._optimize_angle(state.theta)
        speed = state.magnitude
        if abs(a % 360 - state.theta % 360) > 0.01: # not equal
            speed = -speed
        return Polar(speed, a)

