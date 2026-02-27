import phoenix6.hardware.pigeon2
import wpilib
from commands2 import Subsystem
from wpimath.geometry import Translation2d, Pose3d, Translation3d, Rotation3d, Rotation2d
from wpimath.filter import SlewRateLimiter
from copy import copy

from components.swerve.drive import SwerveDrive, SwerveModuleConfig
import navx

BASIC_MODULE = SwerveModuleConfig(
    drive_motor_id=100,
    turn_motor_id=101,
    turn_encoder_id=102,
    relative_position=Translation2d(),
    inverted=False,
    # All the values after this point should be constant
    gear_ratio=8.14,
    max_drive_motor_speed=5676,
    wheel_radius=0.0508,
)

def generate_config(drive_id: int, turn_motor_id: int, turn_encoder_id: int, loc: Translation2d, inverted: bool) -> SwerveModuleConfig:
    return SwerveModuleConfig(
        drive_motor_id=drive_id,
        turn_motor_id=turn_motor_id,
        turn_encoder_id=turn_encoder_id,
        relative_position=loc,
        inverted=inverted,
        # constant values
        gear_ratio=8.14,
        max_drive_motor_speed=5676,
        wheel_radius=0.0508,
    )

class DriveSubsystem(Subsystem):

    swerve: SwerveDrive
    position: Pose3d
    field_relative: bool

    def __init__(self):
        super().__init__()
        # TODO Fix configuration parameters
        front_left = generate_config(1,2,9, Translation2d(0, 0), False)
        front_right = generate_config(3,4,10, Translation2d(0, 0), False)
        rear_right = generate_config(5,6,11, Translation2d(0, 0), False)
        rear_left = generate_config(7,8,12, Translation2d(0, 0), False)

        self.swerve = SwerveDrive(front_left, front_right, rear_left, rear_right, deadband=0.1)
        # self.drive.set_x_deadband(0.1)
        # self.drive.set_y_deadband(0.1)
        # self.drive.set_rotation_deadband(0.2)

        # These are basically arbitrary values that seem to work nicely with our swerve system. This will likely not
        # work quite as well with anything other than the WCP Mk4i with a REV NEOv1.1
        self.xLimiter = SlewRateLimiter(1.8)
        self.yLimiter = SlewRateLimiter(1.8)
        self.rotationLimiter = SlewRateLimiter(1.8)

        # TODO Do something with the Pigeon as well, or perhaps instead of
        # self.gyro = phoenix6.hardware.pigeon2.Pigeon2(113)
        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
        self.yaw_offset = 0

        self.position = Pose3d()

    def drive(self, x_speed: float, y_speed: float, rotation: float, field_relative: bool = True) -> None:
        x_speed = self.xLimiter.calculate(x_speed)
        y_speed = self.yLimiter.calculate(y_speed)
        rotation = self.rotationLimiter.calculate(rotation)

        # print(f"x: {x_speed} y: {y_speed}")

        if field_relative:
            self.swerve.drive(-x_speed, -y_speed, rotation, self.get_angle())
        else:
            self.swerve.drive(-x_speed, -y_speed, rotation)

    def initialize(self) -> None:
        self.swerve.initialize()

    def brace(self) -> None:
        self.swerve.brace()

    def stop(self) -> None:
        self.swerve.stopMotor()

    def get_angle(self) -> float:
        return self.gyro.getAngle() - self.yaw_offset

    def set_angle_offset(self, angle: Rotation2d) -> None:
        self.yaw_offset = angle.degrees()

    def reset_angle(self) -> None:
        self.yaw_offset = self.gyro.getAngle()

    def get_position(self) -> Pose3d:
        # CHECK Make sure the axes of the translation and rotation match up with AprilTag output
        #   This will likely need to be affected by which alliance we're on
        translation = self.position.translation() + Translation3d(self.swerve.location())
        rot = Rotation3d(0, 0, self.gyro.getAngle())
        pose = Pose3d(translation, rot)
        return pose

    # def reset_position(self):
    #     self.swerve.reset_position()

    def set_position(self, pose: Pose3d) -> None:
        self.position = pose
        self.swerve.reset_position()
