import phoenix6.hardware.pigeon2
import wpilib
from commands2 import Subsystem
from wpimath.geometry import Translation2d, Pose3d, Translation3d, Rotation3d, Rotation2d
from wpimath.filter import SlewRateLimiter

from components.swerve.drive import SwerveDrive, SwerveModuleConfig
import navx

class DriveSubsystem(Subsystem):

    swerve: SwerveDrive
    position: Pose3d
    field_relative: bool

    def __init__(self):
        super().__init__()
        long_offset = 0.32940625
        short_offset = 0.24050625
        front_left = SwerveModuleConfig(4, 3, 10, Translation2d(long_offset, short_offset), False, 8.14, max_drive_motor_speed=5676)
        front_right = SwerveModuleConfig(6, 5, 11, Translation2d(long_offset, -short_offset), False, 8.14, max_drive_motor_speed=5676)
        rear_left = SwerveModuleConfig(2, 1, 9, Translation2d(-long_offset, short_offset), False, 8.14, max_drive_motor_speed=5676)
        rear_right = SwerveModuleConfig(8, 7, 12, Translation2d(-long_offset, -short_offset), False, 8.14, max_drive_motor_speed=5676)
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
        return (self.gyro.getAngle() - self.yaw_offset) % 360

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
