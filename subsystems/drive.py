import phoenix6.hardware.pigeon2
import wpilib
from commands2 import Subsystem
from wpimath.geometry import Translation2d, Pose3d, Pose2d, Translation3d, Rotation3d, Rotation2d, Transform3d
from wpimath.filter import SlewRateLimiter

from components.swerve.drive import SwerveDrive, SwerveModuleConfig
import components.util as util
import navx

class DriveSubsystem(Subsystem):

    swerve: SwerveDrive
    location: Translation3d
    field_relative: bool

    def __init__(self):
        super().__init__()
        long_offset = 0.32940625
        short_offset = 0.24050625
        front_left = SwerveModuleConfig(4, 3, 10, Translation2d(long_offset, short_offset), False, 8.14, max_drive_motor_speed=5676)
        front_right = SwerveModuleConfig(6, 5, 11, Translation2d(long_offset, -short_offset), False, 8.14, max_drive_motor_speed=5676)
        rear_left = SwerveModuleConfig(2, 1, 9, Translation2d(-long_offset, short_offset), False, 8.14, max_drive_motor_speed=5676)
        rear_right = SwerveModuleConfig(8, 7, 12, Translation2d(-long_offset, -short_offset), False, 8.14, max_drive_motor_speed=5676)
        self.swerve = SwerveDrive(front_left, front_right, rear_left, rear_right, deadband=0)

        # self.drive.set_x_deadband(0.1)
        # self.drive.set_y_deadband(0.1)
        # self.drive.set_rotation_deadband(0.2)

        # These are basically arbitrary values that seem to work nicely with our swerve system. This will likely not
        # work quite as well with anything other than the WCP Mk4i with a REV NEOv1.1
        self.xLimiter = SlewRateLimiter(2)
        self.yLimiter = SlewRateLimiter(2)
        self.rotationLimiter = SlewRateLimiter(2)

        # TODO Do something with the Pigeon as well, or perhaps instead of
        # self.gyro = phoenix6.hardware.pigeon2.Pigeon2(113)
        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
        self.yaw_offset = 0

        self.location = Translation3d()

    def drive(self, x_speed: float, y_speed: float, rotation: float, field_relative: bool = True, square_inputs: bool = False) -> None:
        x_speed = self.xLimiter.calculate(x_speed)
        y_speed = self.yLimiter.calculate(y_speed)
        rotation = self.rotationLimiter.calculate(rotation)

        if field_relative:
            self.swerve.drive(-x_speed, -y_speed, rotation, -self.get_angle(), square_inputs=square_inputs)
        else:
            self.swerve.drive(-x_speed, -y_speed, rotation, square_inputs=square_inputs)

    def initialize(self) -> None:
        self.swerve.initialize()

    def brace(self) -> None:
        self.swerve.brace()

    def stop(self) -> None:
        self.swerve.stopMotor()

    def get_angle(self) -> float:
        return self.gyro.getAngle() - self.yaw_offset

    def get_field_angle(self) -> float:
        """
        Gets the robot's rotation relative to the field. This relies on two assumptions.
        - Blue alliance is the zero point
        - the robot at some known starting angle relative to "forward" from its alliance's perspective.
        """
        angle = self.get_angle()
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            angle = (angle + 180) % 360

        return angle


    def set_angle_offset(self, angle: Rotation2d) -> None:
        self.yaw_offset = angle.degrees()

    def reset_angle(self) -> None:
        self.yaw_offset = self.gyro.getAngle()

    def get_relative_position(self) -> Pose2d:
        location = self.swerve.location()
        # self_loc = Translation2d(self.location.x, self.location.y)
        # adjusted_location = location + self_loc
        return Pose2d(location, Rotation2d(util.deg2rad(self.get_angle())))

    def reset_relative_location(self):
        self.swerve.reset_position()

    def get_position(self) -> Pose3d:
        # CHECK Make sure the axes of the translation and rotation match up with AprilTag output
        #   This will likely need to be affected by which alliance we're on

        location = Translation3d(self.swerve.location())

        adjusted_location = self.location + location
        rot = Rotation3d(0, 0, util.deg2rad(self.get_field_angle()))
        return Pose3d(adjusted_location, rot)


    def set_location(self, loc: Translation3d) -> None:
        self.location = loc
        self.swerve.reset_position()
