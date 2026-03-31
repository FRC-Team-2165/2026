from commands2 import Command
from subsystems import DriveSubsystem, ShooterSubsystem
from typing import Optional
from wpilib import SmartDashboard as sd

class HoldAngle(Command):
    drive: DriveSubsystem
    shooter: ShooterSubsystem
    target_angle: float
    def __init__(self, drive: DriveSubsystem, shooter: ShooterSubsystem, absolute_angle: float, tilt: Optional[float] = None):
        super().__init__()
        self.drive = drive
        self.shooter = shooter
        self.target_angle = absolute_angle
        self.target_tilt = tilt
        # Something like this could be used as a form of caching. We're not going to worry about that too much
        # self.previous_angle = 0
        self.finished = False

    def stop(self):
        self.finished = True

    def initialize(self):
        self.finished = False
        sd.putBoolean("Fixed Angle Enabled", True)
        sd.putNumber("Fixed Angle Target", self.target_angle)

    def execute(self):
        # Neither turret nor swerve system nor gyro use NWU (instead using **D), so they are compatible without adjustment
        robot_angle = self.drive.get_angle()
        turret_angle = self.target_angle - robot_angle
        self.shooter.request_angle(turret_angle)
        if self.target_tilt is not None:
            self.shooter.request_elevation(self.target_tilt)

    def end(self, interrupted: bool):
        self.shooter.request_minimum_elevation()
        sd.putBoolean("Fixed Angle Enabled", False)

    def isFinished(self) -> bool:
        return self.finished




