from commands2 import Command
from wpimath.geometry import Transform3d, Rotation3d, Translation3d
import wpilib
from wpilib import SmartDashboard as sd

from subsystems import ShooterSubsystem, FeederSubsystem, HopperSubsystem
from subsystems.shooter import TargetingStatus
from components.omni import Tracker
from components.util import get_targeting_angles
import math

class AutoTarget(Command):
    shooter: ShooterSubsystem
    feeder: FeederSubsystem
    hopper: HopperSubsystem
    tracker: Tracker
    target: int

    def __init__(self, shooter: ShooterSubsystem, feeder: FeederSubsystem, hopper: HopperSubsystem, tracker: Tracker):
        super().__init__()
        self.shooter = shooter
        self.feeder = feeder
        self.hopper = hopper
        self.tracker = tracker

        self.target = 0
        self.offset = Translation3d(0.6096, 0, 0.762)
        self.shooter_offset = Transform3d(self.shooter.shooter_mouth, Rotation3d()).inverse()
        print(self.shooter_offset)
        # self.shoot_delay = wpilib.Timer()
        # self.intake_delay_timer = wpilib.Timer()

        # self.addRequirements(shooter)
        self.finished = False

    def stop(self):
        self.finished = True

    def initialize(self):
        print("Enabling AutoTargeting")
        self.finished = False
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue:
            self.target = 26
        else:
            # self.target = 20  # use this one for dry testing
            self.target = 10
        # self.shoot_delay.restart()
        # self.shooter.enable_launcher()
        # self.feeder.enable_kicker()


    def execute(self):
        tag = self.tracker[self.target]
        if tag is None:
            return

        offset = Transform3d(self.offset, tag.rotation().rotateBy(Rotation3d(0, 0, math.pi)))
        target = tag + offset + self.shooter_offset
        angles = get_targeting_angles(target)
        if angles is None:
            sd.putBoolean("In range", False)
            return

        angle, (tilt1, tilt2) = angles

        sd.putNumber("AutoTarget angle", angle)
        sd.putNumberArray("AutoTarget tilt", [tilt1, tilt2])

        tilt = tilt1
        if not tilt in self.shooter.elevation_range:
            tilt = tilt2

        if not tilt in self.shooter.elevation_range:
            sd.putBoolean("In range", False)
            self.shooter.request_minimum_elevation()
            return
        else:
            sd.putBoolean("In range", True)


        self.shooter.request_angle(angle)
        self.shooter.request_elevation(tilt)
        #
        # if (self.shooter.angle_status().is_active() and self.hopper.contains_items()):
        #     self.feeder.enable_intake()
        #     self.intake_delay_timer.reset()
        # elif not self.intake_delay_timer.isRunning():
        #     self.intake_delay_timer.restart()

        # if self.intake_delay_timer.hasElapsed(0.5):
        #     self.feeder.disable_intake()
        #     self.intake_delay_timer.reset() # unnecessary, but I'm paranoid


    def end(self, interrupted: bool):
        print("Disabling AutoTargeting")
        # self.shoot_delay.stop()
        # self.intake_delay_timer.stop()
        # self.intake_delay_timer.reset()
        # self.shooter.disable_launcher()
        # self.feeder.disable_kicker()
        # self.feeder.disable_intake()
        self.shooter.request_minimum_elevation()

    def isFinished(self) -> bool:
        return self.finished