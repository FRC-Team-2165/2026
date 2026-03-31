import commands2.cmd
import sys
import threading
import time
import math

import wpilib
from wpilib import TimedRobot
from commands2 import CommandScheduler, Command
from commands2.cmd import (
    InstantCommand,
    SequentialCommandGroup,
    WaitCommand,
    ParallelRaceGroup,
    ParallelCommandGroup,
    WaitUntilCommand,
    ConditionalCommand,
    StartEndCommand,
    RunCommand)
from commands2.button import CommandXboxController, Trigger
from wpilib import SmartDashboard as sd
import phoenix6


from wpimath import applyDeadband
from wpimath.geometry import Rotation3d

import commands.feeder_intake
import components.util as util
from components.omni import Tracker
from components.omni.adapter import PhotonAdapter
from wpimath.geometry import Pose3d, Pose2d, Rotation2d
from subsystems import *
from subsystems.shooter import TargetingStatus
from commands import *
from commands.shift_intake import IntakePosition
from commands.run_intake import IntakeActivity

import robotpy_apriltag as at

from typing import Optional


class Robot(TimedRobot):
    auto_error: bool
    auto_command: Optional[Command]

    controller: CommandXboxController
    other_controller: CommandXboxController

    drive: DriveSubsystem
    intake: IntakeSubsystem
    hopper: HopperSubsystem
    feeder: FeederSubsystem
    shooter: ShooterSubsystem

    tracker: Tracker

    def robotInit(self):
        self.controller = CommandXboxController(0)
        self.other_controller = CommandXboxController(1)
        self.drive = DriveSubsystem()
        self.intake = IntakeSubsystem()
        self.hopper = HopperSubsystem()
        self.feeder = FeederSubsystem()
        self.shooter = ShooterSubsystem()

        cameras = [
            PhotonAdapter("Shooter", Pose3d(0.34925, 0, -0.3302, Rotation3d())),
            # XFIXME I forgot to take the height measurements of these three
            # These can't be used because something in photonlibpy makes this take a very long time to run per device
            # PhotonAdapter("Hopper Left", Pose3d(0.3937, 0.27305, -0.508, Rotation3d(0, 0, math.pi/2))),
            # PhotonAdapter("Hopper Right", Pose3d(0.40005, -0.27305, -0.508, Rotation3d(0, 0, -math.pi/2))),
            # PhotonAdapter("Intake", Pose3d(-0.04445, 0, -0.508, Rotation3d(0, 0, 0.0243554 + math.pi)))
        ]
        field = at.AprilTagFieldLayout.loadField(at.AprilTagField.k2026RebuiltWelded)
        tag_canon = {tag.ID: tag.pose for tag in field.getTags()}

        self.tracker = Tracker(cameras, tag_canon)

        testing = Trigger(wpilib.DriverStation.isTest)
        enabled = Trigger(wpilib.DriverStation.isEnabled)
        intake_lowered = Trigger(self.intake.is_extended)
        holding_items = Trigger(self.hopper.contains_items)
        kicker_enabled = Trigger(self.feeder.kicker_enabled)


        controller_drive_command = ControllerDrive(self.drive, self.controller)
        self.drive.setDefaultCommand(controller_drive_command)

        self.controller.x().onTrue(ShiftIntake(self.intake, wait=False))
        ((intake_lowered | testing) & self.controller.leftTrigger() & ~self.controller.povLeft()).whileTrue(RunIntake(self.intake))
        ((intake_lowered | testing) & self.controller.leftTrigger() & self.controller.povLeft()).whileTrue(RunIntake(self.intake, IntakeActivity.Reverse))

        (self.controller.leftStick() & self.controller.rightStick()).onTrue(InstantCommand(self.drive.brace))
        (self.controller.back() & self.controller.rightStick()).onTrue(InstantCommand(self.drive.reset_angle).ignoringDisable(True))
        (self.controller.back() & self.controller.start()).onTrue(InstantCommand(controller_drive_command.toggle_field_relative))

        (self.controller.back() & self.controller.y()).onTrue(InstantCommand(self.feeder.toggle_kicker))
        (self.controller.back() & self.controller.b()).onTrue(InstantCommand(self.shooter.toggle_launcher))

        (self.other_controller.rightBumper() & ~self.other_controller.povLeft()).whileTrue(StartEndCommand(
            self.feeder.enable_intake,
            self.feeder.disable_intake
        ))

        (self.other_controller.rightBumper() & self.other_controller.povLeft()).whileTrue(StartEndCommand(
            self.feeder.reverse_intake,
            self.feeder.disable_intake
        ))



        (self.other_controller.rightTrigger()).whileTrue(SequentialCommandGroup(
            InstantCommand(self.feeder.enable_kicker),
            InstantCommand(self.shooter.enable_launcher),
            WaitCommand(0.5),
            InstantCommand(self.feeder.enable_intake)
        )).onFalse(SequentialCommandGroup(
            InstantCommand(self.feeder.disable_intake),
            WaitCommand(0.5),
            InstantCommand(self.shooter.disable_launcher),
            InstantCommand(self.feeder.disable_kicker)
        ))

        auto_target = AutoTarget(self.shooter, self.feeder, self.hopper, self.tracker)
        aim_backward = HoldAngle(self.drive, self.shooter, 180, 15)
        self.other_controller.a().toggleOnTrue(SequentialCommandGroup(
            InstantCommand(aim_backward.stop),
            auto_target
        ))
        self.other_controller.x().toggleOnTrue(SequentialCommandGroup(
            InstantCommand(auto_target.stop),
            aim_backward
        ))

        # This is likely a temporary solution.
        def update_shooter():
            requested_angle = self.shooter.target_angle + applyDeadband(self.other_controller.getRightX(), 0.2) * 3
            requested_angle = self.shooter.angle_range.clamp(requested_angle)
            if requested_angle == self.shooter.angle_range.max:
                requested_angle = requested_angle - 0.01
            self.shooter.request_angle(requested_angle)
            self.shooter.elevation += applyDeadband(self.other_controller.getRightY(), 0.2) * -0.25

        self.other_controller.leftBumper().whileTrue(RunCommand(update_shooter)).onFalse(InstantCommand(self.shooter.request_minimum_elevation))
        (self.other_controller.leftBumper() & self.other_controller.rightStick()).onTrue(InstantCommand(lambda: self.shooter.request_angle(0)))

        def on_start():
            # if testing():
            self.feeder.disable_kicker()
            self.shooter.disable_launcher()
            # else:
            #     pass
                # self.feeder.enable_kicker()
                # self.shooter.enable_launcher()

        enabled.onTrue(InstantCommand(on_start))

        def enable_shooting():
            self.shooter.request_elevation(17)
            self.shooter.enable_launcher()
            self.feeder.enable_kicker()

        def disable_shooting():
            self.shooter.request_minimum_elevation()
            self.shooter.disable_launcher()
            self.feeder.disable_kicker()

        self.auto_error = False
        self.auto_command = SequentialCommandGroup(
            InstantCommand(self.drive.reset_angle),
            InstantCommand(self.drive.reset_relative_location),
            ShiftIntake(self.intake, IntakePosition.Extend, wait=False),
            InstantCommand(enable_shooting),
            WaitCommand(0.5),
            InstantCommand(self.feeder.enable_intake),
            WaitHoldCommand(lambda: not self.hopper.contains_items(), 1),
            InstantCommand(self.feeder.disable_intake),
            InstantCommand(disable_shooting),
            InstantCommand(self.intake.enable),
            PoseTransition(self.drive, Pose2d(-2.886, 0, Rotation2d())),
            WaitCommand(1),
            ParallelCommandGroup(
                PoseTransition(self.drive, Pose2d()), # reset to original position
                InstantCommand(enable_shooting)
            ),
            InstantCommand(self.intake.disable),
            # No need to wait. Basically no way it won't be ready to shoot by the time it's in position
            InstantCommand(self.feeder.enable_intake),
            WaitHoldCommand(lambda: not self.hopper.contains_items(), 1),
            InstantCommand(self.feeder.disable_intake),
            InstantCommand(disable_shooting)
        )

        # Previous autonomous. Left in in case of emergency
        # self.auto_command = SequentialCommandGroup(
        #     # startup
        #     ShiftIntake(self.intake, IntakePosition.Extend, wait=False),
        #     ElevateShooter(self.shooter, 17, wait=False),
        #     InstantCommand(self.feeder.enable_kicker),
        #     InstantCommand(self.shooter.enable_launcher),
        #     WaitCommand(0.5),
        #     InstantCommand(self.feeder.enable_intake),
        #     # doing work automatically
        #     WaitCommand(10),
        #     # shutdown
        #     # ElevateShooter(self.shooter, 5, wait=False),
        #     InstantCommand(self.shooter.request_minimum_elevation),
        #     InstantCommand(self.feeder.disable_intake),
        #     InstantCommand(self.feeder.disable_kicker),
        #     InstantCommand(self.shooter.disable_launcher))

        phoenix6.SignalLogger.stop()
        phoenix6.SignalLogger.enable_auto_logging(False)
        # sd.putNumber("Target distance (input)", 0)

        def update_tracker():
            while True:
                self.tracker.update()
                time.sleep(0.04)
        self.tracker_worker = threading.Thread(target=update_tracker)
        self.tracker_worker.start()


    def robotPeriodic(self):
        phoenix6.SignalLogger.stop()
        CommandScheduler.getInstance().run()

        sd.putBoolean("Items in Hopper", self.hopper.contains_items())
        util.send_pose("Robot Pose", self.drive.get_position())


    def autonomousInit(self):
        try:
            if self.auto_command is not None:
                self.auto_command.schedule()
        except ValueError:
            self.auto_error = True
            print("Auto Command doesn't exist! You probably deleted it from robotInit")
        except Exception as e:
            print("Something went wrong during Autonomous!", file=sys.stderr)
            print(e, file=sys.stderr)
            self.auto_error = True

    def teleopInit(self):
        if self.auto_error:
            return
        try:
            if self.auto_command is not None and self.auto_command.isScheduled():
                self.auto_command.cancel()
        except ValueError:
            self.auto_error = True
            print("Auto Command doesn't exist! You probably deleted it from robotInit")

    def autonomousPeriodic(self):
        pass

    def teleopPeriodic(self):
        pass

