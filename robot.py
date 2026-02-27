import commands2.cmd
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
    StartEndCommand)
from commands2.button import CommandXboxController, Trigger

import commands.feeder_intake
# from components.omni import Tracker
# from components.omni.adapter import PhotonAdapter
from wpimath.geometry import Pose3d
from subsystems import *
from subsystems.shooter import TargetingStatus
from commands import *
from commands.run_intake import IntakeActivity

import robotpy_apriltag as at

from typing import Optional


class Robot(TimedRobot):
    auto_error: bool
    auto_command: Command

    controller: CommandXboxController

    drive: DriveSubsystem

    def robotInit(self):
        self.drive = DriveSubsystem()

        self.controller = CommandXboxController(0)
        controller_drive_command = ControllerDrive(self.drive, self.controller)
        self.drive.setDefaultCommand(controller_drive_command)

        self.auto_error = False
        self.auto_command = None

    def robotPeriodic(self):
        # if self.isEnabled():
        #     self.omniscience.suggest_position(self.drive.get_position())
        # self.omniscience.update()
        # self.drive.set_position(self.omniscience.get_position())
        CommandScheduler.getInstance().run()

    def autonomousInit(self):
        try:
            if self.auto_command is not None:
                self.auto_command.schedule()
        except ValueError:
            self.auto_error = True
            print("Auto Command doesn't exist! You probably deleted it from robotInit")

    def autonomousPeriodic(self):
        if self.auto_error:
            return
        try:
            if self.auto_command is not None and self.auto_command.isScheduled():
                self.auto_command.cancel()
        except ValueError:
            self.auto_error = True
            print("Auto Command doesn't exist! You probably deleted it from robotInit")

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        pass

# class Robot(TimedRobot):
#     auto_command: Optional[Command]
#     auto_error: bool
#     omniscience: Tracker
#     controller: CommandXboxController
#
#     # Subsystems
#     drive_subsystem: DriveSubsystem
#     shooter: ShooterSubsystem
#     feeder: FeederSubsystem
#     intake: IntakeSubsystem
#     hopper: HopperSubsystem
#
#     def robotInit(self):
#         self.drive_subsystem = DriveSubsystem()
#         self.shooter = ShooterSubsystem()
#         self.feeder = FeederSubsystem()
#         self.intake = IntakeSubsystem()
#         self.hopper = HopperSubsystem()
#
#         self.controller = CommandXboxController(0)
#
#         cameras = [
#             PhotonAdapter("OV9721", Pose3d())
#         ]
#         field = at.AprilTagFieldLayout.loadField(at.AprilTagField.k2026RebuiltWelded)
#         tag_canon = {tag.ID: tag.pose for tag in field.getTags()}
#
#         # No, I'm not feeling self-important about this. Whatever gave you that idea?
#         self.omniscience = Tracker(cameras, tag_canon)
#
#         controller_drive_command = ControllerDrive(self.drive_subsystem, self.controller)
#         self.drive_subsystem.setDefaultCommand(controller_drive_command)
#
#         # self.tracking_command = SmartTrack(self.shooter, self.omniscience)
#         # self.shooter.setDefaultCommand(self.tracking_command)
#
#         self.disable_hopper = False
#         self.manual_aiming = False
#
#         manual_shoot = Trigger(lambda: self.manual_aiming)
#         holding_items = Trigger(self.hopper.contains_items)
#         # disabled_hopper = Trigger(lambda: self.disable_hopper)
#         launcher_enabled = Trigger(self.shooter.launcher_enabled)
#         kicker_enabled = Trigger(self.feeder.kicker_enabled)
#         testing = Trigger(wpilib.DriverStation.isTest)
#         def toggle_hopper_control():
#             self.disable_hopper = not self.disable_hopper
#         def toggle_manual_aiming():
#             self.manual_aiming = not self.manual_aiming
#
#         # the "back" button is intended to be the modifier to enable and disable default behaviours
#         # Field-relative drive
#         (self.controller.back() & self.controller.start()).onTrue(InstantCommand(controller_drive_command.toggle_field_relative))
#         # Smart tracking
#         (self.controller.back() & self.controller.rightStick()).onTrue(InstantCommand(toggle_manual_aiming))
#         # Feeder kicker
#         (self.controller.back() & self.controller.y()).onTrue(InstantCommand(self.feeder.toggle_kicker))
#         # Automatic hopper control (doesn't directly affect whether hopper is running)
#         (self.controller.back() & self.controller.x()).onTrue(InstantCommand(toggle_hopper_control))
#         # Shooter launcher
#         (self.controller.back() & self.controller.b()).onTrue(InstantCommand(self.shooter.toggle_launcher))
#
#         (manual_shoot & self.controller.rightBumper()).onTrue(SequentialCommandGroup(
#             ParallelRaceGroup(
#                 FeederIntake(self.feeder, commands.feeder_intake.ShiftDirection.Forward),
#                 WaitUntilCommand(self.feeder.sees_item),
#             ),
#             ParallelRaceGroup(
#                 Shoot(self.shooter),
#                 WaitCommand(1.0)
#             )
#         ))
#         (manual_shoot & self.controller.rightTrigger().whileTrue(ManualAim(self.shooter, self.controller)))
#         # holding_items.onTrue(StartEndCommand(self.hopper.enable_director, self.hopper.disable_director))
#
#         # if all relevant systems are enabled, and smart tracking is enabled
#         (~testing & ~manual_shoot & launcher_enabled & kicker_enabled).whileTrue(ParallelCommandGroup(
#             SmartTrack(self.shooter, self.omniscience),
#             ConditionalCommand(
#                 ParallelRaceGroup(
#                     # only enable the group while we have items in the hopper
#                     WaitUntilCommand(lambda: not holding_items.getAsBoolean()),
#                     # Turn on the intake while on-target
#                     StartEndCommand(self.feeder.enable_intake, self.feeder.disable_intake, self.feeder),
#                 ),
#                 # No-op
#                 InstantCommand(lambda: None),
#                 # If we're on target
#                 lambda: self.shooter.elevation_status() == TargetingStatus.Set and self.shooter.angle_status() == TargetingStatus.Set
#             )
#         ))
#
#         # Run intake in forward and reverse
#         (self.controller.leftTrigger() & ~self.controller.povLeft()).whileTrue(RunIntake(self.intake))
#         (self.controller.leftTrigger() & self.controller.povLeft()).whileTrue(RunIntake(self.intake, IntakeActivity.Reverse))
#         # Toggle intake state
#         self.controller.leftBumper().onTrue(InstantCommand(self.intake.toggle, self.intake))
#
#         # Hopper inventory management
#         self.intake.on_pickup = self.hopper.add_item
#         self.feeder.on_passthrough = self.hopper.remove_item
#
#         self.auto_command = None
#         self.auto_error = False
#
#         # kicker enabled by default outside of testing
#         if not testing.getAsBoolean():
#             self.feeder.enable_kicker()
#             # I can't see a good reason for this, but I also can't see a good reason not to
#             self.shooter.enable_launcher()
#
#     def robotPeriodic(self):
#         if self.isEnabled():
#             self.omniscience.suggest_position(self.drive_subsystem.get_position())
#         self.omniscience.update()
#         self.drive_subsystem.set_position(self.omniscience.get_position())
#         CommandScheduler.getInstance().run()
#
#     def autonomousInit(self):
#         try:
#             if self.auto_command is not None:
#                 self.auto_command.schedule()
#         except ValueError:
#             self.auto_error = True
#             print("Auto Command doesn't exist! You probably deleted it from robotInit")
#
#     def autonomousPeriodic(self):
#         if self.auto_error:
#             return
#         try:
#             if self.auto_command is not None and self.auto_command.isScheduled():
#                 self.auto_command.cancel()
#         except ValueError:
#             self.auto_error = True
#             print("Auto Command doesn't exist! You probably deleted it from robotInit")
#
#     def teleopInit(self):
#         pass
#
#     def teleopPeriodic(self):
#         pass
