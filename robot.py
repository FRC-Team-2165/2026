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
    StartEndCommand,
    RunCommand)
from commands2.button import CommandXboxController, Trigger
from wpilib import SmartDashboard as sd
import phoenix6

from wpimath import applyDeadband

import commands.feeder_intake
from components.omni import Tracker
from components.omni.adapter import PhotonAdapter
from wpimath.geometry import Pose3d
from subsystems import *
from subsystems.shooter import TargetingStatus
from commands import *
from commands.run_intake import IntakeActivity

import robotpy_apriltag as at

from typing import Optional


class Robot(TimedRobot):
    auto_error: bool
    auto_command: Optional[Command]

    controller: CommandXboxController

    drive: DriveSubsystem
    intake: IntakeSubsystem
    hopper: HopperSubsystem
    feeder: FeederSubsystem
    shooter: ShooterSubsystem

    def robotInit(self):
        self.controller = CommandXboxController(0)
        self.drive = DriveSubsystem()
        self.intake = IntakeSubsystem()
        self.hopper = HopperSubsystem()
        self.feeder = FeederSubsystem()
        self.shooter = ShooterSubsystem()


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

        (self.controller.back() & self.controller.y()).onTrue(InstantCommand(self.feeder.toggle_kicker))
        (self.controller.back() & self.controller.b()).onTrue(InstantCommand(self.shooter.toggle_launcher))

        (self.controller.back() & self.controller.start()).onTrue(InstantCommand(controller_drive_command.toggle_field_relative))

        (self.controller.rightBumper() & ~self.controller.povLeft()).whileTrue(StartEndCommand(
            self.feeder.enable_intake,
            self.feeder.disable_intake
        ))

        (self.controller.rightBumper() & self.controller.povLeft()).whileTrue(StartEndCommand(
            self.feeder.reverse_intake,
            self.feeder.disable_intake
        ))



        # (self.controller.rightTrigger() & (holding_items | testing)).whileTrue(StartEndCommand(self.feeder.enable_intake, self.feeder.disable_intake))
        (self.controller.rightTrigger() & ((holding_items & kicker_enabled) | testing)).whileTrue(SequentialCommandGroup(
            ElevateShooter(self.shooter, 17, wait=False),
            WaitCommand(0.25),
            InstantCommand(self.feeder.enable_intake)
        )).onFalse(SequentialCommandGroup(
            ElevateShooter(self.shooter, 5, wait=False),
            InstantCommand(self.feeder.disable_intake)
        ))

        # This is likely a temporary solution.
        def update_shooter():
            requested_angle = self.shooter.target_angle + applyDeadband(self.controller.getRightX(), 0.2) * 3
            requested_angle = self.shooter.angle_range.clamp(requested_angle)
            if requested_angle == self.shooter.angle_range.max:
                requested_angle = requested_angle - 0.01
            self.shooter.request_angle(requested_angle)
            # self.shooter.request_angle(self.shooter.target_angle + applyDeadband(self.controller.getRightX(), 0.2) * 1)
            # self.shooter.target_angle += applyDeadband(self.controller.getRightX(), 0.1) * 1
            self.shooter.elevation += applyDeadband(self.controller.getRightY(), 0.2) * -0.25

        (self.controller.leftBumper()).whileTrue(SequentialCommandGroup(
            InstantCommand(controller_drive_command.disable),
            RunCommand(update_shooter)
        )).onFalse(InstantCommand(controller_drive_command.enable))


        def on_start():
            if testing():
                self.feeder.disable_kicker()
                self.shooter.disable_launcher()
            else:
                self.feeder.enable_kicker()
                self.shooter.enable_launcher()

        enabled.onTrue(InstantCommand(on_start))

        self.auto_error = False
        self.auto_command = None
        phoenix6.SignalLogger.stop()

    def robotPeriodic(self):
        # if self.isEnabled():
        #     self.omniscience.suggest_position(self.drive.get_position())
        # self.omniscience.update()
        # self.drive.set_position(self.omniscience.get_position())
        CommandScheduler.getInstance().run()
        sd.putNumber("Intake Position (angle)", self.intake.angle())
        sd.putBoolean("Kicker", self.feeder.kicker_enabled())
        sd.putNumber("Kicker target", self.feeder.kicker_target)
        sd.putBoolean("Items in Hopper", self.hopper.contains_items())
        sd.putNumber("Gyro angle", self.drive.get_angle())


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
