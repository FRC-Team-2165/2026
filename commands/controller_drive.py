from commands2 import Command
from commands2.button import CommandXboxController
from subsystems import DriveSubsystem
from wpimath import applyDeadband

from wpilib import DriverStation


class ControllerDrive(Command):
    subsystem: DriveSubsystem
    controller: CommandXboxController
    _field_relative: bool
    def __init__(self, subsystem: DriveSubsystem, controller: CommandXboxController):
        super().__init__()

        self.subsystem = subsystem
        self.controller = controller

        self._field_relative = True
        self.enabled = True
        self.addRequirements(subsystem)

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def initialize(self):
        pass

    def execute(self):
        if not self.enabled:
            self.subsystem.drive(0, 0, 0)
            return
        x = -self.controller.getLeftY()
        y = self.controller.getLeftX()
        rot = self.controller.getRightX()

        # Assumes that no alliance set should work like Blue Alliance.
        # if self._field_relative and DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        #     x = -x
        #     y = -y

        x = applyDeadband(x, 0.1)
        y = applyDeadband(y, 0.1)
        rot = applyDeadband(rot, 0.2)

        self.subsystem.drive(x, y, rot, field_relative=False, square_inputs=True)

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return False

    def toggle_field_relative(self) -> None:
        self._field_relative = not self._field_relative