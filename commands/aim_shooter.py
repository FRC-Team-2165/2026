from commands2 import Command
from commands2.button import CommandXboxController
from subsystems import ShooterSubsystem
from subsystems.shooter import TargetingStatus

class ElevateShooter(Command):
    def __init__(self, subsystem: ShooterSubsystem, angle: float, wait: bool = True):
        super().__init__()
        self.target_angle = angle
        self.wait = wait
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.elevation = self.target_angle

    def isFinished(self) -> bool:
        return not self.wait or TargetingStatus.Set in self.subsystem.elevation_status()


class TurnShooter(Command):
    def __init__(self, subsystem: ShooterSubsystem, angle: float, wait: bool = True):
        super().__init__()
        self.target_angle = angle
        self.wait = wait
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.angle = self.target_angle

    def isFinished(self) -> bool:
        return not self.wait or TargetingStatus.Set in self.subsystem.angle_status()

class ManualAim(Command):
    def __init__(self, subsystem: ShooterSubsystem, controller: CommandXboxController):
        super().__init__()
        self.subsystem = subsystem
        self.controller = controller

        self.addRequirements(subsystem)

    def execute(self):
        x = self.controller.getRightX()
        y = -self.controller.getRightY()

        # FIXME These coefficients are not going to be good
        self.subsystem.angle = self.subsystem.angle + 0.1 * x
        self.subsystem.elevation = self.subsystem.elevation + 0.1 * y