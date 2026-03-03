from commands2 import Command
from subsystems import FeederSubsystem
from enum import Enum

class ShiftDirection(Enum):
    Forward = 0
    Reverse = 1

class FeederIntake(Command):
    def __init__(self, subsystem: FeederSubsystem, direction: ShiftDirection = ShiftDirection.Forward, wait: bool = False):
        super().__init__()
        self.subsystem = subsystem
        self.direction = direction

        self.addRequirements(subsystem)

        self.wait = wait

    def initialize(self):
        if self.direction == ShiftDirection.Forward:
            self.subsystem.enable_intake()
        else:
            self.subsystem.reverse_intake()

    def end(self, interrupted: bool):
        self.subsystem.disable_intake()

    def isFinished(self) -> bool:
        return not self.wait