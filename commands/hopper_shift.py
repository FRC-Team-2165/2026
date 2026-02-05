from commands2 import Command
from subsystems import HopperSubsystem
from enum import Enum

class ShiftDirection(Enum):
    Forward = 0
    Reverse = 1

class HopperShift(Command):
    def __init__(self, subsystem: HopperSubsystem, direction: ShiftDirection, wait: bool = False):
        super().__init__()
        self.subsystem = subsystem
        self.direction = direction
        self.wait = wait

        self.addRequirements(subsystem)

    def initialize(self):
        if self.direction == ShiftDirection.Forward:
            self.subsystem.enable_director()
        else:
            self.subsystem.reverse_director()

    def end(self, interrupted: bool):
        self.subsystem.disable_director()

    def isFinished(self) -> bool:
        return not self.wait