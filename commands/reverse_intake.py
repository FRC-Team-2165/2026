from commands2 import Command
from subsystems import IntakeSubsystem

class ReverseIntake(Command):
    def __init__(self, subsystem: IntakeSubsystem, wait: bool = False):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(subsystem)
        self.wait = wait

    def initialize(self):
        self.subsystem.reverse()

    def end(self, interrupted: bool):
        self.subsystem.disable()

    def isFinished(self) -> bool:
        return not self.wait