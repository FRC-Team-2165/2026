from commands2 import Command
from subsystems import FeederSubsystem

class KickerState(Command):
    def __init__(self, subsystem: FeederSubsystem, enable: bool):
        super().__init__()
        self.subsystem = subsystem
        self.enable = enable
        self.addRequirements(subsystem)

    def initialize(self):
        if self.enable:
            self.subsystem.enable_kicker()
        else:
            self.subsystem.disable_kicker()

    def isFinished(self) -> bool:
        return True