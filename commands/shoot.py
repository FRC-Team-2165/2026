from subsystems import ShooterSubsystem
from commands2 import Command

class Shoot(Command):
    def __init__(self, subsystem: ShooterSubsystem):
        super().__init__()
        self.subsystem = subsystem

        self.addRequirements(subsystem)

    def initialize(self):
        self.subsystem.enable_launcher()

    def end(self, interrupted: bool):
        self.subsystem.disable_launcher()