from commands2 import Command
from subsystems import IntakeSubsystem
from typing import Callable

from enum import Enum

class IntakeActivity(Enum):
    Enable = 0
    Reverse = 3

class RunIntake(Command):
    action: IntakeActivity
    subsystem: IntakeSubsystem
    def __init__(self, subsystem: IntakeSubsystem, action: IntakeActivity = IntakeActivity.Enable, wait: bool = True):
        super().__init__()
        self.action = action
        self.subsystem = subsystem

        self.addRequirements(subsystem)
        self.start_enabled = False
        self.wait = wait

    def initialize(self):
        self.start_enabled = self.subsystem.is_enabled()
        match self.action:
            case IntakeActivity.Enable:
                self.subsystem.enable()
            case IntakeActivity.Reverse:
                self.subsystem.reverse()
            # case IntakeActivity.Disable:
            #     self.subsystem.disable()
            # case IntakeActivity.Toggle:
            #     if self.subsystem.is_enabled():
            #         self.subsystem.disable()
            #     else:
            #         self.subsystem.enable()

    def end(self, interrupted: bool):
        self.subsystem.disable()

    def isFinished(self) -> bool:
        if not self.wait:
            return True
        return self.subsystem.is_enabled()
        # match self.action:
        #     case IntakeActivity.Enable | IntakeActivity.Reverse:
        #         return self.subsystem.is_enabled()
        #     case IntakeActivity.Disable:
        #         return not self.subsystem.is_enabled()
        #     case IntakeActivity.Toggle:
        #         return self.start_enabled != self.subsystem.is_enabled()
