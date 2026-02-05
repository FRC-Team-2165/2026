from commands2 import Command
from subsystems import IntakeSubsystem
from subsystems.intake import IntakePositionState
from enum import Enum

class IntakePosition(Enum):
    Extend = 0
    Retract = 1
    Toggle = 2

class ShiftIntake(Command):
    subsystem: IntakeSubsystem
    instruction = IntakePosition
    def __init__(self, subsystem: IntakeSubsystem, position: IntakePosition = IntakePosition.Toggle, wait: bool = True):
        super().__init__()
        self.subsystem = subsystem
        self.instruction = position

        self.addRequirements(subsystem)
        self.start_position = None

        self.wait = wait

    def initialize(self):
        self.start_position = self.subsystem.position()
        match self.instruction:
            case IntakePosition.Extend:
                self.subsystem.extend()
            case IntakePosition.Retract:
                self.subsystem.retract()
            case IntakePosition.Toggle:
                self.subsystem.toggle()

    def isFinished(self) -> bool:
        if not self.wait:
            return True
        match self.instruction:
            case IntakePosition.Extend:
                return self.subsystem.is_extended()
            case IntakePosition.Retract:
                return self.subsystem.is_retracted()
            case IntakePosition.Toggle:
                 if self.start_position == IntakePositionState.Retracting or self.start_position == IntakePositionState.Retracted:
                     return self.subsystem.is_extended()
                 else:
                     return self.subsystem.is_retracted()
