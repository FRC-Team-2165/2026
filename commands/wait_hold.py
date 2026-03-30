from commands2 import Command
from typing import Callable
from wpilib import Timer

class WaitHoldCommand(Command):
    def __init__(self, condition: Callable[[], bool], duration: float = 0):
        self.timer = Timer()
        self.duration = duration
        self.condition = condition

    def execute(self):
        if self.condition():
            self.timer.start()
        else:
            self.timer.stop()
            self.timer.reset()

    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.duration)