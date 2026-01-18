from wpilib import TimedRobot
from commands2 import CommandScheduler, Command

from typing import Optional

class Robot(TimedRobot):
    auto_command: Optional[Command]
    def robotInit(self):


        self.auto_command = None

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

    def autonomousInit(self):
        try:
            if self.auto_command is not None:
                self.auto_command.schedule()
        except ValueError:
            print("Auto Command doesn't exist! You probably deleted it from robotInit")

    def autonomousPeriodic(self):
        try:
            if self.auto_command is not None and self.auto_command.isScheduled():
                self.auto_command.cancel()
        except ValueError:
            print("Auto Command doesn't exist! You probably deleted it from robotInit")

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        pass
