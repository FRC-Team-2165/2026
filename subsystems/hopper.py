import wpilib
from commands2 import Subsystem

from phoenix5 import WPI_TalonSRX

class HopperSubsystem(Subsystem):
    ball_sensor: wpilib.DigitalInput
    def __init__(self):
        super().__init__()

        self.total_items = 0
        self.ball_sensor = wpilib.DigitalInput(1)
        self.director_speed = 0.5


    def contains_items(self) -> bool:
        return not self.ball_sensor.get()
        # return self.total_items > 0

    def add_item(self) -> None:
        self.total_items += 1

    def remove_item(self) -> None:
        if self.total_items <= 0:
            self.total_items = 0
        else:
            self.total_items -= 1