from commands2 import Subsystem

from phoenix5 import WPI_TalonSRX

class HopperSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.total_items = 0
        self.director = WPI_TalonSRX(15)
        self.director_speed = 0.5

    def enable_director(self) -> None:
        self.director.set(self.director_speed)

    def disable_director(self) -> None:
        self.director.stopMotor()

    def toggle_director(self) -> None:
        if self.is_enabled():
            self.disable_director()
        else:
            self.enable_director()

    def reverse_director(self) -> None:
        self.director.set(-self.director_speed)

    def is_enabled(self) -> bool:
        return self.director.get() != 0

    def contains_items(self) -> bool:
        return self.total_items > 0

    def add_item(self) -> None:
        self.total_items += 1

    def remove_item(self) -> None:
        if self.total_items <= 0:
            self.total_items = 0
        else:
            self.total_items -= 1