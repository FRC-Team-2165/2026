from commands2 import Subsystem

from typing import Callable
import math

from phoenix5 import WPI_TalonSRX
from wpilib import DigitalInput

MINI_CIM_MAX_RPM = 6200
# assuming 2-inch diameter wheels
MINI_CIM_MAX_SPEED = MINI_CIM_MAX_RPM * 2 * math.pi / 720

class FeederSubsystem(Subsystem):

    intake_motor: WPI_TalonSRX
    kickers: list[WPI_TalonSRX]
    detector: DigitalInput

    def __init__(self):
        super().__init__()

        self.target_kicker_speed = 35 # ft/s
        self.kicker_target = self.target_kicker_speed / MINI_CIM_MAX_SPEED

        self.intake_speed = 0.35

        self.passthrough_callbacks = []

        kicker_front = WPI_TalonSRX(16)
        kicker_back = WPI_TalonSRX(17)
        kicker_back.setInverted(True)
        # Inversion assumed to be set in the controller
        self.kickers = [kicker_front, kicker_back]

        self.intake_motor = WPI_TalonSRX(18)

        # self.detector = DigitalInput(1)

        # No feedback controllers for this system (yet)

    def enable_intake(self) -> None:
        self.intake_motor.set(self.intake_speed)

    def disable_intake(self) -> None:
        self.intake_motor.set(0)

    def reverse_intake(self) -> None:
        self.intake_motor.set(-self.intake_speed)

    def intake_enabled(self) -> bool:
        return self.intake_motor.get() != 0

    def enable_kicker(self) -> None:
        for kicker in self.kickers:
            kicker.set(self.kicker_target)

    def disable_kicker(self) -> None:
        for kicker in self.kickers:
            kicker.set(0)

    def toggle_kicker(self) -> None:
        if not self.kicker_enabled():
            self.enable_kicker()
        else:
            self.disable_kicker()

    def kicker_enabled(self) -> bool:
        return any(kicker.get() != 0 for kicker in self.kickers)

    def sees_item(self) -> bool:
        # CHECK may need to invert
        # return self.detector.get()
        return False

    def add_passthrough_callback(self, callback: Callable[[], None]):
        self.passthrough_callbacks.append(callback)

    # noinspection PyTypeChecker
    on_passthrough = property(lambda self: self.add_passthrough_callback, add_passthrough_callback, None, "Set a callback when a ball passes through the system")

    def periodic(self) -> None:

        if self.sees_item():
            for callback in self.passthrough_callbacks:
                callback()