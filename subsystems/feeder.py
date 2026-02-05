from commands2 import Subsystem

from typing import Callable

class FeederSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.control_rpm = 0 # FIXME
        self.target_rpm = 0

        self.passthrough_callbacks = []

        # TODO Add hardware

        # CHECK This may need a feedback controller, though that might be handled within the physical controller.

    def enable_intake(self) -> None:
        pass

    def disable_intake(self) -> None:
        pass

    def reverse_intake(self) -> None:
        pass

    def intake_enabled(self) -> bool:
        pass

    def enable_kicker(self) -> None:
        self.target_rpm = self.control_rpm

    def disable_kicker(self) -> None:
        self.target_rpm = 0

    def toggle_kicker(self) -> None:
        if self.target_rpm == 0:
            self.enable_kicker()
        else:
            self.disable_kicker()

    def kicker_enabled(self) -> bool:
        pass

    def has_item(self) -> bool:
        pass

    def add_passthrough_callback(self, callback: Callable[[], None]):
        self.passthrough_callbacks.append(callback)

    # noinspection PyTypeChecker
    on_passthrough = property(lambda self: self.add_passthrough_callback, add_passthrough_callback, None, "Set a callback when a ball passes through the system")

    def periodic(self) -> None:
        # TODO constant feedback setup on the accelerator system, enabled by default
        pass