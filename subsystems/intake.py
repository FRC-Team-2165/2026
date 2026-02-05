from commands2 import Subsystem
from typing import Callable
from enum import Enum

# class IntakePosition(Enum):
#     Extend = 0,
#     Retract = 1,

class IntakePositionState(Enum):
    Extended = 0
    Extending = 1
    Retracted = 2
    Retracting = 3

class IntakeSubsystem(Subsystem):
    _pickup_callbacks: list[Callable[[], None]]
    def __init__(self):
        super().__init__()
        self._pickup_callbacks = []
        self._state = IntakePositionState.Retracted
        # TODO

    def extend(self) -> None:
        self._state = IntakePositionState.Extending

    def retract(self) -> None:
        self._state = IntakePositionState.Retracting

    def toggle(self) -> None:
        if self._state == IntakePositionState.Retracted or self._state == IntakePositionState.Retracting:
            self._state = IntakePositionState.Extending
        if self._state == IntakePositionState.Extended or self._state == IntakePositionState.Extending:
            self._state = IntakePositionState.Retracting

    def in_transit(self) -> bool:
        return self._state == IntakePositionState.Extending or self._state == IntakePositionState.Retracting

    def is_extended(self) -> bool:
        return self._state == IntakePositionState.Extended

    def is_retracted(self) -> bool:
        return self._state == IntakePositionState.Retracted

    def position(self) -> IntakePositionState:
        return self._state

    def enable(self) -> None:
        pass

    def disable(self) -> None:
        pass

    def reverse(self) -> None:
        pass

    def is_enabled(self) -> bool:
        pass

    def sees_item(self) -> int:
        pass

    def add_pickup_callback(self, callback: Callable[[], None]) -> None:
        self._pickup_callbacks.append(callback)

    # noinspection PyTypeChecker
    on_pickup = property(lambda self: self.add_pickup_callback, add_pickup_callback, None, "Set a callback to be run when the system picks up a ball")

    def periodic(self) -> None:
        if self.sees_item():
            for cb in self._pickup_callbacks:
                cb()

        if self._state == IntakePositionState.Extending:
            if True: # TODO move motor, determine completion
                self._state = IntakePositionState.Extended
            # Run the motors after the check
        elif self._state == IntakePositionState.Retracting:
            if True: # TODO move motor, determine completion
                self._state = IntakePositionState.Retracted