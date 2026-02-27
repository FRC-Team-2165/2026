from commands2 import Subsystem
from typing import Callable
from enum import Enum

from components.bar import BarSensor

from phoenix5 import WPI_TalonSRX

from wpilib import DutyCycleEncoder

# class IntakePosition(Enum):
#     Extend = 0,
#     Retract = 1,

class IntakePositionState(Enum):
    Extended = 0
    Extending = 1
    Retracted = 2
    Retracting = 3

class IntakeSubsystem(Subsystem):
    _pickup_callbacks: list[Callable[[int], None]]

    tilt_motor: WPI_TalonSRX
    angle_sensor: DutyCycleEncoder
    intake_motor: WPI_TalonSRX
    lowered_target: float
    raised_target: float

    ball_sensor: BarSensor

    def __init__(self):
        super().__init__()
        self._pickup_callbacks = []
        self._state = IntakePositionState.Retracted

        self.intake_motor = WPI_TalonSRX(13)
        self.tilt_motor = WPI_TalonSRX(14)
        # FIXME the range and zero point
        self.angle_sensor = DutyCycleEncoder(0)

        self.basic_intake_speed = 0.7
        self.basic_tilt_speed = 0.3

        # FIXME determine actual angles
        self.lowered_target = 0
        self.raised_target = 110

        self.ball_sensor = BarSensor([5,6,7,8])


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
        self.intake_motor.set(self.basic_intake_speed)

    def disable(self) -> None:
        self.intake_motor.stopMotor()

    def reverse(self) -> None:
        self.intake_motor.set(-self.basic_intake_speed)

    def is_enabled(self) -> bool:
        return self.intake_motor.get() != 0

    def sees_item(self) -> int:
        return self.ball_sensor.get()

    def add_pickup_callback(self, callback: Callable[[], None]) -> None:
        self._pickup_callbacks.append(callback)

    # noinspection PyTypeChecker
    on_pickup = property(lambda self: self.add_pickup_callback, add_pickup_callback, None, "Set a callback to be run when the system picks up a ball")

    def periodic(self) -> None:
        item_count = self.sees_item()
        if item_count > 0:
            for cb in self._pickup_callbacks:
                cb(item_count)

        if self._state == IntakePositionState.Extending:
            if abs(self.angle_sensor.get() - self.lowered_target) < 0.1:
                self._state = IntakePositionState.Extended
                self.tilt_motor.stopMotor()
            else:
                self.tilt_motor.set(self.basic_tilt_speed)
        elif self._state == IntakePositionState.Retracting:
            if abs(self.angle_sensor.get() - self.raised_target) < 0.1:
                self._state = IntakePositionState.Retracted
                self.tilt_motor.stopMotor()
            else:
                self.tilt_motor.set(-self.basic_tilt_speed)