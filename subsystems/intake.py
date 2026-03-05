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
        self.angle_sensor = DutyCycleEncoder(0)
        self.lowered_target = 0.045
        self.raised_target = 0.40
        self.tilt_tolerance = 0.01

        if self.angle_sensor.get() > self.raised_target - self.tilt_tolerance:
            self._state = IntakePositionState.Retracted
        elif self.angle_sensor.get() < self.lowered_target + self.tilt_tolerance:
            self._state = IntakePositionState.Extended
        else:
            self._state = IntakePositionState.Extending

        self.intake_motor = WPI_TalonSRX(13)
        self.intake_motor.setInverted(True)
        self.tilt_motor = WPI_TalonSRX(14)
        self.tilt_motor.setInverted(True)

        self.basic_intake_speed = 0.8
        self.basic_tilt_speed = 0.6


        # self.ball_sensor = BarSensor([5,6,7,8])


    def extend(self) -> None:
        if self._state != IntakePositionState.Extended:
            self._state = IntakePositionState.Extending

    def retract(self) -> None:
        if self._state != IntakePositionState.Retracted:
            self._state = IntakePositionState.Retracting

    def toggle(self) -> None:
        print("toggling")
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

    def angle(self) -> float:
        return self.angle_sensor.get()

    def enable(self) -> None:
        self.intake_motor.set(self.basic_intake_speed)

    def disable(self) -> None:
        self.intake_motor.set(0)

    def reverse(self) -> None:
        self.intake_motor.set(-self.basic_intake_speed)

    def is_enabled(self) -> bool:
        return self.intake_motor.get() != 0

    def sees_item(self) -> int:
        # return self.ball_sensor.get()
        return 0

    def add_pickup_callback(self, callback: Callable[[int], None]) -> None:
        self._pickup_callbacks.append(callback)

    # noinspection PyTypeChecker
    on_pickup = property(lambda self: self.add_pickup_callback, add_pickup_callback, None, "Set a callback to be run when the system picks up a ball")

    def periodic(self) -> None:
        # This is functionally a no-op, but I'm leaving it just in case it isn't in the future
        item_count = self.sees_item()
        if item_count > 0:
            for cb in self._pickup_callbacks:
                cb(item_count)


        if self._state == IntakePositionState.Extending:
            if abs(self.angle_sensor.get() - self.lowered_target) < self.tilt_tolerance:
                self._state = IntakePositionState.Extended
                self.tilt_motor.set(0)
            else:
                self.tilt_motor.set(self.basic_tilt_speed)
        elif self._state == IntakePositionState.Retracting:
            if abs(self.angle_sensor.get() - self.raised_target) < self.tilt_tolerance:
                self._state = IntakePositionState.Retracted
                self.tilt_motor.set(0)
            else:
                self.tilt_motor.set(-(self.basic_tilt_speed + (self.raised_target - self.angle_sensor.get())))
        else:
            # probably not necessary, but it can't hurt
            self.tilt_motor.set(0)