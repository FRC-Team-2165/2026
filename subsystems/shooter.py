from commands2 import Subsystem
from enum import Flag
from dataclasses import dataclass

class TargetingStatus(Flag):
    Set = 0,
    Adjusting = 1,
    TooLow = 2,
    TooHigh = 3,

    def is_ok(self) -> bool:
        return TargetingStatus.TooHigh not in self and TargetingStatus.TooLow not in self

    def is_active(self) -> bool:
        return TargetingStatus.Adjusting not in self


@dataclass
class Range:
    min: float
    max: float

    def __contains__(self, item: float) -> bool:
        return self.min <= item <= self.max

    def clamp(self, value: float) -> float:
        return max(self.min, min(value, self.max))


class ShooterSubsystem(Subsystem):
    _target_elevation: float
    requested_elevation: float
    elevation_tolerance: float
    elevation_range: Range

    _target_angle: float
    requested_angle: float
    angle_tolerance: float
    angle_range: Range

    def __init__(self):
        super().__init__()

        self._target_elevation = 0.0
        self.requested_elevation = 0.0
        self.elevation_tolerance = 0.1
        self.elevation_range = Range(0.0, 360.0)

        self._target_angle = 0.0
        self.requested_angle = 0.0
        self.angle_tolerance = 0.1
        self.angle_range = Range(0.0, 360.0)
        # TODO add feedback controllers (probably PID, but we'll see)
        #      This may include wheel-speed controllers, though that may be doable inside the physical controller

        self.enabled = False
        # TODO Add actual hardware nonsense

    def enable_launcher(self) -> None:
        self.enabled = True

    def disable_launcher(self) -> None:
        self.enabled = False

    def toggle_launcher(self) -> None:
        self.enabled = not self.enabled

    def launcher_enabled(self) -> bool:
        return self.enabled

    def elevation_status(self) -> TargetingStatus:
        if abs(self.elevation - self.target_elevation) < self.elevation_tolerance:
            res = TargetingStatus.Set
        else:
            res = TargetingStatus.Adjusting

        if self.requested_elevation > self.elevation_range.max:
            res |= TargetingStatus.TooHigh
        elif self.requested_elevation < self.elevation_range.min:
            res |= TargetingStatus.TooLow

        return res

    def angle_status(self) -> TargetingStatus:
        if abs(self.angle - self.target_angle) < self.angle_tolerance:
            res = TargetingStatus.Set
        else:
            res = TargetingStatus.Adjusting

        if self.requested_angle > self.angle_range.max:
            res |= TargetingStatus.TooHigh
        elif self.requested_angle < self.angle_range.min:
            res |= TargetingStatus.TooLow

        return res

    def request_elevation(self, target: float) -> float:
        """requests the elevation change to the specific value. Returns the actual target elevation, which may be
        different from the requested value."""
        self.requested_elevation = target
        self._target_elevation = self.elevation_range.clamp(target)
        return self._target_elevation

    def request_angle(self, target: float) -> float:
        """requests the angle change to the specific value. Returns the actual target angle, which may be
        different from the requested value."""
        target %= 360
        self.requested_angle = target
        self._target_angle = self.angle_range.clamp(target)
        return self._target_angle

    @property
    def elevation(self) -> float:
        #TODO return shooter elevation detected by encoder
        return 0.0

    @elevation.setter
    def elevation(self, target: float) -> None:
        self.request_elevation(target)

    @property
    def target_elevation(self) -> float:
        return self._target_elevation

    @property
    def angle(self) -> float:
        # TODO return turret angle detected by encoder
        return 0.0

    @angle.setter
    def angle(self, target: float) -> None:
        self.request_angle(target)

    @property
    def target_angle(self) -> float:
        return self._target_angle

    def periodic(self) -> None:
        # This is not necessarily the best way to do this, but it's not incorrect.
        # It's functionally equivalent to simply running and stopping the motors in the respective enable and disable methods.
        if self.enabled:
            # enable or disable the shooter here
            pass

        # TODO tune system to target elevation and turret angle

