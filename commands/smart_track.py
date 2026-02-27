import math

import wpilib
from commands2 import Command
from wpimath.geometry import Pose3d, Translation2d

from components.omni import Tracker
from subsystems import ShooterSubsystem

from dataclasses import dataclass, field
from typing import Callable, Optional

def in_range(test: float, rang: tuple[float, float]) -> bool:
    if rang[0] < rang[1]:
        return rang[0] < test < rang[1]
    else:
        return rang[1] < test < rang[0]


class Context:
    condition: Optional[Callable[[], bool]]
    actions: list[Callable[[], None]]
    finishers: list[Callable[[], None]]

    def __init__(self):
        self.condition = None
        self.actions = []
        self.finishers = []

    def add_action(self, action: Callable[[], None]):
        self.actions.append(action)

    def add_finish(self, action: Callable[[], None]):
        self.finishers.append(action)

    def add_condition(self, cond: Callable[[], bool]):
        if self.condition is not None:
            self.condition = lambda: cond() and self.condition()
        else:
            self.condition = cond

    def is_relevant(self) -> bool:
        return self.condition is None or self.condition()

    def activate(self):
        self._active = True
        for act in self.actions:
            act()

    def finalize(self):
        if self.is_active:
            self._active = False
            for act in self.finishers:
                act()
    @property
    def is_active(self):
        return hasattr(self, "_active") and self._active

def location_condition(corners: tuple[Translation2d, Translation2d], source: Callable[[], Translation2d]) -> Callable[[], None]:
    def box_contains():
        point = source()
        xs = (corners[0].x, corners[1].x)
        ys = (corners[0].y, corners[1].y)
        return in_range(point.x, xs) and in_range(point.y, ys)

    return box_contains


# class LocationContext(Context):
#     def __init__(self, corners: tuple[Translation2d, Translation2d], source: Callable[[], Translation2d]):
#         super().__init__()
#         def box_contains():
#             point = source()
#             xs = (corners[0].x, corners[1].x)
#             ys = (corners[0].y, corners[1].y)
#             return in_range(point.x, xs) and in_range(point.y, ys)
#
#         self.add_condition(box_contains)


class ContextManager:
    contexts: dict[str, Context]

    def add_context(self, name: str, context: Context):
        self.contexts[name] = context

    def get_context(self, name: str) -> Optional[Context]:
        return self.contexts.get(name)

    def execute(self):
        for _name, ctx in self.contexts.items():
            if ctx.is_relevant():
                ctx.activate()
            elif ctx.is_active:
                ctx.finalize()

    def finalize(self):
        for _name, ctx in self.contexts.items():
            if ctx.is_active:
                ctx.finalize()


class SmartTrack(Command):
    def __init__(self, subsystem: ShooterSubsystem, tracker: Tracker):
        super().__init__()
        self.subsystem = subsystem
        self.tracker = tracker

        self.addRequirements(subsystem)
        self.enabled = True

        self.contexts = ContextManager()
        self.target = None
        self.target_changed = False

        self.turret_locked = False
        self.elevation_locked = False


    def add_context(self, name: str, ctx: Context):
        self.contexts.add_context(name, ctx)

    def get_context(self, name: str) -> Optional[Context]:
        return self.contexts.get_context(name)

    @property
    def target(self) -> Optional[Pose3d]:
        return self._target

    @target.setter
    def target(self, pose: Optional[Pose3d]):
        self._target = pose
        self.target_changed = True

    def lock_turret(self):
        self.turret_locked = True

    def release_turret(self):
        self.turret_locked = False

    def turret_lock_context(self) -> Context:
        ctx = Context()
        ctx.add_action(self.lock_turret)
        ctx.add_finish(self.release_turret)
        return ctx

    def lock_elevation(self):
        self.elevation_locked = True

    def release_elevation(self):
        self.elevation_locked = False

    def elevation_lock_context(self) -> Context:
        ctx = Context()
        ctx.add_action(self.lock_elevation)
        ctx.add_finish(self.release_elevation)
        return ctx

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    def toggle(self) -> None:
        self.enabled = not self.enabled

    def is_enabled(self) -> bool:
        return self.enabled


    # Command functions

    def initialize(self):
        pass

    def execute(self):
        if not self.enabled:
            return

        self.contexts.execute()

        # nearest = (10000000, "") # just a random huge number
        # alliance = "red" if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed else "blue"
        # me = self.tracker.get_position()
        # if me is None:
        #     target = Pose3d() # If this happens, the robot will essentially target where it started up
        # else:
        #     for name, pose in self.targets:
        #         if name.startswith(alliance):
        #             dist = (me - pose).translation().norm()
        #             if dist < nearest[0]:
        #                 nearest = (dist, name)
        #
        #     target = self.targets[nearest[1]]

        if self.target is None:
            return

        transform = self.target - self.tracker.get_position()
        translation = transform.translation()

        # CHECK The transform's angle may need to messed with to make it negative
        turret_angle = transform.rotation().angle_degrees + self.subsystem.target_angle
        if not self.turret_locked:
            if self.subsystem.request_angle(turret_angle) != turret_angle:
                print(f"Turret angle out of range ({turret_angle}). Skipping elevation")
                return

        if self.elevation_locked:
            return

        v2 = 9.144 ** 2 # CHECK Initial ball speed. Ostensibly 30ft/s (9.144 m/s)?
        g = 9.81 # acceleration due to gravity in m/s
        x = math.sqrt(translation.x ** 2 + translation.y ** 2) # lateral distance to target
        y = translation.z

        sqrt = math.sqrt(v2**2 - (g*x)**2 - (2 * g * v2 * y))

        theta = math.atan2(g * x, v2 + sqrt)

        # This is safe because the functions involved return the same value with no processing if it is valid
        if self.subsystem.request_elevation(theta) != theta:
            theta = math.atan2(g*x, v2 - sqrt)
            if self.subsystem.request_elevation(theta) != theta:
                print(f"Elevation angle out of range of system ({theta})")


    def end(self, interrupted: bool):
        self.contexts.finalize()

    def isFinished(self) -> bool:
        return False
