import math

import wpilib
from commands2 import Command
from wpimath.geometry import Pose3d

from components.omni import Tracker
from subsystems import ShooterSubsystem


class SmartTrack(Command):
    def __init__(self, subsystem: ShooterSubsystem, tracker: Tracker):
        super().__init__()
        self.subsystem = subsystem
        self.tracker = tracker

        self.addRequirements(subsystem)
        self.enabled = True

        self.targets = {
            "blue_hub": Pose3d(),
            "blue_bump_left": Pose3d(),
            "blue_bump_right": Pose3d(),
            "red_hub": Pose3d(),
            "red_bump_left": Pose3d(),
            "red_bump_right": Pose3d(),
        }

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    def toggle(self) -> None:
        self.enabled = not self.enabled

    def is_enabled(self) -> bool:
        return self.enabled

    def initialize(self):
        pass

    def execute(self):
        if not self.enabled:
            return

        nearest = (10000000, "") # just a random huge number
        alliance = "red" if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed else "blue"
        me = self.tracker.get_position()
        if me is None:
            target = Pose3d() # If this happens, the robot will essentially target where it started up
        else:
            for name, pose in self.targets:
                if name.startswith(alliance):
                    dist = (me - pose).translation().norm()
                    if dist < nearest[0]:
                        nearest = (dist, name)

            target = self.targets[nearest[1]]


        transform = target - self.tracker.get_position()
        translation = transform.translation()

        # CHECK The transform's angle may need to messed with to make it negative
        turret_angle = transform.rotation().angle_degrees + self.subsystem.target_angle
        if self.subsystem.request_angle(turret_angle) != turret_angle:
            print(f"Turret angle out of range ({turret_angle}). Skipping elevation")
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


        # c = -9.81 / (2 * 9.144)
        # x = math.sqrt(translation.x ** 2 + translation.y ** 2) # lateral distance to target
        # y = translation.z
        #
        # part1 = math.sqrt((4 * c * y - 3) / (4 * (c * x)**2))
        # part2 = 1 / (2 * c * x)
        # theta = math.atan(part1 - part2)
        # self.subsystem.elevation = theta



    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return False
