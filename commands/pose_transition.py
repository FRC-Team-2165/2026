from commands2 import Command
from wpimath.geometry import Pose2d
import wpimath
from subsystems import DriveSubsystem

import components.util as util

def sign(value: float):
    return value / abs(value)

class PoseTransition(Command):
    drive: DriveSubsystem
    target_pose: Pose2d
    def __init__(self,
                 subsystem: DriveSubsystem,
                 target: Pose2d,
                 reset: bool = False,
                 field_relative: bool = True,
                 position_tolerance: float = 0.05,
                 angle_tolerance: float = 3):
        super().__init__()
        self.drive = subsystem
        self.target_pose = target
        self.reset = reset
        self.field_relative = field_relative
        self.position_tolerance = position_tolerance
        self.angle_tolerance = angle_tolerance

        self.at_target = False

        self.addRequirements(subsystem)

    def initialize(self):
        self.at_target = False
        if self.reset:
            self.drive.reset_relative_location()

    def execute(self):
        speed_x = 0
        speed_y = 0
        speed_rot = 0
        robot_pose = self.drive.get_relative_position()
        delta = self.target_pose - robot_pose

        util.send_planar_transform("Pose Transition", delta)

        self.at_target = True
        if abs(delta.x) > self.position_tolerance:
            # max out at error of 0.5 meters
            # cap of 0.6
            # feed forward of 0.1
            speed_x = sign(delta.x) * (0.4 * min(1.0, abs(delta.x) * 2) + 0.1)
            self.at_target = False
        if abs(delta.y) > self.position_tolerance:
            speed_y = sign(delta.y) * (0.4 * min(1.0, (abs(delta.y) * 2)) + 0.1)
            self.at_target = False
        rot = delta.rotation().degrees()
        if abs(rot) > self.angle_tolerance:
            # max output at error of 90 degrees
            # cap of 0.5
            # feed forward of 0.07
            speed_rot = sign(rot) * (0.1 * min(1.0, (abs(delta.rotation().degrees()) * (1/90))) + 0.07)
            self.at_target = False

        # Serve drive seems to use some version of NED instead of the NWU of the field
        self.drive.drive(speed_x, -speed_y, speed_rot, self.field_relative)

    def isFinished(self) -> bool:
        return self.at_target


