import math
from wpimath.geometry import Pose3d, Transform2d
from components.omni import Tracker
from typing import Optional

from wpilib import SmartDashboard as sd

def rad2deg(ang: float) -> float:
    return ang * 180 / math.pi
def deg2rad(ang: float) -> float:
    return ang * (math.pi / 180)

# value that worked before 6.744
def get_targeting_angles(target_pose: Pose3d, v = 7.494) -> Optional[tuple[float, tuple[float, float]]]:

    angle = -math.atan2(target_pose.y, target_pose.x)
    angle = rad2deg(angle)

    v2 = v ** 2  # CHECK Initial ball speed. Ostensibly 30ft/s (9.144 m/s)?
    g = 9.81  # acceleration due to gravity in m/s
    x = math.sqrt(target_pose.x ** 2 + target_pose.y ** 2)  # lateral distance to target
    y = target_pose.z

    try:
        sqrt = math.sqrt(v2**2 - (g * x)**2 - (2 * g * v2 * y))
    except ValueError:
        return None

    theta1 = math.atan2((v2 + sqrt), (g * x))
    theta1 = 90 - rad2deg(theta1)

    theta2 = math.atan2((v2 - sqrt), (g * x))
    theta2 = 90 - rad2deg(theta2)

    return angle, (theta1, theta2)

def send_pose(name: str, pose: Pose3d):
    sd.putString(name, f"X: {pose.x}\nY:{pose.y}\nZ: {pose.z}\nθ: {pose.rotation().z_degrees}")

def send_planar_transform(name: str, trans: Transform2d):
    sd.putString(name, f"X: {trans.x}\nY:{trans.y}\nθ: {trans.rotation().degrees()}")
