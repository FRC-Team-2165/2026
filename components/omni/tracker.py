from wpimath.geometry import Pose3d, Rotation3d, Transform3d
from typing import Callable, Iterable, Any, Optional
import time
from dataclasses import dataclass

Timestamp = float
Duration = float
Index = Any
LocationData = tuple[Index, Pose3d]
PoseSource = Callable[[], tuple[Iterable[LocationData], Transform3d]]

@dataclass
class AgedPose:
    age: Timestamp
    pose: Pose3d

def average_pose(poses: Iterable[Pose3d]) -> Optional[Pose3d]:
    poses = list(poses)
    if len(poses) == 0:
        return None

    x = sum(map(lambda p: p.x, poses)) / len(poses)
    y = sum(map(lambda p: p.y, poses)) / len(poses)
    z = sum(map(lambda p: p.z, poses)) / len(poses)
    rot = Rotation3d()
    for pose in poses:
        rot += pose.rotation()
    rot /= len(poses)

    return Pose3d(x, y, z, rot)

class Tracker:
    pose_sources: list[PoseSource]
    canon: dict[Index, Pose3d]
    historical_data: dict[Index, AgedPose]
    data_lifespan: Duration
    suggested_position: Optional[AgedPose]

    def __init__(self, source: Iterable[PoseSource] | PoseSource,
                 canon: PoseSource | dict[Index, Pose3d],
                 data_lifespan: Duration = 0.03):
        try:
            self.pose_sources = list(source)
        except TypeError:
            self.pose_sources = [source]
        self.canon = canon if isinstance(canon, dict) else dict(canon()[0])
        self.historical_data = {}
        self.data_lifespan = data_lifespan
        self.suggested_position = None

    def add_source(self, source: PoseSource):
        self.pose_sources.append(source)

    def add_sources(self, source: Iterable[PoseSource]):
        self.pose_sources.extend(source)

    def recent_data(self, acceptable_age: Optional[Duration] = None) -> dict[Index, Pose3d]:
        output = {}
        if acceptable_age is None:
            acceptable_age = self.data_lifespan
        now = time.perf_counter()
        for (index, pose_data) in self.historical_data.items():
            age = pose_data.age
            pose = pose_data.pose
            if now - age < acceptable_age:
                output[index] = pose

        return output

    def get_by_index(self, index: Index) -> Optional[tuple[Pose3d, bool]]:
        now = time.perf_counter()
        if index not in self.historical_data:
            return None
        fresh = self.historical_data[index].age + self.data_lifespan > now
        return self.historical_data[index].pose, fresh

    def update(self):
        data: dict[Index, list[Pose3d]] = {}
        for source in self.pose_sources:
            fiducials, offset = source()
            for (index, pose) in fiducials:
                if index in data:
                    data[index].append(pose + offset)
                else:
                    data[index] = [pose + offset]

        for (index, locs) in data.items():
            avg = average_pose(locs)
            self.historical_data[index] = AgedPose(time.perf_counter(), avg)

    def suggest_position(self, position: Pose3d):
        self.suggested_position = AgedPose(time.perf_counter(), position)

    def __getitem__(self, item: Index) -> Optional[Pose3d]:
        item, fresh = self.get_by_index(item)
        return item if fresh else None

    def get_position(self) -> Optional[Pose3d]:
        now = time.perf_counter()
        valid_fiducials = filter(lambda d: d[1].age + self.data_lifespan > now and d[0] in self.canon, self.historical_data.items())
        positions = map(lambda d: (d[0], d[1].pose.relativeTo(self.canon[d[0]])), valid_fiducials)

        pose = average_pose(p.pose for (_, p) in positions)

        if pose is None:
            return self.suggested_position

        return pose
