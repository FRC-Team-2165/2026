from wpimath.geometry import Pose3d, Transform3d
import photonlibpy.photonCamera as pv
from ..tracker import LocationData
from typing import Iterable

import time

class PhotonAdapter:
    cam: pv.PhotonCamera
    offset: Transform3d

    def __init__(self, name: str, position: Pose3d):
        self.cam = pv.PhotonCamera(name)
        self.offset = Pose3d() - position

    def __call__(self) -> tuple[Iterable[LocationData], Transform3d]:
        start = time.perf_counter()
        res = self.cam.getAllUnreadResults()
        end = time.perf_counter()
        if len(res) == 0:
            return [], self.offset
        res = res[-1]
        if not res.hasTargets():
            return [], self.offset

        return map(lambda t: (t.getFiducialId(), Pose3d() + t.getBestCameraToTarget()), res.getTargets()), self.offset

