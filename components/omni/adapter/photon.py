from wpimath.geometry import Pose3d, Transform3d
import photonlibpy.photonCamera as pv
from ..tracker import LocationData
from typing import Iterable

class PhotonAdapter:
    cam: pv.PhotonCamera
    offset: Transform3d

    def __init__(self, name: str, position: Pose3d):
        self.cam = pv.PhotonCamera(name)
        self.offset = position - Pose3d() # maybe reverse? It's 2AM, I don't know

    def __call__(self) -> tuple[Iterable[LocationData], Transform3d]:
        res = self.cam.getLatestResult()
        if not res.hasTargets():
            return [], self.offset

        return map(lambda t: (t.getFiducialId(), Pose3d() + t.getBestCameraToTarget()), res.getTargets()), self.offset

