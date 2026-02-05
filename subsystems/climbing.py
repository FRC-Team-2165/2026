from commands2 import Subsystem


class ClimbSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        # TODO

    def ascend(self) -> None:
        pass

    def descend(self) -> None:
        pass

    def has_ascended(self) -> bool:
        pass

    # the name "raise" is reserved
    def rise(self) -> None:
        pass

    def lower(self) -> None:
        pass

    def is_raised(self) -> bool:
        pass