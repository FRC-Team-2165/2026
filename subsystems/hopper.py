from commands2 import Subsystem


class HopperSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.total_items = 0
        # TODO

    def enable_director(self) -> None:
        pass

    def disable_director(self) -> None:
        pass

    def toggle_director(self) -> None:
        if self.is_enabled():
            self.disable_director()
        else:
            self.enable_director()

    def reverse_director(self) -> None:
        pass

    def is_enabled(self) -> bool:
        pass

    def contains_items(self) -> bool:
        return self.total_items > 0

    def add_item(self) -> None:
        self.total_items += 1

    def remove_item(self) -> None:
        if self.total_items <= 0:
            self.total_items = 0
        else:
            self.total_items -= 1