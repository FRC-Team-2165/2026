from wpilib import AnalogInput

from typing import Generator, Iterable

class BarSensor:
    sensors: list[AnalogInput]
    active_range: range
    allowed_overlap: int

    def __init__(self, ports: Iterable[int], allowed_overlap: float = 0.5, deadband: float = 0.03):
        self.sensors = [AnalogInput(p) for p in ports]
        self.allowed_overlap = int(4096 * allowed_overlap)
        deadband = int(4096 * deadband)
        self.active_range = range(deadband, 4096 - deadband + 1)
        self.dummy = object()
        self.dummy.getValue = lambda: 0

    def pairs(self) -> Generator[tuple[AnalogInput, AnalogInput]]:
        for i in range(len(self.sensors) - 1):
            yield self.sensors[i], self.sensors[i+1]
        yield self.sensors[-1], self.dummy

    def get(self) -> int:
        count = 0
        ignore_next = False
        for l, r in self.pairs():
            if ignore_next:
                ignore_next = False
                continue
            l_value = l.getValue()
            r_value = r.getValue()
            if l_value > 4000 and r_value < 1000:
                if (r_value + 4096) - l_value < self.allowed_overlap:
                    ignore_next = True
            if l_value in self.active_range:
                count += 1

        return count



