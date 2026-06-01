from dataclasses import dataclass
from typing import FrozenSet


@dataclass(frozen=True)
class State:
    current_node: str
    battery_level: int
    collected_points: FrozenSet[str]

    def collections_text(self) -> str:
        if not self.collected_points:
            return "{}"

        ordered = sorted(self.collected_points)
        return "{" + ",".join(ordered) + "}"

    def __str__(self) -> str:
        return (
            f"({self.current_node}, "
            f"{self.battery_level}%, "
            f"{self.collections_text()})"
        )