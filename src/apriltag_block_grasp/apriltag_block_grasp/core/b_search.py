"""Pure planning helpers for bounded absolute B-joint target search."""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple


@dataclass(frozen=True)
class BSearchConfig:
    offsets_deg: Tuple[float, ...]
    speed_deg_s: float
    acceleration: float
    minimum_absolute_deg: float
    maximum_absolute_deg: float
    maximum_single_delta_deg: float
    arrival_tolerance_deg: float
    arrival_stable_samples: int
    motion_timeout_s: float

    def __post_init__(self) -> None:
        values = (
            *self.offsets_deg,
            self.speed_deg_s,
            self.acceleration,
            self.minimum_absolute_deg,
            self.maximum_absolute_deg,
            self.maximum_single_delta_deg,
            self.arrival_tolerance_deg,
            self.motion_timeout_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("B-search parameters must be finite")
        if not self.offsets_deg or self.offsets_deg[0] != 0.0:
            raise ValueError("B-search offsets must start with 0")
        if len(set(self.offsets_deg)) != len(self.offsets_deg):
            raise ValueError("B-search offsets must not contain duplicates")
        if self.minimum_absolute_deg >= self.maximum_absolute_deg:
            raise ValueError("invalid absolute B range")
        if min(
            self.speed_deg_s,
            self.acceleration,
            self.maximum_single_delta_deg,
            self.arrival_tolerance_deg,
            self.motion_timeout_s,
        ) <= 0.0:
            raise ValueError("B-search positive parameters must be greater than zero")
        if self.arrival_stable_samples <= 0:
            raise ValueError("arrival_stable_samples must be positive")

    def absolute_targets(self, b0_deg: float) -> Tuple[float, ...]:
        if not math.isfinite(b0_deg):
            raise ValueError("B0 must be finite")
        targets = tuple(b0_deg + offset for offset in self.offsets_deg)
        if any(
            target < self.minimum_absolute_deg
            or target > self.maximum_absolute_deg
            for target in targets
        ):
            raise ValueError(
                "B0 plus search offsets exceeds the enabled absolute B range"
            )
        return targets

    def route_between(
        self, b0_deg: float, current_index: int, next_index: int
    ) -> List[float]:
        targets = self.absolute_targets(b0_deg)
        current = targets[current_index]
        target = targets[next_index]
        safe_direct_delta = self.maximum_single_delta_deg - self.arrival_tolerance_deg
        if abs(target - current) <= safe_direct_delta:
            return [target]
        route = self._route_via_b0(b0_deg, current, target)
        if any(
            abs(right - left) > self.maximum_single_delta_deg
            for left, right in zip((current, *route), route)
        ):
            raise ValueError("planned B-search route exceeds maximum single delta")
        return route

    def route_to_b0(self, b0_deg: float, current_index: int) -> List[float]:
        targets = self.absolute_targets(b0_deg)
        current = targets[current_index]
        return self._segment_toward(current, b0_deg)

    def _route_via_b0(
        self, b0_deg: float, current: float, target: float
    ) -> List[float]:
        route = self._segment_toward(current, b0_deg)
        route.extend(self._segment_toward(b0_deg, target))
        return route

    def _segment_toward(self, start: float, target: float) -> List[float]:
        distance = target - start
        safe_direct_delta = self.maximum_single_delta_deg - self.arrival_tolerance_deg
        if safe_direct_delta <= 0.0:
            raise ValueError("arrival tolerance leaves no safe B command delta")
        if abs(distance) <= safe_direct_delta:
            return [] if math.isclose(start, target, abs_tol=1e-9) else [target]
        step = math.copysign(self.maximum_single_delta_deg / 2.0, distance)
        route: List[float] = []
        current = start
        while abs(target - current) > safe_direct_delta:
            current += step
            route.append(current)
        if not math.isclose(current, target, abs_tol=1e-9):
            route.append(target)
        return route


def load_b_search_config(path: str) -> BSearchConfig:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    search = data["b_search"]
    absolute_range = search["driver_absolute_range_deg"]
    return BSearchConfig(
        offsets_deg=tuple(float(value) for value in search["offsets_deg"]),
        speed_deg_s=float(search["command_speed_deg_s"]),
        acceleration=float(search["command_acceleration"]),
        minimum_absolute_deg=float(absolute_range[0]),
        maximum_absolute_deg=float(absolute_range[1]),
        maximum_single_delta_deg=float(
            search["driver_maximum_single_delta_deg"]
        ),
        arrival_tolerance_deg=float(search["arrival_tolerance_deg"]),
        arrival_stable_samples=int(search["arrival_stable_samples"]),
        motion_timeout_s=float(search["motion_timeout_s"]),
    )
