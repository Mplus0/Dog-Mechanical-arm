"""Pure target selection and consecutive XYZ stability logic."""

import math
import statistics
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple


@dataclass(frozen=True)
class TargetStabilityConfig:
    selection_order: Tuple[int, ...]
    stable_frame_count: int
    xyz_peak_to_peak_threshold_mm: Tuple[float, float, float]
    stable_timeout_s: float
    max_pnp_arm_stamp_delta_s: float
    max_arm_state_reported_age_s: float

    def __post_init__(self) -> None:
        if not self.selection_order or any(value not in (0, 1) for value in self.selection_order):
            raise ValueError("selection_order must contain only ID 0 and/or ID 1")
        if len(set(self.selection_order)) != len(self.selection_order):
            raise ValueError("selection_order must not contain duplicate IDs")
        if self.stable_frame_count <= 0:
            raise ValueError("stable_frame_count must be positive")
        if len(self.xyz_peak_to_peak_threshold_mm) != 3 or any(
            not math.isfinite(value) or value <= 0.0
            for value in self.xyz_peak_to_peak_threshold_mm
        ):
            raise ValueError("XYZ peak-to-peak thresholds must be three positive values")
        for name, value in (
            ("stable_timeout_s", self.stable_timeout_s),
            ("max_pnp_arm_stamp_delta_s", self.max_pnp_arm_stamp_delta_s),
            ("max_arm_state_reported_age_s", self.max_arm_state_reported_age_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")


class StableTargetLock:
    """Lock one visible ID and build an immutable median XYZ snapshot.

    A missing/invalid frame clears the current consecutive sample window.  It
    does not change the locked ID.  A terminal result remains latched until the
    owner explicitly calls :meth:`reset`, which will later be used when a new B
    search angle begins.
    """

    def __init__(self, config: TargetStabilityConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.locked_id: Optional[int] = None
        self.attempt_started_s: Optional[float] = None
        self.samples: List[Tuple[float, float, float]] = []
        self.sample_stamps: List[Optional[float]] = []
        self.window_reset_count = 0
        self.ever_saw_allowed_target = False
        self.last_xyz_peak_to_peak_mm: Optional[Tuple[float, float, float]] = None
        self.best_xyz_peak_to_peak_mm: Optional[Tuple[float, float, float]] = None
        self.best_max_threshold_ratio: Optional[float] = None
        self.terminal_result: Optional[Dict[str, Any]] = None

    @staticmethod
    def _finite_float(value: Any) -> Optional[float]:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        return parsed if math.isfinite(parsed) else None

    def _message_gate(self, payload: Dict[str, Any]) -> Tuple[bool, str]:
        if not bool(payload.get("valid", False)):
            return False, "candidate_message_invalid"

        delta = self._finite_float(payload.get("pnp_arm_stamp_delta_s"))
        if delta is None:
            return False, "pnp_arm_stamp_delta_missing"
        if abs(delta) > self.config.max_pnp_arm_stamp_delta_s:
            return False, "pnp_arm_stamp_delta_exceeded"

        age = self._finite_float(payload.get("arm_state_reported_age_s"))
        if age is None:
            return False, "arm_state_age_missing"
        if age < 0.0 or age > self.config.max_arm_state_reported_age_s:
            return False, "arm_state_stale"
        return True, "ok"

    def _candidate_map(self, payload: Dict[str, Any]) -> Dict[int, Dict[str, Any]]:
        result: Dict[int, Dict[str, Any]] = {}
        candidates = payload.get("candidates", [])
        if not isinstance(candidates, list):
            return result
        for candidate in candidates:
            if not isinstance(candidate, dict):
                continue
            try:
                tag_id = int(candidate.get("tag_id"))
            except (TypeError, ValueError):
                continue
            if tag_id in self.config.selection_order and tag_id not in result:
                result[tag_id] = candidate
        return result

    def _candidate_xyz(self, candidate: Dict[str, Any]) -> Optional[Tuple[float, float, float]]:
        position = candidate.get("base_object_mm")
        if not isinstance(position, dict):
            return None
        values = tuple(self._finite_float(position.get(axis)) for axis in ("x", "y", "z"))
        if any(value is None for value in values):
            return None
        return values  # type: ignore[return-value]

    @staticmethod
    def _axis_statistics(
        samples: Iterable[Tuple[float, float, float]],
    ) -> Tuple[List[float], List[float]]:
        materialized = list(samples)
        medians = [statistics.median(sample[axis] for sample in materialized) for axis in range(3)]
        spans = [
            max(sample[axis] for sample in materialized)
            - min(sample[axis] for sample in materialized)
            for axis in range(3)
        ]
        return medians, spans

    def _base_status(self, status: str, reason: str) -> Dict[str, Any]:
        return {
            "status": status,
            "reason": reason,
            "locked_id": self.locked_id,
            "collected_frame_count": len(self.samples),
            "required_frame_count": self.config.stable_frame_count,
            "window_reset_count": self.window_reset_count,
            "ever_saw_allowed_target": self.ever_saw_allowed_target,
        }

    @staticmethod
    def _xyz_dict(values: Tuple[float, float, float]) -> Dict[str, float]:
        return {"x": values[0], "y": values[1], "z": values[2]}

    def _terminal_failure(self, now_s: float, detail_reason: str) -> Dict[str, Any]:
        reason = "target_unstable" if self.ever_saw_allowed_target else "target_not_found"
        result = self._base_status("failed", reason)
        result["elapsed_s"] = (
            None if self.attempt_started_s is None else now_s - self.attempt_started_s
        )
        result["failure_detail"] = detail_reason
        if self.last_xyz_peak_to_peak_mm is not None:
            result["last_xyz_peak_to_peak_mm"] = self._xyz_dict(
                self.last_xyz_peak_to_peak_mm
            )
            result["threshold_exceeded_axes"] = [
                axis
                for axis, span, threshold in zip(
                    ("x", "y", "z"),
                    self.last_xyz_peak_to_peak_mm,
                    self.config.xyz_peak_to_peak_threshold_mm,
                )
                if span > threshold
            ]
        if self.best_xyz_peak_to_peak_mm is not None:
            result["best_xyz_peak_to_peak_mm"] = self._xyz_dict(
                self.best_xyz_peak_to_peak_mm
            )
            result["best_max_threshold_ratio"] = self.best_max_threshold_ratio
        self.terminal_result = result
        return result

    def update(self, payload: Dict[str, Any], now_s: float) -> Dict[str, Any]:
        if self.terminal_result is not None:
            return dict(self.terminal_result)
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        if self.attempt_started_s is None:
            self.attempt_started_s = now_s

        gate_valid, gate_reason = self._message_gate(payload)
        visible_candidates = self._candidate_map(payload)
        candidates = visible_candidates if gate_valid else {}
        if visible_candidates:
            self.ever_saw_allowed_target = True

        if self.locked_id is None:
            self.locked_id = next(
                (tag_id for tag_id in self.config.selection_order if tag_id in candidates),
                None,
            )

        sample_reason = gate_reason
        xyz: Optional[Tuple[float, float, float]] = None
        if gate_valid and self.locked_id is not None:
            candidate = candidates.get(self.locked_id)
            if candidate is None:
                sample_reason = "locked_target_missing"
            else:
                xyz = self._candidate_xyz(candidate)
                sample_reason = "ok" if xyz is not None else "locked_target_xyz_invalid"
        elif gate_valid:
            sample_reason = "no_allowed_target"

        if xyz is None:
            if self.samples:
                self.samples.clear()
                self.sample_stamps.clear()
                self.window_reset_count += 1
        else:
            self.samples.append(xyz)
            self.sample_stamps.append(self._finite_float(payload.get("pnp_stamp")))
            if len(self.samples) > self.config.stable_frame_count:
                self.samples.pop(0)
                self.sample_stamps.pop(0)

            if len(self.samples) == self.config.stable_frame_count:
                medians, spans = self._axis_statistics(self.samples)
                thresholds = self.config.xyz_peak_to_peak_threshold_mm
                spans_tuple = (spans[0], spans[1], spans[2])
                self.last_xyz_peak_to_peak_mm = spans_tuple
                max_threshold_ratio = max(
                    spans[index] / thresholds[index] for index in range(3)
                )
                if (
                    self.best_max_threshold_ratio is None
                    or max_threshold_ratio < self.best_max_threshold_ratio
                ):
                    self.best_max_threshold_ratio = max_threshold_ratio
                    self.best_xyz_peak_to_peak_mm = spans_tuple
                if all(spans[index] <= thresholds[index] for index in range(3)):
                    result = self._base_status("stable", "stable_target_ready")
                    result.update(
                        {
                            "base_object_median_mm": {
                                "x": medians[0],
                                "y": medians[1],
                                "z": medians[2],
                            },
                            "xyz_peak_to_peak_mm": {
                                "x": spans[0],
                                "y": spans[1],
                                "z": spans[2],
                            },
                            "sample_pnp_stamp_first": self.sample_stamps[0],
                            "sample_pnp_stamp_last": self.sample_stamps[-1],
                            "elapsed_s": now_s - self.attempt_started_s,
                        }
                    )
                    self.terminal_result = result
                    return result
                sample_reason = "xyz_peak_to_peak_exceeded"

        elapsed = now_s - self.attempt_started_s
        if elapsed >= self.config.stable_timeout_s:
            return self._terminal_failure(now_s, sample_reason)

        result = self._base_status(
            "collecting" if self.locked_id is not None else "waiting",
            sample_reason,
        )
        result["elapsed_s"] = elapsed
        if len(self.samples) >= 2:
            _, spans = self._axis_statistics(self.samples)
            result["current_xyz_peak_to_peak_mm"] = {
                "x": spans[0],
                "y": spans[1],
                "z": spans[2],
            }
        return result
