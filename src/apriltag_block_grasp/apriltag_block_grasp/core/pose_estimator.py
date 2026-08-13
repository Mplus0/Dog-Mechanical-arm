"""PnP pose estimation for a square AprilTag."""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from apriltag_block_grasp.core.apriltag_detector import AprilTagDetection2D
from apriltag_block_grasp.core.camera_calibration import ColorCameraCalibration


@dataclass(frozen=True)
class AprilTagPose:
    """Tag pose using the project right-handed tag coordinate convention."""

    method: str
    rvec: np.ndarray
    rotation_matrix: np.ndarray
    translation_mm: np.ndarray
    reprojection_error_px: float
    projected_corners: np.ndarray


class AprilTagPoseEstimator:
    """Estimate T_camera_tag with IPPE Square and iterative fallback.

    Project tag coordinates:
      +X: printed tag left -> right
      +Y: printed tag top -> bottom
      +Z: printed tag front -> back (away from the observing camera)

    OpenCV IPPE Square requires a native object-point order whose +Y and +Z
    axes are the opposite of this project convention. The returned rotation is
    therefore composed with a 180-degree rotation about +X.
    """

    def __init__(self, tag_size_mm: float, calibration: ColorCameraCalibration) -> None:
        self.tag_size_mm = float(tag_size_mm)
        if not np.isfinite(self.tag_size_mm) or self.tag_size_mm <= 0.0:
            raise ValueError("tag_size_mm must be finite and positive")
        self.calibration = calibration

        half = self.tag_size_mm / 2.0
        # Detector order is canonical tag TL, TR, BR, BL.
        self.project_object_points = np.array(
            [
                [-half, -half, 0.0],
                [half, -half, 0.0],
                [half, half, 0.0],
                [-half, half, 0.0],
            ],
            dtype=np.float64,
        )
        # Exact order required by SOLVEPNP_IPPE_SQUARE.
        self.ippe_object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )
        self.ippe_native_from_project = np.diag([1.0, -1.0, -1.0])

    def estimate(self, detection: AprilTagDetection2D) -> AprilTagPose:
        image_points = np.asarray(detection.corners, dtype=np.float64).reshape(4, 2)
        pose = self._estimate_ippe(image_points)
        if pose is not None:
            return pose
        pose = self._estimate_iterative(image_points)
        if pose is not None:
            return pose
        raise RuntimeError("IPPE Square and iterative solvePnP both failed")

    def _estimate_ippe(self, image_points: np.ndarray) -> Optional[AprilTagPose]:
        if not hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
            return None
        try:
            success, native_rvec, tvec = cv2.solvePnP(
                self.ippe_object_points,
                image_points,
                self.calibration.camera_matrix,
                self.calibration.distortion_coefficients,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except cv2.error:
            return None
        if not success:
            return None

        native_rotation, _ = cv2.Rodrigues(native_rvec)
        project_rotation = native_rotation @ self.ippe_native_from_project
        project_rvec, _ = cv2.Rodrigues(project_rotation)
        return self._build_pose("IPPE_SQUARE", project_rvec, tvec, image_points)

    def _estimate_iterative(self, image_points: np.ndarray) -> Optional[AprilTagPose]:
        try:
            success, rvec, tvec = cv2.solvePnP(
                self.project_object_points,
                image_points,
                self.calibration.camera_matrix,
                self.calibration.distortion_coefficients,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        except cv2.error:
            return None
        if not success:
            return None
        return self._build_pose("ITERATIVE", rvec, tvec, image_points)

    def _build_pose(
        self,
        method: str,
        rvec: np.ndarray,
        tvec: np.ndarray,
        image_points: np.ndarray,
    ) -> AprilTagPose:
        rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
        translation = np.asarray(tvec, dtype=np.float64).reshape(3)
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        values = np.concatenate((rvec.reshape(3), translation, rotation_matrix.reshape(9)))
        if not np.all(np.isfinite(values)):
            raise ValueError("PnP returned non-finite pose values")
        if translation[2] <= 0.0:
            raise ValueError(f"PnP returned non-positive camera Z: {translation[2]}")

        projected, _ = cv2.projectPoints(
            self.project_object_points,
            rvec,
            translation.reshape(3, 1),
            self.calibration.camera_matrix,
            self.calibration.distortion_coefficients,
        )
        projected = projected.reshape(4, 2)
        error = float(np.sqrt(np.mean(np.sum((projected - image_points) ** 2, axis=1))))
        if not np.isfinite(error):
            raise ValueError("PnP returned non-finite reprojection error")
        return AprilTagPose(
            method=method,
            rvec=rvec.reshape(3),
            rotation_matrix=rotation_matrix,
            translation_mm=translation,
            reprojection_error_px=error,
            projected_corners=projected,
        )


def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert a valid 3x3 rotation matrix to an XYZW quaternion."""

    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    # Robust branch formulation without adding a scipy dependency.
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (matrix[2, 1] - matrix[1, 2]) / s
        qy = (matrix[0, 2] - matrix[2, 0]) / s
        qz = (matrix[1, 0] - matrix[0, 1]) / s
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            s = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            qw = (matrix[2, 1] - matrix[1, 2]) / s
            qx = 0.25 * s
            qy = (matrix[0, 1] + matrix[1, 0]) / s
            qz = (matrix[0, 2] + matrix[2, 0]) / s
        elif index == 1:
            s = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            qw = (matrix[0, 2] - matrix[2, 0]) / s
            qx = (matrix[0, 1] + matrix[1, 0]) / s
            qy = 0.25 * s
            qz = (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            qw = (matrix[1, 0] - matrix[0, 1]) / s
            qx = (matrix[0, 2] + matrix[2, 0]) / s
            qy = (matrix[1, 2] + matrix[2, 1]) / s
            qz = 0.25 * s
    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)
