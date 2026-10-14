import pathlib
import sys
import tempfile
import unittest

import cv2
import numpy as np
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from apriltag import AprilTagDetector  # noqa: E402
from auto_aim import (AttitudeBuffer, AutoAimEngine, ConstantAccelerationFilter,
                      MpcTrajectoryPlanner)  # noqa: E402
from extrinsic import (ExtrinsicCalibrationSession, NOMINAL_CAMERA2GIMBAL,
                       resolve_mount_orientation,
                       validate_mount_orientation)  # noqa: E402
from protocol import AttitudeSample, STATUS_IMU_VALID, STATUS_SAFE  # noqa: E402


class AutoAimTests(unittest.TestCase):
    def test_apriltag_ippe_pose(self):
        image = np.full((1440, 1080, 3), 255, np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
        marker = cv2.aruco.generateImageMarker(dictionary, 0, 160)
        image[640:800, 460:620] = cv2.cvtColor(marker, cv2.COLOR_GRAY2RGB)
        matrix = np.array([[1743., 0., 540.], [0., 1744., 720.], [0., 0., 1.]])
        poses = AprilTagDetector(matrix, np.zeros(5), 0.1).detect(image, 123)
        self.assertEqual(len(poses), 1)
        self.assertEqual(poses[0].tag_id, 0)
        self.assertTrue(1.0 <= np.linalg.norm(poses[0].tvec_m) <= 4.0)
        self.assertLess(poses[0].reprojection_error_px, 1.5)

        roi_poses = AprilTagDetector(matrix, np.zeros(5), 0.1).detect(
            image, 124, locked_id=0, roi=(400, 580, 680, 860))
        self.assertEqual(len(roi_poses), 1)
        self.assertAlmostEqual(roi_poses[0].center[0], 540.0, delta=2.0)
        self.assertAlmostEqual(roi_poses[0].center[1], 720.0, delta=2.0)

    def test_detector_avoids_duplicate_expensive_corner_refinement(self):
        matrix = np.array([[1743., 0., 540.], [0., 1744., 720.], [0., 0., 1.]])
        detector = AprilTagDetector(matrix, np.zeros(5), 0.1)
        parameters = detector.detector.getDetectorParameters()
        self.assertEqual(parameters.cornerRefinementMethod,
                         cv2.aruco.CORNER_REFINE_NONE)

    def test_apriltag_pose_is_visible_inside_one_meter(self):
        image = np.full((1440, 1080, 3), 255, np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
        marker = cv2.aruco.generateImageMarker(dictionary, 0, 300)
        image[570:870, 390:690] = cv2.cvtColor(marker, cv2.COLOR_GRAY2RGB)
        matrix = np.array([[1743., 0., 540.], [0., 1744., 720.], [0., 0., 1.]])
        poses = AprilTagDetector(matrix, np.zeros(5), 0.1).detect(image, 123)
        self.assertEqual(len(poses), 1)
        self.assertLess(np.linalg.norm(poses[0].tvec_m), 1.0)

    def test_constant_acceleration_prediction(self):
        tracker = ConstantAccelerationFilter()
        tracker.update(np.array([2., 0., 0.]), 0)
        tracker.x[3] = 1.0
        predicted = tracker.position_at(1_000_000_000)
        self.assertAlmostEqual(predicted[0], 3.0)

    def test_attitude_interpolation_normalizes_quaternion(self):
        samples = AttitudeBuffer()
        samples.put(AttitudeSample(0, 0, (1., 0., 0., 0.), 0, 0, 0, 0, 100))
        samples.put(AttitudeSample(1, 0, (0., 0., 0., 1.), 1000, 2000, 0, 0, 200))
        middle = samples.at(150)
        self.assertIsNotNone(middle)
        self.assertAlmostEqual(np.linalg.norm(middle.quaternion_wxyz), 1.0)
        self.assertEqual(middle.pitch_mdeg, 500)
        self.assertEqual(middle.yaw_mdeg, 1000)
        self.assertAlmostEqual(samples.nearest_delta_ms(175), 0.000025)

    def test_auto_aim_reports_out_of_engagement_range(self):
        samples = AttitudeBuffer()
        flags = STATUS_IMU_VALID | STATUS_SAFE
        samples.put(AttitudeSample(
            1, flags, (1., 0., 0., 0.), 0, 0, 0, 0, 1_000_000_000))
        engine = AutoAimEngine(samples)
        engine.extrinsic_valid = True
        engine.filter.update(np.array([0.7, 0., 0.]), 1_000_000_000)
        engine.last_observation_ns = 1_000_000_000
        command = engine.command(1_010_000_000, False)
        self.assertFalse(command.valid)
        self.assertIn("1～4 m", command.gate_reason)

    def test_stationary_gate_allows_half_degree_noise(self):
        samples = AttitudeBuffer()
        for index in range(31):
            samples.put(AttitudeSample(
                index, 0, (1., 0., 0., 0.), 0, 0, 500, -500,
                100_000_000 + index * 10_000_000, 4000, 3995,
                (10, -20, 30)))
        attitude, reason = samples.capture_quality(400_000_000)
        self.assertIsNotNone(attitude)
        self.assertIn("质量门控通过", reason)

        samples.put(AttitudeSample(
            32, 0, (1., 0., 0., 0.), 0, 0, 1600, 0,
            410_000_000, 4000, 3995, (10, -20, 30)))
        attitude, reason = samples.capture_quality(410_000_000)
        self.assertIsNone(attitude)
        self.assertIn("峰值", reason)

    def test_planner_respects_limits(self):
        planner = MpcTrajectoryPlanner(horizon=30)
        result = planner.axis(0., 0., np.ones(30)*2., 1.68, 10.)
        self.assertLessEqual(abs(result[1]), 1.68)
        self.assertLessEqual(abs(result[2]), 10.0)
        self.assertTrue(result[3])

    def test_robot_world_hand_eye_with_holdout(self):
        def transform(rotation, translation):
            value = np.eye(4)
            value[:3, :3] = rotation
            value[:3, 3] = translation
            return value

        with tempfile.TemporaryDirectory() as folder:
            session = ExtrinsicCalibrationSession(
                pathlib.Path(folder), np.eye(3), np.zeros(5))
            r_base_world, _ = cv2.Rodrigues(np.array([0.15, -0.08, 0.05]))
            r_gimbal_camera, _ = cv2.Rodrigues(np.array([-0.12, 0.09, 0.04]))
            base_world = transform(r_base_world, np.array([80., -40., 900.]))
            gimbal_camera = transform(r_gimbal_camera, np.array([25., 15., 30.]))
            for index in range(30):
                vector = np.array([
                    0.25 * np.sin(index * 0.43),
                    0.35 * np.cos(index * 0.31),
                    0.45 * np.sin(index * 0.27)])
                r_base_gimbal, _ = cv2.Rodrigues(vector)
                camera_world = (gimbal_camera @ transform(r_base_gimbal, np.zeros(3))
                                @ np.linalg.inv(base_world))
                rvec, _ = cv2.Rodrigues(camera_world[:3, :3])
                session.camera_poses.append((rvec, camera_world[:3, 3].reshape(3, 1)))
                session.gimbal_rotations.append(r_base_gimbal)
            output = session.solve(enforce_mount_prior=False)
            with output.open(encoding="utf-8") as stream:
                result = yaml.safe_load(stream)
            recovered = np.asarray(result["R_camera2gimbal"]).reshape(3, 3)
            expected = r_gimbal_camera.T
            self.assertLess(np.linalg.norm(recovered - expected), 1e-5)
            self.assertLess(result["holdout_max_rotation_error_deg"], 1e-4)

    def test_planar_normal_branch_uses_mount_prior(self):
        ambiguous = NOMINAL_CAMERA2GIMBAL @ np.diag([-1., 1., -1.])
        resolved, corrected = resolve_mount_orientation(ambiguous)
        self.assertTrue(corrected)
        np.testing.assert_allclose(resolved, NOMINAL_CAMERA2GIMBAL)
        validate_mount_orientation(resolved)
        with self.assertRaises(ValueError):
            validate_mount_orientation(ambiguous)

    def test_symmetric_grid_order_is_canonical(self):
        centers = np.arange(140, dtype=np.float32).reshape(70, 1, 2)
        forward = ExtrinsicCalibrationSession.canonical_centers(centers)
        reversed_order = ExtrinsicCalibrationSession.canonical_centers(centers[::-1])
        np.testing.assert_array_equal(forward, reversed_order)


if __name__ == "__main__":
    unittest.main()
